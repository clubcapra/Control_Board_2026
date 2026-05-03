# RG I2C API

This document describes only the I2C API exposed by the PDU firmware to the RG
computer.

The RG is the I2C master. The PDU is an I2C slave on `I2C2`.

- Slave address: `0x31` 7-bit
- SCL: `PB10`
- SDA: `PB11`
- Byte order: little-endian
- Struct packing: packed, no padding
- Command execution: deferred to the PDU main loop

The PDU I2C interrupt callbacks only copy bytes. After RG writes a command or
PMBus bridge request, RG must poll the matching result register to know when the
PDU has processed it.

## Register Map

- `0x00 Info`: read API/version information.
- `0x10 TelemetryAll`: read full telemetry snapshot.
- `0x20 Command`: write one high-level command.
- `0x21 CommandStatus`: read result of the latest high-level command.
- `0x40 PmbusBridge`: write one raw PMBus bridge request.
- `0x41 PmbusResult`: read result of the latest PMBus bridge request.

To read a register:

1. RG writes one byte: the register selector.
2. RG performs an I2C read from slave `0x31`.

To write a register:

1. RG writes one byte: the register selector.
2. RG writes the packed payload immediately after the selector in the same I2C transaction.

## RG Driver Skeleton

The examples below use this simple RG-side pseudo-driver. Adapt the low-level
`i2c_write()` and `i2c_read()` calls to the RG platform.

```c
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define PDU_ADDR 0x31

enum {
  PDU_REG_INFO           = 0x00,
  PDU_REG_TELEMETRY_ALL  = 0x10,
  PDU_REG_COMMAND        = 0x20,
  PDU_REG_COMMAND_STATUS = 0x21,
  PDU_REG_PMBUS_BRIDGE   = 0x40,
  PDU_REG_PMBUS_RESULT   = 0x41,
};

typedef struct __attribute__((packed)) {
  uint8_t command;
  uint8_t arg0;
  uint8_t arg1;
  uint8_t arg2;
  uint8_t arg3;
} PduCommandFrame;

typedef struct __attribute__((packed)) {
  uint8_t sequence;
  uint8_t busy;
  uint8_t status;
  uint8_t command;
  uint8_t arg0;
  uint8_t arg1;
  uint8_t reserved[2];
} PduCommandResult;

static bool pdu_write_reg(uint8_t reg, const void *payload, uint8_t len) {
  uint8_t frame[1 + 32];
  if (len > 32) {
    return false;
  }
  frame[0] = reg;
  memcpy(&frame[1], payload, len);
  return i2c_write(PDU_ADDR, frame, (uint8_t)(1 + len));
}

static bool pdu_read_reg(uint8_t reg, void *payload, uint8_t len) {
  if (!i2c_write(PDU_ADDR, &reg, 1)) {
    return false;
  }
  return i2c_read(PDU_ADDR, payload, len);
}

static bool pdu_command(uint8_t command,
                        uint8_t arg0,
                        uint8_t arg1,
                        uint8_t arg2,
                        uint8_t arg3) {
  PduCommandFrame frame = {command, arg0, arg1, arg2, arg3};
  return pdu_write_reg(PDU_REG_COMMAND, &frame, sizeof(frame));
}

static bool pdu_read_command_status(PduCommandResult *out) {
  return pdu_read_reg(PDU_REG_COMMAND_STATUS, out, sizeof(*out));
}
```

Typical command flow:

```c
PduCommandResult result;

pdu_command(0x01, 1, 1, 0, 0);  // enable 24 V rail

do {
  delay_ms(5);
  pdu_read_command_status(&result);
} while (result.busy != 0);

if (result.status != 0) {
  // Command failed. See the status code list below.
}
```

Signed percent arguments are encoded as `int8_t` but transmitted as a raw byte:

```c
static uint8_t i8_arg(int8_t value) {
  return (uint8_t)value;
}

pdu_command(0x31, 0, i8_arg(-40), 0, 0);  // motor A reverse 40 %
```

## Status Codes

Every command and PMBus bridge result reports one of these status codes.

- `0 OK`: command completed successfully.
- `1 BUS_ERROR`: low-level I2C/PMBus access failed.
- `2 TIMEOUT`: peripheral or peer did not respond in time.
- `3 PARAM`: invalid argument, rail ID, channel, mode, or command.
- `4 NOT_PRESENT`: targeted LM5066H1 did not ACK or is considered absent.
- `5 RANGE`: value is outside the allowed range.
- `6 PEC_MISMATCH`: SMBus PEC/CRC mismatch.
- `7 FAULT`: command refused because the PDU is in a fault or E-Stop condition.
- `8 NOT_INIT`: module not initialized.
- `9 INTERNAL`: unexpected internal error.

## Common Argument Values

Rail IDs:

- `0`: 48 V rail
- `1`: 24 V rail
- `2`: 12 V rail
- `3`: all rails, only for commands that explicitly support all rails

LED channels:

- `0`: Bras
- `1`: Avant
- `2`: Arriere
- `3`: Extra

LED patterns:

- `0`: off
- `1`: solid
- `2`: heartbeat
- `3`: fault blink
- `4`: E-Stop strobe

Winch modes:

- `0`: sleep
- `1`: dual DC
- `2`: stepper
- `3`: parallel DC

Winch motor IDs:

- `0`: motor A
- `1`: motor B

Winch lock channels:

- `0`: lock 1
- `1`: lock 2
- `2`: all locks

## `0x00 Info`

Read length: 16 bytes.

Purpose: lets RG verify that the device at `0x31` is the PDU API and check the
protocol and firmware versions before sending commands.

```text
offset  size  field
0       4     magic = "PDU1"
4       1     protocol_major = 1
5       1     protocol_minor = 9
6       1     fw_major
7       1     fw_minor
8       1     fw_patch
9       1     i2c_addr = 0x31
10      1     rail_count = 3
11      5     reserved
```

Example:

```c
uint8_t info[16];
pdu_read_reg(PDU_REG_INFO, info, sizeof(info));

if (info[0] == 'P' && info[1] == 'D' && info[2] == 'U' && info[3] == '1') {
  uint8_t protocol_major = info[4];
  uint8_t protocol_minor = info[5];
}
```

## `0x20 Command`

Write length: 5 bytes.

```text
offset  size  field
0       1     command
1       1     arg0
2       1     arg1
3       1     arg2
4       1     arg3
```

After writing this register, poll `0x21 CommandStatus`.

## Command Details And Examples

### `0x00 Noop`

Description: Does nothing. Use it to verify the command path and `CommandStatus`
polling without changing PDU outputs.

Arguments:

- `arg0..arg3`: ignored, set to `0`.

Expected result:

- `OK` when the command loop processes the frame.

Example:

```c
pdu_command(0x00, 0, 0, 0, 0);
```

### `0x01 SetRailEnable`

Description: Enables or disables one LM5066H1 rail, or all rails. Enabling a
rail may fail if the rail is absent, latched, faulted, or if the PMBus write
fails. Disabling is the safe path and is always attempted.

Arguments:

- `arg0`: rail ID: `0=48V`, `1=24V`, `2=12V`, `3=all`.
- `arg1`: desired state: `0=disable`, nonzero `=enable`.
- `arg2`, `arg3`: ignored.

Expected result:

- `OK`: rail command accepted.
- `PARAM`: invalid rail ID.
- `NOT_PRESENT`, `BUS_ERROR`, or `FAULT`: rail could not be enabled.

Examples:

```c
// Enable 24 V.
pdu_command(0x01, 1, 1, 0, 0);

// Disable all rails.
pdu_command(0x01, 3, 0, 0, 0);
```

### `0x02 SetLedDuty`

Description: Sets one lighting channel duty cycle and forces LED pattern mode
to `solid`. This is used when RG wants direct brightness control instead of an
automatic blink/strobe pattern.

Arguments:

- `arg0`: LED channel: `0=Bras`, `1=Avant`, `2=Arriere`, `3=Extra`.
- `arg1`: duty cycle percent, `0..100`.
- `arg2`, `arg3`: ignored.

Expected result:

- `OK`: channel duty updated.
- `PARAM` or `RANGE`: invalid channel or duty.

Example:

```c
// Set Avant light to 60 %.
pdu_command(0x02, 1, 60, 0, 0);
```

### `0x03 SetAllLeds`

Description: Sets all lighting channels to the same duty cycle and forces LED
pattern mode to `solid`.

Arguments:

- `arg0`: duty cycle percent, `0..100`.
- `arg1..arg3`: ignored.

Expected result:

- `OK`: all LED duties updated.
- `RANGE`: duty is above `100`.

Examples:

```c
// Lights off.
pdu_command(0x03, 0, 0, 0, 0);

// All lights at 25 %.
pdu_command(0x03, 25, 0, 0, 0);
```

### `0x04 SetLedPattern`

Description: Selects one of the built-in LED patterns. Patterns are generated
inside the PDU main loop.

Arguments:

- `arg0`: pattern: `0=off`, `1=solid`, `2=heartbeat`, `3=fault blink`, `4=E-Stop strobe`.
- `arg1..arg3`: ignored.

Expected result:

- `OK`: pattern selected.
- `PARAM`: invalid pattern.

Example:

```c
// Select heartbeat pattern.
pdu_command(0x04, 2, 0, 0, 0);
```

### `0x05 SetEstopLocal`

Description: Controls the local PDU E-Stop command output on `PA0`. This output
is active-low at the hardware level; the API argument is logical: `1=assert
E-Stop`, `0=release local assertion`.

Arguments:

- `arg0`: `0=release`, nonzero `=assert`.
- `arg1..arg3`: ignored.

Expected result:

- `OK`: local E-Stop command state updated.

Examples:

```c
// Assert local E-Stop.
pdu_command(0x05, 1, 0, 0, 0);

// Release local E-Stop assertion.
pdu_command(0x05, 0, 0, 0, 0);
```

### `0x06 SetEstopVtx`

Description: Controls the redundant E-Stop VTX output on `PB13`.

Arguments:

- `arg0`: `0=off`, nonzero `=on`.
- `arg1..arg3`: ignored.

Expected result:

- `OK`: VTX output state updated.

Example:

```c
// Energize E-Stop VTX output.
pdu_command(0x06, 1, 0, 0, 0);
```

### `0x07 ClearRailLatch`

Description: Clears the firmware latch state for one rail or all rails after a
latched protection event. This resets the rail fault counter and software
protection accumulators, and clears LM5066H1 fault flags. It does not
automatically enable the rail; RG must send `SetRailEnable` afterward if it is
safe to re-energize.

Arguments:

- `arg0`: rail ID: `0=48V`, `1=24V`, `2=12V`, `3=all`.
- `arg1..arg3`: ignored.

Expected result:

- `OK`: latch clear accepted.
- `PARAM`: invalid rail ID.

Example:

```c
// Clear all rail latches.
pdu_command(0x07, 3, 0, 0, 0);
```

### `0x08 ClearFaultLog`

Description: Clears the reset-persistent fault FIFO stored by the PDU. This
only clears the log; it does not clear active rail faults or LM5066H1 black-box
memory.

Arguments:

- `arg0..arg3`: ignored.

Expected result:

- `OK`: fault log cleared.

Example:

```c
pdu_command(0x08, 0, 0, 0, 0);
```

### `0x09 ResetDevice`

Description: Records a host-requested reset fault record, then reboots the MCU.
The I2C transaction may complete before the reset occurs, but RG should expect
the PDU to disappear briefly from the bus.

Arguments:

- `arg0..arg3`: ignored.

Expected result:

- The PDU writes an `OK` command result before requesting reset.
- RG should wait for the PDU to reboot and then read `Info` again.

Example:

```c
pdu_command(0x09, 0, 0, 0, 0);
delay_ms(1000);
```

### `0x0A SetUnixTime`

Description: Sets the PDU fault-log wall-clock reference. New fault records use
this Unix timestamp base along with uptime.

Arguments:

- `arg0`: Unix time bits `7..0`.
- `arg1`: Unix time bits `15..8`.
- `arg2`: Unix time bits `23..16`.
- `arg3`: Unix time bits `31..24`.

Expected result:

- `OK`: time accepted.

Example:

```c
uint32_t unix_s = 1714674600UL;
pdu_command(0x0A,
            (uint8_t)(unix_s >> 0),
            (uint8_t)(unix_s >> 8),
            (uint8_t)(unix_s >> 16),
            (uint8_t)(unix_s >> 24));
```

### `0x0B RefreshHotswapBlackBox`

Description: Forces the PDU to reread LM5066H1 black-box RAM/EEPROM data for
one rail or all rails. The refreshed bytes appear in `TelemetryAll`.

Arguments:

- `arg0`: rail ID: `0=48V`, `1=24V`, `2=12V`, `3=all`.
- `arg1..arg3`: ignored.

Expected result:

- `OK`: refresh completed.
- `PARAM`: invalid rail ID.
- `BUS_ERROR` or `NOT_PRESENT`: LM5066H1 access failed.

Example:

```c
// Refresh 48 V black-box data.
pdu_command(0x0B, 0, 0, 0, 0);
```

### `0x0C EraseHotswapBlackBox`

Description: Clears and erases the LM5066H1 black-box EEPROM for one rail or
all rails, then refreshes the PDU telemetry snapshot. Use only after RG has
saved any required diagnostic data.

Arguments:

- `arg0`: rail ID: `0=48V`, `1=24V`, `2=12V`, `3=all`.
- `arg1..arg3`: ignored.

Expected result:

- `OK`: erase completed.
- `PARAM`: invalid rail ID.
- `BUS_ERROR` or `NOT_PRESENT`: LM5066H1 access failed.

Example:

```c
// Erase 12 V black-box data.
pdu_command(0x0C, 2, 0, 0, 0);
```

### `0x30 SetWinchMode`

Description: Selects the DRV8262 winch operating mode. If E-Stop is active, the
PDU rejects every mode except `sleep`.

Arguments:

- `arg0`: mode: `0=sleep`, `1=dual DC`, `2=stepper`, `3=parallel DC`.
- `arg1..arg3`: ignored.

Expected result:

- `OK`: mode selected.
- `FAULT`: E-Stop is active and requested mode is not `sleep`.
- `PARAM`: invalid mode.

Examples:

```c
// Put winch driver to sleep.
pdu_command(0x30, 0, 0, 0, 0);

// Select dual DC mode.
pdu_command(0x30, 1, 0, 0, 0);
```

### `0x31 SetWinchDcMotor`

Description: Commands one motor in dual DC mode. Command is signed percent:
negative reverses direction, positive drives forward, zero stops that motor.
Rejected while E-Stop is active.

Arguments:

- `arg0`: motor ID: `0=A`, `1=B`.
- `arg1`: signed `int8_t` percent, `-100..100`.
- `arg2`, `arg3`: ignored.

Expected result:

- `OK`: motor command accepted.
- `FAULT`: E-Stop is active.
- `PARAM` or `RANGE`: invalid motor or percent.

Examples:

```c
// Motor A forward 50 %.
pdu_command(0x31, 0, i8_arg(50), 0, 0);

// Motor B reverse 30 %.
pdu_command(0x31, 1, i8_arg(-30), 0, 0);
```

### `0x32 SetWinchParallelDc`

Description: Commands the winch in parallel DC mode. Command is signed percent:
negative reverses direction, positive drives forward, zero stops. Rejected
while E-Stop is active.

Arguments:

- `arg0`: signed `int8_t` percent, `-100..100`.
- `arg1..arg3`: ignored.

Expected result:

- `OK`: parallel DC command accepted.
- `FAULT`: E-Stop is active.
- `RANGE`: percent is outside `-100..100`.

Example:

```c
// Parallel DC forward 70 %.
pdu_command(0x32, i8_arg(70), 0, 0, 0);
```

### `0x33 SetWinchStepperPhases`

Description: Directly commands both stepper phases. Each phase command is a
signed percent. Rejected while E-Stop is active.

Arguments:

- `arg0`: phase A signed `int8_t` percent, `-100..100`.
- `arg1`: phase B signed `int8_t` percent, `-100..100`.
- `arg2`, `arg3`: ignored.

Expected result:

- `OK`: phase commands accepted.
- `FAULT`: E-Stop is active.
- `RANGE`: a phase percent is outside `-100..100`.

Example:

```c
// Phase A forward 40 %, phase B reverse 40 %.
pdu_command(0x33, i8_arg(40), i8_arg(-40), 0, 0);
```

### `0x34 BrakeWinch`

Description: Brakes both DRV8262 bridges. This command is allowed even when
E-Stop is active.

Arguments:

- `arg0..arg3`: ignored.

Expected result:

- `OK`: brake command accepted.

Example:

```c
pdu_command(0x34, 0, 0, 0, 0);
```

### `0x35 ClearWinchFault`

Description: Attempts to clear a latched DRV8262 winch fault by pulsing the
driver sleep/reset path. This does not command motion.

Arguments:

- `arg0..arg3`: ignored.

Expected result:

- `OK`: clear sequence completed.
- Other status: driver clear sequence failed.

Example:

```c
pdu_command(0x35, 0, 0, 0, 0);
```

### `0x36 SetWinchLock`

Description: Controls the TPS2HB16 winch-lock high-side outputs. Rejected while
E-Stop is active.

Arguments:

- `arg0`: lock channel: `0=lock1`, `1=lock2`, `2=all`.
- `arg1`: desired state: `0=off`, nonzero `=on`.
- `arg2`, `arg3`: ignored.

Expected result:

- `OK`: lock output command accepted.
- `FAULT`: E-Stop is active.
- `PARAM`: invalid lock channel.

Examples:

```c
// Turn lock 1 on.
pdu_command(0x36, 0, 1, 0, 0);

// Turn both locks off.
pdu_command(0x36, 2, 0, 0, 0);
```

## `0x21 CommandStatus`

Read length: 8 bytes.

```text
offset  size  field
0       1     sequence
1       1     busy
2       1     status
3       1     command
4       1     arg0
5       1     arg1
6       2     reserved
```

Fields:

- `sequence`: increments each time the PDU processes a high-level command.
- `busy`: currently `0` after the command result is published.
- `status`: command result status code.
- `command`: opcode that was processed.
- `arg0`, `arg1`: echo of the first two command arguments.

Example:

```c
PduCommandResult result;
pdu_read_command_status(&result);

if (result.sequence != previous_sequence) {
  previous_sequence = result.sequence;
  if (result.status == 0) {
    // Latest command succeeded.
  }
}
```

## `0x40 PmbusBridge`

Write length: 28 bytes.

Purpose: lets RG access LM5066H1 PMBus commands through the PDU. RG does not
connect directly to the internal PMBus/SMBus rails.

```text
offset  size  field
0       1     rail_id
1       1     op
2       1     command
3       1     length
4       24    data
```

Fields:

- `rail_id`: `0=48V`, `1=24V`, `2=12V`.
- `op`: `0=read`, `1=write`.
- `command`: raw PMBus command byte.
- `length`: number of bytes to write, or requested read length. The PDU clips it to 24.
- `data`: write payload for `op=1`.

RG-side types:

```c
typedef struct __attribute__((packed)) {
  uint8_t rail_id;
  uint8_t op;
  uint8_t command;
  uint8_t length;
  uint8_t data[24];
} PduPmbusBridgeRequest;

typedef struct __attribute__((packed)) {
  uint8_t sequence;
  uint8_t busy;
  uint8_t status;
  uint8_t rail_id;
  uint8_t op;
  uint8_t command;
  uint8_t length;
  uint8_t data[24];
} PduPmbusBridgeResult;
```

Example read request:

```c
PduPmbusBridgeRequest req = {0};
req.rail_id = 1;       // 24 V
req.op = 0;            // read
req.command = 0x88;    // READ_VIN
req.length = 2;        // word
pdu_write_reg(PDU_REG_PMBUS_BRIDGE, &req, sizeof(req));
```

Example write request:

```c
PduPmbusBridgeRequest req = {0};
req.rail_id = 2;       // 12 V
req.op = 1;            // write
req.command = 0x01;    // OPERATION
req.length = 1;
req.data[0] = 0x80;    // OPERATION on
pdu_write_reg(PDU_REG_PMBUS_BRIDGE, &req, sizeof(req));
```

## `0x41 PmbusResult`

Read length: 31 bytes.

```text
offset  size  field
0       1     sequence
1       1     busy
2       1     status
3       1     rail_id
4       1     op
5       1     command
6       1     length
7       24    data
```

For PMBus reads, `length` is the number of valid bytes returned in `data`.

Example:

```c
PduPmbusBridgeResult result;
pdu_read_reg(PDU_REG_PMBUS_RESULT, &result, sizeof(result));

if (result.status == 0 && result.op == 0 && result.length >= 2) {
  uint16_t raw_word = (uint16_t)result.data[0] |
                      ((uint16_t)result.data[1] << 8);
}
```

## `0x10 TelemetryAll`

Read length: packed `ApiTelemetryAll`.

Purpose: gives RG one coherent snapshot containing supervisor state, PBIT/CBIT
state, fault history, winch state, and all rail telemetry.

Top-level layout:

```text
offset  size  field
0       4     magic = "TLM1"
4       1     protocol_major
5       1     protocol_minor
6       1     mode
7       1     estop_active
8       1     pbit_passed
9       1     cbit_passed
10      2     pbit_failed
12      2     cbit_failed
14      4     uptime_ms
18      1     last_fault_valid
19      1     last_fault_code
20      1     last_fault_rail
21      1     reserved
22      2     last_fault_sequence
24      2     last_fault_status_word
26      2     last_fault_diag_word
28      2     reset_count
30      4     last_fault_uptime_ms
34      4     last_fault_unix_time_s
38      4     reset_flags
42      1     fault_history_count
43      1     fault_history_capacity
44      2     fault_history_dropped
46      13    winch telemetry
59      3 x rail telemetry
...           24 x fault records
```

Supervisor modes:

- `0`: BOOT
- `1`: PBIT
- `2`: NOMINAL
- `3`: DEGRADED
- `4`: FAULT
- `5`: ESTOP

Rail telemetry record:

```text
offset  size  field
0       1     rail_id
1       1     state
2       1     present
3       1     output_on
4       1     pgood
5       1     reserved
6       2     status_word
8       2     diag_word
10      2     status_mfr_specific2
12      2     fault_count
14      1     wd_plb_timer
15      3     rail_reserved
18      4     vin_mV
22      4     vout_mV
26      4     vaux_mV
30      4     iin_mA
34      4     pin_dW
38      1     peak_valid
39      3     peak_reserved
42      4     peak_vin_mV
46      4     peak_iin_mA
50      4     peak_pin_dW
54      2     die_temp_centiC
56      2     ntc_temp_centiC
58      1     bb_valid
59      1     bb_config
60      1     bb_timer
61      1     bb_ram_len
62      1     bb_eeprom_len
63      1     bb_ram_event
64      1     bb_ram_timer_expired
65      1     bb_ram_tick
66      1     bb_eeprom_event
67      1     bb_eeprom_timer_expired
68      1     bb_eeprom_tick
69      2     bb_reserved
71      16    bb_ram
87      16    bb_eeprom
```

Rail states:

- `0`: BOOT
- `1`: ABSENT
- `2`: READY
- `3`: RUNNING
- `4`: WARNING
- `5`: TRIPPED
- `6`: LATCHED

Winch telemetry:

```text
offset  size  field
0       1     mode
1       1     awake
2       1     fault_active
3       1     lock1_on
4       1     lock2_on
5       3     reserved
8       1     motor_a_cmd_pct
9       1     motor_b_cmd_pct
10      1     parallel_cmd_pct
11      1     stepper_a_cmd_pct
12      1     stepper_b_cmd_pct
```

Fault record:

```text
offset  size  field
0       1     valid
1       1     code
2       1     rail
3       1     reserved
4       2     sequence
6       2     status_word
8       2     diag_word
10      2     reset_count
12      4     uptime_ms
16      4     unix_time_s
20      4     reset_flags
```

Example telemetry read:

```c
uint8_t telemetry[4096];

// Use the exact struct size in the RG implementation. The buffer here is only
// intentionally oversized for illustration.
if (pdu_read_reg(PDU_REG_TELEMETRY_ALL, telemetry, sizeof(telemetry))) {
  if (telemetry[0] == 'T' && telemetry[1] == 'L' &&
      telemetry[2] == 'M' && telemetry[3] == '1') {
    uint8_t mode = telemetry[6];
    uint8_t estop_active = telemetry[7];
  }
}
```
# RG I2C API

This document describes only the I2C API exposed to the RG computer.

The PDU firmware is an I2C slave on `I2C2`:

- Slave address: `0x31` 7-bit
- SCL: `PB10`
- SDA: `PB11`
- Byte order: little-endian
- Struct packing: no padding

The RG is the I2C master. The PDU only moves bytes in I2C callbacks; commands are executed later from the main loop. After writing a command or PMBus bridge request, RG should poll the matching result register.

## Register Map

- `0x00 Info`: read `ApiInfo`
- `0x10 TelemetryAll`: read full telemetry snapshot
- `0x20 Command`: write `ApiCommandFrame`
- `0x21 CommandStatus`: read `ApiCommandResult`
- `0x40 PmbusBridge`: write `ApiPmbusBridgeRequest`
- `0x41 PmbusResult`: read `ApiPmbusBridgeResult`

To read a register, RG writes the one-byte register selector, then performs an I2C read from `0x31`.

To write a register, RG writes the one-byte register selector followed by the packed payload.

## Status Codes

- `0`: OK
- `1`: BUS_ERROR
- `2`: TIMEOUT
- `3`: PARAM
- `4`: NOT_PRESENT
- `5`: RANGE
- `6`: PEC_MISMATCH
- `7`: FAULT
- `8`: NOT_INIT
- `9`: INTERNAL

## `0x00 Info`

Read length: 16 bytes.

```text
offset  size  field
0       4     magic = "PDU1"
4       1     protocol_major = 1
5       1     protocol_minor = 9
6       1     fw_major
7       1     fw_minor
8       1     fw_patch
9       1     i2c_addr = 0x31
10      1     rail_count = 3
11      5     reserved
```

## `0x20 Command`

Write length: 5 bytes.

```text
offset  size  field
0       1     command
1       1     arg0
2       1     arg1
3       1     arg2
4       1     arg3
```

Command opcodes:

- `0x00 Noop`
- `0x01 SetRailEnable`: `arg0 rail`, `arg1 0=off, 1=on`
- `0x02 SetLedDuty`: `arg0 LED channel`, `arg1 duty percent 0..100`
- `0x03 SetAllLeds`: `arg0 duty percent 0..100`
- `0x04 SetLedPattern`: `arg0 pattern`
- `0x05 SetEstopLocal`: `arg0 0=release, 1=assert`
- `0x06 SetEstopVtx`: `arg0 0=off, 1=on`
- `0x07 ClearRailLatch`: `arg0 rail`
- `0x08 ClearFaultLog`
- `0x09 ResetDevice`
- `0x0A SetUnixTime`: `arg0..arg3` little-endian Unix time in seconds
- `0x0B RefreshHotswapBlackBox`: `arg0 rail`
- `0x0C EraseHotswapBlackBox`: `arg0 rail`
- `0x30 SetWinchMode`: `arg0 mode`
- `0x31 SetWinchDcMotor`: `arg0 motor`, `arg1 signed percent`
- `0x32 SetWinchParallelDc`: `arg0 signed percent`
- `0x33 SetWinchStepperPhases`: `arg0 phase A signed percent`, `arg1 phase B signed percent`
- `0x34 BrakeWinch`
- `0x35 ClearWinchFault`
- `0x36 SetWinchLock`: `arg0 lock channel`, `arg1 0=off, 1=on`

Rail IDs:

- `0`: 48 V
- `1`: 24 V
- `2`: 12 V
- `3`: all rails, where supported

LED channels:

- `0`: Bras
- `1`: Avant
- `2`: Arriere
- `3`: Extra

LED patterns:

- `0`: off
- `1`: solid
- `2`: heartbeat
- `3`: fault blink
- `4`: E-Stop strobe

Winch modes:

- `0`: sleep
- `1`: dual DC
- `2`: stepper
- `3`: parallel DC

Winch motor IDs:

- `0`: motor A
- `1`: motor B

Winch lock channels:

- `0`: lock 1
- `1`: lock 2
- `2`: all locks

Signed percent arguments are encoded as `int8_t` in a single byte, range `-100..100`.

Winch motion and winch-lock commands return `FAULT` while E-Stop is active. `BrakeWinch` and winch sleep remain available.

## `0x21 CommandStatus`

Read length: 8 bytes.

```text
offset  size  field
0       1     sequence
1       1     busy
2       1     status
3       1     command
4       1     arg0
5       1     arg1
6       2     reserved
```

`sequence` increments when the PDU processes a command. `status` is one of the API status codes.

## `0x40 PmbusBridge`

Write length: 28 bytes.

```text
offset  size  field
0       1     rail_id
1       1     op
2       1     command
3       1     length
4       24    data
```

Fields:

- `rail_id`: `0=48V`, `1=24V`, `2=12V`
- `op`: `0=read`, `1=write`
- `command`: raw PMBus command byte
- `length`: number of bytes to write, or requested read length; clipped to 24
- `data`: write payload for `op=1`

The bridge lets RG access LM5066H1 registers through the PDU without connecting to the internal PMBus directly.

## `0x41 PmbusResult`

Read length: 31 bytes.

```text
offset  size  field
0       1     sequence
1       1     busy
2       1     status
3       1     rail_id
4       1     op
5       1     command
6       1     length
7       24    data
```

For PMBus reads, `length` is the number of valid bytes returned in `data`.

## `0x10 TelemetryAll`

Read length is the packed `ApiTelemetryAll` structure.

Top-level layout:

```text
offset  size  field
0       4     magic = "TLM1"
4       1     protocol_major
5       1     protocol_minor
6       1     mode
7       1     estop_active
8       1     pbit_passed
9       1     cbit_passed
10      2     pbit_failed
12      2     cbit_failed
14      4     uptime_ms
18      1     last_fault_valid
19      1     last_fault_code
20      1     last_fault_rail
21      1     reserved
22      2     last_fault_sequence
24      2     last_fault_status_word
26      2     last_fault_diag_word
28      2     reset_count
30      4     last_fault_uptime_ms
34      4     last_fault_unix_time_s
38      4     reset_flags
42      1     fault_history_count
43      1     fault_history_capacity
44      2     fault_history_dropped
46      12    winch telemetry
58      3 x rail telemetry
...           24 x fault records
```

Supervisor modes:

- `0`: BOOT
- `1`: PBIT
- `2`: NOMINAL
- `3`: DEGRADED
- `4`: FAULT
- `5`: ESTOP

Rail telemetry record:

```text
offset  size  field
0       1     rail_id
1       1     state
2       1     present
3       1     output_on
4       1     pgood
5       1     reserved
6       2     status_word
8       2     diag_word
10      2     status_mfr_specific2
12      2     fault_count
14      1     wd_plb_timer
15      3     rail_reserved
18      4     vin_mV
22      4     vout_mV
26      4     vaux_mV
30      4     iin_mA
34      4     pin_dW
38      1     peak_valid
39      3     peak_reserved
42      4     peak_vin_mV
46      4     peak_iin_mA
50      4     peak_pin_dW
54      2     die_temp_centiC
56      2     ntc_temp_centiC
58      1     bb_valid
59      1     bb_config
60      1     bb_timer
61      1     bb_ram_len
62      1     bb_eeprom_len
63      1     bb_ram_event
64      1     bb_ram_timer_expired
65      1     bb_ram_tick
66      1     bb_eeprom_event
67      1     bb_eeprom_timer_expired
68      1     bb_eeprom_tick
69      2     bb_reserved
71      16    bb_ram
87      16    bb_eeprom
```

Rail states:

- `0`: BOOT
- `1`: ABSENT
- `2`: READY
- `3`: RUNNING
- `4`: WARNING
- `5`: TRIPPED
- `6`: LATCHED

Winch telemetry:

```text
offset  size  field
0       1     mode
1       1     awake
2       1     fault_active
3       1     lock1_on
4       1     lock2_on
5       3     reserved
8       1     motor_a_cmd_pct
9       1     motor_b_cmd_pct
10      1     parallel_cmd_pct
11      1     stepper_a_cmd_pct
12      1     stepper_b_cmd_pct
```

Fault record:

```text
offset  size  field
0       1     valid
1       1     code
2       1     rail
3       1     reserved
4       2     sequence
6       2     status_word
8       2     diag_word
10      2     reset_count
12      4     uptime_ms
16      4     unix_time_s
20      4     reset_flags
```
# PDU Flight Firmware API

Flight-only firmware for the STM32F103C8T6 Power Distribution Unit controlling three LM5066H1 hot-swap controllers, the winch driver, winch locks, lights, and E-Stop outputs.

## Build And Monitor

```powershell
pio run -e bluepill_flight
pio run -e bluepill_flight -t upload
pio run -e bluepill_flight -t upload -t monitor
```

The only supported PlatformIO environment is `bluepill_flight`. Flashing uses ST-Link/SWD. Console output uses ITM/SWO on `PB3`; the upload+monitor command automatically starts the local SWO bridge and exposes decoded text on `socket://127.0.0.1:34430`.

## SWO Debug

Use VS Code `Run and Debug` with `Cortex-Debug: flight ST-Link + SWO` to build `bluepill_flight`, start OpenOCD, halt at `setup()`, and open the `PDU SWO console` decoder for ITM port 0. This keeps the flight-only build while restoring breakpoints, stepping, variables, and SWO console output.

## Hardware Buses

- `48V`: LM5066H1 at `0x52` on software SMBus `PA2=SCL`, `PA3=SDA`.
- `24V`: LM5066H1 at `0x43` on hardware `I2C1`, `PB6=SCL`, `PB7=SDA`.
- `12V`: LM5066H1 at `0x41` on hardware `I2C1`, `PB6=SCL`, `PB7=SDA`.
- PMBus/SMBus rate: `50 kHz`.
- External API: STM32 `I2C2` slave at 7-bit address `0x31`, `PB10=SCL`, `PB11=SDA`.

## I2C API Framing

All multi-byte fields are little-endian and packed with no padding.

For reads, write the one-byte API register selector, then perform an I2C read from the same slave address. For writes, send the selector followed by the packed request payload. Commands execute from the main loop; poll the status/result register after submitting a request.

Registers:

- `0x00 Info`: read `ApiInfo`.
- `0x10 TelemetryAll`: read full supervisor, rail, winch, and fault telemetry.
- `0x20 Command`: write `ApiCommandFrame`.
- `0x21 CommandStatus`: read `ApiCommandResult`.
- `0x40 PmbusBridge`: write `ApiPmbusBridgeRequest`.
- `0x41 PmbusResult`: read `ApiPmbusBridgeResult`.

Status codes: `0 OK`, `1 BUS_ERROR`, `2 TIMEOUT`, `3 PARAM`, `4 NOT_PRESENT`, `5 RANGE`, `6 PEC_MISMATCH`, `7 FAULT`, `8 NOT_INIT`, `9 INTERNAL`.

## Info Register

`ApiInfo` is 16 bytes:

- `magic[4]`: ASCII `PDU1`.
- `protocol_major`, `protocol_minor`: currently `1.9`.
- `fw_major`, `fw_minor`, `fw_patch`: firmware version from `platformio.ini`.
- `i2c_addr`: `0x31`.
- `rail_count`: `3`.
- `reserved[5]`.

## Command Register

Write 5 bytes to register `0x20`: `command, arg0, arg1, arg2, arg3`.

Command opcodes:

- `0x00 Noop`.
- `0x01 SetRailEnable`: `arg0 rail 0=48V, 1=24V, 2=12V, 3=all`; `arg1 0=off, 1=on`.
- `0x02 SetLedDuty`: `arg0 channel 0=Bras, 1=Avant, 2=Arriere, 3=Extra`; `arg1 duty 0..100`.
- `0x03 SetAllLeds`: `arg0 duty 0..100`.
- `0x04 SetLedPattern`: `arg0 pattern 0=off, 1=solid, 2=heartbeat, 3=fault blink, 4=E-Stop strobe`.
- `0x05 SetEstopLocal`: `arg0 0=release local PA0 command, 1=assert local PA0 command`.
- `0x06 SetEstopVtx`: `arg0 0=de-energize PB13 VTX, 1=energize PB13 VTX`.
- `0x07 ClearRailLatch`: `arg0 rail/all`.
- `0x08 ClearFaultLog`.
- `0x09 ResetDevice`: records a host reset fault and reboots the MCU.
- `0x0A SetUnixTime`: `arg0..arg3` are a little-endian Unix timestamp in seconds.
- `0x0B RefreshHotswapBlackBox`: `arg0 rail/all`.
- `0x0C EraseHotswapBlackBox`: `arg0 rail/all`.
- `0x30 SetWinchMode`: `arg0 0=sleep, 1=dual DC, 2=stepper, 3=parallel DC`.
- `0x31 SetWinchDcMotor`: `arg0 motor 0=A, 1=B`; `arg1 int8 percent -100..100`.
- `0x32 SetWinchParallelDc`: `arg0 int8 percent -100..100`.
- `0x33 SetWinchStepperPhases`: `arg0 phase A int8 percent`; `arg1 phase B int8 percent`.
- `0x34 BrakeWinch`.
- `0x35 ClearWinchFault`.
- `0x36 SetWinchLock`: `arg0 0=lock1, 1=lock2, 2=all`; `arg1 0=off, 1=on`.

Winch motion and winch-lock commands are rejected with `FAULT` while E-Stop is active. `BrakeWinch` and sleep mode remain available.

Read `CommandStatus` from register `0x21`: `sequence, busy, status, command, arg0, arg1, reserved[2]`. `sequence` increments when a command is processed.

## PMBus Bridge

Write 28 bytes to register `0x40`: `rail_id, op, command, length, data[24]`.

- `rail_id`: `0=48V`, `1=24V`, `2=12V`.
- `op`: `0=read`, `1=write`.
- `command`: raw PMBus/LM5066H1 command byte.
- `length`: payload length, clipped to 24 bytes.
- `data`: write payload for `op=1`; ignored for reads.

Read 31 bytes from register `0x41`: `sequence, busy, status, rail_id, op, command, length, data[24]`. For reads, `length` is the number of valid bytes returned in `data`.

## Telemetry Register

Read register `0x10` for `ApiTelemetryAll`.

Top-level fields include magic `TLM1`, protocol version, supervisor mode, E-Stop state, PBIT/CBIT status, uptime, fault history, winch telemetry, and three rail telemetry records.

Supervisor modes: `0=BOOT`, `1=PBIT`, `2=NOMINAL`, `3=DEGRADED`, `4=FAULT`, `5=ESTOP`.

Rail IDs: `0=48V`, `1=24V`, `2=12V`. Rail states: `0=BOOT`, `1=ABSENT`, `2=READY`, `3=RUNNING`, `4=WARNING`, `5=TRIPPED`, `6=LATCHED`.

Each rail record includes presence, output state, power-good level, LM5066H1 status words, fault count, `vin_mV`, `vout_mV`, `vaux_mV`, `iin_mA`, `pin_dW`, peak input-power sample, die/NTC temperatures in centi-degrees Celsius, and LM5066H1 black-box RAM/EEPROM bytes.

Winch telemetry includes mode, awake/fault state, both winch-lock states, and the last commanded percentages for dual DC, parallel DC, and stepper operation.

## Safety Behavior

- The firmware is flight-only: `PDU_FLIGHT_BUILD` is required at compile time.
- Bench bring-up bypasses and solenoid/VTX/E-Stop test builds are removed.
- Rails are not automatically re-enabled after E-Stop release; an explicit API command is required.
- Protection trips keep the MCU alive so SWO telemetry, the persistent fault log, LM5066H1 black-box data, and the external I2C API remain available.
