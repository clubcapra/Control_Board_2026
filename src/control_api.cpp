/* =============================================================================
 *  control_api.cpp - External I2C control/telemetry API implementation.
 * =============================================================================
 */
#include "control_api.h"

#include "avionics_config.h"
#include "estop.h"
#include "fault_log.h"
#include "leds.h"
#include "winch.h"
#include "winch_lock.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

/* Route every Serial.print in this TU through the SWO console.
 *
 * Without this, "Serial" resolves to the framework's HardwareSerial USART1,
 * which is never begin()-ed by the firmware (only the SWO alias is started
 * in main.cpp).  The first Serial.print would then spin forever inside
 * HardwareSerial::write() waiting for a TX-interrupt that never fires,
 * blowing the IWDG and causing a boot loop.
 *
 * PDU_API_DEBUG_SOURCE keeps console.h on the real SerialSWO path even when
 * PDU_API_DEBUG_ONLY is defined (other TUs use the silent console in that
 * build to suppress non-API logs).                                          */
#define PDU_API_DEBUG_SOURCE 1
#include "console.h"

namespace pdu {
namespace control_api {

namespace {

constexpr uint8_t kProtocolMajor = 1U;
constexpr uint8_t kProtocolMinor = 9U;
constexpr uint8_t kMaxBridgeData = 24U;
constexpr uint8_t kRailAll       = 3U;
constexpr uint8_t kFaultHistoryRecords = 24U;
constexpr uint8_t kBlackBoxBytes = static_cast<uint8_t>(kLm5066BlackBoxBytes);

TwoWire s_api_wire(cfg::kPin_API_I2C2_SDA, cfg::kPin_API_I2C2_SCL);

struct ApiInfo {
  uint8_t magic[4];
  uint8_t protocol_major;
  uint8_t protocol_minor;
  uint8_t fw_major;
  uint8_t fw_minor;
  uint8_t fw_patch;
  uint8_t i2c_addr;
  uint8_t rail_count;
  uint8_t reserved[5];
} __attribute__((packed));

struct ApiRailTelemetry {
  uint8_t  rail_id;
  uint8_t  state;
  uint8_t  present;
  uint8_t  output_on;
  uint8_t  pgood;
  uint8_t  reserved;
  uint16_t status_word;
  uint16_t diag_word;
  uint16_t status_mfr_specific2;
  uint16_t fault_count;
  uint8_t  wd_plb_timer;
  uint8_t  rail_reserved[3];
  int32_t  vin_mV;
  int32_t  vout_mV;
  int32_t  vaux_mV;
  int32_t  iin_mA;
  int32_t  pin_dW;
  uint8_t  peak_valid;
  uint8_t  peak_reserved[3];
  int32_t  peak_vin_mV;
  int32_t  peak_iin_mA;
  int32_t  peak_pin_dW;
  int16_t  die_temp_centiC;
  int16_t  ntc_temp_centiC;
  uint8_t  bb_valid;
  uint8_t  bb_config;
  uint8_t  bb_timer;
  uint8_t  bb_ram_len;
  uint8_t  bb_eeprom_len;
  uint8_t  bb_ram_event;
  uint8_t  bb_ram_timer_expired;
  uint8_t  bb_ram_tick;
  uint8_t  bb_eeprom_event;
  uint8_t  bb_eeprom_timer_expired;
  uint8_t  bb_eeprom_tick;
  uint8_t  bb_reserved[2];
  uint8_t  bb_ram[kBlackBoxBytes];
  uint8_t  bb_eeprom[kBlackBoxBytes];
} __attribute__((packed));

struct ApiFaultRecord {
  uint8_t  valid;
  uint8_t  code;
  uint8_t  rail;
  uint8_t  reserved;
  uint16_t sequence;
  uint16_t status_word;
  uint16_t diag_word;
  uint16_t reset_count;
  uint32_t uptime_ms;
  uint32_t unix_time_s;
  uint32_t reset_flags;
} __attribute__((packed));

struct ApiWinchTelemetry {
  uint8_t mode;
  uint8_t awake;
  uint8_t fault_active;
  uint8_t lock1_on;
  uint8_t lock2_on;
  uint8_t reserved[3];
  int8_t  motor_a_cmd_pct;
  int8_t  motor_b_cmd_pct;
  int8_t  parallel_cmd_pct;
  int8_t  stepper_a_cmd_pct;
  int8_t  stepper_b_cmd_pct;
} __attribute__((packed));

struct ApiTelemetryAll {
  uint8_t  magic[4];
  uint8_t  protocol_major;
  uint8_t  protocol_minor;
  uint8_t  mode;
  uint8_t  estop_active;
  uint8_t  pbit_passed;
  uint8_t  cbit_passed;
  uint16_t pbit_failed;
  uint16_t cbit_failed;
  uint32_t uptime_ms;
  uint8_t  last_fault_valid;
  uint8_t  last_fault_code;
  uint8_t  last_fault_rail;
  uint8_t  reserved;
  uint16_t last_fault_sequence;
  uint16_t last_fault_status_word;
  uint16_t last_fault_diag_word;
  uint16_t reset_count;
  uint32_t last_fault_uptime_ms;
  uint32_t last_fault_unix_time_s;
  uint32_t reset_flags;
  uint8_t  fault_history_count;
  uint8_t  fault_history_capacity;
  uint16_t fault_history_dropped;
  ApiWinchTelemetry winch;
  ApiRailTelemetry rail[3];
  ApiFaultRecord fault_history[kFaultHistoryRecords];
} __attribute__((packed));

struct ApiCommandFrame {
  uint8_t command;
  uint8_t arg0;
  uint8_t arg1;
  uint8_t arg2;
  uint8_t arg3;
} __attribute__((packed));

struct ApiCommandResult {
  uint8_t sequence;
  uint8_t busy;
  uint8_t status;
  uint8_t command;
  uint8_t arg0;
  uint8_t arg1;
  uint8_t reserved[2];
} __attribute__((packed));

struct ApiPmbusBridgeRequest {
  uint8_t rail_id;
  uint8_t op;
  uint8_t command;
  uint8_t length;
  uint8_t data[kMaxBridgeData];
} __attribute__((packed));

struct ApiPmbusBridgeResult {
  uint8_t sequence;
  uint8_t busy;
  uint8_t status;
  uint8_t rail_id;
  uint8_t op;
  uint8_t command;
  uint8_t length;
  uint8_t data[kMaxBridgeData];
} __attribute__((packed));

ApiInfo s_info = {
    {'P', 'D', 'U', '1'},
    kProtocolMajor,
    kProtocolMinor,
    cfg::kFwVersionMajor,
    cfg::kFwVersionMinor,
    cfg::kFwVersionPatch,
    cfg::kApiI2cAddress,
    3U,
    {0U, 0U, 0U, 0U, 0U},
};

ApiTelemetryAll    s_snapshot = {};
ApiCommandFrame    s_pending_command = {};
ApiCommandResult   s_command_result = {};
ApiPmbusBridgeRequest s_pending_bridge = {};
ApiPmbusBridgeResult  s_bridge_result = {};
fault_log::Record  s_fault_history_scratch[kFaultHistoryRecords] = {};

volatile bool    s_command_pending = false;
volatile bool    s_bridge_pending  = false;
volatile uint8_t s_read_register   = static_cast<uint8_t>(Register::kInfo);
uint8_t          s_sequence        = 0U;
bool             s_initialised     = false;

// #region agent log
/* Counters bumped from the I2C2 ISR callbacks; printed by the throttled
 * dbgDumpI2c2State() so we can see whether the slave is still receiving
 * any traffic at all when the master starts timing out.                   */
volatile uint32_t s_dbg_rx_count = 0U;
volatile uint32_t s_dbg_tx_count = 0U;
// #endregion

void printHex2(uint8_t value) {
  if (value < 0x10U) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

#if defined(PDU_API_DEBUG_ONLY)
enum class ApiDebugKind : uint8_t {
  kRxSelect = 0U,
  kRxCommand = 1U,
  kRxBridge = 2U,
  kTx = 3U,
  kOverflow = 4U,
};

struct ApiDebugEvent {
  ApiDebugKind kind;
  uint8_t reg;
  uint8_t count;
  uint8_t arg0;
  uint8_t arg1;
  uint8_t arg2;
};

constexpr uint8_t kApiDebugQueueLen = 32U;
volatile uint8_t s_api_debug_head = 0U;
volatile uint8_t s_api_debug_tail = 0U;
volatile uint16_t s_api_debug_dropped = 0U;
ApiDebugEvent s_api_debug_queue[kApiDebugQueueLen] = {};

void queueApiDebug(ApiDebugKind kind,
                   uint8_t reg,
                   uint8_t count,
                   uint8_t arg0,
                   uint8_t arg1,
                   uint8_t arg2) {
  const uint8_t next =
      static_cast<uint8_t>((s_api_debug_head + 1U) % kApiDebugQueueLen);
  if (next == s_api_debug_tail) {
    ++s_api_debug_dropped;
    return;
  }
  s_api_debug_queue[s_api_debug_head] = {kind, reg, count, arg0, arg1, arg2};
  s_api_debug_head = next;
}

uint8_t responseLengthForRegister(uint8_t reg) {
  if (reg == static_cast<uint8_t>(Register::kInfo)) {
    return static_cast<uint8_t>(sizeof(ApiInfo));
  }
  if (reg == static_cast<uint8_t>(Register::kCommandStatus)) {
    return static_cast<uint8_t>(sizeof(ApiCommandResult));
  }
  if (reg == static_cast<uint8_t>(Register::kPmbusResult)) {
    return static_cast<uint8_t>(sizeof(ApiPmbusBridgeResult));
  }
  if (reg == static_cast<uint8_t>(Register::kTelemetryAll)) {
    return 0xFFU;  /* larger than a byte; print as "all telemetry". */
  }
  return 1U;
}

void flushApiDebug() {
  for (;;) {
    uint16_t dropped = 0U;
    ApiDebugEvent ev = {};
    bool have_event = false;

    noInterrupts();
    if (s_api_debug_dropped != 0U) {
      dropped = s_api_debug_dropped;
      s_api_debug_dropped = 0U;
    } else if (s_api_debug_tail != s_api_debug_head) {
      ev = s_api_debug_queue[s_api_debug_tail];
      s_api_debug_tail =
          static_cast<uint8_t>((s_api_debug_tail + 1U) % kApiDebugQueueLen);
      have_event = true;
    }
    interrupts();

    if (dropped != 0U) {
      Serial.print(F("[API DROP] events="));
      Serial.println(dropped);
      continue;
    }
    if (!have_event) {
      break;
    }

    if (ev.kind == ApiDebugKind::kRxSelect) {
      Serial.print(F("[API RX] select reg=0x"));
      printHex2(ev.reg);
      Serial.print(F(" bytes="));
      Serial.println(ev.count);
    } else if (ev.kind == ApiDebugKind::kRxCommand) {
      Serial.print(F("[API RX] command reg=0x"));
      printHex2(ev.reg);
      Serial.print(F(" len="));
      Serial.print(ev.count);
      Serial.print(F(" cmd=0x"));
      printHex2(ev.arg0);
      Serial.print(F(" arg0=0x"));
      printHex2(ev.arg1);
      Serial.print(F(" arg1=0x"));
      printHex2(ev.arg2);
      Serial.println();
    } else if (ev.kind == ApiDebugKind::kRxBridge) {
      Serial.print(F("[API RX] pmbus reg=0x"));
      printHex2(ev.reg);
      Serial.print(F(" len="));
      Serial.print(ev.count);
      Serial.print(F(" rail="));
      Serial.print(ev.arg0);
      Serial.print(F(" op="));
      Serial.print(ev.arg1);
      Serial.print(F(" cmd=0x"));
      printHex2(ev.arg2);
      Serial.println();
    } else if (ev.kind == ApiDebugKind::kTx) {
      Serial.print(F("[API TX] reg=0x"));
      printHex2(ev.reg);
      if (ev.arg0 == 0xFFU) {
        Serial.println(F(" len=telemetry_all"));
      } else {
        Serial.print(F(" len="));
        Serial.println(ev.arg0);
      }
    }
  }
}
#endif

int32_t scale1000(double value) {
  if (isnan(value) || value > 2147483.0 || value < -2147483.0) {
    return 0;
  }
  return static_cast<int32_t>(lround(value * 1000.0));
}

int32_t scale10(double value) {
  if (isnan(value) || value > 214748364.0 || value < -214748364.0) {
    return 0;
  }
  return static_cast<int32_t>(lround(value * 10.0));
}

int16_t scaleTemp(double value) {
  if (isnan(value) || value > 327.67 || value < -327.68) {
    return static_cast<int16_t>(0x8000);
  }
  return static_cast<int16_t>(lround(value * 100.0));
}

rail::Controller* railById(uint8_t rail_id,
                           rail::Controller& r48,
                           rail::Controller& r24,
                           rail::Controller& r12) {
  if (rail_id == static_cast<uint8_t>(Rail::k48V)) {
    return &r48;
  }
  if (rail_id == static_cast<uint8_t>(Rail::k24V)) {
    return &r24;
  }
  if (rail_id == static_cast<uint8_t>(Rail::k12V)) {
    return &r12;
  }
  return nullptr;
}

void fillRail(ApiRailTelemetry& out, const rail::Controller& rail) {
  RailTelemetry tlm = {};
  rail.buildTelemetry(tlm);
  out.rail_id = static_cast<uint8_t>(tlm.rail);
  out.state = static_cast<uint8_t>(rail.state());
  out.present = tlm.present ? 1U : 0U;
  out.output_on = tlm.output_on ? 1U : 0U;
  out.pgood = tlm.pgood_pin ? 1U : 0U;
  out.reserved = 0U;
  out.status_word = tlm.status_word;
  out.diag_word = tlm.diag_word;
  out.status_mfr_specific2 = tlm.status_mfr_specific2;
  out.fault_count = static_cast<uint16_t>(rail.faultCount() & 0xFFFFU);
  out.wd_plb_timer = tlm.wd_plb_timer;
  out.rail_reserved[0] = 0U;
  out.rail_reserved[1] = 0U;
  out.rail_reserved[2] = 0U;
  out.vin_mV = scale1000(tlm.vin_v);
  out.vout_mV = scale1000(tlm.vout_v);
  out.vaux_mV = scale1000(tlm.vaux_v);
  out.iin_mA = scale1000(tlm.iin_a);
  out.pin_dW = scale10(tlm.pin_w);
  out.peak_valid = tlm.peak_valid ? 1U : 0U;
  out.peak_reserved[0] = 0U;
  out.peak_reserved[1] = 0U;
  out.peak_reserved[2] = 0U;
  out.peak_vin_mV = scale1000(tlm.peak_vin_v);
  out.peak_iin_mA = scale1000(tlm.peak_iin_a);
  out.peak_pin_dW = scale10(tlm.peak_pin_w);
  out.die_temp_centiC = scaleTemp(tlm.die_temp_c);
  out.ntc_temp_centiC = scaleTemp(tlm.ntc_temp_c);
  out.bb_valid = tlm.bb_valid ? 1U : 0U;
  out.bb_config = tlm.bb_config;
  out.bb_timer = tlm.bb_timer;
  out.bb_ram_len = tlm.bb_ram_len;
  out.bb_eeprom_len = tlm.bb_eeprom_len;
  const uint8_t ram_event = tlm.bb_ram[0];
  const uint8_t eeprom_event = tlm.bb_eeprom[0];
  out.bb_ram_event = (ram_event >> 5U) & 0x07U;
  out.bb_ram_timer_expired = ((ram_event >> 4U) & 0x01U) != 0U ? 1U : 0U;
  out.bb_ram_tick = ram_event & 0x0FU;
  out.bb_eeprom_event = (eeprom_event >> 5U) & 0x07U;
  out.bb_eeprom_timer_expired = ((eeprom_event >> 4U) & 0x01U) != 0U ? 1U : 0U;
  out.bb_eeprom_tick = eeprom_event & 0x0FU;
  out.bb_reserved[0] = 0U;
  out.bb_reserved[1] = 0U;
  for (size_t i = 0U; i < kLm5066BlackBoxBytes; ++i) {
    out.bb_ram[i] = tlm.bb_ram[i];
    out.bb_eeprom[i] = tlm.bb_eeprom[i];
  }
}

ApiFaultRecord toApiFaultRecord(const fault_log::Record& in) {
  ApiFaultRecord out = {};
  out.valid = (in.valid && in.code != fault_log::Code::kNone) ? 1U : 0U;
  out.code = static_cast<uint8_t>(in.code);
  out.rail = static_cast<uint8_t>(in.rail);
  out.reserved = 0U;
  out.sequence = in.sequence;
  out.status_word = in.status_word;
  out.diag_word = in.diag_word;
  out.reset_count = in.reset_count;
  out.uptime_ms = in.fault_uptime_ms;
  out.unix_time_s = in.unix_time_s;
  out.reset_flags = in.reset_flags;
  return out;
}

ApiWinchTelemetry toApiWinchTelemetry(const winch::Telemetry& in) {
  ApiWinchTelemetry out = {};
  const winch_lock::Telemetry locks = winch_lock::telemetry();
  out.mode = static_cast<uint8_t>(in.mode);
  out.awake = in.awake ? 1U : 0U;
  out.fault_active = in.fault_active ? 1U : 0U;
  out.lock1_on = locks.lock1_on ? 1U : 0U;
  out.lock2_on = locks.lock2_on ? 1U : 0U;
  out.reserved[0] = 0U;
  out.reserved[1] = 0U;
  out.reserved[2] = 0U;
  out.motor_a_cmd_pct = in.motor_a_cmd_pct;
  out.motor_b_cmd_pct = in.motor_b_cmd_pct;
  out.parallel_cmd_pct = in.parallel_cmd_pct;
  out.stepper_a_cmd_pct = in.stepper_a_cmd_pct;
  out.stepper_b_cmd_pct = in.stepper_b_cmd_pct;
  return out;
}

uint32_t commandArg32(const ApiCommandFrame& cmd) {
  return static_cast<uint32_t>(cmd.arg0) |
         (static_cast<uint32_t>(cmd.arg1) << 8U) |
         (static_cast<uint32_t>(cmd.arg2) << 16U) |
         (static_cast<uint32_t>(cmd.arg3) << 24U);
}

void publishSnapshot(rail::Controller& r48,
                     rail::Controller& r24,
                     rail::Controller& r12,
                     SupervisorMode mode,
                     const bit::Report& pbit,
                     const bit::Report& cbit) {
  /* Rate-limit to 20 Hz.  publishSnapshot's critical section disables IRQs
   * for the duration of a 944-byte struct copy (~50 us at 72 MHz).  In a
   * tight loop (notably the API_DEBUG_ONLY image, which has nothing else
   * to do) that fires thousands of times per second and starves the I2C2
   * slave ISR enough to corrupt CR1.ACK / Listen state - the master then
   * sees every address byte NACKed.  20 Hz is plenty for telemetry and
   * leaves >99.9% of wall-time available for the I2C ISR.                 */
  static uint32_t s_last_publish_ms = 0U;
  const uint32_t now = millis();
  if ((now - s_last_publish_ms) < 50U) {
    return;
  }
  s_last_publish_ms = now;

  ApiTelemetryAll next = {};
  next.magic[0] = 'T';
  next.magic[1] = 'L';
  next.magic[2] = 'M';
  next.magic[3] = '1';
  next.protocol_major = kProtocolMajor;
  next.protocol_minor = kProtocolMinor;
  next.mode = static_cast<uint8_t>(mode);
  next.estop_active = estop::isEStopActive() ? 1U : 0U;
  next.pbit_passed = pbit.all_passed ? 1U : 0U;
  next.cbit_passed = cbit.all_passed ? 1U : 0U;
  next.pbit_failed = static_cast<uint16_t>(pbit.tests_failed & 0xFFFFU);
  next.cbit_failed = static_cast<uint16_t>(cbit.tests_failed & 0xFFFFU);
  next.uptime_ms = millis();
  const fault_log::Record fault = fault_log::last();
  next.last_fault_valid =
      (fault.valid && fault.code != fault_log::Code::kNone) ? 1U : 0U;
  next.last_fault_code = static_cast<uint8_t>(fault.code);
  next.last_fault_rail = static_cast<uint8_t>(fault.rail);
  next.reserved = 0U;
  next.last_fault_sequence = fault.sequence;
  next.last_fault_status_word = fault.status_word;
  next.last_fault_diag_word = fault.diag_word;
  next.reset_count = fault_log::resetCount();
  next.last_fault_uptime_ms = fault.fault_uptime_ms;
  next.last_fault_unix_time_s = fault.unix_time_s;
  next.reset_flags = fault_log::bootResetFlags();
  next.fault_history_capacity = static_cast<uint8_t>(fault_log::capacity());
  const size_t copied = fault_log::copyLatest(s_fault_history_scratch,
                                              kFaultHistoryRecords);
  next.fault_history_count = static_cast<uint8_t>(copied);
  if (fault.sequence > copied) {
    next.fault_history_dropped = static_cast<uint16_t>(fault.sequence - copied);
  } else {
    next.fault_history_dropped = 0U;
  }
  for (size_t i = 0U; i < copied; ++i) {
    next.fault_history[i] = toApiFaultRecord(s_fault_history_scratch[i]);
  }
  next.winch = toApiWinchTelemetry(winch::telemetry());
  fillRail(next.rail[0], r48);
  fillRail(next.rail[1], r24);
  fillRail(next.rail[2], r12);

  noInterrupts();
  s_snapshot = next;
  interrupts();
}

Status applyToRails(uint8_t rail_id,
                    rail::Controller& r48,
                    rail::Controller& r24,
                    rail::Controller& r12,
                    Status (*fn)(rail::Controller&)) {
  if (rail_id == kRailAll) {
    Status rc = Status::kOk;
    const Status a = fn(r48);
    const Status b = fn(r24);
    const Status c = fn(r12);
    if (a != Status::kOk) { rc = a; }
    if (b != Status::kOk) { rc = b; }
    if (c != Status::kOk) { rc = c; }
    return rc;
  }
  rail::Controller* r = railById(rail_id, r48, r24, r12);
  return (r != nullptr) ? fn(*r) : Status::kParam;
}

Status enableRail(rail::Controller& r) { return r.enable(); }
Status disableRail(rail::Controller& r) { return r.disable(); }
Status clearRail(rail::Controller& r) { return r.clearLatch(); }
Status refreshRailBlackBox(rail::Controller& r) { return r.refreshBlackBoxMemory(); }
Status eraseRailBlackBox(rail::Controller& r) {
  if (!r.device().bbClear()) {
    return Status::kBusError;
  }
  if (!r.device().bbErase()) {
    return Status::kBusError;
  }
  return r.refreshBlackBoxMemory();
}

int8_t signedArg(uint8_t value) {
  return static_cast<int8_t>(value);
}

void processCommand(rail::Controller& r48,
                    rail::Controller& r24,
                    rail::Controller& r12) {
  ApiCommandFrame cmd = {};
  bool pending = false;
  noInterrupts();
  if (s_command_pending) {
    cmd = s_pending_command;
    s_command_pending = false;
    pending = true;
  }
  interrupts();
  if (!pending) {
    return;
  }

  Status rc = Status::kOk;
  switch (static_cast<Command>(cmd.command)) {
    case Command::kNoop:
      break;
    case Command::kSetRailEnable:
      rc = applyToRails(cmd.arg0, r48, r24, r12,
                        (cmd.arg1 != 0U) ? enableRail : disableRail);
      break;
    case Command::kSetLedDuty:
      rc = leds::setPattern(leds::Pattern::kSolid);
      if (rc == Status::kOk) {
        rc = leds::setDuty(static_cast<leds::Channel>(cmd.arg0), cmd.arg1);
      }
      break;
    case Command::kSetAllLeds:
      rc = leds::setPattern(leds::Pattern::kSolid);
      if (rc == Status::kOk) {
        rc = leds::setAll(cmd.arg0);
      }
      break;
    case Command::kSetLedPattern:
      rc = leds::setPattern(static_cast<leds::Pattern>(cmd.arg0));
      break;
    case Command::kSetEstopLocal:
      estop::assertLocal(cmd.arg0 != 0U);
      break;
    case Command::kSetEstopVtx:
      estop::setVtx(cmd.arg0 != 0U);
      break;
    case Command::kClearRailLatch:
      rc = applyToRails(cmd.arg0, r48, r24, r12, clearRail);
      break;
    case Command::kClearFaultLog:
      fault_log::clear();
      break;
    case Command::kResetDevice:
      s_command_result.sequence = ++s_sequence;
      s_command_result.busy = 0U;
      s_command_result.status = static_cast<uint8_t>(Status::kOk);
      s_command_result.command = cmd.command;
      fault_log::recordAndReset(fault_log::Code::kHostRequestedReset,
                                Rail::kCount,
                                0U,
                                0U);
      break;
    case Command::kSetUnixTime:
      fault_log::setUnixTime(commandArg32(cmd));
      break;
    case Command::kRefreshHotswapBlackBox:
      rc = applyToRails(cmd.arg0, r48, r24, r12, refreshRailBlackBox);
      break;
    case Command::kEraseHotswapBlackBox:
      rc = applyToRails(cmd.arg0, r48, r24, r12, eraseRailBlackBox);
      break;
    case Command::kSetWinchMode:
      if (estop::isEStopActive() && cmd.arg0 != static_cast<uint8_t>(winch::Mode::kSleep)) {
        rc = Status::kFault;
      } else {
        rc = winch::setMode(static_cast<winch::Mode>(cmd.arg0));
      }
      break;
    case Command::kSetWinchDcMotor:
      rc = estop::isEStopActive()
               ? Status::kFault
               : winch::setDcMotor(static_cast<winch::Motor>(cmd.arg0),
                                   signedArg(cmd.arg1));
      break;
    case Command::kSetWinchParallelDc:
      rc = estop::isEStopActive()
               ? Status::kFault
               : winch::setParallelDc(signedArg(cmd.arg0));
      break;
    case Command::kSetWinchStepperPhases:
      rc = estop::isEStopActive()
               ? Status::kFault
               : winch::setStepperPhases(signedArg(cmd.arg0),
                                         signedArg(cmd.arg1));
      break;
    case Command::kBrakeWinch:
      rc = winch::brakeAll();
      break;
    case Command::kClearWinchFault:
      rc = winch::clearFault();
      break;
    case Command::kSetWinchLock:
      rc = estop::isEStopActive()
               ? Status::kFault
               : winch_lock::set(static_cast<winch_lock::Channel>(cmd.arg0),
                                 cmd.arg1 != 0U);
      break;
    default:
      rc = Status::kParam;
      break;
  }

  ApiCommandResult result = {};
  result.sequence = ++s_sequence;
  result.busy = 0U;
  result.status = static_cast<uint8_t>(rc);
  result.command = cmd.command;
  result.arg0 = cmd.arg0;
  result.arg1 = cmd.arg1;
  noInterrupts();
  s_command_result = result;
  interrupts();
}

void processBridge(rail::Controller& r48,
                   rail::Controller& r24,
                   rail::Controller& r12) {
  ApiPmbusBridgeRequest req = {};
  bool pending = false;
  noInterrupts();
  if (s_bridge_pending) {
    req = s_pending_bridge;
    s_bridge_pending = false;
    pending = true;
  }
  interrupts();
  if (!pending) {
    return;
  }

  ApiPmbusBridgeResult result = {};
  result.sequence = ++s_sequence;
  result.busy = 0U;
  result.rail_id = req.rail_id;
  result.op = req.op;
  result.command = req.command;

  Status rc = Status::kOk;
  rail::Controller* r = railById(req.rail_id, r48, r24, r12);
  if (r == nullptr || req.length > kMaxBridgeData) {
    rc = Status::kParam;
  } else if (static_cast<BridgeOp>(req.op) == BridgeOp::kRead) {
    size_t out_len = 0U;
    const bool auto_len = req.length == 0U;
    const LM5066H1::RegisterDescriptor* desc =
        LM5066H1::findRegisterDescriptor(req.command);
    const size_t max_len = auto_len ? kMaxBridgeData : req.length;
    if (auto_len && desc == nullptr) {
      rc = Status::kParam;
    } else if (r->device().readRegisterRaw(req.command, result.data, max_len,
                                          out_len)) {
      result.length = static_cast<uint8_t>(out_len);
    } else {
      rc = Status::kBusError;
    }
  } else if (static_cast<BridgeOp>(req.op) == BridgeOp::kWrite) {
    if (!r->device().writeRegisterRaw(req.command, req.data, req.length)) {
      rc = Status::kBusError;
    }
    result.length = 0U;
  } else {
    rc = Status::kParam;
  }

  result.status = static_cast<uint8_t>(rc);
  noInterrupts();
  s_bridge_result = result;
  interrupts();
}

void onReceive(int count) {
  // #region agent log
  /* ISR context - keep absolutely minimal; bump a counter that the
   * foreground throttled dump prints out every 250 ms.                    */
  ++s_dbg_rx_count;
  // #endregion
  if (count <= 0) {
    return;
  }

#if defined(PDU_API_DEBUG_ONLY)
  const uint8_t rx_count = static_cast<uint8_t>(count > 255 ? 255 : count);
#endif
  const int reg = s_api_wire.read();
  s_read_register = static_cast<uint8_t>(reg);
  --count;
  if (count <= 0) {
#if defined(PDU_API_DEBUG_ONLY)
    queueApiDebug(ApiDebugKind::kRxSelect, static_cast<uint8_t>(reg),
                  rx_count, 0U, 0U, 0U);
#endif
    return;
  }

  if (reg == static_cast<int>(Register::kCommand)) {
    ApiCommandFrame cmd = {};
    uint8_t* dst = reinterpret_cast<uint8_t*>(&cmd);
    const size_t max_len = sizeof(cmd);
    size_t i = 0U;
    while (s_api_wire.available() && i < max_len) {
      dst[i++] = static_cast<uint8_t>(s_api_wire.read());
    }
    s_pending_command = cmd;
    s_command_pending = true;
#if defined(PDU_API_DEBUG_ONLY)
    queueApiDebug(ApiDebugKind::kRxCommand, static_cast<uint8_t>(reg),
                  static_cast<uint8_t>(i), cmd.command, cmd.arg0, cmd.arg1);
#endif
  } else if (reg == static_cast<int>(Register::kPmbusBridge)) {
    ApiPmbusBridgeRequest req = {};
    uint8_t* dst = reinterpret_cast<uint8_t*>(&req);
    const size_t max_len = sizeof(req);
    size_t i = 0U;
    while (s_api_wire.available() && i < max_len) {
      dst[i++] = static_cast<uint8_t>(s_api_wire.read());
    }
    if (req.length > kMaxBridgeData) {
      req.length = kMaxBridgeData;
    }
    s_pending_bridge = req;
    s_bridge_pending = true;
#if defined(PDU_API_DEBUG_ONLY)
    queueApiDebug(ApiDebugKind::kRxBridge, static_cast<uint8_t>(reg),
                  static_cast<uint8_t>(i), req.rail_id, req.op, req.command);
#endif
  } else {
    while (s_api_wire.available()) {
      (void)s_api_wire.read();
    }
#if defined(PDU_API_DEBUG_ONLY)
    queueApiDebug(ApiDebugKind::kRxSelect, static_cast<uint8_t>(reg),
                  rx_count, 0U, 0U, 0U);
#endif
  }
}

void onRequest() {
  // #region agent log
  ++s_dbg_tx_count;
  // #endregion
  const uint8_t reg = s_read_register;
#if defined(PDU_API_DEBUG_ONLY)
  queueApiDebug(ApiDebugKind::kTx, reg, 0U, responseLengthForRegister(reg),
                0U, 0U);
#endif
  if (reg == static_cast<uint8_t>(Register::kInfo)) {
    s_api_wire.write(reinterpret_cast<const uint8_t*>(&s_info), sizeof(s_info));
  } else if (reg == static_cast<uint8_t>(Register::kTelemetryAll)) {
    s_api_wire.write(reinterpret_cast<const uint8_t*>(&s_snapshot), sizeof(s_snapshot));
  } else if (reg == static_cast<uint8_t>(Register::kCommandStatus)) {
    s_api_wire.write(reinterpret_cast<const uint8_t*>(&s_command_result),
                     sizeof(s_command_result));
  } else if (reg == static_cast<uint8_t>(Register::kPmbusResult)) {
    s_api_wire.write(reinterpret_cast<const uint8_t*>(&s_bridge_result),
                     sizeof(s_bridge_result));
  } else {
    const uint8_t zero = 0U;
    s_api_wire.write(&zero, 1U);
  }
}

}  // namespace

Status init() {
  if (s_initialised) {
    return Status::kOk;
  }

  s_snapshot.magic[0] = 'T';
  s_snapshot.magic[1] = 'L';
  s_snapshot.magic[2] = 'M';
  s_snapshot.magic[3] = '1';
  s_snapshot.protocol_major = kProtocolMajor;
  s_snapshot.protocol_minor = kProtocolMinor;

  /* Order matters here.
   *
   * stm32duino's TwoWire::begin(uint8_t address) hardcodes the I2C
   * peripheral clock to 100 kHz inside i2c_init(). Calling setClock()
   * BEFORE begin() therefore has no lasting effect: begin() overwrites
   * it. To actually run the bus at kApiI2cClock_Hz we must:
   *   1) bind the SCL/SDA pins,
   *   2) call begin(addr) so HAL_I2C_Init configures the slave (this
   *      also calls HAL_I2C_EnableListen_IT internally),
   *   3) call setClock() to retune CCR/TRISE to the desired frequency,
   *   4) re-arm slave listen mode, because i2c_setTiming() calls
   *      HAL_I2C_Init() again which clears the I2C_IT_EVT|I2C_IT_ERR
   *      enable bits and drops the slave back to STATE_READY (no
   *      longer LISTEN). Without this the slave will silently NACK
   *      every address byte from the master.                          */
  s_api_wire.setSCL(cfg::kPin_API_I2C2_SCL);
  s_api_wire.setSDA(cfg::kPin_API_I2C2_SDA);
  s_api_wire.begin(cfg::kApiI2cAddress);
  s_api_wire.setClock(cfg::kApiI2cClock_Hz);
  (void)HAL_I2C_EnableListen_IT(s_api_wire.getHandle());
  s_api_wire.onReceive(onReceive);
  s_api_wire.onRequest(onRequest);
  s_initialised = true;
  Serial.print(F("[API INIT] I2C2 slave addr=0x"));
  printHex2(cfg::kApiI2cAddress);
  Serial.print(F(" clock="));
  Serial.print(cfg::kApiI2cClock_Hz);
  Serial.println(F("Hz pins SCL=PB10 SDA=PB11"));
  return Status::kOk;
}

// #region agent log
/* Periodic dump of the live I2C2 state to SWO so we can correlate the
 * "WAIT_TIMEOUT on RG side" failure with a specific stuck-bit on the
 * slave side.  Throttled to once per 250 ms to avoid flooding SWO.        */
static void dbgDumpI2c2State(const char* tag) {
  I2C_HandleTypeDef* const h = s_api_wire.getHandle();
  if (h == nullptr || h->Instance == nullptr) {
    return;
  }
  const uint32_t cr1 = h->Instance->CR1;
  const uint32_t sr1 = h->Instance->SR1;
  const uint32_t sr2 = h->Instance->SR2;
  const uint32_t st  = static_cast<uint32_t>(h->State);
  Serial.print(F("[I2C2 DBG "));
  Serial.print(tag);
  Serial.print(F("] CR1=0x"));   Serial.print(cr1, HEX);
  Serial.print(F(" SR1=0x"));    Serial.print(sr1, HEX);
  Serial.print(F(" SR2=0x"));    Serial.print(sr2, HEX);
  Serial.print(F(" hal=0x"));    Serial.print(st, HEX);
  Serial.print(F(" PE="));       Serial.print((cr1 & I2C_CR1_PE)  ? 1 : 0);
  Serial.print(F(" ACK="));      Serial.print((cr1 & I2C_CR1_ACK) ? 1 : 0);
  Serial.print(F(" BUSY="));     Serial.print((sr2 & I2C_SR2_BUSY) ? 1 : 0);
  Serial.print(F(" BERR="));     Serial.print((sr1 & I2C_SR1_BERR)  ? 1 : 0);
  Serial.print(F(" ARLO="));     Serial.print((sr1 & I2C_SR1_ARLO)  ? 1 : 0);
  Serial.print(F(" AF="));       Serial.print((sr1 & I2C_SR1_AF)    ? 1 : 0);
  Serial.print(F(" OVR="));      Serial.print((sr1 & I2C_SR1_OVR)   ? 1 : 0);
  Serial.print(F(" PECERR="));   Serial.print((sr1 & I2C_SR1_PECERR) ? 1 : 0);
  uint32_t rx, tx;
  noInterrupts();
  rx = s_dbg_rx_count;
  tx = s_dbg_tx_count;
  interrupts();
  Serial.print(F(" rx#="));      Serial.print(rx);
  Serial.print(F(" tx#="));      Serial.println(tx);
}
// #endregion

/**
 * @brief  Self-heal the I2C2 slave if a missed event left it deaf.
 *
 *  The bit-banged master on RoboGuard is timing-tolerant on its own pins,
 *  but if a long IRQ-disabled window (long memcpy, long SWO drain, etc.)
 *  causes the F1 I2C peripheral to miss its STOPF/AF window the slave can
 *  end up with CR1.ACK cleared but obj->slaveMode still LISTEN at the
 *  twi.c level - the user's HAL_I2C_EnableListen_IT() never gets re-fired
 *  and every subsequent address byte NACKs forever.  Catch that case
 *  every loop iteration and re-arm.  Cost is two register reads in the
 *  hot path; correctness is worth it.
 */
static void rearmListenIfStuck() {
  if (!s_initialised) {
    return;
  }
  I2C_HandleTypeDef* const h = s_api_wire.getHandle();
  if (h == nullptr || h->Instance == nullptr) {
    return;
  }

  const uint32_t cr1 = h->Instance->CR1;
  const bool pe_set      = (cr1 & I2C_CR1_PE)  != 0U;
  const bool ack_set     = (cr1 & I2C_CR1_ACK) != 0U;
  const bool state_listen = (h->State == HAL_I2C_STATE_LISTEN);

  // #region agent log
  /* Throttled snapshot every 250 ms regardless of action - so we can see
   * if the peripheral looks "fine" (PE=ACK=1, state=LISTEN) but is
   * actually NACKing because of a stuck error/BUSY flag.                  */
  {
    static uint32_t s_dbg_last_ms = 0U;
    const uint32_t now_ms = millis();
    if ((now_ms - s_dbg_last_ms) >= 250U) {
      s_dbg_last_ms = now_ms;
      dbgDumpI2c2State("tick");
    }
  }
  // #endregion

  /* If we're properly armed, leave it alone. */
  if (pe_set && ack_set && state_listen) {
    return;
  }

  /* If a transfer is genuinely in progress (we just clocked into BUSY_TX_LISTEN
   * via AddrCallback and the master is mid-byte), don't tear it down.       */
  if (h->State == HAL_I2C_STATE_BUSY_TX_LISTEN ||
      h->State == HAL_I2C_STATE_BUSY_RX_LISTEN ||
      h->State == HAL_I2C_STATE_BUSY_TX ||
      h->State == HAL_I2C_STATE_BUSY_RX) {
    return;
  }

  // #region agent log
  Serial.println(F("[I2C2 RECOV] re-arming stuck slave"));
  dbgDumpI2c2State("pre-reset");
  // #endregion

  /* Otherwise force the peripheral back to a known-good READY state and
   * re-enable Listen mode + ACK + EVT/ERR interrupts.                       */
  __HAL_I2C_DISABLE(h);
  h->Instance->CR1 |= I2C_CR1_SWRST;
  h->Instance->CR1 &= ~I2C_CR1_SWRST;
  h->State = HAL_I2C_STATE_RESET;
  (void)HAL_I2C_Init(h);
  (void)HAL_I2C_EnableListen_IT(h);

  // #region agent log
  dbgDumpI2c2State("post-reset");
  // #endregion
}

void tick(rail::Controller& r48,
          rail::Controller& r24,
          rail::Controller& r12,
          SupervisorMode mode,
          const bit::Report& pbit,
          const bit::Report& cbit) {
#if defined(PDU_API_DEBUG_ONLY)
  flushApiDebug();
#endif
  if (!s_initialised) {
    return;
  }

  rearmListenIfStuck();

  processCommand(r48, r24, r12);
  processBridge(r48, r24, r12);
  /* publishSnapshot is internally rate-limited to 20 Hz, so the critical
   * 944-byte memcpy under noInterrupts() runs at most every 50 ms.  In the
   * API_DEBUG_ONLY image rails/winch/leds are never initialised, so the
   * snapshot is mostly zeroes - that's fine, the master cannot read more
   * than 32 bytes anyway via the slave's hardware buffer.                   */
  publishSnapshot(r48, r24, r12, mode, pbit, cbit);
}

}  /* namespace control_api */
}  /* namespace pdu */
