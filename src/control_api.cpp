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
  if (count <= 0) {
    return;
  }

  const int reg = s_api_wire.read();
  s_read_register = static_cast<uint8_t>(reg);
  --count;
  if (count <= 0) {
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
  } else {
    while (s_api_wire.available()) {
      (void)s_api_wire.read();
    }
  }
}

void onRequest() {
  const uint8_t reg = s_read_register;
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

  s_api_wire.setSCL(cfg::kPin_API_I2C2_SCL);
  s_api_wire.setSDA(cfg::kPin_API_I2C2_SDA);
  s_api_wire.begin(cfg::kApiI2cAddress);
  s_api_wire.onReceive(onReceive);
  s_api_wire.onRequest(onRequest);
  s_initialised = true;
  return Status::kOk;
}

void tick(rail::Controller& r48,
          rail::Controller& r24,
          rail::Controller& r12,
          SupervisorMode mode,
          const bit::Report& pbit,
          const bit::Report& cbit) {
  if (!s_initialised) {
    return;
  }

  processCommand(r48, r24, r12);
  processBridge(r48, r24, r12);
  publishSnapshot(r48, r24, r12, mode, pbit, cbit);
}

}  /* namespace control_api */
}  /* namespace pdu */
