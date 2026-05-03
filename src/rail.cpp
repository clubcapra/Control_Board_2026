/* =============================================================================
 *  rail.cpp - per-rail controller implementation.
 * =============================================================================
 */
#include "rail.h"
#include "avionics_config.h"

#include <Arduino.h>
#include <math.h>

#include "console.h"

namespace pdu {
namespace rail {

/* ---------------------------------------------------------------------------
 *  Internal helpers
 * ---------------------------------------------------------------------------*/
namespace {

/** Number of consecutive I2C errors tolerated before a rail is considered
 *  absent.  An LM5066H1 that survived initial probing should never glitch
 *  more than a handful of times in a row; a longer streak is a hard fault. */
constexpr uint8_t kMaxBusErrorsInARow = 5U;

constexpr uint32_t kFaultLatchThreshold = 5U;  /* trips before latch */
constexpr uint32_t kBlackBoxRefreshPeriodMs = 10000UL;
/* The LM5066H1 only retries after TIMER discharges below about 0.3 V.
 * Do not keep power-cycling it quickly, or the STM32 can prevent TIMER from
 * ever reaching the restart threshold.                                      */
constexpr uint32_t kOnRecoveryRetryPeriodMs = 15000UL;
constexpr uint32_t kOperationRestartWaitMs = 30000UL;
constexpr uint32_t kPostOnObservePeriodMs = 60000UL;
constexpr uint8_t kStartupWatchdogMax = 0xFU;
constexpr uint8_t kConfigWriteRetries = 3U;

/** [REQ-PROT-005] Decay 1:1 with sample period: time spent below threshold
 *  reduces the accumulator at the same rate it is increased, preventing
 *  oscillating loads from masquerading as continuous high current.        */
inline uint32_t decay(uint32_t accum_ms, uint32_t dt_ms) {
  return (accum_ms > dt_ms) ? (accum_ms - dt_ms) : 0U;
}

inline uint32_t accumulate(uint32_t accum_ms, uint32_t dt_ms,
                           uint32_t cap_ms) {
  const uint32_t next = accum_ms + dt_ms;
  return (next > cap_ms) ? cap_ms : next;
}

}  // namespace

/* ===========================================================================
 *  Controller implementation
 * =========================================================================*/
Controller::Controller(const RailConfig& cfg)
    : cfg_(cfg),
      device_(cfg.i2c_address),
      state_(State::kBoot),
      present_(false),
      fault_count_(0U),
      last_tick_ms_(0U),
      accum_3min_ms_(0U),
      accum_1min_ms_(0U),
      last_vin_v_(0.0),
      last_vout_v_(0.0),
      last_vaux_v_(0.0),
      last_iin_a_(0.0),
      last_pin_w_(0.0),
      peak_valid_(false),
      peak_vin_v_(0.0),
      peak_iin_a_(0.0),
      peak_pin_w_(0.0),
      last_die_temp_c_(NAN),
      last_ntc_c_(NAN),
      last_status_word_(0U),
      last_diag_word_(0U),
      last_status_mfr_specific2_(0U),
      last_operation_raw_(0U),
      last_status_input_(0U),
      last_status_cml_(0U),
      last_status_mfr_specific_(0U),
      last_wd_plb_timer_(0U),
      bb_valid_(false),
      bb_config_(0U),
      bb_timer_(0U),
      bb_ram_len_(0U),
      bb_eeprom_len_(0U),
      bb_ram_{},
      bb_eeprom_{},
      last_bb_refresh_ms_(0U),
      last_on_recovery_ms_(0U),
      pending_on_recovery_ms_(0U),
      post_on_observe_ms_(0U),
      pending_on_recovery_(false),
      post_on_observe_(false),
      desired_on_(false) {}

Controller::Controller(const RailConfig& cfg, LM5066H1Bus& bus)
    : cfg_(cfg),
      device_(cfg.i2c_address, bus),
      state_(State::kBoot),
      present_(false),
      fault_count_(0U),
      last_tick_ms_(0U),
      accum_3min_ms_(0U),
      accum_1min_ms_(0U),
      last_vin_v_(0.0),
      last_vout_v_(0.0),
      last_vaux_v_(0.0),
      last_iin_a_(0.0),
      last_pin_w_(0.0),
      peak_valid_(false),
      peak_vin_v_(0.0),
      peak_iin_a_(0.0),
      peak_pin_w_(0.0),
      last_die_temp_c_(NAN),
      last_ntc_c_(NAN),
      last_status_word_(0U),
      last_diag_word_(0U),
      last_status_mfr_specific2_(0U),
      last_operation_raw_(0U),
      last_status_input_(0U),
      last_status_cml_(0U),
      last_status_mfr_specific_(0U),
      last_wd_plb_timer_(0U),
      bb_valid_(false),
      bb_config_(0U),
      bb_timer_(0U),
      bb_ram_len_(0U),
      bb_eeprom_len_(0U),
      bb_ram_{},
      bb_eeprom_{},
      last_bb_refresh_ms_(0U),
      last_on_recovery_ms_(0U),
      pending_on_recovery_ms_(0U),
      post_on_observe_ms_(0U),
      pending_on_recovery_(false),
      post_on_observe_(false),
      desired_on_(false) {}

Status Controller::init() {
  /* The selected bus is initialised by main(); just attach to it.           */
  device_.setSenseResistorMilliOhms(cfg_.rsns_mohm);

  /* Configure the NTC parameters used by readNtcTemperatureC().             */
  LM5066H1::NtcConfig ntc;
  ntc.pullupOhms    = cfg::kNtcPullup_Ohms;
  ntc.nominalOhms   = cfg::kNtcNominal_Ohms;
  ntc.nominalTempC  = cfg::kNtcNominalTemp_C;
  ntc.beta          = cfg::kNtcBetaK;
  ntc.supplyVolts   = cfg::kNtcSupply_V;
  device_.setNtcConfig(ntc);

  if (cfg_.has_pgood) {
    /* Power-good GPIO is an input from the LM5066H1 PG pin.                 */
    pinMode(cfg_.pgood_pin, INPUT_PULLUP);
  }

  if (!device_.beginAttached(cfg_.smbus_clock_hz)) {
    state_   = State::kAbsent;
    present_ = false;
    return Status::kNotPresent;
  }
  present_ = true;

  const Status s = configureDevice();
  if (s != Status::kOk) {
    state_ = State::kTripped;
    return s;
  }

  state_        = State::kReady;
  last_tick_ms_ = millis();
  (void)refreshBlackBoxMemory();
  return Status::kOk;
}

Status Controller::configureDevice() {
  /* Disable write-protect (legal because the firmware is the only writer). */
  if (!device_.setWriteProtect(0x00U)) {
    return Status::kBusError;
  }

  /* Clear any stale latched faults FIRST.  The chip may have booted with a
   * short NVM watchdog timer and fired an internal watchdog fault before we
   * could disable it; do the wipe early so downstream register writes aren't
   * racing against latched state.                                           */
  (void)device_.clearFaults();

  /* ADC config 2: keep 1x full-scale (more headroom on Iin coefficients).   */
  LM5066H1::AdcConfig2Bits adc2 = {};
  device_.readAdcConfig2(adc2);
  adc2.fullScale = false;
  device_.setAdcConfig2(adc2);
  device_.setAdcFullScale2x(false);

  /* DEVICE_SETUP1: select PMBus VCL (currentLimitConfig=1), CB low-side.
   * retrySetting = 7 -> infinite automatic retry on any latched fault
   * (VIN OV/UV, OC, OT, CB, FET).  The chip re-engages the GATE on its own
   * once the underlying condition has cleared; the STM32 never writes
   * OPERATION.                                                              */
  LM5066H1::DeviceSetup1Bits ds1 = {};
  device_.readDeviceSetup1(ds1);
  ds1.retrySetting       = 0x7U;   /* infinite retries                       */
  ds1.currentLimitConfig = true;   /* use DS2 PMBus VCL                       */
  ds1.cbClRatio          = false;  /* low-side CB ratio set, not 4.0x         */
  ds1.permanentWriteDisable = false;
  device_.setDeviceSetup1(ds1);

  /* DEVICE_SETUP4: disable immediateRetry so the chip waits the retry-delay
   * timer between attempts.  Prevents hot-short pounding.                    */
  LM5066H1::DeviceSetup4Bits ds4 = {};
  device_.readDeviceSetup4(ds4);
  ds4.immediateRetry = false;
  ds4.regulationTimerDischarge = true;  /* fast TIMER discharge for retry */
  device_.setDeviceSetup4(ds4);

  /* DELAY_CONFIG: retryDelay nibble = 5 -> ~285 ms between retry attempts.
   * Long enough to avoid hammering a real short, short enough for VIN UV/OV
   * transients to feel instant to the operator.                              */
  LM5066H1::DelayConfigBits dly = {};
  device_.readDelayConfig(dly);
  dly.retryDelay = 0x5U;
  device_.setDelayConfig(dly);

  /* DEVICE_SETUP2: VCL code + CB ratio + transient blanking off.            */
  LM5066H1::DeviceSetup2Bits ds2 = {};
  device_.readDeviceSetup2(ds2);
  ds2.currentLimitSetting2 = cfg_.vcl_code & 0x07U;
  /* CB ratio mapping (DS1[3]=0 in our config):
   *   ds2.cbClRatio2 = 0  -> 2.0 x  (with vinTranEnable typical)
   *   ds2.cbClRatio2 = 1  -> 1.2 x
   *   ds2.cbClRatio2 = 3  -> 3.0 x                                          */
  if (fabs(cfg_.cb_ratio - 1.2) < 0.05) {
    ds2.cbClRatio2 = 0x1U;
  } else if (fabs(cfg_.cb_ratio - 2.0) < 0.05) {
    ds2.cbClRatio2 = 0x0U;
  } else if (fabs(cfg_.cb_ratio - 3.0) < 0.05) {
    ds2.cbClRatio2 = 0x3U;
  } else {
    ds2.cbClRatio2 = 0x1U;  /* safe default = 1.2 x */
  }
  ds2.vinTranEnable  = false;  /* instant CB action, no VIN-transient blanking */
  ds2.fastRecoveryCb = false;
  device_.setDeviceSetup2(ds2);

  /* Mirror the VCL into the library so direct-format current scaling is
   * accurate when reading IIN / PIN / IOUT.                                 */
  static constexpr double kVclMv[8] = {
      50.0, 10.0, 12.5, 15.0, 17.5, 20.0, 22.5, 25.0,
  };
  device_.setCurrentLimitMilliVolts(kVclMv[cfg_.vcl_code & 0x07U]);

  /* DEVICE_SETUP3: BREAKER mode for ALL rails (currentLimitingDisable=1) -
   * GATEs turn off as soon as any OC timer expires, no current regulation
   * loop.  OC blanking thresholds are taken from the rail descriptor so
   * each rail can pick its own layered protection profile.                  */
  LM5066H1::DeviceSetup3Bits ds3 = {};
  device_.readDeviceSetup3(ds3);
  ds3.currentLimitingDisable = true;                            /* breaker  */
  ds3.ocBlanking1Threshold   = cfg_.ocb1_threshold & 0x03U;     /* OC1 lvl  */
  ds3.ocBlanking2Threshold   = cfg_.ocb2_threshold & 0x03U;     /* OC2 lvl  */
  ds3.foldbackCurrentLimit   = 0U;
  ds3.powerLimitProfile      = false;
  device_.setDeviceSetup3(ds3);

  /* OC_BLANKING_TIMERS register: per-rail blanking durations.  Setting a
   * timer to 0 effectively disables that overcurrent layer (the
   * comparator still fires but the GATEs are not commanded off).            */
  LM5066H1::OcBlankingTimersBits oc = {};
  oc.blanking1 = cfg_.ocb1_timer & 0x0FU;
  oc.blanking2 = cfg_.ocb2_timer & 0x0FU;
  device_.setOcBlankingTimers(oc);

  /* Voltage warn limits.                                                    */
  device_.setVinOvWarnLimit(cfg_.vin_ov_v);
  device_.setVinUvWarnLimit(cfg_.vin_uv_v);

  /* Temperature limits (LM5066 internal diode).                             */
  device_.setOtWarnLimit(cfg::kOtWarn_C);
  device_.setOtFaultLimit(cfg::kOtFault_C);

  /* IIN warn limit: place a sane warning slightly below the HW trip.        */
  const double iin_warn_a = cfg_.hw_trip_a * 0.85;
  device_.setIinOcWarnLimitMfr(iin_warn_a);
  device_.setIinOcWarnLimitStd(iin_warn_a);

  /* Alert mask: enable the most useful alerts; the SMBA pin can be wired
   * later if the schematic is updated.                                      */
  LM5066H1::AlertMaskBits alert = {};
  alert.vinUndervoltageFault = false;  /* not masked: we want SMBA on UV    */
  alert.vinOvervoltageFault  = false;
  alert.iinPfetFault         = false;
  alert.overtempFault        = false;
  alert.cmlFault             = false;
  alert.circuitBreakerFault  = false;
  alert.scpFault             = false;
  alert.fetFailFault         = false;
  alert.powerGood            = false;
  device_.setAlertMask(alert);

  /* LM5066H1 black box: preserve fault/alert/FET-off evidence in the device
   * non-volatile memory.  The STM32 reads it; it is only erased by command. */
  LM5066H1::BbConfigBits bb = {};
  bb.fetOffWr = true;
  bb.fltWr = true;
  bb.alertWr = true;
  bb.bbTick = 0U;  /* 10 us tick, highest timing resolution */
  device_.setBbConfig(bb);

  /* Gate-mask: allow all hardware-level faults to pull the GATE off.        */
  LM5066H1::GateMaskBits gate = {};
  gate.scpFault              = false;
  gate.fetFail               = false;
  gate.vinUvFault            = false;
  gate.vinOvFault            = false;
  gate.iinPfetFault          = false;
  gate.overtempFault         = false;
  gate.watchdogFault         = false;
  gate.circuitBreakerFault   = false;
  device_.setGateMask(gate);

  /* WD_PLB_TIMER: the LM5066H1 startup watchdog is NOT disable-by-zero.
   * Per datasheet, watchdog nibble 0x0 is the shortest timeout (~9.5 ms) and
   * 0xF is the longest (~9.5 s).  Give the output startup/retry sequence the
   * maximum window so the chip does not shut the MOSFET off immediately.     */
  bool watchdog_config_ok = false;
  for (uint8_t attempt = 0U; attempt < kConfigWriteRetries; ++attempt) {
    LM5066H1::WdPlbTimerBits wd = {};
    (void)device_.readWdPlbTimer(wd);
    wd.watchdogTimer = kStartupWatchdogMax;
    if (device_.setWdPlbTimer(wd)) {
      LM5066H1::WdPlbTimerBits wd_verify = {};
      if (device_.readWdPlbTimer(wd_verify) &&
          wd_verify.watchdogTimer == kStartupWatchdogMax) {
        watchdog_config_ok = true;
        break;
      }
    }
    delay(2);
  }
  if (!watchdog_config_ok) {
    Serial.print(F("[CFG "));
    Serial.print(railToString(cfg_.id));
    Serial.println(F("] WD_PLB_TIMER watchdog write/readback failed"));
    return Status::kBusError;
  }

  /* Final clear of any stale latched flags.                                 */
  device_.clearFaults();

  /* --------------------------------------------------------------------- */
  /*  Diagnostic readback: prove to the console that retry/delay/watchdog
   *  actually landed on this specific device.  Invaluable when two parts
   *  on the same board boot with different NVM.                             */
  /* --------------------------------------------------------------------- */
  LM5066H1::DeviceSetup1Bits ds1_rb = {};
  LM5066H1::DeviceSetup4Bits ds4_rb = {};
  LM5066H1::DelayConfigBits  dly_rb = {};
  LM5066H1::WdPlbTimerBits   wd_rb  = {};
  device_.readDeviceSetup1(ds1_rb);
  device_.readDeviceSetup4(ds4_rb);
  device_.readDelayConfig(dly_rb);
  device_.readWdPlbTimer(wd_rb);
  Serial.print(F("[CFG "));
  Serial.print(railToString(cfg_.id));
  Serial.print(F("] DS1.retry="));
  Serial.print(ds1_rb.retrySetting);
  Serial.print(F(" DS4.immRetry="));
  Serial.print(ds4_rb.immediateRetry ? 1 : 0);
  Serial.print(F(" DS4.fastTmrDis="));
  Serial.print(ds4_rb.regulationTimerDischarge ? 1 : 0);
  Serial.print(F(" retryDelay="));
  Serial.print(dly_rb.retryDelay);
  Serial.print(F(" WD="));
  Serial.println(wd_rb.watchdogTimer);

  return Status::kOk;
}

Status Controller::enable() {
  if (state_ == State::kAbsent || state_ == State::kLatched) {
    return Status::kFault;
  }
  /* Commanded ON.  Recovery retries below handle long OFF waits without
   * blocking the watchdog; an API/boot ON command itself is immediate.       */
  desired_on_ = true;
  (void)device_.clearFaults();
  if (!device_.setOperation(true)) {
    return Status::kBusError;
  }
  pending_on_recovery_ = false;
  post_on_observe_ = true;
  post_on_observe_ms_ = millis();
  last_on_recovery_ms_ = millis();
  state_         = State::kRunning;
  accum_3min_ms_ = 0U;
  accum_1min_ms_ = 0U;
  return Status::kOk;
}

Status Controller::disable() {
  /* One-shot OFF.  RG API disable / E-stop map to exactly one
   * OPERATION=0x00 write; no periodic re-assertion anywhere else.          */
  desired_on_ = false;
  if (!device_.setOperation(false)) {
    return Status::kBusError;
  }
  pending_on_recovery_ = false;
  post_on_observe_ = false;
  if (state_ == State::kRunning || state_ == State::kWarning) {
    state_ = State::kReady;
  }
  return Status::kOk;
}

Status Controller::clearLatch() {
  if (state_ != State::kLatched) {
    return Status::kOk;
  }
  device_.clearFaults();
  fault_count_   = 0U;
  accum_3min_ms_ = 0U;
  accum_1min_ms_ = 0U;
  state_         = State::kReady;
  return Status::kOk;
}

void Controller::tripFromProtection(const char* cause, fault_log::Code code) {
  if (!cfg::kHotswapApiOnly) {
    device_.setOperation(false);
  }
  (void)refreshBlackBoxMemory();
  ++fault_count_;
  state_ = (fault_count_ < kFaultLatchThreshold) ? State::kTripped
                                                 : State::kLatched;
  Serial.print(F("[PROT TRIP "));
  Serial.print(railToString(cfg_.id));
  Serial.print(F("] "));
  Serial.print(cause);
  Serial.print(F("  fault_count="));
  Serial.println(fault_count_);

  /* A rail protection event is contained at the rail boundary: the LM5066H1
   * GATE is commanded OFF and the rail state moves to TRIPPED/LATCHED.  The
   * MCU must stay alive so SWO telemetry, persistent fault history, black-box
   * reads, and the external I2C API remain available for root-cause analysis.
   * Full MCU resets are reserved for watchdog expiry or an explicit host reset. */
  fault_log::record(code, cfg_.id, last_status_word_, last_diag_word_);
}

Status Controller::refreshBlackBoxMemory() {
  if (!present_) {
    return Status::kNotPresent;
  }

  uint8_t config = 0U;
  uint8_t timer = 0U;
  uint8_t ram[kLm5066BlackBoxBytes] = {};
  uint8_t eeprom[kLm5066BlackBoxBytes] = {};
  size_t ram_len = 0U;
  size_t eeprom_len = 0U;

  bool ok = device_.readBbConfig(config);
  ok = device_.readBbTimer(timer) && ok;
  (void)device_.fetchBbEeprom();
  ok = device_.readBbRam(ram, sizeof(ram), ram_len) && ok;
  ok = device_.readBbEeprom(eeprom, sizeof(eeprom), eeprom_len) && ok;

  bb_config_ = config;
  bb_timer_ = timer;
  bb_ram_len_ = static_cast<uint8_t>(ram_len);
  bb_eeprom_len_ = static_cast<uint8_t>(eeprom_len);
  for (size_t i = 0U; i < kLm5066BlackBoxBytes; ++i) {
    bb_ram_[i] = ram[i];
    bb_eeprom_[i] = eeprom[i];
  }
  bb_valid_ = ok;
  last_bb_refresh_ms_ = millis();
  return ok ? Status::kOk : Status::kBusError;
}

void Controller::evaluateTimeWindowed(double iin_a, uint32_t dt_ms) {
  /* Only the 48 V rail uses time-windowed protection.                       */
  if (cfg_.id != Rail::k48V) {
    return;
  }

  /* 80 A / 3 min accumulator.                                               */
  if (iin_a >= cfg_.sw_warn_3min_a) {
    accum_3min_ms_ = accumulate(accum_3min_ms_, dt_ms, cfg_.sw_window_3min_ms);
  } else {
    accum_3min_ms_ = decay(accum_3min_ms_, dt_ms);
  }

  /* 100 A / 1 min accumulator.                                              */
  if (iin_a >= cfg_.sw_warn_1min_a) {
    accum_1min_ms_ = accumulate(accum_1min_ms_, dt_ms, cfg_.sw_window_1min_ms);
  } else {
    accum_1min_ms_ = decay(accum_1min_ms_, dt_ms);
  }

  if (accum_1min_ms_ >= cfg_.sw_window_1min_ms) {
    tripFromProtection("48V >100A for 1 min", fault_log::Code::kOver100AOneMinute);
    return;
  }
  if (accum_3min_ms_ >= cfg_.sw_window_3min_ms) {
    tripFromProtection("48V >80A for 3 min", fault_log::Code::kOver80AThreeMinutes);
    return;
  }
}

bool Controller::ntcCelsiusFromVaux(double vaux_v, double& celsius) const {
  /* Divider: 3.3V -- 10k -- VAUX -- 10k NTC -- GND.
   * Rntc = Rpullup * Vaux / (Vsupply - Vaux).                               */
  if (vaux_v <= 0.0 || vaux_v >= cfg::kNtcSupply_V) {
    celsius = NAN;
    return false;
  }
  const double r_ntc = (cfg::kNtcPullup_Ohms * vaux_v) /
                       (cfg::kNtcSupply_V - vaux_v);
  if (r_ntc <= 0.0 ||
      cfg::kNtcNominal_Ohms <= 0.0 ||
      cfg::kNtcBetaK <= 0.0) {
    celsius = NAN;
    return false;
  }

  const double t0_k = cfg::kNtcNominalTemp_C + 273.15;
  const double inv_t = (1.0 / t0_k) +
                       (log(r_ntc / cfg::kNtcNominal_Ohms) / cfg::kNtcBetaK);
  if (inv_t <= 0.0) {
    celsius = NAN;
    return false;
  }

  celsius = (1.0 / inv_t) - 273.15;
  return true;
}

void Controller::tick() {
  if (state_ == State::kAbsent || state_ == State::kBoot) {
    return;
  }

  const uint32_t now    = millis();
  uint32_t       dt_ms  = now - last_tick_ms_;
  last_tick_ms_         = now;
  if (dt_ms > 1000UL) {
    /* Defensive cap: if we were preempted for too long, do not credit the
     * accumulators with that whole interval - it would unfairly penalise
     * the 48 V rail.                                                       */
    dt_ms = 1000UL;
  }

  if ((now - last_bb_refresh_ms_) >= kBlackBoxRefreshPeriodMs) {
    (void)refreshBlackBoxMemory();
  }

  /* ------------------------------------------------------------------ */
  /*  Sample telemetry.  Each I2C read is independent so a transient bus
   *  glitch on one register does not corrupt the others.                */
  /* ------------------------------------------------------------------ */
  double   vin = 0.0, vout = 0.0, vaux = 0.0, iin = 0.0, pin = 0.0;
  uint16_t status_word = 0U, diag_word = 0U, status_mfr2 = 0U;
  uint8_t  wd_plb_timer = 0U;
  double   die_temp_c = NAN;

  bool ok_iin   = device_.readIin(iin);
  bool ok_vin   = device_.readVin(vin);
  bool ok_vout  = device_.readVout(vout);
  bool ok_vaux  = device_.readVaux(vaux);
  bool ok_pin   = device_.readPin(pin);
  bool ok_temp  = device_.readTemperatureC(die_temp_c);
  bool ok_sw    = device_.readStatusWord(status_word);
  bool ok_diag  = device_.readDiagnosticWord(diag_word);
  bool ok_smfr2 = device_.readStatusMfrSpecific2(status_mfr2);
  bool ok_wdplb = device_.readWdPlbTimer(wd_plb_timer);
  uint8_t status_input = 0U;
  uint8_t status_cml   = 0U;
  uint8_t status_mfr   = 0U;
  bool ok_sin   = device_.readStatusInput(status_input);
  bool ok_scml  = device_.readStatusCml(status_cml);
  bool ok_smfr  = device_.readStatusMfrSpecific(status_mfr);
  double ntc_c  = NAN;
  if (ok_vaux) {
    ntcCelsiusFromVaux(vaux, ntc_c);
  }

  if (ok_iin)   { last_iin_a_       = iin; }
  if (ok_vin)   { last_vin_v_       = vin; }
  if (ok_vout)  { last_vout_v_      = vout; }
  if (ok_vaux)  { last_vaux_v_      = vaux; }
  if (ok_pin)   { last_pin_w_       = pin; }
  if (ok_temp)  { last_die_temp_c_  = die_temp_c; }
  if (ok_sw)    { last_status_word_ = status_word; }
  if (ok_diag)  { last_diag_word_   = diag_word; }
  if (ok_smfr2) { last_status_mfr_specific2_ = status_mfr2; }
  if (ok_sin)   { last_status_input_         = status_input; }
  if (ok_scml)  { last_status_cml_           = status_cml; }
  if (ok_smfr)  { last_status_mfr_specific_  = status_mfr; }
  if (ok_wdplb) { last_wd_plb_timer_ = wd_plb_timer; }
  last_ntc_c_                       = ntc_c;

  if (ok_vin && ok_iin && ok_pin &&
      (!peak_valid_ || pin > peak_pin_w_)) {
    peak_valid_ = true;
    peak_vin_v_ = vin;
    peak_iin_a_ = iin;
    peak_pin_w_ = pin;
  }

  if (!(ok_iin && ok_sw)) {
    /* Critical reads failed - count and bail.  Bus may be glitching.        */
    static uint8_t bus_error_streak[kRailCount] = {0U, 0U, 0U};
    const uint8_t  rail_idx = static_cast<uint8_t>(cfg_.id);
    if (rail_idx < kRailCount) {
      if (++bus_error_streak[rail_idx] >= kMaxBusErrorsInARow) {
        present_ = false;
        state_   = State::kAbsent;
      }
    }
    return;
  }

  /* ------------------------------------------------------------------ */
  /*  State mirroring + desired-ON recovery.                              */
  /*                                                                     */
  /*  If RG/boot wants ON but the LM5066H1 is still reporting             */
  /*  device-off, keep clearing latched status and pulsing OPERATION      */
  /*  0x00 -> 0x80.  This provides an actual unlimited retry loop for     */
  /*  latched watchdog/status conditions while still respecting RG OFF.   */
  /*  During the 30 s cooldown we intentionally write OPERATION=0x00, so  */
  /*  the desired state is tracked separately from the raw OP register.   */
  /* ------------------------------------------------------------------ */
  uint8_t op_raw = 0U;
  if (device_.readOperationRaw(op_raw)) {
    last_operation_raw_ = op_raw;
    const bool cmd_on = (op_raw & 0x80U) != 0U;
    if (cmd_on && state_ == State::kReady) {
      state_ = State::kRunning;
    }
    if (!cmd_on && !desired_on_ && state_ == State::kRunning) {
      state_ = State::kReady;
    }
    const bool chip_reports_off =
        (status_word & (1U << 6)) != 0U ||
        (diag_word   & (1U << 6)) != 0U;
    if (pending_on_recovery_) {
      if ((now - pending_on_recovery_ms_) >= kOperationRestartWaitMs) {
        Serial.print(F("[HOTSWAP "));
        Serial.print(railToString(cfg_.id));
        Serial.println(F("] 30s OFF wait done - CLEAR_FAULTS, command ON"));
        const bool ok_clear = device_.clearFaults();
        const bool ok_on = device_.setOperation(true);
        uint8_t op_after = 0U;
        uint16_t sw_after = 0U;
        uint16_t diag_after = 0U;
        const bool ok_op_after = device_.readOperationRaw(op_after);
        const bool ok_sw_after = device_.readStatusWord(sw_after);
        const bool ok_diag_after = device_.readDiagnosticWord(diag_after);
        Serial.print(F("[HOTSWAP "));
        Serial.print(railToString(cfg_.id));
        Serial.print(F("] retry result clr="));
        Serial.print(ok_clear ? 1 : 0);
        Serial.print(F(" on="));
        Serial.print(ok_on ? 1 : 0);
        Serial.print(F(" OP=0x"));
        Serial.print(ok_op_after ? op_after : 0U, HEX);
        Serial.print(F(" SW=0x"));
        Serial.print(ok_sw_after ? sw_after : 0U, HEX);
        Serial.print(F(" DIAG=0x"));
        Serial.println(ok_diag_after ? diag_after : 0U, HEX);
        last_operation_raw_ = ok_op_after ? op_after : 0x80U;
        last_on_recovery_ms_ = now;
        pending_on_recovery_ = false;
        post_on_observe_ = true;
        post_on_observe_ms_ = now;
      }
    } else if (post_on_observe_ &&
               (now - post_on_observe_ms_) < kPostOnObservePeriodMs) {
      /* Let the LM5066H1 finish its startup/retry behavior.  If it flashes on
       * then drops back off, telemetry will show that without the STM32
       * immediately starting another forced OFF wait.                       */
    } else if (desired_on_ && chip_reports_off &&
        (now - last_on_recovery_ms_) >= kOnRecoveryRetryPeriodMs) {
      post_on_observe_ = false;
      last_on_recovery_ms_ = now;
      pending_on_recovery_ms_ = now;
      pending_on_recovery_ = true;
      Serial.print(F("[HOTSWAP "));
      Serial.print(railToString(cfg_.id));
      Serial.println(F("] desired ON but FET=OFF - CLEAR_FAULTS, MFR_POWER_CYCLE, command OFF, wait 30s"));
      const bool ok_clear1 = device_.clearFaults();
      const bool ok_pcycle = device_.powerCycle();
      const bool ok_off = device_.setOperation(false);
      uint8_t op_after = 0U;
      const bool ok_op_after = device_.readOperationRaw(op_after);
      Serial.print(F("[HOTSWAP "));
      Serial.print(railToString(cfg_.id));
      Serial.print(F("] retry start clr="));
      Serial.print(ok_clear1 ? 1 : 0);
      Serial.print(F(" pcycle="));
      Serial.print(ok_pcycle ? 1 : 0);
      Serial.print(F(" off="));
      Serial.print(ok_off ? 1 : 0);
      Serial.print(F(" OP=0x"));
      Serial.println(ok_op_after ? op_after : 0U, HEX);
      last_operation_raw_ = ok_op_after ? op_after : 0U;
    }
    if (!desired_on_ && !pending_on_recovery_) {
      last_on_recovery_ms_ = 0U;
      post_on_observe_ = false;
    }
  }
  /* suppress unused-variable warnings for status decoders kept for telemetry */
  (void)status_mfr2;
  (void)status_input; (void)status_cml; (void)status_mfr;
  (void)dt_ms; (void)ntc_c;
}

void Controller::buildTelemetry(RailTelemetry& out) const {
  out.rail        = cfg_.id;
  out.present     = present_;
  out.output_on   = (state_ == State::kRunning) || (state_ == State::kWarning);
  out.pgood_pin   = cfg_.has_pgood ? (digitalRead(cfg_.pgood_pin) == HIGH) : false;
  out.vin_v       = last_vin_v_;
  out.vout_v      = last_vout_v_;
  out.vaux_v      = last_vaux_v_;
  out.iin_a       = last_iin_a_;
  out.pin_w       = last_pin_w_;
  out.peak_valid  = peak_valid_;
  out.peak_vin_v  = peak_vin_v_;
  out.peak_iin_a  = peak_iin_a_;
  out.peak_pin_w  = peak_pin_w_;
  out.die_temp_c  = last_die_temp_c_;
  out.ntc_temp_c  = last_ntc_c_;
  out.status_word = last_status_word_;
  out.diag_word   = last_diag_word_;
  out.status_mfr_specific2 = last_status_mfr_specific2_;
  out.operation_raw        = last_operation_raw_;
  out.status_input         = last_status_input_;
  out.status_cml           = last_status_cml_;
  out.status_mfr_specific  = last_status_mfr_specific_;
  out.wd_plb_timer = last_wd_plb_timer_;
  out.fault_count = fault_count_;
  out.bb_valid = bb_valid_;
  out.bb_config = bb_config_;
  out.bb_timer = bb_timer_;
  out.bb_ram_len = bb_ram_len_;
  out.bb_eeprom_len = bb_eeprom_len_;
  for (size_t i = 0U; i < kLm5066BlackBoxBytes; ++i) {
    out.bb_ram[i] = bb_ram_[i];
    out.bb_eeprom[i] = bb_eeprom_[i];
  }
}

}  /* namespace rail */
}  /* namespace pdu */
