/* =============================================================================
 *  main.cpp - Power Distribution Unit (PDU) supervisor
 * -----------------------------------------------------------------------------
 *  Top-level firmware for the STM32F103C8T6 ("Blue Pill") that orchestrates
 *  three Texas Instruments LM5066H1 hot-swap controllers (48 V / 24 V / 12 V).
 *
 *  Style notes:
 *      - Single super-loop architecture (no RTOS, no malloc) - easier to
 *        certify against MISRA-C and DO-178C than a multi-threaded design.
 *      - All state is allocated statically.  No dynamic memory anywhere.
 *      - Every external interaction returns an explicit Status code; callers
 *        must check it (compiler warns when the return value is ignored).
 *      - The Independent Watchdog is started early in boot and pet only after
 *        critical work completes.  A hung boot or loop resets the MCU.
 * =============================================================================
 */
#include <Arduino.h>
#include <Wire.h>

#include "avionics_config.h"
#include "avionics_types.h"
#include "console.h"
#include "iwdg.h"
#include "leds.h"
#include "estop.h"
#include "fault_log.h"
#include "rail.h"
#include "bit.h"
#include "control_api.h"
#include "winch.h"
#include "winch_lock.h"
#include "soft_smbus.h"
#include "LM5066H1.h"

using namespace pdu;

/* ---------------------------------------------------------------------------
 *  Static rail descriptors.  Each LM5066H1 is run in pure BREAKER mode
 *  (current-limit disabled).  The 48 V rail additionally uses the three
 *  layered hardware overcurrent levels OC1 / OC2 / OC3.
 *
 *  Reminder of the LM5066H1 threshold ladders:
 *      VCL codes (DS2[5:3])  : 1=10mV  2=12.5mV  3=15mV  4=17.5mV
 *                              5=20mV  6=22.5mV  7=25mV
 *      OCB1 thresholds (DS3) : 0=1.25x 1=1.5x   2=1.75x  3=2.0x  VCL
 *      OCB2 thresholds (DS3) : 0=1.5x  1=1.75x  2=2.0x   3=2.25x VCL
 *      VCB (CB ratio)        : 1.2x, 2.0x, 3.0x, 4.0x VCL
 *
 *  Timer codes for OC_BLANKING_TIMERS (per LM5066H1 lib helper):
 *      tCBL1 codes 0..15: 0us, 19us, 95us, 475us, 712us, 0.95ms, 1.9ms,
 *                         3.8ms, 7.6ms, 9.5ms, 14.25ms, 19ms, 38ms,
 *                         57ms, 76ms, 95ms.
 *      tCBL2 codes 0..15: 0us, 38us, 57us, 95us, 190us, 285us, 380us,
 *                         570us, 760us, 950us, 1.9ms, 2.85ms, 3.8ms,
 *                         4.75ms, 9.5ms, 95ms.
 * ---------------------------------------------------------------------------*/

/* ---------- 48 V (Rsns = 100 uOhm) --------------------------------------- *
 *  VCL=12.5mV (code 2) -> 125 A reference                                   *
 *  OC1 = 1.25 x VCL  = 15.625 mV -> 156 A with timer1 = 95 ms                *
 *  OC2 = 1.5  x VCL  = 18.750 mV -> 188 A with timer2 = 95 us                *
 *  OC3 = VCB = 1.2 x VCL = 15.0 mV -> 150 A instant hardware breaker         *
 *  Software enforces 80 A / 3 min, 100 A / 1 min and redundant 150 A trip.   *
 * --------------------------------------------------------------------------*/
static const rail::RailConfig kCfg48V = {
    /* id                   */ Rail::k48V,
    /* i2c_address          */ cfg::kAddr48V,
    /* rsns_mohm            */ cfg::kRsns48V_mOhm,
    /* smbus_clock_hz       */ cfg::kSmbus48Clock_Hz,
    /* vcl_code             */ 2U,                  /* VCL = 12.5 mV        */
    /* ocb1_threshold       */ 0U,                  /* OC1 = 1.25 x VCL     */
    /* ocb1_timer           */ 0x0FU,               /* tCBL1 = 95 ms        */
    /* ocb2_threshold       */ 0U,                  /* OC2 = 1.5 x VCL      */
    /* ocb2_timer           */ 0x03U,               /* tCBL2 = 95 us        */
    /* cb_ratio             */ 1.2,                 /* OC3 = 1.2 x VCL      */
    /* hw_trip_a            */ cfg::kI48VPeakInstant_A,
    /* sw_warn_3min_a       */ cfg::kI48VWarn3min_A,
    /* sw_warn_1min_a       */ cfg::kI48VWarn1min_A,
    /* sw_window_3min_ms    */ cfg::kI48VWarn3minWindow_ms,
    /* sw_window_1min_ms    */ cfg::kI48VWarn1minWindow_ms,
    /* sw_instant_a         */ cfg::kI48VPeakInstant_A, /* SW backs up 150 A */
    /* pgood_pin            */ cfg::kPin_PB14_Unused,
    /* has_pgood            */ false,
    /* vin_ov_v             */ cfg::kVin48V_OV_V,
    /* vin_uv_v             */ cfg::kVin48V_UV_V,
};

/* ---------- 24 V (Rsns = 1.3 mΩ) ---------------------------------------- *
 *  VCL=22.5 mV / CB=1.2x -> trip @ 27 mV / 1.3 mΩ ≈ 20.8 A (instant cut)    *
 *  OC1 / OC2 disabled (timers = 0).                                         *
 * --------------------------------------------------------------------------*/
static const rail::RailConfig kCfg24V = {
    Rail::k24V,
    cfg::kAddr24V,
    cfg::kRsns24V_mOhm,
    cfg::kI2c1HotswapClock_Hz,
    6U,                /* VCL = 22.5 mV                                     */
    0U, 0U, 0U, 0U,    /* OC1 / OC2 disabled                                 */
    1.2,               /* VCB = 1.2 x VCL                                    */
    cfg::kI24VMax_A,
    0.0, 0.0, 0U, 0U,  /* no time-window protection                          */
    cfg::kI24VMax_A,   /* SW instant redundancy                              */
    cfg::kPin_PG_24V,
    true,
    cfg::kVin24V_OV_V,
    cfg::kVin24V_UV_V,
};

/* ---------- 12 V (Rsns = 2.0 mΩ) ---------------------------------------- *
 *  VCL=17.5 mV / CB=1.2x -> trip @ 21 mV / 2.0 mΩ ≈ 10.5 A (instant cut)    *
 *  OC1 / OC2 disabled (timers = 0).                                         *
 * --------------------------------------------------------------------------*/
static const rail::RailConfig kCfg12V = {
    Rail::k12V,
    cfg::kAddr12V,
    cfg::kRsns12V_mOhm,
    cfg::kI2c1HotswapClock_Hz,
    4U,                /* VCL = 17.5 mV                                     */
    0U, 0U, 0U, 0U,    /* OC1 / OC2 disabled                                 */
    1.2,               /* VCB = 1.2 x VCL                                    */
    cfg::kI12VMax_A,
    0.0, 0.0, 0U, 0U,
    cfg::kI12VMax_A,
    cfg::kPin_PG_12V,
    true,
    cfg::kVin12V_OV_V,
    cfg::kVin12V_UV_V,
};

/* ---------------------------------------------------------------------------
 *  Statically-allocated rail controllers.  48 V uses its own PA2/PA3 software
 *  SMBus; 24 V and 12 V share the hardware I2C1 bus on PB6/PB7.
 * ---------------------------------------------------------------------------*/
static soft_smbus::Master s_smbus48(cfg::kPin_SMBUS48_SDA,
                                    cfg::kPin_SMBUS48_SCL);
static rail::Controller s_rail48(kCfg48V, s_smbus48);
static rail::Controller s_rail24(kCfg24V);
static rail::Controller s_rail12(kCfg12V);

/* ---------------------------------------------------------------------------
 *  Supervisor-wide state.
 * ---------------------------------------------------------------------------*/
static SupervisorMode  s_mode             = SupervisorMode::kBoot;
static bit::Report     s_pbit_report      = {};
static bit::Report     s_cbit_report      = {};

static uint32_t        s_last_protect_ms  = 0U;
static uint32_t        s_last_telemetry_ms= 0U;
static uint32_t        s_last_cbit_ms     = 0U;
static uint32_t        s_last_heartbeat_ms= 0U;
static bool            s_boot_watchdog_reset = false;

static constexpr uint32_t kWatchdogMaxHealthyLoop_ms =
    cfg::kIwdgTimeout_ms / 2U;

/* ---------------------------------------------------------------------------
 *  Helpers
 * ---------------------------------------------------------------------------*/
static void enterMode(SupervisorMode mode) {
  if (s_mode == mode) {
    return;
  }
  s_mode = mode;
  Serial.print(F("[SUPV] mode -> "));
  Serial.println(modeToString(mode));

  switch (mode) {
    case SupervisorMode::kPbit:
      leds::setPattern(leds::Pattern::kHeartbeat);
      break;
    case SupervisorMode::kNominal:
      leds::setAll(50U);
      leds::setPattern(leds::Pattern::kSolid);
      break;
    case SupervisorMode::kDegraded:
      leds::setPattern(leds::Pattern::kHeartbeat);
      break;
    case SupervisorMode::kEStop:
      leds::setPattern(leds::Pattern::kEStop);
      break;
    case SupervisorMode::kFault:
      leds::setPattern(leds::Pattern::kFault);
      break;
    default:
      break;
  }
}

static void disableAllRails() {
  if (!cfg::kHotswapApiOnly) {
    s_rail48.disable();
    s_rail24.disable();
    s_rail12.disable();
  }
  (void)winch::sleep();
  (void)winch_lock::setAll(false);
}

static void enableAllRails() {
  if (cfg::kHotswapApiOnly) {
    Serial.println(F("[HOTSWAP] API-only: automatic rail enable skipped"));
    return;
  }

  /* Sequenced power-up: 12V first (light low-side rail), then 24V, then
   * 48V.  Each step is followed by a short settle delay so the next rail
   * sees a stable bus before its inrush.                                  */
  s_rail12.enable();
  delay(20);
  s_rail24.enable();
  delay(20);
  s_rail48.enable();
}

static void serviceWatchdogIfHealthy(uint32_t loop_started_ms) {
  const uint32_t elapsed_ms = millis() - loop_started_ms;
  if (elapsed_ms <= kWatchdogMaxHealthyLoop_ms) {
    iwdg::kick();
  } else {
    Serial.print(F("[IWDG] loop overrun "));
    Serial.print(elapsed_ms);
    Serial.println(F("ms - watchdog not reloaded"));
  }
}

/* ---------------------------------------------------------------------------
 *  Human-readable FAULT decoder for LM5066H1 status registers.
 *
 *  Only actual protection-event bits are printed.  Pure warning thresholds
 *  (VIN/IIN/PIN/OT warnings) and informational/status bits (PGOOD asserted,
 *  device-off, default config preset, init-done, retry/power-cycle recovery,
 *  averaging done, etc.) are intentionally suppressed - those tell you what
 *  the chip is *doing*, not what is wrong.
 *
 *  Bit positions taken straight from the LM5066H1 driver (lib/LM5066H1) so
 *  they match the chip's documented STATUS register layout (datasheet
 *  SNVSAQ7), not a guess.
 * ---------------------------------------------------------------------------*/
static void describeRailFaults(const RailTelemetry& tlm) {
  const char* rail = railToString(tlm.rail);
  const uint16_t sw    = tlm.status_word;
  const uint16_t diag  = tlm.diag_word;
  const uint16_t smfr2 = tlm.status_mfr_specific2;
  const uint8_t  sin   = tlm.status_input;
  const uint8_t  scml  = tlm.status_cml;
  const uint8_t  smfr  = tlm.status_mfr_specific;

  bool any = false;

  auto emit = [&](const __FlashStringHelper* msg) {
    Serial.print(F("    [FAULT] "));
    Serial.print(rail);
    Serial.print(F(": "));
    Serial.println(msg);
    any = true;
  };

  /* ---- STATUS_WORD (PMBus 0x79, 16-bit summary) -----------------------
   *  Only emit the low-byte fault bits.  High-byte summary bits just say
   *  "look at the dedicated STATUS_xxx register", which we already decode
   *  individually below, so they would only duplicate the message.        */
  if (sw & (1U <<  7)) emit(F("PMBus: device BUSY (unable to respond)"));
  if (sw & (1U <<  5)) emit(F("PMBus: VOUT overvoltage fault"));
  if (sw & (1U <<  4)) emit(F("PMBus: IOUT overcurrent fault"));
  if (sw & (1U <<  3)) emit(F("PMBus: VIN undervoltage fault (UVLO)"));
  if (sw & (1U <<  2)) emit(F("PMBus: temperature fault (over-temp)"));

  /* ---- STATUS_INPUT (PMBus 0x7C, 8-bit) -------------------------------
   *  Bits per LM5066H1 driver:
   *      7 vinOvFault, 6 vinOvWarn, 5 vinUvWarn, 4 vinUvFault,
   *      2 ocFault,    1 ocWarn,    0 inOpWarn                            */
  if (sin & (1U << 7)) emit(F("INPUT: VIN overvoltage fault (above OV threshold)"));
  if (sin & (1U << 4)) emit(F("INPUT: VIN undervoltage fault (below UVLO)"));
  if (sin & (1U << 2)) emit(F("INPUT: IIN overcurrent fault (input overcurrent)"));

  /* ---- STATUS_CML (PMBus 0x7E, 8-bit) ---------------------------------
   *  Bits per LM5066H1 driver:
   *      7 invCmd, 6 invData, 5 invPec, 4 memoryFault, 1 noneOfAbove
   *  noneOfAbove (bit 1) is informational and reads 1 every time the host
   *  issues ANY unsupported PMBus access (very common during scans), so
   *  we skip it here.                                                     */
  if (scml & (1U << 7)) emit(F("CML: invalid or unsupported PMBus command received"));
  if (scml & (1U << 6)) emit(F("CML: invalid or unsupported PMBus data received"));
  if (scml & (1U << 5)) emit(F("CML: PEC (packet error check) failed"));
  if (scml & (1U << 4)) emit(F("CML: NVM/memory fault detected"));

  /* ---- STATUS_MFR_SPECIFIC (PMBus 0x80, 8-bit) ------------------------
   *  Bits per LM5066H1 driver:
   *      7 cbFault,        6 fetFail,
   *      4 defaultsLoaded (INFO - chip on factory config),
   *      3 bbRamFull,      2 fetFaultGate2, 1 fetFaultGate1, 0 fetFaultDrain  */
  if (smfr & (1U << 7)) emit(F("MFR: circuit breaker (CB) tripped, output latched OFF"));
  if (smfr & (1U << 6)) emit(F("MFR: external MOSFET failure (FET fail)"));
  if (smfr & (1U << 3)) emit(F("MFR: black-box RAM full"));
  if (smfr & (1U << 2)) emit(F("MFR: FET fault on GATE2 (open/short)"));
  if (smfr & (1U << 1)) emit(F("MFR: FET fault on GATE1 (open/short)"));
  if (smfr & (1U << 0)) emit(F("MFR: FET drain sense fault"));

  /* ---- STATUS_MFR_SPECIFIC_2 (PMBus 0xF3, 16-bit) ---------------------
   *  Bits per LM5066H1 driver:
   *      12 watchdogFault, 11 scFault,
   *       9 einOfWarn,      8 vinTran,
   *       6 eeProg (INFO),  5 avgDone (INFO),
   *       3 retryRec (INFO),2 powerCycleRec (INFO),
   *       1 initDone (INFO)                                               */
  if (smfr2 & (1U << 12)) emit(F("MFR2: internal watchdog fault (chip lost host service)"));
  if (smfr2 & (1U << 11)) emit(F("MFR2: short-circuit fault detected"));
  if (smfr2 & (1U <<  9)) emit(F("MFR2: energy accumulator (EIN) overflow warning"));
  if (smfr2 & (1U <<  8)) emit(F("MFR2: VIN transient excursion detected"));

  /* ---- DIAGNOSTIC_WORD (PMBus 0xE1, 16-bit) ---------------------------
   *  Bits per LM5066H1 driver - low byte (faults) + bits 9/8 latched:
   *      15 voutUvWarn,   14 iinOpWarn,    13 vinUvWarn,    12 vinOvWarn,
   *      11 powerGood (INFO - asserted means good),
   *      10 overTempWarn,
   *       9 timerLatchedOff, 8 fetFail,
   *       7 configPreset (INFO - default config in use),
   *       6 deviceOff (INFO),
   *       5 vinUvFault, 4 vinOvFault, 3 iinOcPfetOpFault,
   *       2 overTempFault, 1 cmlFault, 0 circuitBreakerFault              */
  if (diag & (1U <<  9)) emit(F("DIAG: timer latched OFF (CB / OC blanking expired)"));
  if (diag & (1U <<  8)) emit(F("DIAG: external MOSFET failure (FET fail)"));
  if (diag & (1U <<  5)) emit(F("DIAG: VIN undervoltage fault (input below UVLO)"));
  if (diag & (1U <<  4)) emit(F("DIAG: VIN overvoltage fault (input above OV)"));
  if (diag & (1U <<  3)) emit(F("DIAG: IIN overcurrent / power-FET op fault"));
  if (diag & (1U <<  2)) emit(F("DIAG: over-temperature fault"));
  /* DIAG bit 1 mirrors STATUS_CML.  If the only CML bit is the "none of
   * the above" noise bit (bit 1 of SCML, set by any unsupported PMBus
   * access), suppress the summary - we already skip that SCML bit above. */
  const bool real_cml_bits =
      (scml & ((1U << 7) | (1U << 6) | (1U << 5) | (1U << 4))) != 0U;
  if ((diag & (1U <<  1)) && real_cml_bits)
    emit(F("DIAG: CML communication fault"));
  if (diag & (1U <<  0)) emit(F("DIAG: circuit breaker tripped"));

  if (!any) {
    Serial.print(F("    [OK   ] "));
    Serial.print(rail);
    Serial.println(F(": no active fault"));
  }
}

static void publishTelemetry() {
  RailTelemetry tlm = {};
  rail::Controller* rails[kRailCount] = {&s_rail48, &s_rail24, &s_rail12};
  Serial.print(F("[TLM] mode="));
  Serial.print(modeToString(s_mode));
  Serial.print(F(" estop="));
  Serial.println(estop::isEStopActive() ? F("YES") : F("no"));
  for (size_t i = 0U; i < kRailCount; ++i) {
    rails[i]->buildTelemetry(tlm);
    Serial.print(F("  "));
    Serial.print(railToString(tlm.rail));
    Serial.print(F(": "));
    Serial.print(tlm.present ? F("PRES") : F("ABS "));
    Serial.print(F(" OP=0x"));
    Serial.print(tlm.operation_raw, HEX);
    Serial.print(F(" CMD="));
    const bool op_cmd_on = (tlm.operation_raw & 0x80U) != 0U;
    Serial.print(op_cmd_on ? F("ON ") : F("OFF"));
    Serial.print(F(" FET="));
    const bool chip_reports_off = (tlm.status_word & (1U << 6)) != 0U;
    Serial.print((tlm.present && op_cmd_on && !chip_reports_off) ? F("ON ") : F("OFF"));
    Serial.print(F(" PG="));
    Serial.print(tlm.pgood_pin ? '1' : '0');
    Serial.print(F(" VIN="));
    Serial.print(tlm.vin_v, 2);
    Serial.print(F(" VOUT="));
    Serial.print(tlm.vout_v, 2);
    Serial.print(F(" VAUX="));
    Serial.print(tlm.vaux_v, 3);
    Serial.print(F(" IIN="));
    Serial.print(tlm.iin_a, 2);
    Serial.print(F(" PIN="));
    Serial.print(tlm.pin_w, 1);
    Serial.print(F(" Ppk="));
    if (tlm.peak_valid) {
      Serial.print(tlm.peak_pin_w, 1);
      Serial.print(F("W@"));
      Serial.print(tlm.peak_vin_v, 2);
      Serial.print(F("V/"));
      Serial.print(tlm.peak_iin_a, 2);
      Serial.print(F("A"));
    } else {
      Serial.print(F("n/a"));
    }
    Serial.print(F(" Tdie="));
    Serial.print(tlm.die_temp_c, 1);
    Serial.print(F(" Tntc="));
    Serial.print(tlm.ntc_temp_c, 1);
    Serial.print(F(" SW=0x"));
    Serial.print(tlm.status_word, HEX);
    Serial.print(F(" DIAG=0x"));
    Serial.print(tlm.diag_word, HEX);
    Serial.print(F(" SMFR2=0x"));
    Serial.print(tlm.status_mfr_specific2, HEX);
    Serial.print(F(" WDPLB=0x"));
    Serial.print(tlm.wd_plb_timer, HEX);
    Serial.print(F(" BB="));
    Serial.print(tlm.bb_valid ? '1' : '0');
    Serial.print(F("/"));
    Serial.print(tlm.bb_ram_len);
    Serial.print(F("/"));
    Serial.print(tlm.bb_eeprom_len);
    Serial.print(F(" SIN=0x"));
    Serial.print(tlm.status_input, HEX);
    Serial.print(F(" SCML=0x"));
    Serial.print(tlm.status_cml, HEX);
    Serial.print(F(" SMFR=0x"));
    Serial.print(tlm.status_mfr_specific, HEX);
    Serial.print(F(" faults="));
    Serial.println(tlm.fault_count);
    describeRailFaults(tlm);
  }
  const winch::Telemetry w = winch::telemetry();
  Serial.print(F("  WINCH: mode="));
  Serial.print(static_cast<uint8_t>(w.mode));
  Serial.print(F(" awake="));
  Serial.print(w.awake ? '1' : '0');
  Serial.print(F(" fault="));
  Serial.print(w.fault_active ? '1' : '0');
  Serial.print(F(" dcA="));
  Serial.print(w.motor_a_cmd_pct);
  Serial.print(F("% dcB="));
  Serial.print(w.motor_b_cmd_pct);
  Serial.print(F("% par="));
  Serial.print(w.parallel_cmd_pct);
  Serial.print(F("% stpA="));
  Serial.print(w.stepper_a_cmd_pct);
  Serial.print(F("% stpB="));
  Serial.print(w.stepper_b_cmd_pct);
  Serial.println('%');
  const winch_lock::Telemetry wl = winch_lock::telemetry();
  Serial.print(F("  WINCH_LOCK: lock1="));
  Serial.print(wl.lock1_on ? '1' : '0');
  Serial.print(F(" lock2="));
  Serial.println(wl.lock2_on ? '1' : '0');
}

static void printHex16(uint16_t value) {
  if (value < 0x1000U) {
    Serial.print('0');
  }
  if (value < 0x0100U) {
    Serial.print('0');
  }
  if (value < 0x0010U) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

static void scanSmbusAddresses() {
  uint8_t found = 0U;

  Serial.println(F("[SMBUS SCAN] I2C1 PB6/PB7 scanning 7-bit addresses 0x03..0x77"));
  for (uint8_t address = 0x03U; address <= 0x77U; ++address) {
    Wire.beginTransmission(address);
    const uint8_t rc = Wire.endTransmission();
    if (rc == 0U) {
      Serial.print(F("[SMBUS SCAN] ACK addr=0x"));
      if (address < 0x10U) {
        Serial.print('0');
      }
      Serial.print(address, HEX);
      Serial.print(F(" rail="));
      if (address == cfg::kAddr12V) {
        Serial.print(F("12V"));
      } else if (address == cfg::kAddr24V) {
        Serial.print(F("24V"));
      } else if (address == cfg::kAddr48V) {
        Serial.print(F("48V"));
      } else {
        Serial.print(F("UNKNOWN"));
      }

      LM5066H1 probe(address);
      LM5066H1::NtcConfig ntc = {};
      ntc.pullupOhms   = cfg::kNtcPullup_Ohms;
      ntc.nominalOhms  = cfg::kNtcNominal_Ohms;
      ntc.nominalTempC = cfg::kNtcNominalTemp_C;
      ntc.beta         = cfg::kNtcBetaK;
      ntc.supplyVolts  = cfg::kNtcSupply_V;
      probe.setNtcConfig(ntc);

      uint16_t raw_vin = 0U;
      double vin_v = 0.0;
      if (probe.beginAttached(cfg::kI2c1HotswapClock_Hz) &&
          probe.readVinRaw(raw_vin) &&
          raw_vin != 0x0000U &&
          raw_vin != 0xFFFFU &&
          probe.readVin(vin_v)) {
        Serial.print(F(" READ_VIN_RAW=0x"));
        printHex16(raw_vin);
        Serial.print(F(" READ_VIN="));
        Serial.print(vin_v, 2);
        Serial.print(F("V"));
      } else {
        Serial.print(F(" READ_VIN_RAW=0x"));
        printHex16(raw_vin);
        Serial.print(F(" READ_VIN=invalid"));
      }

      uint16_t raw_vaux = 0U;
      double vaux_v = 0.0;
      double ntc_c = 0.0;
      if (probe.readVauxRaw(raw_vaux) &&
          raw_vaux != 0x0000U &&
          raw_vaux != 0xFFFFU &&
          probe.readVaux(vaux_v) &&
          probe.readNtcTemperatureC(ntc_c)) {
        Serial.print(F(" READ_VAUX_RAW=0x"));
        printHex16(raw_vaux);
        Serial.print(F(" READ_VAUX="));
        Serial.print(vaux_v, 3);
        Serial.print(F("V Tntc="));
        Serial.print(ntc_c, 1);
        Serial.println(F("C"));
      } else {
        Serial.print(F(" READ_VAUX_RAW=0x"));
        printHex16(raw_vaux);
        Serial.println(F(" READ_VAUX/Tntc=invalid"));
      }
      ++found;
    }
    delay(2);
  }

  Serial.print(F("[SMBUS SCAN] found="));
  Serial.println(found);
}

static void scan48VSmbusAddress() {
  Serial.print(F("[SMBUS48 SCAN] SCL=PA2 SDA=PA3 addr=0x"));
  Serial.print(cfg::kAddr48V, HEX);

  LM5066H1 probe(cfg::kAddr48V, s_smbus48);
  LM5066H1::NtcConfig ntc = {};
  ntc.pullupOhms   = cfg::kNtcPullup_Ohms;
  ntc.nominalOhms  = cfg::kNtcNominal_Ohms;
  ntc.nominalTempC = cfg::kNtcNominalTemp_C;
  ntc.beta         = cfg::kNtcBetaK;
  ntc.supplyVolts  = cfg::kNtcSupply_V;
  probe.setNtcConfig(ntc);

  uint16_t raw_vin = 0U;
  double vin_v = 0.0;
  if (probe.beginAttached(cfg::kSmbus48Clock_Hz) &&
      probe.readVinRaw(raw_vin) &&
      raw_vin != 0x0000U &&
      raw_vin != 0xFFFFU &&
      probe.readVin(vin_v)) {
    Serial.print(F(" ACK rail=48V READ_VIN_RAW=0x"));
    printHex16(raw_vin);
    Serial.print(F(" READ_VIN="));
    Serial.print(vin_v, 2);
    Serial.print(F("V"));
  } else {
    Serial.print(F(" no valid 48V response READ_VIN_RAW=0x"));
    printHex16(raw_vin);
  }

  uint16_t raw_vaux = 0U;
  double vaux_v = 0.0;
  double ntc_c = 0.0;
  if (probe.readVauxRaw(raw_vaux) &&
      raw_vaux != 0x0000U &&
      raw_vaux != 0xFFFFU &&
      probe.readVaux(vaux_v) &&
      probe.readNtcTemperatureC(ntc_c)) {
    Serial.print(F(" READ_VAUX_RAW=0x"));
    printHex16(raw_vaux);
    Serial.print(F(" READ_VAUX="));
    Serial.print(vaux_v, 3);
    Serial.print(F("V Tntc="));
    Serial.print(ntc_c, 1);
    Serial.println(F("C"));
  } else {
    Serial.print(F(" READ_VAUX_RAW=0x"));
    printHex16(raw_vaux);
    Serial.println(F(" READ_VAUX/Tntc=invalid"));
  }
}

static void configureProbeNtc(LM5066H1& probe) {
  LM5066H1::NtcConfig ntc = {};
  ntc.pullupOhms   = cfg::kNtcPullup_Ohms;
  ntc.nominalOhms  = cfg::kNtcNominal_Ohms;
  ntc.nominalTempC = cfg::kNtcNominalTemp_C;
  ntc.beta         = cfg::kNtcBetaK;
  ntc.supplyVolts  = cfg::kNtcSupply_V;
  probe.setNtcConfig(ntc);
}

static void printContinuousRead(const char* rail_name, LM5066H1& probe,
                                uint32_t clock_hz) {
  uint16_t raw_vin = 0U;
  uint16_t raw_vaux = 0U;
  double vin_v = 0.0;
  double vaux_v = 0.0;
  double ntc_c = 0.0;

  Serial.print(F("[SMBUS READ] "));
  Serial.print(rail_name);

  const bool vin_ok =
      probe.beginAttached(clock_hz) &&
      probe.readVinRaw(raw_vin) &&
      raw_vin != 0x0000U &&
      raw_vin != 0xFFFFU &&
      probe.readVin(vin_v);
  if (vin_ok) {
    Serial.print(F(" VIN_RAW=0x"));
    printHex16(raw_vin);
    Serial.print(F(" VIN="));
    Serial.print(vin_v, 2);
    Serial.print(F("V"));
  } else {
    Serial.print(F(" VIN_RAW=0x"));
    printHex16(raw_vin);
    Serial.print(F(" VIN=invalid"));
  }

  const bool vaux_ok =
      probe.readVauxRaw(raw_vaux) &&
      raw_vaux != 0x0000U &&
      raw_vaux != 0xFFFFU &&
      probe.readVaux(vaux_v) &&
      probe.readNtcTemperatureC(ntc_c);
  if (vaux_ok) {
    Serial.print(F(" VAUX_RAW=0x"));
    printHex16(raw_vaux);
    Serial.print(F(" VAUX="));
    Serial.print(vaux_v, 3);
    Serial.print(F("V Tntc="));
    Serial.print(ntc_c, 1);
    Serial.println(F("C"));
  } else {
    Serial.print(F(" VAUX_RAW=0x"));
    printHex16(raw_vaux);
    Serial.println(F(" VAUX/Tntc=invalid"));
  }
}

static void readKnownSmbusDevices() {
  LM5066H1 probe48(cfg::kAddr48V, s_smbus48);
  LM5066H1 probe24(cfg::kAddr24V);
  LM5066H1 probe12(cfg::kAddr12V);

  configureProbeNtc(probe48);
  configureProbeNtc(probe24);
  configureProbeNtc(probe12);

  printContinuousRead("48V PA2/PA3", probe48, cfg::kSmbus48Clock_Hz);
  printContinuousRead("24V PB6/PB7", probe24, cfg::kI2c1HotswapClock_Hz);
  printContinuousRead("12V PB6/PB7", probe12, cfg::kI2c1HotswapClock_Hz);
}

static void publishPeriodicReadout(uint32_t now) {
  if ((now - s_last_telemetry_ms) < cfg::kTelemetry_ms) {
    return;
  }
  s_last_telemetry_ms = now;

  readKnownSmbusDevices();
  publishTelemetry();
}

static SupervisorMode classifyMode() {
  if (estop::isEStopActive()) {
    return SupervisorMode::kEStop;
  }
  const rail::State states[kRailCount] = {
      s_rail48.state(),
      s_rail24.state(),
      s_rail12.state(),
  };
  bool any_latched = false;
  bool any_tripped = false;
  bool any_running = false;
  for (size_t i = 0U; i < kRailCount; ++i) {
    if (states[i] == rail::State::kLatched) any_latched = true;
    if (states[i] == rail::State::kTripped) any_tripped = true;
    if (states[i] == rail::State::kRunning ||
        states[i] == rail::State::kWarning) any_running = true;
  }
  if (any_latched)  return SupervisorMode::kFault;
  if (any_tripped)  return SupervisorMode::kDegraded;
  if (any_running)  return SupervisorMode::kNominal;
  return SupervisorMode::kDegraded;
}

static void configureF1Output(GPIO_TypeDef* port, uint8_t pin, bool high) {
  const uint32_t bit = (1UL << pin);
  volatile uint32_t* const cr =
      (pin < 8U) ? &port->CRL : &port->CRH;
  const uint8_t shift = static_cast<uint8_t>((pin & 0x07U) * 4U);

  if (high) {
    port->BSRR = bit;
  } else {
    port->BRR = bit;
  }
  *cr = (*cr & ~(0x0FUL << shift)) |
        (0x02UL << shift);               /* output push-pull, 2 MHz          */
  if (high) {
    port->BSRR = bit;
  } else {
    port->BRR = bit;
  }
}

static void forceKnownOutputsToSafeStateAtBoot() {
  /* This is the FIRST executable code in setup() and it touches every
   * safety-relevant output before the Arduino framework gets a chance to
   * leave a pin floating.  The controlled outputs are preloaded to their
   * normal released/off levels:
   *
   *    PA0  STM_E_STOP    HIGH = E-Stop not asserted by us
   *    PB13 E-Stop VTX    LOW = VTX loop de-energised
   *    PA6/PA7/PB0/PB1    LOW = winch H-bridge inputs idle
   *    PA8/PB15           LOW = winch locks de-energised
   *    PB4/PB5/PB8/PB9    LOW = lighting outputs off
   *
   * Using direct GPIO register writes guarantees there is no transient
   * window at an unintended level between mode change and data change.    */
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN |
                  RCC_APB2ENR_IOPBEN |
                  RCC_APB2ENR_AFIOEN;
  (void)RCC->APB2ENR;

  configureF1Output(GPIOA, 0U,  true);   /* PA0  E-Stop command  (released)    */
  configureF1Output(GPIOA, 6U,  false);  /* PA6  Winch IN1                     */
  configureF1Output(GPIOA, 7U,  false);  /* PA7  Winch IN2                     */
  configureF1Output(GPIOA, 8U,  false);  /* PA8  Winch lock 2                  */

  configureF1Output(GPIOB, 0U,  false);  /* PB0  Winch IN3                     */
  configureF1Output(GPIOB, 1U,  false);  /* PB1  Winch IN4                     */
  configureF1Output(GPIOB, 4U,  false);  /* PB4  LED Bras                      */
  configureF1Output(GPIOB, 5U,  false);  /* PB5  LED Avant                     */
  configureF1Output(GPIOB, 8U,  false);  /* PB8  LED Arr                       */
  configureF1Output(GPIOB, 9U,  false);  /* PB9  LED Extra                     */
  configureF1Output(GPIOB, 13U, false);  /* PB13 E-Stop VTX (de-energised)     */
  configureF1Output(GPIOB, 15U, false);  /* PB15 Winch lock 1                  */
}

/* ---------------------------------------------------------------------------
 *  Arduino entry points
 * ---------------------------------------------------------------------------*/
void setup() {
  /* (0) Fail-safe GPIO state FIRST, before anything else, including the
   *     console.  The STM32F103 boots with all GPIOs in Input Floating.
   *     Pre-load every actuator/output pin to its safe level, then switch
   *     it to OUTPUT, so no external load sees a wrong pulse during boot.  */
  forceKnownOutputsToSafeStateAtBoot();

  /* E-Stop owns PA0/PB12/PB13 after the safe-state pass above.              */
  estop::init();

  /* (1) Console next.  When PDU_USE_SWO is active this also runs
   *     SerialSWO::enableTrace() which forces AFIO->MAPR.SWJ_CFG = 0b001
   *     (Full SWJ without NJTRST).  That single write achieves two things:
   *        - keeps PB3 alive as TRACESWO (so this very console works);
   *        - frees PB4 as a regular GPIO (so the LED_Bras pinMode()
   *          performed by leds::init() right after will actually drive
   *          the pad).
   *     The order matters: leds::init() MUST run AFTER Serial.begin() so
   *     that PB4 is no longer owned by the SWJ-DP when pinMode() is called.*/
  Serial.begin(cfg::kSerialBaud);
  fault_log::init();
  iwdg::init();
  s_boot_watchdog_reset = iwdg::wasResetByWatchdog();
  iwdg::kick();

  /* PB14 is intentionally unused by firmware logic, but the schematic keeps it
   * connected.  Force it to plain GPIO input early and leave it untouched. */
  pinMode(cfg::kPin_PB14_Unused, INPUT);

  /* (2) Output drivers are initialised after the fail-safe LOW boot pass.   */
  leds::init();
  leds::setPattern(leds::Pattern::kSolid);
  leds::setDuty(leds::Channel::kBras, 20U);
  leds::setDuty(leds::Channel::kAvant, 40U);
  leds::setDuty(leds::Channel::kArr, 60U);
  leds::setDuty(leds::Channel::kExtra, 80U);
  winch::init();
  winch_lock::init();

  delay(100);
  iwdg::kick();

  Serial.println();
  Serial.println(F("==================================================="));
  Serial.print  (F("  PDU Firmware v"));
  Serial.print  (cfg::kFwVersionMajor); Serial.print('.');
  Serial.print  (cfg::kFwVersionMinor); Serial.print('.');
  Serial.println(cfg::kFwVersionPatch);
  Serial.println(F("  Target : STM32F103C8T6 (Blue Pill)"));
  Serial.println(F("  Devices: 3 x LM5066H1 (48V / 24V / 12V)"));
  Serial.println(F("==================================================="));
  if (s_boot_watchdog_reset) {
    Serial.println(F("[BOOT] previous reset cause: WATCHDOG"));
    fault_log::record(fault_log::Code::kWatchdogReset, Rail::kCount, 0U, 0U);
  }

  /* Bring up I2C bus before talking to any LM5066H1 instance.              */
  Wire.setSCL(cfg::kPin_I2C1_SCL);
  Wire.setSDA(cfg::kPin_I2C1_SDA);
  Wire.begin();
  Wire.setClock(cfg::kI2c1HotswapClock_Hz);
  s_smbus48.begin();
  s_smbus48.setClock(cfg::kSmbus48Clock_Hz);

  scanSmbusAddresses();
  scan48VSmbusAddress();
  iwdg::kick();

  /* Make sure rails are de-energised before doing anything else.           */
  enterMode(SupervisorMode::kPbit);

  /* Probe & configure each LM5066H1.                                       */
  s_rail48.init();
  iwdg::kick();
  s_rail24.init();
  iwdg::kick();
  s_rail12.init();
  iwdg::kick();

  control_api::init();
  Serial.print(F("[API] I2C2 slave addr=0x"));
  Serial.print(cfg::kApiI2cAddress, HEX);
  Serial.println(F(" pins SCL=PB10 SDA=PB11"));

  /* Power-On BIT runs for telemetry / logging only; it never gates the
   * boot-time rail enable.  At boot the firmware defaults to the same state
   * RG would command on first contact (all MOSFETs ON).  Rails that are
   * absent simply reject the enable() internally.                           */
  const Status pbit = bit::runPbit(s_rail48, s_rail24, s_rail12, s_pbit_report);
  iwdg::kick();

  /* Default "RG-sent-ON" command for every LM5066H1 at boot.                */
  enableAllRails();

  if (pbit != Status::kOk) {
    Serial.println(F("[BOOT] PBIT failed - continuing with present rails enabled"));
  }
  enterMode(classifyMode());

  s_last_protect_ms   = millis();
  s_last_telemetry_ms = millis();
  s_last_cbit_ms      = millis();
  s_last_heartbeat_ms = millis();
}

void loop() {
  const uint32_t loop_started_ms = millis();
  const uint32_t now = loop_started_ms;

  /* Run housekeeping that must happen at every iteration.                   */
  estop::tick();
  leds::tick();
  control_api::tick(s_rail48, s_rail24, s_rail12, s_mode, s_pbit_report, s_cbit_report);
  publishPeriodicReadout(now);

  /* ---------- E-Stop handling has the highest priority ------------------ */
  if (estop::isEStopActive()) {
    if (s_mode != SupervisorMode::kEStop) {
      disableAllRails();
      enterMode(SupervisorMode::kEStop);
    }
    /* Continue to pet the dog and update LEDs while latched.               */
    serviceWatchdogIfHealthy(loop_started_ms);
    return;
  }
  if (s_mode == SupervisorMode::kEStop) {
    /* E-Stop just released - go through DEGRADED so a human action is
     * required to re-enable rails.                                         */
    enterMode(SupervisorMode::kDegraded);
  }

  /* ---------- Periodic protection tick ---------------------------------- */
  if ((now - s_last_protect_ms) >= cfg::kProtectionSamplePeriod_ms) {
    s_last_protect_ms = now;
    s_rail48.tick();
    s_rail24.tick();
    s_rail12.tick();
    /* Reclassify supervisor mode based on the latest rail states.          */
    const SupervisorMode m = classifyMode();
    if (m != s_mode &&
        s_mode != SupervisorMode::kFault) {
      enterMode(m);
    }
  }

  /* ---------- CBIT ----------------------------------------------------- */
  if ((now - s_last_cbit_ms) >= cfg::kBitContinuous_ms) {
    s_last_cbit_ms = now;
    bit::runCbit(s_rail48, s_rail24, s_rail12, s_cbit_report);
    if (!s_cbit_report.all_passed) {
      Serial.println(F("[CBIT] one or more rails failed continuous test"));
      enterMode(SupervisorMode::kDegraded);
    }
  }

  /* ---------- Heartbeat (visible to operators) -------------------------- */
  if ((now - s_last_heartbeat_ms) >= cfg::kHeartbeat_ms) {
    s_last_heartbeat_ms = now;
    /* Heartbeat is encoded by the LED pattern; nothing more to do here.    */
  }

  serviceWatchdogIfHealthy(loop_started_ms);
}
