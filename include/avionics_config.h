/* =============================================================================
 *  avionics_config.h
 * -----------------------------------------------------------------------------
 *  Master configuration of the Power Distribution Unit (PDU).
 *  All hardware-specific constants, pin mappings, current/voltage thresholds
 *  and protection windows live here so that any field re-configuration can be
 *  done without touching the application logic.
 *
 *  Coding standard: inspired by MISRA-C:2012 and DO-178C Level B principles.
 *  Every literal carries a [REQ-x] traceability tag mapping it to the system
 *  requirement document so that requirement coverage can be audited.
 * =============================================================================
 */
#ifndef PDU_AVIONICS_CONFIG_H_
#define PDU_AVIONICS_CONFIG_H_

#include <Arduino.h>
#include <stdint.h>

namespace pdu {
namespace cfg {

constexpr uint32_t kPin_NotConnected = 0xFFFFFFFFUL;

/* ---------------------------------------------------------------------------
 *  Firmware identification block.  Used by the Built-In Test to confirm the
 *  binary running on the target matches the expected configuration.
 * ---------------------------------------------------------------------------*/
constexpr uint8_t kFwVersionMajor = PDU_FW_VERSION_MAJOR;
constexpr uint8_t kFwVersionMinor = PDU_FW_VERSION_MINOR;
constexpr uint8_t kFwVersionPatch = PDU_FW_VERSION_PATCH;

/* ===========================================================================
 *  1) PMBus / SMBus device addresses
 *  ---------------------------------------------------------------------------
 *  Decoded from LM5066H Table 7-74 (Device Addressing).
 *  Z = pin floating, 0 = tied to GND, 1 = tied to VDD.
 * ===========================================================================
 */

/** [REQ-PWR-001] 48V hot-swap: ADR2=1, ADR1=Z, ADR0=Z -> 7-bit I2C addr 0x52 */
constexpr uint8_t kAddr48V = 0x52U;
/** [REQ-PWR-002] 24V hot-swap: ADR2=Z, ADR1=0, ADR0=Z -> 7-bit I2C addr 0x43 */
constexpr uint8_t kAddr24V = 0x43U;
/** [REQ-PWR-003] 12V hot-swap: ADR2=Z, ADR1=Z, ADR0=0 -> 7-bit I2C addr 0x41 */
constexpr uint8_t kAddr12V = 0x41U;

/* ===========================================================================
 *  2) Sense-resistor values (in milli-ohms)
 *  ---------------------------------------------------------------------------
 *  Each LM5066H1 measures load current through its CS pins.  Accurate Rsns
 *  values are mandatory for the on-chip Direct-Format coefficients used by
 *  READ_IIN / READ_PIN to produce engineering units.
 * ===========================================================================
 */

/** 48V rail: 2 x 200 µΩ in parallel = 100 µΩ = 0.1 mΩ. */
constexpr double kRsns48V_mOhm = 0.1;
/** 24V rail: single 1.3 mΩ shunt. */
constexpr double kRsns24V_mOhm = 1.3;
/** 12V rail: single 2.0 mΩ shunt. */
constexpr double kRsns12V_mOhm = 2.0;

/* ===========================================================================
 *  3) Current protection thresholds and time-windows
 *  ---------------------------------------------------------------------------
 *  Two protection layers are used:
 *    - Hardware  : LM5066H1 internal circuit-breaker (instant, latency ~us).
 *    - Software  : Supervisor running on the STM32 polls READ_IIN at fixed
 *                  cadence and trips the rail when long-duration limits are
 *                  exceeded.  This layer handles the "i^2 * t"-style curves
 *                  that the LM5066H1 cannot enforce directly.
 * ===========================================================================
 */

/* ----- 48V rail -------------------------------------------------------------
 *  [REQ-PWR-010] Continuous 70 A is allowed indefinitely.
 *  [REQ-PWR-011] 80 A may be drawn for up to 3 minutes.
 *  [REQ-PWR-012] 100 A may be drawn for up to 1 minute.
 *  [REQ-PWR-013] 150 A is the absolute peak: must trip instantly.
 * --------------------------------------------------------------------------- */
constexpr double   kI48VContinuous_A      = 70.0;
constexpr double   kI48VWarn3min_A        = 80.0;
constexpr double   kI48VWarn1min_A        = 100.0;
constexpr double   kI48VPeakInstant_A     = 150.0;
constexpr uint32_t kI48VWarn3minWindow_ms = 180000UL; /* 3 minutes */
constexpr uint32_t kI48VWarn1minWindow_ms = 60000UL;  /* 1 minute  */

/* ----- 24V rail -------------------------------------------------------------
 *  [REQ-PWR-020] 20 A continuous, anything above trips instantly.
 * --------------------------------------------------------------------------- */
constexpr double kI24VMax_A = 20.0;

/* ----- 12V rail -------------------------------------------------------------
 *  [REQ-PWR-030] 10 A continuous, anything above trips instantly.
 * --------------------------------------------------------------------------- */
constexpr double kI12VMax_A = 10.0;

/* ---------------------------------------------------------------------------
 *  Sampling cadence of the software protection layer.  Must be fast enough
 *  that, even at the upper end of the inverse-time curve, the firmware can
 *  detect and react before the hardware breaker fires.  100 ms is a good
 *  trade-off between I2C bus loading and reaction time.
 * ---------------------------------------------------------------------------*/
constexpr uint32_t kProtectionSamplePeriod_ms = 100UL;

/* ===========================================================================
 *  4) Voltage / temperature warning limits
 *  ---------------------------------------------------------------------------
 *  Loaded into the LM5066H1 fault registers during initialisation so the
 *  device asserts SMBA on its own without supervisor intervention.
 * ===========================================================================
 */
constexpr double kVin48V_OV_V = 60.0;  /* over-voltage warn at +25 %  */
constexpr double kVin48V_UV_V = 36.0;  /* under-voltage warn at -25 % */
constexpr double kVin24V_OV_V = 30.0;
constexpr double kVin24V_UV_V = 18.0;
constexpr double kVin12V_OV_V = 15.0;
constexpr double kVin12V_UV_V = 9.0;

constexpr double kOtWarn_C  = 100.0;
constexpr double kOtFault_C = 125.0;

/* Hot-swap enable/disable policy:
 *   false  -> the STM32 turns all rails ON once at boot (after configureDevice)
 *             and honours estop / API commands with one-shot writes.
 *   true   -> the STM32 never writes OPERATION; RG is solely responsible for
 *             commanding each rail on via the control API.
 *
 * Either way, tick() never re-asserts OPERATION: each enable() or disable()
 * is a single write, and the LM5066H1's own infinite retry (retrySetting = 7)
 * handles fault recovery without STM32 intervention.                          */
constexpr bool kHotswapApiOnly = false;

/* ===========================================================================
 *  5) NTC thermistor parameters (each rail has its own NTC near the MOSFET)
 *  ---------------------------------------------------------------------------
 *  Voltage divider:   3V3 --[ Rpu = 10 k ]--+--[ NTC ]-- GND
 *                                            |
 *                                          VAUX
 *  Beta value provided by the hardware team: B25/85 = 3435 K, R25 = 10 k.
 * ===========================================================================
 */
constexpr double kNtcPullup_Ohms     = 10000.0;
constexpr double kNtcNominal_Ohms    = 10000.0;
constexpr double kNtcNominalTemp_C   = 25.0;
constexpr double kNtcBetaK           = 3435.0;
constexpr double kNtcSupply_V        = 3.3;

/** Software-side NTC trip and warning thresholds (independent of LM5066 OT). */
constexpr double kNtcWarn_C  = 95.0;
constexpr double kNtcTrip_C  = 115.0;

/* ===========================================================================
 *  6) STM32F103C8T6 (Blue Pill) GPIO map
 *  ---------------------------------------------------------------------------
 *  Pins are referenced by the Arduino-STM32 macro names (PA0, PB12, ...) so
 *  the map remains valid regardless of the variant board pin-numbering.
 *
 *  I2C1 (PB6/PB7) is the native SMBus/PMBus bus for the 12 V and 24 V
 *  LM5066H1 devices.  The 48 V LM5066H1 is isolated on a software SMBus
 *  using PA2/PA3 so a fault or wiring issue on that rail cannot hold the
 *  lower-voltage hot-swap bus.
 * ===========================================================================
 */

/* --- Discrete I/O ---------------------------------------------------------- */
/** [REQ-IO-001] PA0  -> output, master E-Stop command. Active-LOW:
 *  LOW asserts E-Stop / circuit OFF, HIGH releases the local command.        */
constexpr uint32_t kPin_EStopCmd       = PA0;
/** [REQ-IO-002] PA1  -> input,  Power-Good of the 24V LM5066H1 (active high).*/
constexpr uint32_t kPin_PG_24V         = PA1;
/** [REQ-IO-003] PA15 -> input,  Power-Good of the 12V LM5066H1 (active high).*/
constexpr uint32_t kPin_PG_12V         = PA15;
/** [REQ-IO-004] PB14 -> input, reserved / not used by firmware logic.        */
constexpr uint32_t kPin_PB14_Unused    = PB14;
/** [REQ-IO-005] PB12 -> input,  E-Stop status feedback line.                 */
constexpr uint32_t kPin_EStopStatus    = PB12;
/** [REQ-IO-006] PB13 -> output, E-Stop VTX command line.                     */
constexpr uint32_t kPin_EStopVtx       = PB13;

/* --- LED PWM channels ------------------------------------------------------
 *  WARNING: PB3 is RESERVED for ST-Link SWO trace output (Serial Wire Output).
 *  See README §8 for the wiring (Blue Pill PB3 -> ST-Link SWO pin).  Do NOT
 *  reuse PB3 for any other peripheral if you want SWO printf-style debug.
 *
 *  VNQ5E050AKTR-E is a quad high-side driver.  The STM32 pins below drive
 *  the four VNQ inputs; current-sense pins are not connected in this firmware
 *  mapping unless extra ADC pins are assigned later.
 * --------------------------------------------------------------------------- */
constexpr uint32_t kPin_LED_Bras       = PB4;  /* TIM3_CH1_REMAP / PWM         */
constexpr uint32_t kPin_LED_Avant      = PB5;  /* schematic: LED Avant PWM     */
constexpr uint32_t kPin_LED_Arr        = PB8;  /* schematic: LED Arri PWM      */
constexpr uint32_t kPin_LED_Extra      = PB9;  /* schematic: LED Extra PWM     */

/* --- TPS2HB16AQPWPRQ1 winch lock high-side switch -------------------------
 *  Dual-channel smart high-side switch.  These pins drive the channel enable
 *  inputs.  OFF is the fail-safe boot/E-Stop state.
 * --------------------------------------------------------------------------- */
constexpr uint32_t kPin_WinchLock1_EN   = PB15;
constexpr uint32_t kPin_WinchLock2_EN   = PA8;

/* --- Hot-swap I2C bus ------------------------------------------------------ */
constexpr uint32_t kPin_I2C1_SCL       = PB6;
constexpr uint32_t kPin_I2C1_SDA       = PB7;
constexpr uint32_t kPin_SMBUS48_SCL    = PA2;      /* software SMBus only */
constexpr uint32_t kPin_SMBUS48_SDA    = PA3;      /* software SMBus only */
constexpr uint32_t kI2c1HotswapClock_Hz = 100000UL; /* 12V / 24V LM5066H1 */
constexpr uint32_t kSmbus48Clock_Hz     = 50000UL;  /* isolated 48V LM5066H1 */

/* --- DRV8262DDVR winch-lock motor driver ----------------------------------
 *  DRV8262 is used in PWM (IN/IN) interface. MODE1 selects dual H-bridge
 *  (2 x DC or 1 x stepper) versus single parallel H-bridge (1 x high-current
 *  DC). MODE pins are latched when nSLEEP goes high, so firmware changes mode
 *  by sleeping the driver, setting MODE1/MODE2, then waking it.
 *
 *  The schematic exposes only the four bridge inputs as OUT1..OUT4 Winch.
 *  MODE1/MODE2/nSLEEP/nFAULT are not routed to the STM32 in the provided
 *  pinout, so they are marked not-connected here.
 * --------------------------------------------------------------------------- */
constexpr uint32_t kPin_Winch_IN1       = PA6;  /* schematic: OUT1 Winch    */
constexpr uint32_t kPin_Winch_IN2       = PA7;  /* schematic: OUT2 Winch    */
constexpr uint32_t kPin_Winch_IN3       = PB0;  /* schematic: OUT3 Winch    */
constexpr uint32_t kPin_Winch_IN4       = PB1;  /* schematic: OUT4 Winch    */
constexpr uint32_t kPin_Winch_MODE1     = kPin_NotConnected;
constexpr uint32_t kPin_Winch_MODE2     = kPin_NotConnected;
constexpr uint32_t kPin_Winch_nSLEEP    = kPin_NotConnected;
constexpr uint32_t kPin_Winch_nFAULT    = kPin_NotConnected;

/* --- External control API I2C slave bus ------------------------------------
 *  I2C1 is reserved for the STM32 -> LM5066H1 SMBus master.  A host computer
 *  talks to the STM32 on I2C2 so it never arbitrates directly with the
 *  hot-swap controllers.                                                     */
constexpr uint32_t kPin_API_I2C2_SCL    = PB10;
constexpr uint32_t kPin_API_I2C2_SDA    = PB11;
/* Lowered from 100 kHz to 50 kHz: gives more rise-time margin on a bus
 * that relies on STM32 internal pull-ups and a bit-banged master, so the
 * slave can ACK reliably even if external pull-ups are weak/absent.   */
constexpr uint32_t kApiI2cClock_Hz      = 50000UL;
constexpr uint8_t  kApiI2cAddress       = 0x31U;

/* ===========================================================================
 *  7) Supervisor periods
 * ===========================================================================
 */
constexpr uint32_t kHeartbeat_ms       = 1000UL;  /* 1 Hz alive blink     */
constexpr uint32_t kTelemetry_ms       = 2000UL;  /* 0.5 Hz UART dump     */
constexpr uint32_t kBitContinuous_ms   = 5000UL;  /* CBIT cadence         */
constexpr uint32_t kIwdgTimeout_ms     = 4000UL;  /* IWDG window @LSI/256 */
constexpr uint32_t kPgDebounce_ms      = 25UL;    /* power-good debounce  */

/* ---------------------------------------------------------------------------
 *  Flight-build guard.
 * ---------------------------------------------------------------------------*/
#if !defined(PDU_FLIGHT_BUILD)
#  error "Only the bluepill_flight configuration is supported."
#endif

/* ===========================================================================
 *  8) UART / debug
 * ===========================================================================
 */
constexpr uint32_t kSerialBaud         = 115200UL;

}  /* namespace cfg */
}  /* namespace pdu */

#endif /* PDU_AVIONICS_CONFIG_H_ */
