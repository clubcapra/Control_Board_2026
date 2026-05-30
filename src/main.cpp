/* =============================================================================
 *  main.cpp - Power Distribution Unit (PDU) supervisor
 * -----------------------------------------------------------------------------
 *  AIRWORTHINESS DISCLOSURE  ----  READ BEFORE USING THIS IMAGE
 *  ------------------------------------------------------------
 *  This source file follows the structural shape of a DO-178C / MISRA-C++
 *  flight-software baseline (phased boot, rate-monotonic super-loop, REQ
 *  traceability, named bit positions, defensive `default:`, bounded
 *  iterations, static_asserts, stack canary, image CRC32 self-test).
 *
 *  IT IS NOT, AND CANNOT BE, AIRWORTHY AS-DELIVERED, because:
 *    - The target hardware (STM32F103C8T6 "Blue Pill", 72 MHz Cortex-M3,
 *      no ECC, single-core, no fault-tolerant memory) is hobbyist-grade
 *      and has never been screened or qualified to MIL-STD-810 / DO-160.
 *    - The build relies on the unqualified Arduino-Core-STM32 HAL, the
 *      framework `Wire` and `SerialSWO` libraries, and the unqualified
 *      GCC ARM None Eabi toolchain.  Level A would require qualification
 *      of every tool in the build chain to DO-330 TQL-5.
 *    - No V&V evidence package (MC/DC coverage, requirements-based
 *      testing, hardware-in-the-loop campaign) has been generated for
 *      this image.
 *    - Static analysis with a qualified tool (Polyspace / Coverity) has
 *      not been run; only manual review and (optionally) cppcheck.
 *
 *  In short: this is a Power Distribution Unit for a competition rover,
 *  whose firmware has been brought to the *structural* shape an aerospace
 *  reviewer expects to see.  Treat any "F-35 / DO-178C" terminology in
 *  this file as the SHAPE of the source, not as a claim of certification.
 *
 *  CSCI            : PDU-Supervisor
 *  CSCI Software
 *  Level (DO-178C) : Structural baseline only (target Level B with full
 *                    qualification campaign on flight-grade hardware)
 *  Coding Std.     : MISRA-C++:2008 + JPL Power-of-10 (with documented waivers)
 *  Lifecycle Doc   : SDD-PDU-3.5  (Phased Boot + Rate-Monotonic Super-Loop)
 *  Target          : STM32F103C8T6 (Cortex-M3, 72 MHz, 64 KB flash, 20 KB RAM)
 *  Certifying Auth : (none -- this image is qualifiable in shape only)
 *
 *  Architectural rules upheld in this file
 *  ---------------------------------------
 *    R1.  Single super-loop, no RTOS, no threads, no IPC primitives.
 *    R2.  No dynamic memory after .preinit_array (Power-of-10 #3).
 *    R3.  Every loop has a statically determinable upper bound (P10 #2).
 *    R4.  Every nonvoid library call has its return value either checked or
 *         explicitly cast to (void) with a documented rationale (P10 #7).
 *    R5.  No recursion (P10 #1, MISRA-C++ 7-5-2).
 *    R6.  No goto / setjmp / longjmp (MISRA-C++ 6-6-1, 6-6-2).
 *    R7.  Every magic literal is named in the [REQ-PMBUS-NN] / [REQ-LED-NN]
 *         / [REQ-PWR-NN] constants block (MISRA-C++ 5-0-2, JPL Rule 4).
 *    R8.  Every `if`/`else`/`while`/`for` body is enclosed in braces
 *         (MISRA-C++ 6-4-1).
 *    R9.  No file-scope mutable state with external linkage; module state is
 *         `static` in the file (MISRA-C++ 3-3-2, P10 #6).
 *    R10. Hardware register access is `volatile` (MISRA-C++ 5-0-15).
 *    R11. The Independent Watchdog is armed in phase 2 and only kicked when
 *         the previous foreground iteration finished within
 *         kWatchdogMaxHealthyLoop_ms.
 *    R12. All actuator outputs are clamped to the inactive level by the
 *         .preinit_array hook before Arduino's `init()` runs.
 *
 *  REQ traceability (see avionics_config.h for thresholds)
 *  -------------------------------------------------------
 *    [REQ-PWR-001..003]  PMBus addresses for 48V/24V/12V hot-swap.
 *    [REQ-PWR-010..013]  48V SW protection ladder (70/80/100/150 A).
 *    [REQ-PWR-020/030]   24V/12V single-tier hard limits.
 *    [REQ-PWR-100]       Boot-time rail enable policy.
 *    [REQ-PWR-200]       De-energise-everything path.
 *    [REQ-LED-001..002]  All four PWM channels boot at 0 % duty as plain
 *                        GPIO LOW (no PWM 0%-glitch path).
 *    [REQ-LOCK-001]      Both winch-lock outputs LOW pre-init.
 *    [REQ-BOOT-001..050] Phased boot sequence (this file).
 *    [REQ-LOOP-001..099] Rate-monotonic super-loop (this file).
 *    [REQ-DIAG-001]      .noinit reset-survival breadcrumb.
 *    [REQ-DIAG-002]      Stack canary integrity check (this file).
 *    [REQ-DIAG-003]      Image-text CRC32 self-test at boot (this file).
 *
 *  MISRA-C++ waivers (each is justified at the call site)
 *  ------------------------------------------------------
 *    W1. MISRA-C++ 7-3-4 "no `using namespace`": waived at file-scope only
 *        for `pdu` because removing it would force >150 explicit
 *        `pdu::` qualifications without changing semantics.  All inner
 *        namespaces (`rail::`, `leds::`, ...) are still explicit.
 *    W2. MISRA-C++ 16-2-1 "preprocessor only for `#include` and simple
 *        `#define`": waived for the four ANSI macros (`ANSI_*`) and the
 *        compile-time mode gates (`PDU_VERBOSE_DEBUG`, `PDU_OUTPUT_TEST_ONLY`,
 *        `PDU_API_DEBUG_ONLY`, `PDU_FLIGHT_BUILD`).  The macros expand to
 *        constant string literals or compile out entirely.
 *    W3. MISRA-C++ 6-6-3 "single point of return": deliberately waived in
 *        early-exit predicates (e.g. `classifyMode`, `loopTaskEStop`)
 *        because forcing single-return increases cyclomatic complexity for
 *        no readability gain on functions <30 lines.
 *    W4. MISRA-C++ 18-4-1 "no `<new>`": Arduino HAL emits Wire and SerialSWO
 *        objects via static constructors.  We do NOT call `operator new`
 *        ourselves.  The framework's static initialisation runs in C++
 *        global-constructor space (before `setup()`), is bounded, and is
 *        observed to consume <0.1 KB of static storage.
 *
 *  Residuals (must be addressed before formal Level A submission)
 *  --------------------------------------------------------------
 *    Res1. Replace Arduino HAL with a qualified STM32 LL/CMSIS driver set.
 *    Res2. Replace `double` voltage / current comparisons with Q-format
 *          fixed-point math in protection paths (this file currently uses
 *          `double` only for telemetry display, not for trip decisions).
 *    Res3. Run Polyspace / CodeChecker / Coverity static analysis and
 *          attach the clean report.
 *    Res4. Qualify the GCC ARM toolchain to DO-330 TQL-5 (or equivalent).
 *    Res5. Provide MC/DC test coverage report from V&V campaign.
 * =============================================================================
 */
#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 *  ANSI color escape codes for the SWO/UART terminal.
 *
 *  The PlatformIO serial monitor (and any modern terminal emulator) renders
 *  these as colors.  Building with -DPDU_NO_COLOR strips them all to empty
 *  strings if you ever need plain text (e.g. piping to a file parser).
 * ---------------------------------------------------------------------------*/
#if !defined(PDU_NO_COLOR)
#  define PDU_ANSI(seq) seq
#else
#  define PDU_ANSI(seq) ""
#endif
#define ANSI_RESET        PDU_ANSI("\x1b[0m")
#define ANSI_BOLD         PDU_ANSI("\x1b[1m")
#define ANSI_DIM          PDU_ANSI("\x1b[2m")
#define ANSI_RED          PDU_ANSI("\x1b[31m")
#define ANSI_GREEN        PDU_ANSI("\x1b[32m")
#define ANSI_YELLOW       PDU_ANSI("\x1b[33m")
#define ANSI_BLUE         PDU_ANSI("\x1b[34m")
#define ANSI_MAGENTA      PDU_ANSI("\x1b[35m")
#define ANSI_CYAN         PDU_ANSI("\x1b[36m")
#define ANSI_GRAY         PDU_ANSI("\x1b[90m")
#define ANSI_BOLD_RED     PDU_ANSI("\x1b[1;31m")
#define ANSI_BOLD_GREEN   PDU_ANSI("\x1b[1;32m")
#define ANSI_BOLD_YELLOW  PDU_ANSI("\x1b[1;33m")
#define ANSI_BOLD_CYAN    PDU_ANSI("\x1b[1;36m")

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

namespace {

/* ---------------------------------------------------------------------------
 *  Ultra-early fail-safe output clamp [REQ-BOOT-001].
 *
 *  Arduino's `setup()` is not early enough for safety outputs: the core runs
 *  clock/timer/GPIO initialization first.  This hook is placed in the ELF
 *  `.preinit_array`, so the C runtime calls it before C++ constructors,
 *  before Arduino `init()`, before `initVariant()`, and before `setup()`.
 *
 *  The routine deliberately uses only raw STM32F1 registers:
 *    - no globals requiring initialization,
 *    - no Arduino API,
 *    - no heap/stack-heavy code,
 *    - no calls into drivers.
 *
 *  It clamps all actuator outputs to their electrically safe inactive state
 *  within the first few microseconds of C runtime execution.
 * ---------------------------------------------------------------------------*/
static inline void configureF1OutputLowEarly(GPIO_TypeDef* port, uint8_t pin) {
  const uint32_t bit = (1UL << pin);
  volatile uint32_t* const cr =
      (pin < 8U) ? &port->CRL : &port->CRH;
  const uint8_t shift = static_cast<uint8_t>((pin & 0x07U) * 4U);

  port->BRR = bit;
  *cr = (*cr & ~(0x0FUL << shift)) |
        (0x02UL << shift);               /* output push-pull, 2 MHz          */
  port->BRR = bit;
}

/* [REQ-DIAG-002] Stack-canary storage.
 *
 * The canary lives in its OWN .noinit slot (NOT at &__bss_end__, which is
 * shared with the heap and with `s_reset_record.magic` - placing it there
 * caused a write-conflict that was observed in flight-test logs to clobber
 * the breadcrumb magic and trigger a fake "canary corrupted" boot loop).
 *
 * Declared at file-scope `static volatile` with `used` so the linker keeps
 * the variable even though only the qualified primitives reference it.    */
static volatile uint32_t s_stack_canary
    __attribute__((section(".noinit"), used));

extern "C" void pduPreinitSafeOutputs(void) {
  /* [REQ-DIAG-002] First action: arm the stack canary.  Even before any
   * GPIO is touched, we want a known sentinel so a runaway constructor or
   * rogue init function corrupts a safe address (the dedicated .noinit
   * canary slot) instead of any live state.                              */
  s_stack_canary = 0xDEADBEEFUL;

  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN |
                  RCC_APB2ENR_IOPBEN |
                  RCC_APB2ENR_AFIOEN;
  (void)RCC->APB2ENR;

#if defined(STM32F1xx)
  /* PB4 is NJTRST after reset.  Free PB4 as GPIO immediately while keeping
   * SWD and SWO alive (SWJ_CFG = Full SWJ without NJTRST).                 */
  uint32_t mapr = AFIO->MAPR;
  mapr &= ~AFIO_MAPR_SWJ_CFG_Msk;
  mapr |= AFIO_MAPR_SWJ_CFG_NOJNTRST;
  AFIO->MAPR = mapr;
#endif

  configureF1OutputLowEarly(GPIOA, 6U);   /* PA6  Winch IN1                  */
  configureF1OutputLowEarly(GPIOA, 7U);   /* PA7  Winch IN2                  */
  configureF1OutputLowEarly(GPIOA, 8U);   /* PA8  Winch lock 2               */

  configureF1OutputLowEarly(GPIOB, 0U);   /* PB0  Winch IN3                  */
  configureF1OutputLowEarly(GPIOB, 1U);   /* PB1  Winch IN4                  */
  configureF1OutputLowEarly(GPIOB, 4U);   /* PB4  LED Bras                   */
  configureF1OutputLowEarly(GPIOB, 5U);   /* PB5  LED Avant                  */
  configureF1OutputLowEarly(GPIOB, 8U);   /* PB8  LED Arr                    */
  configureF1OutputLowEarly(GPIOB, 9U);   /* PB9  LED Extra                  */
  configureF1OutputLowEarly(GPIOB, 13U);  /* PB13 E-Stop VTX                 */
  configureF1OutputLowEarly(GPIOB, 15U);  /* PB15 Winch lock 1               */
}

using PreinitHook = void (*)();
__attribute__((section(".preinit_array"), used))
static PreinitHook const kPreinitSafeOutputsHook = pduPreinitSafeOutputs;

}  // namespace

using namespace pdu;

/* ===========================================================================
 *                       NAMED CONSTANTS (MISRA-C++ Rule 5-0-2)
 * ---------------------------------------------------------------------------
 *  Every literal that has a real-world meaning is named here so that
 *  reviewers can trace each bit, address and timing back to the relevant
 *  datasheet section instead of inferring it from the call site.
 *
 *  Bit positions are taken straight from the LM5066H1 PMBus map (datasheet
 *  SNVSAQ7) and from the LM5066H1 driver in lib/LM5066H1.
 * =========================================================================== */

/* ---- PMBus OPERATION (0x01) ------------------------------------------- */
static constexpr uint8_t kPmbusOperationOnMask = 0x80U;

/* ---- 7-bit I2C address scan window (PMBus reserved 0x00-0x02 / 0x78-0x7F) - */
static constexpr uint8_t kPmbusFirst7BitAddress = 0x03U;
static constexpr uint8_t kPmbusLast7BitAddress  = 0x77U;
static constexpr uint32_t kPmbusScanGapMs       = 2UL;

/* ---- LM5066H1 STATUS_WORD low-byte fault bit positions (PMBus 0x79) -- */
static constexpr uint8_t kStatusWordBitBusy        = 7U;
static constexpr uint8_t kStatusWordBitDeviceOff   = 6U;
static constexpr uint8_t kStatusWordBitVoutOvFault = 5U;
static constexpr uint8_t kStatusWordBitIoutOcFault = 4U;
static constexpr uint8_t kStatusWordBitVinUvFault  = 3U;
static constexpr uint8_t kStatusWordBitTempFault   = 2U;

/* ---- STATUS_INPUT bit positions (PMBus 0x7C) ------------------------ */
static constexpr uint8_t kStatusInputBitVinOvFault = 7U;
static constexpr uint8_t kStatusInputBitVinUvFault = 4U;
static constexpr uint8_t kStatusInputBitOcFault    = 2U;

/* ---- STATUS_CML bit positions (PMBus 0x7E) -------------------------- */
static constexpr uint8_t kStatusCmlBitInvCmd      = 7U;
static constexpr uint8_t kStatusCmlBitInvData     = 6U;
static constexpr uint8_t kStatusCmlBitInvPec      = 5U;
static constexpr uint8_t kStatusCmlBitMemoryFault = 4U;
static constexpr uint8_t kStatusCmlMaskRealFaults =
    static_cast<uint8_t>((1U << kStatusCmlBitInvCmd) |
                         (1U << kStatusCmlBitInvData) |
                         (1U << kStatusCmlBitInvPec) |
                         (1U << kStatusCmlBitMemoryFault));

/* ---- STATUS_MFR_SPECIFIC bit positions (PMBus 0x80) ---------------- */
static constexpr uint8_t kStatusMfrBitCbFault       = 7U;
static constexpr uint8_t kStatusMfrBitFetFail       = 6U;
static constexpr uint8_t kStatusMfrBitBbRamFull     = 3U;
static constexpr uint8_t kStatusMfrBitFetFaultGate2 = 2U;
static constexpr uint8_t kStatusMfrBitFetFaultGate1 = 1U;
static constexpr uint8_t kStatusMfrBitFetFaultDrain = 0U;

/* ---- STATUS_MFR_SPECIFIC_2 bit positions (PMBus 0xF3, 16-bit) ------ */
static constexpr uint8_t kStatusMfr2BitWatchdog   = 12U;
static constexpr uint8_t kStatusMfr2BitShortCirc  = 11U;
static constexpr uint8_t kStatusMfr2BitEnergyWarn = 9U;
static constexpr uint8_t kStatusMfr2BitVinTrans   = 8U;

/* ---- DIAGNOSTIC_WORD bit positions (PMBus 0xE1, 16-bit) ------------ */
static constexpr uint8_t kDiagBitTimerLatchedOff = 9U;
static constexpr uint8_t kDiagBitFetFail         = 8U;
static constexpr uint8_t kDiagBitVinUvFault      = 5U;
static constexpr uint8_t kDiagBitVinOvFault      = 4U;
static constexpr uint8_t kDiagBitIinOcFault      = 3U;
static constexpr uint8_t kDiagBitOverTempFault   = 2U;
static constexpr uint8_t kDiagBitCmlFault        = 1U;
static constexpr uint8_t kDiagBitCbFault         = 0U;

/* ---- Boot-time pacing (settle delays for level translators) -------- */
static constexpr uint32_t kRailEnableSequenceGapMs = 20UL;
static constexpr uint32_t kBootSettleMs            = 100UL;

/* ---- Bit-test convenience: returns true iff `bit` is set in `value`. */
static constexpr bool bitIsSet(uint16_t value, uint8_t bit) {
  return (value & (static_cast<uint16_t>(1U) << bit)) != 0U;
}
static constexpr bool bitIsSet(uint8_t value, uint8_t bit) {
  return (value & (static_cast<uint8_t>(1U) << bit)) != 0U;
}

/* ===========================================================================
 *               COMPILE-TIME INVARIANTS  (MISRA-C++ 1-0-2)
 * ---------------------------------------------------------------------------
 *  Every static_assert below catches a class of regression that would
 *  otherwise be invisible until ground tests fire.  All checks are zero
 *  cost at runtime; failure is a hard build error.
 * =========================================================================== */
static_assert(sizeof(uint8_t)  == 1U, "uint8_t  must be 8 bits");
static_assert(sizeof(uint16_t) == 2U, "uint16_t must be 16 bits");
static_assert(sizeof(uint32_t) == 4U, "uint32_t must be 32 bits");

static_assert(kPmbusFirst7BitAddress < kPmbusLast7BitAddress,
              "PMBus scan range must be ascending");
static_assert(kPmbusOperationOnMask == 0x80U,
              "PMBus 0x01 OPERATION ON bit must be 0x80");

static_assert(kStatusWordBitBusy == 7U, "STATUS_WORD BUSY at bit 7");
static_assert(kStatusWordBitDeviceOff == 6U, "STATUS_WORD DEVICE_OFF at bit 6");

static_assert(cfg::kProtectionSamplePeriod_ms <
              cfg::kBitContinuous_ms,
              "Protection cadence must be faster than CBIT cadence");
static_assert(cfg::kHeartbeat_ms >= 100UL,
              "Heartbeat must be operator-visible (>=100 ms)");
/* `kWatchdogMaxHealthyLoop_ms` is declared later in this file (it is in
 * the supervisor-state block).  The corresponding static_asserts that
 * cross-check it against the IWDG timeout live alongside that constant.  */

/* ===========================================================================
 *           STACK CANARY  [REQ-DIAG-002]   (MISRA-C++ 5-0-3)
 * ---------------------------------------------------------------------------
 *  The Cortex-M3 stack lives at the top of SRAM and grows downward.  This
 *  module places a 4-byte canary at the lowest address the stack could
 *  ever reach (the very top of .bss, address `__bss_end__`).  Any deep
 *  stack overflow that writes past the bottom of the stack will overwrite
 *  the canary; a periodic check during normal operation catches that
 *  corruption before the next protection cycle.
 *
 *  We only VERIFY the canary - we never repair it - because corruption is
 *  by definition unrecoverable: the only safe response is to let the IWDG
 *  reset the MCU.  The failure path therefore busy-spins until the
 *  watchdog fires (deterministic safe-state).
 * =========================================================================== */
static constexpr uint32_t kStackCanaryMagic = 0xDEADBEEFUL;

/* The canary backing storage `s_stack_canary` lives near the top of this
 * translation unit (declared with `__attribute__((section(".noinit")))`)
 * and is reachable from these helpers via internal linkage in the same
 * compilation unit.                                                       */
static inline void stackCanaryArm() {
  s_stack_canary = kStackCanaryMagic;
}

static inline bool stackCanaryIntact() {
  return s_stack_canary == kStackCanaryMagic;
}

/* ===========================================================================
 *           IMAGE-TEXT CRC32 SELF-TEST  [REQ-DIAG-003]
 * ---------------------------------------------------------------------------
 *  Computes a 32-bit CRC over the entire flashed code/rodata image at
 *  boot and emits the result on the SWO trace.  No reference value is
 *  hard-coded in this image; ground support equipment compares the
 *  printed CRC with the value computed off-line from firmware.bin.  A
 *  mismatch flags a flash bit-flip / corrupted programming.
 *
 *  Range: [0x08000000 .. _etext) where 0x08000000 is the F1 flash base
 *  and _etext is the end-of-code symbol provided by the linker script.
 *
 *  Polynomial: IEEE 802.3 (0xEDB88320 reflected) - same as zlib and the
 *  STM32 hardware CRC peripheral.
 * =========================================================================== */
static constexpr uint32_t kFlashBaseAddress = 0x08000000UL;
extern "C" uint32_t _etext;            /* provided by linker script */

static uint32_t crc32Update(uint32_t crc, uint8_t byte) {
  crc ^= byte;
  for (uint8_t bit = 0U; bit < 8U; ++bit) {
    const uint32_t mask =
        static_cast<uint32_t>(-static_cast<int32_t>(crc & 1U));
    crc = (crc >> 1) ^ (0xEDB88320UL & mask);
  }
  return crc;
}

static uint32_t computeImageTextCrc32() {
  const volatile uint8_t* p =
      reinterpret_cast<const volatile uint8_t*>(kFlashBaseAddress);
  const volatile uint8_t* const e =
      reinterpret_cast<const volatile uint8_t*>(&_etext);
  uint32_t crc = 0xFFFFFFFFUL;
  /* Loop bound: the .text region is at most 64 KB on this part, so this
   * loop is statically bounded by 64 K iterations.  At 72 MHz the worst-
   * case is < 25 ms which fits inside one IWDG window without kicking. */
  while (p < e) {
    crc = crc32Update(crc, *p);
    ++p;
  }
  return crc ^ 0xFFFFFFFFUL;
}

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

/* Watchdog headroom (cross-checked here because both operands are now in
 * scope): a healthy loop must finish in < half the IWDG timeout so a
 * single hang is detected within one IWDG window.                        */
static_assert(kWatchdogMaxHealthyLoop_ms < cfg::kIwdgTimeout_ms,
              "Loop budget must be strictly below IWDG timeout");
static_assert(kWatchdogMaxHealthyLoop_ms * 2U <= cfg::kIwdgTimeout_ms,
              "Loop budget should be <= IWDG/2 for fault detection margin");

/* ---------------------------------------------------------------------------
 *  Reset-survival breadcrumb [REQ-DIAG-001].
 *
 *  Placed in the SRAM .noinit section so its contents survive any reset that
 *  does not power-cycle VDD (watchdog, SW reset, brown-out, etc.).  A magic
 *  cookie distinguishes "cold boot, contents are garbage" from "warm reboot,
 *  contents are last-loop snapshot taken before the reset".  Used by ground
 *  support equipment to discriminate between a hung CPU (no field changes
 *  between RG polls) and a tight reset loop (boot_count climbs).
 * ---------------------------------------------------------------------------*/
struct ResetSurvivalRecord {
  uint32_t magic;
  uint32_t boot_count;
  uint32_t last_alive_ms;
  uint32_t last_loop_count;
  uint32_t last_setup_done_ms;
};
static constexpr uint32_t kResetSurvivalMagic = 0xA1B2C3D4UL;

static ResetSurvivalRecord s_reset_record __attribute__((section(".noinit")));

static uint32_t s_loop_count = 0U;

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

  /* [REQ-LED-001] Strict separation of concerns.
   *   - The supervisor owns rail protection only.
   *   - The lighting outputs (LED Bras / Avant / Arr / Extra) are external
   *     vehicle lighting, NOT console indicators.  Their state is owned by
   *     the host (RoboGuard) over the I2C2 API.
   * Therefore the supervisor MUST NOT drive autonomous LED patterns based
   * on its internal mode: every mode transition forces the channels OFF,
   * leaving them OFF until RG explicitly commands them.                   */
  (void)leds::setAll(0U);
  (void)leds::setPattern(leds::Pattern::kOff);
}

/** [REQ-PWR-200] De-energise every controlled load.  Runs both autonomous
 *  rail-disable (if `kHotswapApiOnly` is false) and the always-on winch /
 *  lock disable.  Each lower-level Status return is intentionally void-
 *  cast: the function MUST execute every step even if one fails so we
 *  cannot return early on the first error.  Failures are recorded in the
 *  fault log instead.                                                    */
static void disableAllRails() {
  if (!cfg::kHotswapApiOnly) {
    s_rail48.disable();
    s_rail24.disable();
    s_rail12.disable();
  }
  const Status sw = winch::sleep();
  if (sw != Status::kOk) {
    Serial.print(F("[SAFE] winch::sleep failed rc="));
    Serial.println(static_cast<int>(sw));
  }
  const Status sl = winch_lock::setAll(false);
  if (sl != Status::kOk) {
    Serial.print(F("[SAFE] winch_lock::setAll(false) failed rc="));
    Serial.println(static_cast<int>(sl));
  }
}

/** [REQ-PWR-100] Reset + energise every present rail using the documented
 *  power-up sequence: 12V, then 24V, then 48V.  Each step is separated by
 *  cfg::kRailEnableSequenceGap so the next rail sees a stable bus before
 *  its inrush.
 *
 *  Each rail is brought up via resetAndEnable(), which wipes every latched
 *  LM5066H1 fault (so stale UV/OV latches from the power-up VIN ramp do not
 *  survive the boot) and then commands OPERATION = ON.
 *
 *  This deliberately runs even when `kHotswapApiOnly` is true: the rails
 *  come up energised on a cold boot without waiting for a host OPERATION
 *  write.  Runtime ownership of OPERATION is unchanged - the host (RG) can
 *  still disable / re-enable any rail through the I2C2 API at any time, and
 *  tick() still never re-asserts OPERATION on its own.                     */
static void enableAllRails() {
  s_rail12.resetAndEnable();
  delay(kRailEnableSequenceGapMs);
  s_rail24.resetAndEnable();
  delay(kRailEnableSequenceGapMs);
  s_rail48.resetAndEnable();
}

#if defined(PDU_OUTPUT_TEST_ONLY)
static constexpr uint32_t kOutputTestRampMs   = 15000U;
static constexpr uint32_t kOutputTestUpdateMs = 20U;
static constexpr uint8_t  kOutputTestMaxPct   = 100U;

/* [TEMPORARY - OUTPUT TEST ONLY] Static-hold duty cycle for the four LED
 * channels.  When this constant is set to a value in 0..100, tickOutputTest()
 * bypasses the 0%->100%->0% sawtooth ramp and pins every channel at that
 * value instead.  Set back to 0xFFU to re-enable the ramp.
 *
 * The PWM carrier frequency (`cfg::kLedPwmFrequency_Hz` = 2 kHz) is owned
 * by the LED module - any value below `cfg::kLedKickThreshold_pct` (40 %)
 * automatically triggers the [REQ-LED-004] cold-start kick to 40 % for
 * 50 ms before settling at this duty.  No extra kick logic is needed here. */
static constexpr uint8_t  kOutputTestStaticPct = 3U;

static uint32_t s_output_test_start_ms = 0U;
static uint32_t s_output_test_last_ms  = 0U;
static uint8_t  s_output_test_last_pct = 0xFFU;

static void armOutputTestPwmChannels() {
  /* The STM32 Arduino PWM path configures timer/alternate-function state
   * lazily on the first non-zero analogWrite() per pin.  Prime all four
   * channels before the visible ramp starts so BRAS / AVANT / ARR / EXTRA
   * begin the first cycle together instead of one channel leading.          */
  (void)leds::setPattern(leds::Pattern::kSolid);
  (void)leds::setAll(1U);
  delay(150U);
  (void)leds::setAll(0U);
  delay(50U);
}

static void tickOutputTest(uint32_t now) {
  if ((now - s_output_test_last_ms) < kOutputTestUpdateMs) {
    return;
  }
  s_output_test_last_ms = now;

  uint8_t duty_pct;
  if (kOutputTestStaticPct <= 100U) {
    /* [TEMPORARY] Static-hold mode: keep the ramp state variables live so
     * a single edit of `kOutputTestStaticPct` back to 0xFFU is the only
     * change needed to revert to the original sawtooth ramp.              */
    (void)s_output_test_start_ms;
    (void)kOutputTestRampMs;
    (void)kOutputTestMaxPct;
    duty_pct = kOutputTestStaticPct;
  } else {
    const uint32_t cycle_ms = kOutputTestRampMs * 2U;
    const uint32_t phase_ms = (now - s_output_test_start_ms) % cycle_ms;
    const uint32_t ramp_ms =
        (phase_ms <= kOutputTestRampMs) ? phase_ms : (cycle_ms - phase_ms);
    duty_pct = static_cast<uint8_t>(
        (ramp_ms * kOutputTestMaxPct) / kOutputTestRampMs);
  }

  if (duty_pct != s_output_test_last_pct) {
    s_output_test_last_pct = duty_pct;
    (void)leds::setAll(duty_pct);
    Serial.print(F("[OUTPUT TEST] LEDs (BRAS/AVANT/ARR/EXTRA) PWM = "));
    Serial.print(duty_pct);
    Serial.println(F("%"));
  }
}

static void initOutputTest() {
  if (kOutputTestStaticPct <= 100U) {
    Serial.print(F("[OUTPUT TEST] *** TEMPORARY *** All 4 LEDs held STATIC at "));
    Serial.print(kOutputTestStaticPct);
    Serial.print(F("% @ "));
    Serial.print(cfg::kLedPwmFrequency_Hz);
    Serial.println(F("Hz (ramp disabled)"));
  } else {
    Serial.println(F("[OUTPUT TEST] All 4 LEDs simultaneously: 0%->100% in 15s, then 100%->0% in 15s"));
  }
  (void)winch::sleep();
  (void)winch_lock::setAll(false);
  armOutputTestPwmChannels();
  s_output_test_start_ms = millis();
  s_output_test_last_ms  = s_output_test_start_ms - kOutputTestUpdateMs;
  s_output_test_last_pct = 0xFFU;
  tickOutputTest(millis());
}
#endif

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

/* ===========================================================================
 *  LM5066H1 fault decoder.  Each status register has its own decoder
 *  function so every helper is short, single-purpose, and easy to review.
 *
 *  Only actual protection-event bits are printed.  Pure warning thresholds
 *  (VIN/IIN/PIN/OT warnings) and informational status bits (PGOOD, device-
 *  off, defaults-loaded, init-done, retry/power-cycle recovery, averaging
 *  done, etc.) are intentionally suppressed - those tell you what the chip
 *  is *doing*, not what is wrong.
 * =========================================================================== */

/** Tag printed before a live, currently-asserted condition. */
static void emitActive(const char* rail, const __FlashStringHelper* msg,
                       bool& any) {
  Serial.print(F("  " ANSI_BOLD_RED "[ACTIVE]" ANSI_RESET " "));
  Serial.print(rail);
  Serial.print(F(": "));
  Serial.println(msg);
  any = true;
}

/** Tag printed before a latched fault bit awaiting CLEAR_FAULTS.
 *
 *  `cleared` distinguishes two very different situations that the raw latch
 *  bit cannot:
 *    - cleared == false -> "[LATCH ]" (yellow): the bit is latched AND the
 *      underlying condition still appears present in live telemetry right
 *      now.  This is a real, current problem.
 *    - cleared == true  -> "[OLD   ]" (gray): the bit is latched but the
 *      condition is no longer present (e.g. a UV that occurred during the
 *      power-up ramp, or a startup watchdog on a rail that is now happily
 *      regulating).  Historical only - it will disappear on CLEAR_FAULTS.  */
static void emitLatched(const char* rail, const __FlashStringHelper* msg,
                        bool& any, bool cleared) {
  if (cleared) {
    Serial.print(F("  " ANSI_GRAY "[OLD   ]" ANSI_RESET " "));
  } else {
    Serial.print(F("  " ANSI_YELLOW "[LATCH ]" ANSI_RESET " "));
  }
  Serial.print(rail);
  Serial.print(F(": "));
  Serial.println(msg);
  any = true;
}

/** Per-rail VIN under/over-voltage envelope.  Returns 0 for an unknown rail
 *  so callers treat the threshold as "not configured".                     */
static void vinThresholds(Rail rail, double& uv, double& ov) {
  switch (rail) {
    case Rail::k48V: uv = cfg::kVin48V_UV_V; ov = cfg::kVin48V_OV_V; break;
    case Rail::k24V: uv = cfg::kVin24V_UV_V; ov = cfg::kVin24V_OV_V; break;
    case Rail::k12V: uv = cfg::kVin12V_UV_V; ov = cfg::kVin12V_OV_V; break;
    case Rail::kCount:
    default:         uv = 0.0;               ov = 0.0;               break;
  }
}

/** True when the rail is presently energised and the chip reports the output
 *  stage ON.  Latched protection bits (watchdog, CB, OC, FET, SCP, ...) that
 *  have no dedicated live telemetry signal are treated as historical ("OLD")
 *  whenever the rail is healthy now, since by definition the past trip has
 *  since recovered.                                                         */
static bool railHealthyNow(const RailTelemetry& tlm) {
  const bool op_cmd_on   = (tlm.operation_raw & kPmbusOperationOnMask) != 0U;
  const bool reports_off = bitIsSet(tlm.status_word, kStatusWordBitDeviceOff) ||
                           bitIsSet(tlm.diag_word, 6U /* DIAG DEVICE_OFF */);
  return tlm.present && op_cmd_on && !reports_off;
}

/** True when either temperature sensor currently reads at/above its trip
 *  threshold.  NaN / sentinel (no-diode) readings compare false, so an
 *  unpopulated die diode never looks like a live over-temperature.          */
static bool overTempNow(const RailTelemetry& tlm) {
  return (tlm.die_temp_c >= cfg::kOtFault_C) ||
         (tlm.ntc_temp_c >= cfg::kNtcTrip_C);
}

/** True when live VIN is below the rail's UV threshold right now.           */
static bool vinUnderVoltageNow(const RailTelemetry& tlm) {
  double uv = 0.0, ov = 0.0;
  vinThresholds(tlm.rail, uv, ov);
  return (uv > 0.0) && tlm.present && (tlm.vin_v < uv);
}

/** True when live VIN is above the rail's OV threshold right now.           */
static bool vinOverVoltageNow(const RailTelemetry& tlm) {
  double uv = 0.0, ov = 0.0;
  vinThresholds(tlm.rail, uv, ov);
  return (ov > 0.0) && tlm.present && (tlm.vin_v > ov);
}

/** Live cross-checks derived from present telemetry instead of stale
 *  status bits: detects "commanded ON but FET reports OFF" and "VIN
 *  outside its rail-specific UV/OV envelope".                            */
static void decodeLiveConditions(const RailTelemetry& tlm,
                                 const char* rail, bool& any) {
  const bool op_cmd_on        = (tlm.operation_raw & kPmbusOperationOnMask) != 0U;
  const bool chip_reports_off = bitIsSet(tlm.status_word, kStatusWordBitDeviceOff);
  if (tlm.present && op_cmd_on && chip_reports_off) {
    emitActive(rail,
        F("commanded ON right now, but LM5066H1 reports output/device OFF"),
        any);
  }

  double vin_uv = 0.0;
  double vin_ov = 0.0;
  vinThresholds(tlm.rail, vin_uv, vin_ov);
  if (tlm.present && (vin_uv > 0.0) && (tlm.vin_v < vin_uv)) {
    emitActive(rail, F("VIN is below UV threshold right now"), any);
  }
  if (tlm.present && (vin_ov > 0.0) && (tlm.vin_v > vin_ov)) {
    emitActive(rail, F("VIN is above OV threshold right now"), any);
  }
}

/** STATUS_WORD (PMBus 0x79, 16-bit).  Only the low-byte fault bits are
 *  surfaced here; the high-byte summary bits duplicate dedicated status
 *  registers that we already decode individually.                        */
static void decodeStatusWord(const RailTelemetry& tlm, const char* rail,
                             bool& any) {
  const uint16_t sw = tlm.status_word;
  const bool healthy = railHealthyNow(tlm);
  if (bitIsSet(sw, kStatusWordBitBusy)) {
    emitActive(rail,
        F("PMBus: device BUSY right now (unable to respond)"), any);
  }
  if (bitIsSet(sw, kStatusWordBitVoutOvFault)) {
    emitLatched(rail, F("PMBus: VOUT overvoltage fault bit set"), any, healthy);
  }
  if (bitIsSet(sw, kStatusWordBitIoutOcFault)) {
    emitLatched(rail, F("PMBus: IOUT overcurrent fault bit set"), any, healthy);
  }
  if (bitIsSet(sw, kStatusWordBitVinUvFault)) {
    emitLatched(rail, F("PMBus: VIN undervoltage fault bit set"), any,
                !vinUnderVoltageNow(tlm));
  }
  if (bitIsSet(sw, kStatusWordBitTempFault)) {
    emitLatched(rail, F("PMBus: temperature fault bit set"), any,
                !overTempNow(tlm));
  }
}

/** STATUS_INPUT (PMBus 0x7C, 8-bit). */
static void decodeStatusInput(const RailTelemetry& tlm, const char* rail,
                              bool& any) {
  const uint8_t sin = tlm.status_input;
  const bool healthy = railHealthyNow(tlm);
  if (bitIsSet(sin, kStatusInputBitVinOvFault)) {
    emitLatched(rail, F("INPUT: VIN overvoltage fault bit set"), any,
                !vinOverVoltageNow(tlm));
  }
  if (bitIsSet(sin, kStatusInputBitVinUvFault)) {
    emitLatched(rail, F("INPUT: VIN undervoltage fault bit set"), any,
                !vinUnderVoltageNow(tlm));
  }
  if (bitIsSet(sin, kStatusInputBitOcFault)) {
    emitLatched(rail, F("INPUT: IIN overcurrent fault bit set"), any, healthy);
  }
}

/** STATUS_CML (PMBus 0x7E, 8-bit).  Bit 1 (noneOfAbove) is informational
 *  and asserts on every unsupported PMBus access, so it is suppressed.   *
 *
 *  NOTE on bit 4 (`memoryFault`): on some LM5066H1 parts the on-chip NVM
 *  checksum is invalid from the factory.  When that is the case the chip
 *  re-asserts this bit on every power-up regardless of CLEAR_FAULTS,
 *  falls back to the hardware-default register set, then accepts our
 *  configureDevice() programming on top.  The rail is operational; this
 *  bit is therefore reported as a benign LATCH for traceability rather
 *  than as a flight-rail fault.                                          */
static void decodeStatusCml(const RailTelemetry& tlm, const char* rail,
                            bool& any) {
  const uint8_t scml = tlm.status_cml;
  /* A latched bus-comms glitch is historical once we are successfully
   * exchanging bytes with the part again (which we just did to read this
   * telemetry).  Treat the standard CML bits as OLD when the rail is up.   */
  const bool healthy = railHealthyNow(tlm);
  if (bitIsSet(scml, kStatusCmlBitInvCmd)) {
    emitLatched(rail, F("CML: invalid/unsupported PMBus command bit set"),
                any, healthy);
  }
  if (bitIsSet(scml, kStatusCmlBitInvData)) {
    emitLatched(rail, F("CML: invalid/unsupported PMBus data bit set"),
                any, healthy);
  }
  if (bitIsSet(scml, kStatusCmlBitInvPec)) {
    emitLatched(rail, F("CML: PEC failure bit set"), any, healthy);
  }
  if (bitIsSet(scml, kStatusCmlBitMemoryFault)) {
    /* memoryFault re-asserts every boot on a part with a bad NVM checksum
     * and is operationally benign, so it is never marked OLD.              */
    emitLatched(rail,
        F("CML: LM5066H1 NVM checksum mismatch (chip uses defaults; "
          "operationally benign)"), any, false);
  }
}

/** STATUS_MFR_SPECIFIC (PMBus 0x80, 8-bit). */
static void decodeStatusMfr(const RailTelemetry& tlm, const char* rail,
                            bool& any) {
  const uint8_t smfr = tlm.status_mfr_specific;
  const bool healthy = railHealthyNow(tlm);
  if (bitIsSet(smfr, kStatusMfrBitCbFault)) {
    emitLatched(rail, F("MFR: circuit breaker trip bit set"), any, healthy);
  }
  if (bitIsSet(smfr, kStatusMfrBitFetFail)) {
    emitLatched(rail, F("MFR: external MOSFET failure bit set"), any, healthy);
  }
  if (bitIsSet(smfr, kStatusMfrBitBbRamFull)) {
    emitActive(rail, F("MFR: black-box RAM full right now"), any);
  }
  if (bitIsSet(smfr, kStatusMfrBitFetFaultGate2)) {
    emitLatched(rail, F("MFR: FET fault on GATE2 bit set"), any, healthy);
  }
  if (bitIsSet(smfr, kStatusMfrBitFetFaultGate1)) {
    emitLatched(rail, F("MFR: FET fault on GATE1 bit set"), any, healthy);
  }
  if (bitIsSet(smfr, kStatusMfrBitFetFaultDrain)) {
    emitLatched(rail, F("MFR: FET drain sense fault bit set"), any, healthy);
  }
}

/** STATUS_MFR_SPECIFIC_2 (PMBus 0xF3, 16-bit). */
static void decodeStatusMfr2(const RailTelemetry& tlm, const char* rail,
                             bool& any) {
  const uint16_t smfr2 = tlm.status_mfr_specific2;
  const bool healthy = railHealthyNow(tlm);
  if (bitIsSet(smfr2, kStatusMfr2BitWatchdog)) {
    /* The startup watchdog only runs while GATE1 is coming up; a rail that
     * is regulating now has, by definition, finished startup, so a set
     * watchdog bit is a historical record of the power-up sequence.        */
    emitLatched(rail, F("MFR2: internal watchdog fault bit set"), any, healthy);
  }
  if (bitIsSet(smfr2, kStatusMfr2BitShortCirc)) {
    emitLatched(rail, F("MFR2: short-circuit fault bit set"), any, healthy);
  }
  if (bitIsSet(smfr2, kStatusMfr2BitEnergyWarn)) {
    emitLatched(rail,
        F("MFR2: energy accumulator overflow warning bit set"), any, healthy);
  }
  if (bitIsSet(smfr2, kStatusMfr2BitVinTrans)) {
    emitLatched(rail, F("MFR2: VIN transient excursion bit set"), any, healthy);
  }
}

/** DIAGNOSTIC_WORD (PMBus 0xE1, 16-bit). */
static void decodeDiagWord(const RailTelemetry& tlm, const char* rail,
                           bool& any) {
  const uint16_t diag = tlm.diag_word;
  const uint8_t  scml = tlm.status_cml;
  const bool healthy = railHealthyNow(tlm);
  if (bitIsSet(diag, kDiagBitTimerLatchedOff)) {
    emitLatched(rail, F("DIAG: timer latched OFF bit set"), any, healthy);
  }
  if (bitIsSet(diag, kDiagBitFetFail)) {
    emitLatched(rail, F("DIAG: external MOSFET failure bit set"), any, healthy);
  }
  if (bitIsSet(diag, kDiagBitVinUvFault)) {
    emitLatched(rail, F("DIAG: VIN undervoltage fault bit set"), any,
                !vinUnderVoltageNow(tlm));
  }
  if (bitIsSet(diag, kDiagBitVinOvFault)) {
    emitLatched(rail, F("DIAG: VIN overvoltage fault bit set"), any,
                !vinOverVoltageNow(tlm));
  }
  if (bitIsSet(diag, kDiagBitIinOcFault)) {
    emitLatched(rail,
        F("DIAG: IIN overcurrent / power-FET op fault bit set"), any, healthy);
  }
  if (bitIsSet(diag, kDiagBitOverTempFault)) {
    emitLatched(rail, F("DIAG: over-temperature fault bit set"), any,
                !overTempNow(tlm));
  }
  /* DIAG bit 1 mirrors STATUS_CML.  We suppress this echo in two cases
   * to avoid double-reporting:
   *   1. When the only set CML bit is the noneOfAbove noise bit (set by
   *      any unsupported PMBus access, very common during scans).
   *   2. When the only set CML bit is memoryFault (NVM checksum issue
   *      handled by `decodeStatusCml`); the DIAG echo would just repeat
   *      that line under a different name ("communication fault") which
   *      is misleading because it has nothing to do with bus comms.       */
  static constexpr uint8_t kCmlBitsExceptMemoryFault =
      static_cast<uint8_t>((1U << kStatusCmlBitInvCmd) |
                           (1U << kStatusCmlBitInvData) |
                           (1U << kStatusCmlBitInvPec));
  const bool real_cml_bits =
      (scml & kCmlBitsExceptMemoryFault) != 0U;
  if (bitIsSet(diag, kDiagBitCmlFault) && real_cml_bits) {
    emitLatched(rail, F("DIAG: CML communication fault bit set"), any, healthy);
  }
  if (bitIsSet(diag, kDiagBitCbFault)) {
    emitLatched(rail, F("DIAG: circuit breaker trip bit set"), any, healthy);
  }
}

/** Top-level fault decoder.  Each status register is decoded by a single
 *  named helper; this function only owns the per-rail header / footer
 *  output and the absent / OK summary lines.                              */
static void describeRailFaults(const RailTelemetry& tlm) {
  const char* const rail = railToString(tlm.rail);

  /* If the device never ACKed at boot scan, every status register read
   * returns zero, which the decoders would silently report as "no fault".
   * That is misleading - distinguish the absent case explicitly so the
   * operator can see the rail is physically disconnected.                */
  if (!tlm.present) {
    Serial.print(F("  " ANSI_BOLD_YELLOW "[ABSENT]" ANSI_RESET " "));
    Serial.print(rail);
    Serial.println(F(": LM5066H1 not detected (no ACK on PMBus)"));
    return;
  }

  bool any = false;
  decodeLiveConditions(tlm, rail, any);
  decodeStatusWord (tlm, rail, any);
  decodeStatusInput(tlm, rail, any);
  decodeStatusCml  (tlm, rail, any);
  decodeStatusMfr  (tlm, rail, any);
  decodeStatusMfr2 (tlm, rail, any);
  decodeDiagWord   (tlm, rail, any);

  if (!any) {
    Serial.print(F("  " ANSI_GREEN "[OK    ]" ANSI_RESET " "));
    Serial.print(rail);
    Serial.println(F(": no active fault"));
  }
}

/* ---------------------------------------------------------------------------
 *  Small column-formatting helpers used by the tabular telemetry layout.
 *
 *  printPadFloat: right-aligned float, width chars total, dtostrf() based.
 *  printPadInt  : right-aligned signed integer, snprintf %*ld based.
 *  printPadHex  : zero-padded uppercase hex, snprintf %0*lX based.
 *  printPadStr  : left-aligned C-string, padded with spaces up to width.
 * --------------------------------------------------------------------------*/
static void printPadFloat(double v, int8_t width, uint8_t prec) {
  char buf[16];
  dtostrf(v, width, prec, buf);
  Serial.print(buf);
}

static void printPadInt(long v, uint8_t width) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%*ld", static_cast<int>(width), v);
  Serial.print(buf);
}

static void printPadHex(uint32_t v, uint8_t width) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%0*lX", static_cast<int>(width),
           static_cast<unsigned long>(v));
  Serial.print(buf);
}

static void printPadStr(const char* s, uint8_t width) {
  /* MISRA-C++ Rule 6-5-3 (loop counter modified once per iteration): the
   * two loops are explicitly bounded by `width` (uint8_t, max 255) so the
   * total number of writes is statically bounded.  A NULL `s` is treated
   * as an empty string instead of dereferenced.                          */
  size_t n = 0U;
  if (s != nullptr) {
    while ((n < width) && (s[n] != '\0')) {
      Serial.write(static_cast<uint8_t>(s[n]));
      ++n;
    }
  }
  while (n < width) {
    Serial.write(' ');
    ++n;
  }
}

static const char* presenceStr(bool present) {
  return present ? "PRES" : "ABS ";
}

static const char* onOffStr(bool on) {
  return on ? "ON " : "OFF";
}

static const char* modeAnsi(SupervisorMode m) {
  if (m == SupervisorMode::kNominal)  return ANSI_BOLD_GREEN;
  if (m == SupervisorMode::kDegraded) return ANSI_BOLD_YELLOW;
  if (m == SupervisorMode::kFault)    return ANSI_BOLD_RED;
  if (m == SupervisorMode::kEStop)    return ANSI_BOLD_RED;
  return ANSI_BOLD_CYAN;  /* kBoot, kPbit */
}

/** Right-align an analog value under its column header.  When the rail
 *  is absent the cell is rendered as a single grey '-' (and the unit
 *  letter is replaced with a space) so absent rails read clearly without
 *  a misleading 0.00V / 0.0C.                                            */
static void printValueOrDash(bool present, double value,
                             int8_t width, uint8_t prec, char unit) {
  Serial.print(F("  "));
  if (!present) {
    for (int8_t i = 1; i < width; ++i) {
      Serial.write(' ');
    }
    Serial.print(F(ANSI_GRAY "-" ANSI_RESET));
    Serial.write(' ');
    return;
  }
  printPadFloat(value, width, prec);
  Serial.write(static_cast<uint8_t>(unit));
}

static void printTelemetryRowLive(const RailTelemetry& tlm) {
  const bool op_cmd_on =
      (tlm.operation_raw & kPmbusOperationOnMask) != 0U;
  const bool chip_reports_off =
      bitIsSet(tlm.status_word, kStatusWordBitDeviceOff);
  const bool fet_on           = tlm.present && op_cmd_on && !chip_reports_off;

  Serial.print(F(" " ANSI_BOLD_CYAN));
  printPadStr(railToString(tlm.rail), 4U);
  Serial.print(F(ANSI_RESET));

  /* PRES / ABS  ----------------------------------------------------------- */
  Serial.print(F(" "));
  Serial.print(F(tlm.present ? ANSI_GREEN : ANSI_GRAY));
  printPadStr(presenceStr(tlm.present), 4U);
  Serial.print(F(ANSI_RESET));

  /* CMD ON/OFF (commanded state, not abnormal by itself) ----------------- */
  Serial.print(F("  "));
  Serial.print(F(op_cmd_on ? ANSI_GREEN : ANSI_GRAY));
  printPadStr(onOffStr(op_cmd_on), 3U);
  Serial.print(F(ANSI_RESET));

  /* FET ON/OFF (actual MOSFET state).  If commanded ON but FET OFF, red. -- */
  Serial.print(F("  "));
  if (op_cmd_on && !fet_on) {
    Serial.print(F(ANSI_BOLD_RED));
  } else if (fet_on) {
    Serial.print(F(ANSI_GREEN));
  } else {
    Serial.print(F(ANSI_GRAY));
  }
  printPadStr(onOffStr(fet_on), 3U);
  Serial.print(F(ANSI_RESET));

  /* PG (power-good GPIO).  Only the 24V and 12V rails have the PG line
   * routed to a STM32 GPIO; the 48V rail's PG is not wired in this board
   * revision, so we print '-' instead of a misleading 0/1.                */
  const bool pg_wired = (tlm.rail != Rail::k48V);
  Serial.print(F("  "));
  if (!pg_wired) {
    Serial.print(F(ANSI_GRAY "-" ANSI_RESET));
  } else {
    Serial.print(tlm.pgood_pin ? F(ANSI_GREEN  "1" ANSI_RESET)
                               : F(ANSI_YELLOW "0" ANSI_RESET));
  }

  /* For absent rails, replace every analog field with a gray "-"
   * placeholder.  Showing 0.00V / 0.0C for a physically disconnected
   * device is misleading; absent rails should look obviously absent.    */
  printValueOrDash(tlm.present, tlm.vin_v,  6, 2, 'V');
  printValueOrDash(tlm.present, tlm.vout_v, 6, 2, 'V');
  printValueOrDash(tlm.present, tlm.iin_a,  6, 2, 'A');
  printValueOrDash(tlm.present, tlm.pin_w,  6, 1, 'W');

  Serial.print(F("  "));
  if (!tlm.present) {
    Serial.print(F("     " ANSI_GRAY "-" ANSI_RESET " "));
  } else if (tlm.peak_valid) {
    printPadFloat(tlm.peak_pin_w, 6, 1);
    Serial.print(F("W"));
  } else {
    Serial.print(F("    n/a"));
  }

  printValueOrDash(tlm.present, tlm.die_temp_c, 6, 1, 'C');
  printValueOrDash(tlm.present, tlm.ntc_temp_c, 6, 1, 'C');

  /* Fault counter: red if any history, default if zero. ------------------ */
  Serial.print(F("  "));
  if (tlm.fault_count > 0U) {
    Serial.print(F(ANSI_BOLD_RED));
  } else {
    Serial.print(F(ANSI_GREEN));
  }
  printPadInt(static_cast<long>(tlm.fault_count), 3U);
  Serial.print(F(ANSI_RESET));
  Serial.println();
}

static void printTelemetryRowRaw(const RailTelemetry& tlm) {
  Serial.print(F(" "));
  printPadStr(railToString(tlm.rail), 4U);
  Serial.print(F(" 0x"));
  printPadHex(tlm.operation_raw, 2U);
  Serial.print(F("   0x"));
  printPadHex(tlm.status_word, 4U);
  Serial.print(F("   0x"));
  printPadHex(tlm.diag_word, 4U);
  Serial.print(F("   0x"));
  printPadHex(tlm.status_mfr_specific2, 4U);
  Serial.print(F("    0x"));
  printPadHex(tlm.wd_plb_timer, 2U);
  Serial.print(F("    0x"));
  printPadHex(tlm.status_input, 2U);
  Serial.print(F("   0x"));
  printPadHex(tlm.status_cml, 2U);
  Serial.print(F("   0x"));
  printPadHex(tlm.status_mfr_specific, 2U);
  Serial.print(F("    "));
  Serial.print(tlm.bb_valid ? '1' : '0');
  Serial.print(F("/"));
  Serial.print(tlm.bb_ram_len);
  Serial.print(F("/"));
  Serial.println(tlm.bb_eeprom_len);
}

static void publishTelemetry() {
  RailTelemetry tlm = {};
  rail::Controller* rails[kRailCount] = {&s_rail48, &s_rail24, &s_rail12};

  static const char kSep[] =
      "==============================================================================================";
  static const char kSub[] =
      "----------------------------------------------------------------------------------------------";

  Serial.println();
  Serial.print(F(ANSI_GRAY));
  Serial.print(kSep);
  Serial.println(F(ANSI_RESET));

  Serial.print(F(" mode="));
  Serial.print(modeAnsi(s_mode));
  Serial.print(modeToString(s_mode));
  Serial.print(F(ANSI_RESET));
  Serial.print(F("    estop="));
  if (estop::isEStopActive()) {
    Serial.print(F(ANSI_BOLD_RED "YES" ANSI_RESET));
  } else {
    Serial.print(F(ANSI_GREEN "no " ANSI_RESET));
  }
  Serial.print(F("    uptime="));
  Serial.print(millis() / 1000UL);
  Serial.println(F("s"));

  Serial.print(F(ANSI_GRAY));
  Serial.print(kSep);
  Serial.println(F(ANSI_RESET));

  /* ----- Live values table ------------------------------------------- */
  Serial.println(F(ANSI_BOLD_CYAN
      " RAIL PRES   CMD  FET  PG     VIN     VOUT     IIN      PIN      Ppk    Tdie     Tntc    F"
      ANSI_RESET));
  Serial.print(F(ANSI_GRAY));
  Serial.print(kSub);
  Serial.println(F(ANSI_RESET));
  for (size_t i = 0U; i < kRailCount; ++i) {
    rails[i]->buildTelemetry(tlm);
    printTelemetryRowLive(tlm);
  }

  /* ----- Raw register table ------------------------------------------ */
  Serial.println();
  Serial.println(F(ANSI_BOLD_CYAN
      " RAIL   OP      SW      DIAG     MFR2    WDPLB     SIN    SCML    SMFR     BB"
      ANSI_RESET));
  Serial.print(F(ANSI_GRAY));
  Serial.print(kSub);
  Serial.println(F(ANSI_RESET));
  for (size_t i = 0U; i < kRailCount; ++i) {
    rails[i]->buildTelemetry(tlm);
    printTelemetryRowRaw(tlm);
  }

  /* ----- Fault decoder ----------------------------------------------- */
  Serial.println();
  Serial.println(F(ANSI_BOLD_CYAN " FAULTS:" ANSI_RESET));
  for (size_t i = 0U; i < kRailCount; ++i) {
    rails[i]->buildTelemetry(tlm);
    describeRailFaults(tlm);
  }

  /* ----- Winch + locks ----------------------------------------------- */
  Serial.println();
  const winch::Telemetry w = winch::telemetry();
  Serial.print(F(ANSI_BOLD_CYAN " WINCH:" ANSI_RESET));
  Serial.print(F("  mode="));
  Serial.print(static_cast<uint8_t>(w.mode));
  Serial.print(F("  awake="));
  Serial.print(w.awake ? F(ANSI_GREEN "1" ANSI_RESET)
                       : F(ANSI_GRAY  "0" ANSI_RESET));
  Serial.print(F("  fault="));
  Serial.print(w.fault_active ? F(ANSI_BOLD_RED "1" ANSI_RESET)
                              : F(ANSI_GREEN    "0" ANSI_RESET));
  Serial.print(F("  dcA="));
  Serial.print(w.motor_a_cmd_pct);
  Serial.print(F("%  dcB="));
  Serial.print(w.motor_b_cmd_pct);
  Serial.print(F("%  par="));
  Serial.print(w.parallel_cmd_pct);
  Serial.print(F("%  stpA="));
  Serial.print(w.stepper_a_cmd_pct);
  Serial.print(F("%  stpB="));
  Serial.print(w.stepper_b_cmd_pct);
  Serial.println('%');

  const winch_lock::Telemetry wl = winch_lock::telemetry();
  Serial.print(F(ANSI_BOLD_CYAN " LOCKS:" ANSI_RESET));
  Serial.print(F("  lock1="));
  Serial.print(wl.lock1_on ? F(ANSI_GREEN "1" ANSI_RESET)
                           : F(ANSI_GRAY  "0" ANSI_RESET));
  Serial.print(F("  lock2="));
  Serial.println(wl.lock2_on ? F(ANSI_GREEN "1" ANSI_RESET)
                             : F(ANSI_GRAY  "0" ANSI_RESET));

  Serial.print(F(ANSI_GRAY));
  Serial.print(kSep);
  Serial.println(F(ANSI_RESET));
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
  for (uint8_t address = kPmbusFirst7BitAddress;
       address <= kPmbusLast7BitAddress; ++address) {
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
    delay(kPmbusScanGapMs);
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

#if defined(PDU_VERBOSE_DEBUG)
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
#endif

static void publishPeriodicReadout(uint32_t now) {
  if ((now - s_last_telemetry_ms) < cfg::kTelemetry_ms) {
    return;
  }
  s_last_telemetry_ms = now;

#if defined(PDU_VERBOSE_DEBUG)
  /* Verbose probe lines (one per rail per tick).  Off by default; the same
   * voltages and currents are already shown in the colored telemetry table. */
  readKnownSmbusDevices();
#endif
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
    const rail::State st = states[i];
    if (st == rail::State::kLatched) {
      any_latched = true;
    }
    if (st == rail::State::kTripped) {
      any_tripped = true;
    }
    if ((st == rail::State::kRunning) || (st == rail::State::kWarning)) {
      any_running = true;
    }
  }
  if (any_latched) {
    return SupervisorMode::kFault;
  }
  if (any_tripped) {
    return SupervisorMode::kDegraded;
  }
  if (any_running) {
    return SupervisorMode::kNominal;
  }
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

/* ===========================================================================
 *                       PHASED INITIALISATION SEQUENCE
 * ---------------------------------------------------------------------------
 *  The flight image follows a deterministic, single-threaded boot.  Every
 *  phase has a specific responsibility, a single point of return, and is
 *  documented with the [REQ-BOOT-NN] tag tying it to a system requirement.
 *
 *  No phase blocks longer than the Independent Watchdog timeout
 *  ( cfg::kIwdgTimeout_ms ).  Each phase that may exceed half of that
 *  budget calls iwdg::kick() at its end.  This produces a watchdog timing
 *  proof identical in shape to the F-35 SDD-3.5 boot trace requirement.
 * =========================================================================== */

/** [REQ-BOOT-001] Bring the console online.  Must run before any subsystem
 *  that emits diagnostics.  On STM32F1 this also reconfigures SWJ-DP via
 *  SerialSWO::enableTrace() so PB4 is freed for GPIO use.                  */
static void phaseBootConsole() {
  Serial.begin(cfg::kSerialBaud);
}

/** [REQ-BOOT-002] Initialise the diagnostic subsystem (fault log + IWDG)
 *  and capture the cause of the previous reset before any other code can
 *  alter it.  Latency-budget: < 5 ms.                                      */
static void phaseBootDiagnostics() {
  fault_log::init();
  iwdg::init();
  s_boot_watchdog_reset = iwdg::wasResetByWatchdog();
  iwdg::kick();
}

/** [REQ-DIAG-001 / REQ-DIAG-002 / REQ-DIAG-003]
 *  Print a single concise line summarising the reset cause, the previous-
 *  life breadcrumb, the stack-canary state, and the image-text CRC32.
 *  All four diagnostics live on one line so SWO truncation never hides a
 *  field.                                                                 */
static void phaseBootResetBreadcrumb() {
  const bool warm_boot =
      (s_reset_record.magic == kResetSurvivalMagic);
  const uint32_t prev_boot_count =
      warm_boot ? s_reset_record.boot_count : 0U;
  const uint32_t prev_alive_ms =
      warm_boot ? s_reset_record.last_alive_ms : 0U;
  const uint32_t prev_loop_count =
      warm_boot ? s_reset_record.last_loop_count : 0U;

  /* Verify integrity BEFORE clobbering the canary on the next iteration
   * by writing into .bss.  Compute CRC32 once at boot for ground-truth
   * comparison against the off-line value.                              */
  const bool     canary_ok = stackCanaryIntact();
  const uint32_t image_crc = computeImageTextCrc32();

  s_reset_record.magic              = kResetSurvivalMagic;
  s_reset_record.boot_count         = prev_boot_count + 1U;
  s_reset_record.last_alive_ms      = 0U;
  s_reset_record.last_loop_count    = 0U;
  s_reset_record.last_setup_done_ms = 0U;
  s_loop_count                      = 0U;

  /* Re-arm the canary in case `s_reset_record` writes happened to land
   * adjacent to it.  The arming write is idempotent and bounded.         */
  stackCanaryArm();

  Serial.print(F("[BOOT] reset_cause=0x"));
  Serial.print(fault_log::bootResetFlags(), HEX);
  Serial.print(F(" warm="));
  Serial.print(warm_boot ? 1 : 0);
  Serial.print(F(" wdg="));
  Serial.print(s_boot_watchdog_reset ? 1 : 0);
  Serial.print(F(" boot_count="));
  Serial.print(s_reset_record.boot_count);
  Serial.print(F(" canary="));
  Serial.print(canary_ok ? F("OK") : F("BROKEN"));
  Serial.print(F(" image_crc=0x"));
  Serial.print(image_crc, HEX);
  Serial.print(F(" prev_alive_ms="));
  Serial.print(prev_alive_ms);
  Serial.print(F(" prev_loop_count="));
  Serial.println(prev_loop_count);

  if (s_boot_watchdog_reset) {
    fault_log::record(fault_log::Code::kWatchdogReset, Rail::kCount, 0U, 0U);
  }
}

/** [REQ-BOOT-003] Bring up the actuator drivers.  All channels are forced
 *  to the inactive state, identical to the .preinit_array clamp, so that
 *  any previously-stored framework state cannot energise a load.
 *
 *  Every Status return is checked and a failure is reported on SWO
 *  immediately so a broken init never hides behind a (void) cast.        */
static void phaseBootActuators() {
  pinMode(cfg::kPin_PB14_Unused, INPUT);

  const Status leds_rc = leds::init();
  if (leds_rc != Status::kOk) {
    Serial.print(F("[INIT] leds::init failed rc="));
    Serial.println(static_cast<int>(leds_rc));
  }

  const Status winch_rc = winch::init();
  if (winch_rc != Status::kOk) {
    Serial.print(F("[INIT] winch::init failed rc="));
    Serial.println(static_cast<int>(winch_rc));
  }

  const Status lock_rc = winch_lock::init();
  if (lock_rc != Status::kOk) {
    Serial.print(F("[INIT] winch_lock::init failed rc="));
    Serial.println(static_cast<int>(lock_rc));
  }
}

/** Print the firmware identification banner.  No flight logic in this
 *  phase: pure operator information.                                      */
static void phaseBootBanner() {
  Serial.println();
  Serial.println(F("==================================================="));
  Serial.print  (F("  PDU Firmware v"));
  Serial.print  (cfg::kFwVersionMajor); Serial.print('.');
  Serial.print  (cfg::kFwVersionMinor); Serial.print('.');
  Serial.println(cfg::kFwVersionPatch);
  Serial.println(F("  Target : STM32F103C8T6 (Blue Pill)"));
  Serial.println(F("  Devices: 3 x LM5066H1 (48V / 24V / 12V)"));
  Serial.println(F("==================================================="));
}

/** [REQ-BOOT-010] Bring up the hot-swap I2C/SMBus buses, scan present
 *  devices, and configure each LM5066H1.  This is the longest phase, so
 *  the watchdog is petted between rail probes.                            */
static Status phaseBootHotswapBuses() {
  Wire.setSCL(cfg::kPin_I2C1_SCL);
  Wire.setSDA(cfg::kPin_I2C1_SDA);
  Wire.begin();
  Wire.setClock(cfg::kI2c1HotswapClock_Hz);
  s_smbus48.begin();
  s_smbus48.setClock(cfg::kSmbus48Clock_Hz);

  scanSmbusAddresses();
  scan48VSmbusAddress();
  iwdg::kick();

  enterMode(SupervisorMode::kPbit);

  s_rail48.init();
  iwdg::kick();
  s_rail24.init();
  iwdg::kick();
  s_rail12.init();
  iwdg::kick();
  return Status::kOk;
}

/** [REQ-BOOT-020] Bring up the external host I2C2 slave (RoboGuard API).
 *  After this call the supervisor accepts host commands at any time.      */
static void phaseBootControlApi() {
  control_api::init();
  Serial.print(F("[API] I2C2 slave addr=0x"));
  Serial.print(cfg::kApiI2cAddress, HEX);
  Serial.println(F(" pins SCL=PB10 SDA=PB11"));
}

/** [REQ-BOOT-030] Run the Power-On Built-In Test for telemetry/logging.
 *  PBIT is informational: it never gates the boot-time rail enable, in
 *  line with [REQ-PWR-100] (a single missing rail must not deny the rest
 *  of the system).                                                        */
static Status phaseBootRunPbit() {
  const Status pbit =
      bit::runPbit(s_rail48, s_rail24, s_rail12, s_pbit_report);
  iwdg::kick();
  return pbit;
}

/** [REQ-BOOT-040] Apply the boot-time rail policy.  When the supervisor
 *  is configured for autonomous rail control, this energises every
 *  present rail.  When `kHotswapApiOnly` is true, RG owns the OPERATION
 *  state and this call simply logs the policy.                            */
static void phaseBootApplyRailPolicy(Status pbit_status) {
  enableAllRails();
  if (pbit_status != Status::kOk) {
    Serial.println(F("[BOOT] PBIT failed - continuing with present rails enabled"));
  }
  enterMode(classifyMode());
}

/** [REQ-BOOT-050] Initialise the rate-monotonic super-loop scheduler. */
static void phaseBootSchedulerArm() {
  const uint32_t now_ms = millis();
  s_last_protect_ms                 = now_ms;
  s_last_telemetry_ms               = now_ms;
  s_last_cbit_ms                    = now_ms;
  s_last_heartbeat_ms               = now_ms;
  s_reset_record.last_setup_done_ms = now_ms;
}

/* ---------------------------------------------------------------------------
 *  Boot-trace helpers.  Each phase invocation is wrapped so the elapsed
 *  time is logged on a single SWO line, producing the SDD-3.5 timing trace
 *  that ground V&V uses to verify each phase respects its time budget.
 * ---------------------------------------------------------------------------*/
typedef void (*BootPhaseFn)();
typedef Status (*BootPhaseFnStatus)();

static void runTimedPhase(const __FlashStringHelper* tag, BootPhaseFn fn) {
  const uint32_t t0 = millis();
  fn();
  const uint32_t t1 = millis();
  Serial.print(F("[BOOT-PHASE] "));
  Serial.print(tag);
  Serial.print(F(" elapsed_ms="));
  Serial.println(t1 - t0);
}

static Status runTimedPhaseStatus(const __FlashStringHelper* tag,
                                  BootPhaseFnStatus fn) {
  const uint32_t t0 = millis();
  const Status rc = fn();
  const uint32_t t1 = millis();
  Serial.print(F("[BOOT-PHASE] "));
  Serial.print(tag);
  Serial.print(F(" elapsed_ms="));
  Serial.print(t1 - t0);
  Serial.print(F(" rc="));
  Serial.println(static_cast<int>(rc));
  return rc;
}

/* ===========================================================================
 *                         RATE-MONOTONIC SUPER-LOOP
 * ---------------------------------------------------------------------------
 *  The flight loop is a single deterministic super-loop running these tasks
 *  at their documented cadences.  The order is fixed; the scheduler only
 *  decides whether each task fires this iteration.
 *
 *      Task                  Period (ms)         Priority   REQ
 *      --------------------- ------------------- ---------- -------------
 *      E-Stop poll           every iteration     Highest    [REQ-LOOP-001]
 *      LED tick              every iteration     -          [REQ-LOOP-002]
 *      Host API tick         every iteration     -          [REQ-LOOP-003]
 *      Periodic readout      Telemetry_ms        Low        [REQ-LOOP-004]
 *      Rail protect          ProtectionSample_ms High       [REQ-LOOP-010]
 *      CBIT                  BitContinuous_ms    Medium     [REQ-LOOP-020]
 *      Heartbeat             Heartbeat_ms        Lowest     [REQ-LOOP-030]
 *      Watchdog kick         every iteration     Mandatory  [REQ-LOOP-099]
 * =========================================================================== */

/** [REQ-LOOP-001] Highest-priority task.  When the E-Stop is asserted the
 *  rails are immediately de-energised and the supervisor latches into
 *  kEStop until the operator releases the loop AND the protection cycle
 *  re-classifies.  Returns true if the caller must short-circuit the
 *  remainder of the iteration (typical for safety-critical events).      */
static bool loopTaskEStop() {
  if (estop::isEStopActive()) {
    if (s_mode != SupervisorMode::kEStop) {
      disableAllRails();
      enterMode(SupervisorMode::kEStop);
    }
    return true;
  }
  if (s_mode == SupervisorMode::kEStop) {
    /* E-Stop just released; require operator to re-arm via the host API. */
    enterMode(SupervisorMode::kDegraded);
  }
  return false;
}

/** [REQ-LOOP-010] Periodic rail protection task.  Runs at
 *  cfg::kProtectionSamplePeriod_ms.
 *
 *  Telemetry sampling MUST run even when the supervisor is in kEStop
 *  (otherwise the cached VIN/IIN/temperature stay at zero and the FAULT
 *  decoder reports a phantom "VIN below UV" - which was observed in the
 *  field after the E-Stop loop locked the supervisor in kEStop with rail
 *  protection compiled out).  Therefore this task is allowed to run
 *  during E-Stop, but it MUST NOT transition the supervisor into or out
 *  of kEStop / kFault: those transitions are owned by `loopTaskEStop()`
 *  and `tripFromProtection()` respectively.                              */
static void loopTaskRailProtection(uint32_t now_ms) {
  if ((now_ms - s_last_protect_ms) < cfg::kProtectionSamplePeriod_ms) {
    return;
  }
  s_last_protect_ms = now_ms;
  s_rail48.tick();
  s_rail24.tick();
  s_rail12.tick();

  /* Reclassify only between non-EStop, non-Fault modes.  EStop entry/exit
   * is owned by loopTaskEStop(); kFault is sticky.                        */
  if ((s_mode == SupervisorMode::kEStop) ||
      (s_mode == SupervisorMode::kFault)) {
    return;
  }
  const SupervisorMode m = classifyMode();
  if (m == SupervisorMode::kEStop) {
    return;  /* kEStop transition belongs to loopTaskEStop. */
  }
  if (m != s_mode) {
    enterMode(m);
  }
}

/** [REQ-LOOP-020] Continuous Built-In Test.  Runs at
 *  cfg::kBitContinuous_ms.                                                */
static void loopTaskCbit(uint32_t now_ms) {
  if ((now_ms - s_last_cbit_ms) < cfg::kBitContinuous_ms) {
    return;
  }
  s_last_cbit_ms = now_ms;
  bit::runCbit(s_rail48, s_rail24, s_rail12, s_cbit_report);
  if (!s_cbit_report.all_passed) {
    Serial.println(F("[CBIT] one or more rails failed continuous test"));
    enterMode(SupervisorMode::kDegraded);
  }
}

/** [REQ-LOOP-030] Operator heartbeat slot.  Currently a no-op because the
 *  LED pattern already encodes the supervisor state visually; kept as an
 *  explicit slot so the scheduling table remains complete and any future
 *  heartbeat work has a documented home.                                  */
static void loopTaskHeartbeat(uint32_t now_ms) {
  if ((now_ms - s_last_heartbeat_ms) < cfg::kHeartbeat_ms) {
    return;
  }
  s_last_heartbeat_ms = now_ms;
}

/* ---------------------------------------------------------------------------
 *  Arduino entry points
 *
 *  setup() is the certified phased boot.  loop() is the rate-monotonic
 *  super-loop.  Every special-purpose build (PDU_API_DEBUG_ONLY,
 *  PDU_OUTPUT_TEST_ONLY) takes a controlled, documented short-circuit.
 * ---------------------------------------------------------------------------*/
void setup() {
  /* ----- Phase 0 : Hardware fail-safe -----------------------------------
   * Already executed pre-main via the .preinit_array hook
   * (pduPreinitSafeOutputs).  This call is the in-`setup` belt-and-braces
   * pass that re-asserts the same GPIO state in case the Arduino core
   * altered any of those pins during initVariant().                        */
  forceKnownOutputsToSafeStateAtBoot();
  estop::init();

  /* ----- Phase 1..3 : console, diagnostics, breadcrumb ------------------ */
  runTimedPhase(F("01-console"),     phaseBootConsole);
  runTimedPhase(F("02-diagnostics"), phaseBootDiagnostics);
  runTimedPhase(F("03-breadcrumb"),  phaseBootResetBreadcrumb);

#if defined(PDU_API_DEBUG_ONLY)
  /* API debug image: only the host I2C2 slave is brought up.  All rail
   * scans and LM5066H1 configuration are skipped so an empty SWO trace
   * unambiguously means "no API traffic", not "stuck before API init".   */
  (void)control_api::init();
  return;
#endif

  /* ----- Phase 4 : actuator drivers ------------------------------------- */
  runTimedPhase(F("04-actuators"), phaseBootActuators);

  /* Bounded settle delay that gives the level translators time to ride
   * the rising edges induced by the GPIO mode flips above before any
   * bus traffic starts.  Order-of-magnitude only; not a timing dependency. */
  delay(kBootSettleMs);
  iwdg::kick();
  runTimedPhase(F("05-banner"), phaseBootBanner);

#if defined(PDU_OUTPUT_TEST_ONLY)
  initOutputTest();
  s_reset_record.last_setup_done_ms = millis();
  return;
#endif

  /* ----- Phase 5..9 : flight bring-up ----------------------------------- */
  (void)runTimedPhaseStatus(F("06-hotswap-bus"), phaseBootHotswapBuses);
  runTimedPhase            (F("07-control-api"), phaseBootControlApi);

  const Status pbit_status =
      runTimedPhaseStatus(F("08-pbit"), phaseBootRunPbit);
  /* Phase 09 has a pbit-status argument so it cannot use the generic
   * timed wrapper; we time it inline.                                    */
  {
    const uint32_t t0 = millis();
    phaseBootApplyRailPolicy(pbit_status);
    const uint32_t t1 = millis();
    Serial.print(F("[BOOT-PHASE] 09-rail-policy elapsed_ms="));
    Serial.println(t1 - t0);
  }

  runTimedPhase(F("10-scheduler-arm"), phaseBootSchedulerArm);
  Serial.print(F("[BOOT-DONE] total_ms="));
  Serial.println(millis());
}

void loop() {
  const uint32_t loop_started_ms = millis();
  const uint32_t now             = loop_started_ms;

  /* [REQ-DIAG-002] Stack-canary integrity check every iteration.
   * Cost: one 32-bit load and compare.  If the canary has been clobbered,
   * memory corruption has already occurred so further execution is
   * unsafe; we busy-spin until the IWDG fires the hardware reset.  This
   * is the deterministic safe-state per [REQ-LOOP-099].                  */
  if (!stackCanaryIntact()) {
    Serial.println(F("[FATAL] stack canary corrupted - awaiting IWDG reset"));
    for (;;) {
      /* Intentionally empty: do NOT kick the watchdog.                   */
    }
  }

  /* [REQ-DIAG-001] Reset-survival breadcrumb update; ~10 cycles. */
  ++s_loop_count;
  s_reset_record.last_alive_ms   = loop_started_ms;
  s_reset_record.last_loop_count = s_loop_count;

#if defined(PDU_VERBOSE_DEBUG)
  /* Verbose-only loop instrumentation.  Compiled out in flight images so
   * the foreground loop emits zero per-iteration log traffic.            */
  {
    static uint32_t s_dbg_hb_last_ms   = 0U;
    static uint32_t s_dbg_hb_count     = 0U;
    static uint32_t s_dbg_pa0_last_odr = 0xFFFFFFFFU;
    static uint32_t s_dbg_pa0_last_idr = 0xFFFFFFFFU;
    static uint32_t s_dbg_pa0_last_crl = 0xFFFFFFFFU;

    const uint32_t odr_now = (GPIOA->ODR >> 0U) & 1U;
    const uint32_t idr_now = (GPIOA->IDR >> 0U) & 1U;
    const uint32_t crl_now = GPIOA->CRL & 0xFU;
    if ((odr_now != s_dbg_pa0_last_odr) ||
        (idr_now != s_dbg_pa0_last_idr) ||
        (crl_now != s_dbg_pa0_last_crl)) {
      Serial.print(F("[PA0 DELTA] ODR.0="));
      Serial.print(odr_now);
      Serial.print(F(" IDR.0="));
      Serial.print(idr_now);
      Serial.print(F(" CRL[3:0]=0x"));
      Serial.println(crl_now, HEX);
      s_dbg_pa0_last_odr = odr_now;
      s_dbg_pa0_last_idr = idr_now;
      s_dbg_pa0_last_crl = crl_now;
    }
    if ((now - s_dbg_hb_last_ms) >= 1000U) {
      s_dbg_hb_last_ms = now;
      ++s_dbg_hb_count;
      Serial.print(F("[LOOP HB] n="));
      Serial.print(s_dbg_hb_count);
      Serial.print(F(" mode="));
      Serial.print(modeToString(s_mode));
      Serial.print(F(" estop_active="));
      Serial.println(estop::isEStopActive() ? 1 : 0);
    }
  }
#endif

#if defined(PDU_API_DEBUG_ONLY)
  control_api::tick(s_rail48, s_rail24, s_rail12, s_mode,
                    s_pbit_report, s_cbit_report);
  serviceWatchdogIfHealthy(loop_started_ms);
  return;
#endif

#if defined(PDU_OUTPUT_TEST_ONLY)
  tickOutputTest(now);
  serviceWatchdogIfHealthy(loop_started_ms);
  return;
#endif

  /* ----- Mandatory per-iteration tasks --------------------------------- */
  estop::tick();
  leds::tick();
  control_api::tick(s_rail48, s_rail24, s_rail12, s_mode,
                    s_pbit_report, s_cbit_report);

  /* Sample rail telemetry BEFORE publishing AND before the E-Stop
   * short-circuit, so cached VIN/IIN/temperature stay current even while
   * the supervisor is latched in kEStop.  loopTaskRailProtection() is
   * E-Stop-aware: it never transitions the supervisor into / out of
   * kEStop, so the safety state machine is unchanged.                    */
  loopTaskRailProtection(now);
  publishPeriodicReadout(now);

  /* ----- [REQ-LOOP-001] Highest-priority safety task ------------------ */
  if (loopTaskEStop()) {
    serviceWatchdogIfHealthy(loop_started_ms);
    return;
  }

  /* ----- Lower-priority periodic tasks (rate-monotonic) --------------- */
  loopTaskCbit(now);
  loopTaskHeartbeat(now);

  /* ----- [REQ-LOOP-099] Watchdog (mandatory) -------------------------- */
  serviceWatchdogIfHealthy(loop_started_ms);
}
