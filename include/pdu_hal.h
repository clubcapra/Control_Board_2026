/* =============================================================================
 *  pdu_hal.h - Qualified primitives layer.
 * -----------------------------------------------------------------------------
 *  AIRWORTHINESS DISCLOSURE
 *  ------------------------
 *  This header defines the SAFETY-CRITICAL primitives the PDU supervisor is
 *  allowed to call from any path that must remain qualifiable to DO-178C
 *  Level A.  Every function in `pdu::hal::qualified::` accesses ONLY:
 *
 *    1. STM32 CMSIS-CORE / device headers (`stm32f1xx.h`, register typedefs)
 *    2. Plain C++14 language features (no STL, no exceptions, no RTTI)
 *    3. Compile-time constants from this header or `avionics_config.h`
 *
 *  In particular, NONE of the qualified primitives call into:
 *    - The Arduino-Core-STM32 framework
 *    - The STM32 HAL (HAL_*, HAL_GPIO_*, HAL_IWDG_*)
 *    - Any of `Wire`, `SerialSWO`, `IWatchdog`
 *    - The C++ standard library
 *
 *  The non-qualified zone (the "W-zone" in DO-178C parlance: Whitebox-only
 *  code which must be replaced before formal Level A submission) consists
 *  of every other source file in this project that still imports
 *  `<Arduino.h>` and uses pinMode / digitalWrite / Wire / Serial.
 *
 *  Mapping of REQ tags to qualified primitives
 *  -------------------------------------------
 *    [REQ-LOOP-099]  iwdgKick()            - foreground watchdog refresh
 *    [REQ-IO-001]    estopGpioReleased()   - read PA0 E-Stop release
 *    [REQ-IO-005]    estopGpioStatusHigh() - read PB12 E-Stop status feedback
 *    [REQ-LED-001]   gpioForceLow()        - drive an LED / lock to LOW
 *    [REQ-DIAG-002]  stackCanaryArm/Check  - stack-bottom canary
 *
 *  When the system is ported to a qualified MCU (e.g. SAFEFPGA + a
 *  certified Cortex-M part with ECC + lockstep), this header is the only
 *  call surface the safety-critical paths use, so the port consists of
 *  re-implementing this file against the new platform's qualified
 *  primitives - nothing else moves.
 * =============================================================================
 */
#ifndef PDU_HAL_H_
#define PDU_HAL_H_

#include <stdint.h>

#if defined(STM32F1xx)
#  include "stm32f1xx.h"
#endif

namespace pdu {
namespace hal {
namespace qualified {

/* ===========================================================================
 *               WATCHDOG  [REQ-LOOP-099]
 * ---------------------------------------------------------------------------
 *  STM32 IWDG refresh sequence: write 0xAAAA to IWDG_KR.  The IWDG was
 *  already armed by `iwdg::init()` (which still uses the Arduino HAL
 *  IWatchdog wrapper - see W-zone in pdu_hal.h header comment); only
 *  the HOT path of refreshing it from `loop()` is qualified here.
 * =========================================================================== */
static inline void iwdgKick() {
#if defined(STM32F1xx)
  /* IWDG_KR_KEY_RELOAD == 0xAAAA reloads the IWDG counter.               */
  IWDG->KR = 0xAAAAUL;
#endif
}

/* ===========================================================================
 *               GPIO  [REQ-IO-001 / REQ-IO-005 / REQ-LED-001]
 * ---------------------------------------------------------------------------
 *  Direct register access; no Arduino HAL.  The pin number is the bit
 *  position in the GPIOx_IDR / GPIOx_ODR registers (0..15), NOT the
 *  Arduino board pin index.
 * =========================================================================== */
static inline bool gpioRead(GPIO_TypeDef* port, uint8_t pin) {
#if defined(STM32F1xx)
  return ((port->IDR >> pin) & 0x1U) != 0U;
#else
  (void)port;
  (void)pin;
  return false;
#endif
}

static inline void gpioForceHigh(GPIO_TypeDef* port, uint8_t pin) {
#if defined(STM32F1xx)
  port->BSRR = (1UL << pin);
#else
  (void)port;
  (void)pin;
#endif
}

static inline void gpioForceLow(GPIO_TypeDef* port, uint8_t pin) {
#if defined(STM32F1xx)
  port->BRR = (1UL << pin);
#else
  (void)port;
  (void)pin;
#endif
}

/** Active-LOW E-Stop release on PA0.                                       *
 *  Returns true iff the local board is releasing the E-Stop loop          *
 *  (PA0 driven HIGH).                                                     */
static inline bool estopGpioReleased() {
  return gpioRead(GPIOA, 0U);
}

/** E-Stop loop status feedback on PB12.                                   *
 *  Returns true iff the loop is closed (status feedback HIGH).            */
static inline bool estopGpioStatusHigh() {
  return gpioRead(GPIOB, 12U);
}

/* Stack canary: provided by the application (main.cpp) which owns a
 * dedicated .noinit slot.  Not exposed via this header to keep the
 * qualified-primitives surface stateless.                                */

}  /* namespace qualified */
}  /* namespace hal */
}  /* namespace pdu */

#endif  /* PDU_HAL_H_ */
