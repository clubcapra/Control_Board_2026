/* =============================================================================
 *  iwdg.cpp - Independent Watchdog driver for STM32F103.
 * =============================================================================
 */
#include "iwdg.h"
#include "avionics_config.h"
#include "fault_log.h"

#include <Arduino.h>

#if defined(ARDUINO_ARCH_STM32) || defined(STM32F1xx) || defined(STM32F1)
#  include <IWatchdog.h>
#  define PDU_IWDG_BACKEND_STM32 1
#else
#  define PDU_IWDG_BACKEND_STM32 0
#endif

namespace pdu {
namespace iwdg {

namespace {
bool s_was_reset_by_wd = false;
bool s_started = false;
}  // namespace

Status init() {
  if (s_started) {
    return Status::kOk;
  }

#if PDU_IWDG_BACKEND_STM32
  /* Capture the reset cause exactly once.  fault_log::init() may already have
   * copied and cleared RCC->CSR, so also consult its preserved boot flags. */
  const uint32_t reset_flags = RCC->CSR | fault_log::bootResetFlags();
  s_was_reset_by_wd = (reset_flags & RCC_CSR_IWDGRSTF) != 0U;
  RCC->CSR |= RCC_CSR_RMVF;

  const uint32_t timeout_us = cfg::kIwdgTimeout_ms * 1000UL;
  /* IWatchdog::begin() expects microseconds.  Maximum on F103 is ~26000000us
   * (with prescaler 256 and 12-bit reload).  We assert at compile time that
   * the configured timeout fits.                                              */
  static_assert(cfg::kIwdgTimeout_ms <= 26000UL,
                "IWDG timeout exceeds STM32F1 hardware capability");

  IWatchdog.begin(timeout_us);
#else
  /* Non-STM32 build (host unit-test or ESP32 prototype): no-op stub.          */
  s_was_reset_by_wd = false;
#endif

  s_started = true;
  return Status::kOk;
}

void kick() {
  if (!s_started) {
    return;
  }
#if PDU_IWDG_BACKEND_STM32
  IWatchdog.reload();
#endif
}

bool wasResetByWatchdog() {
  return s_was_reset_by_wd;
}

}  /* namespace iwdg */
}  /* namespace pdu */
