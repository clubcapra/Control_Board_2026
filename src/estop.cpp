/* =============================================================================
 *  estop.cpp - debounced E-Stop subsystem.
 * =============================================================================
 */
#include "estop.h"
#include "avionics_config.h"

#include <Arduino.h>

namespace pdu {
namespace estop {

namespace {

bool     s_initialised      = false;
bool     s_asserted_stable  = false;  /* debounced public state              */
bool     s_asserted_raw     = false;  /* last raw reading                    */
uint32_t s_last_change_ms   = 0U;
bool     s_local_assert     = false;
bool     s_vtx_state        = false;

inline void drivePin(uint32_t pin, bool high) {
  if (pin == PA0) {
    if (high) {
      GPIOA->BSRR = (1UL << 0U);
    } else {
      GPIOA->BRR = (1UL << 0U);
    }
  } else if (pin == PB13) {
    if (high) {
      GPIOB->BSRR = (1UL << 13U);
    } else {
      GPIOB->BRR = (1UL << 13U);
    }
  } else {
    digitalWrite(pin, high ? HIGH : LOW);
  }
}

inline void driveLocalEStopAsserted(bool asserted) {
  /* STM_E_STOP is active-LOW in hardware: LOW opens the E-Stop loop and turns
   * the controlled circuit off; HIGH releases the local STM32 assertion.     */
  drivePin(cfg::kPin_EStopCmd, !asserted);
}

inline void setOutputsToSafeState() {
  /* Leave the local E-Stop command released (PA0 HIGH) and the companion VTX
   * loop de-energised (PB13 LOW). */
  driveLocalEStopAsserted(false);
  drivePin(cfg::kPin_EStopVtx, false);
}

inline bool readStatusRaw() {
  /* Hardware convention: the E-Stop status line is active-HIGH.  When the
   * external bus drives PB12 HIGH it means "E-Stop is asserted".  PB12 is
   * configured as INPUT_PULLDOWN, so an unconnected/floating bus reads LOW
   * (de-asserted) and a positive drive over ~1.5 V asserts.                */
  return digitalRead(cfg::kPin_EStopStatus) == HIGH;
}

}  // namespace

Status init() {
  if (s_initialised) {
    return Status::kOk;
  }

  /* Pre-load the output data registers BEFORE switching the pins to OUTPUT.
   * STM_E_STOP is active-LOW, so HIGH releases the local command.  VTX remains
   * active-HIGH and is held LOW until firmware explicitly engages it.        */
  digitalWrite(cfg::kPin_EStopCmd, HIGH);
  digitalWrite(cfg::kPin_EStopVtx, LOW);
  pinMode(cfg::kPin_EStopCmd, OUTPUT);
  pinMode(cfg::kPin_EStopVtx, OUTPUT);
  digitalWrite(cfg::kPin_EStopCmd, HIGH);
  digitalWrite(cfg::kPin_EStopVtx, LOW);
  setOutputsToSafeState();
  s_local_assert = false;
  s_vtx_state = false;

  /* Pull-down the status input so a disconnected bus reads "not asserted". */
  pinMode(cfg::kPin_EStopStatus, INPUT_PULLDOWN);

  s_asserted_raw    = readStatusRaw();
  s_asserted_stable = s_asserted_raw;
  s_last_change_ms  = millis();
  s_initialised     = true;
  return Status::kOk;
}

void tick() {
  if (!s_initialised) {
    return;
  }

  const bool     raw = readStatusRaw();
  const uint32_t now = millis();

  if (raw != s_asserted_raw) {
    /* Edge detected on the raw signal: restart debounce window.            */
    s_asserted_raw   = raw;
    s_last_change_ms = now;
  } else if ((now - s_last_change_ms) >= cfg::kPgDebounce_ms) {
    /* Signal has been stable long enough: promote it.                      */
    s_asserted_stable = raw;
  }
}

bool isAsserted() {
  return s_asserted_stable;
}

void assertLocal(bool engage) {
  if (!s_initialised) {
    return;
  }
  s_local_assert = engage;
  driveLocalEStopAsserted(engage);
}

void setVtx(bool engage) {
  if (!s_initialised) {
    return;
  }
  /* Active-HIGH: engage==true drives PB13 HIGH (VTX loop engaged). */
  s_vtx_state = engage;
  drivePin(cfg::kPin_EStopVtx, engage);
}

bool isEStopActive() {
  return s_asserted_stable || s_local_assert;
}

}  /* namespace estop */
}  /* namespace pdu */
