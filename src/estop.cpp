/* =============================================================================
 *  estop.cpp - debounced E-Stop subsystem.
 * =============================================================================
 */
#include "estop.h"
#include "avionics_config.h"

#include <Arduino.h>

// #region agent log
#include "console.h"
// #endregion

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
  // #region agent log
  /* Snapshot PA0's mode-register nibble (CRL[3:0]), the register-state
   * ODR (immediate, never lags), AND the sampled IDR pin level BEFORE the
   * BRR/BSRR write so we can distinguish:
   *   - firmware bug: ODR doesn't follow what we write
   *   - electrical:   ODR follows but pad/IDR can't reach the commanded level
   *                   (external short, fight from another driver, etc.)   */
  const uint32_t crl_before = GPIOA->CRL;
  const uint32_t mode_pa0   = crl_before & 0xFU;          /* CRL[3:0] */
  const uint32_t odr_before = (GPIOA->ODR >> 0U) & 1U;
  const uint32_t idr_before = (GPIOA->IDR >> 0U) & 1U;
  // #endregion
  if (!s_initialised) {
    // #region agent log
    Serial.print(F("[ESTOP DBG] assertLocal SKIPPED s_initialised=0 engage="));
    Serial.println(engage ? 1 : 0);
    // #endregion
    return;
  }
  s_local_assert = engage;
  driveLocalEStopAsserted(engage);
  // #region agent log
  /* A few NOPs to allow the pad to settle if it's heavily capacitively
   * loaded; rules out pure IDR-sampling-too-fast as a confounder.         */
  for (uint8_t i = 0U; i < 16U; ++i) {
    __NOP();
  }
  const uint32_t odr_after = (GPIOA->ODR >> 0U) & 1U;
  const uint32_t idr_after = (GPIOA->IDR >> 0U) & 1U;
  Serial.print(F("[ESTOP DBG] assertLocal engage="));
  Serial.print(engage ? 1 : 0);
  Serial.print(F(" CRL[3:0]=0x"));
  Serial.print(mode_pa0, HEX);
  Serial.print(F(" ODR.0 "));
  Serial.print(odr_before);
  Serial.print(F("->"));
  Serial.print(odr_after);
  Serial.print(F(" IDR.0 "));
  Serial.print(idr_before);
  Serial.print(F("->"));
  Serial.print(idr_after);
  Serial.print(F(" expect_pad="));
  Serial.println(engage ? 0 : 1);   /* PA0 is active-LOW: assert -> 0 */
  // #endregion
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
