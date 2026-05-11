/* =============================================================================
 *  leds.cpp - VNQ5E050AKTR-E lighting high-side driver for the PDU.
 * -----------------------------------------------------------------------------
 *  [REQ-LED-001]  Every channel MUST be at 0 % at boot and stay at 0 % until
 *                 the host commands a non-zero duty.
 *  [REQ-LED-002]  The OFF state MUST drive the high-side switch input as a
 *                 GPIO LOW push-pull, NOT as a PWM "0 %" alternate function,
 *                 to eliminate any reconfiguration glitch on the input pin
 *                 of the VNQ5E050AKTR-E quad smart switch.
 * =============================================================================
 */
#include "leds.h"
#include "avionics_config.h"

#include <Arduino.h>

namespace pdu {
namespace leds {

namespace {

constexpr uint8_t kChannels = static_cast<uint8_t>(Channel::kCount);

/**  Map a logical channel to its physical pin.  Single source of truth.    */
constexpr uint32_t kPinMap[kChannels] = {
    cfg::kPin_LED_Bras,
    cfg::kPin_LED_Avant,
    cfg::kPin_LED_Arr,
    cfg::kPin_LED_Extra,
};

uint8_t  s_duty[kChannels] = {0U, 0U, 0U, 0U};
Pattern  s_pattern         = Pattern::kOff;
uint32_t s_pattern_t0_ms   = 0U;
bool     s_initialised     = false;

inline uint8_t dutyToPwm(uint8_t duty_pct) {
  /* Clamp to [0..100] then map to the Arduino 0..255 PWM range.            */
  if (duty_pct > 100U) {
    duty_pct = 100U;
  }
  return static_cast<uint8_t>((static_cast<uint32_t>(duty_pct) * 255UL) / 100UL);
}

/* Drive a lighting channel to the OFF state by force.  This always returns
 * the pin to plain GPIO push-pull driving LOW, even if a previous call
 * placed it in TIM3 alternate-function mode.  Used everywhere instead of
 * `analogWrite(pin, 0)` because the PWM peripheral reconfiguration window
 * is not guaranteed to leave the pin low.                                   */
void writeOffViaGpio(uint32_t pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void writeRaw(Channel ch, uint8_t duty_pct) {
  const uint8_t idx = static_cast<uint8_t>(ch);
  if (idx >= kChannels) {
    return;
  }
  if (duty_pct == 0U) {
    writeOffViaGpio(kPinMap[idx]);
    return;
  }
  analogWrite(kPinMap[idx], dutyToPwm(duty_pct));
}

}  // namespace

Status init() {
  if (s_initialised) {
    return Status::kOk;
  }
  for (uint8_t i = 0U; i < kChannels; ++i) {
    writeOffViaGpio(kPinMap[i]);
    s_duty[i] = 0U;
  }
  s_pattern       = Pattern::kOff;
  s_pattern_t0_ms = millis();
  s_initialised   = true;
  return Status::kOk;
}

Status setDuty(Channel ch, uint8_t duty_pct) {
  if (!s_initialised) {
    return Status::kNotInit;
  }
  const uint8_t idx = static_cast<uint8_t>(ch);
  if (idx >= kChannels) {
    return Status::kParam;
  }
  if (duty_pct > 100U) {
    duty_pct = 100U;
  }
  s_duty[idx] = duty_pct;
  if (s_pattern == Pattern::kSolid || s_pattern == Pattern::kOff) {
    writeRaw(ch, duty_pct);
  }
  return Status::kOk;
}

Status setAll(uint8_t duty_pct) {
  Status rc = Status::kOk;
  for (uint8_t i = 0U; i < kChannels; ++i) {
    const Status s = setDuty(static_cast<Channel>(i), duty_pct);
    if (s != Status::kOk) {
      rc = s;
    }
  }
  return rc;
}

Status setPattern(Pattern p) {
  if (!s_initialised) {
    return Status::kNotInit;
  }
  s_pattern       = p;
  s_pattern_t0_ms = millis();
  if (p == Pattern::kOff) {
    for (uint8_t i = 0U; i < kChannels; ++i) {
      writeOffViaGpio(kPinMap[i]);
    }
  } else if (p == Pattern::kSolid) {
    for (uint8_t i = 0U; i < kChannels; ++i) {
      writeRaw(static_cast<Channel>(i), s_duty[i]);
    }
  }
  return Status::kOk;
}

void tick() {
  if (!s_initialised) {
    return;
  }
  const uint32_t now = millis();
  const uint32_t dt  = now - s_pattern_t0_ms;

  /* Helper to push a duty to every channel for blink / strobe patterns
   * while honouring [REQ-LED-002]: 0 % must always be a GPIO LOW.          */
  auto driveAll = [&](uint8_t duty) {
    for (uint8_t i = 0U; i < kChannels; ++i) {
      if (duty == 0U) {
        writeOffViaGpio(kPinMap[i]);
      } else {
        analogWrite(kPinMap[i], dutyToPwm(duty));
      }
    }
  };

  switch (s_pattern) {
    case Pattern::kHeartbeat: {
      /* short bright pulse (50 ms ON, 950 ms OFF).                         */
      const uint32_t cycle = dt % 1000UL;
      driveAll((cycle < 50UL) ? 70U : 0U);
      break;
    }
    case Pattern::kFault: {
      /* 5 Hz square wave at full intensity.                                */
      driveAll(((dt / 100UL) & 0x1U) ? 100U : 0U);
      break;
    }
    case Pattern::kEStop: {
      /* 1 Hz strobe (200 ms ON, 800 ms OFF).                               */
      const uint32_t cycle = dt % 1000UL;
      driveAll((cycle < 200UL) ? 100U : 0U);
      break;
    }
    case Pattern::kOff:
    case Pattern::kSolid:
    default:
      /* Static patterns do not need periodic refresh.                      */
      break;
  }
}

}  /* namespace leds */
}  /* namespace pdu */
