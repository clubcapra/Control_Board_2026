/* =============================================================================
 *  leds.cpp - VNQ5E050AKTR-E lighting high-side driver for the PDU.
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

void writeRaw(Channel ch, uint8_t duty_pct) {
  const uint8_t idx = static_cast<uint8_t>(ch);
  if (idx >= kChannels) {
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
    pinMode(kPinMap[i], OUTPUT);
    analogWrite(kPinMap[i], 0);
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
      analogWrite(kPinMap[i], 0);
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

  switch (s_pattern) {
    case Pattern::kHeartbeat: {
      /* short bright pulse (50 ms ON, 950 ms OFF).                         */
      const uint32_t cycle = dt % 1000UL;
      const uint8_t  duty  = (cycle < 50UL) ? 70U : 0U;
      for (uint8_t i = 0U; i < kChannels; ++i) {
        analogWrite(kPinMap[i], dutyToPwm(duty));
      }
      break;
    }
    case Pattern::kFault: {
      /* 5 Hz square wave at full intensity.                                */
      const uint8_t duty = ((dt / 100UL) & 0x1U) ? 100U : 0U;
      for (uint8_t i = 0U; i < kChannels; ++i) {
        analogWrite(kPinMap[i], dutyToPwm(duty));
      }
      break;
    }
    case Pattern::kEStop: {
      /* 1 Hz strobe (200 ms ON, 800 ms OFF).                               */
      const uint32_t cycle = dt % 1000UL;
      const uint8_t  duty  = (cycle < 200UL) ? 100U : 0U;
      for (uint8_t i = 0U; i < kChannels; ++i) {
        analogWrite(kPinMap[i], dutyToPwm(duty));
      }
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
