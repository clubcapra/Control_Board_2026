/* =============================================================================
 *  leds.cpp - VNQ5E050AKTR-E lighting high-side driver for the PDU.
 * -----------------------------------------------------------------------------
 *  [REQ-LED-001]  Every channel MUST be at 0 % at boot and stay at 0 % until
 *                 the host commands a non-zero duty.
 *  [REQ-LED-002]  The OFF state MUST drive the high-side switch input as a
 *                 GPIO LOW push-pull, NOT as a PWM "0 %" alternate function,
 *                 to eliminate any reconfiguration glitch on the input pin
 *                 of the VNQ5E050AKTR-E quad smart switch.
 *  [REQ-LED-003]  The PWM carrier frequency for every lighting channel is
 *                 fixed at `cfg::kLedPwmFrequency_Hz` (2 kHz).  The frequency
 *                 is asserted at init() time AND re-asserted immediately
 *                 before every analogWrite() so that any module that later
 *                 calls analogWriteFrequency() (a framework-global setting)
 *                 cannot silently drift the LED carrier rate.
 *  [REQ-LED-004]  When a host command sets a channel from OFF to a duty
 *                 below `cfg::kLedKickThreshold_pct`, the driver briefly
 *                 pulses the channel at `cfg::kLedKickDuty_pct` for
 *                 `cfg::kLedKickHold_ms`, then settles at the commanded
 *                 duty.  This wakes the VNQ5E050AKTR-E internal charge
 *                 pump so the high-side FET reaches full enhancement
 *                 before being asked to switch at a low duty.  The kick
 *                 fires only on COLD transitions (previous duty was 0):
 *                 ramping 5 %->6 %->7 % does NOT re-trigger it.
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

uint8_t  s_duty[kChannels]    = {0U, 0U, 0U, 0U};
/* [REQ-LED-004] Per-channel "currently driving non-zero PWM" latch.
 * Set true on every non-zero analogWrite() through `writeRaw`, cleared
 * back to false whenever the channel is forced to OFF via GPIO LOW.
 * The cold-start kick only fires when this latch is false at the moment
 * a non-zero duty command arrives.                                       */
bool     s_driven[kChannels]  = {false, false, false, false};
Pattern  s_pattern            = Pattern::kOff;
uint32_t s_pattern_t0_ms      = 0U;
bool     s_initialised        = false;

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

/* [REQ-LED-003] Single funnel for every LED analogWrite.  Re-asserts the
 * 2 kHz carrier frequency immediately before the analogWrite() so that the
 * framework's pwm_start() path (which reads `_writeFreq` and pushes it into
 * TIM3/TIM4 ARR via setOverflow(..., HERTZ_FORMAT)) always reconfigures the
 * timer to the LED-module-owned rate, regardless of what any other module
 * may have set the global frequency to since the last call.                 */
void writeLedPwm(uint32_t pin, uint8_t pwm_0_255) {
  analogWriteFrequency(cfg::kLedPwmFrequency_Hz);
  analogWrite(pin, pwm_0_255);
}

/* [REQ-LED-004] Command-path write for a single channel.  Implements the
 * VNQ5E050AKTR-E cold-start kick:
 *   - duty == 0 %             -> force GPIO LOW, clear the "driven" latch.
 *   - duty >= kick threshold  -> direct write, set the "driven" latch.
 *   - duty < kick threshold AND channel currently NOT driven (cold) ->
 *         pulse at the kick duty for the configured hold time, then
 *         settle at the commanded duty.
 *   - duty < kick threshold AND channel already driven (warm) ->
 *         direct write, no kick (this is how live ramps stay smooth).
 *
 * Bounded delay: the kick adds at most cfg::kLedKickHold_ms per cold
 * channel.  Even the worst case (four channels kicked back-to-back from
 * a single `leds::setAll(<threshold>)`) is 4 * kLedKickHold_ms, which
 * fits inside one IWDG window without an explicit watchdog kick.        */
void writeRaw(Channel ch, uint8_t duty_pct) {
  const uint8_t idx = static_cast<uint8_t>(ch);
  if (idx >= kChannels) {
    return;
  }
  if (duty_pct == 0U) {
    writeOffViaGpio(kPinMap[idx]);
    s_driven[idx] = false;
    return;
  }

  const bool cold        = !s_driven[idx];
  const bool below_thres = (duty_pct < cfg::kLedKickThreshold_pct);
  if (cold && below_thres) {
    writeLedPwm(kPinMap[idx], dutyToPwm(cfg::kLedKickDuty_pct));
    delay(cfg::kLedKickHold_ms);
  }

  writeLedPwm(kPinMap[idx], dutyToPwm(duty_pct));
  s_driven[idx] = true;
}

}  // namespace

Status init() {
  if (s_initialised) {
    return Status::kOk;
  }
  /* [REQ-LED-003] Lock the framework-global PWM frequency to the
   * LED-module-owned 2 kHz carrier BEFORE any analogWrite() so the very
   * first non-zero duty already comes out at the right rate.  The same
   * value is re-asserted before each subsequent analogWrite() via
   * `writeLedPwm()` to defend against any future module changing it.       */
  analogWriteFrequency(cfg::kLedPwmFrequency_Hz);

  for (uint8_t i = 0U; i < kChannels; ++i) {
    writeOffViaGpio(kPinMap[i]);
    s_duty[i]    = 0U;
    s_driven[i]  = false;
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
      s_driven[i] = false;
    }
  } else if (p == Pattern::kSolid) {
    /* [REQ-LED-004] Going back to solid mode counts as a command receipt
     * per channel: writeRaw() will cold-start-kick any channel whose
     * stored duty is below kLedKickThreshold_pct.                       */
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
   * while honouring:
   *   [REQ-LED-002] 0 % must always be a GPIO LOW;
   *   [REQ-LED-003] every PWM write goes through the LED funnel that
   *                 re-asserts the 2 kHz carrier;
   *   [REQ-LED-004] blink/strobe refreshes intentionally bypass the
   *                 cold-start kick (the patterns only ever drive 0 %
   *                 or >= 70 %, so a kick would never apply anyway, and
   *                 a per-tick delay would destroy the pattern timing).
   * The `s_driven[]` latch IS still maintained so a subsequent solid /
   * setDuty command sees a coherent "channel currently driven?" state. */
  auto driveAll = [&](uint8_t duty) {
    for (uint8_t i = 0U; i < kChannels; ++i) {
      if (duty == 0U) {
        writeOffViaGpio(kPinMap[i]);
        s_driven[i] = false;
      } else {
        writeLedPwm(kPinMap[i], dutyToPwm(duty));
        s_driven[i] = true;
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
