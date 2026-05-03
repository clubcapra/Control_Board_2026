/* =============================================================================
 *  winch.cpp - DRV8262DDVR winch-lock motor driver implementation.
 * =============================================================================
 */
#include "winch.h"

#include "avionics_config.h"

#include <Arduino.h>

namespace pdu {
namespace winch {

namespace {

constexpr uint8_t kPwmMax = 255U;

Mode   s_mode = Mode::kSleep;
bool   s_initialised = false;
bool   s_mode_control_available = false;
bool   s_sleep_control_available = false;
bool   s_fault_input_available = false;
int8_t s_motor_a_cmd_pct = 0;
int8_t s_motor_b_cmd_pct = 0;
int8_t s_parallel_cmd_pct = 0;
int8_t s_stepper_a_cmd_pct = 0;
int8_t s_stepper_b_cmd_pct = 0;

int8_t clampPct(int8_t value) {
  if (value > 100) {
    return 100;
  }
  if (value < -100) {
    return -100;
  }
  return value;
}

uint8_t absDuty(int8_t value) {
  const int16_t clamped = clampPct(value);
  const uint16_t magnitude = (clamped < 0) ? static_cast<uint16_t>(-clamped)
                                          : static_cast<uint16_t>(clamped);
  return static_cast<uint8_t>((magnitude * kPwmMax) / 100U);
}

bool pinConnected(uint32_t pin) {
  return pin != cfg::kPin_NotConnected;
}

void writeCoast(uint32_t in_a, uint32_t in_b) {
  analogWrite(in_a, 0U);
  analogWrite(in_b, 0U);
}

void writeBrake(uint32_t in_a, uint32_t in_b) {
  analogWrite(in_a, kPwmMax);
  analogWrite(in_b, kPwmMax);
}

void writeSignedBridge(uint32_t in_a, uint32_t in_b, int8_t command_pct) {
  const int8_t command = clampPct(command_pct);
  const uint8_t duty = absDuty(command);
  if (command > 0) {
    analogWrite(in_a, duty);
    analogWrite(in_b, 0U);
  } else if (command < 0) {
    analogWrite(in_a, 0U);
    analogWrite(in_b, duty);
  } else {
    writeCoast(in_a, in_b);
  }
}

void stopOutputs() {
  writeCoast(cfg::kPin_Winch_IN1, cfg::kPin_Winch_IN2);
  writeCoast(cfg::kPin_Winch_IN3, cfg::kPin_Winch_IN4);
  s_motor_a_cmd_pct = 0;
  s_motor_b_cmd_pct = 0;
  s_parallel_cmd_pct = 0;
  s_stepper_a_cmd_pct = 0;
  s_stepper_b_cmd_pct = 0;
}

void latchModePins(Mode mode) {
  /* MODE2 HIGH selects PWM (IN/IN) interface. MODE1 selects dual/parallel. */
  if (!s_mode_control_available) {
    return;
  }
  digitalWrite(cfg::kPin_Winch_MODE2, HIGH);
  digitalWrite(cfg::kPin_Winch_MODE1,
               (mode == Mode::kParallelDc) ? HIGH : LOW);
}

Status requireMode(Mode expected) {
  return (s_initialised && s_mode == expected) ? Status::kOk : Status::kParam;
}

}  // namespace

Status init() {
  if (s_initialised) {
    return Status::kOk;
  }

  pinMode(cfg::kPin_Winch_IN1, OUTPUT);
  pinMode(cfg::kPin_Winch_IN2, OUTPUT);
  pinMode(cfg::kPin_Winch_IN3, OUTPUT);
  pinMode(cfg::kPin_Winch_IN4, OUTPUT);
  s_mode_control_available = pinConnected(cfg::kPin_Winch_MODE1) &&
                             pinConnected(cfg::kPin_Winch_MODE2);
  s_sleep_control_available = pinConnected(cfg::kPin_Winch_nSLEEP);
  s_fault_input_available = pinConnected(cfg::kPin_Winch_nFAULT);
  if (s_mode_control_available) {
    pinMode(cfg::kPin_Winch_MODE1, OUTPUT);
    pinMode(cfg::kPin_Winch_MODE2, OUTPUT);
  }
  if (s_sleep_control_available) {
    pinMode(cfg::kPin_Winch_nSLEEP, OUTPUT);
    digitalWrite(cfg::kPin_Winch_nSLEEP, LOW);
  }
  if (s_fault_input_available) {
    pinMode(cfg::kPin_Winch_nFAULT, INPUT_PULLUP);
  }

  latchModePins(Mode::kDualDc);
  stopOutputs();
  s_mode = s_sleep_control_available ? Mode::kSleep : Mode::kDualDc;
  s_initialised = true;
  return Status::kOk;
}

Status sleep() {
  if (!s_initialised) {
    return Status::kNotInit;
  }
  stopOutputs();
  if (s_sleep_control_available) {
    digitalWrite(cfg::kPin_Winch_nSLEEP, LOW);
  }
  s_mode = Mode::kSleep;
  return Status::kOk;
}

Status clearFault() {
  if (!s_initialised) {
    return Status::kNotInit;
  }
  stopOutputs();
  if (!s_sleep_control_available) {
    return Status::kNotPresent;
  }
  digitalWrite(cfg::kPin_Winch_nSLEEP, LOW);
  delayMicroseconds(100U);
  if (s_mode != Mode::kSleep) {
    latchModePins(s_mode);
    digitalWrite(cfg::kPin_Winch_nSLEEP, HIGH);
    delay(2U);
  }
  return Status::kOk;
}

Status setMode(Mode mode) {
  if (!s_initialised) {
    return Status::kNotInit;
  }
  if (mode == Mode::kSleep) {
    return sleep();
  }
  if (!s_mode_control_available || !s_sleep_control_available) {
    if (mode == Mode::kDualDc || mode == Mode::kStepper) {
      stopOutputs();
      s_mode = mode;
      return Status::kOk;
    }
    return Status::kNotPresent;
  }
  if (mode != Mode::kDualDc &&
      mode != Mode::kStepper &&
      mode != Mode::kParallelDc) {
    return Status::kParam;
  }

  stopOutputs();
  digitalWrite(cfg::kPin_Winch_nSLEEP, LOW);
  delayMicroseconds(100U);
  latchModePins(mode);
  digitalWrite(cfg::kPin_Winch_nSLEEP, HIGH);
  delay(2U);  /* DRV8262 wake time is around 1 ms; keep margin. */
  s_mode = mode;
  return Status::kOk;
}

Status setDcMotor(Motor motor, int8_t command_pct) {
  if (requireMode(Mode::kDualDc) != Status::kOk) {
    return Status::kParam;
  }
  const int8_t command = clampPct(command_pct);
  if (motor == Motor::kA) {
    writeSignedBridge(cfg::kPin_Winch_IN1, cfg::kPin_Winch_IN2, command);
    s_motor_a_cmd_pct = command;
    return Status::kOk;
  }
  if (motor == Motor::kB) {
    writeSignedBridge(cfg::kPin_Winch_IN3, cfg::kPin_Winch_IN4, command);
    s_motor_b_cmd_pct = command;
    return Status::kOk;
  }
  return Status::kParam;
}

Status setParallelDc(int8_t command_pct) {
  if (requireMode(Mode::kParallelDc) != Status::kOk) {
    return Status::kParam;
  }
  const int8_t command = clampPct(command_pct);
  writeSignedBridge(cfg::kPin_Winch_IN1, cfg::kPin_Winch_IN2, command);
  writeSignedBridge(cfg::kPin_Winch_IN3, cfg::kPin_Winch_IN4, command);
  s_parallel_cmd_pct = command;
  return Status::kOk;
}

Status setStepperPhases(int8_t phase_a_pct, int8_t phase_b_pct) {
  if (requireMode(Mode::kStepper) != Status::kOk) {
    return Status::kParam;
  }
  const int8_t phase_a = clampPct(phase_a_pct);
  const int8_t phase_b = clampPct(phase_b_pct);
  writeSignedBridge(cfg::kPin_Winch_IN1, cfg::kPin_Winch_IN2, phase_a);
  writeSignedBridge(cfg::kPin_Winch_IN3, cfg::kPin_Winch_IN4, phase_b);
  s_stepper_a_cmd_pct = phase_a;
  s_stepper_b_cmd_pct = phase_b;
  return Status::kOk;
}

Status brakeAll() {
  if (!s_initialised) {
    return Status::kNotInit;
  }
  if (s_mode == Mode::kSleep) {
    return Status::kOk;
  }
  writeBrake(cfg::kPin_Winch_IN1, cfg::kPin_Winch_IN2);
  writeBrake(cfg::kPin_Winch_IN3, cfg::kPin_Winch_IN4);
  s_motor_a_cmd_pct = 0;
  s_motor_b_cmd_pct = 0;
  s_parallel_cmd_pct = 0;
  s_stepper_a_cmd_pct = 0;
  s_stepper_b_cmd_pct = 0;
  return Status::kOk;
}

Telemetry telemetry() {
  Telemetry t = {};
  t.mode = s_mode;
  t.awake = s_initialised && (s_mode != Mode::kSleep);
  t.fault_active = s_initialised && s_fault_input_available &&
                   (digitalRead(cfg::kPin_Winch_nFAULT) == LOW);
  t.motor_a_cmd_pct = s_motor_a_cmd_pct;
  t.motor_b_cmd_pct = s_motor_b_cmd_pct;
  t.parallel_cmd_pct = s_parallel_cmd_pct;
  t.stepper_a_cmd_pct = s_stepper_a_cmd_pct;
  t.stepper_b_cmd_pct = s_stepper_b_cmd_pct;
  return t;
}

}  /* namespace winch */
}  /* namespace pdu */
