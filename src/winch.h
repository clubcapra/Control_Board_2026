/* =============================================================================
 *  winch.h - DRV8262DDVR winch-lock motor driver.
 * -----------------------------------------------------------------------------
 *  The DRV8262 can be configured as:
 *    - dual H-bridge: 2 brushed DC motors, or 1 bipolar stepper motor;
 *    - single parallel H-bridge: 1 brushed DC motor with both bridges paralleled.
 * =============================================================================
 */
#ifndef PDU_WINCH_H_
#define PDU_WINCH_H_

#include "avionics_types.h"

namespace pdu {
namespace winch {

enum class Mode : uint8_t {
  kSleep      = 0U,
  kDualDc     = 1U,
  kStepper    = 2U,
  kParallelDc = 3U,
};

enum class Motor : uint8_t {
  kA = 0U,
  kB = 1U,
};

struct Telemetry {
  Mode    mode;
  bool    awake;
  bool    fault_active;
  int8_t  motor_a_cmd_pct;
  int8_t  motor_b_cmd_pct;
  int8_t  parallel_cmd_pct;
  int8_t  stepper_a_cmd_pct;
  int8_t  stepper_b_cmd_pct;
};

Status init();
Status sleep();
Status clearFault();
Status setMode(Mode mode);
Status setDcMotor(Motor motor, int8_t command_pct);
Status setParallelDc(int8_t command_pct);
Status setStepperPhases(int8_t phase_a_pct, int8_t phase_b_pct);
Status brakeAll();
Telemetry telemetry();

}  /* namespace winch */
}  /* namespace pdu */

#endif /* PDU_WINCH_H_ */
