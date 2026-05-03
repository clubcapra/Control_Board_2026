/* =============================================================================
 *  control_api.h - External I2C control/telemetry API.
 * -----------------------------------------------------------------------------
 *  I2C1 (PB6/PB7) remains the internal SMBus master for LM5066H1 devices.
 *  This module exposes the STM32 as an I2C2 slave (PB10/PB11) for another
 *  computer.  Callbacks only move bytes; commands are executed from loop().
 * =============================================================================
 */
#ifndef PDU_CONTROL_API_H_
#define PDU_CONTROL_API_H_

#include "avionics_types.h"
#include "bit.h"
#include "rail.h"

namespace pdu {
namespace control_api {

enum class Register : uint8_t {
  kInfo         = 0x00U,  /* read: ApiInfo                                   */
  kTelemetryAll = 0x10U,  /* read: ApiTelemetryAll                           */
  kCommand      = 0x20U,  /* write: ApiCommandFrame                           */
  kCommandStatus= 0x21U,  /* read: ApiCommandResult                           */
  kPmbusBridge  = 0x40U,  /* write: ApiPmbusBridgeRequest; read len 0 = auto  */
  kPmbusResult  = 0x41U,  /* read: ApiPmbusBridgeResult                       */
};

enum class Command : uint8_t {
  kNoop          = 0x00U,
  kSetRailEnable = 0x01U, /* arg0 rail: 0=48,1=24,2=12,3=all; arg1 0/1       */
  kSetLedDuty    = 0x02U, /* arg0 channel 0..3; arg1 duty 0..100             */
  kSetAllLeds    = 0x03U, /* arg0 duty 0..100                                */
  kSetLedPattern = 0x04U, /* arg0 leds::Pattern numeric value                */
  kSetEstopLocal = 0x05U, /* arg0 0/1                                        */
  kSetEstopVtx   = 0x06U, /* arg0 0/1                                        */
  kClearRailLatch= 0x07U, /* arg0 rail: 0=48,1=24,2=12,3=all                */
  kClearFaultLog = 0x08U, /* clears reset-persistent fault FIFO              */
  kResetDevice   = 0x09U, /* records host-requested reset then reboots MCU   */
  kSetUnixTime   = 0x0AU, /* arg0..3 little-endian Unix time in seconds      */
  kRefreshHotswapBlackBox = 0x0BU, /* arg0 rail/all; reread LM5066 BB memory */
  kEraseHotswapBlackBox   = 0x0CU, /* arg0 rail/all; erase LM5066 BB EEPROM  */
  kSetWinchMode          = 0x30U, /* arg0 winch::Mode                         */
  kSetWinchDcMotor       = 0x31U, /* arg0 motor 0/1, arg1 signed percent      */
  kSetWinchParallelDc    = 0x32U, /* arg0 signed percent                      */
  kSetWinchStepperPhases = 0x33U, /* arg0 phase A signed %, arg1 phase B %    */
  kBrakeWinch            = 0x34U, /* brake both bridges                       */
  kClearWinchFault       = 0x35U, /* nSLEEP low pulse clears DRV8262 fault    */
  kSetWinchLock          = 0x36U, /* arg0 0=lock1,1=lock2,2=all; arg1 0/1     */
};

enum class BridgeOp : uint8_t {
  kRead  = 0U,
  kWrite = 1U,
};

Status init();

void tick(rail::Controller& r48,
          rail::Controller& r24,
          rail::Controller& r12,
          SupervisorMode mode,
          const bit::Report& pbit,
          const bit::Report& cbit);

}  /* namespace control_api */
}  /* namespace pdu */

#endif /* PDU_CONTROL_API_H_ */
