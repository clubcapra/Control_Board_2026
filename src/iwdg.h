/* =============================================================================
 *  iwdg.h
 * -----------------------------------------------------------------------------
 *  Independent Watchdog (IWDG) driver.  The IWDG is clocked by the LSI RC
 *  oscillator and runs even when the main core clock fails - it is the last
 *  line of defence against software lockup, exactly the role it plays in
 *  flight-critical avionics computers.
 *
 *  The watchdog is FAIL-SAFE: once started it CANNOT be disabled in software.
 *  This is intentional: any silent return from main() or any hung loop will
 *  reset the MCU within `kIwdgTimeout_ms`.
 * =============================================================================
 */
#ifndef PDU_IWDG_H_
#define PDU_IWDG_H_

#include "avionics_types.h"

namespace pdu {
namespace iwdg {

/**
 *  Initialise the independent watchdog with the configured timeout.
 *  Must be called once during boot, AFTER any RAM tests (which take time and
 *  could otherwise trigger a spurious reset).
 *
 *  Returns Status::kOk on success.  On STM32F1 the IWDG cannot fail to start
 *  if the LSI starts correctly; the call still validates the timeout range.
 */
Status init();

/**
 *  Pet the dog.  MUST be called from the main loop at least every
 *  (kIwdgTimeout_ms / 2) milliseconds, ideally once per supervisor cycle.
 *
 *  Calling kick() while the supervisor is in the kFault state is permitted
 *  intentionally: a controlled fault state is preferable to an uncontrolled
 *  reboot loop.  The supervisor decides when to stop kicking.
 */
void kick();

/**
 *  Returns true if the most recent reset was caused by an IWDG time-out.
 *  Must be queried before kick() is first called - the flag is auto-cleared
 *  by the boot procedure.
 */
bool wasResetByWatchdog();

}  /* namespace iwdg */
}  /* namespace pdu */

#endif /* PDU_IWDG_H_ */
