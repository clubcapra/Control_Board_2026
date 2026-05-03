/* =============================================================================
 *  estop.h
 * -----------------------------------------------------------------------------
 *  Emergency Stop subsystem.  Two independent paths exist:
 *
 *      1)  STM_E_Stop  (PA0)  -> driven by the STM32 to engage the E-Stop.
 *                                 Active-LOW: PA0 LOW asserts E-Stop and
 *                                 opens the circuit; PA0 HIGH releases it.
 *      2)  ESTOP_STATUS (PB12) <- read by the STM32 to learn the global
 *                                 E-Stop bus level (could be asserted by
 *                                 another node on the bus).
 *      3)  ESTOP_VTX   (PB13)  -> companion output sometimes wired in series
 *                                 with the redundant E-Stop loop.
 *
 *  Reading the status pin is safety-critical: a stuck-low or stuck-high pin
 *  must NOT be interpreted as a real assertion.  We therefore debounce the
 *  input over `kPgDebounce_ms` and require two consistent samples before
 *  promoting a state change.
 * =============================================================================
 */
#ifndef PDU_ESTOP_H_
#define PDU_ESTOP_H_

#include "avionics_types.h"

namespace pdu {
namespace estop {

/**  Initialise GPIOs and start the input in the de-asserted state.          */
Status init();

/**  Periodic tick: must run at least every 5 ms to keep debouncing live.    */
void tick();

/**  True if the external E-Stop bus is currently asserted (debounced).      */
bool isAsserted();

/**  Drive the local E-Stop output pin.  `engage == true` asserts the active-
 *   LOW PA0 command, so the physical pin is driven LOW.                     */
void assertLocal(bool engage);

/**  Drive the VTX companion line.  Active-HIGH: `engage == true` drives PB13
 *   HIGH which engages the VTX loop.  At reset PB13 is held LOW so the VTX
 *   relay sits in its de-energised (safe) state until firmware decides
 *   otherwise.                                                              */
void setVtx(bool engage);

/**  Convenience: returns whether either the local command or the bus is
 *   asserting the E-Stop loop.                                              */
bool isEStopActive();

}  /* namespace estop */
}  /* namespace pdu */

#endif /* PDU_ESTOP_H_ */
