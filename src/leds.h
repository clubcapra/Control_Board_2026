/* =============================================================================
 *  leds.h
 * -----------------------------------------------------------------------------
 *  Driver for the four PWM-controlled lighting channels through the
 *  VNQ5E050AKTR-E quad high-side driver.  The driver exposes a small command surface
 *  (set duty cycle, run pattern) and is the only module allowed to touch the
 *  associated GPIOs.  This isolation is required by DO-178C (data coupling
 *  control) so a bug in one module cannot silently corrupt unrelated I/O.
 * =============================================================================
 */
#ifndef PDU_LEDS_H_
#define PDU_LEDS_H_

#include "avionics_types.h"

namespace pdu {
namespace leds {

enum class Channel : uint8_t {
  kBras  = 0U,
  kAvant = 1U,
  kArr   = 2U,
  kExtra = 3U,
  kCount
};

/**  Built-in playback patterns, used to render supervisor mode visually.    */
enum class Pattern : uint8_t {
  kOff       = 0U,  /* all channels at 0 %                                   */
  kSolid     = 1U,  /* user-defined static duty                              */
  kHeartbeat = 2U,  /* short blip every kHeartbeat_ms                        */
  kFault     = 3U,  /* fast blink at ~5 Hz to flag a fault                   */
  kEStop     = 4U,  /* slow strobe to flag emergency stop                    */
};

/**  Initialise GPIOs as PWM outputs and force every channel to 0 %.          */
Status init();

/**  Set the duty cycle (0..100 %) of one channel.                            */
Status setDuty(Channel ch, uint8_t duty_pct);

/**  Convenience: set a single solid duty for ALL channels at once.           */
Status setAll(uint8_t duty_pct);

/**  Switch to a built-in pattern.                                            */
Status setPattern(Pattern p);

/**  Periodic tick.  Must be called from the main loop; the driver advances
 *   any blink/strobe pattern based on millis().                              */
void tick();

}  /* namespace leds */
}  /* namespace pdu */

#endif /* PDU_LEDS_H_ */
