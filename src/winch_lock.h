/* =============================================================================
 *  winch_lock.h - TPS2HB16AQPWPRQ1 winch-lock high-side outputs.
 * =============================================================================
 */
#ifndef PDU_WINCH_LOCK_H_
#define PDU_WINCH_LOCK_H_

#include "avionics_types.h"

namespace pdu {
namespace winch_lock {

enum class Channel : uint8_t {
  kLock1 = 0U,
  kLock2 = 1U,
  kAll   = 2U,
};

struct Telemetry {
  bool lock1_on;
  bool lock2_on;
};

Status init();
Status set(Channel channel, bool on);
Status setAll(bool on);
Telemetry telemetry();

}  /* namespace winch_lock */
}  /* namespace pdu */

#endif /* PDU_WINCH_LOCK_H_ */
