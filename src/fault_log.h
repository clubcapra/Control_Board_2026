/* =============================================================================
 *  fault_log.h - Reset-persistent fault history.
 * -----------------------------------------------------------------------------
 *  Uses the reserved last STM32F1 flash page as a reset-persistent FIFO.
 *  Records survive NVIC_SystemReset() and watchdog resets, so the external
 *  I2C API can report the recent fault history after reboot.
 * =============================================================================
 */
#ifndef PDU_FAULT_LOG_H_
#define PDU_FAULT_LOG_H_

#include "avionics_types.h"

#include <stddef.h>

namespace pdu {
namespace fault_log {

enum class Code : uint8_t {
  kNone                = 0U,
  kRailFaultFlagged    = 1U,
  kSoftwareInstantOc   = 2U,
  kOver100AOneMinute   = 3U,
  kOver80AThreeMinutes = 4U,
  kMosfetNtcOverTemp   = 5U,
  kHostRequestedReset  = 6U,
  kWatchdogReset       = 7U,
};

struct Record {
  bool     valid;
  uint16_t sequence;
  Code     code;
  Rail     rail;
  uint16_t status_word;
  uint16_t diag_word;
  uint32_t fault_uptime_ms;
  uint32_t unix_time_s;
  uint32_t reset_flags;
  uint16_t reset_count;
};

Status init();
void setUnixTime(uint32_t unix_time_s);
Record last();
size_t count();
size_t capacity();
size_t copyLatest(Record* out, size_t max_records);
uint32_t bootResetFlags();
uint16_t resetCount();
void clear();
void record(Code code, Rail rail, uint16_t status_word, uint16_t diag_word);
void recordAndReset(Code code, Rail rail, uint16_t status_word, uint16_t diag_word);

}  /* namespace fault_log */
}  /* namespace pdu */

#endif /* PDU_FAULT_LOG_H_ */
