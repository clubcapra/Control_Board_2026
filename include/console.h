/* =============================================================================
 *  console.h - Firmware debug / telemetry console abstraction.
 * -----------------------------------------------------------------------------
 *  This header textually redirects every `Serial.print(...)` call in the PDU
 *  firmware to the configured console backend:
 *
 *      PDU_USE_SWO=1   -> SerialSWO   (ITM trace on PB3, ST-Link SWO).
 *                         REQUIRES an active debug session on the ST-Link
 *                         that is draining the SWO/TPIU FIFO at runtime.
 *                         Without that drainage, ITM_SendChar() spins
 *                         forever inside its
 *                            while (ITM->PORT[0U].u32 == 0UL) {}
 *                         once the FIFO fills, which trips the IWDG and
 *                         puts the board into an infinite reset loop.
 *
 *      PDU_USE_SWO unset -> PduSilentSerial  (drop-in stub, all writes
 *                         are no-ops).  Use this for any binary intended
 *                         to run standalone (no ST-Link plugged in), so
 *                         the firmware never depends on a debugger and
 *                         never enables the ITM trace at all.
 *
 *  Include this header AFTER <Arduino.h> in any .cpp that prints to Serial.
 *  The framework's own translation units do not include this file, so the
 *  framework `HardwareSerial Serial` symbol remains intact for them - we
 *  only rename the identifier *our* compilation sees.
 * =============================================================================
 */
#ifndef PDU_CONSOLE_H_
#define PDU_CONSOLE_H_

#include <Arduino.h>

#if defined(PDU_USE_SWO)
#  include <SerialSWO.h>
#endif

#undef  Serial

/* Silent stub: matches the subset of the Arduino Print API that the firmware
 * actually calls.  Templated print/println swallow every overload (char,
 * const char*, integers, floats, FlashStringHelper, ...) without any I/O.   */
class PduSilentConsole {
 public:
  void begin(unsigned long /*baud*/) {}
  void flush() {}
  explicit operator bool() const { return true; }

  template <typename... Args>
  void print(Args...) {}

  template <typename... Args>
  void println(Args...) {}
};

#if defined(PDU_USE_SWO) && \
    !(defined(PDU_API_DEBUG_ONLY) && !defined(PDU_API_DEBUG_SOURCE))
/* Real SWO console: flight image with debugger attached, OR the
 * control_api.cpp translation unit in the API-debug image (which sets
 * PDU_API_DEBUG_SOURCE so its own RX/TX prints still come out on SWO). */
#define Serial SerialSWO
#else
/* Silent: either standalone build (PDU_USE_SWO unset) or a non-API TU in
 * the API-debug image.                                                      */
static PduSilentConsole PduSilentSerial;
#define Serial PduSilentSerial
#endif

#endif /* PDU_CONSOLE_H_ */
