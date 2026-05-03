/* =============================================================================
 *  SerialSWO.h
 * -----------------------------------------------------------------------------
 *  Drop-in replacement for `Serial` that routes every byte to the ARM Cortex-M3
 *  ITM (Instrumentation Trace Macrocell) on stimulus port 0.  The data leaves
 *  the chip on the SWO pin (PB3 on the STM32F103) and is captured by the
 *  attached ST-Link / J-Link, then forwarded to the host through the same
 *  USB connection used for SWD programming.
 *
 *  Pros vs UART / USB CDC:
 *    - One cable: programming + console + breakpoints all share the ST-Link.
 *    - Zero CPU overhead - ITM_SendChar pushes directly to a hardware FIFO.
 *    - No driver setup on Windows; trace is delivered through the existing
 *      ST-Link debug interface.
 *
 *  Pre-conditions:
 *    1) The ST-Link's SWO pin must be physically wired to MCU PB3.
 *    2) PB3 must NOT be reused for GPIO (we keep it idle in our pinmap).
 *    3) A debug session (or OpenOCD with `tpiu config`) must be active to
 *       enable TPIU/ITM and to consume the SWO bytes.  Without this, calls
 *       still complete but the FIFO is discarded - nothing is lost beyond
 *       the trace itself.
 *
 *  Style: this header purposely sticks to the Arduino `Print` interface so
 *  it can be aliased to `Serial` via the project-wide console.h shim, and
 *  every existing `Serial.print(...)` call works unchanged.
 * =============================================================================
 */
#ifndef PDU_SERIAL_SWO_H_
#define PDU_SERIAL_SWO_H_

#include <Arduino.h>
#include <Print.h>

class SerialSWOClass : public Print {
 public:
  SerialSWOClass() = default;

  /** Optionally enable TPIU + ITM from the firmware itself.  In most cases
   *  the debugger does this for us, but having a fallback path makes the
   *  trace work even when the firmware was started outside any debug
   *  session.  The function is idempotent.                                  */
  void begin(uint32_t /*baud*/ = 0U) {
    enableTrace();
  }

  /** Mimic HardwareSerial::operator bool() so the existing
   *  `while (!Serial)` boot wait doesn't block.                              */
  explicit operator bool() const { return true; }

  void end()    {}
  void flush()  {}

  size_t write(uint8_t b) override {
#if defined(__CORTEX_M) && (__CORTEX_M >= 3)
    (void)ITM_SendChar(b);
#else
    (void)b;
#endif
    return 1U;
  }

  using Print::write;

 private:
  static void enableTrace();
};

extern SerialSWOClass SerialSWO;

#endif /* PDU_SERIAL_SWO_H_ */
