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
    /* Bounded substitute for the unbounded CMSIS ITM_SendChar.
     *
     * The CMSIS implementation does
     *     while (ITM->PORT[0U].u32 == 0UL) { __NOP(); }
     * which spins forever when nobody is draining the SWO/TPIU FIFO -
     * exactly the case when the firmware boots without an ST-Link
     * (DBGMCU is power-gated, the TPIU never empties the 32-entry FIFO,
     * and the very first burst of Serial.print blocks the CPU until the
     * IWDG resets it).  That triggers a permanent reset loop because the
     * banner-print on the next boot re-blocks the moment the loop hits
     * Serial.print.
     *
     * We replace it with a bounded wait: try briefly for FIFO space, and
     * if none appears within a fraction of a SWO byte time, drop the
     * character on the floor.  The chip stays alive, the IWDG keeps
     * getting kicked, and trace just goes silent when the host can't
     * keep up - which is the right policy for a flight controller.   */
    if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&
        ((ITM->TER & 1UL)               != 0UL)) {
      /* The SWO FIFO is 32 entries, draining at the TPIU bit-rate
       * (~5 us / byte at 2 Mbps NRZ).  A whole-FIFO drain is therefore
       * ~150 us, plus margin for ST-Link host buffering.  We give the
       * write up to ~3 ms of busy-wait at 72 MHz before dropping the
       * byte.  3 ms is small relative to the 2 s healthy-loop budget, so
       * the IWDG cannot fire because of it; but it is plenty to outlast
       * any normal back-pressure spike, which kills the scrambled-text
       * problem we used to get when the 64-iteration guard expired
       * mid-burst and threw bytes on the floor.                          */
      uint32_t guard = 200000U;
      while ((ITM->PORT[0U].u32 == 0UL) && (--guard != 0U)) {
        __NOP();
      }
      if (guard != 0U) {
        ITM->PORT[0U].u8 = b;
      }
    }
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
