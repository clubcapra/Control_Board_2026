/* =============================================================================
 *  SerialSWO.cpp - Implementation of the SWO/ITM console.
 * -----------------------------------------------------------------------------
 *  STM32F103 SWO ground rules
 *  --------------------------
 *  PB3 carries TRACESWO (the SWO bit stream) when the SWJ-DP is in either
 *
 *     SWJ_CFG = 0b000   -> Full SWJ (default after reset).  TRACESWO live.
 *     SWJ_CFG = 0b001   -> Full SWJ without NJTRST.         TRACESWO live.
 *
 *  But the moment the Arduino HAL calls __HAL_AFIO_REMAP_SWJ_NOJTAG() to free
 *  one of the JTAG pins (PA15/PB3/PB4), it writes SWJ_CFG = 0b010
 *  (JTAG-DP DISABLED, SW-DP ENABLED) which **also disables the TRACESWO
 *  function**.  SWD keeps working (OpenOCD still connects), but PB3 goes
 *  silent and `swo.log` stays empty - exactly what we observed in the field.
 *
 *  This file owns SWO, so it is the natural place to force the SWJ-DP into
 *  the only F1 mode that satisfies all of:
 *     - SWD active           (so the ST-Link can flash and debug)
 *     - PB4 free for GPIO    (we drive LED_Bras from there)
 *     - PB3 still TRACESWO   (so this driver can talk)
 *
 *  i.e. SWJ_CFG = 0b001.  We re-apply the value after every Arduino-induced
 *  pinMode() so a stray late call to a JTAG-shared pin cannot turn the trace
 *  back off.  enableTrace() is therefore safe to call from anywhere and as
 *  often as needed; it is idempotent.
 * =============================================================================
 */
#include "SerialSWO.h"

#if defined(STM32F1xx)
#  include "stm32f1xx.h"
#endif

SerialSWOClass SerialSWO;

#if defined(STM32F1xx)
/**
 * Force SWJ_CFG to 0b001 (Full SWJ without NJTRST).
 *
 * The AFIO peripheral clock must be enabled before the AFIO->MAPR write
 * takes effect.  We also unlock the AFIO->MAPR register by performing a
 * read-modify-write, masking only the SWJ_CFG bits so any other remap
 * the user code might rely on (USART, I2C, timers, ...) is preserved.
 */
static inline void forceSWO_NJTRST() {
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  uint32_t mapr = AFIO->MAPR;
  mapr &= ~AFIO_MAPR_SWJ_CFG_Msk;
  mapr |=  AFIO_MAPR_SWJ_CFG_NOJNTRST;   /* 0b001 */
  AFIO->MAPR = mapr;
}
#endif

void SerialSWOClass::enableTrace() {
#if defined(__CORTEX_M) && (__CORTEX_M >= 3) && defined(DBGMCU)
  /* (1) Force SWJ-DP into the only mode that keeps PB3 = TRACESWO while
   *     leaving PB4 free for GPIO.  Must be done BEFORE the LED driver
   *     calls pinMode(PB4, ...), and re-applied AFTER it as well.        */
#if defined(STM32F1xx)
  forceSWO_NJTRST();
#endif

  /* (2) Enable the trace-pin output on the DBGMCU control register and
   *     turn on the Cortex-M3 trace clock.  Without these bits, TPIU
   *     can be configured fine but no signal will ever reach PB3.       */
  DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* (3) TPIU async (NRZ) mode at 2 Mbit/s out of a 72 MHz core clock:
   *        prescaler = (72e6 / 2e6) - 1 = 35.
   *     Must match the OpenOCD `tpiu config` line in tools/swo_capture.cfg.*/
  TPI->ACPR = 35U;
  TPI->SPPR = 2U;                /* 2 = NRZ                                 */
  TPI->FFCR = 0x00000100U;       /* TPIU formatter disabled, sync ok        */

  /* (4) ITM unlock + global enable + stimulus port 0 enable.             */
  ITM->LAR = 0xC5ACCE55U;
  ITM->TCR = (1U << 0) |         /* ITMENA: enable ITM                       */
             (1U << 3) |         /* TXENA : enable hardware events           */
             (1U << 4);          /* SYNCENA: enable sync packets             */
  ITM->TER = 0x00000001U;        /* Enable stimulus port 0                   */
  ITM->TPR = 0x0000000FU;        /* Unprivileged access to ports 0..7        */

  /* (5) Defensive re-write: in case any driver constructed before this
   *     point already triggered a SWJ_NOJTAG remap, restore the right
   *     mode.  This is the call you want to repeat at the end of setup(). */
#if defined(STM32F1xx)
  forceSWO_NJTRST();
#endif
#endif
}
