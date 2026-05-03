/* =============================================================================
 *  console.h - Firmware debug / telemetry console abstraction.
 * -----------------------------------------------------------------------------
 *  This header textually redirects every `Serial.print(...)` call in the PDU
 *  firmware to the flight console backend:
 *
 *      PDU_USE_SWO=1 -> SerialSWO  (ITM trace on PB3, ST-Link SWO)
 *
 *  Include this header AFTER <Arduino.h> in any .cpp that prints to Serial.
 *  The framework's own translation units do not include this file, so the
 *  HardwareSerial object remains intact for them - we only rename the
 *  identifier our compilation sees.
 * =============================================================================
 */
#ifndef PDU_CONSOLE_H_
#define PDU_CONSOLE_H_

#include <Arduino.h>

#if !defined(PDU_USE_SWO)
#  error "console.h: the flight build requires PDU_USE_SWO"
#endif

#include <SerialSWO.h>
#undef  Serial
#define Serial SerialSWO

#endif /* PDU_CONSOLE_H_ */
