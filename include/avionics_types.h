/* =============================================================================
 *  avionics_types.h
 * -----------------------------------------------------------------------------
 *  Common types, error codes and bit-field helpers shared across all firmware
 *  modules.  All enums use a fixed underlying type (uint8_t / uint16_t) to
 *  avoid implementation-defined sizing - one of the MISRA-C 2012 mandatory
 *  rules ([Rule 10.1] essential type model).
 * =============================================================================
 */
#ifndef PDU_AVIONICS_TYPES_H_
#define PDU_AVIONICS_TYPES_H_

#include <stdint.h>
#include <stddef.h>

namespace pdu {

/* ---------------------------------------------------------------------------
 *  Standard return code used everywhere a function may fail.  Returning an
 *  enum (instead of bool / errno) gives the static analyser an easy hook to
 *  flag missing error checks.
 * ---------------------------------------------------------------------------*/
enum class Status : uint8_t {
  kOk            = 0U,  /* nominal completion                                  */
  kBusError      = 1U,  /* I2C / SMBus low-level failure                      */
  kTimeout       = 2U,  /* peripheral or peer did not respond in time         */
  kParam         = 3U,  /* invalid argument                                    */
  kNotPresent    = 4U,  /* device did not ACK its address                      */
  kRange         = 5U,  /* value out of acceptable range                       */
  kPecMismatch   = 6U,  /* PEC / CRC mismatch on SMBus frame                   */
  kFault         = 7U,  /* peripheral reported a hardware fault                */
  kNotInit       = 8U,  /* module used before it was initialised               */
  kInternal      = 9U,  /* unexpected internal error - should never happen     */
};

/* ---------------------------------------------------------------------------
 *  Identifies which physical rail an action targets.  Used to pass a context
 *  through generic helpers without coupling them to LM5066H1 instances.
 * ---------------------------------------------------------------------------*/
enum class Rail : uint8_t {
  k48V = 0U,
  k24V = 1U,
  k12V = 2U,
  kCount      /* sentinel - keep last */
};

/* Number of rails managed by the firmware.  Never use a magic 3.             */
constexpr size_t kRailCount = static_cast<size_t>(Rail::kCount);

/* Raw bytes retained from the LM5066H1 internal black-box memories.           */
constexpr size_t kLm5066BlackBoxBytes = 16U;

/* ---------------------------------------------------------------------------
 *  Live operating mode of the supervisor.  Public so that other modules (BIT,
 *  LED driver, telemetry) can render the current mode in their own way.
 * ---------------------------------------------------------------------------*/
enum class SupervisorMode : uint8_t {
  kBoot      = 0U,  /* power-on, peripherals being initialised                */
  kPbit      = 1U,  /* Power-On Built-In Test running                          */
  kNominal   = 2U,  /* normal operation                                        */
  kDegraded  = 3U,  /* one or more rails reported a recoverable fault          */
  kFault     = 4U,  /* unrecoverable fault, rails latched off                  */
  kEStop     = 5U,  /* Emergency stop asserted, all rails commanded off        */
};

/* ---------------------------------------------------------------------------
 *  Aggregated status of a single rail.  The supervisor publishes a snapshot
 *  every CBIT cycle; the LED driver and telemetry modules consume it.
 * ---------------------------------------------------------------------------*/
struct RailTelemetry {
  Rail     rail;
  bool     present;        /* device ACKs its address                          */
  bool     output_on;      /* OPERATION bit                                    */
  uint8_t  operation_raw;  /* raw OPERATION register                           */
  bool     pgood_pin;      /* GPIO power-good level                            */
  double   vin_v;
  double   vout_v;
  double   vaux_v;         /* VAUX pin voltage used for external NTC          */
  double   iin_a;
  double   pin_w;
  bool     peak_valid;     /* peak fields are valid since this boot            */
  double   peak_vin_v;     /* VIN at max instantaneous input power             */
  double   peak_iin_a;     /* IIN at max instantaneous input power             */
  double   peak_pin_w;     /* max instantaneous input power since boot         */
  double   die_temp_c;     /* internal LM5066H1 diode temperature              */
  double   ntc_temp_c;     /* external MOSFET NTC                              */
  uint16_t status_word;
  uint16_t diag_word;
  uint16_t status_mfr_specific2; /* watchdog/SC/retry/power-cycle status       */
  uint8_t  status_input;          /* STATUS_INPUT raw                          */
  uint8_t  status_cml;            /* STATUS_CML raw                            */
  uint8_t  status_mfr_specific;   /* STATUS_MFR_SPECIFIC raw                   */
  uint8_t  wd_plb_timer;    /* watchdog timer + power-limit blanking raw     */
  uint32_t fault_count;    /* monotonic, never reset until boot                */
  bool     bb_valid;       /* LM5066H1 black-box was read successfully         */
  uint8_t  bb_config;      /* MFR_BB_CONFIG raw                                */
  uint8_t  bb_timer;       /* MFR_BB_TIMER raw                                 */
  uint8_t  bb_ram_len;     /* valid bytes in bb_ram                            */
  uint8_t  bb_eeprom_len;  /* valid bytes in bb_eeprom                         */
  uint8_t  bb_ram[kLm5066BlackBoxBytes];
  uint8_t  bb_eeprom[kLm5066BlackBoxBytes];
};

/* ---------------------------------------------------------------------------
 *  Compile-time helper: convert a `Status` into a printable string.  Kept
 *  inline + constexpr so it costs nothing in flash if unused.
 * ---------------------------------------------------------------------------*/
constexpr const char* statusToString(Status s) {
  return (s == Status::kOk)            ? "OK"
       : (s == Status::kBusError)      ? "BUS_ERROR"
       : (s == Status::kTimeout)       ? "TIMEOUT"
       : (s == Status::kParam)         ? "PARAM"
       : (s == Status::kNotPresent)    ? "NOT_PRESENT"
       : (s == Status::kRange)         ? "RANGE"
       : (s == Status::kPecMismatch)   ? "PEC_MISMATCH"
       : (s == Status::kFault)         ? "FAULT"
       : (s == Status::kNotInit)       ? "NOT_INIT"
       : "INTERNAL";
}

constexpr const char* railToString(Rail r) {
  return (r == Rail::k48V) ? "48V"
       : (r == Rail::k24V) ? "24V"
       : (r == Rail::k12V) ? "12V" : "?";
}

constexpr const char* modeToString(SupervisorMode m) {
  return (m == SupervisorMode::kBoot)     ? "BOOT"
       : (m == SupervisorMode::kPbit)     ? "PBIT"
       : (m == SupervisorMode::kNominal)  ? "NOMINAL"
       : (m == SupervisorMode::kDegraded) ? "DEGRADED"
       : (m == SupervisorMode::kFault)    ? "FAULT"
       : (m == SupervisorMode::kEStop)    ? "ESTOP" : "?";
}

}  /* namespace pdu */

#endif /* PDU_AVIONICS_TYPES_H_ */
