/* =============================================================================
 *  rail.h
 * -----------------------------------------------------------------------------
 *  Per-rail controller wrapping a single LM5066H1 hot-swap device.  Implements
 *  a small state machine and a layered protection strategy:
 *
 *    - Hardware  : the LM5066H1 internal circuit-breaker is configured to
 *                  fire at the absolute peak current (instant cutoff in ~us).
 *    - Software  : on 48 V we additionally enforce a time-windowed protection
 *                  curve:
 *                    >= 80 A  for >  3 min  -> trip
 *                    >= 100 A for >  1 min  -> trip
 *                    >= 150 A                -> trip instantly (HW + SW)
 *                  On 24 V / 12 V the spec is "above N amps -> instant cut",
 *                  so the same threshold is enforced both in HW and as a
 *                  software backup in case the device is mis-programmed.
 *
 *  All I/O with the LM5066H1 goes through the existing vendor-style class so
 *  every one of the 80+ PMBus / SMBus commands remains accessible to upper
 *  layers (telemetry, ground-station diagnostics, etc.).
 * =============================================================================
 */
#ifndef PDU_RAIL_H_
#define PDU_RAIL_H_

#include "avionics_types.h"
#include "fault_log.h"

#include <LM5066H1.h>

namespace pdu {
namespace rail {

/** Operational state of a single rail.                                       */
enum class State : uint8_t {
  kBoot      = 0U,  /* before init() called                                   */
  kAbsent    = 1U,  /* device did not ACK its address                         */
  kReady     = 2U,  /* configured but OUTPUT off                              */
  kRunning   = 3U,  /* OUTPUT on, no fault                                    */
  kWarning   = 4U,  /* a warning bit set, output still on                     */
  kTripped   = 5U,  /* protection fired, OUTPUT commanded off                 */
  kLatched   = 6U,  /* repeated trips, requires explicit reset                */
};

/** Static description of a rail.  Built once at boot, never modified.       */
struct RailConfig {
  Rail        id;                 /* logical rail identifier                  */
  uint8_t     i2c_address;        /* 7-bit PMBus address                      */
  double      rsns_mohm;          /* sense-resistor value, milli-ohm          */
  uint32_t    smbus_clock_hz;     /* bus speed for this LM5066H1              */

  /* ---------- Hardware breaker configuration ----------
   *  All rails operate in pure BREAKER mode (current-limiting disabled).
   *  The LM5066H1 exposes three layered overcurrent levels:
   *
   *    OC1 = VCBL1 - moderate overcurrent + slow blanking timer (up to 95 ms)
   *    OC2 = VCBL2 - higher overcurrent  + fast blanking timer (up to 95 ms)
   *    OC3 = VCB   - circuit breaker, instant trip (~100 us)
   *
   *  When any timer expires, GATEs turn off (breaker mode).  Setting a
   *  timer to 0 ms effectively disables that layer.
   */
  uint8_t     vcl_code;           /* DEVICE_SETUP2[5:3], 1..7  (VCL setting)  */
  uint8_t     ocb1_threshold;     /* DS3[1:0]  0=1.25x 1=1.5x 2=1.75x 3=2x VCL*/
  uint8_t     ocb1_timer;         /* OC_BLANKING[3:0] 0..15  (0=disabled)     */
  uint8_t     ocb2_threshold;     /* DS3[3:2]  0=1.5x 1=1.75x 2=2x 3=2.25x VCL*/
  uint8_t     ocb2_timer;         /* OC_BLANKING[7:4] 0..15  (0=disabled)     */
  double      cb_ratio;           /* 1.2, 2.0, 3.0 or 4.0  (VCB / VCL)        */
  double      hw_trip_a;          /* expected VCB trip current, A             */

  /* ---------- Software-side protection ---------- */

  /* Time-windowed software protection (48 V only - others leave at 0).      */
  double      sw_warn_3min_a;
  double      sw_warn_1min_a;
  uint32_t    sw_window_3min_ms;
  uint32_t    sw_window_1min_ms;

  /* Instant software back-up (when HW is misprogrammed or for 24 V/12 V).    */
  double      sw_instant_a;

  /* GPIO pin used as power-good feedback when has_pgood is true.             */
  uint32_t    pgood_pin;
  bool        has_pgood;

  /* VIN warning limits.                                                     */
  double      vin_ov_v;
  double      vin_uv_v;
};

/**
 *  Runtime state of a rail.  All members are private to the .cpp - callers
 *  see only the const accessors below.  This header exposes only the
 *  configuration descriptor and the public surface.
 */
class Controller {
 public:
  /** Construct around a static configuration descriptor.                    */
  explicit Controller(const RailConfig& cfg);
  Controller(const RailConfig& cfg, LM5066H1Bus& bus);

  /** Probe the device, configure thresholds, leave OUTPUT off.              */
  Status init();

  /** Command OUTPUT ON.  Refuses if state is kAbsent / kLatched.            */
  Status enable();

  /** Command OUTPUT OFF.  Always accepted - safe failure path.              */
  Status disable();

  /** Reset latched fault and re-arm the protection logic.                   */
  Status clearLatch();

  /** Periodic protection cadence: must be called every kProtectionSamplePeriod_ms. */
  void   tick();

  /** Build a snapshot of the current rail state for telemetry consumers.    */
  void   buildTelemetry(RailTelemetry& out) const;

  /** Refresh the LM5066H1 internal black-box RAM/EEPROM snapshot.            */
  Status refreshBlackBoxMemory();

  /** Const accessors used by BIT and the supervisor.                        */
  State  state()       const { return state_; }
  bool   isPresent()   const { return present_; }
  uint32_t faultCount() const { return fault_count_; }
  const RailConfig& config() const { return cfg_; }

  /** Direct (read-only) access to the underlying device for advanced use
   *  cases such as ground-station diagnostics that need to reach any of the
   *  80+ PMBus commands. */
  LM5066H1& device()             { return device_; }
  const LM5066H1& device() const { return device_; }

 private:
  /** Helper: program every fault / warn limit register on the device.       */
  Status configureDevice();

  /** Helper: convert a measured VAUX voltage into MOSFET NTC temperature.   */
  bool   ntcCelsiusFromVaux(double vaux_v, double& celsius) const;

  /** Update the time-window accumulators and trip if necessary.             */
  void   evaluateTimeWindowed(double iin_a, uint32_t dt_ms);

  /** Force the rail OFF on a protection event and record the cause.         */
  void   tripFromProtection(const char* cause, fault_log::Code code);

  const RailConfig& cfg_;
  LM5066H1          device_;
  State             state_;
  bool              present_;
  uint32_t          fault_count_;
  uint32_t          last_tick_ms_;

  /* Time-windowed accumulators (48 V only, idle on other rails).            */
  uint32_t accum_3min_ms_;
  uint32_t accum_1min_ms_;

  /* Cached telemetry sampled in tick().                                     */
  double   last_vin_v_;
  double   last_vout_v_;
  double   last_vaux_v_;
  double   last_iin_a_;
  double   last_pin_w_;
  bool     peak_valid_;
  double   peak_vin_v_;
  double   peak_iin_a_;
  double   peak_pin_w_;
  double   last_die_temp_c_;
  double   last_ntc_c_;
  uint16_t last_status_word_;
  uint16_t last_diag_word_;
  uint16_t last_status_mfr_specific2_;
  uint8_t  last_operation_raw_;
  uint8_t  last_status_input_;
  uint8_t  last_status_cml_;
  uint8_t  last_status_mfr_specific_;
  uint8_t  last_wd_plb_timer_;
  bool     bb_valid_;
  uint8_t  bb_config_;
  uint8_t  bb_timer_;
  uint8_t  bb_ram_len_;
  uint8_t  bb_eeprom_len_;
  uint8_t  bb_ram_[kLm5066BlackBoxBytes];
  uint8_t  bb_eeprom_[kLm5066BlackBoxBytes];
  uint32_t last_bb_refresh_ms_;
  uint32_t last_on_recovery_ms_;
  uint32_t pending_on_recovery_ms_;
  uint32_t post_on_observe_ms_;
  bool     pending_on_recovery_;
  bool     post_on_observe_;
  bool     desired_on_;
};

}  /* namespace rail */
}  /* namespace pdu */

#endif /* PDU_RAIL_H_ */
