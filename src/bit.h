/* =============================================================================
 *  bit.h - Built-In Test framework (PBIT + CBIT)
 * -----------------------------------------------------------------------------
 *  Avionics standards (DO-178C, MIL-HDBK-2155) require any safety-critical
 *  computer to run at three categories of self-test:
 *
 *      PBIT (Power-On BIT)        : exhaustive, runs once at boot before any
 *                                   external load is energised.
 *      CBIT (Continuous BIT)      : runs in the background during normal
 *                                   operation, opportunistically samples
 *                                   internal state.
 *      IBIT (Initiated BIT)       : invoked on demand from a maintenance
 *                                   console.
 *
 *  This module provides the entry points; individual tests are registered
 *  internally.
 * =============================================================================
 */
#ifndef PDU_BIT_H_
#define PDU_BIT_H_

#include "avionics_types.h"
#include "rail.h"

namespace pdu {
namespace bit {

/** Aggregated results structure.                                           */
struct Report {
  bool     all_passed;
  uint32_t tests_run;
  uint32_t tests_failed;
  uint32_t last_run_ms;
};

/**
 *  Power-On Built-In Test.
 *  Runs to completion before any rail is enabled.  All loads remain de-
 *  energised until this call returns Status::kOk.
 *
 *  Tests performed:
 *    - SRAM walking-bit pattern over a small known buffer
 *    - LM5066H1 presence and MFR_ID match for each rail
 *    - PMBus read-back of OPERATION / STATUS / DIAG
 *    - GPIO direction and pull configuration sanity
 */
Status runPbit(rail::Controller& r48, rail::Controller& r24,
               rail::Controller& r12, Report& report);

/**
 *  Continuous Built-In Test.
 *  Runs at every kBitContinuous_ms.  Lighter than PBIT - mostly checks
 *  that devices still ACK and that no fault flag is stuck.
 */
Status runCbit(rail::Controller& r48, rail::Controller& r24,
               rail::Controller& r12, Report& report);

}  /* namespace bit */
}  /* namespace pdu */

#endif /* PDU_BIT_H_ */
