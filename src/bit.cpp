/* =============================================================================
 *  bit.cpp - Built-In Test routines.
 * =============================================================================
 */
#include "bit.h"
#include "avionics_config.h"

#include <Arduino.h>
#include <string.h>

#include "console.h"

namespace pdu {
namespace bit {

namespace {

/** Walking-1 / walking-0 SRAM test on a fixed buffer.  Cheap but catches
 *  most stuck-bit faults in the data RAM region we will actually use.    */
bool sramWalkingPattern() {
  static uint32_t scratch[16];
  for (uint8_t bit = 0U; bit < 32U; ++bit) {
    const uint32_t pat_one  = 1UL << bit;
    const uint32_t pat_zero = ~pat_one;
    for (size_t i = 0U; i < (sizeof(scratch) / sizeof(scratch[0])); ++i) {
      scratch[i] = pat_one;
    }
    for (size_t i = 0U; i < (sizeof(scratch) / sizeof(scratch[0])); ++i) {
      if (scratch[i] != pat_one) {
        return false;
      }
      scratch[i] = pat_zero;
    }
    for (size_t i = 0U; i < (sizeof(scratch) / sizeof(scratch[0])); ++i) {
      if (scratch[i] != pat_zero) {
        return false;
      }
    }
  }
  return true;
}

bool checkRailPresence(rail::Controller& r) {
  if (!r.isPresent()) {
    Serial.print(F("[PBIT FAIL] rail "));
    Serial.print(railToString(r.config().id));
    Serial.println(F(" not present"));
    return false;
  }
  uint16_t status = 0U;
  if (!r.device().readStatusWord(status)) {
    Serial.print(F("[PBIT FAIL] rail "));
    Serial.print(railToString(r.config().id));
    Serial.println(F(" STATUS_WORD read failed"));
    return false;
  }
  return true;
}

bool checkRailIdentity(rail::Controller& r) {
  char model[16] = {0};
  if (!r.device().readMfrModel(model, sizeof(model))) {
    /* Some bootloaders return a blank string the first time - retry once. */
    delay(2);
    if (!r.device().readMfrModel(model, sizeof(model))) {
      Serial.print(F("[PBIT WARN] rail "));
      Serial.print(railToString(r.config().id));
      Serial.println(F(" MFR_MODEL unreadable"));
      return false;
    }
  }
  Serial.print(F("[PBIT INFO] rail "));
  Serial.print(railToString(r.config().id));
  Serial.print(F(" MFR_MODEL='"));
  Serial.print(model);
  Serial.println(F("'"));

  /* The LM5066H family always reports a model that starts with "LM5066".  */
  if (strncmp(model, "LM5066", 6) != 0) {
    Serial.println(F("[PBIT FAIL] unexpected MFR_MODEL signature"));
    return false;
  }
  return true;
}

bool quickCbit(rail::Controller& r) {
  if (!r.isPresent()) {
    /* CBIT is a live health check for devices that are actually populated /
     * responding.  Absence is already visible in telemetry and PBIT logs.   */
    return true;
  }
  uint8_t cap = 0U;
  if (!r.device().readCapability(cap)) {
    return false;
  }
  /* Capability byte is non-zero on a healthy LM5066H1 (PEC + SMBA bits).  */
  return (cap != 0U) && (cap != 0xFFU);
}

}  // namespace

Status runPbit(rail::Controller& r48, rail::Controller& r24,
               rail::Controller& r12, Report& report) {
  report           = Report{};
  report.last_run_ms = millis();

  bool ok = true;

  /* 1.  RAM walking pattern.                                              */
  ++report.tests_run;
  if (!sramWalkingPattern()) {
    Serial.println(F("[PBIT FAIL] SRAM walking-bit"));
    ++report.tests_failed;
    ok = false;
  }

  /* 2.  Rail presence + identity.                                         */
  rail::Controller* rails[kRailCount] = {&r48, &r24, &r12};
  for (size_t i = 0U; i < kRailCount; ++i) {
    ++report.tests_run;
    if (!checkRailPresence(*rails[i])) {
      ++report.tests_failed;
      ok = false;
    }
    ++report.tests_run;
    if (!checkRailIdentity(*rails[i])) {
      ++report.tests_failed;
      ok = false;
    }
  }

  /* 3.  PMBus revision read-back - simple but proves the bus is two-way.  */
  for (size_t i = 0U; i < kRailCount; ++i) {
    if (!rails[i]->isPresent()) {
      continue;
    }
    ++report.tests_run;
    uint8_t rev = 0U;
    if (!rails[i]->device().readPmbusRevision(rev) || rev == 0xFFU) {
      Serial.print(F("[PBIT FAIL] PMBUS_REVISION on "));
      Serial.println(railToString(rails[i]->config().id));
      ++report.tests_failed;
      ok = false;
    }
  }

  report.all_passed = ok;
  Serial.print(F("[PBIT] tests_run="));
  Serial.print(report.tests_run);
  Serial.print(F(" failed="));
  Serial.print(report.tests_failed);
  Serial.println(ok ? F(" RESULT=PASS") : F(" RESULT=FAIL"));
  return ok ? Status::kOk : Status::kFault;
}

Status runCbit(rail::Controller& r48, rail::Controller& r24,
               rail::Controller& r12, Report& report) {
  report.last_run_ms = millis();
  rail::Controller* rails[kRailCount] = {&r48, &r24, &r12};
  bool ok = true;
  for (size_t i = 0U; i < kRailCount; ++i) {
    ++report.tests_run;
    if (!quickCbit(*rails[i])) {
      ++report.tests_failed;
      ok = false;
    }
  }
  report.all_passed = ok;
  return ok ? Status::kOk : Status::kFault;
}

}  /* namespace bit */
}  /* namespace pdu */
