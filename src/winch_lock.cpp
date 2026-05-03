/* =============================================================================
 *  winch_lock.cpp - TPS2HB16AQPWPRQ1 winch-lock high-side outputs.
 * =============================================================================
 */
#include "winch_lock.h"

#include "avionics_config.h"

#include <Arduino.h>

namespace pdu {
namespace winch_lock {

namespace {

bool s_initialised = false;
bool s_lock1_on = false;
bool s_lock2_on = false;

void writeLock1(bool on) {
  digitalWrite(cfg::kPin_WinchLock1_EN, on ? HIGH : LOW);
  s_lock1_on = on;
}

void writeLock2(bool on) {
  digitalWrite(cfg::kPin_WinchLock2_EN, on ? HIGH : LOW);
  s_lock2_on = on;
}

}  // namespace

Status init() {
  if (s_initialised) {
    return Status::kOk;
  }
  pinMode(cfg::kPin_WinchLock1_EN, OUTPUT);
  pinMode(cfg::kPin_WinchLock2_EN, OUTPUT);
  writeLock1(false);
  writeLock2(false);
  s_initialised = true;
  return Status::kOk;
}

Status set(Channel channel, bool on) {
  if (!s_initialised) {
    return Status::kNotInit;
  }
  if (channel == Channel::kLock1) {
    writeLock1(on);
    return Status::kOk;
  }
  if (channel == Channel::kLock2) {
    writeLock2(on);
    return Status::kOk;
  }
  if (channel == Channel::kAll) {
    return setAll(on);
  }
  return Status::kParam;
}

Status setAll(bool on) {
  if (!s_initialised) {
    return Status::kNotInit;
  }
  writeLock1(on);
  writeLock2(on);
  return Status::kOk;
}

Telemetry telemetry() {
  Telemetry t = {};
  t.lock1_on = s_lock1_on;
  t.lock2_on = s_lock2_on;
  return t;
}

}  /* namespace winch_lock */
}  /* namespace pdu */
