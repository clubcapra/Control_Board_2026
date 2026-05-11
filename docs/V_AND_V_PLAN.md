# V&V Plan (R5 / DO-178C residual)

> **Disclaimer.** This document is a *test-plan skeleton*.  It enumerates the
> requirements-based test cases that must be passed before the PDU image is
> acceptable for a DO-178C Level B / Level A submission.  The PDU project
> has **not** yet executed any of these tests, and the document is therefore
> a deliverable for future work, not a record of completed verification.

## 1. Scope

The PDU CSCI under verification is the firmware that runs on the
STM32F103C8T6 control board, configured by `PDU_FLIGHT_BUILD=1`.  The
out-of-flight builds (`bluepill_api_debug`, `bluepill_output_test`,
`bluepill_flight_nodebug`) are *test images* and are not part of the
flight CSCI; they have their own (lighter) test campaign.

## 2. Coverage objectives (DO-178C Level A)

For Level A, every requirement-based test must achieve **MC/DC (Modified
Condition/Decision Coverage)** on the source modules implementing the
requirement.  For Level B, decision coverage is sufficient.

| Level | Statement | Decision | MC/DC |
|---|---|---|---|
| C | required | optional | optional |
| B | required | required | optional |
| A | required | required | **required** |

The test cases below are sized for MC/DC coverage so the same campaign
satisfies any of the three levels.

## 3. Requirements-based test cases

Each test case below is keyed by the matching `[REQ-XXX]` tag in the
source.  Every case lists:
- **Setup**: hardware-in-the-loop fixture, voltage sources, instrumentation.
- **Stimulus**: input pattern.
- **Expected observable**: the externally-visible outcome the supervisor
  must produce.
- **Pass/Fail oracle**: how the test bench measures success.

### 3.1 Boot phase requirements

#### TC-BOOT-001 (covers REQ-BOOT-001 console-up)
- Setup: ST-Link attached, SWO trace receiver running.
- Stimulus: power-on reset; observe SWO from t=0.
- Expected: a `[BOOT-PHASE] 01-console elapsed_ms=N` line where 0 ≤ N ≤ 50.
- Oracle: SWO line present with elapsed_ms within budget.

#### TC-BOOT-002 (covers REQ-BOOT-002 + REQ-DIAG-001)
- Setup: same as TC-BOOT-001.
- Stimulus: power-on reset.
- Expected: a `[BOOT] reset_cause=0xNN warm=0 wdg=0 boot_count=1 ...` line
  with `canary=OK`, `image_crc=0xNNNNNNNN`.
- Oracle: parse the line; assert `canary == "OK"`, `image_crc != 0`.

#### TC-BOOT-003 (covers REQ-BOOT-003 actuator init)
- Setup: scope on PB4/PB5/PB8/PB9 + PB15/PA8.
- Stimulus: power-on reset.
- Expected: all six pins remain LOW from t=0 through t=10 s.
- Oracle: scope captures peak voltage on each pin <= 0.4 V.

#### TC-BOOT-010 (covers REQ-BOOT-010 hot-swap bus init)
- Setup: 3 LM5066H1 boards on I2C1 + bit-banged SMBus48.
- Stimulus: power-on reset.
- Expected: SWO trace contains `[SMBUS SCAN] ACK addr=0x41/0x43/0x52` once.
- Oracle: parse SWO; assert all 3 ACK lines present.

#### TC-BOOT-020 (covers REQ-BOOT-020 control API)
- Setup: I2C2 master attached on PB10/PB11.
- Stimulus: send `kPing` command from master after [BOOT-DONE] is seen.
- Expected: STM32 ACKs the address and returns the ping response.
- Oracle: master receives the expected payload.

#### TC-BOOT-030 (covers REQ-BOOT-030 PBIT)
- Setup: same as TC-BOOT-010.
- Stimulus: power-on reset.
- Expected: `[BOOT-PHASE] 08-pbit elapsed_ms=N rc=0` and a `[PBIT]` summary.
- Oracle: parse rc field, assert `rc == 0` if all rails present.

#### TC-BOOT-040 (covers REQ-BOOT-040 + REQ-PWR-100)
- Setup: scope on each LM5066H1 OUT pin.
- Stimulus: power-on reset; do NOT send any RG command.
- Expected: with `kHotswapApiOnly=true`, all rails remain de-energised
  until RG sends `kSetRailEnable`.
- Oracle: rail OUT < 0.5 V for 30 s after [BOOT-DONE] without RG command.

#### TC-BOOT-050 (covers REQ-BOOT-050 scheduler arm)
- Setup: SWO trace receiver.
- Stimulus: power-on reset.
- Expected: first `[CBIT]` and first `[PROT TICK]` events appear within
  the documented periods after `[BOOT-DONE]`.
- Oracle: timestamp delta within ±10% of `cfg::kBitContinuous_ms` and
  `cfg::kProtectionSamplePeriod_ms`.

### 3.2 Power requirements

#### TC-PWR-013 (covers REQ-PWR-013 48V instant trip @ 150 A)
- Setup: 48V rail loaded to 145 A then ramped to 155 A in <100 ms.
- Stimulus: ramp current.
- Expected: LM5066H1 hardware breaker fires; FET goes OFF within 1 ms of
  threshold crossing.
- Oracle: scope on FET drain.

#### TC-PWR-200 (covers REQ-PWR-200 disable-everything path)
- Setup: rails energised, both winch locks ON, winch motor spinning.
- Stimulus: assert E-Stop input.
- Expected: within 100 ms of E-Stop assert: rails OFF, locks OFF, winch
  in coast.
- Oracle: scope on each output pin + LM5066H1 PMBus telemetry.

### 3.3 LED + Lock requirements

#### TC-LED-001 (covers REQ-LED-001 LEDs OFF at boot)
- Same as TC-BOOT-003 with focus on PB4/PB5/PB8/PB9 only.

#### TC-LOCK-001 (covers REQ-LOCK-001 locks OFF at boot)
- Same as TC-BOOT-003 with focus on PB15/PA8 only.

### 3.4 Loop / scheduler requirements

#### TC-LOOP-001 (covers REQ-LOOP-001 E-Stop highest priority)
- Setup: rails energised by RG.
- Stimulus: assert E-Stop while a CBIT is mid-flight.
- Expected: rails de-energise before the CBIT completes.
- Oracle: telemetry shows CBIT abort + rail OFF in same iteration.

#### TC-LOOP-099 (covers REQ-LOOP-099 watchdog)
- Setup: nominal flight loop.
- Stimulus: artificially block the loop with a `while(1) {}` injected at a
  test-only debug entry-point.
- Expected: IWDG fires within `cfg::kIwdgTimeout_ms` of the block.
- Oracle: ground equipment sees a reset; the next `[BOOT]` line shows
  `wdg=1` and `boot_count` increments.

### 3.5 Diagnostic requirements

#### TC-DIAG-001 (covers REQ-DIAG-001 reset breadcrumb)
- Setup: cold-boot + warm-boot sequence (warm via RG `kResetDevice`).
- Stimulus: send `kResetDevice` after the loop has run for 10 s.
- Expected: next `[BOOT]` line shows `warm=1`, `boot_count >= 2`,
  `prev_alive_ms` ≈ 10000.
- Oracle: parse the line.

#### TC-DIAG-002 (covers REQ-DIAG-002 stack canary)
- Setup: special test image with a deep-recursion entry point that grows
  the stack until it overlaps `__bss_end__`.
- Stimulus: trigger the recursion.
- Expected: SWO emits `[FATAL] stack canary corrupted - awaiting IWDG reset`,
  followed by an IWDG reset, followed by `[BOOT] ... canary=BROKEN` on
  the very next boot.  Wait - actually since the canary is re-armed in
  the breadcrumb phase, the *next* boot will see canary=OK.  The TEST
  observation point is therefore the SWO `[FATAL]` line itself, BEFORE
  the reset.
- Oracle: SWO contains `[FATAL] stack canary corrupted` once.

#### TC-DIAG-003 (covers REQ-DIAG-003 image CRC)
- Setup: nominal cold boot.
- Stimulus: capture the `[BOOT]` line's `image_crc=0x...` field.
- Expected: matches the off-line CRC32 computed by:
  ```
  python -c "import zlib; print(hex(zlib.crc32(open('firmware.bin','rb').read())))"
  ```
- Oracle: hex equality.

## 4. MC/DC coverage measurement

Coverage is measured with **gcov + lcov** during host-target shadow
execution, then merged with the on-target campaign results.  Process:

1. Build the unit-test variants of every module with `-fprofile-arcs
   -ftest-coverage`.
2. Run the host test harness (`tests/`) and each on-target case via the
   PlatformIO unity-test framework.
3. `lcov --capture --output-file coverage.info`
4. `genhtml coverage.info --output-directory coverage_report`
5. Inspect every uncovered branch; either author an additional test or
   waive with documented justification.

Acceptance: 100% statement coverage, 100% decision coverage, 100% MC/DC
coverage, with every uncovered/waived condition cross-referenced against
this document.

## 5. Trace matrix (REQ ↔ Test ↔ Source)

| REQ | Test case | Source location |
|---|---|---|
| REQ-BOOT-001 | TC-BOOT-001 | `main.cpp::phaseBootConsole` |
| REQ-BOOT-002 | TC-BOOT-002 | `main.cpp::phaseBootDiagnostics` |
| REQ-BOOT-003 | TC-BOOT-003 | `main.cpp::phaseBootActuators` |
| REQ-BOOT-010 | TC-BOOT-010 | `main.cpp::phaseBootHotswapBuses` |
| REQ-BOOT-020 | TC-BOOT-020 | `main.cpp::phaseBootControlApi` |
| REQ-BOOT-030 | TC-BOOT-030 | `main.cpp::phaseBootRunPbit` |
| REQ-BOOT-040 | TC-BOOT-040 | `main.cpp::phaseBootApplyRailPolicy` |
| REQ-BOOT-050 | TC-BOOT-050 | `main.cpp::phaseBootSchedulerArm` |
| REQ-PWR-013 | TC-PWR-013 | `rail.cpp::tick` (HW breaker config) |
| REQ-PWR-200 | TC-PWR-200 | `main.cpp::disableAllRails` |
| REQ-LED-001 | TC-LED-001 | `leds.cpp::init`, `main.cpp::enterMode` |
| REQ-LOCK-001 | TC-LOCK-001 | `winch_lock.cpp::init` |
| REQ-LOOP-001 | TC-LOOP-001 | `main.cpp::loopTaskEStop` |
| REQ-LOOP-099 | TC-LOOP-099 | `main.cpp::serviceWatchdogIfHealthy` |
| REQ-DIAG-001 | TC-DIAG-001 | `main.cpp::phaseBootResetBreadcrumb` |
| REQ-DIAG-002 | TC-DIAG-002 | `main.cpp::stackCanaryArm/Intact` |
| REQ-DIAG-003 | TC-DIAG-003 | `main.cpp::computeImageTextCrc32` |

## 6. Out-of-scope

The following items are explicitly NOT covered by this V&V plan and must
be addressed separately before formal Level A:

- **Hardware-fault tolerance**: the STM32F103C8T6 does not have ECC-RAM,
  lockstep cores, or fault-tolerant flash.  No amount of software V&V
  qualifies the hardware.
- **Environmental qualification**: MIL-STD-810 / DO-160 vibration, shock,
  thermal, EMI testing.  These are part-procurement activities.
- **Toolchain qualification**: see `TQL-5-IDENT.md` (residual R4).
- **Static analysis**: see `tools/cppcheck.cfg` (residual R3).
