# Performance implementation: evaluation foundation

**Workspace follow-up:** [inspection recovery and docking admission](CROWDED_DOCKING_20261015.md)
preserve deadlines and strict outcomes. The zero-person control completed after
a native placement correction; all six one-person development attempts failed.

**Earlier cadence checkpoint:** [minimum-spaced policy observations](POLICY_CADENCE_20261015.md) remove
catch-up samples, with a 4/4 compact task cohort and complete medium-building
delivery. One subsequent recording-run fall remains in the evidence.

**Placement follow-up:** [world-relative alignment and route diagnostics](PLACEMENT_ALIGNMENT_20261015.md)
are regression-tested; all five native follow-ups fell before placement. Optional
numerical worker limits remain experimental with no default change.

**Earlier checkpoint:** [15 October navigation integrity](NAVIGATION_INTEGRITY_20261015.md)
adds checked path-goal transforms and time-bounded scan registration, with three
complete deliveries among eight retained development attempts. The unchanged-code
four-task exploration cohort is 2/4, not an all-task qualification.

The 14 October implementation increments strengthen reproducibility, experiment
accounting and control transitions. They do not establish improved native mission
performance. This checkpoint includes the preceding October 12–14 performance
work and its [retained successes and failures](PERFORMANCE_IMPLEMENTATION_20261012.md).

## Changes

- The simulator fallback import matches the submodule and reviewed patch base.
  MJLab fallback checkout reads its revision from the patch manifest.
- A regression check compares patch bases with Git submodule revisions and the
  Unitree import manifest.
- The performance matrix and portable publisher share a fail-closed success
  predicate: explicit mission success/completion, valid data, no fall, and an
  integer zero contact count. The matrix also requires a zero process exit code.
- Every matrix attempt has a unique explicit transport run ID. Directory-set
  differences are no longer used to infer ownership.
- All planned attempts are checkpointed before execution. Summary replacement
  is atomic, and an existing summary cannot be overwritten.
- Missing or malformed results remain failed attempts without aborting the
  remainder of an otherwise normally terminating cohort.
- A host lock excludes concurrent performance matrices. Standalone launchers
  do not yet participate in this lock; continue running only one simulator.
- Each attempt has an outer wall-clock timeout of mission duration plus
  `--startup-cleanup-budget` (default 900 seconds). This includes build time.
  Timeout/SIGTERM/Ctrl-C requests wrapper cleanup, waits up to 45 seconds, then
  kills the wrapper process group if necessary and stops the cohort. Detached
  simulator processes/containers still require inspection after forced cleanup.
- `--fast-start` is now explicit for the whole cohort; a failed initial build
  cannot implicitly make subsequent attempts reuse incomplete artifacts.

## Usage

After a successful build, run a frozen policy cohort:

```bash
python3 scripts/run_performance_matrix.py \
  --policy vendor/mjlab/logs/payload_training/<candidate>/policy.onnx \
  --tasks red:red red:blue blue:red blue:blue \
  --layouts 0 1 --seeds 48 49 50 51 52 --fast-start
```

Each attempt has a wrapper log in the matrix output directory and a named run
under `ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/`.
`complete` means the cohort finished execution, not that its missions passed.
Planned but unexecuted attempts remain visible after interruption. Automatic
resume is not implemented; use a new output directory for a new cohort.

## Ordered output and explicit motion settling

Compositor output selection, duplicate gating and publication now share the same
lock as control-state changes. A barrier-controlled concurrent regression checks
that a newer output cannot overtake an already selected publication. The timer
and locomotion subscription already share ROS's mutually exclusive default
callback group; this also closes the selection/publication gap against reentrant
action state changes. Native timing under this lock still needs measurement.

Normal navigation and inspection braking now returns `settled` only after fresh,
finite measured velocities remain below the existing thresholds across 0.2 seconds
of advancing measurement timestamps. A clock regression resets that evidence.
Deadline exhaustion raises `MotionSettlingError` with `timeout` or `stale_state`,
and records the outcome and elapsed braking time. Existing action cleanup and
intent revocation run before the failure propagates. The original operation
deadline is retained. Emergency cleanup still revokes intent immediately.

This contract covers `navigate` and inspection `rotate`; docking and retreat
retain their existing geometric settling checks. There is no new automatic retry
for failed braking, nor a claim that these thresholds guarantee physical balance.

## Device-observation progress watchdog

The subsequent 14 October follow-up separates valid packet receipt from device
measurement progression in policy execution. With `use_device_tick=true`, the
controller rejects duplicate and reordered ticks before replacing accepted
LowState or updating upright/reset evidence. Packet receipt is still counted;
it cannot renew the independent progress lease.

Both missing traffic and frozen progress use the existing `lowstate_timeout`
(default 0.1 s). Timeout invalidates pending inference, clears its lineage context,
publishes zero policy motion feedback, and invokes the existing sensor-fault
state hold. A frozen-but-live stream is labelled `frozen_lowstate_hold`; missing
traffic retains `stale_lowstate_hold`. The result-consumption callback performs
the same checks, so completed work cannot bypass the watchdog. Only a newly
accepted tick clears the stale latch. Device wrap and the existing near-zero
restart convention match the policy-cadence contract.

Diagnostics expose receipt age, progress age, duplicate count and reordered
count. Legacy wall-clock profiles do not require advancing ticks. No synchronized
source-age or physical actuator guarantee is implied: a consistently delayed
but advancing stream can still pass this check, and the restart heuristic is
not a unique device-session identifier.

Verification: the complete bridge suite passed **567 tests**, including eight
new tests for duplicates/reordering, wrap/restart, callback rejection, inference
invalidation on both control/result paths, recovery and legacy compatibility.
Native frozen-stream injection and physical hold/recovery behavior remain to be
qualified; the existing sensor-fault hold is not a guarantee of standing balance.

## Fixed-arm posture completion and native capture follow-up

Carry and final restoration now share `move_arm_posture`. A fixed arm target
contains only the commanded arm joints; completion compares measured and requested
arm geometry at the same current measured waist configuration. The 3.5 cm hand
error limit is unchanged. Missing/nonfinite arm or waist joints fail explicitly
instead of entering FK as zero defaults. Restoration releases arm authority only
after this validation. Cartesian reach/IK targets retain their separate contract.

Sixteen real-URDF regressions cover yaw/roll/pitch waist movement, true arm error,
incomplete/nonfinite measurements, interrupted restoration, invalid arm-only
targets, retained authority on failure and Cartesian carry behavior.

The README capture session also exercised the new braking contract natively.
Initial 1.5-second limits rejected two attempts; a retained trace showed measured
settling shortly after that limit. Normal navigation/inspection braking now has
up to five seconds **inside the original operation deadline**, with identical
freshness, velocity and 0.2-second measurement dwell thresholds. It returns as
soon as measured settling succeeds; five regressions cover slower settling and
deadline caps. Subsequent native stops included 2.83-second settling, but no
capture attempt completed a mission. No failed outcome was upgraded.

The [media manifest](media/locomanipulation/recordings.json) retains all seven
attempts, commands, source/policy identities, errors and events. Captures include
successful pickup and loaded motion, alongside falls and a navigation/localization
failure. They are development evidence under screen-capture workload, not a
controlled reliability cohort or proof of the final restoration path.

The complete bridge suite now passes **588 tests**. Documentation links, generated
graphics/navigation checks, repository metadata checks and nine README-navigation
tests also pass. Native repeated delivery, contact-only grasping, swept-path arm
collision checking and physical fault recovery remain open.

## Verification and next steps

Focused tests cover malformed/missing results, nonzero exits, strict outcomes,
timeout accounting, process timeout cleanup, competing matrices, output reuse,
and dependency revision consistency. They run in the workspace orchestration
test tier.

Verification on 14 October:

- Full `test_workspace_inside.sh ci`: 744 ROS results, zero failures/errors/skips;
  includes 559 bridge tests. 142 orchestration tests and 12 native CTests passed.
- Deterministic mock benchmark and MJLab environment listing passed. The shell
  tool's initial 120-second wait expired during the final environment check;
  waiting for that same container confirmed exit code zero.
- Focused control/lifecycle tests: 59 passed before the full run.
- Command-application/repository-quality tests: 17 passed.
- MJLab training-contract and native arm-geometry checks: 12 passed. These CPU
  checks use `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` in the MJLab environment to avoid
  loading unrelated ROS pytest plugins into its isolated dependency set.
- Reviewed-source checks and pristine patch reconstruction passed.

No new native delivery cohort was run for this control increment. Earlier
successful deliveries and failed missions remain dated evidence for their own
configurations; cycle-06 remains unpromoted.

Next foundation work: shared launcher-wide simulator ownership, verified native
build provenance, readiness-driven startup, interruption-safe resume, and complete
artifact inventories. Next control work is native settling/latency qualification,
acquisition-to-application freshness contracts, measured loaded response and
delay-robust locomotion without turn-to-rest regressions.
