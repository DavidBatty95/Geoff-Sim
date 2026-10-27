# Escape continuation and survey resumption — 27 October 2026

**Timed escape continuation and bounded survey handoff are implemented and
tested. Reliable crowd avoidance remains unqualified.** Seed 42 still failed
with overlap and a fall; seed 43 completed its survey without overlap or a fall.
The trace audit also corrects the interpretation of earlier publication counts.

## What the trace exposed

In `early_admission_batched42_20261026`, a valid primitive was selected, then a
single revalidation deadline caused the planner to discard its program. A
fallback `previous_command` could retain the first velocity without retaining
the planned phase switches and braking interval. In addition, the nominal
precision shortcut could cancel an admitted program when the immediate risk
trigger cleared, before the timed program completed.

These are distinct from a fresh geometric rejection: an unsafe continuation
must still be discarded, while a missed computation must not silently erase
the program's timing and turn it into an untimed velocity.

## Implemented behavior

### Checked continuation

- Keep the admitted primitive's original start time and end time while checking
  its remaining phases on each tick. Successful checks do not restart the clock.
- Retain the program across a computation deadline so a later tick can revalidate
  it. **Retention itself returns no command.** Existing checked fallback behavior
  remains responsible for that tick.
- A fresh geometric or terminal-state rejection invalidates the commitment.
  Changed goals, revoked mission intent and expiry still cancel it.
- While a valid commitment remains, do not let a clear short-horizon trigger
  automatically replace it with a survey turn. Fresh full-path checks remain
  mandatory, including when a fresh track message contains no people.
- Diagnostics record original-clock revalidation, retention pending a fresh
  check, invalidation, and the commitment's original end timestamp.

### Bounded survey handoff and fresh resumption

A nominal braking timeout may transfer to survey yielding only when there is
fresh evidence of checked avoidance translation: an eligible planner state,
nonzero fresh policy translation, fresh pose and stable fresh body orientation.
Stale-state failures, zero-command physical motion, instability and unrelated
errors do not qualify for this handoff.

The survey owner retains the existing goal during that handoff rather than
revoking it and immediately submitting a new one. Yield completion still requires
the existing low-velocity, near-zero-command, clear-workspace and body-stability
dwell. The original turn deadline and two-recovery bound remain in force, and
the owner revokes intent on every final exit. A timeout is never relabelled as
successful braking.

After yielding, the target must be confirmed by an observation acquired after
the yield-completion timestamp. Old sensed hints can direct viewing but cannot
satisfy discovery. Survey turns may stop on a fresh confirmed detection, while
post-braking displacement checks remain active. No simulator truth is used.

## Important correction to earlier evidence

The previous report counted **eleven** primitive publications in the successful
seed-43 run. The [verified scope audit](results/continuation_scope_verified_20261027.json)
shows that all eleven followed the `survey_complete` milestone, during the
post-completion observation/shutdown interval. They were real admitted commands,
but **not evidence that those commands helped complete the survey**.

The prior seed-42 run's three publications occurred before its final failure.
The original JSON reports remain unchanged. New reports retain whole-trace
counts and add phase/period splits. New runs also record `task_end_monotonic_s`
at the task method's return or exception, separately from the subsequent
observation and teardown interval. Classification uses logger receipt times;
these are not actuator-application timestamps.

The [first scope-audit output](results/continuation_scope_audit_20261027.json) is
retained as an instrumentation failure: a local variable shadowed some phase
labels with `beam`. The verified report fixes that issue, and regression tests
cover both explicit task-end timestamps and legacy completion-milestone fallback.

## Both live attempts

Runs use three people, native cycle05, `transport_dual`, red-to-red survey,
`payload_v1`, heading hold, 15 mm grasp offset, no RViz and a 120-second limit.
[Complete retained evidence](results/sustained_survey_validation_20261027.json)
includes runtime/source hashes and full logger-tail statistics.

| Run | Outcome | Primitive publications and scope |
| --- | --- | --- |
| `sustained_survey42_20261027` | 42 overlap samples; fall; valid evaluator data | Two before task end; one original-clock continuation, then fresh-check invalidation |
| `sustained_survey43_20261027` | Survey complete; zero overlap; no fall; reported minimum clearance 1.120 m | 24 after task end; 23 original-clock revalidations and two retained-check events |

The seed-43 continuation records exercise the mechanism after task completion;
they must not be credited as causing the survey pass. The checked-braking-timeout
handoff is covered by regression tests but was not exercised by these two live
runs. No delivery was attempted or qualified in this session.

## Verification and remaining limit

Tests cover continuation after a missed deadline, fresh-obstacle invalidation,
trigger clearing, cancellation, original end-time preservation, checked-avoidance
handoff eligibility, cleanup, stale-confirmation rejection and reporting scope.

Complete workspace CI passes **1,003 ROS results** (807 bridge tests), **240
orchestration tests** and **13 native CTests**. Documentation, repository and
source-patch checks pass.

The remaining live blocker is a continuation becoming invalid under fresh
forecasts, followed by inadequate physical avoidance. It is no longer appropriate
to attribute every interruption to lost schedule state. Forecast/response mismatch
at the first invalidating tick needs to be isolated before changing prediction
or control behavior. The strategic option remains experimental and off by default.
