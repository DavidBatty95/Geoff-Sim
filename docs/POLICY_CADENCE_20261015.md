# Minimum-spaced policy observations and native delivery follow-up

## Result and scope

The unchanged compact-world exploration cohort completed **4/4 colour tasks**.
A separate red → blue pilot and a medium-building red → blue task also completed.
All six non-recording attempts had valid evaluator data, no falls and no dropped
objects. Four subsequent screen-capture attempts produced three completed
missions and one fall; two completed missions had unusable footage.

The [cumulative evidence export](results/policy_cadence_20261015.json) retains all
23 attempts from 15 October, including 13 earlier attempts. The ten attempts on
this cadence candidate are the IDs beginning `20261015T15`; nine completed. Do not
pool different worlds, executives and recording workloads into a reliability claim.
Earlier failures remain published in their original reports.

## Timing defect

The device-clock observation scheduler previously preserved nominal phase after
a late sample. For a 20 ms policy period, accepting a delayed observation at
38 ms left the next nominal slot at 40 ms. The controller could therefore add
history observations only **2 ms apart**. Retained pre-fall windows include
2–6 ms observation intervals, despite a configured 20 ms period.

This is a directly reproducible cadence-contract defect, not proof that it alone
caused the recorded falls. Existing policies also have delay, transition and
loaded-motion limitations, and host scheduling varied between runs.

Reanalysis of the complete retained traces also finds a **2 ms minimum in both
a failed and a successful earlier run**: [failure, 1,055 matched publications](results/cadence_baseline_failure_20261015.json)
and [success, 5,856 matched publications](results/cadence_baseline_success_20261015.json).
The reports retain raw-trace SHA-256 identities. Catch-up timing is therefore not
an outcome classifier, and the native passes below do not establish causality.

## Implementation

`DeviceTickCadence` now schedules the next observation at:

```text
next device deadline = last accepted device sample + policy period
```

Late samples skip elapsed slots rather than compressing the next interval. No
interpolated or synthetic history frames are inserted. Duplicate/reordered tick
rejection, uint32 wrap, near-zero restart fencing, wall-clock freshness watchdogs,
inference deadlines and stop invalidation remain active.

This applies when `use_device_tick=true`, as in transport. Legacy ROS-timer mode
is unchanged. Under sparse delivery this intentionally reduces sampling rate
rather than performing a catch-up burst; a guaranteed 50 Hz rate is not claimed.

Matched inference telemetry now distinguishes:

- `observation_dt_s`: actual accepted device interval, including long gaps.
- `observation_filter_dt_s`: bounded interval used by command/filter integration.
- `observation_period_s` and `observation_cadence`: declared contract.

The response analyzer reports interval distributions separately from inference
latency. In pilot `20261015T150700Z-minimum-cadence`, **6,473** matched published
observations had minimum/median **20 ms**, p95 **26 ms**, p99 **30 ms**, maximum
**44 ms**. These are observation intervals for matched publications, not complete
sensor-to-actuator latency or every received packet.

Regression tests cover the 38/40 ms catch-up case, irregular delivery, long gaps,
real adapter history, wrap/restart and context-matched timing. Command filtering
still bounds its integration step; telemetry no longer hides long device gaps
behind that cap.

## Native outcomes

All runs used zero people and the unchanged cycle-05 payload policy. No numerical
worker-limit override was used. The four-task matrix used `transport_dual`, layout
0, seed 48, the exploration executive and 600-second mission budgets:

| Task | Completion time | Outcome |
| --- | ---: | --- |
| Red → red | 123.24 s | Verified delivery, restoration complete |
| Red → blue | 159.85 s | Verified delivery, restoration complete |
| Blue → red | 162.62 s | Verified delivery, restoration complete |
| Blue → blue | 233.51 s | Verified delivery, restoration complete |

Matrix artifact: `performance_matrix/20261015T150925Z/summary.json` under the
ignored benchmark results tree. Separate checks:

- `150700Z-minimum-cadence`: compact red → blue, **142.67 s**, complete.
- `152255Z-cadence-medium`: medium-building layout 0, red → blue, **288.46 s**,
  complete after 13 box inspections and one destination inspection.

These runs exercised the world-relative pre-release alignment **acceptance path**;
the four matrix runs passed it on their first lowering attempt. Repeated corrective
reaches after significant base drift remain covered by regression tests rather
than a demonstrated native recovery. Zero person-contact counts in empty worlds
do not qualify crowd safety. The improved cohort is not a controlled causal A/B
or a demonstrated speedup over runs with different search trajectories/load.

## Recording attempts and complete-workflow GIFs

| Run suffix | Native outcome | Media outcome |
| --- | --- | --- |
| `152852Z-readme-cadence` | Complete, 145.21 s | Wrong pre-existing Dougie window; rejected |
| `153405Z-readme-owned-window` | Complete, 200.08 s | Correct viewer obscured by update notice; rejected |
| `154025Z-readme-complete` | Fell during mission | Failed attempt retained |
| `154429Z-readme-red-delivery` | Scripted red → red complete, 122.14 s | Visually inspected, used for the gallery |

The recorder now excludes windows that existed before launch and never silently
switches to another viewer. Exports reject captures explicitly marked unusable,
use the capture date/person count, and label success using the shared strict
predicate. A new placement/verification/restoration clip completes the README
workflow. [All media provenance](media/locomanipulation/recordings.json).

Screen recording changes workload; those attempts are not part of the four-task
cohort. The one later fall remains evidence that general stability is unresolved.

## Verification and next work

Full `test_workspace_inside.sh ci` passed: **807 ROS results**, zero failures,
errors or skips, including **611 bridge tests**; **155 orchestration tests**,
**12 native CTests**, deterministic mock benchmark and MJLab environment listing.

Next: larger held-out geometry/layout cohorts, controlled host-load/delay tests,
people-present delivery, and native correction/recovery qualification. Do not
promote the separate cycle-06 candidate or relax existing acceptance thresholds
based on this zero-person development cohort.
