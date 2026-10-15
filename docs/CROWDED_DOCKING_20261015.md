# Bounded inspection recovery and docking-workspace admission

## Status

Seven native attempts were retained: **none of the six one-person attempts
completed**, while the zero-person red → blue control completed in **149.10 s**.
Several development attempts recorded person-contact samples and two recorded
falls. The two corridor-build people trials ended without contact/fall but did
not complete their task. These evolving configurations are not a causal A/B or
an acceptance cohort. People-present delivery remains unqualified.

[Complete selected-window evidence](results/crowd_workspace_20261015.json) ·
[Receipt-aligned failure diagnosis](results/crowd_workspace_diagnostics_20261015.json)

## 1. Reobserve after a settled scan displacement

The baseline stopped during support inspection after 0.356 m of measured scan
drift. The 0.35 m guard completed braking, but its caller treated the stopped
viewpoint interruption as a terminal mission error.

`ViewpointDisplaced` now explicitly identifies this case. It is raised only after
checked braking succeeds; the motion owner revokes intent before the caller
handles it. Support inspection then:

- Invalidates prepared viewpoints and reads new images from the actual pose.
- Requires two acquisitions after settling and post-settling support evidence.
- Recomputes an observed-clear viewpoint if the edge is still unavailable.
- Keeps the existing three-attempt bound and original mission deadline.

Failed braking, body instability, uncertain action termination and generic errors
do not become permission to retry. The drift limit and settling checks are unchanged.
The `172233Z-dock-corridor` run exercised this recovery and replanned from its
displaced pose, then correctly stopped on a later braking timeout.

## 2. Withdraw docking intent when the approach is occupied

The first recovery follow-up reached docking, then recorded person overlap and a
fall. Its receipt-aligned diagnostic places the first geometric overlap about
5.18 seconds before the first fall threshold. Around that overlap the planner
reported emergency stop, zero selected command and all candidates rejected.
This is not exact actuator chronology and does not prove a single cause.

Docking now checks workspace admission before dispatch, during approach and after
settling. On intrusion or stale input it revokes nominal intent, leaves collision
supervision active and waits for the existing clear-workspace dwell. Waiting uses
the original docking deadline; avoidance movement does not become a learned
coasting correction.

Admission covers the **whole straight approach corridor**, including a
coast-compensated endpoint, rather than only the current robot position. It
computes the distance between that corridor and the age-corrected constant-velocity
person segment over the existing two-second horizon. The 1.6 m workspace threshold
is unchanged. This conservative spatial check can reject a corridor even if the
robot and person might traverse it at different times; it is not a probabilistic
collision guarantee or synchronized motion rollout.

Workspace clearance also requires a finite robot pose no more than 0.5 seconds
old, alongside the existing ≤0.8-second odom-frame person-packet check. Empty
person packets cannot hide a stale/missing robot pose. No sensed obstacles are
cleared, and docking/placement tolerances remain unchanged.

The final review bound admission to the actual compensated endpoint; that narrow
adjustment was regression-tested after the native trials below. It does not turn
their results into qualification of every recovery branch.

## Retained native outcomes

All used cycle-05, `transport_dual`, a 600-second budget and no numerical-thread
override. All evaluator streams were valid. Contact samples are not counts of
distinct encounters; negative geometric clearance indicates overlap.

| Run suffix | People / seed / task | Configuration | Outcome | Contacts | Min. clearance |
| --- | --- | --- | --- | ---: | ---: |
| `170111Z-crowd-baseline` | 1 / 46 / red → red | Earlier cadence baseline | Settled scan displacement terminated support inspection | 0 | 1.522 m |
| `170800Z-support-reobserve` | 1 / 46 / red → red | Support recovery | Person overlap and fall during docking | 57 | −0.523 m |
| `171433Z-docking-yield` | 1 / 46 / red → red | Current-body docking yield | Contact; stopped at manipulation tilt limit | 19 | −0.421 m |
| `171715Z-crowd-refuge` | 1 / 46 / red → red | Yield + opt-in predictive refuge | Yield/resume observed; contact and fall | 21 | −0.433 m |
| `172233Z-dock-corridor` | 1 / 46 / red → red | Corridor admission | Recovered displaced support view; later braking timeout | 0 | 1.054 m |
| `172421Z-workspace-control` | 0 / 48 / red → blue | Corridor admission | **Complete, 149.10 s** | 0 | N/A |
| `172733Z-crowd-seed48` | 1 / 48 / red → blue | Corridor admission | Docking deadline expired | 0 | 0.811 m |

The zero-person control also exercised a native placement correction: estimated
payload error went from **3.79 cm** on the first lower attempt to **2.73 cm** on
the second. Release followed the unchanged 3.5 cm alignment gate; post-release
verification and restoration passed. This demonstrates the correction path, not
general manipulation reliability.

Predictive refuge remains opt-in: its failed trial does not support promotion.
The caller-level changes do not solve the underlying encounter: a stationary or
yielding robot can still be approached by a pedestrian, and sensing/prediction/
execution errors still matter.

## Verification

Full `test_workspace_inside.sh ci`: **834 ROS results**, zero errors/failures/skips,
including **638 bridge tests**; **159 orchestration tests**, **12 native CTests**,
the deterministic mock benchmark and MJLab environment check passed.

New regressions exercise explicit interrupted-view recovery, fresh-image gating,
deadline preservation, nonretryable faults, docking intrusion during approach and
settling, compensated-endpoint admission, invalid robot poses, crossing paths and
parallel/degenerate prediction segments.

## Evidence selection and reproduction

The exporter can now select a mission/matrix run prefix within a declared date.
It records the selection, retains failures, and omits unrelated date-wide probe
and training records when a prefix is specified. This report selects all seven
`20261015T17...` attempts rather than re-publishing earlier zero-person cohorts.

```bash
python3 scripts/run_exploration_transport_demo.py --world transport_dual \
  --task red:red --people 1 --seed 46 --duration 600 \
  --policy vendor/mjlab/logs/payload_training/native_contact_cycle_05_20261012/policy.onnx \
  --fast-start --no-rviz

python3 scripts/publish_performance_evidence.py --date 20261015 \
  --run-prefix 20261015T17 --output docs/results/crowd_workspace_20261015.json
```

Next priorities are controlled pedestrian-encounter replay, measured escape and
stopping envelopes, and clearance-preserving deadlock recovery. Wider crowd
acceptance requires repeatable complete tasks with the existing strict scoring.
