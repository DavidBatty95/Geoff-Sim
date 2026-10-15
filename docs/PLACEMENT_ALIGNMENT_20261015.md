# World-relative placement alignment and route diagnostics

## Status

The implementation and regression checks pass, but **native qualification remains
open**. All five new attempts fell before placement; none exercised the new
pre-release correction loop. The single-worker numerical-library experiment also
fell, so its setting remains opt-in. Earlier successful deliveries belong to the
[preceding navigation-integrity checkpoint](NAVIGATION_INTEGRITY_20261015.md).

The [supplemental evidence export](results/placement_alignment_20261015.json)
contains all 13 retained attempts from 15 October: the earlier eight plus these
five. Its three successful deliveries are the earlier results, not new passes.
Do not treat the cumulative export as a frozen cohort.

## Placement failure diagnosed

Run `20261015T072614Z-13bd091e9c7a400fbbd1861e33e131c2` placed the red cube near
the edge of the blue pad. Both onboard verification and the independent evaluator
rejected the delivery. The final camera snapshot detected the cube about 17.6 cm
from the pad centre, beyond the unchanged 13 cm visual verification tolerance.

The lower action reported only **1.52 cm pelvis-frame hand error**, but body yaw
changed from approximately **3.109 to 2.554 rad** during lowering—about 32°. An
accurate arm target relative to a moving pelvis therefore did not establish
alignment with the world-fixed destination.

`TransportMission.lower_to_pad` now:

1. Requires measured base pose age ≤0.3 s and a still-confirmed destination estimate
   within the existing 45-second pad occlusion-memory limit.
2. Reprojects the observed destination into the current pelvis frame.
3. Executes the normal measured arm reach with its existing workspace, tilt,
   cancellation and action-termination checks.
4. Re-estimates the payload centre from measured hand position/orientation and
   the retained hand-to-object offset; compares it with the freshly reprojected
   destination, rather than accepting the old pelvis-frame command alone.
5. Retries at most three reaches and admits success only within **3.5 cm**.
   A 30-second acceptance budget is checked around actions; existing action and
   global mission deadlines still govern in-flight termination.

Failures retain attachment/authority and cannot advance to release. A revoked or
expired destination does not become permission to use a fabricated estimate.
Post-release camera verification remains mandatory and unchanged. The new guard
is based on onboard geometry, not evaluator truth; its own pose/offset uncertainty
and occlusion limits still require native qualification.

Nine fault-injection regressions cover base translation/rotation, current payload
orientation/offset, repeated misalignment, stale pose, revoked pad, late completion
and uncertain action termination.

## Explain blocked loaded routes before changing clearance

The prior blue → red failure reported `start_clear=false`, `goal_clear=true` with
a 0.65 m loaded search reserve, but did not retain the occupancy grid. There was
insufficient evidence to determine whether the blocker was occupied space,
unknown space, map boundary or conservative reserve inflation.

Blocked-route events now include footprint cell counts and separate search/physical
reserve checks. The last blocked query is saved as:

- `route_blockage.json`: start/goal, grid origin/resolution/yaw/dimensions, map
  acquisition stamp, reserve radii, rejection categories and map-file SHA-256.
- `route_blockage_map.npz`: the observed occupancy array used for that decision.

These are diagnostics, not obstacle clearing or a new escape policy. The existing
conservative square footprint, unknown-space rejection and recovery constraints
remain active. Three regressions check category separation, reserve-only blockage
and exact snapshot retention. No new native blockage snapshot was produced by the
five attempts because they failed earlier.

## Optional numerical-library worker experiment

A direct query of NumPy's loaded OpenBLAS library in the tested container reported
**20 workers** by default; an explicit limit of one reported **one worker**. This
does not demonstrate that worker count caused a fall or that one is faster.

Transport launchers and the performance matrix now accept `--numeric-threads N`
for 1–32 workers. The option forwards `OPENBLAS_NUM_THREADS`, `OMP_NUM_THREADS`,
`MKL_NUM_THREADS` and `NUMEXPR_NUM_THREADS` to both simulation and mission
containers. Manifests record requested settings; runtime provenance records the
four effective environment values, without dumping unrelated environment data.
ONNX inference already has separate one-thread settings.

```bash
python3 scripts/run_exploration_transport_demo.py \
  --world transport_dual --task red:blue --people 0 --seed 48 --duration 600 \
  --policy vendor/mjlab/logs/payload_training/native_contact_cycle_05_20261012/policy.onnx \
  --numeric-threads 1 --fast-start --no-rviz
```

Omit the option to retain existing image defaults. The one native limited-worker
attempt fell during search, before pickup. No default promotion or speedup claim
is supported by this experiment.

## All new native outcomes

| Run suffix | Executive/task | Numerical threads | Outcome |
| --- | --- | --- | --- |
| `080000Z-world-placement` | Exploration, red → blue | Existing defaults | Fell during search navigation |
| `080326Z-0f787149a3b6410bb63df82503f45313` | Exploration, red → blue | Existing defaults | Fell during search |
| `080326Z-60e1e3a9cb8648b7810ea7be068bf327` | Exploration, blue → red | Existing defaults | Fell during search |
| `080800Z-placement-scripted` | Scripted survey, red → blue | Existing defaults | Fell during initial destination discovery |
| `081100Z-placement-single-worker` | Exploration, red → blue | Explicit 1 | Fell during search navigation |

All retained evaluator streams were valid; none reached pickup or placement.
These attempts changed executive/thread configuration and are not a causal A/B
study. Existing host workloads were left running; no inference about their causal
effect is claimed.

## Verification

Full `test_workspace_inside.sh ci` passed: **803 ROS results**, including **607
bridge tests**; **152 orchestration tests**, **12 native CTests**, deterministic
mock benchmark and MJLab environment listing. Shell syntax checks passed.

Next qualification should first stabilize/reproduce the pre-pickup failures under
a declared load/timing envelope, then exercise world-relative lowering natively.
Route-policy changes should use captured blockage maps rather than reducing
clearance based on a boolean failure alone.
