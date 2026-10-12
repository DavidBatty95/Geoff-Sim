# Wider transport aisles and people-clearance follow-up

Development checkpoint, 6 October 2026. The changes below improve layout space,
workspace prediction and interruption handling. **Full people-present delivery
has not passed in this follow-up; the avoidance changes remain experimental.**

## Layout

The `transport_dual` four-table scene now uses pickup/delivery table centres at
`(2.5, 0) / (-2.5, 0)` and `(2.5, 3) / (-2.5, 3)` metres. Table dimensions remain
1.1 × 1.0 m. The central edge-to-edge aisle grows from **2.9 to 3.9 m**, and the
gap between adjacent rows from **1.2 to 2.0 m**. The outer row retains 1.5 m to
the room boundary. Coordinates remain simulator/evaluator data; mission pair
files still contain only object/destination IDs and colours.

## Transport-specific avoidance

When `--people` is nonzero, the launcher derives and saves a transport profile
from the onboard profile. It adds **0.10 m** to both the planner's body prediction
margin and the supervisor's prediction collision radius; existing payload
expansion remains active. Comfort radius is at least 2.0 m, prediction activation
clearance at least 3.5 m, and ordinary route speed at most 0.45 m/s.

The shared maximum resultant planar speed is **0.45 m/s** in the planner,
supervisor and policy controller. This prevents independent reverse/lateral
components from producing a ~0.64 m/s diagonal command, observed before a loaded
fall. It does not disable sideways/reverse escape candidates. The lower cap did
not by itself eliminate falls in the retained trials.

`--people-clearance-extra 0.1` controls the added geometry margin (allowed range
0–0.5 m); setting it to zero does not disable the other transport-profile changes.
Zero-person runs retain the original navigation profile. Each people-present run
saves `transport_navigation.yaml`, its checksum, and resolved runtime parameters.
Existing scoring and grasp/placement tolerances are unchanged.

The arm-workspace guard now computes continuous closest approach over the next
two seconds, starting from the age-corrected measured person position. It catches
crossings between the former 0/1/2-second samples and rejects nonfinite tracks.

## Recovery and balance corrections

- Fixed joint carry/rest postures can resume after a person interruption despite
  base x/y/yaw drift. They restart from measured joints and retain workspace,
  tilt and final-pose checks. Cartesian reaches still require re-observation and
  re-docking after base movement.
- A Nav2 success with stale/out-of-tolerance sensed staging pose triggers the
  existing bounded localization retry, rather than accepting that arrival.
- Once policy control is established, the startup upright gate cannot replace
  it during a transient avoidance tilt. The separate tilt guard remains active;
  explicit reset/error/tilt-hold paths rearm startup as before.

## Retained native attempts

All trials used cycle-03 ONNX, `payload_v1`, the wider dual scene and a 420 s
deadline. All had valid evaluator data. Runs are under the benchmark package's
`results/transport_demo/` directory.

| Run | People / seed | Outcome |
|---|---|---|
| `20261006T163200Z` | 1 / 46 | Lift interruption followed by base-drift rejection; no fall/drop/contact samples; minimum clearance 0.613 m |
| `20261006T163646Z` | 1 / 46 | Lift resumed; later fell/dropped during loaded avoidance; no contact samples |
| `20261006T164142Z` | 1 / 46 | Nav2 staging success rejected; no fall/drop/contact samples |
| `20261006T164523Z` | 1 / 46 | Staging re-observation worked; later fell/dropped during delivery navigation |
| `20261006T165052Z` | 1 / 42 | Fell during survey; 55 contact samples |
| `20261006T165505Z` | 0 / 46 | Control run fell during survey; this showed the issue was not limited to people clearance |
| `20261006T174443Z` | 1 / 46 | With startup-gate fix: resumed preparation after base drift, resumed interrupted lift, reached delivery table; no fall/drop, but 24 contact samples and lower-arm tolerance failure |

These are evolving development attempts, not a paired causal comparison or an
acceptance cohort. No full delivery passed. Contacts here are evaluator contact
samples, not a count of distinct physical contact events. Remaining work includes
person-encounter safety and reprojecting world-relative placement targets when
avoidance moves the base during manipulation. Do not relax the current tolerance
to conceal the latest 0.0253 m hand-target error.

Verification: **445 bridge tests and 9 focused layout/profile tests passed**.
Regression cases cover between-sample crossings, observation age, malformed
tracks, shared diagonal limits, carry-posture resumption, stale arrival and the
startup gate. Native outcomes above qualify the limits of that software evidence.

Portable evidence: [all seven attempts](results/transport_people_clearance_20261006.json).

## Run the updated scene

```bash
python3 scripts/run_transport_demo.py --world transport_dual --people 1 --seed 46 \
  --duration 420 --people-clearance-extra 0.1 \
  --policy vendor/mjlab/logs/payload_training/native_delay_cycle_03_20261006/policy.onnx \
  --carry-posture payload_v1 --no-rviz --fast-start
```

The trained policy is a local artifact. The generator updates the scene even
with `--fast-start`; no layout coordinates are supplied to navigation/perception.
