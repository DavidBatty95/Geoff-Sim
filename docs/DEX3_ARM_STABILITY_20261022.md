# Dex3 arm-extension stability — 22 October 2026

Follow-up: [23 October control capture and tabletop-clearance findings](DEX3_TABLE_CLEARANCE_20261023.md).

**Heading feedback improves the isolated arm-extension diagnostic, but mobile
delivery remains unqualified.** Eleven native diagnostic runs and three new
mobile attempts are retained. All diagnostics completed without a fall or
non-floor robot contact. All three mobile attempts stopped before grasping;
none fell or reported a dropped object.

## Repeatable diagnostic

`scripts/probe_dex3_arm_stability.py` runs the full free-base Dex3 robot with the
native cycle05 ONNX actor, 2 ms physics and 50 Hz inference. Joint observations
and motor PD use native joint sensors; previous-action feedback is the published
actor action. Both hands are driven open with bounded finger torques.

Modes cover actor-only standing, prescribed carry posture, and arm extension.
Optional conditions add a turn/walk preamble, a retracted staging posture, a
rotated starting heading or a recorded right-arm endpoint. There are no object
welds, base pins, root forces or object forces. Root position is used only for
offline scoring; heading feedback uses the IMU. This synchronous diagnostic
does not reproduce ROS scheduling or the entire deployment command pipeline.

The principal comparison uses a prescribed turn/walk, five seconds stopped,
carry posture, extension, and a ten-second extended hold:

| Condition | Maximum hand drift during extended hold | Yaw excursion |
| --- | ---: | ---: |
| No heading correction | 48.6 mm | 0.1002 rad |
| Heading PD, gain 2, no command filtering | 11.7 mm | 0.0100 rad |
| Gain 2 with deadband/slew approximation | 27.6 mm | 0.0472 rad |
| Gain 4 with deadband/slew approximation | 13.9 mm | 0.0154 rad |
| Gain 4, rotated initial heading | 4.2 mm | 0.0004 rad |

The filtered runs retain 0.05 command deadband, 0.01 hysteresis and 0.7 rad/s²
yaw slew. The chosen feedback is `4 * wrapped_yaw_error - 0.35 * yaw_rate`,
capped at 0.6 rad/s and the configured planner limit. Additional staged and
recorded-joint runs completed, but still did not reproduce the large backward
translation seen during ROS arm motion. Actor-only and carry-only controls are
also included; completion is not a stability qualification threshold.

## Experimental integration

`--manipulation-heading-hold` enables a new `manipulate` planner mode during arm
operations. It requests angular correction only, through the existing candidate
rollout, static/person clearance, command envelope and slew checks. The intent
is revoked on arm completion, failure or workspace interruption. It is **off by
default**, and does not change waist motor ownership.

The reach loop also stops counting base retreat twice: it reprojects the observed
object after each trajectory and integrates only the arm's tracking residual.
The previous world-residual integrator could add the base displacement again,
pushing an already reprojected target beyond reachable space. A regression
models a 70 mm backward balance step and checks the second target.

An explicit `--pickup-standoff` parameter permits bounded goal-distance studies
between 0.35 and 0.60 m, retaining the original **0.43 m default**. This changes
the requested docking pose, not the clearance or arrival acceptance gates.

## Mobile outcomes and unresolved mismatch

All three trials used native cycle05, `payload_v1` carry posture and experimental
heading hold. IDs have prefix `dex3_contact_delivery_` and suffix `_20261022`.

| Run middle | Outcome |
| --- | --- |
| `headinghold` | Heading controlled during reaches; repeated backward translation made subsequent Cartesian targets unreachable |
| `heading_reproject` | Tracking/reprojection correction did not eliminate backward translation; reach remained infeasible |
| `heading_standoff37` | Requested 0.37 m standoff; docking settling and subsequent retreat still left the reach infeasible |

All three reported valid evaluator data. In the first run, an extension changed
onboard odometry x from about **1.614 to 1.546 m**; independent evaluator x changed
from about **1.613 to 1.546 m** over the same interval. Thus the backward shift is
real, rather than merely an odometry jump. Nearby logged policy commands had
zero translational components. These evaluator values were inspected offline,
never used for control.

Replaying a recorded arm endpoint in the native diagnostic still produced only
about **5 mm translation during extension**, so endpoint choice alone does not
explain the ROS/native mismatch. Timing, full initial state and command
application need matched capture/replay before attributing the remaining drift
to one cause or promoting a stabilization controller.

## Evidence, checks and reproduction

[All diagnostic and mobile results](results/dex3_arm_stability_20261022.json)
include hashes and summaries, with bulk observations retained locally. The two
initial probes retain their original `dex3_arm_stability_20261021` directory;
the publisher includes them explicitly. Later probe records also include helper
source hashes and a compiled-model dynamics fingerprint. Source snapshots from
different stages must not be treated as identical final-source runs.

```bash
docker compose run --rm --no-deps -T unitree-g1-sim \
  /workspace/scripts/run_mjlab.sh --with mujoco==3.3.6 --with onnxruntime==1.20.1 python \
  /workspace/scripts/probe_dex3_arm_stability.py \
  --policy /workspace/vendor/mjlab/logs/payload_training/native_contact_cycle_05_20261012/policy.onnx \
  --preamble --stabilization heading --filtered-heading \
  --output /workspace/ros2_ws/src/dynamic_obstacle_benchmarks/results/NEW_arm_probe.json
```

Final workspace CI passed **896 ROS results** (700 bridge tests) and **219
orchestration tests**. New tests cover wrapped/damped/bounded heading commands,
zero nominal translation, intent cleanup, preserved hazard checks and the
base-retreat double-counting regression. No locomotion checkpoint was promoted.
