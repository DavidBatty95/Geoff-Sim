# Transport survey failure assessment — 24 October 2026

## Conclusion

The two latest demos fail **before pickup**, during destination discovery. Their
large displacement is not primarily an early-arm-reaching problem or an
uncommanded pure-turn drift. The planner deliberately replaces the survey turn
with translational avoidance as pedestrians approach. The system then lacks a
reliable combination of sustained crowd avoidance, settled-viewpoint recovery
and post-braking survey validation.

The 0.35 m viewpoint check is the final abort condition, not the root cause.
Increasing that limit would mask a real failure: the three-person run overlaps
pedestrian clearance envelopes, and the four-person logger tail does too.

This assessment used retained traces, source inspection and timestamp-matched
odometry comparisons. It did not change controller settings or run an acceptance
matrix. [Machine-readable assessment](results/transport_scan_assessment_20261024.json)
includes input hashes, mode windows, candidate decisions and timing statistics.

## Runs and evidence scope

Both runs used seed 42, native cycle05, `transport_dual`, red-to-red transport,
`payload_v1`, 15 mm grasp offset, heading hold, and visible MuJoCo/RViz. Pedestrians
start after the configured 10-second simulator delay. Neither reached arm
preparation, grasping or loaded transport.

| Measurement | Four people: `20261024T185744Z` | Three people: `20261024T185845Z` |
| --- | ---: | ---: |
| Completed delivery | No | No |
| Fallen / evaluator data valid | No / yes | No / yes |
| Displacement at abort detection | 0.377 m | 0.357 m |
| First scan endpoint displacement, evaluator truth | 0.080 m | 0.075 m |
| Maximum translational policy command during avoidance/braking | 0.450 m/s | 0.450 m/s |
| Reported overlap samples | 0 | 44 |
| Minimum reported person clearance | +0.269 m | −0.424 m |
| Full logger trace overlap samples, including shutdown tail | 5 | 55 |
| Minimum full logger trace clearance | −0.339 m | −0.424 m |
| Mean timestamp-matched odometry/world XY difference | 7.2 mm | 8.9 mm |
| Maximum matched XY difference | 46.2 mm | 70.7 mm |
| Active planner ticks exceeding the 50 ms period | 25.0% | 44.3% |

**Contact terminology matters:** `TransportEvaluator._people` increments
`contact_samples` when center distance minus robot radius 0.35 m minus the
person radius is negative. These are geometric envelope-overlap samples, not
distinct collision events, tactile detections or contact-force measurements.
The logger continues briefly after mission evaluation ends. Thus the four-person
result's zero count does not mean its entire simulator shutdown trace was clear.
The reports preserve both scopes rather than rewriting the original result.

## What happened

### 1. Initial turning is reasonably localized

The first scan in each run uses zero translational policy command. Endpoint
movement is about 7.5–8 cm, below the 35 cm reserve. Sensor odometry and native
evaluation positions broadly agree at identical acquisition timestamps.
Localization error alone cannot explain the later half-meter-scale movement.

### 2. Approaching pedestrians change the requested motion

During the next scan, the nominal reference remains a turn, but the planner
predicts a collision for standing/turning and selects translation. The trace
contains `local_escape`, `human_margin`, and ordinary continuation decisions;
the filtered policy input reaches approximately 0.45 m/s.

For example, around ROS time 1792868351.06 in the three-person run:

- Nominal reference: `(0, 0, 0.491)` — a yaw turn.
- `yield_stop` and `raw_reference` are rejected as `hard_human_collision`.
- The selected short-horizon escape is approximately `(0.425, 0.147, 0)`.
- Its full-horizon candidate is marked `terminal_conflict`; the short-horizon
  recovery is explicitly not a complete-horizon safe plan.

This is intended collision-avoidance intervention, but the resulting route fails
to establish a safe stopping location before the independent walkers arrive.
Reducing the crowd count does not remove the incoming walker: both runs retain
the same seed and initial first-three pedestrian identities. The first walker's
recorded position at simulation time 17.536 s is the same in both traces.

### 3. A braking request can still produce avoidance translation

`brake_motion()` requests the planner's `brake` mode; it does not bypass people
avoidance. The planner first checks nominal braking and otherwise searches for
escape commands. In the four-person active braking window, evaluator displacement
is about **0.508 m**. In the three-person second-turn braking window it is about
**0.582 m**. These windows include translation commands, so describing all that
motion as passive braking overshoot would be incorrect.

The supervisor checks response/braking risk and can request a short escape, then
falls back to zero when no acceptable escape is available. In the three-person
trace, emergency/temporal stopping does not prevent the approaching person from
entering the robot's envelope.

### 4. The three-person run then moves under zero policy command

After the second turn's braking reports settled, the mission spends its normal
observation interval with inactive intent. Over that approximately 1.61-second
planner window the robot moves **0.522 m**, while every logged translational
policy command in that window is zero. The pedestrian envelope overlap begins
around ROS time 1792868355.62; substantial measured robot motion follows.

This is consistent with interaction with a moving pedestrian, not just a late
walking command. A physical push is **not proven** by these traces: they lack
contact-force and full motor-application capture, so residual policy balance
response cannot be completely separated from contact dynamics.

The native natural-pedestrian model explicitly avoids other pedestrians rather
than the robot. Waiting at zero velocity is therefore not guaranteed to remove
an incoming conflict.

## Software integration gaps

### Confirmed: simple discovery does not recover displaced viewpoints

`TransportMission.rotate()` raises `ViewpointDisplaced` after its drift guard and
checked braking. The basic `discover()` scan loop does not handle that exception.
The localization retry decorator only catches `LocalizationInterrupted`.
Therefore a safety-induced sidestep becomes a terminal task failure rather than
a bounded reobservation/replanning transition.

### Confirmed: the scan's displacement check ends before braking finishes

The drift check runs in `aligned()`. Once alignment/discovery completes,
`brake_motion()` can translate further, and `rotate()` does not recheck total
viewpoint displacement or final heading afterward. The three-person second turn
returns after substantial braking translation without a displaced-viewpoint event.

### Confirmed: the next ordinary discovery scan lacks a fresh settling gate

`rotate(..., settle_before_turn=False)` is the default, and the basic discovery
loop uses it after a fixed 1.5-second observation pause. Being settled at the
previous brake completion is not proof of being settled after a moving pedestrian
arrives. The next scan in the three-person run starts while the base is already
moving, then quickly trips the 0.35 m guard.

### Confirmed limit: short escapes do not guarantee a safe terminal state

The planner's `_short_horizon_escape()` validates the first second and ranks
later risks. It can explicitly choose recovery from candidates rejected over the
full horizon. The final supervisor also limits clearance-recovery checking to
one second. The logged behavior progresses from these recoveries to emergency
stopping and envelope overlap. This is a multi-step avoidance limitation, not
evidence that the nominal in-place turn should be forcibly retained.

### Contributing concern: planner timing and perception delay

Active planning overruns occur in both runs, more often with three people.
The three-person active planning p95 is approximately 68 ms against a 50 ms
period. At the example avoidance decision, tracks are about 0.358 seconds old at
publication. These delays reduce maneuvering reserve and warrant profiling, but
they do not establish a single timing root cause. Tracks for the approaching
person remain present; this is not a total person-detection outage.

There is no evidence of an ongoing low-state freeze during the failure. Recorded
hold events occur at initial device ticks 0 and 2. Planner-to-supervisor command
lineage matches all eligible samples: 1008/1008 and 1195/1195. That rules out a
recorded envelope mismatch, not every possible downstream actuator issue.

## Recommended fixes, in order

1. **Make survey yielding a mission state.** Allow checked crowd avoidance to
   finish, establish a safe observed viewpoint, then reacquire the target with
   bounded retries under the original deadline. Preserve the displacement guard.
2. **Validate the whole turn/brake/observe transition.** Recheck heading and total
   displacement after braking, and require fresh measured settling before every
   new survey scan. A fixed pause must not substitute for measured state.
3. **Improve sustained escape and safe-stop selection.** Replay these recorded
   conflicts against multi-step candidates, including braking and terminal
   pedestrian motion. Demonstrate that any candidate improves the retained
   failures before enabling it broadly.
4. **Capture the zero-command interaction precisely.** Use a short, targeted
   control/force diagnostic near the overlap to distinguish contact push from
   residual balance response. Include the simulator shutdown interval in outcome
   reporting, or label its exclusion explicitly.
5. **Then qualify the demo in layers:** zero-person survey/pickup; the same
   three-person seed with survey only; full three-person delivery; multiple seeds.
   Longer pedestrian start delay is useful for isolation, not proof that crowd
   interaction is fixed.

The earlier arm-transition balance/retention problems remain separate downstream
work. These two failures never exercised the walk-then-reach change.

## Reproduce the offline assessment

```bash
python3 scripts/assess_transport_scan.py \
  ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/20261024T185744Z \
  ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/20261024T185845Z \
  --output docs/results/transport_scan_assessment_repeat.json
```

The output name must be new. Relevant implementation:
[mission and evaluator](../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/transport_demo.py),
[planner](../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/human_aware_motion_planner.py),
[supervisor](../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/human_collision_avoidance.py),
and [localization exception handling](../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/localization_recovery.py).
