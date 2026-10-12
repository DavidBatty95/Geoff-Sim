# Earlier avoidance and turn-transition diagnostics

2026-10-07 follow-up to [perception and response work](PERCEPTION_RESPONSE_FOLLOWUP.md).

**Implemented:** an experimental forward-overtaking lookahead, response-aware
regressions, measured settling before optional inspection turns, and filtered
command replay diagnostics. **Native efficacy remains unproven:** all three
retained mission attempts failed. The overtaking feature is therefore opt-in,
not the default people-present configuration.

## Specific planning issue

In `20261007T131448Z`, the planner continued selecting forward motion while a
faster walker approached from behind. By the first overlap, all 24 evaluated
candidates were rejected for hard human collision.

The existing terminal check looked farther ahead for backward retreat/holding
commands than for forward motion. Forward motion can also be a losing retreat
when a faster person is following.

The new `being_overtaken()` cue uses world/target-frame positions and velocities,
not the sign of the robot's body-frame forward command. It requires:

- A confirmed track with confidence at least 0.8.
- Human speed of at least 0.25 m/s.
- A person behind the robot along the person's travel direction.
- Robot motion substantially aligned with that direction, with less than
  0.1 m/s cross-track motion.
- A human longitudinal speed advantage exceeding 0.1 m/s.

When enabled, `overtaking_terminal_horizon=6.0` extends the remaining-time
terminal conflict check. This is beyond the ordinary rollout endpoint, not a
six-second certified trajectory. It uses a constant-velocity extrapolation and
adds `score_terms.overtaking_conflict` to the existing terminal-conflict penalty.
It leaves hard body/static collision checks, speed caps, and the existing
terminal-conflict fallback intact. It does not authorize a hard-collision-rejected
trajectory.

Regressions verify heading invariance, distinguish following from being
overtaken, and retain a response-aware sidestep while penalizing the forward
route before its short-horizon collision. A deterministic planar replay starts
lateral avoidance earlier with the feature and retains sampled body separation
above 0.8 m. That replay uses a simplified response model, not the native humanoid.

### Experimental switch

```bash
python3 scripts/run_exploration_transport_demo.py --people 1 --seed 48 \
  --task-seed 48 --duration 120 --early-overtaking-avoidance \
  --policy vendor/mjlab/logs/payload_training/native_delay_cycle_03_20261006/policy.onnx \
  --fast-start
```

Use `--no-early-overtaking-avoidance` for the baseline; it is the default. The
flag affects the generated people-present profile and is recorded in the
manifest. `transport_crowd.py` counts planner snapshots containing the new
penalty to distinguish enabled configuration from actual activation.

## Inspection-turn settling

Optional exploration turns now clear mission intent and wait for fresh measured
odometry velocities: planar speed below 0.08 m/s and yaw rate below 0.15 rad/s,
spanning at least 0.2 seconds of acquisition timestamps. Samples must be no more
than 0.3 seconds old. The wait is capped at two seconds within the existing turn
deadline.

If the base does not settle, no inspection turn is issued. The executive logs a
partial/skipped turn and uses fresh images from its actual heading. Independent
person avoidance can still move the robot during this wait; the inspection does
not override it. Required manipulation alignment and final placement checks are
not relaxed. Tests verify multi-sample settling and no rotation dispatch while
motion continues.

## Native attempts

[Outcomes](results/overtaking_followup_20261007.json) and
[incident diagnostics](results/overtaking_diagnostics_20261007.json) retain:

| Run, 2026-10-07 UTC | Configuration | Outcome |
| --- | --- | --- |
| `142832Z` | Overtaking enabled, before settling change | 19 overlap samples; fall; new penalty present in two planner snapshots |
| `143059Z` | Overtaking disabled, before settling change | Fall during an early inspection turn; zero person-overlap samples; minimum person clearance 1.27 m |
| `150103Z` | Overtaking disabled, settling change present | Fall during navigation; zero person-overlap samples; minimum person clearance 1.58 m |

All used the medium world, one person, seed 48 and task seed 48. Their real-time
exploration paths differed, and the third used changed code. They are not a
controlled efficacy comparison. None completed delivery. The settling change
has not established an end-to-end stability improvement.

All three failures now surfaced as **sustained measured body instability**,
rather than repeatedly retrying localization after a fall. The third trace
explicitly records navigation cancellation and terminal status before exit.

## Isolated dynamics and command replay

[Probe summaries](results/turn_transition_probes_20261007.json) retain raw hashes
for four MuJoCo 3.3.6 diagnostics using the cycle-03 policy:

| Probe | Result |
| --- | --- |
| Two-second `+0.6 rad/s` turn request after standing | Upright; net episode yaw +0.116 rad; maximum tilt 0.066 rad |
| Two-second `-0.9 rad/s` turn request after standing | Upright; net episode yaw -0.552 rad; maximum tilt 0.099 rad |
| Failed baseline's filtered-command timeline, zero policy delay | Upright for 30 simulated seconds; maximum tilt 0.143 rad |
| Same timeline, 40 ms policy delay | Upright for 30 simulated seconds; maximum tilt 0.203 rad |

The short turn pulses used different command magnitudes, so they do not isolate
directional asymmetry or establish steady-state yaw gains. Net yaw includes
standing and post-command portions of the episode.

`probe_native_policy.py --replay-trace` consumes source-stamped filtered commands,
starts the first motion after five seconds standing, parks pedestrian mocaps,
and stops commanding motion after the recorded timeline ends. It rejects
conflicting/reordered samples, missing empty-hand state evidence and recorded
arm manipulation. Fixed commands and prescribed arm composition cannot be mixed
with this replay mode.

These are **not exact incident reconstructions**: the initial dynamic state is
reset, inference cadence is idealized, moving actors and ROS scheduling are
absent, and filtered command publication is not an applied-action acknowledgement.
The stable replays do not prove ROS scheduling caused the falls, nor rule out
static contact or initial-state sensitivity.

Example diagnostic:

```bash
bash scripts/run_mjlab_docker.sh --no-sync --with mujoco==3.3.6 python \
  /workspace/scripts/probe_native_policy.py \
  --policy /workspace/vendor/mjlab/logs/payload_training/native_delay_cycle_03_20261006/policy.onnx \
  --scene /workspace/ros2_ws/src/unitree_mujoco/unitree_robots/g1/scene_29dof_transport_transport_medium.xml \
  --replay-trace /workspace/ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/20261007T143059Z/trace.jsonl \
  --duration 30 --delay-steps 2 --require-mujoco-version 3.3.6 \
  --output /workspace/vendor/mjlab/logs/native_turn_replay_delay2_20261007.json
```

## Next priority from this evidence

Capture time-aligned proprioceptive inputs, applied joint targets and actual
inference/application cadence around live navigation turns. Replay those with
matched initial state, and distinguish command-transition, timing and static
contact effects. Establish a stable fixed-route one-person overtaking scenario
before judging the experimental lookahead through random exploration. No
collision or task-success threshold has been reduced.
