# Checkpoint stop branches reveal one-update sensitivity

## Main finding

A **single 20 ms observation interval** changes the stopping outcome on the same
parent trajectory. For the long-delay seed-77 candidate, stopping near 7.00 s
travels **0.491 m** and fails; stopping at the next observation near 7.02 s travels
**0.255 m** and passes the branch gate. Cycle-05 shows the opposite deterioration:
the earlier stop stays upright but settles late; the later stop falls.

This supports testing command-transition timing much more densely. It does not
support a universal delayed-stop rule or promotion of a trained policy.

[14 broad stop branches](results/stop_branches_20261016.json) ·
[Four adjacent-update branches](results/stop_branch_tick_20261016.json) ·
[Integer-step clock comparison](results/step_clock_20261016.json) ·
[Legacy-clock control](results/legacy_clock_control_20261016.json) ·
[Continuation regression](results/checkpoint_branch_regression_20261016.json)

## Implemented improvements

### Explicit stop branches

`probe_native_policy.py --resume-checkpoint ... --branch-stop` changes only the
requested command at the first observation after restore. The pending physics
step, delayed action queue, policy history and measured state are preserved.
Branches require a moving fixed-command checkpoint, disallow ramp/replay input,
and record the checkpoint identity, retained-prefix length and actual stop time.

`run_checkpoint_stop_branches.py` predeclares all cases and checks:

- The parent trajectory's complete sample digest is identical across captures
  for each policy.
- Each branch preserves the parent's recorded prefix exactly.
- Position, velocity, attitude, payload state, foot support and leg joint state
  match the parent at the command-change observation.
- The parent is still commanding movement and the branch changes to zero at
  the declared time; policy and checkpoint hashes match.

The scorer uses the **actual branch stop time**, not the parent's scheduled stop.
Branch results carry a distinct diagnostic scope. They share history and are
correlated counterfactuals, not independent acceptance trials. Fresh-start
matrices continue to reject checkpoint continuations.

### Offline support-state diagnostics

`--record-support` records left/right foot normal force against plane geometry,
ankle-body heights and hip/knee positions. These are simulator diagnostics, not
actor inputs or permission to continue moving. The readings follow the existing
probe sampling boundary; they do not claim continuous contact coverage.

### Integer-step command clock

The legacy probe compares accumulated floating-point simulation time against
nominal boundaries. At step 3500, the retained time is
`6.999999999999451`, so a nominal 7 s stop is postponed until the next observation,
near 7.02 s. The nominal 5 s start is also postponed in these runs.

New opt-in `--command-clock physics_step` uses decimal-to-integer conversion and
rounds a non-grid boundary up to its first physics step. Exact 5 s and 7 s
boundaries are steps **2500 and 3500** at 2 ms physics cadence. Observation cadence
still determines when the policy sees a transition. The probe records both
scheduled steps and actual first-moving/first-stopped observation times; scoring
checks those transitions and measures the stop window from the actual stop.

`legacy_float` remains the default for reproducing previous experiments and their
original scoring origin. New-clock results are published separately. The integer
clock improves experimental timing accuracy; it does not alter deployed locomotion.

## Broad branch experiment

Both cycle-05 and `long77` use a native 150 g load, negative lateral command −0.45,
published-action feedback and 60 ms application delay. The reference command
starts nominally at 5 s and continues for four seconds. Seven checkpoints per
policy produce the following earlier stop observations. All prefix/state/hash
checks passed. Parent outcomes are retained alongside every branch.

| Actual stop time, rounded | Cycle-05 travel / outcome | Long77 travel / outcome |
| ---: | --- | --- |
| 6.50 s | 1.127 m; fall | 3.205 m; still moving at end |
| 6.76 s | 1.713 m; fall with scene/payload contact | 0.299 m; late dwell |
| 7.00 s | 0.162 m; late dwell | 0.491 m; late dwell |
| 7.26 s | 2.480 m; fall | 2.097 m; fall |
| 7.50 s | 0.186 m; late dwell | 2.261 m; fall |
| 7.76 s | 1.171 m; fall | 2.963 m; fall |
| 8.00 s | 1.698 m; fall | 2.096 m; fall |

All **14 branches fail** the unchanged stopping criteria. Cycle-05 has five branch
falls and long77 four. Since branch time also changes movement duration and state,
this experiment does not isolate a causal gait-phase feature by itself.

## Adjacent-update follow-up

Four further branches compare the two observations around 7 s directly:

| Policy / stop observation | Travel | Dwell completed | Result |
| --- | ---: | ---: | --- |
| Cycle-05 / ~7.00 s | 0.162 m | 3.64 s | Fail |
| Cycle-05 / ~7.02 s | 1.225 m | Never | Fall |
| Long77 / ~7.00 s | 0.491 m | 5.34 s | Fail |
| Long77 / ~7.02 s | 0.255 m | 2.36 s | Pass |

The last branch reproduces the earlier fixed-probe travel. Its dwell time is
2.36 s from the **actual** zero-command observation; the historical fixed-probe
score was 2.38 s from the **nominal** 7 s boundary. The limits are unchanged.
Both long77 observations have zero measured left-foot normal force and substantial
right-foot load, so a simple left/right-support label does not explain the result.

## Fresh-start clock comparison

Eight additional fixed-command probes compare the two clocks under the same
loaded 60 ms-delay conditions and two-second request. Changing the clock shifts
both start and stop relative to the initial gait; it is not an isolated stop-time
intervention. Every process and protocol audit completed successfully.

| Policy / direction | Legacy travel / result | Integer-step travel / result |
| --- | --- | --- |
| Cycle-05 / positive | 0.321 m; fail | 0.321 m; fail |
| Cycle-05 / negative | 1.225 m; fall | 1.813 m; fall |
| Long77 / positive | 0.214 m; fail | 0.211 m; fail |
| Long77 / negative | 0.255 m; pass | 2.882 m; still moving at end |

Legacy passes **1/4**, integer-step passes **0/4**. These expose timing-sensitive
behavior rather than an improvement in policy reliability. The candidate remains
experimental and cycle-05 remains the deployment baseline.

## Reproduction

```bash
docker compose run --rm --no-deps -T unitree-g1-sim \
  /workspace/scripts/run_mjlab.sh --with onnxruntime==1.23.2 --with mujoco==3.3.6 \
  python /workspace/scripts/run_checkpoint_stop_branches.py \
  --policy baseline=/workspace/vendor/mjlab/logs/payload_training/native_contact_cycle_05_20261012/policy.onnx \
  --policy long77=/workspace/vendor/mjlab/logs/payload_training/delay3_seed77_20261016/policy.onnx \
  --times 6.99 7.01 \
  --output /workspace/ros2_ws/src/dynamic_obstacle_benchmarks/results/stop_branch_review

python3 scripts/run_checkpoint_stop_branches.py \
  --export-summary ros2_ws/src/dynamic_obstacle_benchmarks/results/stop_branch_review/summary.json \
  --report /tmp/stop_branch_review.json
```

The native matrix runner accepts `--command-clock physics_step` or
`--command-clock legacy_float`. Use new artifact paths for every run. Clock and
source hashes are retained in the respective reports; the broad branch experiment
predates the optional clock addition and is kept with its original provenance.

## Verification and next work

Full CI passes **866 ROS results (670 bridge), 200 orchestration tests and
12 native CTests**, deterministic mock benchmark and MJLab environment check.
The separate pinned MuJoCo checkpoint/support suite passes **8 tests**. Both
end-to-end exact continuation regressions still pass after the branch extensions.

Next: use the integer-step protocol for new timing studies, evaluate a dense grid
of command start/stop offsets, and train/evaluate explicit stop transitions across
those states. Foot-load diagnostics can help analyze failures, but these data do
not justify a phase-based waiting controller or simulator-truth onboard decisions.
