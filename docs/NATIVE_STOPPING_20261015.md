# Delayed rollout timing and native stopping evidence

## Outcome

Eight held-empty probes pass the declared sampled stopping gate **4/8**. All four
negative-lateral published-feedback probes fail; one falls. Three integrated
follow-ups complete **1/3**, with no recorded contacts or falls. These results do
not demonstrate a speedup or improved mission reliability. People-present
delivery remains unqualified.

[Stopping report](results/native_stopping_20261015.json) ·
[All three mission outcomes](results/rollout_timing_20261015.json)

## Timing correction

`MotionEnvelope.rollout` previously delayed the first command but selected the
following command against an undelayed duration. It now places the handoff at
`delay + command_duration` and splits integration at both arrival boundaries,
preserving the output time grid. Response fractions use the actual subinterval;
short pulses survive, pre-braking coast is retained, and a zero target still has
a stopping trajectory.

The first full CI attempt exposed ten scalar/vectorized parity failures. The
vectorized schedule checker and sequence search now also split at delayed phase
arrivals, including delays spanning multiple phases. Thirty randomized parity
cases cover ordinary handoffs, short pulses and long delays. No fitted response
gain or time constant was promoted: the measured behavior below does not support
a single stopping-decay calibration.

## Native probes

All probes use cycle-05 (`4582897893b9730781b2e2ddd682eb312479184b636582efb4f67d59fc338c6c`),
synchronous MuJoCo 3.3.6, 15 seconds requested duration, 40 ms application delay,
and prescribed carry arms **without a payload**. Commands start at 5 seconds.
There is no ROS or camera workload. Travel is maximum displacement from the first
post-command sample, not path length or exact actuator-stop displacement.

| Lateral command | Command duration | Action feedback | Travel | Settled by | Gate |
| ---: | ---: | --- | ---: | ---: | --- |
| +0.45 | 3 s | Published | 0.192 m | 2.60 s | Pass |
| +0.30 | 3 s | Published | 0.060 m | 2.32 s | Pass |
| −0.45 | 3 s | Published | 0.639 m | 5.36 s | Fail |
| −0.45 | 2 s | Published | 3.183 m | Never | Fail |
| −0.45 | 4 s | Published | 1.459 m | Never | Fail |
| −0.30 | 3 s | Published | 3.257 m | Never | Fail; fall |
| −0.45 | 2 s | Applied | 0.158 m | 2.28 s | Pass |
| −0.30 | 3 s | Applied | 0.135 m | 2.34 s | Pass |

The two matched applied-feedback probes improve markedly, but use simulator-known
action history. Deployment lacks that actuator acknowledgement; guessed applied
feedback is not substituted into the deployed policy. Held-empty results do not
qualify loaded motion, crowds, hardware, or physical finger grasping.

The scorer requires travel ≤0.30 m, speed ≤0.08 m/s and yaw rate ≤0.12 rad/s for
a continuous 1 s dwell completed by 3 s, at least 5 s post-stop observation,
the full requested probe window, sample gaps ≤0.05 s, no resumed motion, no fall
and no sampled nonfloor robot contacts. Invalid/reordered/nonfinite samples,
missing coverage, replay/ramp inputs and nonzero post-stop commands fail closed.
Sampled contacts cannot establish continuous coverage. Reports retain input and
scorer hashes; `--require-pass` saves failures before returning nonzero.

Two initial launches failed before producing probe artifacts because the plain
MJLab environment lacked `onnxruntime` and system Python lacked `mujoco`. These
are setup failures, not physics outcomes. The working dependency-isolated entry is:

```bash
docker compose run --rm --no-deps -T unitree-g1-sim \
  /workspace/scripts/run_mjlab.sh --with onnxruntime==1.23.2 --with mujoco==3.3.6 \
  python /workspace/scripts/probe_native_policy.py --help

python3 scripts/score_native_stopping.py \
  ros2_ws/src/dynamic_obstacle_benchmarks/results/response_probe_20261015 \
  --output /tmp/native_stopping_review.json --require-pass
```

Use a new output path for each score report; existing reports are not overwritten.

## Integrated follow-ups

All three use `transport_dual` exploration and cycle-05. The report retains source
archive/result hashes and attachment state. Times on failures are last recorded
events, not completion times.

| Run suffix | People / seed / task | Outcome | Time |
| --- | --- | --- | ---: |
| `182645Z-rollout-timing-control` | 0 / 48 / red → blue | Fresh pose/destination unavailable after lowering; release withheld, object held | 145.99 s |
| `182955Z-rollout-timing-person` | 1 / 46 / red → red | Braking timeout; empty attachment | 43.32 s |
| `183223Z-rollout-timing-repeat` | 0 / 48 / red → blue | Complete; empty attachment | 169.04 s |

These runs precede the final vectorized parity correction found by CI. They are
retained development evidence, not native qualification of the final tree.
The next qualification step is controlled stopping/encounter replay with measured
action history and actual payloads, followed by final-tree integrated repeats.

## Verification

Final `test_workspace_inside.sh ci` passed: **866 ROS results**, zero errors,
failures or skips, including **670 bridge tests**; **172 orchestration tests**,
**12 native CTests**, deterministic mock benchmark and MJLab environment check.
The focused scorer/evidence suite passed 21 tests. The initial parity failure is
retained above rather than represented as an uninterrupted green run.
