# Matched stopping-reward ablation and delay stress tests

## Result

The added stopping penalty is **not a consistently better choice** in this
single-seed comparison. Across eight matched conditions, cycle-05 and the
stopping-penalty candidate each pass **2/8**; the matched fine-tune without that
penalty passes **3/8**. A separate three-second-command follow-up reverses part
of the advantage: the stopping candidate passes **1/2**, the ablation **0/2**.
All three policies fail every tested **60 ms delay** condition. Neither candidate
is promoted; cycle-05 remains the deployed baseline.

[24-condition matrix](results/stop_ablation_20261015.json) ·
[Four longer-command follow-ups](results/stop_ablation_duration3_20261015.json) ·
[Retained setup failure](results/stop_ablation_setup_failure_20261015.json)

## Matched training

`stop_ablation_cycle_07_20261015` uses the same cycle-05 checkpoint, training source,
seed **76**, **256 environments**, **100 iterations / 614,400 environment steps**,
published-action feedback, 0–40 ms application-delay randomization, native-contact
dynamics, mixed payload profiles and optimizer settings as the stopping candidate.
The selected objective is `reactive` instead of `stopping`, removing only the
explicit zero-command residual-motion reward term from that configuration.
Both training manifests and checkpoint/source hashes are included in the matrix.

The ablation policy SHA-256 is
`17cd8caa3fd46754e8a00d172590cea400ea056a354cce5949245e584745d1d1`.
The stopping candidate is `2230f6462bed6b707ca3276a55ef5b5e2c51a7ee0a9d5e43fe9372673af5e3fe`;
cycle-05 is `4582897893b9730781b2e2ddd682eb312479184b636582efb4f67d59fc338c6c`.
One training seed does not establish a robust causal effect across training seeds.

## Broader native conditions

All use MuJoCo 3.3.6, ±0.45 lateral requests for **two seconds**, published-action
feedback, 20 ms observations, and 15 seconds requested probe duration. The load
factor is either the native **150 g magnetic fixture** or prescribed carry arms
without a payload. The added 60 ms delay lies beyond the candidates' 0–40 ms
training exposure. Each row is one deterministic initial-state condition per policy.

Cells show sampled maximum post-stop displacement and strict outcome. A fall
ends the probe early and also fails the observation-window gate.

| Load / delay / direction | Cycle-05 | Stopping penalty | Ablation |
| --- | --- | --- | --- |
| Loaded / 40 ms / positive | 0.135 m; pass | 0.128 m; pass | 0.125 m; pass |
| Loaded / 40 ms / negative | 0.738 m; fail | 0.309 m; fail | **0.188 m; pass** |
| Loaded / 60 ms / positive | 0.321 m; fail | 0.981 m; fall | 0.236 m; fail |
| Loaded / 60 ms / negative | 1.225 m; fall | 2.048 m; fall | 1.986 m; fall |
| Held-empty / 40 ms / positive | 0.141 m; pass | 0.139 m; pass | 0.127 m; pass |
| Held-empty / 40 ms / negative | 3.183 m; fail | 0.349 m; fail | 0.203 m; fail |
| Held-empty / 60 ms / positive | 1.384 m; fall | 1.717 m; fail | 0.228 m; fail |
| Held-empty / 60 ms / negative | 2.489 m; fail | 0.609 m; fail | 1.349 m; fail |

The loaded 40 ms negative ablation completes its stationary dwell at **2.42 s**,
passing both travel and settling gates. Its held-empty counterpart takes **3.08 s**
and fails despite short travel. The 60 ms positive ablation takes **3.14 s loaded**
and **3.38 s held-empty**; these remain failures. Thresholds are unchanged.
Recorded falls total **2 baseline, 2 stopping, 1 ablation**. All 24 processes
produced scoreable results, and all action-history/protocol checks passed.

### Three-second command follow-up

Four additional loaded probes retain 40 ms delay and extend the command to three
seconds. This is a separately declared follow-up, not part of the original matrix.

| Direction | Stopping penalty | Ablation |
| --- | --- | --- |
| Positive | 0.150 m; dwell 2.82 s; **pass** | 0.152 m; dwell 3.08 s; fail |
| Negative | 0.362 m; dwell 3.12 s; fail | 0.547 m; dwell 4.36 s; fail |

Thus the no-penalty candidate's advantage in the two-second loaded negative case
does not generalize to all durations. Additional seeds and initial gait states
are required before selecting a training objective for promotion.

## Reproducibility improvement

`scripts/run_native_stop_matrix.py` replaces manual sequences with a serial,
predeclared condition grid. It:

- Writes the entire plan and policy/training/source hashes before any probe.
- Retains process failures, timeouts, missing/invalid results and protocol
  mismatches; later conditions still run after a failed probe.
- Checks policy identity, requested commands, load, observation cadence, engine
  version and delay settings; recorded application ages must also match the
  declared fixed delay.
- Saves the artifact summary atomically after each attempt. Planned and attempted
  counts remain distinct when a run is interrupted. Completion does not mean
  qualification; any failing condition returns a nonzero exit status.
- Exports portable summaries on the host while preserving raw artifacts locally.

The first matrix launch attempted to publish directly into the container's
read-only documentation mount. It failed **before any physics probe**. Its plan
is retained as incomplete with 24 planned and zero attempted conditions. The
retry writes to the writable results mount and exports from the host. Final
review added per-policy counts and stricter malformed-result handling; measured
runner hashes remain in the reports.

```bash
docker compose run --rm --no-deps -T unitree-g1-sim \
  /workspace/scripts/run_mjlab.sh --with onnxruntime==1.23.2 --with mujoco==3.3.6 \
  python /workspace/scripts/run_native_stop_matrix.py \
  --policy baseline=/workspace/vendor/mjlab/logs/payload_training/native_contact_cycle_05_20261012/policy.onnx \
  --policy stopping=/workspace/vendor/mjlab/logs/payload_training/stop_feedback_cycle_07_20261015/policy.onnx \
  --policy ablation=/workspace/vendor/mjlab/logs/payload_training/stop_ablation_cycle_07_20261015/policy.onnx \
  --output /workspace/ros2_ws/src/dynamic_obstacle_benchmarks/results/stop_review

python3 scripts/run_native_stop_matrix.py \
  --export-summary ros2_ws/src/dynamic_obstacle_benchmarks/results/stop_review/summary.json \
  --report /tmp/stop_review.json
```

Use new output/report paths. Defaults are both load states, delays 2/3 observation
periods, both lateral signs and a two-second command. The follow-up specifies
`--loads payload --delays 2 --command-durations 3` and only the two candidates.

## Replay-state requirements

Inspection confirms that recorded filtered commands cannot restore the policy's
five-frame per-term observation deques, pending action queue, delayed measurement
buffer, arm targets or physics/controller scheduling phase. Robot pose alone also
omits joint velocities, contacts, solver state, weld transforms and actor state.
Faithful incident replay needs a coherent physics/policy checkpoint at a shared
application boundary, followed by a continuation-parity test. This increment
adds reproducible comparisons; synchronized ROS incident capture is still pending.

## Verification and next work

Full CI passes **866 ROS results (670 bridge), 191 orchestration tests and
12 native CTests**, plus the deterministic mock benchmark and MJLab environment
check. Eight new matrix regressions cover identical condition pairing, failed
process continuation, timeout/missing results, changed policy identity and
incomplete-plan export.

Priorities: repeat the matched ablation across training seeds, expose candidates
to measured longer-delay distributions, test varied initial gait states, and
implement synchronized replay checkpoints. The duration-dependent tradeoff and
60 ms falls rule out promoting either candidate from this evidence.
