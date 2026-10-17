# Transition curriculum and dense stop-timing evaluation

## Outcome

The new transition-focused candidate is **rejected for promotion**: it passes
**0/10** dense 60 ms-delay stopping conditions and falls in six. The retained
long-delay seed-77 candidate passes **1/10** with one fall; cycle-05 passes
**0/10** with eight falls. None passes an entire five-offset timing neighborhood.
Cycle-05 remains the deployment baseline, not a claim that it qualifies for these
conditions. The implemented gains are audited training coverage and more honest
neighborhood-level qualification, not improved native stopping reliability.

[30-probe dense matrix, training provenance and request audit](results/dense_transitions_20261017.json) ·
[Eight lower-delay controls](results/transition_delay_controls_20261017.json)

## Opt-in transition curriculum

`g1_payload_training.py --command-profile transitions` adds a command sampler
outside the vendor tree. It retains the reactive reward, measured-arm
480-observation/29-action actor contract and published-action feedback:

- Episodes initially stand, then alternate movement and explicit zero commands.
- Movement duration samples every integer control tick from **75–175** inclusive
  (**1.5–3.5 s** at 20 ms).
- Standing duration samples **250–300 ticks** (**5–6 s**) so training observes a
  substantial post-stop interval before requesting movement again.
- Half of movement-category samples are lateral. Lateral magnitudes span
  0.30–0.45; half of lateral requests use the ±0.45 endpoint. Forward/reverse,
  diagonal, arc and yaw requests remain in the mixture.

The existing command manager decrements a floating-point timer before testing
zero. The sampler places each deadline halfway between its intended control
ticks, preventing accumulated roundoff from adding a control update. A regression
checks every duration from 75 through 300 ticks against the actual countdown
operation. This is training command-timing logic, not a deployed stopping rule.

The experimental `transition_seed77_20261017` run starts from cycle-05 and uses
**256 environments, 100 iterations / 614,400 environment steps, seed 77**, native
foot-contact dynamics and 0–60 ms application-delay exposure. Sensor lag, missed
updates and arm transitions are disabled. The explicit stopping reward from the
earlier ablation is not enabled. Multiple command-distribution changes form one
candidate; this is not a single-factor causal ablation.

### Audited exposure

The retained command audit records **327 positive lateral requests, 370 negative
lateral requests and 1,266 move-to-stop requests**. Every one of the 101 movement
duration ticks was sampled. Counts include requests across resets and do not
assert that the robot completed each transition successfully.

The exporter now retains the audit, its hash and the command-sampler source hash.
Candidate policy SHA-256:
`81124134729f6671fe347d8e7c2bf5efe23e504ed7270dcc14f5e7e43b6c1a20`.
The retained `long77` reference uses the earlier mixed command sampler; it was
not retrained during this increment. Training and native load geometry still
differ as documented in the prior investigations.

## Dense native timing sweep

All 30 probes use MuJoCo 3.3.6, a native **150 g payload**, published-action
feedback, **60 ms** application delay, 20 ms observation cadence, 15 s requested
duration and the explicit **physics-step command clock**. The command begins at
step 2500 (5 s). Both lateral signs are tested at command durations
**1.96, 1.98, 2.00, 2.02 and 2.04 s**, spanning five consecutive stop updates.
All processes produced results and passed protocol/action-history audits.

| Policy | Strict passes | Falls | Fully passing five-offset neighborhoods |
| --- | ---: | ---: | ---: |
| Cycle-05 baseline | 0/10 | 8 | 0/2 |
| Earlier long-delay seed 77 | 1/10 | 1 | 0/2 |
| Transition-focused seed 77 | 0/10 | 6 | 0/2 |

The only passing condition is the earlier long77 policy's positive-lateral
2.04-second request: **0.210 m travel, dwell completed at 2.92 s**. Its four
adjacent positive-lateral conditions all miss the settling deadline. Selecting
only that successful duration would hide the surrounding failures.

For the transition-focused candidate, positive-lateral travel ranges from
**1.300–1.989 m**, and negative-lateral travel from **0.745–2.448 m**. Both signs
fail every tested offset. The training request audit confirms exposure occurred,
but exposure and training return did not transfer into qualified native stops.

### Neighborhood summaries

The matrix report now groups conditions by policy, load, delay and direction.
Each group lists the declared durations, attempted/passing counts and worst
sampled travel. `all_passed` requires every declared case to be present and pass;
missing attempts or duplicate case identities cannot complete the neighborhood.
This is a finite tested-set criterion, not a guarantee for untested offsets.
The host exporter records its own hash separately from the measured runner hash.

## Lower-delay controls

Eight follow-ups keep the payload, integer clock and a two-second command, while
testing zero and 40 ms delay on the two fine-tuned policies. Both pass **3/4**,
with no falls. These controls are separate from the dense 60 ms matrix.

| Delay / direction | Earlier long77 travel / dwell | Transition77 travel / dwell | Result |
| --- | --- | --- | --- |
| 0 ms / positive | 0.084 m / 1.94 s | 0.089 m / 1.84 s | Both pass |
| 0 ms / negative | 0.074 m / 1.40 s | 0.121 m / 2.70 s | Both pass |
| 40 ms / positive | 0.133 m / 2.54 s | 0.156 m / 2.30 s | Both pass |
| 40 ms / negative | 0.218 m / 3.50 s | 0.543 m / 4.08 s | Both fail |

The new candidate is not universally unable to stop, but it does not improve the
declared qualification set and worsens several relevant conditions. No scoring
threshold, deadline or default deployed policy is changed.

## Reproduction

```bash
docker compose -f docker-compose.yml -f docker-compose.gpu.yml run --rm --no-deps -T unitree-g1-sim \
  /workspace/scripts/run_mjlab.sh python /workspace/scripts/g1_payload_training.py \
  --mode train --num-envs 256 --iterations 100 --seed 77 --dynamics native_contact \
  --max-delay-steps 3 --action-feedback published --objective reactive \
  --smoothness-weight .05 --profile mixed --command-profile transitions \
  --checkpoint /workspace/vendor/mjlab/logs/payload_training/native_contact_cycle_05_20261012/final.pt \
  --output /workspace/vendor/mjlab/logs/payload_training/transition_review

docker compose run --rm --no-deps -T unitree-g1-sim \
  /workspace/scripts/run_mjlab.sh --with onnxruntime==1.23.2 --with mujoco==3.3.6 \
  python /workspace/scripts/run_native_stop_matrix.py \
  --policy baseline=/workspace/vendor/mjlab/logs/payload_training/native_contact_cycle_05_20261012/policy.onnx \
  --policy long77=/workspace/vendor/mjlab/logs/payload_training/delay3_seed77_20261016/policy.onnx \
  --policy transition77=/workspace/vendor/mjlab/logs/payload_training/transition_review/policy.onnx \
  --loads payload --delays 3 --command-durations 1.96 1.98 2 2.02 2.04 \
  --command-clock physics_step \
  --output /workspace/ros2_ws/src/dynamic_obstacle_benchmarks/results/dense_review
```

Use new paths; export summaries on the host using the matrix runner's
`--export-summary ... --report ...` mode. The training audit is copied into each
policy's provenance record, including failed or incomplete training records.

## Verification and next priority

Full CI passes **866 ROS results (670 bridge), 201 orchestration tests and
12 native CTests**, the deterministic mock benchmark and MJLab environment check.
The separate MJLab environment passes **14 training-contract tests**, including
tick-countdown and profile/actor-contract checks.

Before another larger training run, compare training and native dynamics at
paired state/action histories to distinguish transfer mismatch from insufficient
optimization or curriculum coverage. Preserve dense timing-neighborhood checks
when evaluating any subsequent candidate; the present candidate remains unpromoted.
