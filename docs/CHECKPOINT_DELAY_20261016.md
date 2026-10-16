# Exact native-probe continuation and two-seed delay comparison

## Outcome

Native probe checkpoints now reproduce **all recorded samples and outcome fields
exactly** in two continuation tests, including applied/published actions. This
establishes synchronous diagnostic restart, not live ROS incident replay.

A 40-probe comparison tests two training-delay ranges across seeds 76 and 77,
plus cycle-05. Longer-delay training reaches **3/8 strict passes at each seed**,
but still has falls and inconsistent stopping. Four additional longer-command
probes all fail. No candidate is promoted; cycle-05 remains the deployment baseline.

[40-probe matrix and training provenance](results/delay_seeds_20261016.json) ·
[Four duration follow-ups](results/delay_duration3_20261016.json) ·
[Passing continuation comparisons](results/checkpoint_parity_20261016.json) ·
[Initial dtype-related mismatches](results/checkpoint_initial_20261016.json)

## Checkpoint implementation

`probe_native_policy.py` accepts `--checkpoint-at`, `--checkpoint-output` and
`--resume-checkpoint`. A checkpoint is captured **after control is composed but
immediately before a physics step**. It includes:

- MuJoCo's integration state, including time, positions/velocities, controls,
  applied forces, solver warm-start state and active equality constraints.
- All six policy observation-history deques, pending action queue, last published
  and applied actions, their sequence IDs, and float32/float64 controller dtypes.
- Delayed measurement packets, arm targets, filtered commands and outage state.
- The pending physics-step index, next observation step, observation cadence
  index and last observation timestamp.
- Recorded sample prefix and contact counters so the resumed result remains
  comparable with the uninterrupted run.

Restore requires matching engine version, compiled-model hash, policy hash,
configuration and relevant source hashes. The model is reconstructed using the
same fixture; the checkpoint does not permit arbitrary model or policy swaps.
The pending physics step executes once before sensors or contacts are read.
An extra `mj_forward` here would change the sampling boundary. JSON avoids
executable object deserialization, rejects nonfinite numeric data, and checkpoint
files are never overwritten.

The first implementation preserved values but widened queued float32 actions
to float64. The first differing sample in both tests was `action_norm` (about
1e-7 difference). Dtype-preserving serialization corrected this; both initial
failed comparisons are retained. The native helper regression also checks exact
position, velocity and solver warm-start continuation on a contacting body.

### End-to-end continuation results

Both cases use cycle-05, 40 ms application delay and a negative-lateral two-second
request. Snapshots occur between observation updates near the stop transition.

| Case | Actual checkpoint time | Recomputed samples | Exact continuation | Physical outcome |
| --- | ---: | ---: | --- | --- |
| Native 150 g payload, 20 ms cadence | 6.988 s | 400 | Pass | Upright through 15 s |
| Held-empty, 6 ms sensor delay, alternating 14/26 ms observations, measured-hold outage at 7.5 s | 6.992 s | 254 | Pass | Reproduces fall at 12.054 s |

The complete outputs contain 750 and 604 samples respectively; only the suffix
after capture is recomputed. The comparison checks the complete sample arrays
and outcome fields, excluding only capture/resume provenance flags. A reproduced
fall is a successful **continuation test**, not a successful locomotion trial.
The matrix runner now rejects resumed-prefix results as fresh-start qualification.

This state belongs to the synchronous Python probe. Live ROS capture still needs
a shared simulator/policy application boundary, DDS/in-flight inference state,
and synchronized actor/sensor scheduling before an old mission can be faithfully
recreated. Cross-engine/platform continuation is also unqualified.

## Matched delay-training comparison

All fine-tunes start from cycle-05 and use the reactive objective, published-action
feedback, native-contact dynamics, mixed load profiles, 256 environments and
100 iterations (614,400 environment steps). Learning rates and other settings
match the previous no-stopping-penalty ablation. The factors are:

- Training seed **76 or 77**.
- Per-episode application-delay range **0–40 ms** (`short`) or **0–60 ms** (`long`).

Three new runs supply `long76`, `short77` and `long77`; `short76` is the retained
matched ablation. Manifest, source, checkpoint and policy hashes are embedded
in the report. Each policy is evaluated serially on both lateral signs, loaded
and held-empty arms, and 40/60 ms application delays. Commands last two seconds;
probes request 15 seconds with the existing stopping gates.

| Policy | Strict passes | Falls | 60 ms passes |
| --- | ---: | ---: | ---: |
| Cycle-05 baseline | 2/8 | 2 | 0/4 |
| Short-delay seed 76 | 3/8 | 1 | 0/4 |
| Long-delay seed 76 | 3/8 | 1 | 0/4 |
| Short-delay seed 77 | 2/8 | 3 | 0/4 |
| Long-delay seed 77 | 3/8 | 2 | 1/4 |

All 40 attempts produced results and passed the protocol/action-history audits.
Long77's loaded negative-lateral 60 ms case stops within **0.255 m** and completes
its stationary dwell at **2.38 s**, while short77 falls in that condition. But
long77's loaded negative 40 ms case still takes **3.38 s** to settle, and both
held-empty 60 ms cases fall. Delay effects remain state-dependent and nonmonotonic.
Two training seeds do not establish general reliability or a statistically robust
improvement.

### Longer-command follow-up

Four separately declared loaded 60 ms probes extend the request to three seconds:

| Candidate / direction | Travel | Dwell completed | Result |
| --- | ---: | ---: | --- |
| Long76 / positive | 0.327 m | 3.64 s | Fail |
| Long76 / negative | 2.047 m | Never | Fail |
| Long77 / positive | 0.204 m | 3.08 s | Fail |
| Long77 / negative | 1.851 m | Never | Fall |

The two-second long77 success does not survive this duration change. No tolerance
or deadline is relaxed to promote a candidate. The next training experiment
should target transition/gait-state coverage rather than treat a wider delay
range alone as sufficient.

## Reproduction

Training uses the previous [matched training command](STOPPING_CANDIDATE_20261015.md#reproduction)
with `--objective reactive`, `--seed 76` or `77`, and `--max-delay-steps 2` or `3`.
Use a new output directory for each run.

```bash
docker compose run --rm --no-deps -T unitree-g1-sim \
  /workspace/scripts/run_mjlab.sh --with onnxruntime==1.23.2 --with mujoco==3.3.6 \
  python /workspace/scripts/verify_native_probe_checkpoint.py \
  --policy /workspace/vendor/mjlab/logs/payload_training/native_contact_cycle_05_20261012/policy.onnx \
  --output /workspace/ros2_ws/src/dynamic_obstacle_benchmarks/results/checkpoint_review

python3 scripts/verify_native_probe_checkpoint.py \
  --output ros2_ws/src/dynamic_obstacle_benchmarks/results/checkpoint_review \
  --export-report /tmp/checkpoint_review.json
```

For an individual probe, add `--checkpoint-at 6.987 --checkpoint-output NEW.json`
to the normal invocation. Repeat with the identical configuration, a new output
path, and `--resume-checkpoint NEW.json` instead of the capture arguments.
`--inspect --output ARTIFACT_DIRECTORY` on the verifier reports the first
differing sample from an existing comparison.

## Verification and next step

Full CI passes **866 ROS results (670 bridge), 192 orchestration tests and
12 native CTests**, deterministic mock benchmark and MJLab environment check.
The pinned MuJoCo suite passes **seven checkpoint tests plus one payload-fixture
test**. Two end-to-end continuation cases pass after the retained dtype fix.

Next: use captured gait states to characterize stop-transition sensitivity,
extend evaluation across command durations and initial states, and design the
shared-boundary capture protocol needed for live ROS incidents. All four
fine-tuned policies remain experimental.
