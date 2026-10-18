# Native-timing training and payload-offset variation

## Outcome

Training can now preserve the native post-substep sensor epoch for both policy
observations and explicit motor PD. The actual GPU environment passes timing,
gain and partial-reset checks. A short combined candidate was then trained and
evaluated: **0/10** strict passes in the dense 60 ms-delay sweep, versus **0/10**
for the retained reference. Separate 0.19 m offset controls pass **1/4** for the
candidate and **2/4** for the reference. The candidate remains **unpromoted**.

[GPU timing/reset verification](results/training_timing_verified_20261018.json) ·
[Initial GPU contract check](results/training_timing_contract_20261018.json) ·
[20-probe dense comparison and training provenance](results/native_timing_dense_20261018.json) ·
[Eight offset controls](results/native_timing_offset_20261018.json)

## 1. Preserve the measured sensor epoch during training

The previous audit established that native sensor arrays after `mj_step` refer
to the pre-integration state, whereas joint `qpos/qvel` have advanced. The training
environment normally performs another forward pass before constructing observations.
Simply reading those refreshed sensors would lose the native sampling boundary.

The new opt-in `--native-timing` path:

- Adds scalar joint-position/velocity sensors and an orientation sensor on the
  same pelvis IMU site as the gyro.
- Captures a **65-value packet** after each physics substep: 29 positions,
  29 velocities, gyro and quaternion. The packet survives the final forward refresh.
- Builds the existing five-frame **480-observation/29-action** actor input from
  that packet. Commands and published-action history retain their current meanings;
  the critic continues to use fresh privileged state.
- Uses one grouped explicit PD motor implementation, with native per-joint gains
  and effort limits, reading the same captured encoder epoch. Position targets
  remain current; this is feedback timing, not another target delay.
- Updates sampled arm gains on the explicit PD tensors instead of multiplying
  motor controls again through built-in position-actuator gain/bias parameters.
- Marks only reset environments for fresh seeding. Before refreshed reset sensors
  are available, their PD uses the newly reset integrated joints, never the previous
  episode. Surviving environments retain their cached packet.

The `native_sensor_epoch_delta` per-substep metrics hook owns packet capture and
is **required** for this mode; its returned norm is diagnostic only. Disabling
logging does not remove the hook. Native timing requires native-contact dynamics
and Euler integration. Historical training behavior remains the default.

### Actual GPU verification

`verify_training_timing.py` checks 16 environments over three control steps and
a partial reset:

- Cached positions and velocities match the actual pre-step integrated joint
  values **exactly** in the recorded check.
- The Euler identity `q_after − q_sensor = dt × dq_after` holds within
  **2.98×10⁻⁸ rad**.
- Actor joint, gyro and gravity fields match the cached packet **exactly**.
- Per-environment arm gains, per-joint default gains and effort limits are correct;
  motor gain is one and bias is zero, avoiding double application of PD gains.
- Resetting one environment preserves the other environments' packets exactly.

Both the initial check and the final, expanded verification are retained. These
are short zero-action timing-contract checks, not locomotion qualification.

## 2. Diagnostic attachment variation

`--payload-offset-range .12 .19` samples wrist-local load x position per episode,
using per-world body-position updates and recomputed model constants. The y/z
offset stays at 0.01/0 m. This range spans the existing native-probe and rigid
training placements; it is **not a measured hardware grasp distribution**.

The candidate sampled **0.120038–0.189505 m**. The manifest's `hand_offset_m`
remains the nominal compiled placement; the range and sampled bounds describe the
per-world override. The rigid training attachment still differs from a compliant
magnetic weld and physical fingers.

Native probes now accept `--payload-offset-x`, require an attached-load fixture,
and record both the override and effective offset. Matrix protocol checks reject
unexpected offsets. Weld regressions cover the default 0.12 m and explicit 0.19 m
placements. Portable-state audits retain the actual captured offset; compiled
offset matching requires a common offset across the selected states.

## 3. Training candidate

`native_timing_seed77_20261018` starts from cycle-05 and uses **256 environments,
100 iterations / 614,400 environment steps, seed 77**, the reactive objective,
mixed commands and load profiles, published-action feedback, 0–60 ms application
delay, Euler integration, factor-0.9 target limits and native joint friction.
Additional control-period sensor delay, missed updates and arm transitions are
disabled. The native timing and load-range changes form a combined candidate,
not a single-factor causal ablation.

Candidate policy SHA-256:
`40b199c79527850d25801de7b4a9fffdd9ef91e81a8511957e8eb5225ead8f74`.
The reference is the retained Euler-plus-clipping candidate from the prior study.
Source snapshots and full manifests are included in the result reports. Final
review added default-gain metadata and diagnostic references; the measured
training source snapshot is retained separately.

## 4. Native evaluation

All probes use MuJoCo 3.3.6, a 150 g magnetic fixture, the integer-step command
clock, published-action feedback and **sensor-based policy joints and motor PD**.
Training uses the MJLab MuJoCo/Warp environment; engine/backend and attachment
representation differences remain.

### Dense 0.12 m fixture sweep

The matrix covers ±0.45 lateral commands lasting **1.96, 1.98, 2.00, 2.02 and
2.04 seconds**, at 60 ms action delay, with 15 seconds requested observation.

| Policy | Strict passes | Falls | Fully passing timing neighborhoods |
| --- | ---: | ---: | ---: |
| Retained Euler + clipping | 0/10 | 3 | 0/2 |
| Native timing + offset variation | 0/10 | 4 | 0/2 |

The reference's positive cases stay within the travel bound but complete dwell
late (3.08–3.50 s). The new candidate also misses settling deadlines and falls
in two positive and two negative cases. Correct implementation of the timing
contract does not establish that this short optimization run learned robust stops.

### 0.19 m fixture controls

Eight further probes use a two-second command and 40/60 ms delay at the other
endpoint of the sampled offset range:

| Delay / direction | Reference travel / dwell | Candidate travel / dwell | Outcome |
| --- | --- | --- | --- |
| 40 ms / positive | 0.148 m / 2.46 s | 0.111 m / 2.30 s | Both pass |
| 40 ms / negative | 0.237 m / 2.76 s | 0.404 m / 3.52 s | Reference passes; candidate fails |
| 60 ms / positive | 0.276 m / 3.06 s | 0.246 m / 3.44 s | Both fail |
| 60 ms / negative | 1.211 m / fall | 0.324 m / 2.90 s | Both fail |

The candidate stays upright in these four controls, but its final case still
exceeds the **0.30 m** travel bound. No threshold or deadline is relaxed. All 28
native probes produced retained results and passed protocol/action-history checks.

## Reproduction and verification

```bash
docker compose -f docker-compose.yml -f docker-compose.gpu.yml run --rm --no-deps -T unitree-g1-sim \
  /workspace/scripts/run_mjlab.sh python /workspace/scripts/g1_payload_training.py \
  --mode train --num-envs 256 --iterations 100 --seed 77 --dynamics native_contact \
  --max-delay-steps 3 --action-feedback published --objective reactive \
  --smoothness-weight .05 --profile mixed --command-profile mixed --integrator euler \
  --target-soft-limit-factor .9 --native-joint-friction --native-timing \
  --payload-offset-range .12 .19 \
  --checkpoint /workspace/vendor/mjlab/logs/payload_training/native_contact_cycle_05_20261012/final.pt \
  --output /workspace/vendor/mjlab/logs/payload_training/native_timing_review
```

Use new output paths. GPU contract verification runs
`verify_training_timing.py --output NEW.json` in the same GPU environment.
Native matrices use `--joint-feedback sensors --pd-feedback sensors
--command-clock physics_step`; the offset controls add `--payload-offset-x .19`.

Full CI passes **866 ROS results (670 bridge), 205 orchestration tests and
12 native CTests**, deterministic mock benchmark and MJLab environment check.
Separate checks pass **19 training-contract tests**, **two native payload/weld
tests**, and the actual GPU timing/reset verification.

Next: isolate timing and offset variation in matched ablations and evaluate
optimization progress against held-out stopping neighborhoods. The new interfaces
are verified, but this candidate remains rejected for promotion.
