# Published-feedback stopping candidate and incident replay

## Outcome

The new cycle-07 candidate passes **2/4** matched native loaded stopping conditions,
compared with **1/4** for cycle-05. Both negative-lateral candidate conditions still
fail the unchanged gate. **Cycle-07 remains experimental; cycle-05 remains the
deployment baseline.** This is a small controlled diagnostic, not reliability or
people-present delivery qualification.

[All eight scored probes](results/stop_candidate_20261015.json) ·
[Training provenance](results/stop_candidate_training_20261015.json) ·
[Both command-only replays](results/stop_replay_20261015.json)

## Training change

Training already supported per-episode application-delay randomization and the
deployed published-action observation convention. `--objective stopping` now adds
a reward penalty only when all three commanded velocities are exactly zero:

`-4 × (vx² + vy² + 0.25 × yaw_rate²)`.

The reactive tracking, uprightness and turn-drift objectives remain active, so
stopping is not rewarded in place of obeying a nonzero movement request. Physical
velocity is reward/critic information only; the actor retains the existing
480-observation/29-action measured-arm contract without base linear velocity.

The opt-in `--probe signed_stops` evaluation schedule covers ±0.45 lateral
commands for two and three seconds, each followed by six seconds at zero command.
Its episode limit covers the whole schedule. The native qualification below uses
separate fixed-command probes, avoiding dependence between consecutive conditions.

One fine-tune used **256 environments, 100 iterations (614,400 environment steps),
seed 76**, starting from cycle-05. Application delay was sampled from 0–2 control
periods (0–40 ms), with published-action feedback, mixed unloaded/held/loaded
profiles, native-contact feet and a fixed carry-pose bank. Sensor delay, missed
updates and arm transitions were disabled in this diagnostic candidate. Actor and
critic learning rates were 1e-5 and 3e-4. This combined candidate is not a
reward-only ablation. Training uses a rigid wrist load at 0.19 m; the native probe
uses the magnetic capture fixture at 0.12 m. That geometry mismatch remains.

## Matched native evaluation

Both policies use MuJoCo 3.3.6, a 150 g payload, published-action feedback, 40 ms
application delay, 20 ms observation cadence and 15 seconds total observation.
All eight stayed upright, retained the load and recorded no nonfloor robot or
payload/scene contacts. All action-history audits passed.

| Direction / command duration | Cycle-05 travel / dwell completion | Cycle-07 travel / dwell completion | Strict result: baseline → candidate |
| --- | --- | --- | --- |
| +0.45 / 2 s | 0.135 m / 2.56 s | 0.128 m / 2.56 s | Pass → Pass |
| +0.45 / 3 s | 0.183 m / 6.40 s | 0.150 m / 2.82 s | Fail → Pass |
| −0.45 / 2 s | 0.738 m / 5.42 s | 0.309 m / 4.24 s | Fail → Fail |
| −0.45 / 3 s | 0.518 m / 4.60 s | 0.362 m / 3.12 s | Fail → Fail |

The negative two-second case reduces sampled travel by about **58%**, and the
three-second case by about **30%**, in these exact conditions. Both still exceed
the **0.30 m travel and 3 s dwell-completion** limits. No rounding or threshold
relaxation upgrades them to passes. Mean lateral speed during movement remains
similar: baseline/candidate −0.443/−0.421 m/s for the negative two-second request,
and −0.457/−0.440 m/s for three seconds. Reduced travel is not simply a stationary
policy ignoring the command, but broader tracking qualification is still needed.

## Replay finding and coverage correction

The retained one-person failure `20261015T202400Z-loaded-check-person` supplied
3,567 filtered-command samples over 80.687 seconds from the first movement. A
60-second-offset command-only replay starts after five seconds standing, with
30 seconds requested duration and actors parked. Both policies use the same
trace, neutral starting state and 40 ms application delay.

- Cycle-05 contacts the pickup table (21 sampled contacts) and falls at **17.26 s**.
- Cycle-07 records table/leg contacts (24 sampled contacts), stays upright and
  finishes the 30-second replay.

Neither is an acceptable replay outcome. The neutral pose does not reproduce
the incident's starting position, gait state, sensor history or moving actors;
the contacts therefore cannot establish the cause of the original braking timeout.
The fixed-command scorer explicitly rejects both replays as the wrong protocol;
their metadata and failures remain published. A synchronized physical-state and
policy-history snapshot is needed for faithful incident replay.

`load_commands` now rejects gaps exceeding **250 ms after motion starts** rather
than silently holding the previous command across missing evidence. A regression
checks this rejection, and the real incident trace passes the coverage gate.

## Reproduction

```bash
docker compose -f docker-compose.yml -f docker-compose.gpu.yml run --rm --no-deps -T unitree-g1-sim \
  /workspace/scripts/run_mjlab.sh python /workspace/scripts/g1_payload_training.py \
  --mode train --num-envs 256 --iterations 100 --seed 76 --dynamics native_contact \
  --max-delay-steps 2 --action-feedback published --objective stopping \
  --smoothness-weight .05 --profile mixed \
  --checkpoint /workspace/vendor/mjlab/logs/payload_training/native_contact_cycle_05_20261012/final.pt \
  --output /workspace/vendor/mjlab/logs/payload_training/stop_feedback_review
```

Use the [loaded probe command](LOADED_STOPPING_20261015.md#reproduction-and-verification)
with each policy, `--linear-y .45` or `-.45`, and `--command-duration 2` or `3`.
All outputs must use new paths. The exported candidate SHA-256 is
`2230f6462bed6b707ca3276a55ef5b5e2c51a7ee0a9d5e43fe9372673af5e3fe`.

## Verification and next step

**12 training-contract tests** passed in the MJLab environment, covering reward
scope, unchanged actor inputs, feedback selection and signed schedule coverage.
Full CI passed **866 ROS results (670 bridge), 183 orchestration tests and
12 native CTests**, the deterministic mock benchmark and MJLab environment check.

Next: a reward-only matched training ablation, additional initial states/delays
and held-empty checks, and synchronized incident-state capture. The remaining
negative-lateral failures must pass the existing gates before considering policy
promotion or wider integrated acceptance.
