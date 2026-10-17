# Matched-state actuator audit and deployment-target alignment

## Outcome

Two concrete training/native mismatches are measured and now have explicit,
tested training options: **integration method** and **position-target soft limits**.
They improve model-contract fidelity, but the resulting policies remain
**unpromoted**. Euler-only training passes **1/10** dense stopping conditions;
Euler plus native soft limits passes **0/10 distinct measured conditions**, though
it stays upright in all ten. Cycle-05 remains the deployment baseline.

[Matched-state actuation audit](results/native_actuation_20261017.json) ·
[Initial target audit](results/target_clipping_20261017.json) ·
[Target audit split by motion phase](results/target_clipping_phases_20261017.json) ·
[Euler candidate comparison](results/euler_dense_20261017.json) ·
[Interrupted clipping comparison](results/euler_clipped_dense_20261017.json) ·
[Clipping follow-up](results/euler_clipped_followup_20261017.json)

## 1. Equal PD forces do not imply equal integrated motion

Source inspection found that the native probe's compiled model uses **Euler**,
with PD torque computed externally before each physics step. Training uses
MuJoCo built-in position actuators and historically defaults to **implicit-fast**.
Built-in position PD exposes velocity-dependent actuator damping to the implicit
integrator; externally supplied motor torque does not expose that derivative.

`compare_native_actuation.py` reconstructs four retained loaded checkpoints from
cycle-05 and long77, checks their compiled-model identity, and holds the native
plant, state, composed joint targets, gains, effort limits and 2 ms timestep fixed.
Each state is advanced with four actuator/integrator combinations. Derived
quantities are refreshed consistently for the comparison.

| Combination | Maximum joint-velocity difference after one step, relative to native motor/Euler |
| --- | ---: |
| External motor PD / native Euler | Reference |
| External motor PD / implicit-fast | 0 in these four states |
| Built-in position PD / native Euler | ≤2.42×10⁻¹⁵ rad/s |
| Built-in position PD / implicit-fast | **0.0141–0.0519 rad/s** |

Instantaneous actuator generalized forces differ by no more than **1.96×10⁻¹⁴**
between the equivalent PD formulations. The discrepancy appears in integration,
not in the initial force formula. The largest observed joint-position difference
after one 2 ms step is approximately **0.000104 rad**. A separate one-joint
regression reproduces the mechanism and checks force-cap equivalence under both
positive and negative saturation.

This is an **identical-native-plant CPU isolation test**, not full training-model
or CPU/Warp parity. It uses current joint positions/velocities, as the Python
probe does. The ROS bridge calculates PD from simulator joint sensor arrays;
their scheduling/sample boundary is not qualified by this audit.

### Training option

`g1_payload_training.py --integrator euler` makes the choice explicit. The manifest
records requested integration, compiled integration and training MuJoCo version.
The historical `implicitfast` default remains available for reproducibility.
Tests verify that selecting Euler changes integration configuration without
changing command distributions, actuator gains or the actor observation layout.

## 2. Training and deployment differed on target clipping

The deployment adapter converts raw policy actions into joint-position targets,
then applies `clamp_joint_targets(..., soft_limit_factor=0.9)`. The training
position actuators intentionally permit targets outside joint ranges, and the
previous action path did not reproduce this deployment clipping.

`audit_native_targets.py` reconstructs SDK-order targets from retained applied
actions and compares the reconstructed clipped targets against recorded composed
targets. It excludes externally composed arms and checks all 15 leg/waist targets.
The reconstruction error is **zero** in three selected negative-lateral probes.

| Policy | Moving samples affected | Largest moving target change | Post-stop samples affected |
| --- | ---: | ---: | ---: |
| Cycle-05 | 35/100 | 0.361 rad | 90/167 |
| Long77 | 29/100 | 0.284 rad | 177/400 |
| Euler77 | 22/100 | 0.085 rad | 111/141 |

All three have zero clipped standing samples. The largest changes in some runs
occur during unstable post-stop motion, so this audit establishes an active
contract difference, not that clipping alone caused the falls. The initial and
phase-split reports analyze the same three artifacts, not six independent runs.

### Training option

`--target-soft-limit-factor .9` applies the same midpoint/half-range bounds in
policy joint order. It clamps processed position targets after scale/offset and
before external-arm composition. Raw actions and published-action history retain
their deployment meaning. The default remains unbounded for historical training
reproduction. Regression tests compare all 29 mapped target limits against the
deployment transform; provenance now includes the archived `policy_safety.py`.

## 3. Native qualification of both candidates

Both fine-tunes start from cycle-05, with **256 environments, 100 iterations,
614,400 environment steps and seed 77**. They use the reactive objective, mixed
commands/load profiles, published-action feedback and 0–60 ms delay exposure.
The first selects Euler; the second adds factor-0.9 target limits. No stopping
reward or transition-focused command curriculum is enabled.

Training reports **MuJoCo 3.8.1** with compiled Euler, while qualification uses
**MuJoCo 3.3.6**. The native audit holds the plant fixed specifically to isolate
actuation semantics; it does not remove that version/backend difference. The
rigid training payload offset (0.19 m) and magnetic native probe offset (0.12 m),
other collision geometry and sensor timing also remain distinct.

Fresh native conditions use a 150 g payload, 60 ms application delay, published
feedback, the integer-step command clock, both lateral signs at ±0.45, and five
durations: **1.96, 1.98, 2.00, 2.02 and 2.04 s**. Each requests 15 s observation.

| Policy | Strict passes | Falls | Fully passing five-offset neighborhoods |
| --- | ---: | ---: | ---: |
| Cycle-05 | 0/10 | 8 | 0/2 |
| Retained long77 | 1/10 | 1 | 0/2 |
| Euler77 | 1/10 | 2 | 0/2 |
| Euler + soft limits | 0/10 distinct conditions* | 0 | 0/2* |

Euler77 passes only the positive 1.96 s condition (0.247 m travel, dwell completed
at 2.94 s). The combined candidate's positive stops travel **0.242–0.253 m** but
complete dwell only at **3.16–3.70 s**. Negative stops travel **0.669–1.177 m** and
complete dwell at **4.66–6.54 s**. Staying upright does not turn these into passes.

*The clipping matrix was interrupted by the external command timeout after
**19 retained results out of 20 planned**. Its final launched probe has a log but
no result; the report remains incomplete. A separate two-case follow-up repeats
the positive 2.04 s control and measures the missing negative 2.04 s condition.
Both fail. Together they cover all ten distinct clipping-candidate conditions,
but they are not represented as one uninterrupted completed matrix. The Euler-only
reference's repeated outcomes match the first comparison. All completed records
pass their protocol/action-history audits; the interrupted launch is not a
physics result.

## Reproduction and verification

```bash
docker compose -f docker-compose.yml -f docker-compose.gpu.yml run --rm --no-deps -T unitree-g1-sim \
  /workspace/scripts/run_mjlab.sh python /workspace/scripts/g1_payload_training.py \
  --mode train --num-envs 256 --iterations 100 --seed 77 --dynamics native_contact \
  --max-delay-steps 3 --action-feedback published --objective reactive \
  --smoothness-weight .05 --profile mixed --command-profile mixed \
  --integrator euler --target-soft-limit-factor .9 \
  --checkpoint /workspace/vendor/mjlab/logs/payload_training/native_contact_cycle_05_20261012/final.pt \
  --output /workspace/vendor/mjlab/logs/payload_training/actuation_review
```

Use a new output path; omit the soft-limit option for the Euler-only comparison.
`compare_native_actuation.py --checkpoint PATH... --output NEW.json` runs in the
pinned MuJoCo probe environment. `audit_native_targets.py INPUT... --output
NEW.json` runs on retained action-recording probes. Native qualification uses the
[dense matrix command](DENSE_TRANSITIONS_20261017.md#reproduction) with the relevant
policy paths. Full manifests, source hashes and all outcomes are in the linked
reports.

Full CI passes **866 ROS results (670 bridge), 202 orchestration tests and
12 native CTests**, the deterministic mock benchmark and MJLab environment check.
Separate environments pass **16 training-contract tests** and **3 native actuator
tests** covering unsaturated and saturated PD behavior.

Next: extend the paired-state comparison to the full training plant/backend and
the native sensor/application boundary, then repeat controlled training across
seeds or longer optimization budgets. These measured fidelity corrections alone
have not established a qualified stopping policy.
