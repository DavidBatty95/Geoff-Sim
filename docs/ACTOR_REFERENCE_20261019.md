# Successful-state actor rehearsal — 19 October 2026

An optional supervised actor-reference loss now rehearses successful native
movement and stopping states during PPO training. The matched two-seed test
shows a **small condition-specific gain for seed80**, but **regression for
seed81**. No candidate meets the unchanged all-conditions promotion gate.

## Implementation and corpus

`probe_native_policy.py --record-observations --record-actions` retains exact
480-element actor inputs and published 29-element action vectors.
`build_actor_reference.py INPUT... --output NEW.json` admits only sources that
pass both strict stopping and movement progress. It retains source/policy hashes
and excludes initial standing, keeping movement and post-stop observations.

Four source cases use `braking79`, loaded and held-empty fixtures, both lateral
signs, 60 ms delay, and 2.5 s movement. All passed. The resulting corpus has
2,000 observation/action pairs: 500 moving and 1,500 stopped samples. It is
narrow rehearsal coverage, not a general motion dataset. Source conditions
are part of the historical regression set and are **training-exposed** here.

Training flags `--actor-reference CORPUS --actor-reference-weight 10` add
10 times action-vector mean squared error to PPO's loss. The implementation
cycles deterministically through 256 stored samples per minibatch, avoiding
extra consumption of PPO's random stream. Targets are fixed and gradient-free;
actor gradients are combined with PPO gradients before clipping and optimization.
The feature requires an unnormalized feed-forward actor and a corpus whose
policy hash matches the checkpoint's sibling ONNX policy.

The installed RSL-RL version exposes an auxiliary-loss interface via its symmetry
slot. This adapter performs **no symmetry augmentation**; the console's
`symmetry loss` label denotes reference MSE in these runs. Audits explicitly
name the actual objective. Existing symmetry objectives cannot be overwritten.
Reference data, implementation snapshots, manifest hashes and loss audits are
retained with training artifacts. Exported actors need no reference data at runtime.

## Matched experiment

The [predeclared plan](results/actor_reference_plan_20261019.json) starts four
100-iteration runs from `braking_seed79_20261019/final.pt`, pairing constrained
and unconstrained training for seeds80/81. All use 256 environments, stopping
reward, transition commands, mixed loads, native timing/friction, Euler,
0.9 soft limits, published action feedback, fixed 0.12 m load offset and
0–60 ms application delay. Each run completed 614,400 environment steps.

Both constrained runs executed 2,000 auxiliary-loss updates. Mean minibatch MSE
was 0.00013045 for seed80 and 0.00016281 for seed81. Low rehearsal error does
not guarantee closed-loop trajectory preservation, as the failures demonstrate.

## Results

All entries are combined movement-and-stop passes. Falls are included among
failures. Every planned case was attempted; no process/protocol errors occurred.

| Policy | Development /10 | Historical /24 | Fresh /24 | Total /58 | Falls |
| --- | ---: | ---: | ---: | ---: | ---: |
| Reference seed80 | 9 | 20 | 23 | **52** | 0 |
| Unconstrained seed80 | 5 | 21 | 21 | 47 | 0 |
| Reference seed81 | 6 | 18 | 18 | 42 | 3 |
| Unconstrained seed81 | 10 | 22 | 18 | 50 | 1 |
| Parent braking79 | 9 | 20 | 22 | 51 | 0 |

Development covers loaded 60 ms delayed stops at 1.96/1.98/2/2.02/2.04 s.
Historical regression covers durations 1.5/2.5/3 s; fresh validation covers
1.6/2.4/2.9 s. Both latter sets cross two lateral signs, loaded/held-empty
fixtures and 40/60 ms delay. Probes request 15 s, use native MuJoCo 3.3.6,
sensor/sensor feedback and integer physics-step command timing. All gates,
including contact, attachment, fall, progress, travel and deadline, are unchanged.

There are 290 evaluation probes plus four corpus-collection probes. Fresh
durations differ from earlier reports, so aggregate counts are not directly
comparable across rounds. Seeds share a trained parent. The known-case total
is descriptive and partly training-exposed, not independent reliability evidence.

## Decision and next work

Seed80 reduces failures from 7/58 to 6/58 versus the parent on this set and
from 11/58 to 6/58 versus its matched control. Seed81 reverses the control
comparison and introduces three falls. This does not establish a repeatable
benefit for the constraint, and no integrated candidate mission was run.

The next experiment should broaden successful-state coverage across delays and
stop phases, and measure per-case fixes/regressions rather than relying on mean
action error. The narrow four-trajectory corpus and a single weight cannot
establish which coverage or coefficient preserves behavior. Fresh validation
must be declared again before tuning. Magnetic fixtures and synchronous probes
remain distinct from physical-grasp and asynchronous ROS mission qualification.

## Artifacts

- [Development](results/reference_development_20261019.json)
- [Historical regression](results/reference_regression_20261019.json)
- [Fresh validation](results/reference_validation_20261019.json)

Raw sources/corpus: `ros2_ws/src/dynamic_obstacle_benchmarks/results/actor_reference_sources_20261019/`.
Raw matrices: `ros2_ws/src/dynamic_obstacle_benchmarks/results/reference_{development,regression,validation}_20261019/`.
Training: `vendor/mjlab/logs/payload_training/{reference,unconstrained}_seed{80,81}_20261019/`.
The separate `reference_smoke_20261019` is an implementation check, not a candidate.

## Verification

Full workspace CI passed: 218 orchestration tests, 866 ROS test results, native
checks, deterministic mock benchmark and MJLab helper check. The separate MJLab
training/reference suite passed 27 tests, including gradient direction,
fixed targets, unchanged random state and rejection of malformed/unqualified
corpora. A real GPU smoke run exercised the auxiliary loss through PPO and
checkpoint/ONNX export. Documentation, graphics, repository hygiene, reviewed
source patches and whitespace checks passed.
