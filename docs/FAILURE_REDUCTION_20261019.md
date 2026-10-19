# Longer-budget stopping comparison — 19 October 2026

The selected `timing_fixed` candidate reduced failures on the declared held-out
native conditions from **18/24 to 6/24**, with **zero falls versus seven** for
cycle-05. This is condition-specific evidence, not deployment qualification.
The candidate still fails the all-conditions gate and has not been promoted.

## Matched training

The [predeclared plan](results/failure_reduction_plan_20261019.json) compares
three 400-iteration runs (256 environments, seed 77), each starting from
cycle-05. All use native-contact dynamics, Euler, native joint friction,
0.9 soft target limits, published action feedback, 0–60 ms application delay,
the reactive objective, 0.05 smoothness weight and mixed commands/load profiles.
`timing_fixed` uses native sensor timing and a fixed 0.12 m payload offset;
`timing_varied` uses native timing and 0.12–0.19 m offsets; `fresh_fixed` uses
fresh feedback and the fixed offset. Each completed 2,457,600 environment steps.
Artifacts are retained under `vendor/mjlab/logs/payload_training/`, in directories
named `<arm>_seed77_20261019`, including final checkpoints, ONNX policies,
manifests and console logs.

## Development results

Ten conditions per policy: 150 g payload at 0.12 m, 60 ms application delay,
signed 0.45 m/s lateral commands, and durations 1.96/1.98/2/2.02/2.04 s.

| Policy | Combined passes | Falls |
| --- | ---: | ---: |
| timing_fixed | 5/10 | 0 |
| timing_varied | 5/10 | 0 |
| fresh_fixed | 5/10 | 0 |
| cycle-05 | 0/10 | 8 |

All candidates pass the complete positive-lateral timing neighborhood and fail
the negative neighborhood. Native timing cannot be credited uniquely for the
pass-count improvement: the fresh-feedback arm ties. The varied-offset arm
keeps negative-direction travel below 0.30 m but misses the settling deadline.
All three arms tie under the original ranking. The first-listed arm was selected;
this additional tie-break was [recorded before held-out evaluation](results/failure_reduction_selection_20261019.json).

## Held-out results

Twenty-four conditions per policy: durations 1.5/2.5/3 s, delays 40/60 ms,
both lateral signs, and payload/held-empty fixtures.

| Policy | Loaded passes | Held-empty passes | Total passes | Falls |
| --- | ---: | ---: | ---: | ---: |
| timing_fixed | 9/12 | 9/12 | 18/24 | 0 |
| cycle-05 | 5/12 | 1/12 | 6/24 | 7 |

The candidate has six remaining failures from travel or settling deadlines.
It also regresses on the loaded, negative-lateral, 60 ms delay, 2.5 s case:
cycle-05 passes; the candidate travels 0.30236 m, exceeding the unchanged
0.30 m limit. No integrated candidate trial was run because qualification failed.

## Evidence and scoring

- [Development report](results/failure_reduction_development_20261019.json)
- [Held-out report](results/failure_reduction_heldout_20261019.json)

Reports retain every planned attempt, policy/training/source hashes, stopping
reasons, motion progress and timing-neighborhood results. Raw traces and logs
are in `ros2_ws/src/dynamic_obstacle_benchmarks/results/failure_reduction_{development,heldout}_20261019/`.
Both matrices use MuJoCo 3.3.6, sensor/sensor feedback, the integer physics-step
command clock and 15 s requested observation windows. Existing stopping,
contact, fall and attachment gates are unchanged.

The optional matrix flag `--require-motion-progress` additionally requires mean
body velocity projected along the requested planar command to reach at least
50% of requested speed, excluding the first 0.5 s. Invalid, missing, reordered
or nonfinite timestamp evidence fails closed. This prevents stationary policies
from qualifying merely by ignoring movement requests. Historical scoring remains
available without the flag.

These are single-seed, synchronous native probes with magnetic wrist attachment.
They do not establish physical finger-grasp, asynchronous ROS mission, or general
stopping reliability. Next work should target the remaining negative-lateral
braking failures and verify repeatability with independent training seeds, using
newly declared validation conditions now that this held-out set has been observed.

## Verification

Full workspace CI passed: 218 orchestration tests, 866 ROS test results,
native checks and the deterministic mock benchmark/MJLab helper check.
The focused scoring/provenance suite passed 57 tests. Documentation links,
generated graphics, repository hygiene, reviewed source patches and whitespace
checks passed.
