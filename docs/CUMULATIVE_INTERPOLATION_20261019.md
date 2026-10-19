# Cumulative stopping and actor interpolation — 19 October 2026

The strongest zero-fall interpolation currently passes **195/208 retained native
conditions**, versus **188/208** for broad-reference seed80. Thirteen cases still
fail. This is experimental policy selection, not flawless behavior or deployment
qualification. Every failed timing remains in the cumulative regression set.

## Cumulative inventory

The [initial cumulative plan](results/cumulative_stopping_plan_20261019.json)
crosses 20 previously observed durations with two lateral signs, two loads and
40/60 ms delay: 160 conditions per policy. The runner reused 352 already recorded
same-policy-hash/same-protocol results and executed 288 missing probes. Reused
records retain report/case provenance and are not additional independent trials.

| Policy | Passes /160 | Falls |
| --- | ---: | ---: |
| braking79 parent | 141 | 0 |
| Broad reference80 | 145 | 0 |
| Narrow reference80 | 143 | 0 |
| Unconstrained81 | 149 | 1 |

Relative to braking79, broad80 fixes eight cases and regresses on four; narrow80
fixes ten and regresses on eight; unconstrained81 fixes17 and regresses on nine.
The earlier four-fix/no-regression broad80 result therefore applies only to its
58-condition comparison, not this expanded inventory.

## Interpolation experiment

The [first interpolation plan](results/interpolation_plan_20261019.json) combines
actor MLP weights from broad80 (left) and unconstrained81 (right). The actor
architecture, 480 inputs and29 outputs stay identical. This is fixed parameter
interpolation, not runtime switching or an additional trained policy. Synthesized
actor-only checkpoints are explicitly marked non-resumable training artifacts.
Exports include endpoint hashes and Torch/ONNX parity errors below5e-7.

| Right-endpoint weight | Cumulative passes /160 | Falls | Fresh passes /24 | Fresh falls |
| --- | ---: | ---: | ---: | ---: |
| 0.25 | 146 | 0 | 22 | 0 |
| 0.50 | 143 | 2 | 24 | 0 |
| 0.75 | 146 | 0 | 20 | 1 |
| Broad80 endpoint | 145 | 0 | 21 | 0 |
| Unconstrained81 endpoint | 149 | 1 | 22 | 1 |

Fresh durations were1.3/2.1/3.4 s. The perfect fresh result at weight0.50 cannot
override its cumulative falls. All three coefficients and both endpoints were
included; 480 new cumulative probes and120 fresh probes completed.

## Refinement, retaining prior failures

The [refinement plan](results/interpolation_refinement_plan_20261019.json) adds
the first fresh timings to the regression inventory (184 conditions). It tests
0.85/0.90/0.95 with fresh durations1.2/2.16/3.6 s, again covering both loads,
signs and delays. There are552 new cumulative probes and120 fresh probes.

| Right-endpoint weight | Cumulative passes /184 | Falls | Fresh passes /24 | Fresh falls |
| --- | ---: | ---: | ---: | ---: |
| 0.85 | 168 | 2 | 24 | 0 |
| 0.90 | 167 | 2 | 24 | 0 |
| **0.95** | **171** | **0** | **24** | **0** |
| Broad80 endpoint | 166 | 0 | 22 | 0 |
| Unconstrained81 endpoint | 171 | 2 | 24 | 0 |

The0.95 candidate has17 fixes and10 regressions against broad80 over208 cases,
for a net gain of seven. It eliminates the unconstrained endpoint's falls on
this set but exchanges individual successes/failures. No promotion follows.
The all-condition gate and every contact/fall/progress/travel/deadline threshold
remain unchanged. Protocol: MuJoCo3.3.6, sensor/sensor feedback, physics-step
command clock and15 s requested observation.

## Continuing search

The [layer-search plan](results/native_layer_search_plan_20261019.json) starts
from four0.95 coefficients and explores one coefficient per actor linear layer.
`iterate_native_layers.py` runs serially and records the complete intended suite
before evaluating any candidate. Incumbent failures run first. A fall or a
failure lower bound that cannot beat the incumbent triggers early rejection;
these reports remain explicitly incomplete, never qualified.

Only complete zero-fall candidates with strictly fewer failures can replace the
incumbent. A zero-failure cumulative candidate must pass48 newly declared
validation cases. Failed validation is retained in the cumulative suite;
validation falls require review. This is a native-suite search only: runtime
deployment and integrated missions are not changed automatically.

The worker supports explicit interruption, detects changed source files and
stops on setup errors or less than10 GiB free disk. `status.json` records its
current state, incumbent, full suite and accepted improvements. The bounded
one-proposal smoke run correctly rejected a candidate after a fall in its third
case, retaining the208-case plan as incomplete.

The continuing worker was launched as `geoff-native-layer-search`. Inspect it
with `docker logs --tail 20 geoff-native-layer-search`; interrupt it with
`docker stop geoff-native-layer-search`. Its live status is stored at
`ros2_ws/src/dynamic_obstacle_benchmarks/results/layer_search_continuous_20261019/status.json`.
This publication does not claim that the continuing search has qualified a policy.

Verification: 66 focused scoring/provenance/cumulative/layer-screen tests passed;
the real one-proposal smoke exercised export, simulation and rejection. Full
workspace CI passed at the preceding snapshot (218 orchestration tests,866 ROS
results and native checks); the added scripts do not alter runtime control.
The two updated README GIFs passed independent frame-count, duration, result
and hash checks and visual inspection of initial, moving, braking and final frames.

## Evidence

- [Cumulative inventory](results/cumulative_stopping_20261019.json) and [paired changes](results/cumulative_paired_changes_20261019.json)
- [Interpolation cumulative](results/interpolation_cumulative_20261019.json), [fresh](results/interpolation_validation_20261019.json), [paired changes](results/interpolation_paired_changes_20261019.json)
- [Refinement cumulative](results/interpolation_refinement_cumulative_20261019.json), [fresh](results/interpolation_refinement_validation_20261019.json), [paired changes](results/interpolation_refinement_paired_changes_20261019.json)

Policies are retained under `vendor/mjlab/logs/payload_training/interpolation_<weight>_20261019/`.
The0.95 policy SHA-256 is `f1e34f3673f48e9eabe2c43d20275ccb3d4ab8589e9ec60de465187da268506f`.
Raw runs use matching experiment names under
`ros2_ws/src/dynamic_obstacle_benchmarks/results/`. These synchronous magnetic
wrist fixtures do not establish physical-grasp or live ROS mission reliability.
