# Broader rehearsal coverage — 19 October 2026

Broad-reference seed80 fixes four parent failures without introducing a
parent-relative regression across this round's 58 conditions. It passes **51/58**
versus the parent's **47/58**, with zero falls for both. Seed81 regresses,
and no candidate qualifies for promotion. The expanded corpus is not a
consistent improvement over narrow rehearsal on fresh validation.

## Experiment

The [predeclared plan](results/broad_reference_plan_20261019.json) selects every
passing parent condition from the preceding development, historical and fresh
reports. All **51/51** source probes were re-captured and passed again before
corpus construction. The corpus contains 25,500 actor observation/action pairs
covering both lateral signs, loaded/held-empty arms, 40/60 ms delay and multiple
movement durations. These are observed successes, not unseen validation data.

Two new 100-iteration runs use seeds80/81, 256 environments and reference-loss
weight10, starting from braking79. Training options match the retained narrow
reference and unconstrained runs. Only corpus coverage changes. Both new runs
complete 614,400 environment steps. Retained controls are re-evaluated on exactly
the same conditions, rather than retrained unnecessarily.

## Full comparison

| Policy | Development /10 | Historical /24 | Fresh /24 | Total /58 | Falls |
| --- | ---: | ---: | ---: | ---: | ---: |
| Broad reference80 | 9 | 22 | 20 | 51 | 0 |
| Narrow reference80 | 9 | 20 | 23 | 52 | 0 |
| Unconstrained80 | 5 | 21 | 21 | 47 | 0 |
| Broad reference81 | 8 | 16 | 22 | 46 | 0 |
| Narrow reference81 | 6 | 18 | 24 | 48 | 3 |
| Unconstrained81 | 10 | 22 | 24 | 56 | 0 |
| Parent braking79 | 9 | 20 | 18 | 47 | 0 |

Development uses loaded, 60 ms delayed signed stops at
1.96/1.98/2/2.02/2.04 s. Historical cases use durations1.5/2.5/3 s,
both loads/signs and delays40/60 ms. Fresh cases use **1.4/2.6/3.2 s** with the
same loads/signs/delays. Fresh timings differ from prior rounds; totals cannot
be compared across rounds as though they were identical conditions.

All 406 evaluation probes completed without process/protocol errors. They use
MuJoCo3.3.6, sensor/sensor feedback, integer physics-step command timing and
15 s requested observation. Movement progress and all strict stopping gates
remain unchanged. Successful development/historical states are training-exposed
in the broad corpus; fresh conditions are reported separately.

## Paired fixes and regressions

`summarize_native_pairs.py` matches load, delay, lateral command and duration,
rejects incomplete/unmatched/duplicate evidence, and retains individual changed
cases with stopping/progress reasons and process errors. Equal pass totals
cannot conceal an exchanged success and failure.

| Candidate versus comparator | Development fixes/regressions | Historical fixes/regressions | Fresh fixes/regressions |
| --- | ---: | ---: | ---: |
| Broad80 vs parent | 0 / 0 | 2 / 0 | 2 / 0 |
| Broad81 vs parent | 0 / 1 | 1 / 5 | 5 / 1 |
| Broad80 vs narrow80 | 0 / 0 | 3 / 1 | 0 / 3 |
| Broad81 vs narrow81 | 2 / 0 | 5 / 7 | 0 / 2 |
| Broad80 vs unconstrained80 | 4 / 0 | 3 / 2 | 2 / 3 |
| Broad81 vs unconstrained81 | 0 / 2 | 0 / 6 | 0 / 2 |

The expanded corpus improves known-case preservation for seed80, but introduces
fresh regressions relative to narrow rehearsal in both seeds. Seed81 removes
the narrow candidate's falls here while losing passes. Neither broad candidate
is promoted. Unconstrained81's 56/58 here does not erase its earlier fresh-set
fall: it remains an experimental policy with known failures.

Next iterations should address the full retained failure set, not replace it
with each new fresh set. Assess cumulative fixes/regressions before choosing a
new parent. Replay sampling/phase balance and reference-loss weight remain
unisolated influences; broader coverage alone has not solved stopping.

## Evidence and media

- [Development](results/broad_development_20261019.json)
- [Historical regression](results/broad_regression_20261019.json)
- [Fresh validation](results/broad_validation_20261019.json)
- [Every paired change](results/broad_paired_changes_20261019.json)
- [Complete-run GIFs and provenance](media/braking/README.md)

Raw collection/corpus: `ros2_ws/src/dynamic_obstacle_benchmarks/results/broad_reference_sources_20261019/`.
Raw evaluations: `ros2_ws/src/dynamic_obstacle_benchmarks/results/broad_{development,regression,validation}_20261019/`.
Training: `vendor/mjlab/logs/payload_training/broad_reference_seed{80,81}_20261019/`.

Two additional recorded loaded probes from broad80 pass movement and stopping,
with complete frames from0 to15 s. They illustrate complete test runs, not
general reliability, physical finger-grasp or asynchronous ROS mission acceptance.
