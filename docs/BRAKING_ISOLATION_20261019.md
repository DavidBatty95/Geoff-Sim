# Smaller-update braking factor isolation — 19 October 2026

Six 100-iteration continuation runs and **464 native probes** tested stopping
reward alone, transition curriculum alone, and matched controls across seeds
78/79. Neither isolated intervention consistently improves its matched control
across both seeds. The retained `braking79` remains the leading experimental
candidate: it ties the highest total pass count without the tied control's fall.
No policy qualifies for promotion.

## Declared comparison

The [plan](results/braking_isolation_plan_20261019.json) was written before
training. Every run starts from `timing_fixed_seed77_20261019/final.pt`, uses
256 environments, and performs 100 iterations (614,400 environment steps).
The learning rate remains the existing 1e-5; this reduces the number of updates,
not their learning rate. All arms retain native sensor timing, Euler integration,
native friction, 0.9 target limits, published action feedback, mixed loads,
0–60 ms delay, 0.12 m load offset and 0.05 smoothness weight.

- `reward100`: stopping objective, mixed commands.
- `curriculum100`: reactive objective, transition commands.
- `control100`: reactive objective, mixed commands.

The six new policies, parent and retained 400-iteration combined-intervention
`braking79` were tested on all three matrices. No arm was omitted after seeing
development outcomes. Seeds independently resample continuation but share a
trained parent; these are not independent from-scratch replications.

## Results

All entries are combined movement-and-stop passes. Falls are summed over the
three sets and are already failures, not additional cases.

| Policy | Development /10 | Historical /24 | Fresh /24 | Total /58 | Falls |
| --- | ---: | ---: | ---: | ---: | ---: |
| reward100 seed78 | 5 | 17 | 22 | 44 | 0 |
| reward100 seed79 | 5 | 21 | 17 | 43 | 0 |
| curriculum100 seed78 | 6 | 19 | 20 | 45 | 0 |
| curriculum100 seed79 | 5 | 15 | 15 | 35 | 0 |
| control100 seed78 | 5 | 12 | 20 | 37 | 1 |
| control100 seed79 | 8 | 19 | 21 | 48 | 1 |
| timing_fixed parent | 5 | 18 | 15 | 38 | 0 |
| retained braking79 | 9 | 20 | 19 | 48 | 0 |

Development uses loaded signed lateral stops at 60 ms delay and durations
1.96/1.98/2/2.02/2.04 s. Historical regression uses 1.5/2.5/3 s, both signs,
40/60 ms delays and loaded/held-empty fixtures. Fresh validation uses the same
load/delay/sign combinations at previously undeclared durations 1.8/2.2/2.7 s.
All probes use MuJoCo 3.3.6, sensor/sensor feedback, integer physics-step command
timing, 15 s observation and the unchanged qualification gates. All 464 attempts
completed without process/protocol errors.

The fresh set differs from the preceding follow-up's 1.7/2.3/2.8 s set. Thus
the retained braking79's **10/58 failures here** must not be confused with its
**6/58 failures in the preceding report**. The identical known conditions
reproduce the earlier pass counts; new timings reveal more failures.

## What this establishes

- Reward-only seed78 reduces fresh failures from the parent's 9 to 2, but its
  historical failures increase from 6 to 7. It is not an improvement everywhere.
- Relative to same-seed controls, reward-only gains 7 total passes for seed78
  but loses 5 for seed79. Curriculum-only gains 8 for seed78 but loses 13 for
  seed79. Two seeds do not support a consistent factor effect.
- Smaller budgets alone do not ensure preservation: both controls have a fall.
- No new policy exceeds the retained braking79 in total passes. Choosing the
  best fresh-only score would hide known regressions. No integrated trial or
  deployment change follows from this experiment.

Next work should test an explicit preservation constraint (for example, an
actor-reference penalty on previously successful states) against an otherwise
matched continuation, rather than assuming fewer updates preserve behavior.
The reference data must cover both successful movement and stopping so that
preserving stillness cannot replace tracking. Use new validation conditions;
these sets are now observed. Magnetic wrist fixtures and synchronous probes
remain distinct from physical grasp and live ROS mission qualification.

## Evidence

- [Development](results/isolation_development_20261019.json)
- [Historical regression](results/isolation_regression_20261019.json)
- [Fresh validation](results/isolation_validation_20261019.json)

Reports retain training manifests, policy/source hashes, commands, all stopping
reasons, motion progress and timing-neighborhood results. Training artifacts are
in `vendor/mjlab/logs/payload_training/{reward100,curriculum100,control100}_seed{78,79}_20261019/`.
Raw probes are in `ros2_ws/src/dynamic_obstacle_benchmarks/results/isolation_{development,regression,validation}_20261019/`.
No training implementation or runtime controller was modified in this increment.
