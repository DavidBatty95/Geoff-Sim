# Braking curriculum follow-up — 19 October 2026

Targeted continued training reduced failures further for seed 79: **6/58**
across the development, fresh-validation and historical-regression conditions,
versus **16/58** for the previous `timing_fixed` candidate. Neither fell in these
58 probes. This aggregate is descriptive, not an independent success-rate
estimate: development cases were already known, and timings are correlated.
No candidate meets the unchanged all-conditions promotion gate.

## Experiment

The [predeclared plan](results/braking_followup_plan_20261019.json) compares
400 additional iterations on 256 environments for seeds 78 and 79, starting
from `timing_fixed_seed77_20261019/final.pt`. The targeted arm combines the
existing `stopping` objective (zero-command velocity penalty) with the
`transitions` command curriculum (more signed lateral move-stop exposure).
Matched controls continue the reactive objective and mixed command curriculum.
Native sensor timing, Euler integration, native friction, 0.9 target limits,
published action feedback, 0–60 ms delay, mixed loads and fixed 0.12 m load
offset are common. Each run completed 2,457,600 environment steps.

All four policies and the parent were evaluated on both planned sets, without
discarding either seed. Independent continuation seeds share the same trained
parent; this is not independent training from scratch. The bundled intervention
does not isolate reward effects from curriculum effects.

## Complete results

Entries show **passes / cases (falls)**. Every pass includes motion progress.

| Policy | Development | Fresh validation | Historical regression |
| --- | ---: | ---: | ---: |
| braking78 | 2/10 (4) | 24/24 (0) | 18/24 (4) |
| braking79 | 9/10 (0) | 23/24 (0) | 20/24 (0) |
| control78 | 6/10 (0) | 21/24 (0) | Not run |
| control79 | 0/10 (0) | 18/24 (2) | Not run |
| timing_fixed parent | 5/10 (0) | 19/24 (0) | 18/24 (0) |

Development uses loaded, 60 ms delayed signed lateral stops at
1.96/1.98/2/2.02/2.04 s. Fresh validation uses durations 1.7/2.3/2.8 s,
40/60 ms delay and loaded/held-empty fixtures. Historical regression uses
1.5/2.5/3 s and the same delay/load combinations. The regression batch was
added after observing validation, announced before execution, and is not a new
held-out set. Both targeted seeds and the parent were included.

All 242 probes completed without process/protocol errors. They use native
MuJoCo 3.3.6, sensor/sensor feedback, integer physics-step command timing,
15 s observation and the unchanged stopping/contact/attachment/fall gates.

## Interpretation and remaining failures

- Seed 79 improves each set relative to the parent: failures 5→1, 5→1 and 6→4.
  Its development negative-lateral passes improve from 0/5 to 4/5.
- Both targeted seeds beat their matched controls on fresh validation, but
  seed 78 introduces severe timing-local regressions. Its perfect fresh set
  cannot override eight falls across development and historical regression.
- Seed 79's development failure travels 0.30417 m, exceeding 0.30 m.
  Historical failures include loaded negative stops at 40 ms/3 s and
  60 ms/1.5 s, plus held-empty negative stops at 60 ms/1.5 s and 60 ms/3 s.
  Three of these historical cases passed with the parent, so the net reduction
  includes regressions rather than being a strict improvement everywhere.
- Neither policy is promoted, and no integrated mission was launched.
  Magnetic wrist attachment and synchronous probes do not establish physical
  finger-grasp or asynchronous ROS mission reliability.

The next useful intervention is to preserve the parent's successful behavior
while increasing braking exposure: isolate reward-only versus curriculum-only
changes, use shorter or smaller updates, and explicitly include dense timing
neighborhoods around both old and newly observed failures. Declare fresh
validation before further tuning; the sets above are now observed.

## Artifacts and reproduction

- [Development report](results/braking_development_20261019.json)
- [Fresh-validation report](results/braking_validation_20261019.json)
- [Historical-regression report](results/braking_regression_20261019.json)

Reports retain policies, training manifests, source hashes, full commands,
individual stopping/progress verdicts and timing neighborhoods. Training
artifacts are in `vendor/mjlab/logs/payload_training/{braking,control}_seed{78,79}_20261019/`.
Raw probes are in `ros2_ws/src/dynamic_obstacle_benchmarks/results/braking_{development,validation,regression}_20261019/`.
This increment uses existing training/scoring implementations without changing
qualification thresholds or runtime control code.
