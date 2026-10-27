# Dynamic-obstacle benchmarks

<!-- geoff-nav:start -->
<p align="center">
  <a href="../../../README.md"><img src="../../../docs/media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../../../README.md#quick-start"><img src="../../../docs/media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="../../../docs/DEMO_GUIDE.md"><img src="../../../docs/media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="../../../docs/README.md"><img src="../../../docs/media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="../../../docs/REPOSITORY_GUIDE.md"><img src="../../../docs/media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="../../../docs/PROJECT_STATUS.md"><img src="../../../docs/media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="../../../docs/PROJECT_PLAN.md"><img src="../../../docs/media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="../../../docs/RESULTS_REVIEW.md"><img src="../../../docs/media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="../../../docs/literature_review/README.md"><img src="../../../docs/media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="../../../docs/DEVELOPMENT.md"><img src="../../../docs/media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

[Workspace map](../README.md) · [Experiment guide](../../../docs/AVOIDANCE_EXPERIMENTS.md) · [Results review](../../../docs/RESULTS_REVIEW.md)

This package runs navigation trials, validates input-source contracts and writes
scored results. Higher-level scripts coordinate suites and world/seed matrices.

## Package layout

| Path | Responsibility |
| --- | --- |
| [`config/`](config/) | Scenarios, trial budgets, goal settings and benchmark modes |
| [`dynamic_obstacle_benchmarks/`](dynamic_obstacle_benchmarks/) | Runner, configuration, source audits, metrics and result writers |
| [`launch/`](launch/) | ROS launch integration |
| [`test/`](test/) | Scoring, configuration, source-boundary and orchestration regressions |
| `results/` | Ignored local experiment artifacts; created by runs |

The installed command is `person_avoidance_benchmark`. For normal use, prefer the
suite/matrix wrappers: they preserve launch settings, policy identity and case
status together with the benchmark output.

## Run a trial or cohort

From the repository root after installation:

```bash
./scripts/run_demo_suite.sh --case onboard-waypoints --goals 3 --people 4 --seed 42 --headless
python3 scripts/run_avoidance_matrix.py --dry-run
```

The default matrix uses six worlds, seeds 42/43/44, four people, three goals and a
90-second trial budget. Read the [cohort runbook](../../../docs/AVOIDANCE_EXPERIMENTS.md#collect-a-cohort)
before collecting or publishing results.

## How to interpret a result

| Category | Meaning |
| --- | --- |
| Scored onboard trial | Sensor-driven navigation evaluated against configured acceptance gates |
| Oracle diagnostic | Selected truth/known-map inputs; useful for ablations, not onboard acceptance |
| Mock trial | Deterministic mechanics and scoring exercise, not physical locomotion |
| Visual session | Unscored display/goal-cycling session that retains contacts and failures |
| Invalid attempt | Missing or unusable evaluation; never silently converted to zero contacts |

A raw result and a strict-validator outcome are separate records. Full acceptance
requires the configured validity, mission, collision, stability and other checks.
Source-contract checks protect the onboard/evaluation boundary. Optional
synchronized crowd startup is a separate condition and must be reported as such.

Published JSON is in [`docs/results/`](../../../docs/results/README.md).
The [status page](../../../docs/PROJECT_STATUS.md) summarizes current evidence;
the [development guide](../../../docs/DEVELOPMENT.md) explains package verification.
