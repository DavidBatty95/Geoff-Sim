# Experiments, datasets and evidence links

<!-- geoff-nav:start -->
<p align="center">
  <a href="../../README.md"><img src="../media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../../README.md#quick-start"><img src="../media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="../DEMO_GUIDE.md"><img src="../media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="../README.md"><img src="../media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="../REPOSITORY_GUIDE.md"><img src="../media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="../PROJECT_STATUS.md"><img src="../media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="../PROJECT_PLAN.md"><img src="../media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="../RESULTS_REVIEW.md"><img src="../media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="../literature_review/README.md"><img src="../media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="../DEVELOPMENT.md"><img src="../media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

[← Resource library](README.md) · Related: [Research evaluation references](RESEARCH.md)

[![Results dashboard with filtering and trajectory replay — open the results-review guide](../media/results-dashboard.png)](../RESULTS_REVIEW.md)

## Read and interpret a run

| Resource | Kind | Why open it? |
| --- | --- | --- |
| [Results-review guide](../RESULTS_REVIEW.md) | Guide | Metrics, provenance, invalid attempts and fair cohort comparisons |
| [Published dataset catalogue](../results/README.md) | Evidence index | Entry points into the portable records |
| [Published demo evidence](../DEMO_RESULTS.md) | Evidence | v12 and separately labelled historical demo modes |
| [Avoidance methodology](../AVOIDANCE_EXPERIMENTS.md) | Guide | World versions, trial conditions, matrix collection and dashboard behavior |
| [General experiment batches](../BENCHMARK_EXPERIMENTS.md) | Guide | Frozen method/scenario/seed/repetition batches |
| [Current evidence snapshot](../PROJECT_STATUS.md#current-evidence) | Evidence summary | Compare assessed snapshots without treating them as one configuration |
| [Reliability ledger](../NAVIGATION_RELIABILITY_INVESTIGATION.md) | Evidence | Failed tuning pilots, complete matrices and retained regressions |
| [Artifact locations](../REPOSITORY_GUIDE.md#where-results-go) | Guide | Find raw logs, manifests and training output on the originating machine |

## Portable records worth bookmarking

| Dataset | Role |
| --- | --- |
| [Published avoidance dataset](../results/avoidance_dataset.json) | Default dashboard publication snapshot |
| [Earlier demo results](../results/demo_results.json) | Visual, mock, oracle and locomotion records with their mode labels |
| [Simple-world baseline](../results/avoidance_baseline_v1.json) | Earlier world-version reference |
| [v82 consolidated matrix](../results/avoidance_v82_matrix.json) | Investigation snapshot with replay samples |
| [v84 full matrix](../results/avoidance_v84_matrix.json) | Later retained complete matrix |
| [v85 alignment audit](../results/avoidance_v85_alignment_audit.json) | Targeted evaluation diagnostic |
| [Payload cycle 01](../results/payload_training_cycle_01.json) | Training evaluations and native candidate attempts |
| [Visual goal handoffs](../results/visual_goal_handoffs.json) | Unscored arrival/goal-cycling observations |
| [Turn-response record](../results/turn_response.json) | Isolated response measurements |

The [dataset directory](../results/) contains additional retained pilots and
validation records. Follow the ledgers for their interpretation. Raw trial logs
and checkpoints are not automatically present in a clean checkout.

## Run, inspect and validate

| Tool | Kind | Purpose |
| --- | --- | --- |
| [Benchmark package](../../ros2_ws/src/dynamic_obstacle_benchmarks/README.md) | Code / guide | Scoring and input-source contracts |
| [Matrix runner](../../scripts/run_avoidance_matrix.py) | Code | Serial world/seed collection and portable publication |
| [Dataset analysis](../../scripts/analyze_avoidance_dataset.py) | Code | Summarize retained cohorts |
| [Results server](../../scripts/avoidance_results_gui.py) | Code | Local Python dashboard and CSV export |
| [Dashboard frontend](../../tools/README.md) | Code / guide | Browser filtering, per-run details and qualitative replay |
| [Benchmark validator](../../scripts/validate_benchmark_result.py) | Code | Strict acceptance checks |
| [Payload summary](../../scripts/summarize_payload_results.py) | Code | Specialized payload experiment interpretation |

Launch existing-result inspection with `./geoff results`. A strict pass, valid
record, partial goal completion and unscored visual session mean different things.
The [review guide](../RESULTS_REVIEW.md) is the reference for reporting those distinctions.

[Back to the library](README.md) · [How to contribute evidence](DEVELOPMENT.md)
