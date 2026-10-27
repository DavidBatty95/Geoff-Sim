# Results-dashboard frontend

<!-- geoff-nav:start -->
<p align="center">
  <a href="../README.md"><img src="../docs/media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../README.md#quick-start"><img src="../docs/media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="../docs/DEMO_GUIDE.md"><img src="../docs/media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="../docs/README.md"><img src="../docs/media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="../docs/REPOSITORY_GUIDE.md"><img src="../docs/media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="../docs/PROJECT_STATUS.md"><img src="../docs/media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="../docs/PROJECT_PLAN.md"><img src="../docs/media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="../docs/RESULTS_REVIEW.md"><img src="../docs/media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="../docs/literature_review/README.md"><img src="../docs/media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="../docs/DEVELOPMENT.md"><img src="../docs/media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

Related: [Published dataset catalogue](../docs/results/README.md).

[`avoidance_dashboard.html`](avoidance_dashboard.html) is the local browser UI
served by `scripts/avoidance_results_gui.py`. It provides dataset selection,
filtering, per-run detail, charts, CSV export and recorded-trajectory replay where
the dataset supplies it.

[`requirements-quality.txt`](requirements-quality.txt) contains the separate
host/CI documentation and repository-check dependencies. They are not part of
the robot runtime; use the isolated setup in [Development](../docs/DEVELOPMENT.md).

## Launch

From the repository root:

```bash
./geoff results
# Open http://127.0.0.1:8765
```

Use the Python server rather than opening the HTML as a standalone file: the UI
loads normalized data from its local endpoints. Inspecting published results
requires Python 3 and a browser, not ROS or a running simulator.

## Development and interpretation

Keep missing values distinct from zero and retain invalid attempts in the UI.
Replay samples are asynchronous/downsampled visual aids; scored contact metrics
come from the evaluator. The [results guide](../docs/RESULTS_REVIEW.md) explains
cohort comparisons and strict-validator semantics.

Dashboard regression tests live in `scripts/test_avoidance_dashboard.py`.
The optional Firefox/geckodriver check is described in
[development](../docs/DEVELOPMENT.md). When adding metrics, update both the
normalization layer and UI labels, then verify existing datasets remain readable.
