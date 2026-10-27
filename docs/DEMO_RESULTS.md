# Published demo evidence

<!-- geoff-nav:start -->
<p align="center">
  <a href="../README.md"><img src="media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../README.md#quick-start"><img src="media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="DEMO_GUIDE.md"><img src="media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="README.md"><img src="media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="REPOSITORY_GUIDE.md"><img src="media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="PROJECT_STATUS.md"><img src="media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="PROJECT_PLAN.md"><img src="media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="RESULTS_REVIEW.md"><img src="media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="literature_review/README.md"><img src="media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="DEVELOPMENT.md"><img src="media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

The portable source-linked records are in [demo_results.json](results/demo_results.json).
The newer multi-world cohort is documented in [avoidance experiments](AVOIDANCE_EXPERIMENTS.md)
and published separately as [avoidance_dataset.json](results/avoidance_dataset.json).

For a guided inspection workflow, see [Reviewing results and evidence](RESULTS_REVIEW.md).
Later tuning matrices and diagnostics are retained in the
[navigation reliability investigation](NAVIGATION_RELIABILITY_INVESTIGATION.md).
The cohort below describes the published v12 snapshot, not the latest working controller.

## Published representative-world cohort (v12)

Source commit: `7c4cd3e`; controller label `calibrated-actions-v12`.
Six representative worlds × seeds 42/43/44, four people, three requested goals,
90 s per trial. All 18 trials produced valid records. **4/18 passed the strict
gate; 19/54 goals were reached and 12 contacts were recorded. Avoidance is still
not reliable across these conditions.**

| Environment | Strict passes | Goals reached | Contacts |
| --- | --- | --- | --- |
| Residential courtyard | 0/3 | 1/9 | 3 |
| Café courtyard | 0/3 | 2/9 | 2 |
| Station entrance | 0/3 | 2/9 | 2 |
| Clinic | 1/3 | 3/9 | 2 |
| Station concourse | 2/3 | 8/9 | 1 |
| Office lobby | 1/3 | 3/9 | 2 |

The simple-v1 baseline had 1/18 strict passes, 14/54 goals and 16 contacts. The
representative-v8 and pivot-v10 cohorts are also retained. Geometry/appearance,
controller settings and (between earlier cohorts) container dependencies changed;
these descriptive counts are not an isolated or statistically proven speedup.
The cohort manifest records source hashes, commands and Docker image ID.

The separate 90 s one-person clinic visual session reached four goals with zero
contacts/near misses, handing off within 0.5 m. It establishes goal cycling under
that condition, not scored crowd reliability. See [handoff evidence](results/visual_goal_handoffs.json).

## Earlier demo evidence

| Earlier run | Observed outcome | Interpretation |
| --- | --- | --- |
| Natural eight-person visual, seed 44 | 75 s; 2 goals; 3 contacts; 4 near misses | Unscored visualization, not a pass |
| Natural eight-person scored, seed 44 | 0/3 goals; 1 contact; 2 near misses; 34.637 s | Failed onboard trial |
| Mock quick, seed 1 | 2/3 goals; 1 contact | Mechanics diagnostic; acceptance failed |
| Historical oracle seeds 42/43/44 | 8/8 goals each, zero recorded contacts/near misses | Oracle diagnostic, not onboard acceptance |

The JSON additionally includes a fixed-forward response run and its measurements.
Source artifact paths and SHA-256 hashes establish which local run supplied each
record. Raw logs are intentionally outside maintained documentation.

The recorded eight-person GUI run had local-planner p95 58.921 ms (median
46.164 ms, maximum 70.073 ms), with perception-stale stops. Earlier four-person
timings must not be substituted for this configuration. Goal handoffs were
logged at 0.494 m and 0.438 m using prepared next goals.

Use `python3 scripts/avoidance_results_gui.py --dataset docs/results/demo_results.json`
to inspect this evidence separately from the new multi-world experiment.
