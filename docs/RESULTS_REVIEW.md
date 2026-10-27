# Reviewing results and evidence

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
  <a href="RESULTS_REVIEW.md"><img src="media/navigation/results-current.svg" width="132" height="44" alt="Results (current page)"></a>
  <a href="literature_review/README.md"><img src="media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="DEVELOPMENT.md"><img src="media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

Related: [Published evidence](DEMO_RESULTS.md) · [Experiment methodology](AVOIDANCE_EXPERIMENTS.md).

Geoff-Sim retains successful runs, failed trials and invalid attempts. This guide
explains how to review them without confusing a visual demonstration, a diagnostic
test and an accepted navigation result.

## Evidence map

| Record | Purpose | Interpretation boundary |
| --- | --- | --- |
| [Published avoidance dataset](results/avoidance_dataset.json) | Portable representative-world cohort used by the default dashboard | A specific recorded configuration, not a live summary of the current code |
| [Published demo results](results/demo_results.json) | Earlier visual, mock, locomotion and oracle evidence | Read each run's mode before comparing outcomes |
| [Reliability investigation](NAVIGATION_RELIABILITY_INVESTIGATION.md) | Iteration ledger with linked pilots, matrices and diagnostics | Targeted subsets and full matrices have different denominators |
| [Visual goal handoffs](results/visual_goal_handoffs.json) | Arrival distances and continuous goal cycling | Unscored display/mission integration evidence |
| [Payload candidate evidence](results/payload_training_cycle_01.json) | Training evaluation and native transport attempts | Isolated fixture evidence with its own mission criteria |
| [Literature review](literature_review/README.md) | Critical research synthesis and proposed experiments | Bibliographic verification and paper analysis are distinct from experimental replication |

The published v12 cohort recorded **4/18 strict passes, 19/54 goals and 12
contacts**. The investigation's consolidated v82 configuration recorded **14/18
strict passes, 49/54 goals and four contacts**. These are different snapshots;
neither establishes collision-free reliability. Consult the complete ledger for
later targeted diagnostics rather than treating the highest file version as a
new accepted release.

## 1. Inspect a dataset in the browser

Only Python 3 is required to inspect published data:

```bash
python3 scripts/avoidance_results_gui.py
# Visit http://127.0.0.1:8765
```

1. Select the dataset and read its cohort/configuration context.
2. Filter by run type or search for a world, seed or failure.
3. Select a row, or focus it and press **Enter**, to inspect metrics and source
   paths/hashes.
4. Inspect invalid attempts as well as successful records. Missing values are
   missing, not zero.
5. Where available, use trajectory replay to understand the interaction, then
   consult the scored metrics for the actual outcome.

The replay uses downsampled, asynchronous evaluation samples. It is a qualitative
view; contact counts come from the scorer, not the browser drawing. The chart
shows goal progress within world/people/source groups, not a pooled safety rate.
Refresh rereads the results.

Open the earlier demos separately, or export the default dataset:

```bash
python3 scripts/avoidance_results_gui.py --dataset docs/results/demo_results.json --port 8766
python3 scripts/avoidance_results_gui.py --export docs/results/avoidance_dataset.csv
```

## 2. Establish what was actually run

Before interpreting a metric, identify:

- **Mode:** onboard, oracle, mock, visual or transport.
- **Conditions:** world ID/version, seed, people, requested goals and time budget.
- **Implementation:** Git revision and dirty state, source/patch/config hashes,
  policy hash and container identity where recorded.
- **Execution:** exact command, return code, timeout, data validity and validator
  outcome.
- **Provenance:** source artifact path/hash and whether raw logs are available.

A Git revision alone does not reconstruct a dirty working tree. Keep the recorded
hashes and configuration with the result. Portable records may refer to raw
artifacts on the original machine; a source hash identifies a file but does not
make that file available in a clean clone.

## 3. Interpret the metrics together

| Signal | Review question |
| --- | --- |
| Strict pass | Did the configured validator pass, including validity, goals, contacts, stability and other gates? |
| Goals reached/requested | Was the mission completed, or did the robot only make partial progress? |
| Contacts and near misses | What body-clearance definition and actor conditions did the scorer use? |
| Clearance | Was the minimum gap meaningful with valid, correctly aligned evaluation data? |
| Duration | Is this the benchmark interval, session budget or wall time including startup? |
| Fall or stop reason | Did the episode fail physically, time out, lose data or finish normally? |
| Timing and freshness | Were tracking and planning observations timely under this exact workload? |
| Invalid attempt | Was the result missing or unusable, rather than a measured zero-contact run? |

The dashboard treats a raw benchmark success with a nonzero strict-validator
return code as a failure. A visual session cannot become a benchmark pass merely
because it reached goals. A mock pass establishes benchmark mechanics; an oracle
pass does not validate the onboard perception pipeline.

## 4. Compare cohorts fairly

Compare matched world versions, people counts, goal counts, budgets, policy and
input-source modes. Document changed factors explicitly. Appearance changes can
also change occlusion and sensing. Observed-free goals and asynchronous startup
can differ even for the same seed, so repeated seeds do not guarantee identical
executed trajectories.

Report attempted trials, valid trials, strict passes, goals and contacts together.
Keep invalid attempts in the report rather than dropping them from the cohort.
Do not combine a small targeted pilot with an 18-trial matrix or choose only the
best tuning iteration to claim generalization. Held-out seeds and repeated trials
are needed to assess reliability beyond the development conditions.

## 5. Write a useful review

Use this structure for an experiment summary, issue or pull request:

```text
Question / hypothesis:
Dataset path and source revision:
Mode, world version, policy and changed configuration:
Seeds, people, goals and trial budget:
Attempted / valid / strict-pass trials:
Goals reached / requested, contacts, falls and invalid reasons:
Representative failure and source artifact:
Comparison cohort and differing conditions:
Conclusion supported by the measurements:
Remaining uncertainty and next experiment:
```

For implementation reviews, pair the evidence with the
[development checks](DEVELOPMENT.md) and [source-boundary reference](ONBOARD_SOCIAL_NAVIGATION.md).
Passing unit tests supports implementation correctness within their coverage;
live acceptance still needs scored runs.

## Research review and further reading

The [literature-review package](literature_review/README.md) includes a checked
PDF, editable LaTeX, verified bibliography, search log, reading audit and claim
audit. Its 50 works cover classical avoidance, learned crowd navigation,
perception/prediction, humanoid control and evaluation. The central recommendation
is observation-age-aware, response-calibrated navigation with controlled
oracle-to-onboard experiments.

Read the synthesis for the research argument, the reading audit for the depth of
source inspection, and the project datasets for measured implementation outcomes.
The review's project snapshot has its own provenance; later engineering results
should be cited from their corresponding experiment records.
