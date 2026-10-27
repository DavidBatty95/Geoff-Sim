# Team handoff

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

[Current status](PROJECT_STATUS.md) · [Forward plan](PROJECT_PLAN.md) · [Repository guide](REPOSITORY_GUIDE.md)

Reviewed 5 October 2026 against published development baseline `3a7ceac`.

**6 October follow-up:** start with [hardware fidelity](HARDWARE_FIDELITY.md) and
[the native payload ledger](NATIVE_PAYLOAD_CONTRACT.md). Full CI and two new
unchanged-code dual-delivery trials passed; the robot-facing contract now includes
CRC/machine-mode handling, measured sensor IMUs/TF and portable point-cloud layouts.
No physical hardware is available; documented calibration and crowd-acceptance
gaps remain part of the handoff.

## First session

1. Read the [README](../README.md), [folder guide](REPOSITORY_GUIDE.md), [architecture](ARCHITECTURE.md), and
   [dataset methodology](AVOIDANCE_EXPERIMENTS.md).
2. Install on a Linux graphics host, verify fork access and apply the pinned
   [source patches](../patches/README.md).
3. Run fast checks, then one short visual demonstration and one scored trial.
4. Inspect JSON/CSV in the results GUI, including invalid and failed trials.

## Current engineering priorities

- Reliable sensor-only avoidance: existing natural-crowd trials include contacts.
- Latest retained full matrix v84: 10/18 strict passes, 46/54 goals and six
  contacts. The default published dashboard cohort remains v12 (4/18 passes).
  See [the evidence snapshot](PROJECT_STATUS.md#current-evidence) before comparing them.
- Reproducible source/runtime identity and complete mission-state/evaluator
  qualification, especially uncertain attachment and bounded action termination.
- Perception latency/dropouts under eight-person rendering and GUI load.
- RViz indexed-map GLSL sampler warnings on this host.
- Longer endurance evidence: the recorded short sessions do not establish an
  uninterrupted 30-minute run.
- Deterministic startup/reset orchestration and command-source exclusivity.
- Broader paired seeds/counts/world comparisons, with uncertainty estimates.

The navigation input contract, goal criteria and scoring definitions must remain
explicit. Historical oracle passes cannot establish onboard acceptance. Visual
sessions may restart, log contacts and remain unscored.

Use the [project plan](PROJECT_PLAN.md) to select a reviewable S0–S7 work package.
Assign an owner and issue when starting work; preserve the distinction between
implemented helpers, regression checks and a satisfied native acceptance gate.

## Publication and ownership

- Preserve existing license terms. Confirm root-license policy and pedestrian
  asset/policy provenance before broader redistribution: [inventory](../LICENSES.md).
- Confirm incoming team access to all pinned GitHub forks. Private forks need a
  read-only CI credential; the workflow falls back to the repository token.
- Assign maintainers and configure GitHub branch protection, required reviews,
  required checks, private vulnerability reporting and organization contacts.
- Review Dependabot PRs; actions are pinned by commit and workflow permissions
  are read-only. Do not grant write credentials to untrusted pull requests.
- Run a fresh image/build on the incoming team's hardware and retain its output.

## Verification history

The October publication built 15 ROS packages and passed 333 bridge tests, 83
benchmark tests, 18 bringup tests, 79 general script tests, seven training tests
and all ten simulator CTests. Both source patches reproduced from their pinned
bases. See [development](DEVELOPMENT.md#publication-verification--5-october-2026)
for environment details and the three payload tests rerun in MJLab.

The earlier September development-host verification included a fresh Docker image build, 196
bridge tests, 81 benchmark tests, 18 bringup tests, 69 script tests and four
Torch-specific tests; seven simulator and two costmap/BT CTest targets also passed.
All six generated worlds were loaded by MuJoCo in the simulator tests. The GUI
was checked in Firefox for dataset selection, filtering, details and replay.
These software checks do not turn failed navigation trials into passes.

The repository's maintained documentation is indexed [here](README.md); research
drafts are kept in [archive](../archive/README.md).
