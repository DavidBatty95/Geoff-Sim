# Resource library

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

![Geoff-Sim resource library: guides, code, results, research, official tools and history](../media/resources/library.svg)

**A topic-clustered archive of useful project links.** Choose a picture to open
a collection, or use the text index below. Each collection explains what a link
is useful for and connects reading material to the corresponding code or evidence.

**Catalogue assembled: 5 October 2026.** The library indexes existing material;
it does not change the measured status of the project. Start with
[current status](../PROJECT_STATUS.md) when interpreting development claims.

**Looking for online references?** The
[tutorials, repositories and libraries directory](SIMULATION.md) collects ROS 2
Humble docs, step-by-step lessons, Nav2/MoveIt, simulation, policy learning,
perception libraries and development tools in one place. Each entry explains
its purpose and whether it is used here, training-only or an optional reference.

## Browse by topic

<p align="center">
  <a href="START_HERE.md"><img src="../media/resources/start.svg" width="420" alt="Start here and demos — installation, worlds and viewer walkthroughs"></a>
  <a href="NAVIGATION.md"><img src="../media/resources/navigation.svg" width="420" alt="Perception and navigation — sensors, SLAM, tracking and planning"></a>
</p>
<p align="center">
  <a href="MANIPULATION.md"><img src="../media/resources/manipulation.svg" width="420" alt="Locomotion and manipulation — policies, arms, grasping and transport"></a>
  <a href="EXPERIMENTS.md"><img src="../media/resources/experiments.svg" width="420" alt="Experiments and evidence — datasets, dashboard and reproducibility"></a>
</p>
<p align="center">
  <a href="RESEARCH.md"><img src="../media/resources/research.svg" width="420" alt="Research and papers — the review and all 50 references grouped by topic"></a>
  <a href="SIMULATION.md"><img src="../media/resources/simulation.svg" width="420" alt="Online tutorials, important repositories and libraries — ROS 2, navigation, simulation, manipulation and learning"></a>
</p>
<p align="center">
  <a href="DEVELOPMENT.md"><img src="../media/resources/development.svg" width="420" alt="Development and contribution — builds, tests, CI and source patches"></a>
  <a href="HISTORY.md"><img src="../media/resources/history.svg" width="420" alt="Progress and historical archive — milestones, plans and earlier research"></a>
</p>

| Collection | Topics and useful starting points |
| --- | --- |
| [Start here and demos](START_HERE.md) | Installation, daily commands, world selection, viewers, recordings and troubleshooting |
| [Perception and navigation](NAVIGATION.md) | Architecture, data-source boundaries, localization, people, routes and supervision |
| [Locomotion and manipulation](MANIPULATION.md) | Policy contracts, response calibration, arm control, MoveIt, BrainCo and payload training |
| [Experiments and evidence](EXPERIMENTS.md) | Scored trials, datasets, replay, raw artifact locations and report interpretation |
| [Research and paper archive](RESEARCH.md) | Review PDF, source audits and every bibliography entry, grouped into seven research themes |
| [Online tutorials, repositories and libraries](SIMULATION.md) | ROS 2 learning paths and APIs, Nav2, MoveIt, simulation, ML, perception, diagnostics and build tools |
| [Development and contribution](DEVELOPMENT.md) | Source layout, package READMEs, checks, CI, patch workflow, issues and contribution guidance |
| [Progress and historical archive](HISTORY.md) | Published milestones, forward plan, earlier proposals, project history and profile package |

## Choose a reading route

- **First visit:** [project overview](../../README.md) → [demo walkthrough](../DEMO_GUIDE.md)
  → [worlds](../AVOIDANCE_EXPERIMENTS.md) → [current evidence](../PROJECT_STATUS.md#current-evidence).
- **Understand a navigation decision:** [architecture](../ARCHITECTURE.md) →
  [onboard source boundaries](../ONBOARD_SOCIAL_NAVIGATION.md) →
  [route planning](../DYNAMIC_ROUTE_PLANNING.md) → [failure investigation](../NAVIGATION_RELIABILITY_INVESTIGATION.md).
- **Work on transport:** [payload report](../PAYLOAD_POLICY_TRAINING.md) →
  [implementation map](MANIPULATION.md) → [project plan](../PROJECT_PLAN.md).
- **Review an experiment:** [results guide](../RESULTS_REVIEW.md) →
  [dataset catalogue](../results/README.md) → [research evaluation references](RESEARCH.md).
- **Make a contribution:** [folder guide](../REPOSITORY_GUIDE.md) →
  [development checks](../DEVELOPMENT.md) → [contributing](../../CONTRIBUTING.md).
- **Learn the underlying tools:** [online learning paths](SIMULATION.md#suggested-learning-paths)
  → [ROS 2 fundamentals](SIMULATION.md#ros-2-fundamentals-and-tutorials)
  → the topic-specific library/repository sections on the same page.

## Read the link labels

| Kind | Meaning |
| --- | --- |
| **Guide** | Maintained project instructions or explanation |
| **Code** | An implementation, interface, configuration or test entry point |
| **Evidence** | Dated measurements or a report with a defined evaluation scope |
| **Research** | Bibliographic records, synthesis or research analysis |
| **Upstream** | Official external documentation or source; version compatibility still matters |
| **Historical** | Earlier notes retained for context, not current instructions |

Pictures are clickable navigation aids. Demo GIFs are short visual recordings;
SVGs are illustrations. See [media provenance](../MEDIA.md) for their source and scope.
The research index preserves the existing bibliography's identity and inspection
records rather than implying every linked paper was read in full again.

## Keeping the archive useful

Add a link to the most relevant topic with a one-line explanation and its kind.
Prefer maintained runbooks, primary papers and official upstream sources. Keep
historical material visibly labelled and link its current replacement when one
exists. Point to dataset catalogues and experiment ledgers rather than duplicating
every raw trial or local checkpoint path.

When adding a new topic or paper, update `scripts/resource_library.py` and regenerate:

```bash
python3 scripts/generate_readme_graphics.py
python3 scripts/generate_readme_graphics.py --check
python3 scripts/check_docs.py
```

Local links, SVG syntax, generated cards and the complete paper index are checked
offline. External sites can move or restrict automated access; maintain their
official landing URLs and report unavailable resources through
[GitHub issues](https://github.com/DavidBatty95/Geoff-Sim/issues).

[Documentation hub](../README.md) · [Folder-by-folder guide](../REPOSITORY_GUIDE.md)
