# Documentation

<!-- geoff-nav:start -->
<p align="center">
  <a href="../README.md"><img src="media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../README.md#quick-start"><img src="media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="DEMO_GUIDE.md"><img src="media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="INDEX.md"><img src="media/navigation/docs-current.svg" width="132" height="44" alt="Docs (current page)"></a>
  <a href="REPOSITORY_GUIDE.md"><img src="media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="PROJECT_STATUS.md"><img src="media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="PROJECT_PLAN.md"><img src="media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="RESULTS_REVIEW.md"><img src="media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="literature_review/README.md"><img src="media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="DEVELOPMENT.md"><img src="media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

Geoff-Sim combines Unitree G1 locomotion, MuJoCo simulation, ROS 2 onboard
navigation and source-linked evaluation. Choose a reading path below, or browse
the reference tables.

**Browse visually:** the [resource library](resources/README.md) groups useful
guides, code, evidence, upstream references and historical notes into eight
collections. Start there for topic-based exploration, or use the catalogue below.

## Project orientation

Latest checkpoint: [delayed rollout timing and native stopping](NATIVE_STOPPING_20261015.md)
— declared stopping gate, eight probes, three mission outcomes and verification.

| Page | Purpose |
| --- | --- |
| [Visual resource library](resources/README.md) | Clickable topic cards and annotated links across project content |
| [Complete paper archive](resources/RESEARCH.md) | All 50 reviewed source identities grouped into seven research themes |
| [Online tutorials, repositories and libraries](resources/SIMULATION.md) | Central external directory: ROS 2, Nav2, MoveIt, simulation, learning, perception, diagnostics and development |
| [Repository and folder guide](REPOSITORY_GUIDE.md) | Visual map, annotated tree, package ownership and where to make a change |
| [Project status and progression](PROJECT_STATUS.md) | Dated published baseline, milestone timeline, latest evidence and blockers |
| [Detailed project plan](PROJECT_PLAN.md) | Work packages, dependencies, exit criteria and update process |
| [Performance reliability checkpoint](PERFORMANCE_FOUNDATION.md) | Experiment accounting, ordered command publication, explicit settling outcomes and current verification |
| [15 October navigation integrity](NAVIGATION_INTEGRITY_20261015.md) | Checked goal transforms, motion-bounded localization, native deliveries and retained failures |
| [Placement alignment and route diagnostics](PLACEMENT_ALIGNMENT_20261015.md) | World-relative pre-release correction, blocked-map snapshots and optional worker limits; native qualification remains open |
| [Policy cadence and delivery follow-up](POLICY_CADENCE_20261015.md) | Minimum-spaced observations, 4/4 compact tasks, medium-building delivery and complete-workflow footage |
| [Crowded docking and workspace recovery](CROWDED_DOCKING_20261015.md) | Bounded support reinspection, corridor admission, native placement correction and unresolved people-present failures |
| [Gap-aligned research programme](RESEARCH_PROGRAMME.md) | P0–P7 phases, G1–G5 traceability, experiment design, novelty gates and candidate papers |
| [ROS workspace map](../ros2_ws/src/README.md) | Every local package and the two manipulation integration paths |
| [Published dataset catalogue](results/README.md) | Entry points into the portable experiment records |

## Start here

- **Run your first demo:** [installation](../README.md#quick-start) →
  [demo walkthrough](DEMO_GUIDE.md) → [visual runbook](ONBOARD_VISUAL_DEMO.md).
- **Inspect the research:** [results review](RESULTS_REVIEW.md) →
  [published evidence](DEMO_RESULTS.md) → [reliability investigation](NAVIGATION_RELIABILITY_INVESTIGATION.md).
- **Understand the implementation:** [architecture](ARCHITECTURE.md) →
  [source boundaries](ONBOARD_SOCIAL_NAVIGATION.md) → [route planning](DYNAMIC_ROUTE_PLANNING.md).
- **Contribute:** [development checks](DEVELOPMENT.md) →
  [contribution guide](../CONTRIBUTING.md) → [handoff](HANDOFF.md).
- **Follow progress:** [status](PROJECT_STATUS.md) → [forward plan](PROJECT_PLAN.md) →
  [technical implementation ledger](SOCIAL_LOCOMANIPULATION_ROADMAP.md).

## Demos and experiments

| Guide | What it covers |
| --- | --- |
| [Demo walkthrough](DEMO_GUIDE.md) | Choosing a mode, launching viewers, observing behavior and locating artifacts |
| [Visual runbook](ONBOARD_VISUAL_DEMO.md) | Viewer overlays, goal handoffs, episode restarts and session semantics |
| [Demo suite reference](DEMO_SUITE.md) | Presets, individual cases, prerequisites, validators and policy selection |
| [Avoidance experiments](AVOIDANCE_EXPERIMENTS.md) | Six worlds, dataset collection, browser inspection and methodology |
| [General benchmark batches](BENCHMARK_EXPERIMENTS.md) | Broader research batch execution and aggregation |
| [Dual-cube perception diagnostics](DUAL_TRANSPORT_PERCEPTION.md) | Camera timing fix, retained native outcomes and remaining stability blockers |
| [Native payload contract](NATIVE_PAYLOAD_CONTRACT.md) | Delay/inertial correction, cycle-03 training, two consecutive dual-delivery passes and open phase gates |
| [Hardware fidelity and performance](HARDWARE_FIDELITY.md) | G1/BrainCo interfaces, sensor semantics, model audit, live rates and publication validation |
| [Transport people-clearance follow-up](TRANSPORT_PEOPLE_CLEARANCE.md) | Wider table aisles, continuous workspace checks, bounded recovery and retained native failures |
| [Unknown-environment search prototype](EXPLORATION_SEARCH.md) | RViz reasoning overlays, measured goal-selection timing, ORI/STRANDS attribution and incomplete delivery outcomes |
| [Room search and colour tasks](ROOM_SEARCH_AND_TASKS.md) | Medium multi-room fixture, random cross-colour tasks, region decisions and visually inferred table approach sides |
| [Demo failure review](DEMO_FAILURE_REVIEW.md) | Incident chronology, demonstrated false tracks, runtime bottlenecks and ranked validation paths |
| [Perception/response follow-up](PERCEPTION_RESPONSE_FOLLOWUP.md) | Wall-supported track rejection, body-instability termination, native pilots and measured stopping response |
| [Earlier avoidance follow-up](EARLY_AVOIDANCE_FOLLOWUP.md) | Experimental overtaking lookahead, inspection settling, failed native pilots and command-replay diagnostics |
| [Media catalogue](MEDIA.md) | All README GIFs, recording provenance and diagram descriptions |
| [Script entry points](../scripts/README.md) | Daily CLI and lower-level maintenance tools |

## Results and research reviews

| Guide | What it covers |
| --- | --- |
| [Reviewing results and evidence](RESULTS_REVIEW.md) | Evidence categories, metrics, provenance and reproducible reporting |
| [Published demo evidence](DEMO_RESULTS.md) | Published v12 cohort and separately labelled historical demos |
| [Navigation reliability investigation](NAVIGATION_RELIABILITY_INVESTIGATION.md) | Retained corrections, failed experiments and the iteration ledger |
| [Research gaps and literature review](literature_review/README.md) | Intended contributions first; 74-page PDF, 50 critical appraisals, systematic extraction, novelty roadmap and linked bibliography |
| [Grove-G1 implementation review](GROVE_G1_REVIEW.md) | Dated source review, architecture comparisons and proposed transfer experiments; separate from the pinned integration profile |
| [Training and calibration](TRAINING_AND_CALIBRATION.md) | Policy training, response measurements and calibration tools |
| [Payload-aware training](PAYLOAD_POLICY_TRAINING.md) | Training cycle, native transport results and explicit checkpoint reproduction |
| [Transport implementation plan](TRANSPORT_DEMO_PLAN.md) | Mission design, integration requirements and acceptance goals |
| [Social locomanipulation roadmap](SOCIAL_LOCOMANIPULATION_ROADMAP.md) | Active staged plan for BrainCo-equipped indoor delivery, implementation ledger and promotion gates |
| [Goal-facing navigation](GOAL_FACING_NAVIGATION.md) | Forward-travel preference, lateral exceptions, native turn measurements and remaining calibration gates |
| [S0/R1–R3 phase progress](PHASE_PROGRESS_20261005.md) | Runtime capture, timing, bounded scheduling and interruption implementation with native outcomes |
| [Command handoff and supervisor](COMMAND_HANDOFF_AND_SUPERVISOR.md) | Atomic command epochs, writer checks, lineage and collision-query performance evidence |
| [Asynchronous escape supervisor](ASYNC_ESCAPE_SUPERVISOR.md) | Worker/control separation, fresh-result revalidation, 50 Hz candidate and retained native failures |

## Architecture and maintenance

| Guide | What it covers |
| --- | --- |
| [Architecture](ARCHITECTURE.md) | Components and end-to-end data flow |
| [Onboard source boundaries](ONBOARD_SOCIAL_NAVIGATION.md) | Sensor-driven inputs, simulator-truth isolation and validation scope |
| [Dynamic route planning](DYNAMIC_ROUTE_PLANNING.md) | Replanning triggers, timing and route execution |
| [Development and verification](DEVELOPMENT.md) | Host checks, Docker tests and source-patch maintenance |
| [Maintainer guide](MAINTAINER_GUIDE.md) | Required checks, branch rules, security settings and owner responsibilities |
| [Release and artifact policy](RELEASES.md) | Versioning, clean-checkout reproduction, model access and citation |
| [Changelog](../CHANGELOG.md) / [Software citation](../CITATION.cff) | User-facing publication history and machine-readable citation metadata |
| [Team handoff](HANDOFF.md) | Operational context and remaining work |
| [Source patches](../patches/README.md) | Reproducing changes to pinned upstream sources |
| [Contributing](../CONTRIBUTING.md) | Contribution workflow and review expectations |
| [Code of conduct](../CODE_OF_CONDUCT.md) | Collaboration expectations and private reporting routes |
| [Security](../SECURITY.md) / [Licenses](../LICENSES.md) | Reporting scope and existing licensing information |
| [GitHub profile package](../profile/SETUP.md) | Optional companion profile setup |

Portable measurements live in [`docs/results/`](results/). Raw simulation logs,
generated scenes and locally trained policies follow the artifact locations in
their respective runbooks. The [calibration schemas](schemas/) are machine
contracts. [Archived research drafts](../archive/README.md) preserve historical
context; use maintained guides for current entry points.
