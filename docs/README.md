# Geoff-Sim documentation

<!-- geoff-nav:start -->
<p align="center">
  <a href="../README.md"><img src="media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../README.md#quick-start"><img src="media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="DEMO_GUIDE.md"><img src="media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="README.md"><img src="media/navigation/docs-current.svg" width="132" height="44" alt="Docs (current page)"></a>
  <a href="REPOSITORY_GUIDE.md"><img src="media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="PROJECT_STATUS.md"><img src="media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="PROJECT_PLAN.md"><img src="media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="RESULTS_REVIEW.md"><img src="media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="literature_review/README.md"><img src="media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="DEVELOPMENT.md"><img src="media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

The documentation hub for the completed development phase of Geoff-Sim. Start
with the [project showcase](../README.md), then choose a guide below. Dated
experiment reports preserve the configuration and evidence from each checkpoint;
their next-step statements describe that historical point in the project.

## Start here

| Goal | Reading path |
| --- | --- |
| Run the project | [Quick start](../README.md#quick-start) → [demo walkthrough](DEMO_GUIDE.md) → [visual runbook](ONBOARD_VISUAL_DEMO.md) |
| Understand the system | [Animated architecture](../README.md#architecture) → [architecture guide](ARCHITECTURE.md) → [onboard data flow](ONBOARD_SOCIAL_NAVIGATION.md) |
| Explore the code | [Repository map](REPOSITORY_GUIDE.md) → [ROS packages](../ros2_ws/src/README.md) → [development checks](DEVELOPMENT.md) |
| Reproduce experiments | [Testing guide](DEMO_TESTING.md) → [suite reference](DEMO_SUITE.md) → [published records](results/README.md) |
| Read the research | [Literature review and PDF](literature_review/README.md) → [paper catalogue](resources/RESEARCH.md) |
| Build on the work | [Project record](PROJECT_STATUS.md) → [handoff](HANDOFF.md) → [research programme](RESEARCH_PROGRAMME.md) |

## Demos, worlds and results

| Guide | Contents |
| --- | --- |
| [Demo walkthrough](DEMO_GUIDE.md) | Modes, launchers, viewers, stopping and artifacts |
| [Visual navigation runbook](ONBOARD_VISUAL_DEMO.md) | Overlays, goal handoffs, session duration and episode semantics |
| [Demo suite reference](DEMO_SUITE.md) | Presets, cases, prerequisites and result validators |
| [Layered transport testing](DEMO_TESTING.md) | Physics, ROS/DDS and mobile-stage protocols |
| [Avoidance experiments](AVOIDANCE_EXPERIMENTS.md) | Six procedural worlds, datasets and results dashboard |
| [Benchmark batches](BENCHMARK_EXPERIMENTS.md) | Research batch execution and aggregation |
| [Media catalogue](MEDIA.md) | Architecture animations, recorded demonstrations and provenance |
| [Results interpretation](RESULTS_REVIEW.md) | Measurement scope, configuration and experiment comparison |
| [Published records](results/README.md) | Portable datasets and dated investigations |

## Implementation references

| Topic | References |
| --- | --- |
| System architecture | [Component model](ARCHITECTURE.md), [sensor/control boundaries](ONBOARD_SOCIAL_NAVIGATION.md), [hardware-facing contracts](HARDWARE_FIDELITY.md) |
| Navigation | [Dynamic route planning](DYNAMIC_ROUTE_PLANNING.md), [goal-facing motion](GOAL_FACING_NAVIGATION.md), [command handoff](COMMAND_HANDOFF_AND_SUPERVISOR.md), [asynchronous supervisor](ASYNC_ESCAPE_SUPERVISOR.md) |
| Search and perception | [Exploration](EXPLORATION_SEARCH.md), [room search and colour tasks](ROOM_SEARCH_AND_TASKS.md), [RGB-D transport perception](DUAL_TRANSPORT_PERCEPTION.md) |
| Manipulation | [Dex3 contact integration](DEX3_CONTACT_MANIPULATION_20261020.md), [manipulation collection](resources/MANIPULATION.md), [payload contract](NATIVE_PAYLOAD_CONTRACT.md) |
| Training | [Training and calibration](TRAINING_AND_CALIBRATION.md), [payload policy work](PAYLOAD_POLICY_TRAINING.md), [Grove-G1 review](GROVE_G1_REVIEW.md) |
| Survey and prediction experiments | [Survey recovery](SURVEY_RECOVERY_IMPLEMENTATION_20261025.md), [escape primitives](ESCAPE_PRIMITIVES_20261026.md), [forecast audit](FORECAST_INVALIDATION_20261027.md), [heading history](HEADING_HISTORY_20261027.md) |
| Source and execution | [ROS package map](../ros2_ws/src/README.md), [script entry points](../scripts/README.md), [reviewed patches](../patches/README.md) |

## Research and project history

[![Browse the visual resource library](media/resources/library.svg)](resources/README.md)

- [Eight-topic resource library](resources/README.md): guides, code, papers,
  upstream tools and historical context.
- [Literature review](literature_review/README.md): complete review, bibliography,
  source appraisals and reading audits.
- [Project status and progression](PROJECT_STATUS.md): wrap-up and dated evidence.
- [Research programme](RESEARCH_PROGRAMME.md): preserved experiment proposals and
  literature-gap traceability for future reuse.
- [Project plan](PROJECT_PLAN.md) and [technical roadmap](SOCIAL_LOCOMANIPULATION_ROADMAP.md):
  retained work-package definitions and acceptance criteria.
- [Archive](../archive/README.md): earlier implementation investigations and
  specialist calibration designs.

The maintained review and research programme replace the earlier literature-brief
and research-plan drafts. Those drafts remain available in Git history. The former
`INDEX.md` catalogue is consolidated into this page.

## Development and reuse

- [Development and checks](DEVELOPMENT.md)
- [Repository and artifact layout](REPOSITORY_GUIDE.md)
- [Maintainer guide](MAINTAINER_GUIDE.md) and [handoff](HANDOFF.md)
- [Release and artifact policy](RELEASES.md)
- [Contributing](../CONTRIBUTING.md), [licenses](../LICENSES.md) and [citation](../CITATION.cff)

Architecture GIFs are illustrated system models; demonstration GIFs are recorded
simulation sequences. Their metadata and static reading copies are linked in the
[media catalogue](MEDIA.md).
