# Start here and demo links

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

[← Resource library](README.md) · Next: [Perception and navigation](NAVIGATION.md)

[![Recorded G1 crowd demonstration — open the visual-demo runbook](../media/onboard-crowd.gif)](../ONBOARD_VISUAL_DEMO.md)

*Recorded visual session; open the image for the viewer/runbook guide. This is
not a scored reliability claim.*

## Install and orient yourself

New to the underlying tools? Start with the
[online learning paths](SIMULATION.md#suggested-learning-paths) and
[ROS 2 Humble tutorials](SIMULATION.md#ros-2-fundamentals-and-tutorials).
Use Geoff-Sim's installation guide for this repository's actual setup.

| Resource | Kind | Why open it? |
| --- | --- | --- |
| [README quick start](../../README.md#quick-start) | Guide | Supported host, clone/install commands and the first demo |
| [Installation script](../../install.sh) | Code | Inspect what setup provisions and builds |
| [Container guide](../../docker/README.md) | Guide | Image, Compose, mounts, display and build commands |
| [Folder guide](../REPOSITORY_GUIDE.md) | Guide | Find the correct source, script, dataset or artifact folder |
| [Daily CLI](../../geoff) / [script catalogue](../../scripts/README.md) | Code / guide | Discover `worlds`, `demo`, `benchmark`, `results` and experimental transport |
| [Project status](../PROJECT_STATUS.md) | Guide / evidence | Understand what is implemented and what remains below acceptance |

## Pick a demonstration

| Resource | Kind | Why open it? |
| --- | --- | --- |
| [Demo walkthrough](../DEMO_GUIDE.md) | Guide | Select visual, scored, diagnostic or transport modes |
| [Long visual-demo runbook](../ONBOARD_VISUAL_DEMO.md) | Guide | Understand MuJoCo/RViz, goal handoffs, restarts and logs |
| [Demo suite reference](../DEMO_SUITE.md) | Guide | Presets, individual cases, prerequisites and acceptance rules |
| [World definitions](../AVOIDANCE_EXPERIMENTS.md#representative-world-definitions) | Guide | Courtyard, café, station, clinic, concourse and office layouts |
| [Demo case catalogue](../../scripts/demo_suite.py) | Code | Exact orchestration and case-specific settings |
| [Bringup package](../../ros2_ws/src/unitree_g1_sim_bringup/README.md) | Code / guide | Launch composition, configuration profiles and viewer layouts |
| [Payload fixture guide](../PAYLOAD_POLICY_TRAINING.md) | Guide / evidence | Experimental transport with an explicit local policy |

## Browse the media

| Preview | Simulation | Perception / planning |
| --- | --- | --- |
| Pedestrian plaza | [Crowd GIF](../media/onboard-crowd.gif) | [RViz GIF](../media/onboard-decisions.gif) |
| Residential courtyard | [Crowd GIF](../media/courtyard/onboard-crowd.gif) | [RViz GIF](../media/courtyard/onboard-decisions.gif) |
| Station entrance | [Crowd GIF](../media/station/onboard-crowd.gif) | See the [visual guide](../ONBOARD_VISUAL_DEMO.md) for live displays |

[Complete media catalogue](../MEDIA.md) explains recordings, diagrams and provenance.
For a stalled session, start with [troubleshooting](../DEMO_GUIDE.md#troubleshooting)
and the run's status/logs; for a failed scored run, use [results review](../RESULTS_REVIEW.md).

[Back to the library](README.md) · [Inspect results next](EXPERIMENTS.md)
