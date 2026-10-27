# Development and contribution links

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

[← Resource library](README.md) · Related: [Build/test references](SIMULATION.md#build-test-and-community-resources)
· [Diagnostic tools](SIMULATION.md#recording-debugging-and-profiling) · [Forward plan](../PROJECT_PLAN.md)

[![Verification layers — open the development guide](../media/verification-tiers.svg)](../DEVELOPMENT.md)

## Find and understand the source

| Resource | Kind | Why open it? |
| --- | --- | --- |
| [Repository guide](../REPOSITORY_GUIDE.md) | Guide | Annotated tree, edit locations and generated/source boundaries |
| [ROS package map](../../ros2_ws/src/README.md) | Guide | Every local package, upstream checkout and manipulation integration path |
| [Script catalogue](../../scripts/README.md) | Guide | Public launchers and maintained internal tools |
| [Container environment](../../docker/README.md) | Guide | Build dependencies, source mounts and environment assumptions |
| [Dockerfile](../../docker/Dockerfile) / [requirements](../../docker/requirements.txt) | Code | Actual image and direct Python dependencies |
| [Compose configuration](../../docker-compose.yml) / [GPU overlay](../../docker-compose.gpu.yml) | Code | Host/ROS/display settings and optional training hardware |
| [Source patch guide](../../patches/README.md) | Guide | Export, check and pristine replay of upstream modifications |
| [Integration profiles](../../integrations/README.md) | Guide | Optional external stacks and their boundaries |

## Check a change

| Resource | Kind | Why open it? |
| --- | --- | --- |
| [Development and verification](../DEVELOPMENT.md) | Guide / evidence | Supported commands, separate ROS/MJLab environments and recorded check scope |
| [Workspace test runner](../../scripts/test_workspace_inside.sh) | Code | Build/test modes and strict mock benchmark gate |
| [Verification workflow](../../.github/workflows/verification.yml) | Code | What CI actually runs |
| [GitHub Actions runs](https://github.com/DavidBatty95/Geoff-Sim/actions/workflows/verification.yml) | Live project page | Inspect actual CI outcomes and artifacts |
| [Documentation checker](../../scripts/check_docs.py) | Code | Maintained links and SVG syntax |
| [Graphics/navigation generator](../../scripts/generate_readme_graphics.py) | Code | Regenerate diagrams, shared menus and this library's paper index |
| [Navigation definitions](../../scripts/readme_navigation.py) | Code | One source of truth for button labels, icons and targets |
| [Resource definitions](../../scripts/resource_library.py) | Code | Topic cards and complete bibliography grouping |
| [Dashboard browser check](../../scripts/check_dashboard_browser.py) | Code | Optional real-browser verification of results inspection |

The [media catalogue](../MEDIA.md) describes the graphics and navigation workflow.
The [results-review guide](../RESULTS_REVIEW.md) explains why passing software
checks and passing a native navigation trial are different evidence tiers.

## Contribute and report findings

| Resource | Kind | Purpose |
| --- | --- | --- |
| [Contributing](../../CONTRIBUTING.md) | Guide | Focused changes, evidence, source boundaries and documentation expectations |
| [Project plan](../PROJECT_PLAN.md) | Plan | Choose an S0–S7 or R1–R6 work package and its completion evidence |
| [Research programme](../RESEARCH_PROGRAMME.md) | Research plan | Choose a P0–P7 scientific phase, define the hypothesis and preserve claim-to-artifact traceability |
| [Team handoff](../HANDOFF.md) | Guide | First-session workflow, priorities and ownership context |
| [Issue tracker](https://github.com/DavidBatty95/Geoff-Sim/issues) | Live project page | Report reproducible problems and coordinate work |
| [Pull requests](https://github.com/DavidBatty95/Geoff-Sim/pulls) | Live project page | Review proposed publications and changes |
| [Issue templates](../../.github/ISSUE_TEMPLATE/) / [PR template](../../.github/pull_request_template.md) | Guide | Structure a useful report |
| [Security scope](../../SECURITY.md) | Guide | Supported deployment and reporting context |
| [License inventory](../../LICENSES.md) | Reference | Existing package/upstream licensing and owner decisions |

[Back to the library](README.md) · [Project history](HISTORY.md)
