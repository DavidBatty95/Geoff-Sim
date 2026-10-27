# Repository and folder guide

<!-- geoff-nav:start -->
<p align="center">
  <a href="../README.md"><img src="media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../README.md#quick-start"><img src="media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="DEMO_GUIDE.md"><img src="media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="README.md"><img src="media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="REPOSITORY_GUIDE.md"><img src="media/navigation/folders-current.svg" width="132" height="44" alt="Folders (current page)"></a>
  <br>
  <a href="PROJECT_STATUS.md"><img src="media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="PROJECT_PLAN.md"><img src="media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="RESULTS_REVIEW.md"><img src="media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="literature_review/README.md"><img src="media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="DEVELOPMENT.md"><img src="media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

Use this page to find the right command, source package or result without having
to understand the entire stack first. Paths and commands below are relative to
the repository root unless noted otherwise.

For content grouped by subject rather than folder, use the
[visual resource library](resources/README.md).

![Task-oriented map of the Geoff-Sim repository](media/repository-map.svg)

## Start with the task

| I want to… | Start here | Then look at |
| --- | --- | --- |
| Run a demo | [`./geoff`](../geoff), [demo walkthrough](DEMO_GUIDE.md) | [`scripts/`](../scripts/README.md) |
| Understand the current state | [Project status](PROJECT_STATUS.md) | [Forward plan](PROJECT_PLAN.md) |
| Find the architecture | [System overview](ARCHITECTURE.md) | [ROS package map](../ros2_ws/src/README.md) |
| Tune perception or navigation | [Bridge package](../ros2_ws/src/unitree_g1_mujoco_bridge/README.md) | Bringup `config/social_navigation_onboard_v1.yaml` |
| Change a scene or create a trial | [`scripts/`](../scripts/README.md) | `avoidance_worlds.py`, `world_appearance.py`, `transport_worlds.py` |
| Work on payload transport | [Payload report](PAYLOAD_POLICY_TRAINING.md) | `transport_demo.py`, `transport_perception.py`, `run_transport_demo.py` |
| Understand MoveIt-based actions | [Manipulation package](../ros2_ws/src/g1_manipulation/README.md) | [Action orchestration](../ros2_ws/src/g1_task_orchestration/README.md) |
| Inspect measurements | [Results review](RESULTS_REVIEW.md) | [`docs/results/`](results/README.md), [`tools/`](../tools/README.md) |
| Change simulator internals | [`patches/`](../patches/README.md) | `ros2_ws/src/unitree_mujoco/simulate/` |
| Rebuild or run checks | [Development](DEVELOPMENT.md) | [`docker/`](../docker/README.md), `.github/workflows/verification.yml` |
| Read the research argument | [Literature review](literature_review/README.md) | PDF, reading audit and source records |

## Annotated tree

This is the maintained layout, not an exhaustive listing of generated files.

```text
Geoff-Sim/
├── README.md                   # Landing page, gallery and entry points
├── geoff                       # Daily demo / benchmark / results CLI
├── install.sh                  # Supported installation workflow
├── docker-compose*.yml         # Runtime service and optional GPU overlay
├── .github/                    # CI, Dependabot and contribution templates
├── docker/                     # Image, pinned runtime requirements, entrypoint
├── docs/
│   ├── README.md               # Canonical documentation hub
│   ├── REPOSITORY_GUIDE.md     # This folder guide
│   ├── PROJECT_STATUS.md       # Dated status, milestones and evidence
│   ├── PROJECT_PLAN.md         # Deliverables and promotion criteria
│   ├── media/                  # GIFs, screenshots and generated SVGs
│   ├── results/                # Portable published measurements
│   ├── resources/              # Visual topic collections and complete paper index
│   ├── literature_review/      # Review PDF, source and bibliography audit
│   └── schemas/                # Machine-readable calibration contracts
├── scripts/                    # Builds, demos, experiments, training and checks
├── tools/                      # Results-dashboard HTML frontend
├── ros2_ws/
│   ├── config/                 # DDS configuration
│   ├── src/                    # Local ROS packages and pinned upstream sources
│   └── .verification/          # Ignored build / install / log overlay
├── patches/                    # Reproducible changes on top of upstream pins
├── vendor/                     # MJLab and external/build dependencies
├── integrations/               # Optional external stack profiles
├── profile/                    # Companion GitHub-profile package
└── archive/                    # Historical experiments and research drafts
```

## How the directories fit together

1. **Install and build:** `install.sh` and `scripts/` prepare pinned sources,
   apply `patches/`, generate assets and build the Docker/ROS/MuJoCo environment.
2. **Launch:** `./geoff` selects a script; the script supplies case settings to
   the ROS bringup package and starts the simulator.
3. **Run:** packages under `ros2_ws/src/` handle sensors, planning, locomotion,
   manipulation and evaluation. [Architecture](ARCHITECTURE.md) explains topic flow.
4. **Record:** experiment tools save commands, identity metadata, logs and results
   under the benchmark artifact directories.
5. **Publish and inspect:** portable records in `docs/results/` feed the Python
   results server and the frontend in `tools/`. Documentation explains their scope.

## Source ownership and generated artifacts

| Kind | Examples | Maintenance rule |
| --- | --- | --- |
| Project source | `scripts/`, local ROS packages, Docker configuration | Edit and test in the parent repository |
| Pinned upstream source | `unitree_mujoco`, `unitree_ros2`, `unitree_sdk2`, `vendor/mjlab` | Preserve revision identity; use the documented patch workflow for local changes |
| Reviewed source patch | `patches/*.patch`, `patches/manifest.json` | Export deliberately, verify against the pinned base, publish with matching source changes |
| Generated scene/mesh | Procedural world XML, walking assets, generated hand model | Change the generator and rebuild; do not treat local generated output as the source |
| Build products | `.verification/`, `build/`, `install/`, logs | Recreate in the supported environment |
| Local experiment output | Benchmark result directories, MJLab logs and policies | Preserve provenance locally; publish selected portable evidence with its scope |
| Presentation asset | README SVGs and demo GIFs | Regenerate SVGs from the graphics tool; retain recording provenance for GIFs |

Patched submodules can correctly appear dirty after setup. Their changes are
carried by the parent repository's patch snapshot. Machine-specific simulator
configuration is separate from that reviewed source snapshot.

## Where results go

```text
ros2_ws/src/dynamic_obstacle_benchmarks/results/
├── demo_suite/          # suite_<timestamp> and visual_session_<timestamp>
├── avoidance_matrix/    # Raw serial world/seed experiments
├── fixed_command/       # Command-response measurements
└── transport_demo/      # Pickup / carry / placement artifacts

vendor/mjlab/logs/        # Local training checkpoints and evaluation output
docs/results/            # Published portable evidence for readers
```

Raw folders may be absent in a clean clone. See each runbook for output overrides
and the [results-review guide](RESULTS_REVIEW.md) for provenance expectations.

## Keep the map current

When moving a package or adding an entry point, update its README, this guide and
the documentation index. Graphics are generated by
[`generate_readme_graphics.py`](../scripts/generate_readme_graphics.py):

```bash
python3 scripts/generate_readme_graphics.py
python3 scripts/generate_readme_graphics.py --check
python3 scripts/check_docs.py
```

The [forward plan](PROJECT_PLAN.md#keeping-the-project-current) describes the
status-update process so folder documentation and project progress stay aligned.
