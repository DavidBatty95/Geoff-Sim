# ROS 2 source workspace

<!-- geoff-nav:start -->
<p align="center">
  <a href="../../README.md"><img src="../../docs/media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../../README.md#quick-start"><img src="../../docs/media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="../../docs/DEMO_GUIDE.md"><img src="../../docs/media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="../../docs/INDEX.md"><img src="../../docs/media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="../../docs/REPOSITORY_GUIDE.md"><img src="../../docs/media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="../../docs/PROJECT_STATUS.md"><img src="../../docs/media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="../../docs/PROJECT_PLAN.md"><img src="../../docs/media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="../../docs/RESULTS_REVIEW.md"><img src="../../docs/media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="../../docs/literature_review/README.md"><img src="../../docs/media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="../../docs/DEVELOPMENT.md"><img src="../../docs/media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

[Project home](../../README.md) · [Repository guide](../../docs/REPOSITORY_GUIDE.md) · [Architecture](../../docs/ARCHITECTURE.md) · [Forward plan](../../docs/PROJECT_PLAN.md)

This workspace combines local Humble packages with pinned Unitree sources.
`scripts/setup_sources.sh` prepares the upstream checkouts and applies the
[reviewed patches](../../patches/README.md). Local packages are maintained in
the parent repository.

## Package map

| Package | Responsibility | Start here |
| --- | --- | --- |
| [unitree_g1_mujoco_bridge](unitree_g1_mujoco_bridge/README.md) | Sensors, state estimation, tracking, navigation supervision, locomotion and experimental transport | Python modules and `test/` |
| [unitree_g1_sim_bringup](unitree_g1_sim_bringup/README.md) | Launch composition, parameter profiles and RViz layouts | `launch/g1_sim.launch.py`, `config/` |
| [dynamic_obstacle_benchmarks](dynamic_obstacle_benchmarks/README.md) | Trial orchestration, source contracts, scoring and result writing | `config/`, runner and tests |
| [unitree_g1_description](unitree_g1_description/README.md) | G1 URDF and meshes with default articulated Dex3 hands | `urdf/`, `meshes/`, `dex3_sources/` |
| [unitree_social_navigation_msgs](unitree_social_navigation_msgs/README.md) | Social-navigation message contracts | `msg/` |
| [unitree_social_costmap](unitree_social_costmap/README.md) | C++ Nav2 social costmap and behavior-tree integration | `src/`, `include/` |
| [g1_msgs](g1_msgs/README.md) | Approach/manipulation/transport actions and object-selectable grasp service | `action/`, `srv/` |
| [g1_locomotion](g1_locomotion/README.md) | Fine base approach and retreat actions | `config/`, C++ planner/server |
| [g1_moveit_config](g1_moveit_config/README.md) | Stationary arm planning configuration | SRDF, kinematics and controllers |
| [g1_manipulation](g1_manipulation/README.md) | MoveIt-based pick, place and named-posture actions | Action server and geometry helpers |
| [g1_task_orchestration](g1_task_orchestration/README.md) | Action-based transport sequencing | Transport server, launch and config |

### Pinned external sources

| Checkout | Role |
| --- | --- |
| `unitree_mujoco/` | Simulator, robot assets and native C++ tests; local modifications are carried in `patches/` |
| `unitree_ros2/` | Unitree messages and examples, including body and hand interfaces |
| `unitree_sdk2/` | DDS SDK used at the simulator/low-level command boundary |

MJLab lives at `vendor/mjlab/`, outside this ROS workspace. The optional Grove-G1
profile has its own Jazzy workspace and runtime; see
[integration boundaries](../../integrations/grove_g1/README.md).

## Two manipulation paths

The earlier action stack links `g1_locomotion`, MoveIt, `g1_manipulation` and
`g1_task_orchestration`. The newer experimental sensor-driven transport workflow
uses bridge modules and `scripts/run_transport_demo.py`. They are distinct
integration paths; the [project plan](../../docs/PROJECT_PLAN.md) tracks their
future consolidation. The payload report describes the latter path's isolated
fixture evidence, not general acceptance of both stacks.

## Build and check

Run from the repository root after installation:

```bash
docker compose run --rm unitree-g1-sim /workspace/scripts/build_ws.sh
docker compose run --rm unitree-g1-sim /workspace/scripts/test_workspace_inside.sh fast
```

Package README `colcon` and `ros2` commands assume a shell inside the supported
container, a working directory of `/workspace/ros2_ws`, and a sourced ROS/installed
workspace environment. Use one build/install overlay consistently. See
[development](../../docs/DEVELOPMENT.md) for `.verification` builds and test scope.

## Where to make a change

- Sensor/estimator or planner behavior → bridge module, its tests and bringup profile.
- Trial criteria or evaluation data → benchmark package and result validator.
- Action request/response contract → `g1_msgs`, rebuild consumers, update package docs.
- Physics or native sensors → simulator source, CTests, then patch export/replay.
- World geometry → the generator in `scripts/`, not a generated XML file.

The [repository guide](../../docs/REPOSITORY_GUIDE.md) connects these packages to
launchers, datasets and documentation.
