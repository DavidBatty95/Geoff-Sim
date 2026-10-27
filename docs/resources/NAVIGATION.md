# Perception and navigation links

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

[← Resource library](README.md) · Related: [Research](RESEARCH.md) · [Experiments](EXPERIMENTS.md)

[![Onboard runtime architecture — open the system guide](../media/architecture.svg)](../ARCHITECTURE.md)

## Understand the pipeline

| Resource | Kind | Why open it? |
| --- | --- | --- |
| [Architecture](../ARCHITECTURE.md) | Guide | ROS/DDS/socket boundaries and the complete command path |
| [Onboard-only navigation](../ONBOARD_SOCIAL_NAVIGATION.md) | Guide / evidence | Sensor-driven inputs, evaluation-only truth and measured limitations |
| [Event-driven routes](../DYNAMIC_ROUTE_PLANNING.md) | Guide | Replanning events, timing and goal/route execution |
| [Navigation investigation](../NAVIGATION_RELIABILITY_INVESTIGATION.md) | Evidence | Retained corrections, regressions and cohort ledger |
| [Bridge/autonomy map](../../ros2_ws/src/unitree_g1_mujoco_bridge/README.md) | Code / guide | Locate estimators, planners, controllers and diagnostics |
| [Social costmap](../../ros2_ws/src/unitree_social_costmap/README.md) | Code / guide | C++ Nav2 people costs and behavior-tree integration |
| [Social message contracts](../../ros2_ws/src/unitree_social_navigation_msgs/README.md) | Code | Shared person/trajectory interfaces |

## Follow one observation through the source

All links below point to implementation entry points; use the guides above for
their configuration and data-source contracts.

| Stage | Code links |
| --- | --- |
| Receive sensors | [Sensor bridge](../../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/g1_sim_sensors.py) · [Packet decoding](../../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/sensor_packets.py) |
| Estimate motion | [Onboard odometry node](../../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/onboard_lidar_odometry.py) · [Registration implementation](../../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/lidar_odometry.py) |
| Track people | [Dynamic tracker](../../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/dynamic_obstacle_tracker.py) · [Dynamic scan filter](../../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/scan_dynamic_filter.py) |
| Choose motion | [Human-aware planner](../../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/human_aware_motion_planner.py) · [Observed routes](../../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/observed_route.py) · [Candidates](../../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/avoidance_candidates.py) |
| Supervise commands | [Collision supervisor](../../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/human_collision_avoidance.py) · [Motion envelope](../../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/motion_envelope.py) |
| Configure / observe | [Onboard profile](../../ros2_ws/src/unitree_g1_sim_bringup/config/social_navigation_onboard_v1.yaml) · [Run logger](../../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/human_aware_run_logger.py) |

## Connect implementation to evidence

- [Source-contract audit](../../ros2_ws/src/dynamic_obstacle_benchmarks/dynamic_obstacle_benchmarks/source_contract.py)
  checks the onboard/evaluation boundary.
- [Trace inspection](../../scripts/inspect_social_trace.py) and
  [perception evaluation](../../scripts/evaluate_onboard_perception.py) support diagnostics.
- [Evaluation alignment](../../scripts/audit_evaluation_alignment.py) separates
  scoring alignment questions from controller behavior.
- [S2 navigation plan](../PROJECT_PLAN.md#s2--reliable-onboard-navigation--open)
  defines the acceptance work; [latest status](../PROJECT_STATUS.md) identifies assessed evidence.

For framework tutorials and important repositories, use the
[navigation/mapping directory](SIMULATION.md#navigation-and-mapping),
[ROS 2 API references](SIMULATION.md#ros-2-package-apis-and-standards) and
[perception libraries](SIMULATION.md#perception-geometry-and-numerical-libraries).
For alternative algorithms use the [grouped paper archive](RESEARCH.md).

[Back to the library](README.md)
