# G1 MuJoCo bridge and autonomy

<!-- geoff-nav:start -->
<p align="center">
  <a href="../../../README.md"><img src="../../../docs/media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../../../README.md#quick-start"><img src="../../../docs/media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="../../../docs/DEMO_GUIDE.md"><img src="../../../docs/media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="../../../docs/README.md"><img src="../../../docs/media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="../../../docs/REPOSITORY_GUIDE.md"><img src="../../../docs/media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="../../../docs/PROJECT_STATUS.md"><img src="../../../docs/media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="../../../docs/PROJECT_PLAN.md"><img src="../../../docs/media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="../../../docs/RESULTS_REVIEW.md"><img src="../../../docs/media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="../../../docs/literature_review/README.md"><img src="../../../docs/media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="../../../docs/DEVELOPMENT.md"><img src="../../../docs/media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

[Workspace map](../README.md) · [Architecture](../../../docs/ARCHITECTURE.md) · [Source boundaries](../../../docs/ONBOARD_SOCIAL_NAVIGATION.md)

This Python package connects simulated G1 state and actuation to the onboard
navigation pipeline. It also contains arm-command composition and the newer
experimental sensor-driven transport modules.

## Find the implementation

All module names below are inside `unitree_g1_mujoco_bridge/`.

| Area | Main modules | Responsibility |
| --- | --- | --- |
| Robot boundary | `g1_mujoco_bridge.py`, `g1_constants.py` | Joint state, validated low-level body commands and hand interfaces |
| Sensors | `g1_sim_sensors.py`, `sensor_packets.py` | DDS/socket inputs, ROS sensor messages and scoring-only truth output |
| Localization | `onboard_lidar_odometry.py`, `lidar_odometry.py` | Scan-derived navigation odometry |
| Person perception | `dynamic_obstacle_tracker.py`, `scan_dynamic_filter.py` | Observed people and dynamic scan handling |
| Planning | `human_aware_motion_planner.py`, `observed_route.py`, `avoidance_candidates.py` | Route following, candidate motion and recovery decisions |
| Supervision | `human_collision_avoidance.py`, `motion_envelope.py`, `policy_safety.py` | Final command checks and executable-motion constraints |
| Locomotion | `g1_policy_cmd_vel_controller.py`, `unitree_rl_lab_policy.py` | ONNX policy input/output and joint targets |
| Manipulation | `arm_command_compositor.py`, `sim_manipulation_io.py`, `arm_kinematics.py` | Arm authority, simulated grasp I/O and measured kinematics |
| Transport | `transport_demo.py`, `transport_perception.py`, `transport_actions.py`, `transport_health.py` | RGB-D mission, action lifecycle and evaluation coverage |
| Diagnostics | `human_aware_run_logger.py`, `evaluation_alignment.py`, `policy_timing.py`, `runtime_parameters.py` | Decision/timing traces, scoring alignment and resolved runtime configuration |

## Runtime contract

The [hardware fidelity guide](../../../docs/HARDWARE_FIDELITY.md) defines the
current contract. `/lowcmd` now uses the observed machine mode and Unitree HG CRC;
ROS `rt/` aliases are opt-in. Native camera/Livox IMUs are acquisition-stamped
measurements with unavailable orientation marked explicitly. Raw sensor TF uses
measured IMU/encoders, and odometry supports padded/organized hardware point clouds.
The default hand interface is now seven-channel Dex3 radians on
`/dex3/{left,right}/cmd`, with measured states on `/lf/dex3/{left,right}/state`.
The contact-only grasp backend confirms opposing contacts and loaded joints,
and verifies release from measured open fingers and unloaded contacts.
[Dex3 implementation and evidence](../../../docs/DEX3_CONTACT_MANIPULATION_20261020.md).

The legacy `brainco_stark_adapter` optionally maps the normalized six-channel application
topics to the vendor Stark driver; it requires the vendor message package and
normalized SDK units. Neither interface tests nor simulation grasp welds qualify
physical deployment.

The onboard path uses measured LiDAR, IMU and encoders for navigation. Simulator
truth is reserved for evaluation and explicitly selected oracle cases. The
command path runs from navigation intent through social planning and collision
supervision, then policy inference and the DDS bridge. See
[architecture](../../../docs/ARCHITECTURE.md) for topics and authority boundaries.

Bringup composes these nodes; launching an individual module is not equivalent
to launching a complete demo. Installed node entry points are listed in
[`setup.py`](setup.py). The experimental transport module is launched by its
dedicated script rather than a `transport_demo` console entry point.

## Use and verification

From the repository root:

```bash
./geoff demo plaza --duration 120
./scripts/run_demo_suite.sh --case onboard-waypoints --goals 3 --people 4 --seed 42 --headless
```

Use the [payload guide](../../../docs/PAYLOAD_POLICY_TRAINING.md) for transport
with an explicit trained policy. Tests are in [`test/`](test/); the
[development guide](../../../docs/DEVELOPMENT.md) records the source-test and
build environment. Changes to planners should be checked with regression tests
and scored trials; visual success alone does not establish acceptance.

Transport and onboard matrix runs now capture resolved parameter values and check
shared motion settings. `/g1/policy_timing` identifies inference results reaching
the joint-target publisher; it is not a motor acknowledgement. `/g1/policy_diagnostics`
reports worker, stop-invalidation and late-result counters. The logger retains
both these topics and filtered `/g1/policy_cmd_vel`, with their distinct semantics.
See [S0/R1–R3 evidence](../../../docs/PHASE_PROGRESS_20261005.md) for commands,
measurements and the remaining native gates.

The onboard profile requires an atomic planner command envelope. Goal changes
(including heading-only updates), cancellation and publisher restarts cannot
refresh an old request. Supervisor diagnostics expose the accepted epoch/sequence
for offline lineage checks. Prepared predictions and batched static queries
reduce repeated work without claiming complete collision-free behavior. See the
[handoff/supervisor report](../../../docs/COMMAND_HANDOFF_AND_SUPERVISOR.md).

`escape_search.py` now performs alternative search on immutable snapshots in a
single-flight worker; `prediction_risk.py` contains the ROS-independent risk
queries. The control callback revalidates every proposal against current inputs
and rejects expired or superseded epochs. The experimental onboard configuration
uses 50 Hz checks, with measured scheduling tails still above 20 ms. See the
[async checkpoint](../../../docs/ASYNC_ESCAPE_SUPERVISOR.md) for tests and native outcomes.

## Current direction

The published baseline includes navigation reliability corrections, payload
transport, action termination and evaluation-health helpers. Full navigation and
social-transport acceptance remain open. Follow
[project status](../../../docs/PROJECT_STATUS.md) and the
[S1/S2/S4 work packages](../../../docs/PROJECT_PLAN.md).
