# Locomotion and manipulation links

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

[← Resource library](README.md) · Related: [Navigation](NAVIGATION.md) · [Experiment records](EXPERIMENTS.md)

[![Command flow from intent to actuation — open the architecture guide](../media/command-safety.svg)](../ARCHITECTURE.md#authority-and-data-flow)

## Policies, response and training

| Resource | Kind | Why open it? |
| --- | --- | --- |
| [Training and calibration](../TRAINING_AND_CALIBRATION.md) | Guide | Policy helpers, response datasets and calibration workflow |
| [Payload cycle report](../PAYLOAD_POLICY_TRAINING.md) | Evidence / guide | Initialization, training, native attempts and explicit reproduction checkpoint |
| [Official-policy downloader](../../scripts/download_unitree_rl_lab_policy.sh) | Code | Policy origin and expected SHA-256 |
| [Policy controller](../../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/g1_policy_cmd_vel_controller.py) | Code | Observation/inference and joint-target publication |
| [Policy adapter](../../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/unitree_rl_lab_policy.py) | Code | Unitree policy contract |
| [Payload trainer](../../scripts/g1_payload_training.py) | Code | Experimental training and evaluation implementation |
| [Fixed-command measurements](../../scripts/measure_g1_fixed_command.py) | Code | Motion onset, tracking and stopping diagnostics |
| [Response validation](../../scripts/validate_fixed_command_result.py) | Code | Explicit acceptance versus mixed-command diagnostics |

## Robot, arms and action interfaces

| Resource | Kind | Why open it? |
| --- | --- | --- |
| [Robot description](../../ros2_ws/src/unitree_g1_description/README.md) | Code / guide | URDF, meshes and joint/frame contracts |
| [Action and grasp interfaces](../../ros2_ws/src/g1_msgs/README.md) | Code | Approach, retreat, pick, place, posture, transport and `GraspObject` |
| [Base approach](../../ros2_ws/src/g1_locomotion/README.md) | Code / guide | Final positioning between navigation and manipulation |
| [MoveIt configuration](../../ros2_ws/src/g1_moveit_config/README.md) | Code / guide | Stationary arm groups, planning and controller mapping |
| [Manipulation server](../../ros2_ws/src/g1_manipulation/README.md) | Code / guide | MoveIt-based pick/place/posture actions |
| [Action orchestration](../../ros2_ws/src/g1_task_orchestration/README.md) | Code / guide | Earlier action-based transport path and recovery latch |
| [Arm composition](../../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/arm_command_compositor.py) | Code | Arm authority and locomotion/manipulation target composition |
| [Arm kinematics](../../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/arm_kinematics.py) | Code | Measured joint geometry and inverse-kinematics helpers |

## Sensor-driven transport and hand experiments

- [Transport launcher](../../scripts/run_transport_demo.py),
  [mission module](../../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/transport_demo.py)
  and [RGB-D perception](../../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/transport_perception.py)
  form the newer experimental transport path.
- [Transport fixtures](../../scripts/transport_worlds.py) define the physical
  test layouts; [provenance capture](../../scripts/transport_evidence.py) identifies runs.
- [BrainCo model generation](../../scripts/brainco_hand_model.py) and the
  [contact-grasp bench](../../scripts/measure_brainco_contact_grasp.py) support isolated hand research.
- [Task roadmap](../SOCIAL_LOCOMANIPULATION_ROADMAP.md) and
  [S1–S7 deliverables](../PROJECT_PLAN.md) connect component work to native acceptance.

The MoveIt/action path and newer sensor-driven path are distinct. Magnetic
attachment fixtures, contact-only hand benches and hardware grasping have different
evidence scopes. Use the [payload report](../PAYLOAD_POLICY_TRAINING.md) and
[Grove implementation review](../GROVE_G1_REVIEW.md) when comparing them.

[Back to the library](README.md) · [MoveIt/control tutorials](SIMULATION.md#manipulation-and-control)
· [Training and inference libraries](SIMULATION.md#learning-and-policy-deployment)
