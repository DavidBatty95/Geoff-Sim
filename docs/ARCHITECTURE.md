# Geoff-Sim Architecture

<!-- geoff-nav:start -->
<p align="center">
  <a href="../README.md"><img src="media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../README.md#quick-start"><img src="media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="DEMO_GUIDE.md"><img src="media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="README.md"><img src="media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="REPOSITORY_GUIDE.md"><img src="media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="PROJECT_STATUS.md"><img src="media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="PROJECT_PLAN.md"><img src="media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="RESULTS_REVIEW.md"><img src="media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="literature_review/README.md"><img src="media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="DEVELOPMENT.md"><img src="media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

[Repository guide](REPOSITORY_GUIDE.md) · [ROS package map](../ros2_ws/src/README.md) · [Project plan](PROJECT_PLAN.md)

![Animated system architecture](media/architecture.svg)

The default runtime is a simulation-first, closed-loop ROS 2 locomotion and social-navigation system. `g1_sim.launch.py` starts MuJoCo first, the bridge and autonomy nodes after 3 seconds, Nav2 after 7 seconds, and optional goal clients after 11 seconds.

## Runtime Boundaries

- **ROS 2:** navigation, perception, policy inference, safety, visualization, and diagnostics.
- **Unitree DDS:** `rt/lowcmd`, `rt/lowstate`, and `rt/sportmodestate` cross the low-level robot boundary on domain 42 by default.
- **Unix datagrams:** MuJoCo raycast LiDAR, camera, native sensor-site IMU and pedestrian packets reach `g1_sim_sensors` through `/tmp` sockets.
- **MuJoCo:** owns physics, actuator PD, simulated sensors, and animated actors.

ROS `/lowcmd` and `/lowstate` map to the corresponding DDS `rt/` names; the
extra ROS `/rt/*` aliases are opt-in. Commands include the vendor-layout CRC and
observed machine mode. Sensor IMUs carry measured simulated specific force and
angular velocity, while raw sensor TF comes from IMU/encoders. See
[hardware fidelity](HARDWARE_FIDELITY.md) for exact message types, the optional
BrainCo Stark adapter and the limits of sensor/model equivalence.
- **External packages:** Nav2, SLAM Toolbox, RViz, robot state publisher, ONNX Runtime or PyTorch, and the locomotion policy artifact.

## Architecture Evaluation

### Strengths

- The command path is layered: social trajectory selection, a final collision supervisor, policy inference, bridge validation, and a simulator watchdog each have distinct responsibilities.
- The simulator uses the same Unitree low-level DDS contract as the intended robot boundary, reducing integration differences at the bridge.
- The recommended onboard demo explicitly enables `onboard_only`: measured LiDAR, relative IMU heading and encoder transforms feed scan odometry and online SLAM. Simulator truth is isolated to diagnostics, scoring and explicit oracle ablations.
- Current tracked people and time-indexed predictions remain separate. The social planner evaluates trajectories; the global PeopleLayer adds current lethal footprints and short, soft future costs.
- Expensive simulated sensors use latest-only workers and private MuJoCo snapshots, keeping them off the physics loop.

### Risks And Constraints

- The bridge publishes `LowCmd` directly; there is no `ros2_control` hardware abstraction, controller manager, or certified independent safety layer.
- Command-source exclusivity is operational rather than enforced. Teleoperation, Nav2, explorers, and goal clients can all publish `/cmd_vel` if launched together.
- The large bringup launch file owns many concerns and timer-based startup ordering. Readiness checks or lifecycle orchestration would make startup and recovery more deterministic.
- ROS 2 DDS and three custom Unix datagram channels create two transport and observability models. Sensor packet versioning, drop metrics, and timestamps are therefore safety-relevant.
- The simulator does not provide synchronized `/clock` or deterministic ROS services for reset, actor spawn, and actor trajectories.
- Fall handling clears localization and costmaps, but there is no integrated get-up state machine.
- The physical G1 is an architectural target, not a validated deployment. Hardware requires independent emergency stop, current and torque limits, latency budgets, and controlled test-stand validation.

## Authority And Data Flow

1. Teleoperation, Nav2, or one goal/exploration mode produces `/cmd_vel`.
2. The human-aware planner combines the command with scans, tracked people, and predicted trajectories.
3. The collision supervisor fails closed on imminent collision or stale safety inputs and publishes `/cmd_vel_safe`.
4. The locomotion policy combines the safe command with `LowState` and odometry to produce 29-joint targets.
5. `g1_mujoco_bridge` validates and converts targets into Unitree `LowCmd` DDS messages.
6. MuJoCo applies actuator PD and feed-forward torque, then returns state over DDS and sensor packets over Unix datagrams.

Optional benchmark, logging, camera, detector, and goal-generation nodes are intentionally shown as sidecars or annotations rather than part of the primary command chain.

## Onboard and experiment boundaries

`/evaluation/odom` is scoring-only. Scan odometry owns navigation `/odom` and the
odometry TF; online SLAM owns `/map`. The source-contract audit checks forbidden
subscriptions and ownership. Goals are sampled from observed reachable free
space. Event-driven route requests are latched through asynchronous planning.

Generated avoidance worlds provide static geometry and bounds to the simulator's
independent actor planner, never a prior map to onboard navigation. The results
GUI is a separate read-only local HTTP process; it consumes published datasets
without connecting to ROS or commanding the robot.
