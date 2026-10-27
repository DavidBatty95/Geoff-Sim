# g1_locomotion

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

[Workspace map](../README.md) · [Action interfaces](../g1_msgs/README.md) · [Transport orchestration](../g1_task_orchestration/README.md)

The step between navigation and manipulation. Nav2 parks the robot near an object and this package
closes the final gap into the arm's reach window.

One C++20 node, `g1_base_approach`, serves `~/approach_object` and `~/retreat`, consumes
`vision_msgs/msg/Detection3DArray` on `/objects`, and publishes `geometry_msgs/msg/Twist`.
`cmd_vel_topic` defaults to `/cmd_vel`, preserving this workspace's `/cmd_vel` to human-aware to
`/cmd_vel_safe` command pipeline.

Configuration is in `config/g1_base_approach.yaml`. The pure planner uses proportional commands
with measured linear speed floors, caps, simultaneous forward/lateral/yaw control, and a
recoverable reverse when the object is just past the reach window.

## Build and verify

Run inside the sourced ROS container at `/workspace/ros2_ws`:

```bash
colcon build --symlink-install --packages-select g1_msgs g1_locomotion
colcon test --packages-select g1_msgs g1_locomotion
```

## Integration contract

This is fine base positioning, not the walking policy. The policy and simulator
must already be running, and its velocity output must traverse the same
human-aware planner and collision supervisor as other navigation commands.
The caller supplies an object ID and approach intent; detections and TF must
describe that target in the expected frame and time window.

The newer sensor-driven transport demo has a separate docking implementation.
Consolidation of the two paths is tracked in the
[mission-recovery plan](../../../docs/PROJECT_PLAN.md). Changing approach limits
requires response and stopping evidence, not just a reachable geometric target.

## Folder guide

- [`config/`](config/) — approach and retreat parameters.
- [`src/`](src/) / [`include/`](include/) — C++ action server and planning helpers.
- [`test/`](test/) — approach/retreat geometry and behavior checks.

## Attribution

Ported from Grove-G1 commit `97bfb9ba13569a593b5e3ecfc5ed1d4208b2b681` under the
BSD 3-Clause license. See [NOTICE](NOTICE) for attribution and license terms.
