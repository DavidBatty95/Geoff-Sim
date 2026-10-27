# g1_manipulation

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

[Workspace map](../README.md) · [MoveIt configuration](../g1_moveit_config/README.md) · [Project plan](../../../docs/PROJECT_PLAN.md)

Manipulation gate 3 provides real ROS 2 action servers for `g1_msgs/action/Pick`,
`Place`, and `SetArmPosture`. Motion is planned and then executed with MoveIt's
`MoveGroupInterface` against the local `g1_moveit_config` groups.

This package is explicitly simulation-first. Grasp and release use the MuJoCo
bridge's object-selectable `GraspObject` service by default, with `Trigger`
services for release and an optional legacy grasp path. It does not acquire controller
authority, start `move_group`, or start a controller. The existing
`/arm_trajectory_controller/follow_joint_trajectory` controller must already have
authority before execution.

This is the MoveIt/action integration path. The newer
[sensor-driven transport demo](../../../docs/PAYLOAD_POLICY_TRAINING.md) uses
separate bridge modules; its fixture results do not validate this action server
as a complete mobile manipulation system.

## Interfaces

Actions are `/g1_manipulation_server/pick`, `/place`, and `/set_arm_posture`.
Pick and place accept only `left` or `right`; named posture accepts only
`left_arm`, `right_arm`, or `both_arms`. One goal may run globally across all
three servers.

Object data comes from `/objects` as `vision_msgs/Detection3DArray`. The server
matches `results.front().hypothesis.class_id`, reads
`results.front().pose.pose`, requires a fresh array stamp, transforms poses to
the MoveIt planning frame, and uses a padded bounding-box collision object.

Pick phases are locate, pregrasp, approach, grasp service, attach to the
configured tip, and lift. Place requires an object attached to that tip, prefers
`surface_object_id` over the goal pose, places the object's center at surface top
plus half its own height, lowers, releases, detaches, retreats, and checks a fresh
final detection when one is available. Gripper service failures and timeouts
abort the action.

There are no MoveIt hand groups, no allowed-collision-matrix edits, and no
octomap collision exemptions. The target world collision object is removed only
for the final pick approach and then represented as an attached body.

## Running

Start MoveIt separately, ensure the arm trajectory controller has authority, and
start the simulation manipulation I/O before this server:

Run each launch in a sourced ROS container shell. The
[workspace README](../README.md#build-and-check) describes the environment.

```bash
ros2 launch g1_moveit_config move_group.launch.py
ros2 launch g1_manipulation manipulation.launch.py
```

The launch file starts only `g1_manipulation_server`; it supplies the same robot
description, SRDF, kinematics, joint limits, and OMPL settings used by the local
MoveIt package so `MoveGroupInterface` can initialize.

Configuration is in `config/manipulation.yaml`. `tip_to_object_offset` is the tip
position relative to the object, expressed in the planning frame, not rotated by
the object. `grasp_rpy` describes the right-side orientation and its roll is
mirrored for the left side.

`use_object_grasp_service` defaults to true in the server. The object-selectable
service takes the requested object ID; setting it false uses the configured
left/right legacy targets. `/objects` must contain the intended perception source;
the onboard bringup routes simulator object truth to `/evaluation/objects`.

## Folder guide

| Path | Purpose |
| --- | --- |
| [`src/`](src/) / [`include/`](include/) | Action server and pure geometry helpers |
| [`config/`](config/) | Manipulation parameters |
| [`launch/`](launch/) | Server launch and MoveIt description inputs |
| [`test/`](test/) | Geometry and input-contract regressions |

## Verification

Pure geometry is isolated from ROS and MoveIt in `geometry.hpp`/`geometry.cpp`.
The gtests cover arm resolution, mirrored orientation, tip offset and vertical
approach/lift geometry, placement height, detection lookup, and invalid inputs.

```bash
colcon test --packages-select g1_manipulation
```

See [NOTICE](NOTICE) for the pinned Grove G1 source attribution. Recovery,
collision coverage and orchestration consolidation are tracked in
[S1–S5 of the project plan](../../../docs/PROJECT_PLAN.md).
