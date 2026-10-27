# Grove-G1 source profile

<!-- geoff-nav:start -->
<p align="center">
  <a href="../../README.md"><img src="../../docs/media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../../README.md#quick-start"><img src="../../docs/media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="../../docs/DEMO_GUIDE.md"><img src="../../docs/media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="../../docs/README.md"><img src="../../docs/media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="../../docs/REPOSITORY_GUIDE.md"><img src="../../docs/media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="../../docs/PROJECT_STATUS.md"><img src="../../docs/media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="../../docs/PROJECT_PLAN.md"><img src="../../docs/media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="../../docs/RESULTS_REVIEW.md"><img src="../../docs/media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="../../docs/literature_review/README.md"><img src="../../docs/media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="../../docs/DEVELOPMENT.md"><img src="../../docs/media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

[Integration overview](../README.md) · [Project home](../../README.md) · [Workspace map](../../ros2_ws/src/README.md)

This optional profile pins the complete
[`Adyansh04/grove-g1`](https://github.com/Adyansh04/grove-g1) repository under
`vendor/grove-g1`. Grove-G1 is a self-contained ROS 2 Jazzy/Ubuntu 24.04 stack with its own
workspace, dependencies, patches, Docker Compose file, and lifecycle scripts. It is not a package
dependency of this repository's ROS 2 Humble workspace and must not be copied into `ros2_ws/src` or
included in the Humble `colcon` build.

## Fetch and enter its container

On the host, from this repository root:

```bash
./scripts/setup_grove_g1.sh
cd vendor/grove-g1
cp .env.example .env
./scripts/import-externals.sh
./scripts/manage.sh start
./scripts/manage.sh exec
```

The first command verifies the exact Grove-G1 commit. The remaining commands are Grove-G1's own
documented dependency and Jazzy container workflow; review `vendor/grove-g1/README.md` before
building or running a simulation. Hardware use is not part of this integration profile.

## Runtime boundary

Stop the default stack before running Grove-G1 against the same simulator, robot, or DDS domain.
The stacks are alternative owners of the same interfaces, not composable overlays:

- Both can publish `rt/lowcmd`. Grove-G1's controller owns balance and all 29 body motors, so a
  second command publisher can cause conflicting actuator commands and a fall.
- Both provide overlapping world/odometry/robot TF chains. Concurrent authorities make TF lookup
  nondeterministic and invalidate navigation and localization.
- Both publish simulated LiDAR, IMU, and related sensor streams. Duplicate topic/frame publishers
  can mix incompatible samples and corrupt perception and state estimation.

Process isolation alone is insufficient when host networking places both containers on the same
DDS graph. Use only one complete stack at a time for a given simulator or robot.

## Relationship to Geoff-Sim development

Selected local `g1_msgs`, locomotion and manipulation components carry their own
pinned Grove-G1 attribution. That local port is distinct from fetching and running
the complete optional Jazzy profile. The [workspace map](../../ros2_ws/src/README.md)
identifies the local Humble packages; the [project plan](../../docs/PROJECT_PLAN.md)
tracks their integration with the newer sensor-driven transport workflow.

The optional profile is verified by
`python3 integrations/grove_g1/test/test_profile.py` from the repository root.
This checks the source-profile contract, not a complete external-stack deployment.

The [October source review](../../docs/GROVE_G1_REVIEW.md) inspects a newer Grove
revision than this profile's pin. Its recommendations feed the
[R1–R6 response workstream](../../docs/SOCIAL_LOCOMANIPULATION_ROADMAP.md#reactive-simulation-workstream-s1-s2-s4-and-s5);
they do not automatically update the checkout or establish benchmark compatibility.

## Attribution and license

Source: `Adyansh04/grove-g1`, pinned at
`97bfb9ba13569a593b5e3ecfc5ed1d4208b2b681`. Grove-G1 is Copyright (c) 2026 Adyansh Gupta and is
distributed under the BSD 3-Clause License; its checked-out `LICENSE` file is authoritative.
Grove-G1 also fetches and vendors third-party components that retain their own licenses and notices.
