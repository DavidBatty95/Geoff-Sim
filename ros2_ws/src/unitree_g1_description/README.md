# G1 robot description

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

[Workspace map](../README.md) · [MoveIt configuration](../g1_moveit_config/README.md)

This package supplies the ROS G1 robot description and meshes used for TF,
visualization, kinematics and arm planning.

- [`urdf/g1_29dof_dex3.urdf`](urdf/g1_29dof_dex3.urdf) is the default body-and-hand
  description for MoveIt, TF and transport. Each hand has seven independent joints.
  The earlier BrainCo description is retained as a legacy/source body reference.
- [`meshes/`](meshes/) contains the referenced visual/collision assets.
- [`package.xml`](package.xml) identifies build dependencies and package metadata.

Joint names, limits, axes and frame origins are integration contracts. Changes
must be reconciled with bridge joint order, MoveIt groups/controller mappings,
arm-kinematics tests and any generated MuJoCo hand model. The simulator's MJCF
assets live in `unitree_mujoco`; a URDF edit does not automatically update them.

The [hardware fidelity audit](../../../docs/HARDWARE_FIDELITY.md) verifies the 29
body-joint limits and actuator mapping and records the remaining torso/waist
revision differences. The native build generates articulated Dex3 scenes from
pinned official Unitree hand assets; raw sensor TF is derived from measured IMU and
encoders. Model mass and sensor bandwidth are documented rather than presented
as measurements from a physical robot.

The [Dex3 migration report](../../../docs/DEX3_CONTACT_MANIPULATION_20261020.md)
describes physical contact grasping, measured evidence and remaining mobile-delivery gaps.

The [development checks](../../../docs/DEVELOPMENT.md) cover description parsing
and related package tests. Preserve upstream attribution and the
[license inventory](../../../LICENSES.md) when changing assets.
