# Licensing inventory

<!-- geoff-nav:start -->
<p align="center">
  <a href="README.md"><img src="docs/media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="README.md#quick-start"><img src="docs/media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="docs/DEMO_GUIDE.md"><img src="docs/media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="docs/INDEX.md"><img src="docs/media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="docs/REPOSITORY_GUIDE.md"><img src="docs/media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="docs/PROJECT_STATUS.md"><img src="docs/media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="docs/PROJECT_PLAN.md"><img src="docs/media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="docs/RESULTS_REVIEW.md"><img src="docs/media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="docs/literature_review/README.md"><img src="docs/media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="docs/DEVELOPMENT.md"><img src="docs/media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

The departing maintainer requested **preservation of existing licenses**, not
assignment of a new blanket root license. There is intentionally no new root
license grant for previously unlicensed original scripts or documentation.
The incoming team should confirm that policy before broader redistribution.

| Material | Existing declaration / authority |
| --- | --- |
| `unitree_g1_mujoco_bridge`, `unitree_g1_sim_bringup`, `unitree_g1_description`, `dynamic_obstacle_benchmarks`, social messages/costmap packages | Apache-2.0 in their `package.xml` files |
| `g1_locomotion`, `g1_msgs`, manipulation/MoveIt/task-orchestration packages | BSD-3-Clause in their `package.xml` files |
| MuJoCo, Unitree repositories and MJLab | Their own source/submodule notices and licenses |
| Dex3 hand geometry and source descriptions | Pinned Unitree ROS source; upstream license and per-file hashes retained in `ros2_ws/src/unitree_g1_description/dex3_sources/` |
| Grove-G1 integration | Its integration/upstream notices; optional sources are not bundled by the setup |
| Pedestrian meshes/textures and generated derivatives | Bundled through the simulator fork; asset redistribution provenance is not fully documented in this root repository and needs owner confirmation |
| Downloaded policies and optional training checkpoints | Their upstream artifact terms; checksums establish identity, not redistribution permission |

Keep upstream copyright/license headers when modifying files. The checked-in
source patches do not replace their underlying licenses. Generated natural-walk
meshes retain the licensing obligations of their input assets.
