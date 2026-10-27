# Social-navigation interfaces

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

[Workspace map](../README.md) · [Bridge and autonomy](../unitree_g1_mujoco_bridge/README.md) · [Social costmap](../unitree_social_costmap/README.md)

This package defines the ROS messages shared by person tracking, prediction,
planning and navigation visualization. The authoritative field definitions are
in [`msg/`](msg/); dependencies and interface generation are specified in
[`CMakeLists.txt`](CMakeLists.txt) and [`package.xml`](package.xml).

Treat frame IDs, timestamps, identity and prediction semantics as shared
contracts. When changing a message, rebuild producers and consumers, update
source-contract tests, and check the corresponding RViz displays and benchmark
interpretation. Message compatibility alone does not establish observation
freshness or correct frame alignment.

See [onboard source boundaries](../../../docs/ONBOARD_SOCIAL_NAVIGATION.md) for
which observations may reach navigation, and [development](../../../docs/DEVELOPMENT.md)
for the supported build environment.
