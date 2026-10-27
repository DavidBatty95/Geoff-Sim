# Optional integration profiles

<!-- geoff-nav:start -->
<p align="center">
  <a href="../README.md"><img src="../docs/media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../README.md#quick-start"><img src="../docs/media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="../docs/DEMO_GUIDE.md"><img src="../docs/media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="../docs/README.md"><img src="../docs/media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="../docs/REPOSITORY_GUIDE.md"><img src="../docs/media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="../docs/PROJECT_STATUS.md"><img src="../docs/media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="../docs/PROJECT_PLAN.md"><img src="../docs/media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="../docs/RESULTS_REVIEW.md"><img src="../docs/media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="../docs/literature_review/README.md"><img src="../docs/media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="../docs/DEVELOPMENT.md"><img src="../docs/media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

Related: [ROS workspace packages](../ros2_ws/src/README.md).

Integration profiles document external systems that can be explored alongside
Geoff-Sim while retaining their own dependencies and runtime assumptions.

| Profile | Environment | Relationship to the default stack |
| --- | --- | --- |
| [Grove-G1](grove_g1/README.md) | ROS 2 Jazzy / Ubuntu 24.04 | Optional complete external checkout; separate from the Humble workspace |

The local Humble manipulation packages include attributed ports of selected
interfaces and components. Running the entire Grove-G1 stack is a different
workflow and requires the profile's explicit ownership and DDS/TF boundaries.

When adding a profile, document the pinned source, setup commands, attribution,
runtime ownership, compatibility assumptions and a source-profile check. Link
the profile here and explain its relevance to the [project plan](../docs/PROJECT_PLAN.md).
