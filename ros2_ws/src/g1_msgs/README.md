# g1_msgs

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

[Workspace map](../README.md) · [Locomotion](../g1_locomotion/README.md) · [Manipulation](../g1_manipulation/README.md)

The stack's own interfaces: what `g1_locomotion` serves for the base approach, and what
manipulation and mission orchestration serve. The package defines six actions
and the simulation-specific `GraspObject` service.

Run the following inside the sourced ROS container at `/workspace/ros2_ws`:

```bash
colcon build --symlink-install --packages-select g1_msgs
```

## Interfaces

| Action | Goal |
|---|---|
| `ApproachObject` | `object_id`, `arm`, `working_yaw`, `use_current_heading`, `timeout_s` |
| `Retreat` | `distance_m`, `timeout_s` |
| `Pick` | `object_id`, `arm` |
| `Place` | `surface_object_id` or `pose`, `arm` |
| `SetArmPosture` | `group`, `named_target` |
| `TransportObject` | pickup/delivery staging poses, object, surface, arm, and timeouts |

All six are actions rather than services because each runs for seconds and has to be cancellable.

### Object-selectable simulation grasp

[`srv/GraspObject.srv`](srv/GraspObject.srv) accepts `arm` and `object_id` and
returns `success` and `message`. It selects the simulated grasp target by ID;
it does not supply a privileged object pose. The bridge exposes it at
`/g1_sim_manipulation_io/grasp_object`. An acknowledged simulated attachment is
different from measured finger-force closure.

## Maintaining the contracts

The [`action/`](action/) and [`srv/`](srv/) definitions are authoritative for
goal, result and feedback fields. Rebuild this package and every consumer after
changing a definition. Coordinate cancellation, timeout and uncertain-object
semantics with the action servers and transport lifecycle tests. See the
[forward plan](../../../docs/PROJECT_PLAN.md) for interface-consolidation work.

## Attribution

Ported from Grove-G1 commit `97bfb9ba13569a593b5e3ecfc5ed1d4208b2b681` under the
BSD 3-Clause license. See [NOTICE](NOTICE) for attribution and license terms.
