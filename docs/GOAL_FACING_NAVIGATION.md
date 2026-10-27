# Goal-facing travel and turn-speed investigation

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

Updated 2026-10-05. The onboard profile now turns toward the **observed route**
before normal travel, including ordinary 1–3 m goals. Facing the route rather
than blindly facing the final goal preserves routing around walls and people.

## Active behavior

- `prefer_forward_travel: true` remains enabled.
- `path_reference_holonomic_distance` is **0.75 m**, down from 3.0 m.
- The final approach can move sideways to avoid orbiting a nearby goal.
- A 0.25 m hysteresis band prevents repeated switching near the boundary.
  Moving beyond **1.0 m** restores forward-facing travel. Previously the
  holonomic mode stayed latched until the next goal.
- The route reference pivots beyond its existing 20-degree alignment gate and
  uses bounded forward arcs once sufficiently aligned.
- Person-avoidance and escape candidates retain lateral movement. This is a
  travel preference, not a prohibition on evasive motion.
- Precise manipulation docking and short retreats are separate task modes.
- The later `prefer_safe_route_alignment` experiment retains a fully scored
  pure turn ahead of sideways progress when its forecast stays outside the
  comfort/body reserve. It excludes payload carrying and does not bypass
  collision rejection. See the [follow-up results](COMMAND_HANDOFF_AND_SUPERVISOR.md).

Implementation: `forward_reference.py`, `human_aware_motion_planner.py` and
`unitree_g1_sim_bringup/config/social_navigation_onboard_v1.yaml`.

## Why the yaw limit was not increased yet

Two new six-second, empty-scene, official-policy trials used the same 0.90 rad/s
command cap and 0.90 rad/s² yaw slew, followed by four seconds of stopping:

| Positive yaw command | Mean steady yaw response | In-command net translation | Post-stop net drift | Maximum tilt | Existing validator |
| --- | --- | --- | --- | --- | --- |
| 0.60 rad/s | 0.180 rad/s | 0.351 m | 0.027 m | 0.064 rad | Pass |
| 0.75 rad/s | 0.370 rad/s | 0.640 m | 0.017 m | 0.085 rad | Pass |

Raw fixed-command trial IDs: `20261005T110418Z` (0.60) and
`20261005T110346Z` (0.75), under the workspace's `results/fixed_command/`.
These are one trial each, not repeatability or hardware acceptance evidence.

The faster command produced more rotation **and more translational drift**.
The existing validator checks post-stop drift but does not bound in-turn
translation. The current planar rollout has no calibrated pivot-induced
translation model. Increasing yaw everywhere would therefore increase a motion
component the avoidance model does not yet represent accurately.

Active counterclockwise/clockwise reference caps remain **0.60/0.90 rad/s**.
These are command limits, not achieved angular velocities. Before changing them,
repeat both directions with arm/payload conditions, measure in-turn swept
displacement and stopping, then update planner and supervisor response envelopes
together. Loaded cycle-01 behavior needs separate calibration.

## Native evidence

The four-person courtyard smoke trial with the new facing preference completed
**2/3 goals and reported one contact**. It failed strict acceptance. Its trace
shows pivot references in normal navigation and retained lateral evasions in
human-caution states. This demonstrates the behavioral wiring, not reliable
crowd navigation or a causal improvement over earlier variants.

Portable record: [goal-facing diagnostic](results/avoidance_goal_facing_20261005.json).

The [transport ledger](results/transport_lifecycle_20261005.json) contains changing
implementations, including passes and failures, and must not be reported as a
frozen reliability cohort.
