# Event-driven route planning

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

`onboard-waypoints` now enables `nav2_dynamic_replanning:=true`. Rebuild once to
install the new C++ behavior-tree plugin:

```bash
./scripts/run_demo_suite.sh --case onboard-waypoints --goals 4 --headless --seed 43
```

Subsequent runs can add `--fast-start`. This feature uses the online map and
observed tracks; it does not read pedestrian destinations, simulator poses, or
the scene-derived map.

## When routes change

`dynamic_route_monitor` checks the active mission at 20 Hz and requests a new
global route for:

- A missing route or a newly assigned goal.
- A newly observed obstruction on the remaining route, or its clearance.
- A person entering/leaving the route corridor, including an observed-velocity
  crossing predicted within 1.5 seconds.
- Meaningful motion changes near the route: 0.20 m position change or 0.25 m/s
  velocity change since the last route update.
- The robot drifting more than 0.45 m from its route during local avoidance.
- Periodic refresh, normally every 0.5 seconds, to discover shorter alternatives.

Requests are limited to one per 0.20 seconds (up to 5 Hz), matching the global
costmap update cadence. The monitor checks up to 6 m of the route ahead; an
obstacle on a segment already passed does not itself trigger a blockage event.
A stale tracking stream is not treated as a freshly observed empty corridor.
Individual lost tracks still expire according to the bounded observation-age policy.

New Nav2 goals are also detected directly by the behavior tree, including small
position changes and orientation changes. A 0.75-second fallback refresh works
even if the event monitor is unavailable.

## Route handoff and recovery

The new tree is `config/nav2_dynamic_replanning.xml`. `PipelineSequence` keeps
`FollowPath` running while `ComputePathToPose` calculates a replacement. The
monitor requests replanning, rather than repeatedly cancelling and resending
navigation goals.

The `ReplanRequested` C++ condition latches an in-flight request until the
blackboard path changes. Requests arriving during that computation are retained
for a subsequent refresh. The social controller rejects out-of-order route
messages and routes belonging to a different goal.

Planning failures use non-blocking 200/250 ms retry delays rather than the stock
spin/backup/five-second-wait recovery sequence. These are BehaviorTree.CPP
millisecond delays: Humble's `Wait` action only accepts integer seconds. Old
observed maps are not erased merely because a route is temporarily blocked.

Smac's search-time budget is 150 ms, with a 0.25 m endpoint tolerance. Costmap
availability, action transport, recovery and genuinely unreachable routes can
take longer; this is not an end-to-end route delivery deadline.

## Short-horizon people costs

The global `PeopleLayer` keeps confirmed current body footprints lethal and adds
**soft, decaying** costs along 1.2 seconds of constant-velocity prediction. These
costs encourage earlier detours without turning every possible future position
into an impassable wall. Very recent lost tracks contribute soft costs briefly.

Old current and predicted footprints are cleared when a person moves, turns,
disappears or expires. Clearing preserves underlying static obstacles. Forecasts
never change unknown map cells into known free space. The local response-aware
planner and final collision supervisor still check executable motion.

## Configuration and diagnostics

- Launch switch: `nav2_dynamic_replanning` (defaults to `onboard_only`).
- Monitor parameters: `minimum_replan_interval`, `maximum_replan_interval`,
  `prediction_horizon`, `off_route_distance`, `robot_radius`, `people_timeout`,
  and `map_timeout`.
- BT timing ports: `minimum_interval`, `fallback_interval` in the XML tree.
- Global people-layer parameters: `prediction_horizon`, `prediction_step`,
  `prediction_cost`; a zero horizon preserves current-footprints-only behavior.
- Request topic: `/navigation/replan_request`.
- Diagnostic topic: `/navigation/replanning_status`.

The event monitor follows `/benchmark/current_goal` and the mission heartbeat
`/benchmark/goal_active`. Cancelling the mission or losing its heartbeat stops
monitor requests. The Nav2 action remains the owner of route computation.

The run logger and `scripts/summarize_social_run.py` report request reasons and
time to the next matching route. That timing includes blocked-route retries and
coalesced requests; it is not an isolated planner-computation benchmark.

## Validation and remaining limitations

The live run in
`ros2_ws/src/dynamic_obstacle_benchmarks/results/demo_suite/suite_20260921T190557Z/onboard-waypoints/`
confirmed the event-driven tree was active and the onboard source audit passed:

- 136 matching route updates received.
- Requests included 112 motion changes, 82 corridor occupancy changes, 70
  off-route events, 11 observed blockages and two observed clearances.
- Local motion-planner wall time: **47.429 ms p95**, 59.299 ms maximum.
- Request to next matching route: 10.9 ms median, **2.649 s p95**, 8.883 s maximum,
  including periods with no valid route and recovery.
- Mission outcome: **3/4 goals, one collision**. The result validator failed.

Thus event-driven routing and retained local responsiveness are demonstrated,
but the broader collision-free, multi-seed mission acceptance remains open.
An earlier four-goal passing wiring trial still ran the stock BT because Humble
silently ignored absent YAML parameters; it is not counted as validation of the
new tree. Explicit missing-parameter insertion and a regression now cover that
launch issue.

Regression coverage includes event throttling, crossing forecasts, cleared
corridors, cancellation/stale-input handling, goal-associated route handoff,
in-flight request latching, fallback refresh, millisecond recovery delays,
prediction clearing, preservation of static/unknown cells, and launch rewrites.

Final checks passed: 228 targeted Python tests, both CTest targets (seven C++
tests), the five-package ROS build, `./scripts/test_workspace.sh fast`, and
`git diff --check`. These checks do not override the failed live mission result.
