# Long onboard visual demo

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

Run both viewers and a 30-minute visual session:

```bash
./scripts/run_onboard_navigation_demo.sh
```

Every representative world supports the same continuous demo: `./geoff worlds`
lists names, then use e.g. `./geoff demo office --people 4 --duration 300`.
`./geoff demo plaza` selects the original plaza.

The first invocation builds ROS and MuJoCo, including the new render-only goal
overlay. After that, add `--fast-start` to reuse those builds.

For an hour:

```bash
./scripts/run_onboard_navigation_demo.sh --duration 3600 --fast-start
```

`--duration` is in seconds. Startup/build/shutdown work can add overhead to the
session budget. Press **Ctrl+C** to stop the whole session.

## What opens

### MuJoCo

- A bright green ground ring shows the active waypoint and the 0.50 m arrival
  tolerance used by this demo.
- A downward arrow and a `GOAL` distance label make it easier to locate.
- The right-hand control panel starts hidden to give the scene more space.
  Tab / Shift-Tab toggle the viewer panels.
- The annotation clears when the mission is cancelled, transforms become stale,
  or goal updates expire.

The ROS visualizer sends only the goal relative to the estimated robot frame.
The MuJoCo viewer projects that relative goal into its display. Consequently the
annotation can adjust slightly as localization improves: it shows the perceived
target, not a privileged world-frame navigation input.

The ring, arrow and label are appended only to the viewer's `mjvScene`. They are
not model geometry, cannot collide with the robot, and are absent from LiDAR and
camera measurements. The source audit also checks the ROS goal visualizer for
privileged subscriptions.

### RViz

RViz starts automatically with:

- Measured LiDAR points, coloured by height.
- The projected ground scan.
- The live SLAM occupancy map and current planned route.
- Tracked people and predicted walking trajectories.
- The active waypoint and robot model.
- RGB and depth-camera preview panels at 160×120, 1 Hz.

Camera previews are visualization inputs; the navigation stack still uses its
LiDAR/IMU/encoder pipeline. Predictions are inferred from observed motion, not
pedestrian destinations or simulator itineraries. Large danger-corridor overlays
are available in the display's Namespaces settings but hidden initially for a
clearer map.

Use `--headless` to disable RViz explicitly.

## Longer sessions and episode restarts

The visual session keeps sampling new observed-free waypoints until its time
budget ends. Contact events are recorded without terminating the visual episode.
The next goal is prepared near arrival and published immediately once the robot
is within 0.5 m. Its actual footprint/path and distance from people are rechecked
at handoff. If observed free space is unavailable, the log explains the wait;
the system does not invent an unreachable target. The live metrics include
`goal_distance_m` and `next_goal_ready` for diagnosing a stalled handoff.
A fall or data-health failure ends that episode; the launcher then restarts the
simulator, RViz and navigation stack with a clean sensor-only map. The overall
session continues with the remaining budget. Viewers briefly close and reopen
during a restart.

Each restarted episode advances the base seed. For a reproducible seed series:

```bash
./scripts/run_onboard_navigation_demo.sh --seed 43 --duration 1800
```

To run one short episode with a goal limit:

```bash
./scripts/run_onboard_navigation_demo.sh --goals 4 --duration 120 --fast-start
```

An explicit `--goals` limit selects one episode, without automatic restarts.
`./scripts/run_demo_suite.sh --case onboard-visual` is also available for a single
visual episode. The scored `onboard-waypoints` case retains its acceptance rules.

## Results and verification

This is an **unscored visualization session**, not a collision-free benchmark
claim. Per-episode JSON/CSV/logs retain contact events, failed goals and stop
reasons. Results contain `visual_session: true`, cannot report benchmark success,
and are rejected by the scored result validator.

Artifacts are stored under:

```text
ros2_ws/src/dynamic_obstacle_benchmarks/results/demo_suite/
  visual_session_<timestamp>/
    session.json
    episode_001/onboard-visual/...
    episode_002/onboard-visual/...
```

Live checks exercised the real viewers (historical configurations):

- `visual_session_20260921T212846Z`: ran for the requested 75 seconds despite two
  recorded contacts; ended with `duration_elapsed`.
- `visual_session_20260921T213343Z`: ran for 40 seconds, reached three goals and
  recorded no contacts. Screenshots confirmed the MuJoCo green goal overlay and
  RViz's sensor previews, map, route and predictions.
- An earlier single-episode check ended on a fall, motivating the session-level
  restart wrapper. Restart sequencing and remaining-budget handling are covered
  by an injected-episode regression test.

These short checks verify duration handling and display integration; a complete
30-minute run and reliable navigation performance are not claimed.

The newer representative courtyard session `visual_session_20260922T001347Z`
logged a prepared-goal handoff at **0.474 m**. Its preview is under
`docs/media/courtyard/`. Prefetch now preserves the full configured sampling
annulus, and forward travel includes short holonomic final-goal adjustment to
avoid circling outside the arrival region.

With the calibrated-action profile, the 90 s clinic session
`visual_session_20260922T013828Z` reached **four goals**, with handoffs at
**0.481, 0.487, 0.482 and 0.490 m**, and recorded zero contacts/near misses.
This one-person visual run verifies goal cycling; it is not scored crowd
acceptance. Both handoff records are in
[visual_goal_handoffs.json](results/visual_goal_handoffs.json).

The overlay regression verifies that model geometry, robot state and raycast
results are unchanged by drawing the goal annotation. See the development guide
for current verification commands.
