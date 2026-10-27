# G1 Demo Suite

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

For a first visit, start with the [demo walkthrough and selection table](DEMO_GUIDE.md).
This page is the detailed suite reference; [results review](RESULTS_REVIEW.md)
explains how to interpret its artifacts.

The demo suite provides one entry point for reproducible locomotion, navigation,
and social-navigation demonstrations. Scored runs save their commands, logs,
policy identity, Git revision, durations, and exit status under a timestamped
artifact directory.

## Quick Start

List every preset and case:

```bash
./scripts/run_demo_suite.sh --list
```

Run the deterministic, headless regression that does not require MuJoCo or
external assets:

```bash
./scripts/run_demo_suite.sh --suite quick --headless
```

Run the official Unitree policy through the mock regression and a live forward
response/stop measurement in the bundled empty scene:

```bash
./scripts/run_demo_suite.sh --suite locomotion --headless
```

Run the prolonged four-pedestrian avoidance demonstration:

```bash
./scripts/run_demo_suite.sh --case social-endurance --headless
```

Preview all commands and preflight requirements without launching a demo:

```bash
./scripts/run_demo_suite.sh --suite full --headless --dry-run
```

## Presets

| Preset | Cases | Requirements |
| --- | --- | --- |
| `quick` | `mock-avoidance` | Docker image and built benchmark package |
| `locomotion` | quick plus `fixed-forward` | MuJoCo; downloaded checksum-pinned official policy |
| `navigation` | locomotion plus `waypoint-live` | Extreme Lab assets, SLAM, and Nav2 |
| `social` | quick plus `social-live` | Extreme Lab assets and social-navigation stack |
| `endurance` | quick, `fixed-forward`, and `social-endurance` | Bundled pedestrian plaza, social-navigation stack, and official policy |
| `full` | Combined locomotion, navigation, and social regression cases | All dependencies above |

The live presets are intentionally sequential. Only one simulation and one
command source should run at a time.

## Visual Cases

For a longer visual session with both viewers, sensor previews, SLAM, predictions,
and a render-only MuJoCo goal marker:

```bash
./scripts/run_onboard_navigation_demo.sh --duration 1800
```

The launcher automatically restarts early-ended episodes within the session
budget. This mode is unscored and preserves contact logs. See
[the visual-demo guide](ONBOARD_VISUAL_DEMO.md). The single-episode case is
`--case onboard-visual`; RViz is enabled unless `--headless` is supplied.

The experimental sensor-only case is:

```bash
./scripts/run_demo_suite.sh --case onboard-waypoints --goals 8 --headless --seed 42
```

It uses LiDAR/IMU/joint-derived localization, online SLAM, observed-person tracks,
and observed-free goals. Remove `--headless` for its map-frame RViz configuration.
It has not met reliable multi-seed completion/avoidance acceptance; see
[the onboard validation record](ONBOARD_SOCIAL_NAVIGATION.md).
Event-driven replanning is enabled automatically for this case; see
[dynamic route planning](DYNAMIC_ROUTE_PLANNING.md) for its triggers and timing.

For reachable random waypoints with independently wandering people, use:

```bash
./scripts/run_demo_suite.sh --case social-waypoints --goals 8 --seed 44
```

This scored case also supports `--headless`. It uses oracle tracking, a known
static scene map, and its own world-frame RViz view. Its agile command envelope
and controller differ from the zero-lateral staged endurance baseline below.
See [Historical social navigation upgrade](../archive/experiments/SOCIAL_NAVIGATION_UPGRADE.md) for the exact scope
and validation record. Pedestrians plan around static obstacles and one another,
but never react to the robot.

Visual cases are interactive and must be selected alone:

```bash
./scripts/run_demo_suite.sh --case waypoint-visual
./scripts/run_demo_suite.sh --case social-visual
```

`waypoint-visual` uses the Extreme Lab map, the checksum-pinned official Unitree
RL Lab policy, SLAM, Nav2, generated reachable goals, and RViz. Goal positions
are sampled across the full configured map bounds and are submitted only after
the active Smac planner returns a nonempty path from the robot's current pose.
`social-visual` is the historical oracle live showcase. For the current onboard
showcase, use `./geoff demo plaza`. The oracle case uses the bundled pedestrian
plaza and displays the same seeded four-crossing oracle scenario and yield/resume
gates as `social-endurance`, with MuJoCo and RViz visible. RViz follows the robot
and emphasizes the occupancy map, mission goal, global route, tracked and
predicted people, candidate motion, and one compact status marker. Diagnostic
labels, duplicate point clouds, local paths, risk-grid overlays, and dock panels
are hidden by default but remain available for debugging. The status marker
retains the explicit `ORACLE` tracking-source label.

## Safety And Policy Selection

The suite never selects `policy_path:=latest`. Recency does not imply policy
quality and the current reactive fine-tuning checkpoints fail bilateral lateral,
yaw, stop-settling, and action-clip acceptance gates. They remain research
artifacts and are not demo defaults.

Clean-clone suite cases use the official Unitree RL Lab G1-29DoF velocity policy.
The downloader verifies SHA-256 before every required suite. Staged social cases
(`social-live`, `social-visual` and `social-endurance`) keep
`max_lin_vel_y:=0.0`; avoidance therefore relies on early stopping and stable
forward arcs rather than unsupported last-moment sidesteps.

Waypoint and scored social cases use the measured official-policy walking
envelope and `0.35 m/s²` acceleration. Map-wide waypoint cases use a `0.45 m/s`
forward clamp, allow `0.35 rad/s` yaw, and use a `0.06 m/s` turning-arc floor so
the robot can converge on cross-map goals without sidestepping. Smac Hybrid
still plans with a `0.6 m` Dubins turning radius. Social planner and safety
limits retain the validated `0.20 rad/s` yaw cap and `0.20 m/s` turning-arc floor.

The scored `waypoint-live` case uses four seeded random goals across the Extreme
Lab map. Each goal is sampled after the previous goal finishes, checked against
the map bounds and wall margin, limited to an executable heading change, and
accepted only when Nav2 finds a path. Goal orientation follows the incoming
travel direction instead of requesting an unnecessary random in-place turn.

The fixed-forward case uses the measured official-policy showcase envelope of
`0.35 m/s` with `0.35 m/s²` acceleration and a five-second stop phase. It
validates all of the following:

- no fall and maximum tilt at most `0.35 rad`;
- a recorded policy SHA-256 and scene;
- motion onset within `1.5 s`;
- steady response at least 25% of the command;
- speed below `0.03 m/s` or `rad/s` for a sustained `0.5 s` window, or a
  final one-second p95 speed no greater than `0.08` when odometry is noisy;
- post-stop drift no greater than `0.20 m`.

Live waypoint and social cases also run the benchmark result validator. A zero
collision count, no fall, valid trial, movement, completed goals, finite metrics,
speed compliance, and the configured score threshold are required for success.
`social-live` is an inactive-person integration smoke test. It holds pedestrian
animation for 30 seconds, longer than its short fixed goal normally takes. This
validates startup, tracking, Nav2, locomotion, and command-chain compatibility,
but is not evidence of active avoidance.

`social-endurance` is the active oracle-driven command-chain demonstration; it
does not claim perception-only capability. Four analytic, one-shot pedestrians
cross the bundled plaza's forward route at separate locations without waypoint
wrapping or teleportation. Their crossing slopes and speeds are generated from a
new seed each suite run. Each crossing starts from robot progress and is timed so
that an unmodified `0.45 m/s` command reaches the interaction. The robot must
therefore repeatedly yield and resume across four forward checkpoints while
remaining collision-free for at least 30 seconds. Validation also requires at
least 20 seconds of pedestrian motion, positive body clearance, at least three
complete planner yield/return-to-`NORMAL_NAVIGATION` cycles across the four-person course,
at least two seconds where yielding overlaps moving pedestrians, at least `0.75 m`
of progress after yielding, and no lateral safety command. Hard collision-filter
overrides remain separate diagnostics and are not counted as planner yields. The
simulation-time actor profile is selected with
`mujoco_pedestrian_scenario:=demo_endurance_plaza`.

The generated seed is stored in `manifest.json` and forwarded to MuJoCo. Pass
`--seed N` to replay the same pedestrian courses and benchmark inputs.

After one normal run has populated `ros2_ws/.verification`, all live cases can
reuse those builds. This avoids redundant ROS workspace builds and, for the
social cases, explicit MuJoCo build containers on each launch:

```bash
./scripts/run_demo_suite.sh --case social-visual --fast-start
```

Before a scored run starts, the wrapper waits for robot state, LiDAR, a SLAM map,
and an active Nav2 behavior-tree navigator instead of relying on a fixed startup
sleep. Independent readiness conditions are checked concurrently at 0.2-second
intervals, so a healthy demo proceeds as soon as the stack is usable. MuJoCo
and ROS builds are reused in fast-start mode. Omit `--fast-start` after changing
ROS packages or simulator sources.

## Artifacts

The default root is:

```text
ros2_ws/src/dynamic_obstacle_benchmarks/results/demo_suite/
```

Each run creates:

```text
suite_<UTC timestamp>/
  manifest.json
  status.json
  <case>/
    suite.log
    suite_status.json
    ...case-specific JSON, CSV, YAML, and simulation logs
```

Use `--result-root PATH` to choose another location. The location must remain
inside the repository for cases whose containers write artifacts through the
workspace bind mount.

`manifest.json` captures the exact commands, case descriptions, timeouts, Git
revision and dirty state, seed, preflight results, and official policy hash.
`status.json` is written even after a case failure or interruption.

## Individual Cases

Cases can be combined with repeated `--case` arguments. Interactive cases cannot
be combined with scored cases.

```bash
./scripts/run_demo_suite.sh \
  --case mock-avoidance \
  --case fixed-forward \
  --headless
```

By default, the suite stops at the first failed gate. Use
`--continue-on-failure` when collecting diagnostics from every requested case.
This changes orchestration only; failed cases still remain failed in
`status.json` and the suite exits nonzero.

## Interpretation

The compact mock case uses four people and three goals to remain feasible across
the supported seeded smoke scenarios. It validates deterministic benchmark
mechanics and scoring. It does
not demonstrate MuJoCo locomotion, perception, Nav2, or physical person
avoidance. The live cases transform MuJoCo odometry and framed pedestrian truth
into the configured benchmark frame before scoring. Plaza course geometry is
deterministic for a fixed seed, but full actor and controller timing is not yet
bit-for-bit deterministic across runs. Keep both distinctions in reports and
demonstrations.
