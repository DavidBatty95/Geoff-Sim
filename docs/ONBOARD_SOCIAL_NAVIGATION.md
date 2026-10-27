# Onboard-only social navigation: implementation and measured status

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

## Status

The sensor-only pipeline is implemented and runs in MuJoCo. **The earlier oracle
24/24 result has not been reproduced reliably with onboard perception.** Live
development trials include complete missions, timeouts, and collisions. This is
an experimental mode, not a completed performance upgrade.

```bash
./scripts/run_demo_suite.sh --case onboard-waypoints --goals 8 --headless --seed 42
```

The first invocation builds the updated ROS packages and simulator. Later runs
can use `--fast-start`. Omit `--headless` to launch the map-frame RViz view.
For the longer session with MuJoCo goal markers and verified sensor/map/prediction
views, use [the onboard visual-demo launcher](ONBOARD_VISUAL_DEMO.md).

`social-waypoints` remains the explicitly labelled oracle diagnostic baseline.

The onboard case now also enables [event-driven route planning](DYNAMIC_ROUTE_PLANNING.md):
up to 5 Hz event requests, 2 Hz background refresh, and short-lived predicted-person
costs in the global costmap. That document records its implementation and live
validation; broader collision-free mission acceptance is still open.

## Information boundary

Navigation receives local-frame raycast LiDAR measurements, `/lowstate` IMU and
joint measurements, and joint-derived robot kinematics. It does not receive a
scene-derived map, simulator-root pose/velocity, pedestrian truth, destinations,
or future pedestrian routes.

- `g1_sim_sensors` in `onboard_only` mode never loads scene geometry, generates
  fallback geometry returns, or publishes simulator-derived navigation TF. Its
  `/evaluation/odom` output is reserved for scoring and offline diagnostics.
- Raw LiDAR packet world/root fields are ignored by the navigation data path.
  A counterfactual regression changes those fields while preserving returns and
  verifies that the navigation payload and pose state do not change.
- A runtime graph audit requires `/odom` to be owned by
  `onboard_lidar_odometry`, `/map` by `slam_toolbox`, and checks the navigation
  nodes for privileged subscriptions. The benchmark refuses to start on a failed
  audit. This is a graph/configuration check, not a security sandbox.
- Person truth and exact robot pose remain available to the benchmark and logger
  so collisions can be measured independently of perception errors. They do not
  select goals. A regression verifies that changing hidden simulator people
  cannot change the observed-person list used by goal selection.

Robot dimensions, sensor mounting calibration, the locomotion policy, and generic
human-size/motion assumptions are still necessary robot/model knowledge. There
is no preloaded environment map or pedestrian itinerary.

## Pipeline

1. **Onboard odometry:** measured IMU attitude and waist encoders level the LiDAR
   cloud. Robot kinematics and a measured floor-height correction establish the
   vertical reference. Trimmed, spatially indexed planar scan registration
   estimates translation against successive observed keyframes; relative IMU
   heading supplies yaw. Failed registrations do not publish fresh pose/TF.
2. **Online SLAM:** `slam_toolbox` starts an empty map in `map`, using estimated
   `odom -> base_footprint`. The 0.10 m map uses measured rays; unknown cells are
   not globally filled as free. Dynamic-person masking precedes mapping.
3. **LiDAR tracking:** geometric body detection, temporal association, and motion
   filtering operate on `/onboard/lidar`. This is not a camera/ML semantic person
   detector. Clipped body observations can update an existing confirmed identity
   without shrinking its footprint or retracting its prediction. Track-conditioned
   crops recover measured fragments merged into larger clusters; empty regions
   cannot refresh tracks. Lost tracks coast with uncertainty and expire.
4. **Prediction:** current motion is estimated from observation history. The
   onboard profile uses constant-velocity prediction rather than extrapolating
   noisy surface-centroid acceleration. Future destinations remain unknown.
5. **Navigation:** goals are sampled locally in observed free space, clear of
   observed people. The footprint and returned path must be known free, and Nav2
   must actually reach the proposed endpoint. Routes retained during temporary
   replanning failures are rechecked against the evolving observed map. Aborted
   Nav2 actions can be retried asynchronously for the same goal, retaining the
   original goal deadline and respecting cancellation.

The odometry front end assumes approximately planar walking and an available
IMU attitude estimate. Its reported covariance is an engineering setting, not
a calibrated localization uncertainty model. Camera fusion, uneven terrain,
sensor noise realism, and relocalization after extended loss remain unvalidated.

## Responsiveness

The translational envelope is retained: 0.65 m/s forward, 0.45 m/s lateral and
1.20 m/s² software slew. The current profile prefers bounded pivots (0.60 rad/s
requested, 0.65 hard command limit, 0.70 rad/s² slew) instead of forced forward
arcs. Isolated response tests inform the approximate 0.32 yaw-response gain.
Official-policy demos and calibration now share disabled deployment-only
joint-action slew limiting; command slew and bridge validation remain active.
The policy still runs with its bounded non-spinning inference worker.

A 45 ms candidate-search target rechecks stop, early crossing escapes when
relevant, the previous command and the route command first. Remaining
directions/refinements rotate across ticks. Every
selectable candidate receives its full collision check; the budget limits how
many candidates are evaluated, not their prediction horizon. Individual checks
and recovery can overrun the target, so this is not a hard real-time guarantee.

Historical timings in `suite_20260921T180949Z/onboard-waypoints/`:

| Measurement | p95 | Maximum |
| --- | --- | --- |
| Planner wall time | 47.115 ms | 65.681 ms |
| LiDAR odometry computation | 11.375 ms | 16.667 ms |
| Tracker computation | 53.501 ms | 93.490 ms |
| Track age at planning output | 0.305 s | 0.406 s |

An earlier unbudgeted trial had planner p95 111.096 ms and maximum 357.109 ms.
These are separate live trials, not a controlled identical-input speedup test.
Faster processing has **not** yet established reliable collision avoidance.

## Live validation record

Artifacts are under
`ros2_ws/src/dynamic_obstacle_benchmarks/results/demo_suite/`.
These are development trials across iterations, not an aggregate success rate
for one frozen configuration. Failed trials have been retained.

| Artifact suite | Seed | Goals | Collisions | Outcome |
| --- | --- | --- | --- | --- |
| `suite_20260921T170240Z` | 44 | 4/4 | 0 | Early sensor-only mission passed; no near misses |
| `suite_20260921T173133Z` | 43 | 8/8 | 0 | Passed in 183.8 s; no near misses; score 87.505 |
| `suite_20260921T174813Z` | 44 | 2/8 | 1 | Failed |
| `suite_20260921T175454Z` | 43 | 3/8 | 1 | Repeat failed, including a waypoint timeout |
| `suite_20260921T180514Z` | 42 | 7/8 | 0 | Budgeted search; no near misses, one goal timed out; validator correctly failed |
| `suite_20260921T180949Z` | 42 | 3/8 | 1 | Budgeted search repeat failed |
| `suite_20260921T181300Z` | 42 | 3/8 | 1 | Larger-margin experiment failed; override subsequently reverted |
| `suite_20260921T182436Z` | 42 | 1/8 | 1 | Continuous velocity-obstacle proposal experiment failed; proposal subsequently removed |

Thus neither reliable eight-goal completion nor oracle-level throughput has been
established. Seeded actors do not make wall-time scheduling, map admission, or
the accepted goal sequence deterministic.

## Diagnostics and remaining acceptance work

```bash
python3 scripts/summarize_social_run.py PATH_TO_human_aware.jsonl
python3 scripts/summarize_social_run.py PATH_TO_human_aware.jsonl --last 3 --tracker
python3 scripts/evaluate_onboard_perception.py PATH_TO_human_aware.jsonl
```

The logger records estimated and evaluation odometry with source timestamps,
localization quality/timing, tracking, predictions, and command-chain decisions.
The offline evaluator never publishes control data. Its nearest-track diagnostic
includes occluded/missed nearby people, uses no identity matching, and assigns
10 m to completely unmatched observations; it is not visibility-conditioned
detection accuracy or recall.

In the crop-enabled `suite_20260921T180028Z` trial, offline odometry drift after
initial alignment was 0.070 m p95 / 0.089 m maximum, and nearby truth-to-confirmed
track distance was 0.292 m p95. That trial still collided. Localization quality
alone therefore does not establish mission safety.

Remaining work is to resolve close-encounter prediction/control failures and
waypoint liveness under independently moving crowds, then rerun a frozen
configuration across multiple seeds. Collision checks, completion requirements,
and deadlines have not been relaxed to obtain passing results.

## Build and regression checks

- Updated ROS packages and the MuJoCo simulator built successfully.
- 285 targeted Python tests passed: 173 bridge, 71 benchmark, 16 bringup, and
  25 demo orchestration/result-validator tests.
- `./scripts/test_workspace.sh fast` passed.
- Root and simulator-submodule `git diff --check` passed.

New behavioral coverage includes scan-registration outliers/loss, truth-packet
counterfactuals, absence of synthetic sensor fallback, unknown-space/footprint
rejection, partial-body identity and footprint retention, measured-only track
crops, bounded search coverage, source-graph rejection, goal-selection independence
from hidden people, and cancellation of late Nav2 retry acceptances. Passing these
checks does not override the failed live acceptance trials above.
