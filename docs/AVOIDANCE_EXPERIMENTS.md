# Avoidance worlds, dataset and results GUI

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

## Inspect measurements

```bash
python3 scripts/avoidance_results_gui.py
```

Open <http://127.0.0.1:8765>. The server binds to loopback and serves only the
dashboard, normalized JSON results and CSV export. Results are reread on Refresh.
There is no upload service and no browser-triggered robot command.

Run type and text filters apply to the table and chart. Select a row (or focus it
and press Enter) for measured metrics and source paths/hashes. Missing values are
shown as missing, not zero. The chart reports goal progress within each world,
people count and source mode; it is not a pooled safety success rate.
Where attached, the replay slider shows downsampled robot/people positions from
the offline evaluation log. Samples are asynchronous and qualitative; benchmark
collision metrics come from the scorer, not the browser drawing.

## Representative world definitions

`scripts/avoidance_worlds.py` is the versioned source of six bounded worlds:

| World | Interior size | Obstacles beyond four boundary walls |
| --- | --- | --- |
| small_open (`courtyard`) | 8 × 8 m | Residential courtyard façades, doors and lighting |
| small_clutter (`cafe`) | 8 × 8 m | Café furniture and a planted tree |
| medium_open (`station`) | 12 × 10 m | Station entrance with marked circulation and lighting |
| medium_doorway (`clinic`) | 12 × 10 m | Glazed partitions and a 2 m opening |
| large_pillars (`concourse`) | 16 × 14 m | Six station columns and platform signage |
| large_office (`office`) | 16 × 14 m | Lobby partitions and four seating blocks |

`world_appearance.py` adds original procedural tile textures, bitmap signage,
windows, doors, beams, lights, wooden furniture and planting. Furniture uses
conservative collision proxies within the declared footprints. These are stylized
representative environments, not surveyed digital twins or photoreal scans.
The generator preserves robot/actor assets and replaces plaza scenery. The simulator reads custom
world extents for actor spawn/destinations and uses the world's static boxes for
actor path admission. Actors never read robot state in the natural-crowd mode.
World extents stay inside the actor A* grid (16 × 14 m maximum).

Use `./geoff demo NAME` with any friendly name above or `plaza`. The JSON records
`world_version: representative-v2`; the earlier simple-box baseline is preserved
in [avoidance_baseline_v1.json](results/avoidance_baseline_v1.json). Appearance and
occlusion have changed, so a v1/v2 comparison also changes the sensor environment.
Unscored demos default to two people in small rooms, four in medium rooms and
eight in large rooms/the plaza; `--people 1..8` overrides this. The scored matrix
keeps an explicit uniform four-person cohort, including the dense small rooms.

Generated scene XML and walking meshes are ignored build products. Regenerate
with `docker compose run --rm unitree-g1-sim /workspace/scripts/build_unitree_mujoco.sh`.
Onboard navigation gets no scene map: SLAM and free-space goals remain sensor-driven.

## Collect a cohort

Complete installation first. The matrix refreshes the verification build once;
`--fast-start` reuses an existing build. Stop other simulation
sessions; the existing launch wrapper shares ROS/simulator resources and cleans
up stale simulation containers. Run serially:

```bash
python3 scripts/run_avoidance_matrix.py
# Smaller explicitly selected experiment:
python3 scripts/run_avoidance_matrix.py --worlds small_open medium_doorway --seeds 45 46 --people 8
# Generate the command manifest without running simulations:
python3 scripts/run_avoidance_matrix.py --dry-run
```

Defaults: six worlds × seeds 42/43/44 × four people, three goals, 90 s benchmark
budget per trial. There is additional startup/cleanup time and a process timeout.
`--trial-seconds`, `--people`, `--goals`, `--worlds`, `--seeds`, `--output`, and
`--publish` are explicit overrides. Output directories must be fresh and under
`ros2_ws/` for the container mount. Do not mix different budgets/counts as one cohort.

Each trial records the exact command, world hash, seed, return code, timeout,
result source path/hash and full benchmark result. The cohort records source
revision/dirty state, patch/policy/config hashes and world geometry. Missing
results become invalid attempts, not silent omissions or zero-collision passes.
Dry runs never publish fabricated measurements.
The matrix returns nonzero when any trial fails strict acceptance, after saving
the complete dataset; an interrupted run is retained as an invalid attempt.

The published JSON is `docs/results/avoidance_dataset.json`; raw logs remain in
`ros2_ws/src/dynamic_obstacle_benchmarks/results/avoidance_matrix/`.

```bash
python3 scripts/avoidance_results_gui.py --export docs/results/avoidance_dataset.csv
python3 scripts/enrich_avoidance_dataset.py docs/results/avoidance_dataset.json
```

This is an initial development dataset, not a statistically powered acceptance
study. Three seeds do not establish general reliability. World complexity and
size vary together for some layouts; use the matched open/clutter pairs for
more controlled comparisons. Random observed-space goals are not identical
trajectories across worlds. Contacts/near misses use the existing benchmark's
body-clearance definitions. Keep invalid trials and all failure reasons when
reporting rates; do not substitute historical oracle measurements.

The GUI dataset selector includes the baseline, published cohort, tuning pilots
and historical demos as separate datasets. A raw benchmark success whose strict
validator returned nonzero is displayed as a failure, not a pass.

## Engineering iterations

The repository retains failed tuning pilots as separate JSON datasets. Native
clustering preserves adaptive-radius connectivity while reducing Python CPU
load. Measured-only filter response tests cover walking onset after a long idle
and false motion from stationary centroid noise. Route references favor forward
travel, use observed-free shortcuts, and permit short final-goal adjustments.

Isolated yaw diagnostics are in [turn_response.json](results/turn_response.json).
The 0.6 rad/s request produced approximately 0.188 rad/s steady response and
0.416 m net drift over 8 s; the 0.9 request drifted 1.214 m. The latter was not
adopted. Response gain remains an engineering approximation, not a fitted safety
certificate. Official-policy demos now match the calibration tool's disabled
deployment-only joint-action slew filter; command slew limits remain enforced.
