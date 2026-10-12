# Unknown-environment search and transport prototype

**Latest follow-up:** [medium rooms, random colour tasks, region-level search and
visually identified destination approach sides](ROOM_SEARCH_AND_TASKS.md).
This page retains the initial 2026-10-06 experiment history.

Status, 2026-10-06: autonomous viewpoint selection and hidden-cube discovery have
run in native MuJoCo, with live RViz reasoning overlays. **Complete exploration →
pickup → destination search → placement is not yet validated.** The latest
cube-discovery run stopped because navigation did not reach the pickup staging
pose. The five retained development attempts include one fall and no completed
deliveries.

## Run and observe

From the repository root, with the locally trained cycle-03 policy available:

```bash
python3 scripts/run_exploration_transport_demo.py --world transport_search --people 0 --seed 48 \
  --layout-seed 0 --duration 600 --search-max-viewpoints 16 \
  --policy vendor/mjlab/logs/payload_training/native_delay_cycle_03_20261006/policy.onnx \
  --fast-start
```

`--fast-start` requires an already built workspace; omit it when a build is
needed. RViz opens by default. `--layout-seed 1` mirrors the fixture. Layout
generation happens in the simulator launcher; the mission receives object IDs
and colors rather than layout coordinates or the seed. The policy is a local
training artifact, not supplied by this document.

The fixture contains four tables, distractors, and partitions that initially
occlude the task objects. In RViz, use fixed frame `map` and the search
MarkerArray display on `/exploration/search_markers` (enabled in both supplied
navigation RViz profiles):

| Display | Meaning |
| --- | --- |
| Green points | Frontier-information candidates |
| Cyan points | Visual-coverage candidates in already mapped space |
| Orange arrow and line | Selected viewpoint and bearing from the robot; the line is not a collision-checked route |
| Grey points | Inspected locations |
| Red points | Failed viewpoint regions |
| Object labels | Previously confirmed RGB-D object locations, with observation age; stale memory fades |
| Status text | Target, executive state, candidate count and worker planning time |

`/exploration/search_status` publishes JSON at 2 Hz and is recorded as
`search_status` in `trace.jsonl`. Search completion retains `found` or
`search_failed` and the last target for display. `planning_ms` measures candidate
computation; the mission event's `selection_wait_ms` separately measures the
foreground selection/revalidation interval. Neither is total navigation latency.

## Decision flow and delay reductions

1. Inspect the current camera view using fresh RGB-D acquisitions. Later stops
   add a quarter-turn rather than repeating a complete 360-degree scan.
2. Inflate observed obstacles and unknown space by the search footprint, then
   flood-fill reachable known-free space. Score candidates using first-hit
   unknown information, visual novelty, path length and heading-change cost.
3. Prepare candidates in a single background worker while the robot moves or
   inspects. Detached inputs, target/generation/payload checks and a two-second
   result-age bound prevent obsolete plans from becoming new goals. Revalidate
   against current map, people and pose before dispatch.
4. Send a managed navigation goal through the existing Nav2 → human-aware
   planner → collision supervisor → policy chain. Discovery can stop navigation,
   but the executive must establish action termination before handing off.
5. After confirmed pickup, search for the destination. A previous confirmed
   RGB-D location can suggest a reachable stand-off reobservation view; it cannot
   substitute for a fresh detection.

An absence of frontiers does not establish object absence: visual-coverage
candidates remain useful on an already mapped floor. Search terminates under a
viewpoint budget or mission deadline. Unknown cells are never treated as
traversable. People freshness/admission checks remain active, including with a
larger search footprint when an object may be held.

Performance changes also include batched NumPy information rays, scalar/list
flood-fill operations, subscriber-aware marker construction, a 0.1-second
search-navigation settle interval, and fixed executor polling arguments that
preserve callback fairness. Freshness requirements and action-termination rules
remain prerequisites for movement. Camera acquisition and physical turns still
dominate some stops; seamless continuous motion has not been established.

Implementation: `exploration_search.py`, `exploration_mission.py`, and
`search_visualization.py` in
`ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/`.

## Retained native evidence

[Portable outcomes and timings](results/exploration_search_20261006.json) include
all five attempts, source/archive hashes and the explicit policy identity. Raw
artifacts are under
`ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/<run>/`.
All used seed 48 and zero people; implementations and fixture geometry evolved
between runs. These are development observations, not a controlled speedup or
reliability study.

| Run (2026-10-06 UTC) | Layout | First goal after search request | Median selection wait | Outcome |
| --- | --- | --- | --- | --- |
| `183529Z` | 0 | 30.19 s | Not instrumented | Cube found after 133.05 s; navigation aborted |
| `185522Z` | 0 | 2.18 s | 55.73 ms | Search budget exhausted; repeated localization recovery |
| `190954Z` | 0 | 2.38 s | 61.35 ms | Fell; fresh-image inspection failed |
| `191455Z` | 0 | 1.78 s | 65.32 ms | Cube found after 19.11 s; pickup staging navigation timed out |
| `192855Z` | 1 | 2.21 s | 39.65 ms | Three inspected viewpoints; deliberately short 60-second budget expired |

The last run's three candidate computations took 125.94–161.80 ms in the worker;
foreground selection/revalidation took 35.94–40.32 ms. It stayed upright and
recorded zero contact samples, with no confirmed cube discovery. All five runs
had valid evaluator data and zero recorded contact samples; this does not negate
the earlier fall or constitute crowd qualification.

Evaluator `observed_object_ids` are not mission discoveries. The report uses
`search_found` events to establish discovery, and reports perception-history
confirmations separately. No exploration delivery was verified. The transport
grasp remains a capture-radius simulated weld, with no base pinning.

Verification after the flood-fill optimization: **458 bridge tests passed**.
The full mission's next gate is reaching a sensed pickup staging pose after
discovery, followed by loaded destination search and verified placement. Only
then can unchanged-code mirrored-layout and people-present repeats establish
broader behavior. See also the [people-clearance follow-up](TRANSPORT_PEOPLE_CLEARANCE.md).

To regenerate the portable report from retained artifacts:

```bash
python3 scripts/analyze_transport_perception.py \
  ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/20261006T183529Z \
  ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/20261006T185522Z \
  ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/20261006T190954Z \
  ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/20261006T191455Z \
  ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/20261006T192855Z \
  --output docs/results/exploration_search_20261006.json
```

## Research attribution and implementation scope

This is a heuristic search executive informed by the following work. It does
not implement SafeEstMDP or formal MDP/LTL synthesis, and inherits none of their
formal guarantees.

| Source | Relevant idea and present scope |
| --- | --- |
| [ORI GOALS: SafeEstMDP hazardous-environment exploration](https://ori.ox.ac.uk/groups/goals/news-goals/exploring-hazardous-environments-with-mobile-robots) | Balances information gain against estimated hazard exposure, including safe return. Our prototype uses information scoring and geometric/people admission; it has no learned hazard distribution or safe-return proof. |
| [STRANDS MDP plan execution](https://strands.readthedocs.io/en/latest/strands_executive/mdp_plan_exec.html), Lacerda, Parker and Hawes (IROS 2014) | Probabilistic route planning and co-safe LTL task specifications motivate explicit task progression and monitored execution. Our implementation uses bounded managed actions, not PRISM or product-MDP policy synthesis. |
| [STRANDS SOMA](https://strands.readthedocs.io/en/latest/soma/index.html) | Time-stamped semantic memory motivates age-labelled observation hints. The prototype retains confirmed RGB-D hints, not a SOMA database. |
| [STRANDS next-best-view planning](https://strands.readthedocs.io/en/latest/strands_3d_mapping/nbv_planning.html) | Candidate scoring and visualization are relevant. Its documented algorithm credits Potthast and Sukhatme (2014), not an ORI invention. Our 2-D ray heuristic is not that probabilistic 3-D implementation. |
| [ORI VILENS](https://ori-drs.github.io/projects/vilens-multi-sensor-odometry/) | Multi-sensor odometry/mapping supported the ORI exploration deployment. VILENS is not integrated here; this mission consumes the existing observed SLAM map and transforms. |

These sources identify useful future extensions: navigation success/time models,
uncertainty-aware semantic search and explicit risk budgets. Such extensions
need their own implementation and comparative evaluation.
