# Room search, random colour tasks, and visual table approach

## Current status — 2026-10-07

The default exploration fixture is now a **16 × 12 m medium building**, with
four rooms, a central circulation area, connecting corridors, and two 2.8 m
entrances per room. It contains red and blue boxes and separate red and blue
destination pads. A single task is randomly selected at startup, including
cross-colour assignments such as **red box → blue goal**.

The robot now identifies a destination's approach side from RGB-D tabletop
geometry. A native `transport_dual` regression completed red-box-to-blue-goal
delivery with independent placement verification, zero recorded contacts, no
fall and no drop. **The medium-building task has not yet completed a verified
delivery.** Its retained attempts include successful pickups, arrivals at the
destination, docking failures and an exploration fall. The compact pass does
not qualify the medium building or people-present operation.

## Run

From the repository root, with the local cycle-03 policy available:

```bash
python3 scripts/run_exploration_transport_demo.py --people 0 --seed 48 \
  --policy vendor/mjlab/logs/payload_training/native_delay_cycle_03_20261006/policy.onnx
```

The command above explicitly selects zero people for the original regression.
Without `--people`, the exploration launcher now starts **one walking person**.
It defaults to `transport_medium`, a fresh random objective, a 600-second
mission budget and 40 search attempts per discovery. RViz opens by default.
Add `--fast-start` to reuse an already built workspace.

Useful options:

| Option | Meaning |
| --- | --- |
| `--task-seed 48` | Replay the random objective **red box → blue goal** |
| `--people 1` | One natural walker (exploration default); counts 0–8 are supported |
| `--pedestrian-start-delay 10` | Start walking after ten simulation seconds; configurable 0–120 |
| `--no-tracker-wall-evidence` | Disable upper-wall rejection for a matched diagnostic comparison |
| `--early-overtaking-avoidance` | Opt into the experimental longer lookahead for faster following pedestrians; disabled by default |
| `--task red:blue` | Specify that objective directly |
| `--task blue:red` | Specify the reverse cross-colour objective |
| `--task all` | Deliver all fixture-defined matching-colour pairs |
| `--layout-seed 1` | Mirror the simulator layout; distinct from task selection |
| `--world transport_atrium` | Retained 28 × 24 m atrium building, eight table stations |
| `--world transport_corridors` | Retained 32 × 24 m corridor building, twelve table stations |
| `--world transport_search` | Earlier four-table occlusion diagnostic |

The medium footprint is 192 m², compared with 672 m² for the large atrium.
Each run saves `objective.json` and records the task in `manifest.json` and
`result.json`. The mission and evaluator receive the same selected object and
destination IDs. A task seed reproduces the assignment, not an identical
real-time execution trace. The mission receives no table coordinates or layout
seed. Both colours remain physically present regardless of the selected task.

## Walking people and measured performance

```bash
python3 scripts/run_exploration_transport_demo.py --people 1 --seed 48 \
  --task-seed 48 --pedestrian-start-delay 10 \
  --policy vendor/mjlab/logs/payload_training/native_delay_cycle_03_20261006/policy.onnx \
  --fast-start
```

The launcher explicitly selects `social_natural_plaza`, whose native walking
controller uses the building's static collision geometry, seeded destinations,
continuous motion and animated gait. Its routes are independent of robot goals;
walkers can cross corridors and doorways. A ten-second startup delay is recorded
in the manifest. Robot tracking remains sensor-driven; actor truth is used only
by evaluation and offline reports.

People-present runs retain the generated navigation profile with extra human
clearance, a shared 0.45 m/s planar cap, dynamic avoidance, and workspace checks
around manipulation. RViz's existing tracked-person/prediction displays run
alongside the search overlay.

**This is experimental crowd integration, not validated crowded delivery.**
Three retained medium-building trials all fell before completing delivery:

| Run (2026-10-07 UTC) | People | Measured walking distance | Contacts | Planner p95 |
| --- | --- | --- | --- | --- |
| `105516Z` | 2 | 33.56 m and 37.67 m | 77 samples | 63.78 ms |
| `105937Z` | 1 | 34.16 m | 14 samples | 54.54 ms |
| `110220Z` | 1 | 31.61 m | 43 samples | 57.72 ms |

All had valid evaluator streams. These contact counts are samples, not distinct
collisions. The last trial tested fewer extra search proposals per control tick;
it did not improve the measured p95 or outcome, so that tuning was reverted.
Different execution trajectories make these diagnostic comparisons, not causal
performance estimates.

The trace exercised caution, yielding and clearance recovery, but reported up
to eight tracks for two actors and six for one. Tracking quality and robot
stability during interactions remain concrete bottlenecks. Planner timing tails
also exceeded the 45 ms soft budget. No manipulation-interruption or complete
delivery qualification follows from these exploration-only failures.

[Crowd evidence](results/exploration_crowd_20261007.json) includes per-actor
measured travel, movement samples, observation gaps, planner-state counts and
timing distributions. `analyze_transport_perception.py` adds these measurements
for people-present runs by reading retained traces offline. Replayed samples
and long observation gaps do not add measured travel. Use `--people 0` for the
zero-person regression and `--people 2` for the denser diagnostic.

The [failure review](DEMO_FAILURE_REVIEW.md) examines incident order, false tracks,
active planning latency and the staged improvement priorities in more detail.
The [implementation follow-up](PERCEPTION_RESPONSE_FOLLOWUP.md) records wall
rejection, body-fault handling, response probes and the remaining failures.

## Exploration decisions

`room_search.py` creates room-like **topological regions from the observed map**:

1. Conservatively coarsen known-free footprint cells to approximately 0.4 m.
2. Erode traversable space to find wide cores separated by narrow passages.
3. Expand cores through connected free space; seed remaining furnished areas
   at clearance maxima. Bound expansion to 4 m geodesically so a single atrium
   core cannot absorb the entire building.
4. Rank regions using remaining visibility coverage, frontier information,
   travel distance and a bounded preference for continuing the current region.
5. Rank viewpoints inside the selected region and aim the camera toward
   uninspected area, accounting for turning cost.

There are at most 128 planning regions. Region labels are provisional and may
change as the map grows; inspection memory is attached to map-frame positions,
not permanent room numbers. Furnished rooms can split into several regions.
This is not architectural room recognition.

Coverage is a conservative **2-D visibility proxy** from fresh RGB-D inspection
events and observed-map line of sight, using a 3.5 m range and 60° horizontal
cone. It does not model every 3-D occlusion and is never proof of object absence.
Regions are deprioritized at 70% proxy coverage, after four distinct inspections
with little frontier gain, or after six inspections. They remain revisitable.
All task colours are processed in each registered image, so soft inspection
memory is shared between box and destination searches.

The bounded background worker prepares candidates during movement. Dispatch
revalidates map/people/pose and recalculates novelty and travel cost after the
latest inspection: otherwise a plan prepared before arrival can incorrectly
rank the just-inspected floor as novel. Reachability supports routes up to 80 m
within a 500,000-cell map bound. Unknown cells remain non-traversable.

Observed-map routes use line-of-sight corner checkpoints. At a corner, the robot
advances only until the continuation is clear, or until it can replan from the
corner; it does not demand manipulation-level settling at every route vertex.
All motion still passes through managed navigation, the human-aware planner,
the collision supervisor and the locomotion policy. An unknown action
termination never permits a new goal.

## Task-aware pickup and delivery

The selected box is sought first. If the requested destination has not been
observed recently, the robot surveys it **before lifting the box**, then returns
to reobserve and pick up the box. This trades an extra empty-handed trip against
a potentially long loaded search and camera occlusion. It is a heuristic,
not an established optimal ordering.

After pickup, an old confirmed destination sighting prioritizes a reobservation
viewpoint. Fresh perception is still required for delivery. The executor stops
after the selected task; it need not completely map the building or deliver the
unused box.

Timing distinguishes discovery, grasp request, held acknowledgement, verified
lift, and post-release placement. `discovery_to_placement_s` includes the
empty-handed destination survey; `grasp_to_placement_s` starts at the held
acknowledgement. Missing completion remains `null`, not zero.

## Identifying the destination approach side

The blue/red square's outline alone has four equivalent orientations. Choosing
the closest square normal from the robot's bearing previously allowed an
approach from the wrong table face.

The updated visual processing:

- Reconstructs the surrounding tabletop from registered depth and measured
  camera/joint/IMU geometry.
- Fits an oriented rectangle to the visible support plane.
- Locates the pad relative to its edges and selects a uniquely nearby edge.
  The inward normal is the desired robot facing direction; the stand-off lies
  outside that edge.
- Rejects insufficient geometry, a centred pad, near-equal corner alternatives,
  conflicting support components, and support components clipped by the image
  boundary. Pad centres must be within 0.30 m of an inferred edge, with at least
  0.08 m separation from the next edge-distance alternative.
- Confirms the direction across two distinct, consistent acquisitions. Direction
  confirmation is separate from position confirmation; a new conflicting
  direction removes the old approach permission.

The observations expose `approach_yaw`, `approach_source=rgbd_support_edge` and
`approach_observation_count`. This is an inference from visible support geometry,
not a decoded arrow or a simulator table orientation. Partial 3-D occlusion
remains a limitation.

If the side is unavailable, the robot obtains another stand-off view. Delivery
staging uses the confirmed side at 1.4 m, reobserves the support direction, then
precision-docks from that side. It no longer substitutes the nearest square
axis for the destination's approach direction. Freshness, arm workspace checks,
release verification and independent placement scoring remain in force.

Docking now enters a 3.5 cm moving-position band before stopping, with a 2.5 cm
planner deadband. Final settled acceptance remains 7 cm and 0.15 rad. This
reserves room for gait settling instead of relaxing final acceptance.

## RViz and trace interpretation

The existing `/exploration/search_markers` overlay now includes:

- The selected task in the robot status label.
- Region centres, proxy-coverage percentages and inspection counts; the active
  region is highlighted and deferred regions are labelled.
- Red/blue object and pad markers matching their actual detected colour.
- An inward-pointing approach arrow beside objects/pads with confirmed support
  direction. Stale semantic memory fades and cannot authorize manipulation.

The orange robot-to-viewpoint line is a decision bearing, not a navigation path.
`/exploration/search_status` also reports the objective and region summaries.
Trace events include `search_room_decision`, `search_memory`, `pickup_route`,
`delivery_route`, `inspect_support`, and `delivery_approach`.

## Evidence and open bottlenecks

[Portable evidence](results/room_search_and_tasks_20261007.json) retains all 20
development attempts in this follow-up, including invalid startup, falls,
timeouts and docking failures. It includes source/archive hashes, task metadata
where captured, search timings and manipulation milestones. Worlds, code and
budgets evolved; this is not a controlled performance cohort.

| Run | Fixture | Outcome |
| --- | --- | --- |
| `20261006T212613Z` | Large atrium | Invalid startup: old native pedestrian-grid bound; subsequently corrected |
| `20261006T214319Z` | Compact dual-table pairs | Verified red-box lift 22.19 s after discovery; destination-search timeout |
| `20261007T092021Z` | Large atrium | Region-level search/navigation exercised; 300-second deadline, no delivery |
| `20261007T093454Z` | Medium, random seed 48 | Red box discovered and lifted; loaded blind search timed out |
| `20261007T094803Z` | Medium, random seed 48 | Both task locations observed; pickup and return to blue goal; delivery-docking deadline |
| `20261007T095500Z` | Medium, random seed 48 | Both task objects identified and red box carried to destination; settling failure |
| `20261007T100357Z` | Medium, random seed 48 | Pickup succeeded; destination docking failed before support-side correction |
| `20261007T102750Z` | Medium, random seed 48 | With support-side perception enabled, fell during exploration before delivery |
| **`20261007T103313Z`** | **Compact `transport_dual`, red → blue** | **Strict complete pass with visually inferred destination side** |

The successful run inferred destination normals of 3.14540 and 3.14404 rad
(approximately 180°), each supported by four consistent observations. It recorded
0.222 m lift and 5.197 m maximum object displacement, then passed fresh
post-release camera verification and independent stable-placement containment
checks. Discovery-to-placement was **245.74 s**, including destination survey;
held-acknowledgement-to-placement was **75.51 s**. No fall, drop or contact samples
were recorded. The grasp model remains a capture-radius simulated weld.

Reproduce the successful test configuration:

```bash
python3 scripts/run_exploration_transport_demo.py --world transport_dual \
  --people 0 --seed 48 --task red:blue --duration 420 \
  --policy vendor/mjlab/logs/payload_training/native_delay_cycle_03_20261006/policy.onnx \
  --fast-start
```

Checks at this checkpoint: **476 bridge tests**, **20 targeted script tests**,
and **11 native CTests** passed. Tests cover rotated tabletop geometry, ambiguous
pads, actual perspective RGB-D projection, distinct-frame direction confirmation,
cross-colour scoring, room/door connectivity and conservative route clearance.

Remaining bottlenecks are medium-building route execution/locomotion near
doorways, repeated region inspections and variable search latency. A compact
delivery pass establishes the approach-side integration, not a fastest-policy
claim or repeatable medium-building acceptance.

## Research basis

| Work | Relevant idea | Implemented scope |
| --- | --- | --- |
| Zhou et al., [FUEL](https://arxiv.org/abs/2010.11561), RA-L 2021; [code](https://github.com/HKUST-Aerial-Robotics/FUEL) | Frontier information structures, global coverage order and local viewpoint refinement | Bounded background candidate preparation and region/viewpoint hierarchy; no FUEL trajectory optimizer, incremental FIS implementation or inherited speedup claim |
| Cao et al., [TARE](https://github.com/caochao39/tare_planner), RSS 2021; representation-granularity follow-up, Science Robotics 2023 | Sparse global planning with detailed local coverage | Coarse connected regions and local view ranking; no TARE/OR-Tools tour solver |
| Bormann et al., [room segmentation and indoor coverage](https://github.com/ipa320/ipa_coverage_planning), ICRA 2016/2018 | Occupancy-based spatial decomposition and systematic coverage | Morphological cores plus bounded geodesic regions, not semantic architectural-room recognition |
| Chaplot et al., [Goal-Oriented Semantic Exploration](https://arxiv.org/abs/2007.00643), 2020 | Episodic semantic memory and task-conditioned search | Confirmed box/pad memory and task-specific reobservation; no trained semantic prior |
| [ORI GOALS/STRANDS references](EXPLORATION_SEARCH.md#research-attribution-and-implementation-scope) | Information/risk trade-offs, monitored execution and temporal semantic memory | Heuristic admission and managed action ownership; no SafeEstMDP or MDP/LTL synthesis guarantees |

These ideas motivate the implementation. The planner is original heuristic
code, not a reproduction of the cited systems. Comparative efficiency needs
unchanged-code multi-seed task runs with completed-placement timing.
