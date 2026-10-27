# Reliable people-aware box transport: implementation plan

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

**Active roadmap (2026-10-05):** [staged BrainCo indoor-delivery autonomy
plan](SOCIAL_LOCOMANIPULATION_ROADMAP.md). It defines current priorities and
promotion gates. The dated checkpoints and experiment ledger below are retained
as historical evidence; their older next-step statements are superseded by the
active roadmap.

The [gap-aligned P0–P7 research programme](RESEARCH_PROGRAMME.md) specifies how
the engineering work can support distinct scientific studies and candidate
papers. Use its protocol, comparison and claim gates alongside the operational
milestones; this document preserves the earlier transport experiment history.

## End-state demonstration

The G1 repeatedly discovers a box on a table, selects its matching empty marked
square, approaches and picks it up, carries it through a furnished environment
with independently moving people, and places it on the correct square. The
results GUI reports each delivery and every failure, contact, drop and recovery.

**Current status:** navigation remains below acceptance; the last complete
published navigation cohort passed 4/18 strict trials. Subsequent troubleshooting
reached a best observed 16/18 strict passes; the latest full matrix (v84)
passed 10/18 with six contacts. The v85 alignment audit confirmed real geometric
contacts in both audited courtyard failures. Investigation history is in the
[reliability investigation](NAVIGATION_RELIABILITY_INVESTIGATION.md).
The existing transport action server is an integration starting
point, not evidence that the complete mission works.

**Latest follow-up:** a 300-iteration payload-aware RL fine-tune and held-out
locomotion probes are complete. Two consecutive unchanged zero-person native
runs completed pickup, loaded carry, verified placement, retreat and arm restoration;
see [payload training and locomanipulation status](PAYLOAD_POLICY_TRAINING.md)
for the newer commands and evidence. The checkpoint below records the earlier
integration state.

## Executable transport checkpoint (2026-09-25)

Run the experimental, sensor-driven physical fixture with:

```bash
python3 scripts/run_transport_demo.py --world transport_lab --people 0 --no-rviz
```

The launcher generates fixtures, builds the selected ROS packages and simulator,
starts RGB-D sensing, navigation, arm composition and simulated grasp services,
then writes a manifest, simulation log, navigation trace and mission result under
`ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/<UTC timestamp>/`.
Use `--fast-start` only after those builds are current. `--no-rviz` disables RViz;
the native MuJoCo viewer still requires the usual display setup.

**No end-to-end transport pass yet.** Five isolated zero-person attempts were
retained; none reached grasp or lift. These are manipulation integration trials,
not crowd-navigation acceptance:

| UTC run directory | Outcome | Evaluator data valid |
| --- | --- | --- |
| `20260924T225241Z` | Detected box, reached staging, pickup docking timed out | Yes |
| `20260924T225511Z` | Staging wait timed out; mission/evaluation updates stale | No |
| `20260924T225750Z` | Discovery turn timed out; final mission odometry 44.3 s old | No |
| `20260924T225933Z` | Fresh mission state, pickup docking stalled at 0.20 m/s | Yes |
| `20260924T230124Z` | Pickup docking completed at 0.30 m/s; pregrasp IK rejected a 0.138 m position error | Yes |

Changes at this checkpoint:

- Bitmap signage accepts spaces; destination outlines remain inside tabletops
  after the pad enlargement. The latter fixture correction was made after the
  five trials and passed fixture tests; it has no new end-to-end trial yet.
- A three-thread executor keeps image processing separate from mission and
  evaluation callbacks. Active motion aborts if measured odometry is stale.
- Docking uses a 0.30 m/s approach command through the existing motion checks;
  lower commands stalled in the recorded trials. This is experimental and has
  only one successful pickup-docking observation so far.
- Mission results include final sensed pose, pose age and requested goal poses.
- Onboard graph auditing includes the new mission/perception nodes.

Verification: **280 bridge tests**, **8 fixture/source-contract tests**, and
**10 simulator CTests** pass. New geometry tests check the actual URDF Jacobian
against finite differences, reachable-target IK, RGB-D projection and rejection
of below-table colour patches.

Next gate: resolve the sensed pregrasp pose versus actual wrist/hand reach,
validate swept arm/table clearance, and demonstrate unpinned lift, carry and
release. Do not enlarge the capture tolerance merely to bypass the failed reach.
The current evaluator's person-clearance counter does not certify absence of
arm/table or other static contacts. Unknown-world exploration, repeatable crowd
transport and comprehensive collision acceptance remain outstanding.

## Existing components and required changes

- Nav2, online SLAM, measured LiDAR tracking, local social planning and the final
  collision supervisor provide the navigation foundation.
- `g1_task_orchestration` already sequences navigation, approach, Pick/Place,
  retreat and arm authority. It currently uses base-pinning phases and expects
  supplied staging poses; these are not sufficient for the mobile acceptance run.
- `g1_manipulation` already offers MoveIt Pick/Place and named arm-posture actions.
- The arm compositor separates upper-body trajectories from locomotion commands.
- MuJoCo's existing grasp model activates a palm–object weld only inside a
  capture radius. It preserves the object's relative pose and carries its mass.
  This is a **simulation attachment model**, not demonstrated finger force closure.
- Existing `/objects` detections come from simulator state. They can support an
  explicitly labelled manipulation fixture, but final mission goals/perception
  must use measured RGB-D/LiDAR data and estimated transforms. Truth stays in
  scoring. The final demo must not pin the base or teleport boxes.
- The old transport scene has decorative, non-colliding tabletops and a single
  cube. Replace those with physical tables, real collision geometry, several
  matched source/destination pairs and measured object/surface perception.

## Milestones and acceptance gates

| Milestone | Implementation | Required evidence before completion |
| --- | --- | --- |
| M0: reproducible platform | Worlds, CLI, GUI, versioned datasets and source patches | Build/tests, browser check and honest baseline records; implemented in the existing milestone commit |
| M1: navigation reliability | Resolve arrival, command-response, route-progress, timing and avoidance failures | Every requested goal reached, zero contacts, valid sensor-only trials across the six-world/three-seed/four-person matrix; repeat on held-out seeds |
| M2: transport environments | Physical tables, lightweight boxes, colored/identified empty squares, furniture and clear aisles | All models load; every pairing is unique; boxes rest on supports; robot and loaded-footprint routes are connected |
| M3: verified manipulation | Measured target pose, observed-free docking pose, stable arm authority, grasp/lift/place verification | Repeated stationary pick/place for every object type, no pinning, no teleportation, no false-success attachment or wrong placement |
| M4: mobile transport | Shared navigation intent, precise docking, carry posture, payload-aware clearance and placement | Multiple complete deliveries with real physics in an empty-person control condition, without drops or falls |
| M5: dynamic missions | Integrate moving people, intrusion handling and bounded recovery | Complete all requested deliveries across representative worlds/seeds with zero person contacts, drops, wrong destinations or invalid sensor intervals |
| M6: release evidence | Updated GUI, JSON/CSV/replay, runbooks and convenience commands | Repeated acceptance runs from the published source snapshot and a clean-checkout build; commit and push verified milestones |

Per the user's navigation-first direction, further autonomous box-transport
integration is gated on M1, including held-out-seed verification. Existing fixtures
and manipulation interfaces remain preparatory work. Here, complete navigation
acceptance means every requested goal, zero contacts, valid sensing and every
strict validator check passing; it does not mean a guaranteed 100/100 composite
score, which also includes speed and efficiency.
Acceptance thresholds must not be relaxed to make a failing run pass.

### Navigation investigation: 22 September 2026

- The six-trial `alignment-cost-v25` validation recorded one strict pass,
  five collisions and four of eighteen requested goals reached. The courtyard
  seed-42 trace showed avoidance reversals followed by an emergency stop in an
  approaching person's path; all 18 evaluated proposals at elapsed 16.9 seconds
  were rejected for predicted body collision, and the search budget was exhausted.
- A separate deterministic regression reproduced candidate starvation in the
  budget scheduler: fixed-stride rotation examined only two of twelve optional
  directions when one slot was available, and none when priority checks exhausted
  the budget. The scheduler now advances by actual evaluated work and reserves
  one rotating proposal beyond priority checks. The soft deadline can therefore
  overrun by that proposal's evaluation cost. Collision checks are unchanged.
- All **210 bridge tests passed**, including both new starvation regressions
  (which failed before the fix). This verifies search coverage, not collision-free
  navigation.
- The scored `fair-search-v26` pilot used four people, three goals, seed 42,
  a 90-second deadline and the existing unsynchronized crowd startup:

  | World | Goals reached | Contacts | Strict result |
  | --- | --- | --- | --- |
  | Courtyard (`small_open`) | 1/3 | 1 | Fail |
  | Station (`medium_open`) | 3/3 | 0 | Pass (86.743/100) |

  Full records: [v25 validation](results/avoidance_v25_validation.json) and
  [v26 pilot](results/avoidance_v26_pilot.json). These different-sized samples
  and unsynchronized startup do not establish a causal improvement. M1 remains
  open. Next investigate command reversals, measured response and predicted
  clearance before the courtyard collision, then repeat the full validation.

`scripts/inspect_social_trace.py` accepts `--start`, `--end` and `--candidates`
to inspect a time window with candidate rejection reasons and planning timings.

### Continued navigation investigation: 23–24 September 2026

**M1 is still incomplete.** The following scored experiments retain every failure.
All used four people, three requested goals and a 90-second deadline per trial,
with unsynchronized crowd startup. Different-sized subsets are not directly
comparable; even matched seeds can have different startup/map/goal timing.

| Revision / dataset | Strict passes | Goals reached | Contacts |
| --- | --- | --- | --- |
| [v27: short holonomic routes](results/avoidance_v27_validation.json) | 3/6 | 15/18 | 2 |
| [v28: executable-clearance soft costs](results/avoidance_v28_validation.json) | 4/6 | 15/18 | 2 |
| [v29: progress balance, full matrix](results/avoidance_v29_matrix.json) | 12/18 | 50/54 | 4 |
| [v30: coarse escape coverage](results/avoidance_v30_validation.json) | 2/6 | 14/18 | 4 |
| [v31: bounded speed and stronger smoothing](results/avoidance_v31_validation.json) | 5/10 | 25/30 | 5 |
| [v32: two-phase route candidates](results/avoidance_v32_validation.json) | 5/8 | 20/24 | 3 |
| [v33: growing forecast reserve](results/avoidance_v33_validation.json) | 4/8 | 18/24 | 4 |
| [v34: extra escape search](results/avoidance_v34_validation.json) | 2/8 | 17/24 | 6 |
| [v35: goal-crossing progress](results/avoidance_v35_validation.json) | 3/8 | 18/24 | 5 |
| [v36: full-horizon recovery, full matrix](results/avoidance_v36_matrix.json) | 12/18 | 49/54 | 5 |
| [v37: response calibration, full matrix](results/avoidance_v37_matrix.json) | 9/18 | 41/54 | 8 |

At this checkpoint the best complete cohort was v29. V36 additionally included
the combined translation cap, two-phase candidates and distinct prediction and
preferred-clearance buffers. Its failures were five contacts and one validator
rejection. V37 regressed, including one timeout, so its 0.30-second response delay
and lateral/yaw coupling overrides were removed from the active profile, restoring
v36's control settings. The optional coupling model remains available for further
calibration. Stronger smoothing, growing forecast margins and the expensive
escape-search fallback were also not retained as active changes. No all-pass or
held-out-seed claim is supported.

Retained implementation changes:

- Short routes up to 3 m can use holonomic motion rather than waiting through a
  stationary pivot; forward-facing travel remains available for longer routes.
- Instantaneous-command CPA soft penalties are weighted by executable swept
  clearance; the complete braking path still undergoes collision checks.
- The coarse escape directions are checked before rotating fine refinements.
  Perpendicular proposals preserve direction while using available axis limits.
- Two-phase translation/route candidates retain lateral and angular continuation
  components through normalization, simulation and debug reporting.
- Safe trajectories receive goal-progress credit when passing through the goal,
  rather than being penalized solely for continuing beyond it in the forecast.
- Planner and supervisor share a 0.75 m/s combined command-speed cap. This is a
  command constraint, not a certificate of actual robot peak speed.
- Physical body radii are unchanged. The prediction buffer is 0.25 m and the
  preferred clearance reserve is 0.40 m, allowing full-horizon recovery before
  considering a shorter escape. Scorer contact/score thresholds are unchanged.

Isolated official-policy lateral tests (`--duration 6 --stop-duration 4`,
`--cmd-accel-y 1.20`, empty scene) found:

| Lateral command | Motion onset | Steady response magnitude | Yaw rate | Stop drift | Fixed-command validation |
| --- | --- | --- | --- | --- | --- |
| +0.45 m/s | 0.449 s | 0.374 m/s | +0.046 rad/s | 0.217 m | Fail: exceeds 0.20 m drift limit |
| −0.45 m/s | 0.380 s | 0.398 m/s | −0.038 rad/s | 0.139 m | Pass |
| +0.60 m/s | 0.358 s | 0.423 m/s | +0.104 rad/s | 0.196 m | Pass |

These single-axis tests expose response delay, stopping and cross-axis drift,
but do not establish a calibrated mixed-command model. Increasing the lateral
command limit was not adopted. Local raw artifacts remain under
`ros2_ws/src/dynamic_obstacle_benchmarks/results/fixed_command/`, trial timestamps
`20260923T230748Z`, `20260923T230846Z` and `20260923T230913Z` (see each result's
embedded provenance). Final checks passed: **222 bridge tests, 82 benchmark
tests and 18 bringup tests**.

Next required evidence: diagnose forecast versus executed motion during the
remaining encounters, including supervisor vetoes and goal transitions; obtain
an all-pass complete matrix and held-out trials. Only then proceed to measured,
unpinned pickup/carry/place integration and scored deliveries.

## Mission state machine

`DISCOVER → SELECT_PAIR → PLAN_STAGING → NAVIGATE_PICKUP → DOCK → VERIFY_CLEAR_WORKSPACE
→ ACQUIRE_ARM → PREGRASP → GRASP → VERIFY_HELD → LIFT → CARRY_POSTURE
→ NAVIGATE_DELIVERY → DOCK_DELIVERY → LOWER → RELEASE → VERIFY_PLACED
→ RETREAT → STOW → NEXT_PAIR → COMPLETE`

- Pair selection uses semantic IDs/colors and observed object/surface poses, not
  simulator coordinates. Initially use distinct red/blue/green box/pad pairs.
- Staging and docking poses must be reachable in observed free space. Keep the
  actual goal distinct from a planner's tolerated path endpoint.
- Share navigation intent with the social planner so task navigation, docking
  and holding cannot silently compete for `/cmd_vel` or suppress avoidance.
- Keep locomotion/balance active during manipulation. Use explicit arm ownership
  and a known carry posture; do not substitute a stationary hold controller for
  mobile walking.
- Before transport, update navigation and person-clearance geometry for the
  carried box. Restore the empty configuration only after confirmed release.
- A successful grasp requires actuator acknowledgement and held-state evidence.
  A successful delivery requires the correct box on the correct pad, released,
  supported and settled; an action server's success alone is insufficient.

## Failure handling

Retain the original mission deadline through retries. Handle stale detections,
blocked staging/docking, people entering the arm workspace, rejected grasps,
attachment loss, unstable posture, placement errors and action cancellation.
Retry only when the sensed physical state supports it. If attachment state is
uncertain, preserve arm authority and enter a recoverable stopped state rather
than silently releasing or declaring success. Every such transition is logged.

## World and perception design

Start with a furnished, wide-aisle room: two pairs of tables with a colored box
or a matching empty square. Expand to a workshop/lobby and the plaza, then to
narrower passages and occluding furniture. Table dimensions and approach
clearances must fit measured G1 reach and the loaded footprint.

Use original procedural assets. Begin measured perception with known-color
segmentation plus registered depth, geometric box/pad tests and stable temporal
IDs. Validate against simulator truth offline, including false detections on
people/furniture, occlusion and camera latency. Keep confidence/age gates explicit.

## Data and usability

Extend `./geoff` with transport-world listing, a one-command transport demo,
scored transport batches and results viewing. Record source/policy/world hashes,
seed, people count, object pairs, phase times, navigation goals, arm actions,
grasp/release acknowledgements, payload state, contacts, drops, placement errors,
timeouts and recovery reasons. Keep oracle fixtures, visual sessions and scored
sensor-only missions in separate datasets.

Each milestone is committed and pushed after its stated checks, with failures
and remaining gates documented. A single successful run does not establish
reliability; the release report must state exactly which matrix passed.
