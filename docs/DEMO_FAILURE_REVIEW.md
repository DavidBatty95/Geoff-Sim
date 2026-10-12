# Demo failure review and improvement paths

Implementation follow-up: [wall evidence, body-fault handling and motion-response
measurements](PERCEPTION_RESPONSE_FOLLOWUP.md).

Review date: 2026-10-07. **The highest priorities are false-person tracking and
avoiding late, unrecoverable pedestrian conflicts.** Increasing walking speed or
reducing the candidate search further is not supported by the retained results.
For task duration, route execution and repeated inspections offer substantially
more opportunity than shaving milliseconds from goal selection.

## Evidence and interpretation

This review uses the [retained room/task outcomes](results/room_search_and_tasks_20261007.json),
[crowd outcomes](results/exploration_crowd_20261007.json), and a new
[incident chronology and phase report](results/failure_review_20261007.json).
Detailed trace analysis covers three crowd failures, a zero-person medium-world
docking failure, and the successful compact cross-colour delivery.

The experiments used evolving code, worlds and execution trajectories. They
identify failure mechanisms and useful experiments, not causal speedup estimates.
Geometric overlap means the evaluator's robot disk intersected a person's disk;
it is not a count of distinct MuJoCo contact events. Receipt-time chronology can
have cross-topic skew, and crossing the recorded fall threshold is later than
the possible onset of instability. A zero requested velocity does not prove the
physical robot has already stopped.

### 1. The crowd failures begin before the reported localization error

Times below are relative to the first trace row, on the logger's monotonic clock.

| Run | People | First geometric overlap | First recorded fall threshold | Interval | Overlap samples before / after fall |
| --- | --- | --- | --- | --- | --- |
| `20261007T105516Z` | 2 | 51.07 s | 60.49 s | 9.43 s | 33 / 44 |
| `20261007T105937Z` | 1 | 45.88 s | 46.72 s | 0.84 s | 10 / 4 |
| `20261007T110220Z` | 1 | 42.17 s | 43.17 s | 1.00 s | 15 / 28 |

All ended with `localization did not recover before deadline`. That terminal
label does not identify the initiating problem. Overlap was already occurring
while the robot had not yet crossed the evaluator's fall threshold.

At the first overlap in every case:

- The latest planner snapshot was `EMERGENCY_STOP`, requesting zero velocity.
- A nearby track was confirmed, with confidence 0.987–1.0 and reported distance
  0.622–0.687 m.
- **All 10, 12 or 13 evaluated candidates were rejected for
  `hard_human_collision`.**
- Reported prediction age was 0.239–0.439 s. The snapshot preceded the overlap
  receipt by approximately 42–63 ms.

Thus, people were not simply invisible at the incident. The planner had reached
a state with no checked collision-free candidate and used its stop fallback.
Existing escape and reduced-comfort recovery logic is present in
`human_aware_motion_planner.py`; the work is to understand why it did not prevent
that state earlier. The chronology does not prove the first overlap caused the
later fall, particularly in the two-person run with a nine-second interval.

### 2. False-person tracking is demonstrated even without pedestrians

The zero-person medium-world run `20261007T095500Z` reported up to **eight person
tracks**. The compact zero-person success `20261007T103313Z` reported none.
The crowd runs reported maxima of eight tracks for two actors and six for one.

There is a concrete implementation mechanism worth isolating:

- `_is_confirmed_person()` in `dynamic_obstacle_tracker.py` allows persistent,
  stationary person-shaped observations to confirm a track. Stationary people
  must remain detectable, so requiring motion for everyone is not a solution.
- `_update_static_background()` skips high-object-score clusters and clusters
  near confirmed tracks. A structural fragment that initially looks person-like
  can therefore avoid accumulating static-background evidence.
- Partial track-conditioned crops help preserve real occluded people, but their
  contribution to maintaining mistaken identities needs a separate audit.

Door/wall fragments and self-returns are hypotheses to test, not classifications
established for every erroneous track. The empty-world result establishes the
false hypotheses; a controlled replay must establish which mechanisms maintain
them.

Offline frame-aligned diagnostics also show that many real close people were
tracked reasonably well: the three crowd cases had 9/107, 1/28 and 3/36 unmatched
samples within 2 m under the existing 0.5 m matching gate. These include occlusion
and are **not visibility-conditioned recall**. Large reported error tails include
the tool's 10 m unmatched sentinel and must not be interpreted as literal
ten-metre localization errors.

### 3. Active planning is slower than whole-run summaries suggest

| Run | Planner p95 before first fall | Planner p95 after first fall |
| --- | --- | --- |
| Two people, `105516Z` | 66.00 ms | 2.81 ms |
| One person, `105937Z` | 56.45 ms | 2.11 ms |
| Reduced proposals, `110220Z` | 60.24 ms | 2.27 ms |

Cheap post-failure stop cycles dilute whole-run timing summaries. Assess control
latency on healthy, active navigation intervals and report p95/p99, input age and
command age separately. The 45 ms budget is soft: mandatory checks can exceed it.
Reducing extra proposals from four to two did not improve the observed outcome
or timing tail and was reverted; these runs are not a controlled ablation.

### 4. Travel, turning and docking dominate completed-task time

In the successful compact red-box → blue-goal run:

| Mission phase | Accumulated time |
| --- | --- |
| Search route execution | 84.21 s |
| Search scans/inspection waits | 35.34 s |
| Pickup route execution | 35.46 s |
| Destination precision docking | 15.86 s |
| Destination inspection | 13.11 s |
| Viewpoint-selection-labelled phase | 1.52 s |

These are mission-phase wall times, including waits, not pure computation
measurements. The delivery took 245.74 s from initial box discovery and 75.51 s
from held acknowledgement to verified placement.

The earlier medium-world failure spent 75.06 s in destination docking and
40.65 s in pickup docking. The subsequently implemented support-edge approach
and inner docking stop band address concrete failures, and the compact pass
exercised them. They are not yet a repeatable medium-world qualification.

## Ranked improvement paths

### P1 — Correct tracking births and static-structure confusion

**Change:** distinguish static structure, generic moving obstacles and human
hypotheses. Audit static-background exclusion, self-filtering, track association
and partial-crop persistence. Add stronger full-body/structural evidence or
onboard RGB-D confirmation for ambiguous births while retaining stationary and
partially occluded people. Preserve generic obstacle protection when a human
classification is uncertain.

**First experiment:** replay zero-person medium-room scans while the robot
walks and turns. Attribute every persistent confirmed track to sensor evidence
and location. Then replay stationary, crossing and partially occluded people.

**Proposed gate:** no persistent confirmed people in the fixed empty-room
regression, without degrading real-person near-field detection. Do not cap
tracks to the simulator's requested population or feed evaluator positions to
the navigation stack.

### P1 — Avoid conflicts before all candidate trajectories become infeasible

**Change:** make early yielding/escape reservation account for measured command
response, stopping distance and pedestrian motion uncertainty. Check whether
false tracks, stale velocity estimates, route constraints or candidate scheduling
remove otherwise usable escape space. Maintain a reachable lateral refuge when
approaching a crossing or being overtaken, rather than assuming stopping alone
will resolve it.

**First experiments:** one walker crossing, overtaking, approaching head-on and
turning back near a doorway. Record first predicted conflict, first avoidance
request, actual body response and the instant the feasible candidate set becomes
empty. Replay the final seconds of the retained incidents with candidate-level
rejection reasons.

**Proposed gate:** five fixed seeds per isolated interaction, with no fall or
geometric contact samples and successful resumption. Retain hard collision
checks; forcing a currently rejected trajectory to win would conceal the issue.

### P2 — Calibrate motion and report physical instability directly

**Change:** measure empty-hand and loaded turning, lateral response, braking,
coasting and settling. Update rollout response models from those measurements.
Use onboard IMU/odometry evidence to distinguish instability/fall from an ordinary
localization interruption, preserving managed cancellation and possession state.

**Why:** stale localization after a fall is a symptom. The current failures spend
about ten additional seconds in localization recovery. Turning and doorway
execution also fail in zero-person runs, so perception fixes alone are unlikely
to establish reliability.

**Proposed gate:** reproducible turn/stop/dock trials before people are added;
the existing final dock and placement tolerances remain the acceptance criteria.

### P2 — Optimize measured hot paths, not candidate count blindly

**Change:** profile tracker clustering/association, prediction preparation,
static-clearance queries and debug publication on healthy active intervals.
Reuse immutable geometry with explicit map/version invalidation and further
batch the measured expensive kernels. Compare with RViz disabled to quantify
observer overhead. Existing batching and asynchronous control separation should
be measured before being redesigned.

**Proposed gate:** meet the current 45 ms planning target at p95 on a fixed
replay and reduce input-to-command age, while retaining all required checks and
unchanged interaction outcomes. Track p99/max and missed control periods too;
an average below budget is insufficient.

### P3 — Search tables and useful views, rather than repeatedly covering floor

**Change:** detect candidate support surfaces independently of a successful
box/pad detection. Rank reachable table-inspection views and room transitions by
expected task information per travel-and-turn time. Keep spatial region identity
stable across map growth where possible; attribute coverage to what was viewed,
not just the region where the robot stood.

**Implementation targets:** `room_search.py`, `exploration_search.py` and
`exploration_mission.py`. Current grouping and deferred-region penalties are
heuristic; candidate potential does not fully represent 3-D occlusion. Viewpoint
admission also suppresses recently visited positions independently of heading,
which can exclude a useful second look from the same doorway.

**Proposed gate:** all four colour assignments on both medium-layout variants,
using only sensor inputs. Compare task completion, path length, turns and
repeated inspections against the current implementation. Whole-building coverage
is not the objective once both required task locations are known.

### P3 — Make task ordering and approach execution time-aware

**Change:** replace the unconditional unknown-goal survey before pickup with a
decision using observed route lengths, visibility confidence and empty/loaded
travel costs. Continue collecting incidental sightings of both task objects.
Score arrival headings using the route's final segment, not just the straight
bearing from the current robot pose, to reduce stop-and-turn inspections.

Keep visually confirmed support-side staging and final reobservation. Improve
local relative visual alignment and settling based on calibration; do not loosen
the final containment/dwell checks to produce a faster apparent success.

**Proposed gate:** reduce completed discovery-to-placement and
grasp-to-placement median/p95 without lowering completion rate or increasing
contacts/drops. Report failures separately, never as short successful episodes.

## Suggested sequence

1. Establish empty-room tracking and actor-free motion baselines.
2. Fix false hypotheses and calibrate response; validate isolated one-person
   interactions with candidate rejection diagnostics.
3. Repeat the compact visual-side delivery across seeds and both cross-colour
   directions.
4. Validate all four assignments and both medium layouts without people.
5. Add one walker, then two. Optimize task routing and latency on those fixed
   scenarios before increasing crowd density or speed.

The proposed gates above are future acceptance targets, not achieved results.

## Reproduce this review

```bash
python3 scripts/review_transport_failures.py \
  ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/20261007T105516Z \
  ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/20261007T105937Z \
  ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/20261007T110220Z \
  ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/20261007T095500Z \
  ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/20261007T103313Z \
  --perception --output docs/results/failure_review_20261007.json
```

This is offline analysis. It uses scoring truth to diagnose incidents, never as
a runtime navigation input. Two synthetic regressions check incident ordering,
pre/post-failure timing separation, and that error labels alone cannot fabricate
fall/contact evidence.
