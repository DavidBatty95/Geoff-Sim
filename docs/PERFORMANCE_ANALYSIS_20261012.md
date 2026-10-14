# Performance analysis: turning, motion quality and exploration

Implementation follow-up: [runtime improvements and retained native validation](PERFORMANCE_IMPLEMENTATION_20261012.md).

Review date: 2026-10-12. This review covers the current working-tree launchers,
navigation/control/perception/search implementations, training configuration,
and retained native evidence. The working tree contains substantial staged
development beyond the README's principal 6 October checkpoint.

**Main conclusion:** the largest opportunities are better physical yaw response,
continuous route execution, and task-directed camera viewpoints. Increasing the
maximum walking speed or optimizing viewpoint selection alone will not address
the dominant delays. Some unreliable motion also occurs without close people,
so crowd tuning alone cannot solve it.

Evidence labels used below:

- **Code:** directly established by the current implementation.
- **Recorded:** measured in retained runs, with their original configurations.
- **Hypothesis:** a plausible causal explanation requiring a controlled trial.
- **Proposal:** an improvement to implement and compare, not a measured speedup.

## 1. Active architecture and configuration

The exploration launcher selects `transport_medium`, one person, a 600-second
budget and 40 viewpoint attempts. It delegates to `run_transport_demo.py`, which
derives settings from the onboard waypoint case, generates an additional
transport navigation profile, and starts the mission separately.

```text
Native LiDAR + measured IMU/joints
    → scan odometry / measured TF → SLAM map
    → person tracking → trajectory prediction

RGB-D + measured robot geometry → confirmed objects / observation memory

Observed-map region/viewpoint search → managed navigation/checkpoints
    → Nav2 global path → custom human-aware command selection
    → collision supervisor → command filter → ONNX policy
    → arm composition → bridge / DDS → MuJoCo actuators
```

The important effective settings are:

| Component | Onboard / exploration behavior |
| --- | --- |
| Global planner | **SmacPlanner2D**, overridden by `scripts/demo_suite.py:301`; the base YAML's Hybrid/Dubins settings are not the active planner choice |
| Nav2 controller | Configured 15 Hz; custom path-reference commands can replace its velocity output |
| Human-aware planner | Configured **20 Hz** in `g1_sim.launch.py:759`, with a **45 ms soft** planning budget |
| Collision supervisor | Configured **50 Hz**, asynchronous escape preparation and command-envelope checks |
| Policy | Configured 50 Hz, asynchronous inference, 2 ms completed-result polling, 50 ms inference deadline |
| Translation | Onboard component limits 0.65/0.45 m/s; people-present transport applies a shared **0.45 m/s resultant cap** |
| Command slew | Translation **2.40 m/s²**, yaw **0.90 rad/s²** |
| Turning | Empty-hand route/scan limits +0.60 / −0.90 rad/s; ordinary loaded turns capped at 0.60 with a 0.35 floor |
| Sensing | Requested 2,400 LiDAR points at 15 Hz; transport RGB-D **192×108 at 2 Hz** |
| Search footprint | Square inflation around a 0.55 m empty / 0.65 m held radius |

Sources: `scripts/run_exploration_transport_demo.py:9`,
`scripts/run_transport_demo.py:46–67,139–167`,
`scripts/demo_suite.py:285–373`, and
`ros2_ws/src/unitree_g1_sim_bringup/config/social_navigation_onboard_v1.yaml`.

Policy identity matters: the ordinary onboard demo uses the official policy.
The retained exploration trials explicitly selected native-delay **cycle-03**.
The exploration wrapper itself does not supply that checkpoint. A run without
`--policy` therefore inherits the official checkpoint despite selecting
`payload_v1` posture. Make policy/profile identity visible at startup and bind
response calibration to its hash, arm state and payload.

## 2. Turning: the highest-impact motion problem

### 2.1 Weak policy response is measured

**Recorded:** two-second native cycle-03 pulses, with 40 ms policy delay, produced:

| Request | Net episode rotation | Maximum tilt | Fell |
| --- | --- | --- | --- |
| +0.60 rad/s | +0.116 rad, approximately 6.6° | 0.066 rad | No |
| −0.90 rad/s | −0.552 rad, approximately 31.7° | 0.099 rad | No |

Source: [turn-transition probes](results/turn_transition_probes_20261007.json).
Net episode rotation includes standing and post-command motion. Different request
magnitudes do not isolate directional asymmetry or give steady-state gains.
Nevertheless, commanded yaw is clearly not equivalent to achieved yaw.

The existing planar predictor already assumes a pure-turn gain of **0.32** and
a walking-turn gain of **0.55**. Under its steady-state assumption, a 90° pivot
takes about **8.2 s at +0.60**, or **5.5 s at −0.90**, before transients. These are
model calculations, not newly measured turn times. Inspection turns have only
12 seconds, including up to two seconds of pre-turn settling.

**Recorded:** cycle-03 improved survival in the matched 32-environment loaded-turn
evaluation, but left-pivot net rotation fell from 1.355 rad to 0.432 rad. The
training report explicitly notes that smoothness did not improve universally at
equivalent tracking. See [native payload contract](NATIVE_PAYLOAD_CONTRACT.md).

**Proposal:** build a policy-specific turn response map, then use bounded
feed-forward compensation plus measured yaw-rate feedback, a no-progress
detector, anti-windup, and measured deceleration near the heading target.
Heading feedback already exists; the missing piece is a calibrated dynamic
response rather than another heading-error multiplier. Fit empty-hand,
prescribed-arm and genuinely loaded conditions separately.

Measure both signs at equal magnitudes, initial speeds, 30°/60°/90°/180° goals,
walking arcs, reversals, walk→turn→walk transitions and stop requests. Record
latency, achieved yaw, translation drift, overshoot, settling, tilt and contacts.
Use the deployed command chain as well as synchronous native probes.

### 2.2 The route controller has a discontinuous 20° alignment gate

**Code:** `forward_reference.py:14–49` sets translation to zero outside ±20°.
Inside that gate it resumes `speed*cos(heading)^2`; clockwise yaw also switches
between a walking cap and a much larger pivot request.

A direct evaluation of the current function with speed 0.45 m/s gives:

| Heading error | Forward request | Yaw request |
| --- | --- | --- |
| +19.9° | 0.3979 m/s | +0.350 rad/s |
| +20.1° | 0 | +0.571 rad/s |
| −19.9° | 0.3979 m/s | −0.350 rad/s |
| −20.1° | 0 | −0.900 rad/s |

Execution slew limits moderate some transitions, but noisy route bearings can
still alternate the desired gait between pivot and walking. **Hypothesis:** this
contributes to hesitation and unstable transitions; incident attribution needs
time-aligned state and applied-action data.

**Proposal:** a stateful align/travel controller with separate entry/exit angles,
a continuous speed taper and bounded yaw-rate transitions. Use checked walking
arcs in open space, reserving pivots for tight geometry and large heading errors.
Do not force a minimum forward arc everywhere: that previously caused orbiting
near goals and consumes clearance at doorways.

### 2.3 Policy improvement must reward useful turning as well as stability

Cycle-03 uses native inertials, a fluid-motion penalty, arm transitions, and
0/20/40 ms action delay sampled per episode. That is valuable, but constant
per-episode delay does not reproduce time-varying missed deadlines or irregular
action holds. The actor has five-frame observation history; irregular sampling
also changes the history's physical time span.

**Proposal:** after measuring the deployed timing distribution, train/evaluate
with matched jitter, action holds and turn-transition curricula. Promote policies
using both tracking and stability criteria so standing almost still cannot win
as a smoother turn. Compare a dedicated alternative locomotion adapter only on
the same measured tests; [Grove/WBC-AGILE](GROVE_G1_REVIEW.md) is a candidate, not a
drop-in 480-observation/29-action replacement.

## 3. Smoother translation and route execution

### 3.1 Normal stops currently share the immediate-zero path

**Code:** `_active_command()` marks deadband-zero as stopped;
`_control_tick()` passes this as `immediate_stop`; `filter_command()` immediately
returns zero. Thus a routine goal handoff uses the same command-filter bypass as
a stale-command stop. A direct check takes 0.45 m/s to zero in one filter call,
whereas normal slew at 20 ms would change it by at most 0.048 m/s.

This is a discontinuity in the **request**, not instantaneous physical braking.
Recorded native probes traveled another **0.18–0.289 m** after zero requests,
depending on translation/arm condition. Those probes were short and not loaded
stopping guarantees. See [response follow-up](PERCEPTION_RESPONSE_FOLLOWUP.md).

**Proposal:** explicit motion intent distinguishing normal deceleration/arrival
from urgent stop or invalid input. Plan the normal braking trajectory and include
it in the same collision rollout. Add jerk-limited nominal command transitions;
retain immediate preemption for urgent stops. Avoid adding an unmodeled low-pass
filter after collision checking, which would change the executed trajectory.

### 3.2 Route checkpoints create repeated handoffs

**Code:** `exploration_mission.py:272–347` replans a short observed route, sends its
first checkpoint as a fresh `NavigateToPose` action, and repeats. Navigation
accepts a fresh sensed position within 0.50 m and terminates the managed action;
it does not require the requested arrival heading. A corner can then need a
separate `dock` continuation before the next route is dispatched. Search passes
0.1 s settle time, but action termination, intent changes and alignment still
cost time. Reducing that 0.1 s alone cannot remove the handoff overhead.

**Proposal:** one owned route-following session with progress-based intermediate
checkpoint advancement and replanning within that session. Specify terminal
camera heading independently from transit headings. Smooth corners only within
observed collision-free space, and choose curvature/lookahead according to
measured yaw capability and clearance. Preserve explicit cancellation on object
discovery, faults and mission transitions.

### 3.3 Docking is necessarily abrupt with the current response envelope

**Code:** `_dock_reference()` first requests rotation, then chooses the dominant
translation axis and requests a fixed **0.30 m/s** until a **0.025 m** deadband.
The mission enters a 0.035 m moving-position band, stops, waits up to 1.5 s and
checks the existing 0.07 m settled tolerance. This is discrete, axis-switching
control compensating for observed low-command stalls.

**Proposal:** calibrate short pulses and coast displacement; use bounded
pulse-and-observe alignment or visual relative docking with measured velocity.
Eventually train dependable low-speed motion. Blindly lowering 0.30 can restore
stalling; extra joint-target smoothing can add policy phase lag. Start with the
task-space command trajectory and policy response contract.

## 4. Exploration: optimize finding the task, not covering floor

The current search already has background candidate computation, region
commitment, frontier information, soft visual coverage and semantic hints.
These should be retained while correcting the following specific limitations.

| Finding established in code | Consequence | Recommended change |
| --- | --- | --- |
| `exploration_search.py:175–213`: 0.8 m lattice; candidates within 1 m of the robot or 1.1 m of a visited position are excluded, regardless of heading | Useful second looks from the same doorway are removed; arbitrary grid alignment affects available viewpoints | Candidate state `(x,y,yaw)` with heading-aware inspection memory, doorway/table views and bounded in-place inspection candidates |
| `information_batch()` casts 360° rays; visual inspection uses a 60° cone | Geometric mapping gain is not task-object visibility | Separate omnidirectional LiDAR mapping gain from directional RGB-D information |
| `RoomTopology.decide():154–169` counts nearby unseen points without line-of-sight checking; actual `visible()` does check line of sight | Predicted useful view can look toward occluded floor | Apply sensor visibility checks when scoring candidate headings, not only when recording coverage |
| `visible()` uses `segment_free()` on a footprint-inflated traversability map | Camera visibility is conflated with where the robot can travel; inflation can hide visible areas in the coverage proxy | Separate navigation clearance from optical visibility; progress toward depth/height-aware occlusion |
| Support extraction in `transport_perception.py:120–146` runs only after a box/pad candidate exists | Tables without detected target objects cannot guide search | Detect and remember candidate support surfaces independently; prioritize uninspected reachable table faces |
| View yaw/cost is initially based on straight bearing from the robot; route corners are not included | Arrival can require an avoidable stop-and-turn | Estimate travel time and accumulated turns from the feasible route, including its final tangent |
| Region geometry is rebuilt; region inspection counts depend on where the robot stood | Region split/merge and repeated proxy coverage can waste inspections | Stable overlapping region/portal identities; credit actually observed surfaces and track failed viewpoints by reason |
| Coverage uses the current pose/heading with `last_image_stamp` | During residual or avoidance motion, the pose used for coverage can differ from acquisition pose | Store image-stamped camera pose and valid visibility evidence with every inspection |

Use a task-conditioned objective such as:

```text
expected useful target information × probability of reaching the view
-----------------------------------------------------------------
measured-model travel time + turn time + expected observation time
```

Add a bounded risk cost and uncertainty in reachability/time. The first version
can be deterministic and geometry-based; a learned semantic prior is optional.
Once the box and destination are known, plan completion rather than continued
coverage. Compare survey-before-pickup against pickup-first using observed route
costs, loaded maneuverability and camera occlusion; the current unconditional
unknown-destination survey may be appropriate in some layouts but is not optimal
by construction.

### Reachability and computation

The search uses four-connected flood fill and square footprint inflation.
Its path length is a grid travel proxy, not the length/time of the subsequently
executed smoothed route. Square inflation is conservative relative to a circular
radius and particularly restricts diagonal clearance near furniture.

Use a shared, calibrated footprint contract across search, local planning and
payload state. Consider circular/distance-field inflation or a posture-aware
swept footprint, rather than simply decreasing clearance. Distinguish an
uncertain/blocked start from a truly unreachable goal.

The background worker rebuilds inflated geometry/topology approximately once a
second while active, and foreground revalidation performs another flood fill.
`route_towards()` can repeat reachability calculations. Cache immutable map
geometry/topology by map version and payload geometry; recompute robot-dependent
reachability as needed. Do not equate the existing target/generation key with a
map-version key: current revalidation checks reachability but can reuse older
information gain/topology for up to the result-age bound.

The worker is a Python thread. NumPy work can release the GIL, but Python flood
fill and geometry loops still contend with foreground Python execution. Profile
before moving kernels to native code or a separate process.

## 5. Perception, localization and crowd behavior

### False people create both behavioral and computational load

**Recorded:** an empty medium-room run produced eight person tracks. The later
matched-duration wall-evidence pilots reduced peak/mean tracks from 7/2.44 to
3/0.65, but did not eliminate false tracks; planner p95 was worse in the enabled
pilot. These are trajectory-dependent observations, not an established speedup.

**Code:** the active object-motion branch skips `_update_static_background()`.
Wall evidence is already implemented, so recommending merely enabling that
feature or tuning the inactive background branch would miss the remaining issue.

**Proposal:** fixed scan replay with per-birth provenance; audit structural
fragments, self-returns, association and track-conditioned crops. Distinguish
generic obstacles from confirmed people and use additional appearance/shape
evidence for ambiguous births. Retain stationary-person detection. False people
can trigger unnecessary turns, caution, alternative routes and more scoring.

The retained conflicts often reached a state where every candidate was rejected
for hard human collision before overlap. Improve early crossing/overtaking
decisions using measured response and an escape route that remains feasible;
stopping in a faster pedestrian's path does not guarantee separation. The
six-second overtaking extension is already present but opt-in, with unsuccessful
native pilots. It needs isolated fixed-route evaluation before broader adoption.

### Localization publication is coupled to accepted scan registration

**Code:** `onboard_lidar_odometry.py:76–170` publishes odometry and transformed
tracking clouds only after successful scan registration. Even though IMU data
arrives independently, navigation yaw/velocity updates are scan-paced. Failed
registration therefore interrupts both navigation feedback and downstream
tracking freshness. IMU matching uses LowState receipt time; velocity smoothing
adds a 0.12 s time constant. Scan keyframes refresh after 0.45 m translation,
with no explicit angular refresh criterion (`lidar_odometry.py:123`).

**Proposal:** quality-tagged high-rate inertial/kinematic propagation with scan
corrections, explicit covariance/freshness semantics and calibrated acquisition
timestamps. Evaluate keyframe refresh on turn/overlap change. A propagated pose
must not be labeled a new accepted LiDAR observation. First measure registration
acceptance during turning and doorway occlusion to establish whether this is a
material cause of the user's observed pauses.

### Camera latency adds unavoidable inspection time at the current rate

Two post-inspection images at 2 Hz nominally require approximately 0.5–1.0 s
from an arbitrary request phase, plus acquisition/processing/delivery delay.
`rotate()` also waits one second after reaching the heading. Earlier retained
complete transport runs had processed-image age p95 around 1.26–1.27 s; those are
different runs from the latest turn failures.

First measure acquisition, delivery, TF/state wait and detection separately.
Use image-stamped observation coverage during transit and stop only for views
requiring reliable confirmation. Test adaptive camera resolution/rate and faster
connected-component/support fitting. Higher camera rate without compute headroom
can make the locomotion timing problem worse.

## 6. Runtime timing: measured tails matter

The retained `20261007T150103Z/response_diagnostics.json` contains:

| Measurement | Median | p95 | p99 / maximum |
| --- | --- | --- | --- |
| Active planner wall time | 17.13 ms | 46.77 ms | p99 63.83 ms; max 93.42 ms |
| Policy submit→target publication | 9.55 ms | 26.30 ms | p99 38.18 ms; max 50.01 ms |
| Supervisor tick interval | 20.04 ms | 37.55 ms | p99 49.95 ms; max 95.67 ms |
| Odometry source age at logger receipt | 119 ms | 200 ms | p99 218 ms |
| Track age at command publication | 296 ms | 518 ms | p99 1.35 s |

There were 11 planner-period overruns in 276 active plans, two recorded policy
expirations, and two stale-result rejections. Track/scan age summaries include
startup/failure/stop states; the extreme tails are not proof that motion was
authorized with those old inputs. Publication timestamps are upstream of actual
actuator application, and topic percentiles must not be added as though they
formed a single causal transaction.

**Proposal, in order:**

1. Record observation→inference→composed target→bridge→applied actuator sequence
   IDs, source times and initial dynamic state around turns. Existing planner /
   supervisor lineage is useful but stops short of motor application.
2. Profile healthy active intervals. Candidate scoring's recorded static-query
   component reaches about 19 ms p95; reuse immutable distance fields and batch
   queries with explicit map/scan version invalidation. Tracker decoding and
   clustering are also worth profiling; the current tracker still iterates
   points in Python.
3. Compare identical workloads with RViz/viewer/debug publication reduced. Keep
   latest-only sensor delivery, bounded inference queues and existing worker
   separation. They are already implemented.
4. Benchmark CPU affinity/priorities for physics and policy, then adjust heavy
   perception placement and native kernels. Existing planner/controller launch
   already limits BLAS/OMP threads to one.
5. Verify simulation-time versus wall-time effects. Runtime uses wall-clock ROS
   timing without synchronized simulation `/clock`; slow physics changes policy
   action duration in simulated time. Instrument before adopting a timing redesign.

The stable synchronous replay of a failed command timeline does not prove that
ROS jitter caused the original fall: initial state, contacts, actors and actual
action timing differed. Preserve that distinction in diagnosis.

## 7. Where the task time is actually spent

The successful compact cross-colour delivery had the following accumulated
mission-phase times, from [failure review](DEMO_FAILURE_REVIEW.md):

| Phase | Time |
| --- | --- |
| Search route execution | 84.21 s |
| Search scanning / inspection | 35.34 s |
| Pickup route execution | 35.46 s |
| Destination docking | 15.86 s |
| Destination inspection | 13.11 s |
| Viewpoint-selection-labelled phase | 1.52 s |

These are phase wall times including waits, not independent CPU measurements.
The medium-world task has no verified delivery in the reviewed 7 October search
cohort. Improving a failed mission's elapsed time is not a completion speedup.

This distribution favors fewer unnecessary journeys, turns and stops over
micro-optimizing the already-backgrounded viewpoint computation.

## 8. Prioritized implementation and evaluation plan

| Priority | Work package | Main files | Acceptance measurement |
| --- | --- | --- | --- |
| P0 | Freeze effective policy/profile and instrument applied actions | launch, manifests, controller, compositor, bridge/native loop | Known policy hash and resolved parameters; attributable timing through actuation |
| P1 | Calibrate yaw/stop response; add hysteretic, continuous align/travel behavior | `forward_reference.py`, `motion_envelope.py`, planner | Both-sign heading trials: lower median/p95 time and overshoot without more tilt, drift or failures |
| P1 | Separate nominal deceleration from urgent stop; improve route continuity | `policy_safety.py`, `transport_demo.py`, `exploration_mission.py` | Fewer stop/start transitions and lower measured acceleration/jerk on fixed routes, same endpoint accuracy |
| P1 for people-present runs | Eliminate persistent false-person hypotheses and intervene earlier | tracker, wall evidence, planner | Empty-room precision plus stationary/occluded-person recall; fixed crossing/overtaking completion |
| P2 | Orientation-aware, occlusion-aware, table-directed search | `exploration_search.py`, `room_search.py`, perception | Lower discovery/delivery time, route length, total turn angle and repeated views across fixed tasks/layouts |
| P2 | Reduce timing tails and scan-coupled feedback gaps | odometry, scoring kernels, runtime scheduling | Lower active p95/p99 input age and deadline misses; unchanged detection and motion outcomes |
| P2 | Calibrated precision approach and pulse/coast docking | planner docking reference, transport mission | Fewer corrections and lower settled docking time at existing final tolerances |
| P3 | Policy retraining / alternative backend comparison | training and policy adapters | Joint tracking/stability/latency evaluation, then full task repeats |

Start with actor-free turn/transition and doorway routes, then one isolated
pedestrian encounter. For exploration, evaluate all four colour assignments on
both medium-layout variants before returning to randomized crowd search. Keep
the executable, policy, geometry and resolved configuration fixed within each
A/B comparison. Separate random pedestrian seeds from repeated zero-person runs.

Record completion/failure rate, time-to-target, total path length, accumulated
absolute yaw, stationary time, repeated viewpoints, useful observations per metre,
turn overshoot, measured motion smoothness, contacts/falls/drops, and active-loop
deadline/input-age distributions. Use paired repeats and distributions rather
than a single visually pleasing run.

## 9. Verification performed for this review

- Read current source and launch overrides, portable reports, cycle-03 training
  manifest, and retained native run settings/timing summaries.
- **36 focused tests passed**: forward reference, policy safety, exploration
  geometry and room search.
- Directly reproduced the ±20° command discontinuity and immediate-zero filter
  behavior; these are software checks, not new robot-motion measurements.
- ROS-dependent search/reactivity/inference/recovery tests could not collect in
  the host environment: generated `unitree_social_navigation_msgs` was unavailable,
  and the existing verification install references container/build paths missing
  on the host. Those suites are not claimed as passing in this review.
- No new native performance cohort was collected. All native values above are
  explicitly retained evidence; proposed improvements still need matched runs.

The first implementation milestone should combine **measured turn calibration,
continuous alignment transitions and uninterrupted route execution**. That
directly targets the reported symptoms and improves the substrate on which
better exploration depends.
