# Motion and exploration performance implementation — 12 October 2026

Follow-up to the [performance analysis](PERFORMANCE_ANALYSIS_20261012.md).

**Measured progress:** the native medium-building unknown-environment mission has
completed red-box → blue-goal delivery, with fresh visual placement verification,
arm restoration, no fall/drop, and zero recorded person-contact samples. A later
development trial completed in **202.48 seconds**, using five box-search
inspections and one destination-search inspection. These are development trials
with evolving code, not a controlled speedup estimate or general crowd acceptance.

People-present trials still include collisions and falls. The locally available
native-contact cycle-05 checkpoint is now preferred for exploration, with an
explicit policy-hash-bound response profile. It has complete empty-room compact
and medium-building deliveries, including a mirrored red → red delivery, but is
not a collision-free crowd qualification. The grasp model is still the existing
capture-radius weld. [Portable attempts and native probes](results/performance_development_20261012.json)
retain the unsuccessful candidates and intermediate implementations.

## 13 October navigation-stability follow-up

The retained falls `20261012T164043Z`, `20261012T165128Z`, and
`20261012T171203Z` include translation reversals or sideways escapes around
inspection/braking transitions. In the last run, the planner selected −0.45 m/s
reverse travel during a scan and subsequently +0.325 m/s forward travel during
braking. These traces identify command churn preceding instability, not a
complete causal explanation of every fall. The loaded-retreat failure
`20261012T164223Z` also became unstable after zero commands and remains a separate
qualification concern.

The planner now evaluates an executable scan/brake reference **before** continuing
an old escape or optimizing social comfort. It prefers that reference only when
the full response-aware rollout passes collision checks and retains the existing
nominal clearance reserve. If unsafe or too close, normal avoidance still applies.
Accepting the reference clears obsolete escape/warm-start plans. This behavior
uses the existing `prefer_safe_route_alignment` flag enabled by the onboard
profile; runtime decisions still use onboard measurements.

Ten new regressions cover scan/brake priority with empty and loaded footprints,
an existing escape, incoming people, static collision, and insufficient clearance.
The full bridge suite passed **532 tests**. Native follow-ups used seed 48,
zero people, the cycle-05 policy, and the existing 600-second deadline:

| Run | Task/layout | Outcome |
| --- | --- | --- |
| `20261013T072558Z` | Blue → red, layout 1 | **Complete in 389.19 s**, 15 box and 3 destination inspections; no fall/drop/recorded person contact. Pickup, loaded navigation, placement verification and arm restoration all completed. |
| `20261013T073302Z` | Blue → red, layout 0 | Upright, no drop/recorded person contact, but **mission failed** after two navigation failures and no reachable useful search viewpoint. Ended before pickup, so it did not retest the loaded-retreat incident. |

The first run also exercised outward-shoulder IK: an unreachable initial reach
triggered redocking, followed by successful pickup and placement. It is an
end-to-end development pass, not an exact replay of the earlier arm failure.
[Portable 13 October evidence](results/performance_development_20261013.json)
retains both outcomes and source identities. These two runs do not establish a
general fall-rate reduction or all-layout reliability. Remaining priorities are
search/navigation dead ends, residual false person tracks, and repeatable loaded
transitions.

### Search dead-end investigation and turn-displacement guard

The `20261013T073302Z` dead end was not exhausted visual coverage: the trace
still reported more than 140 prepared viewpoints around 100–120 seconds.
Nav2 repeatedly reported **starting point in lethal space** while the robot
waited near odom `(4.35, 1.67)`. The subsequent inspection moved it roughly
three metres toward the outer wall. By about 140 seconds, nearby sensed static
points were only approximately 0.27 m away and search produced zero candidates.
The existing physical-clearance recovery could not admit a route from there.
False stationary person tracks remained around the doorway during the initial
planning failure; costmap-layer attribution is not established by the retained
trace alone.

`TransportMission.rotate` now detects more than **0.35 m of fresh measured
displacement during the active turn**, requests checked braking, and reports
`turn_displaced`. Optional exploration treats this as a partial inspection and
can replan from the new measured viewpoint. Confirmed discovery survives braking.
This is an early-abort condition, not a guaranteed physical displacement bound:
coasting during braking, stale odometry, and loss of balance remain limitations.
Three regressions cover braking before intent revocation, stale-pose rejection,
and discovery after a displaced optional inspection. **535 bridge tests passed.**

Two additional layout-0 blue → red runs retained the original seed and deadline:

| Run | Outcome |
| --- | --- |
| `20261013T140530Z` | Fell after an inspection turn, during zero-command settling; 13 box-search inspections, no pickup. |
| `20261013T141134Z` | Fell during route navigation; five box-search inspections, no pickup. |

Neither run emitted `turn_displaced` or completed delivery. The guard is therefore
unit-verified but **not yet exercised in native recovery**, and the search
dead-end fix is not established end to end. Both failed runs are included in
the 13 October portable evidence. The immediate blocker is repeatable
locomotion/turn-to-rest stability; clearing observed obstacles or relaxing
search-clearance requirements would not address that failure.

### Controlled turn-to-rest probes and recovery observability

The next investigation retained **25 native MuJoCo 3.3.6 probes**, all upright:

| Diagnostic group | Runs | Conditions |
| --- | --- | --- |
| Extended turn/stop | 8 | ±0.6 yaw, zero/40 ms action delay, abrupt/ramped command transitions; 5 s standing, 6 s command, 14 s post-command observation |
| Brief inference outage | 8 | Positive 0.6 yaw and 40 ms delay; a 60 ms outage at 9.0, 10.8, 11.2 or 12.0 s; compare history/action reset with measured-joint hold against retained history/targets |
| Observation cadence | 8 | Both turn signs; repeating 40/2/18 ms and 40/10/10 ms intervals, plus constant 30 ms and 40 ms intervals; zero added action delay |
| Incident-command window | 1 | Final turn command window from `20261013T140530Z`, starting 161.5 s after its first moving command; 40 ms delay, neutral initial state |

The fixed-command groups reached at most approximately **0.128 rad tilt** and
**0.086 m post-command displacement**. These are isolated dynamics diagnostics,
not exact-state incident replays or ROS acceptance runs. In particular, the
outage comparison changes both history treatment and hold target; it does not
identify their individual causal contributions. Neither ordinary stopping, one
brief outage, cadence variation alone, nor the replayed command window reproduced
the mission failure. The runtime policy/recovery behavior was therefore not
retuned on the basis of those hypotheses.

`probe_native_policy.py` now accepts `--outage-at`, `--outage-duration`,
`--outage-reset-history`, `--outage-measured-hold`, `--observation-steps` (2 ms
physics steps), and `--replay-start`. It refuses to overwrite an existing probe.
`calibrate_native_turns.py` adds `--duration`, `--command-duration`, and `--ramp`.
A representative extended matched pair is:

```bash
docker compose run --rm --no-deps unitree-g1-sim \
  /workspace/scripts/run_mjlab.sh --no-sync --with mujoco==3.3.6 --with onnxruntime \
  python /workspace/scripts/calibrate_native_turns.py \
  --policy /workspace/vendor/mjlab/logs/payload_training/native_contact_cycle_05_20261012/policy.onnx \
  --output /workspace/ros2_ws/src/dynamic_obstacle_benchmarks/results/my_turn_stop_pair \
  --magnitudes .6 --duration 25 --command-duration 6 --delay-steps 2
```

The controller now retains the latest 16 hold-entry events with ROS/monotonic
times, device tick, pre-hold command/action norm, measured tilt, target jump and
history-reset flag. Existing periodic diagnostics carry this bounded history so
brief holds are visible. The trace analyzer deduplicates repeated histories while
retaining restarted sequence IDs at different times. This is target-publication
observability, not a motor-application acknowledgement.

The instrumented follow-up `20261013T144107Z` completed layout-0 blue → red in
**254.09 s**, with four box and seven destination inspections, no fall/drop or
recorded person contact. Pickup, loaded retreat, delivery and arm restoration
all completed. Two recorded stale-result holds produced approximately 0.213 and
0.334 rad target jumps without causing a fall in this run. This further limits
the isolated-timeout hypothesis; it does not prove all recovery states safe.
The full bridge suite passed **536 tests**. All probes and mission outcomes are
retained in the [13 October evidence](results/performance_development_20261013.json).
The remaining causal gap is the combination of prior robot state, observation
timing and downstream actuation during an actual failed mission.

### Inference-dropout target continuity

Instrumented run `20261013T145125Z` (red → red, layout 1, seed 48) fell before
pickup. Three recovery events preceded the failure. The last event reported a
**1.055 rad joint-target jump** from the previous validated target to measured
state hold, while measured tilt was still 0.063 rad. This identifies a concrete
recovery discontinuity, but does not establish it as the sole cause of the fall.

Brief inference deadline/stale-result holds now retain the previous validated
target and matching action/history state. Expired results remain rejected and
command intent is cleared as before. Sustained failures escalate to the existing
state hold after **two inference-deadline budgets** (100 ms with the current
50 ms deadline), measured from the first hold until a fresh action is published.
Changing between deadline and stale-result reasons does not restart that budget.
Sensor staleness, device-clock resets and other error holds retain their existing
state/history reset behavior. Telemetry includes `recovery_kind` and records
escalation even if the fault reason stays the same.

| Run | Change under test | Outcome |
| --- | --- | --- |
| `20261013T145125Z` | Original inference recovery | Fall; last captured hold target jump 1.055 rad |
| `20261013T145430Z` | Target/history retention | Fall; both captured inference holds had zero target jump and no history reset |
| `20261013T145836Z` | Bounded target/history retention | Fall; both captured stale-result holds had zero target jump and no history reset |

All three used the same requested task/layout/seed, but asynchronous trajectories
and shared host compute differ; this is not an exact-state A/B experiment.
**539 bridge tests passed**, including target/action continuity, fallback without
a validated target, preserved sensor-reset handling, and sustained-failure
escalation. The target discontinuity is removed in tests and native telemetry;
**general navigation stability is still unresolved**. The retained failures also
show that eliminating that discontinuity alone is insufficient.

### Combined translation and measurement-delay investigation

Eight additional isolated probes retained the cycle-05 policy and native
MuJoCo 3.3.6 fixture. Four constant diagonal trials used forward 0.5344 m/s and
lateral ±0.37 m/s, with zero or 40 ms action delay. All stayed upright, with
maximum tilt at most 0.143 rad and post-command displacement up to 0.288 m.
A command-window replay of `20261013T145836Z` (start 31 s after first motion,
40 ms action delay) also stayed upright. These trials sampled no non-floor
robot-to-scene contacts; they retain the original probe scene geometry.

Separately delaying the joint/IMU measurements reproduced a failure:

| Measurement delay | Action delay | Command | Outcome |
| --- | --- | --- | --- |
| 20 ms | 40 ms | `(0.5344, -0.37, 0)` | Upright through 20 s; maximum tilt 0.184 rad |
| 40 ms | 40 ms | Same | Fell at 6.58 s, about 1.58 s after motion began |
| 60 ms | 40 ms | Same | Fell at 6.68 s, about 1.68 s after motion began |

Neither failed probe sampled non-floor robot-to-scene contacts. This establishes
combined delay sensitivity under these initial conditions, **not** the latency
or exact cause of a ROS mission fall. `--sensor-delay-steps` specifies measurement
delay in 2 ms physics steps; it is independent of action delay and observation
cadence. The previous 25 probes plus these eight are retained, including the two
new failures.

The controller now records DDS publication-to-callback and publication-to-target
timing for the specific LowState used by each matched inference. Missing, zero
or future timestamps remain unavailable. ROS 2 Humble discards message metadata
in its standard Python executor, so `DDSMetadataExecutor` retains it for the
designated LowState callback while preserving ordinary callbacks. A live DDS
integration test checks both paths. These metrics assume comparable system
clocks and are explicitly **not acquisition-to-motor-application latency**.

The initial metadata attempt `20261013T152017Z` failed startup because the standard
Humble executor supplied only one callback argument; it remains retained as an
invalid-data failed attempt. After the adapter fix, `20261013T152331Z` ran normally
but fell during navigation before pickup. Its 865 matched timing samples showed:

| Metric | Median | p95 | Maximum |
| --- | --- | --- | --- |
| DDS publication → LowState callback | 3.77 ms | 9.46 ms | 19.48 ms |
| DDS publication → policy target publication | 13.61 ms | 32.32 ms | 52.93 ms |

The RMW did not supply usable receive timestamps, so receive-to-callback queue
age remains null. Source publication can follow sensor acquisition, and target
publication precedes composition/bridge/DDS/motor application; those unmeasured
segments prevent equating these results with the injected-delay trials.
**542 bridge tests passed**, including DDS metadata delivery. The next causal
priority is measuring downstream application latency and qualifying robustness
against the now-reproduced measurement-plus-action-delay failure.

### Native command-application timing

`--application-timing` enables opt-in native application sampling and bridge
LowCmd publication brackets. After a usable command fills MuJoCo's actuator
controls, the physics thread records its native receive sequence, wire CRC,
monotonic timestamps and device tick. Formatting/logging happens on the SDK
thread at 10 Hz. Unchanged-content refreshes are distinguished from actual
content transitions; sampling retains the first application time of the latest
transition. Wire messages, watchdogs and control decisions are not altered.

`analyze_command_application.py RUN --output REPORT` joins these records with
LowCmd publication brackets and the policy's wire-float leg/waist target hashes.
Repeated content yields conditional lower/upper bounds rather than invented
unique command IDs. The default matching window is one second; missing records,
CRC collisions and composed arms remain attribution limitations. Device-tick
intervals use simulation time, while monotonic intervals use wall time. Paired
intervals are combined per observation, not by adding unrelated percentiles.

Both retained red → red/layout-1/seed-48 runs fell before pickup:

- `20261013T160731Z`: initial sampler included unchanged-target refreshes (1,591
  samples). DDS receipt → physics application was 3.52 ms median / 14.54 ms p95.
  Its policy-target ages must not be interpreted as first-transition latency.
  An interrupted inspection recovered and continued search before the later fall.
- `20261013T161415Z`: corrected content-transition sampler captured 129 records.
  All matched observed LowCmd content; 14 had multiple possible LowCmd
  publications. 128 also matched policy target content within the window.

For the corrected run:

| Interval | Median | p95 | Maximum |
| --- | --- | --- | --- |
| Native DDS receipt → physics control assignment (wall time) | 2.70 ms | 11.13 ms | 22.91 ms |
| Policy target publication → assignment (content-matched wall time) | 24.28 ms | 45.48 ms | 54.20 ms |
| LowState DDS publication → assignment (paired, content-matched wall time) | 47.67 ms | 78.17 ms | 128.39 ms |
| Observation device tick → assignment device tick (content-matched simulation time) | 40 ms | 90.6 ms | 122 ms |

The conditional policy/observation lower and upper bounds coincide for these
128 matched samples; that does not make content hashes unique wire IDs. The
device-time tail reaches the range where combined-delay probes failed, making
latency reduction and delay-robust policy qualification concrete next priorities.
These runs used shared host compute and enabled instrumentation; there is no
controlled overhead estimate or proof that latency alone caused the falls.
The acknowledgement is **native control assignment before a physics step**, not
a physical hardware motor response measurement or a navigation input.

Verification: **543 bridge tests**, **12 native CTests**, and analyzer regressions
passed. Native changes are exported through `source_patches.py`; pristine patch
application reproduces the reviewed sources. The application summaries, source
hashes, ambiguities and both failed outcomes are included in the portable evidence.

## 14 October actuation-pipeline experiments

Arm action execution now awaits a future resolved by the existing composition/
watchdog timer instead of sleeping in an executor worker for the entire trajectory.
The same core retains measured completion, cancellation, authority, stale-input
checks and trajectory interpolation. Single-threaded live DDS/action tests cover
success, cancellation, stale-input abort and shutdown while locomotion callbacks
continue. Shutdown cancels the active core trajectory before reporting its result.

Two experimental controls are available:

- `--arm-executor-threads 1` selects a single-threaded compositor; the default
  remains **2** because the native comparison did not show an improvement.
- `--arm-repeat-period .02` coalesces identical composed-target refreshes within
  20 ms. Changed targets publish immediately. The timer still runs all freshness
  and action-completion checks before suppression, and unchanged valid targets
  retain a heartbeat. The default remains **0** (disabled) pending broader
  qualification; accepted experimental values are finite and at most 50 ms.

All medium-building comparisons requested red → red, layout 1, seed 48, zero
people, cycle-05 and a 600-second deadline, with native application timing enabled:

| Run | Compositor setting | Policy publication → application p95 | Device observation → application p95 | Outcome |
| --- | --- | --- | --- | --- |
| `20261014T054530Z` | 2 threads, ordinary refreshes | 35.29 ms | 66 ms | Fall during destination survey |
| `20261014T054928Z` | 1 thread, ordinary refreshes | 37.32 ms | 73.8 ms | Fall after initial inspection |
| `20261014T055343Z` | 2 threads, 20 ms duplicate coalescing | 30.27 ms | 56 ms | Fall during pickup approach |

The baseline and coalescing runs respectively recorded **98.44 and 50.70 LowCmd
publications/s**, with consecutive identical-CRC fractions **61.62% and 23.49%**.
Neither trace had gaps in its recorded publication sequence. Policy-to-application
p99 was 49.26 versus 40.07 ms. These are development comparisons with different
asynchronous paths and durations, not controlled speedup or reliability estimates.
The lower traffic and latency tails support continued coalescing investigation;
all three failed missions remain retained.

Two additional compact trials tested broader behavior:

- `20261014T055743Z`: ordinary red → blue transport, `transport_dual`, IK posture;
  stayed upright but failed destination discovery before any arm trajectory.
- `20261014T055934Z`: exploratory red → red, `transport_dual`, `payload_v1`;
  successfully completed the asynchronous `prepare_pickup` arm trajectory, then
  fell during docking before grasp. It does not establish full manipulation
  lifecycle qualification. Its policy-to-application p95 was 68.44 ms, illustrating
  that the favorable medium-room timing is not a universal bound.

Verification passed **549 bridge tests** and **26 diagnostic/orchestration tests**; the
[14 October portable evidence](results/performance_development_20261014.json)
retains all five attempts, publication traffic, settings and matching ambiguity.
Next priorities are delay-robust locomotion qualification and repeatable
end-to-end validation of coalescing, rather than promoting the single-threaded
candidate based on an isolated timing hypothesis.

### Sensor-delay training candidate and qualification

The cycle-05 training recipe exposed application delay and dropped actuator
updates, but not delayed joint/IMU observations. It also used the physically
applied action as previous-action feedback, whereas deployment records the last
published policy action before downstream application. The probe now has an
explicit `--action-feedback {applied,published}` option; historical behavior
remains reproducible. Eight matched convention checks stayed upright, so that
semantic difference alone did not reproduce the failures. Cycle-05 still fell
with published feedback at 40 ms sensor + 40 ms application delay.

`payload_sensor_delay.py` adds coherent per-environment IMU/joint lag to training.
All sensor terms share the same per-episode lag, repeated same-step observation
reads do not advance history, and reset environments cannot inherit old-episode
samples. Commands and published-action feedback stay current. Critic observations
remain separate; the actor retains its **480 inputs / 29 outputs**, five-frame
term-major history, and no world/object/evaluator inputs. The training script
adds `--max-sensor-delay-steps` and `--action-feedback` without changing historical
defaults.

After a 32-environment GPU smoke test, candidate **cycle-06** completed 500 PPO
iterations / 12,288,000 environment steps from cycle-05, using 1,024 environments,
seed 75, native-contact geometry, mixed payload/posture training, 0–60 ms sensor
lag, 0–40 ms application delay, and 5% actuator-update holds. The candidate is
local at `vendor/mjlab/logs/payload_training/sensor_delay_cycle_06_20261014/policy.onnx`,
SHA256 `9c89aebd14ab0277d3efb3437bf802fa11ea0b3b6a149640aa53a46be2e5a7a7`.
Training logs, configurations, checkpoints and source snapshots are retained.

The matched native MuJoCo 3.3.6 qualification used published-action feedback,
40 ms application delay, a 25-second observation window, and unchanged fall
criteria:

| Qualification cases | Cycle-05 upright | Final cycle-06 upright |
| --- | --- | --- |
| Negative diagonal, sensor delay 0/20/40/60 ms | 2/4 | **4/4** |
| Positive diagonal, sensor delay 0/40 ms | 1/2 | **2/2** |
| ±0.6 turns, sensor delay 0/40 ms | 2/4 | **4/4** |
| Prescribed empty-hand carry posture, negative diagonal, sensor delay 0/40 ms | 1/2 | **2/2** |
| Total | **6/12** | **12/12** |

All of these cases sampled no non-floor robot-to-scene contacts. Cycle-06 retained
substantial motion: at 60 ms sensor delay its negative diagonal averaged roughly
`(0.401, −0.343) m/s`, rather than surviving by standing still. Low-delay lateral
response is slower; for example, the fresh-measurement negative-diagonal lateral
rate changed from −0.331 to −0.247 m/s. These are selected deterministic probe
conditions, not a general reliability estimate or an isolated causal ablation of
each training change.

The broader signed-turn sweep then exposed a regression: cycle-06 fell at
**16.08 s**, approximately five seconds after stopping a −0.35 rad/s turn with
fresh measurements. The matched cycle-05 probe stayed upright. Cycle-06 passed
the other 11/12 free/held signed-turn cases. The failed case had no sampled
non-floor scene contact. It must not be used as a valid calibration knot.

Two earlier checkpoints were exported with strict 480/29 layout and ONNX parity
checks using `export_payload_actor.py`:

| Checkpoint | Low clockwise turn | Diagonal, 40 ms sensor lag | Diagonal, 60 ms sensor lag |
| --- | --- | --- | --- |
| 1300 | Upright | Upright | Fell, no sampled non-floor scene contact |
| 1450 | Upright | Upright | Fell; sampled pickup-table contacts make this a scene-contact-confounded trial |

These are development-selection trials, not held-out acceptance. **No candidate
was promoted**, and no cycle-05 response calibration was assigned to cycle-06.
The default exploration policy remains cycle-05. Next work is targeted low-speed
turn/stop retention alongside the demonstrated delay robustness, followed by
complete native qualification and then ROS mission validation. The 14 October
portable report includes all **52 probes** from this investigation, the training
manifests, source hashes and intermediate export identities. Eleven training-
contract tests passed; ONNX export parity errors for the intermediate checkpoints
were below `3e-7`.

## Implemented runtime changes

### Turning and motion transitions

- `forward_reference.py` blends walking into pivoting over 20–35°, removing the
  former single-threshold velocity/yaw jump.
- `NominalMotionReference` adds align/travel hysteresis, bounded acceleration and
  jerk, and a measured-speed interlock before strong pivots. It shapes nominal
  proposals **before** trajectory/collision evaluation; urgent avoidance is
  independently selected and checked.
- The mission's explicit `brake` mode requests gradual nominal deceleration
  through the planner/supervisor. Fault cleanup still immediately clears intent.
  Completion checks use fresh measured velocities and a bounded settling window.
- Inspection turns can terminate as soon as the target is confirmed. Heading
  progress is monitored, and unproductive optional turns are reported instead
  of silently consuming the full turn deadline.
- The stall monitor requires a fresh filtered turn command in the requested
  direction. Deliberate safety stops/sidesteps do not masquerade as weak yaw
  response. Discovery evidence is retained through turn braking as well as
  navigation cancellation.
- Native transport defaults to device-tick observation cadence. The policy's
  20 ms history interval follows the robot's millisecond counter, while command,
  inference and state watchdogs retain wall time. Duplicate/reordered ticks do
  not add observations; resets invalidate pre-reset work. Use
  `--no-policy-device-clock` for the earlier wall-timer comparison.
- Complete observed routes dispatch a single final navigation goal. Nav2 follows
  the path through intermediate corners without a stop/cancel/restart at each
  visibility checkpoint. Partial observed routes still advance only through
  known space and replan.
- The mission publishes circumscribed empty/held navigation footprints to both
  Nav2 costmaps, aligned with the local planner's navigation clearance. Nav2 no
  longer plans a small empty-body route while the loaded controller requires a
  larger clearance.
- The arm compositor publishes a valid newly received locomotion target promptly
  through the same composition core, avoiding another 0–10 ms timer wait. Its
  periodic interpolation/watchdog publication remains active.
- Transport startup now includes one second of ordinary joint-PD standing hold.
  There is no base pinning or robot-pose reset.

### Search and task execution

- Viewpoint novelty includes heading, and the search can inspect a new camera
  cone at its current location without issuing a redundant navigation action.
- Candidate camera headings use line-of-sight checks. Optical visibility is
  evaluated on occupancy, independently of robot-footprint inflation.
- RGB-D perception emits **colour-independent horizontal support-surface
  proposals**. Surface normals reject a height slice through a vertical wall;
  these proposals guide inspection but never establish object discovery.
- Support normals are computed before point reduction so a distant tabletop
  occupying only a few image rows is not erased. Tables already explained by
  another confirmed object receive a lower inspection priority, while remaining
  revisitable.
- Camera coverage uses acquisition-associated camera position, heading and
  calibrated horizontal FOV rather than the robot's later heading and a fixed
  60° assumption.
- Scoring includes travel/turn time proxies. Support views account for both
  travel bearing and final inspection bearing. Coverage bonuses use compatible
  time costs so a high floor-coverage score cannot erase a costly turn.
- Region identities are reconciled by spatial overlap across map updates.
  Inspection positions/headings are reprojected through map-frame corrections;
  large corrections clear the coarse negative-evidence coverage proxy and revoke
  prepared plans.
- Region inspection counts are credited to the floor actually visible in fresh
  images, including views from outside the region. Repeated failed turns at the
  same pose stop generating more in-place turn goals.
- Geometry/topology and repeated reachability results are cached with explicit
  geometry/payload/start-cell keys.
- A confirmed object acquisition that caused navigation to stop is retained
  through braking and managed cancellation. Previously, a subsequent occluded
  frame could lose that discovery and restart a long search. Manipulation still
  independently requires fresh observations at staging.
- Semantic reobservation views retain their full requested facing direction;
  the generic exploration turn cap no longer truncates that direction and then
  abandons a known target.
- When a recent confirmed destination hint and a loaded-clear observed route
  exist, task ordering compares estimated empty/loaded travel costs. Unknown or
  unreachable destinations retain empty-handed surveying.
- If the physical footprint is clear but gait settling has entered the larger
  search reserve, the executive can request a bounded, checked body-axis move
  back into the search-clear area. `reserve_recovery` retains navigation static
  clearance and the existing monotonic-clearance recovery checks. Unknown space
  and the physical footprint are not waived.
- Repeated scans without a reachable useful view now produce a bounded explicit
  failure instead of spending the entire viewpoint budget rotating in place.

### Perception, localization and planning cost

- Tracker point-cloud decoding is vectorized and respects declared fields,
  endianness, row padding and noise tags.
- Sparse upper-wall evidence has a separately bounded 1.5-second onboard history,
  with bounded fitting work and per-observation result reuse. Person-motion
  history is separate; semantic confirmations retain precedence.
- A strict body-height sheet cue covers near walls whose upper sections are
  outside the LiDAR view. It requires a thin, extended plane and vertical support
  in three separated columns; flat torsos, arms and tabletops have regressions.
  Moving confirmed tracks are exempt from this accumulated body-height cue.
  `--no-tracker-body-wall-sheets` provides an ablation. A 90-second zero-person
  pilot recorded peak one/mean 0.071 tracks and 19.76 ms planner p95, with 98
  body-height sheet rejections. This is a development observation, not a
  visibility-conditioned person-recall result or a causal speedup estimate.
- Static distance fields can be reused across ticks only when obstacle contents,
  extent and a padded origin cell agree. Obstacle insertion/removal invalidates
  reuse; out-of-window queries remain conservative.
- Scan-odometry keyframes refresh on sufficient rotation as well as translation.
- A full-band reference is anchored to each accepted pose. If upper-structure
  overlap disappears, full-band registration must independently pass the same
  gates before supplying a pose. Failure of both bands still stops fresh
  odometry; no unobserved pose is published to hide registration failure.
- Dynamic scan filtering has a bounded latest-only wait for acquisition-time TF,
  rather than immediately inserting people into SLAM because TF arrived one
  callback later. Empty-person messages take a fast path. Expiry preserves the
  unfiltered obstacle returns and original acquisition stamp.
- Measured attitude publication is decoupled from successful LiDAR registration
  and capped near 50 Hz. LowState lacks a synchronized ROS acquisition epoch;
  the existing receipt-time contract remains explicit.
- Exploration exposes `--slam-loop-closing` / `--no-slam-loop-closing`. It defaults
  off in the repetitive unknown-building fixture after a retained multi-metre
  SLAM correction disrupted search. Other transport defaults retain loop closure.
  This is an explicit configuration choice, not a general SLAM solution.

### Docking and independent avoidance

- Retreat crosses an inner moving band before checking the original settled
  clearance, avoiding repeated millimetre-scale threshold crossings.
- Docking learns a bounded (at most 4 cm) offset from measured stop-to-settle
  displacement. Final acceptance is still against the original goal and its
  unchanged 7 cm/0.15 rad tolerance.
- Fixed-joint carry posture is checked against the **current waist configuration**.
  The same 35 mm hand-position tolerance remains; a policy-controlled waist
  change no longer makes an otherwise correct joint posture fail against an old
  Cartesian reference. World-relative placement/reach checks remain separate.
- Independent clearance sequences are no longer erased by every repeated idle
  mission heartbeat. Mission-goal plans and idle-clearance plans have distinct
  ownership keys, and continuations remain rechecked each control tick.
- `--predictive-refuge` enables a bounded, experimental multi-stage clearance
  search during inspection stops as well as navigation. It is opt-in. The
  earlier `--early-overtaking-avoidance` switch also remains experimental.

## Policy diagnostics and training

`scripts/calibrate_native_turns.py` runs signed, equal-magnitude native probes,
retaining failures and policy/scene hashes. `probe_native_policy.py` now records
yaw-rate mean/variation and supports a ramped-command transition comparison.

The matched cycle-03 probes showed that increasing positive command magnitude
alone does little for turning: +0.35/+0.60/+0.90 requests produced approximately
0.057/0.077/0.090 rad/s mean yaw rate after the first second of a six-second
request. The −0.60 probe fell after stopping. These are synchronous native
diagnostics, not deployed ROS guarantees.

The reactive training objective adds stronger angular tracking, explicit pivot
translation-drift cost, and per-tick action-hold exposure alongside per-episode
delay. A 300-iteration cycle-04 candidate improved some native yaw rates but
failed a turn probe and did not qualify integrated delivery. It is not selected
as the default.

The subsequent `native_contact` training profile addresses a concrete mismatch:

| Property | Earlier training | Native deployment / new foot profile |
| --- | --- | --- |
| Contacts per foot | Seven capsules | Four spheres |
| Contact radius | 0.01 m capsules | 0.005 m spheres |
| Foot friction coefficient | 0.6 | 1.0 |

The new profile copies native contact placement, size, orientation, friction and
solver parameters, in addition to native inertials and motor limits. Other body
collision assets still differ; complete physics equivalence is not claimed.
Candidate training/evaluation artifacts remain local under `vendor/mjlab/logs/`.

Cycle-05 trained for 500 additional iterations from cycle-03, seed 74, with 1,024
environments, native foot contacts, the reactive objective, 0–40 ms delay and
5% per-tick action-hold exposure. Its ONNX SHA-256 is
`4582897893b9730781b2e2ddd682eb312479184b636582efb4f67d59fc338c6c`.

All **12** signed native turn probes (three magnitudes, policy-controlled and
prescribed held-arm conditions) stayed upright. At +0.60 rad/s, the unheld
mean yaw rate after the first second was **0.465 rad/s**, versus cycle-03's
**0.077 rad/s** in the matched diagnostic. The −0.60 rate was −0.489 rad/s.
These measurements do not establish a sixfold mission speedup.

`policy_response.py` supplies bounded signed interpolation and inverse response
with yaw-rate feedback for this exact policy hash. Planning and supervision use
the same curve, and startup parameter comparison checks profile agreement.
The held-arm curve is a posture proxy; actual payload missions remain a separate
validation condition.

## Reproduction

Current native-contact policy, zero-person medium-building regression:

```bash
python3 scripts/run_exploration_transport_demo.py --people 0 --seed 48 \
  --task red:blue --duration 600 \
  --policy vendor/mjlab/logs/payload_training/native_contact_cycle_05_20261012/policy.onnx \
  --fast-start --no-rviz
```

Omit `--fast-start` to build. With no `--policy`, exploration prefers the
checksum-matched local cycle-05 artifact, falling back to checksum-matched cycle-03
when available. The resolved policy is printed and captured in the manifest.
Checkpoints are local training artifacts, not source-distributed files. Explicit
policy paths are preferable for comparisons. The wrapper still defaults to one
person; use `--people 0` to reproduce the actor-free evidence.

Serial colour/layout matrix, retaining unsuccessful attempts:

```bash
python3 scripts/run_performance_matrix.py \
  --policy vendor/mjlab/logs/payload_training/native_contact_cycle_05_20261012/policy.onnx \
  --tasks red:red red:blue blue:red blue:blue --layouts 0 1 \
  --duration 600 --fast-start
```

`--detach` writes a persistent matrix log and summary path and prints the process
ID. Only one native simulator from this project should run at a time.

Matched native turn diagnostics:

```bash
bash scripts/run_mjlab_docker.sh --no-sync --with mujoco==3.3.6 --with onnxruntime \
  python /workspace/scripts/calibrate_native_turns.py \
  --policy /workspace/vendor/mjlab/logs/payload_training/native_delay_cycle_03_20261006/policy.onnx \
  --output /workspace/ros2_ws/src/dynamic_obstacle_benchmarks/results/my_turn_calibration
```

`summarize_motion_incident.py TRACE --start 30 --end 45 --period 2` provides compact
receipt-aligned command, body-state and tracker snapshots. Evaluator truth in
these offline diagnostics is not a runtime navigation input.

## Verification and evidence status

### Arm self-contact follow-up

The failed mirrored blue → red pickup in `20261012T165825Z` requested shoulder
roll `+0.1526` rad but measured only `+0.0365` rad. Offline native robot-geometry
evaluation of the recorded arm target found **21 mm upper-arm/torso penetration**
and 9.8 mm elbow/torso penetration. The measured configuration lay near the torso
contact boundary. This supports self-contact as the cause of the tracking failure.

Tabletop IK now uses a conservative outward shoulder-roll workspace (right at
most −0.10 rad, left at least +0.10 rad), including clipping the initial solver
seed. The URDF actuator limits and action completion tolerance are unchanged.
This is a workspace restriction, not a full swept-path collision planner.
`scripts/test_arm_ik_native.py` reconstructs the failed target using robot
geometry alone and verifies a replacement with less than 3 mm hand-position
error, preserved orientation, and no torso penetration. Run it with:

```bash
docker compose run --rm --no-deps -e PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  unitree-g1-sim /workspace/scripts/run_mjlab.sh --no-sync python -m pytest -q \
  /workspace/scripts/test_arm_ik_native.py
```

After this change, **522 bridge tests** and the native geometry regression passed.
The first full native retry `20261012T171203Z` fell during navigation before pickup.
The 13 October follow-up above subsequently exercised IK in a completed mission.

Full `test_workspace_inside.sh ci` passed after the implementation checkpoint:
**704 ROS test results**, no errors/failures/skips; **124 orchestration tests**;
**11 native CTests**; strict mock benchmark **99.818**, 2/2 goals, no collisions.
That full checkpoint contained **519 passing bridge tests**. Subsequent focused
regressions cover the additional turn-monitor and distant-table fixes. Eight
training-contract tests passed in the MJLab environment. Mock acceptance
establishes test mechanics, not physical crowd performance.

Key retained native runs:

| Run | Scope | Outcome |
| --- | --- | --- |
| `20261012T122105Z` | Original actor-free medium baseline | Fell during early route execution |
| `20261012T123513Z` | Early continuous-motion/search integration, 180 s | Upright, no contacts; discovery incomplete |
| `20261012T123948Z` | Longer medium search, loop closure enabled | Upright; box found; search lost useful reachability after map correction |
| `20261012T130141Z` | Compact ordinary red → blue transport | Complete verified delivery, no fall/drop/contacts |
| `20261012T130413Z` | Compact exploration, 420 s | Placement verified; deadline during arm restoration; **mission failed** |
| `20261012T131241Z` | Medium unknown-environment red → blue | Complete verified delivery, no fall/drop/contacts; 23 inspections |
| `20261012T132451Z` | One person, early overtaking enabled | Collision samples and fall |
| `20261012T132748Z` | One person, experimental refuge search | Fall without recorded person contact |
| `20261012T133901Z` | Medium red → red matrix attempt | Upright; blocked-search failure; motivated reserve recovery |
| `20261012T134244Z` | Medium red → blue matrix attempt | Complete in **202.48 s**, no fall/drop/contacts; six inspections |
| `20261012T141212Z` | Cycle-05 compact exploration | Complete verified delivery, no fall/drop/contacts |
| `20261012T142827Z` | Cycle-05 medium red → blue | Complete verified delivery, no fall/drop/contacts |
| `20261012T143551Z` | Cycle-05 with one walker, 180 s | Upright and pickup exercised, but 42 contact samples; deadline |
| `20261012T154256Z` | Body-height wall evidence, empty mirrored room, 90 s | Upright; peak one false track; 98 new structural rejections; diagnostic deadline |
| `20261012T155936Z` | Updated mirrored medium red → red | Complete verified delivery, no fall/drop/contacts |
| `20261012T162031Z` | Later default one-person profile | Upright; contacts and required-heading interruption; not a delivery pass |

The first complete cycle-05 colour/layout development matrix,
`performance_matrix/20261012T144001Z`, passed **2/8** before the later footprint,
registration and near-wall fixes. It is retained rather than replaced by the
subsequent mirrored success. The later serial matrix,
`performance_matrix/20261012T163028Z`, completed at **3/8**: layout-0 blue → blue,
layout-1 red → blue, and layout-1 blue → blue succeeded. Failures comprised one
search deadline, three body-instability failures, and the arm tracking failure
analyzed above. This matrix predates the outward-shoulder IK restriction.
Development changes and asynchronous trajectories prevent a causal
percentage-speedup claim from these individual runs.

## Remaining work that is not established by this checkpoint

- Repeatable all-task/all-layout and people-present completion. False person
  hypotheses, unmodeled gait drift and collisions remain in retained trials.
- Broader payload-conditioned identification and uncertainty validation of the
  implemented yaw-response controller. The finite native-probe curve does not
  identify every ROS timing, initial-state and contact effect.
- Actual motor-application acknowledgements and exact-state incident replay;
  current policy timing remains upstream of actuation.
- High-rate uncertainty-aware translational state propagation, full 3-D camera
  occlusion/semantic likelihoods, and system-wide command leases.
- A qualified alternative locomotion backend. Faster unqualified candidates are
  not automatically promoted into task demos.

These boundaries distinguish implemented engineering improvements from the
broader research directions in the original analysis.
