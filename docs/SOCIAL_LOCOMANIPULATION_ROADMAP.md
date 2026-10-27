# Socially aware G1 indoor delivery: staged roadmap

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

Updated 2026-10-06. This is the active implementation roadmap; the
[transport plan](TRANSPORT_DEMO_PLAN.md) retains the historical experiment ledger.

**Reader guide:** [current status and progression](PROJECT_STATUS.md) ·
[detailed deliverables and forward plan](PROJECT_PLAN.md) ·
[repository map](REPOSITORY_GUIDE.md). This page is the technical design and
implementation ledger; the public plan uses the same S0–S7 stage identifiers.

The [P0–P7 research programme](RESEARCH_PROGRAMME.md) adds the scientific delivery
phases aligned with G1–G5: novelty/protocol audit, response identification, causal
diagnosis, calibrated planning, safety–liveness coordination, body/load risk,
human evaluation and repeatable paper release. Its
[crosswalk](RESEARCH_PROGRAMME.md#phase-and-gap-crosswalk) maps every phase to the
S-stage and R-workstream support it needs.

## Target and acceptance scope

Target hardware: **Unitree G1 with BrainCo hands**. First accepted mission:
repeated indoor table-to-table delivery around people, including yielding,
blocked routes, workspace intrusion and autonomous recovery. Specify the exact
hand model/API, sensors, compute, object sizes/masses, floor, lighting and aisle
limits before physical acceptance. General unknown-environment autonomy and
human handover are later extensions.

The robot must discover the requested object and destination from measurements,
select feasible approaches, physically grasp, carry, release and independently
verify delivery. Normal disturbances must not require a mission restart.
Unrecoverable conditions must produce a truthful, bounded safe-hold/assistance
state. No base pinning, object teleportation or mission use of evaluation truth.

## Assessed baseline (retained evidence, not a new validation run)

- Sensor-driven, unpinned transport has documented zero-person native simulation
  passes using a payload-trained policy and capture-radius weld attachment.
  The payload report documents two; two additional local successful artifacts
  exist. These do not form a frozen four-trial reliability cohort.
- Latest full navigation matrix v84: **10/18 strict passes, 46/54 goals, six
  reported contacts**. v85 confirmed geometric overlaps in audited failures.
- A later four-person transport run stopped during reaching without resuming.
- BrainCo articulation/contact-bench work exists; prescribed wrist bench motion
  does not establish mobile physical grasping. The bench and transport fixture
  use different object sizes (5 cm versus 11 cm).
- New sensor-driven transport bypasses the older MoveIt/action orchestration
  stack. Neither stack yet supplies the complete target mission.
- Physical G1 operation is unvalidated. The source publication and historical
  test reports are not a clean-checkout native acceptance release.

Evidence: [payload training](PAYLOAD_POLICY_TRAINING.md),
[v84](results/avoidance_v84_matrix.json),
[v85](results/avoidance_v85_alignment_audit.json),
[architecture](ARCHITECTURE.md).

The [Grove-G1 review](GROVE_G1_REVIEW.md) adds concrete reference designs and a
locomotion-backend comparison gate. Its AGILE controller is a candidate for
isolated evaluation, not a drop-in replacement for the current 480/29 actor.

## Architecture to converge on

Task request → persistent mission executive → uncertainty-aware semantic world
model → navigation/docking/manipulation actions → coordinated base/arm execution
→ final collision and command-authority supervision → locomotion policy, arm
controller and BrainCo controller → simulation or hardware adapters.

- One mission/action lifecycle, with explicit preconditions, postconditions,
  cancellation, bounded retries and recoverable checkpoints.
- Enforced command ownership, readiness-driven startup and measured health.
- Shared timestamped geometry, uncertainty and executable-motion contracts
  between planner and supervisor; independent final command validation.
- Actual articulated arm/payload swept geometry, not just a carrying-radius flag.
- Hand interfaces express supported actuator commands and measured status;
  object-ID weld services remain explicitly labelled development fixtures.
- Acquisition stamps, frame, confidence and validity accompany estimates.
- Simulator truth stays in training/evaluation, outside deployed decisions.

## Stages and promotion gates

Latest [native-contract checkpoint](NATIVE_PAYLOAD_CONTRACT.md): two consecutive
zero-person dual-cube fixture passes with unchanged final code/checkpoint;
bounded localization recovery exercised in the second. Native inertials and
0–40 ms action-delay exposure improved standing robustness. These advance
S1/S4/S5 components without closing their full gates. The subsequent two-world
four-person navigation pilot passed 0/2 strict trials, so S2 and S6 remain open.
The follow-up adds native acquisition-stamped object evaluation, replay rejection,
bounded viewpoint search and observed support-edge pickup approaches. Two later
dual-delivery attempts also passed; cycle 03 totals 4/9 across evolving integration
attempts. Complete contact accounting and broader acceptance gates remain open.

| Stage | Work | Required exit evidence | Status |
| --- | --- | --- | --- |
| S0 Reproducible baseline | Full source/submodule/config/model/policy/build identity; effective parameters; reconcile documentation; native vs mock verification tiers | Clean rebuild reproduces baseline; all attempts retained; source and installed-code identities agree | Open |
| S1 Measurement and lifecycle correctness | Uncertain attachment; post-release evidence; cancel-and-await on all exits; timestamp continuity; evaluator health/contact coverage; command ownership; response/latency calibration | Fault injection rejects false success; bounded termination; measured stopping/prediction envelopes and replayable command decisions | In progress: first bounded fixes only |
| S2 Reliable navigation | Reproduce v85; isolate tracking/model/tactical/veto failures; shared motion contract; earlier avoidance; bounded side commitment; occlusion speed control; runtime profiling | Existing six-world/three-seed matrix passes every strict check, then held-out seeds and encounters; thresholds unchanged | Open; gates crowd-transport promotion |
| S3 BrainCo physical grasp | Confirm hardware/API; calibrate coupling/limits/friction; choose feasible objects; contact-only grasp/lift/hold/release; collision-aware arm approach; measured grasp evidence | Repeated contact-only grasps with no unintended collisions or weld; perturbation/offset/material tests; real-hand comparison when available | Open; isolated work may proceed alongside S2 |
| S4 Recoverable mission | Consolidate orchestration; wait/reobserve/redock/regrasp/replan/safe-hold states; reproject after base motion; destination occupancy; translational search | Recoverable fault scenarios resume; unrecoverable scenarios preserve known state; deadlines remain bounded | Open |
| S5 Smooth loaded transport | Loaded swept geometry; arm/load transitions; noise/delay/dynamics variation; reverse/stop training regressions; precision docking; target-compute validation | Repeated empty-room deliveries across objects/layouts without falls, drops, unintended contacts or resets; declared motion/placement tolerances met | Open |
| S6 Social transport | Crossings, head-on/overtaking, doorways, occluded emergence, workspace intrusion, occupied destinations, groups and repeated missions | Held-out complete deliveries; valid sensing; zero prohibited contacts/falls/drops; measured yielding/resumption, deadlock and human comfort | Open; requires S2–S5 |
| S7 Physical deployment | Same interfaces through calibrated hardware adapters; timing/actuation limits; watchdogs; controlled progression from supported motion to human encounters | Relevant acceptance suite passes on physical G1 within a declared operating envelope | Open |

Stage order is a promotion order, not permission to claim later-stage success
from isolated component work. Freeze each acceptance candidate before measuring
reliability. Retain invalid runs and failures; distinguish development attempts
from comparable cohorts. Commits/pushes remain explicit owner actions.

The [reactive simulation workstream](#reactive-simulation-workstream-s1-s2-s4-and-s5)
provides the detailed backlog: R1/R2 support S1 measurement and ownership;
R3/R5 support S2 bounded response and perception; R4 supports S1 calibration and
S5 locomotion; R6 supports S4 persistent mission recovery.

### Research contribution handoff

| Engineering work | Research consumer | Evidence to retain |
| --- | --- | --- |
| S0/S1 and R1 provenance/measurement | P0/P1/P2 | Frozen identity, timestamp boundaries, evaluator coverage and repeatable interventions |
| R4 response/backend evaluation | P1/P3/P5 | Held-out response, posture/load conditions, matched policy contracts and stopping failures |
| R3/R5 planning/perception work | P2/P3 | Source-matched traces, model/age/uncertainty ablations and compute/exposure accounting |
| R2/R6 ownership and lifecycle | P4 | A fixed execution contract, interruption traces, veto/resume behavior and recovery outcomes |
| S3/S5 articulated loaded motion | P5 | Reference geometry/contact data and decision errors of simpler abstractions |
| S6/S7 qualified interactions | P6/P7 | Declared operating scope, participant protocol where applicable and repeatable final evidence |

Record implementation corrections as baseline work. Promote a scientific claim
only through the [research gates](RESEARCH_PROGRAMME.md), using the unchanged
engineering acceptance criteria for any capability/deployment claim. The P phases
remain planned until their protocol and artifacts are recorded.

## First implementation slice: S1 attachment and placement correctness

Scope:

1. Represent attachment as `empty`, `uncertain` or `held`.
2. Mark a dispatched grasp/release uncertain until acknowledged. A lost or
   rejected acknowledgement must not clear possible possession. Do not retry an
   ambiguous grasp or release automatically.
3. Initialize payload geometry before dispatching grasp and retain conservative
   carrying geometry while attachment is uncertain.
4. Report both confirmed possession and possible possession in telemetry/results.
5. Require a fresh box observation acquired strictly after release acknowledgement
   before accepting visual placement. This is a necessary check, not independent
   proof of release, support, or grasp stability.
6. Add regression tests for lost/rejected acknowledgements, failure before
   dispatch, carrying-state publication and stale/pre-release visual evidence.

The October source publication adds terminal-action, evaluator-health and
source/runtime-provenance helpers beyond this initial slice. These remain
partial implementation progress: actual contact coverage and native lifecycle
qualification are still required, followed by measurement and unification of
planning/supervision/execution. Resumable recovery and physical finger grasping
retain their separate promotion gates.

Implemented follow-up slices are recorded below. The next open S1 work is actual
contact accounting, acquisition-stamped object truth, effective live parameter
capture, enforced command ownership and measured dynamics/latency envelopes.

### Grove-inspired backend and docking gate

Before promoting a new locomotion backend, validate its tensor/joint/gain/history
contract and compare it with cycle-01 on the same measured sensor inputs, arm
transitions, carry poses, payloads, reversals and stops. Record falls, displacement,
settling drift and timing tails. Recalibrate navigation's executable-motion model
for the winner; do not reuse another policy's response constants.

Docking must remain inside its existing tolerances after stopping and settling,
using fresh odometry and a shared original deadline. Camera view feasibility is
part of approach/verification planning, not a reason to accept stale detections.

## Reactive simulation workstream: S1, S2, S4 and S5

Reactivity means timely obstacle response, command onset/stopping, disturbance
recovery and mission interruption. Optimize these together with completion,
clearance and balance: a faster decision that causes a fall or indefinite waiting
is a regression. The work below extends the existing stages; it does not promote
their status or replace the attachment, evaluator and physical-grasp gates.

Keep Geoff-Sim's onboard social planner, prediction pipeline and evidence tools.
Adapt Grove's ownership, execution and cancellation patterns selectively. The
[reviewed Grove revision](https://github.com/Adyansh04/grove-g1/tree/e158588eb6b99264247d64e2d811d48d704e0b67)
has no demonstrated advantage on our crowd benchmark. Its documented gait
deadband, velocity-source arbitration gap and timeout-based base stopping need
independent evaluation rather than adoption as defaults.

### R1 — End-to-end response baseline (S1, highest priority)

After the evaluator/lifecycle correctness slices, trace sensor acquisition → ROS
receipt → tracking → planner → supervisor → policy → applied actuator command →
measured motion. Associate observations, commands and mission generations so a
trace explains which measurement produced each applied response. Keep acquisition,
simulation and monotonic wall-clock timing explicit; account for resets and report
real-time factor separately from simulated response time.

Extend `g1_locomotion_diagnostics.py`, `human_aware_run_logger.py`, the fixed-command
tools and `scripts/run_avoidance_matrix.py` to report:

- Median/p95/p99 acquisition-to-command latency and input age at application.
- Command-to-motion onset, stopping time/distance and docking settling drift.
- Planner overruns, policy deadline misses, dropped/obsolete results and load.
- Contacts, falls, completed goals, unnecessary stops and time to resume.

The [dated response measurements](NAVIGATION_RELIABILITY_INVESTIGATION.md#calibration-corrections-and-evidence)
recorded 248 ms forward onset and 300–312 ms lateral onset. Forward stopping drift
was 0.206 m, failing the existing 0.20 m limit. Re-measure the frozen current
candidate before selecting numeric improvement targets; these historical results
are not today's baseline. Existing asynchronous inference and bounded ONNX threads
already address some compute contention, so profile before rewriting them.

**Exit:** repeatable, source-identified unloaded and loaded response reports with
timing tails and all failures retained. Use identical sensor replay to isolate
software changes, and closed-loop native trials to establish physical behaviour.

### R2 — Command ownership and immediate interruption (S1, highest priority)

Introduce an explicit base-command arbiter upstream of social planning and final
supervision. Navigation, teleop, docking and recovery acquire one active ownership
lease; commands carry source, sequence, expiry and mission/ownership generation.
Changing ownership invalidates obsolete plans and inference work. No source may
bypass the final collision supervisor.

Extend `arm_command_compositor.py`, the bridge contracts and bringup readiness
checks with continuous joint ownership during arm handover. Borrow Grove's atomic
controller-exchange principle; evaluate a `ros2_control` adapter separately if
needed. Stop/cancel requests must explicitly request a balanced stop at the next
eligible policy update rather than waiting for publisher silence to expire.
Maintain arm/payload hold and uncertain-possession semantics during cancellation.

**Exit:** competing-source, lease-expiry, cancel-during-inference and restart tests
show no stale-owner output or subsequent action dispatch, no unclaimed/dual-owned
joint interval, and bounded stop-request propagation. Measure physical stopping
separately; command cancellation does not imply instantaneous rest.

### R3 — Fast collision response independent of planning (S2, high priority)

Profile `human_collision_avoidance.py`, `human_aware_motion_planner.py`,
`dynamic_obstacle_tracker.py` and `g1_sim_sensors.py` under full sensing/viewer load.
Separate lightweight immediate collision checks from expensive alternative search.
Revalidate the current executable trajectory while asynchronous planning runs;
discard results invalidated by newer observations or mission changes. Extrapolate
tracks from acquisition time to decision time, with uncertainty growth. Near-field
obstacle response must not wait for semantic person classification.

Candidate scheduling budgets to evaluate, not validated defaults:

| Layer | Candidate execution contract |
| --- | --- |
| Physics/actuation | Preserve the validated timestep and actuator model |
| Locomotion | Preserve the policy's trained 50 Hz cadence |
| Lightweight collision check | Benchmark 50 Hz operation independently of search |
| Local candidate selection | Bounded 20–30 Hz if profiling supports it |
| Global routes | Asynchronous, event-driven replanning |
| Semantic perception/mission reasoning | Separate from control-critical execution |

The current onboard profile's 45 ms planning budget cannot simply be carried into
a 50 Hz callback. Set per-layer budgets from R1 measurements, bound pending work,
and retain a defined validated fallback when a deadline is missed. Port only
measured hot paths to C++ or optimized numerical code.

**Exit:** improved p95/p99 response under load without more contacts, false stops,
deadlocks or falls, followed by the unchanged S2 matrix and held-out gates.

### R4 — Physical response and backend comparison (S1/S5, high priority)

Apply the backend/docking gate above to the deployed baseline, payload/cycle-01
candidate and a dedicated AGILE runner. Verify named joints, state/history reset,
absolute-position outputs, per-joint gains, arm/waist ownership and inference
cadence. Match worlds, payloads and test conditions; each closed-loop policy uses
its own measured state. Replayed inputs check contracts, not balance performance.

Compare starts/stops, forward/lateral steps, reversals, pivots, pushes, arm
transitions and loaded settling before broad crowd-cost tuning or expensive
retraining. If training is justified, extend `scripts/g1_payload_training.py` with
frequent command transitions, residual-stop-motion penalties, push recovery,
arm/load variation and observation/actuation-delay randomization. Track slip,
torque saturation, oscillation and falls alongside tracking lag.

**Exit:** reproducible onset/stop improvement, including the existing forward
stopping-drift gate, without balance regression. Recalibrate planner/supervisor
motion envelopes for the selected backend; loaded promotion still requires S5.

### R5 — Occlusion and state-estimation robustness (S2, medium-high priority)

Use failure traces to prioritize acquisition-time transforms, extrinsic validation,
near-field obstacle memory and uncertainty during pedestrian occlusion. Compare
sudden-stop/turn predictors with the constant-velocity baseline. A retained track
must remain distinguishable from a fresh observation; test the current two-second
retention against ghost obstacles and missed encounters.

Evaluate FAST-LIO2 as an isolated alternative to current onboard odometry using
compatible LiDAR and a correctly colocated IMU. Compare drift, latency, dropouts
and crowd/turning behaviour before adoption. Grove's frozen-snapshot simulated
sweeps do not establish rolling-scan deskew performance. Use arm/payload geometry
for self-filtering rather than suppressing broad obstacle-height bands.

**Exit:** fewer tracking/odometry-induced failures and improved close-range
clearance on doorway, occlusion and turning trials, then S2 acceptance.

### R6 — Interruptible, persistent skills (S4, medium priority)

Consolidate the native transport and existing `g1_task_orchestration` paths into
one lifecycle with running, cancelling, holding, recovering and failed states.
Persist known/unknown possession, expose active-skill/failure feedback, and test
cancel, obstacle appearance, perception loss and restart at every transport phase.
Recovery must use measured conditions and preserve the original bounded deadline.
Canopy-style semantic/viewpoint planning can support S4; conversational and VLA
extensions remain outside the fast response path and are not reactivity prerequisites.

**Exit:** recoverable cases resume without stale motion; other cases terminate in
a truthful hold/assistance state without incorrect payload release.

### Integration and delivery order

The [optional Grove profile](../integrations/grove_g1/README.md) currently pins
`97bfb9ba13569a593b5e3ecfc5ed1d4208b2b681`, older than the review. Before running
the comparison, review/pin its update and refresh setup instructions against that
revision. Keep its Jazzy workspace separate from Humble and run one stack per
simulator/DDS authority. Retain applicable upstream/component notices. A distro
migration is a separate change, not a prerequisite for these work items.

First delivery: finish current S1 evidence/cancellation fixes, then R1 tracing and
R2 ownership. Next deliver R3 bounded response and the R4 backend comparison,
using R5 where traces identify sensing/estimation failures. Feed these measured
contracts into R6 and loaded S5/S6 acceptance. Isolated S3 grasp work can continue
alongside this sequence. All R1–R6 deliverables remain planned until their exit
evidence is recorded in the implementation ledger.

## Verification and metrics

Track task completion, interventions/mission-hour, recovery success, wrong
destinations; contacts by body part and impulse where available; clearance,
drops/falls; acceleration/jerk, reversals, stop/start count and docking settling;
yield timing, front cuts, passing clearance, group intrusion, human delay and
resumption; perception coverage/identity continuity/acquisition age; latency
tails/deadline misses; grasp retention and placement/release error.

Use targeted lifecycle regressions for S1, native deterministic scenarios for
integration, full matrices for promotion, and held-out conditions for acceptance.
The reactivity suite includes sudden crossings, occluded emergence, pedestrian
stops/reversals, mid-walk goal changes, sensor delay/dropout, push disturbances and
arm/payload transitions. Freeze scenario schedules as well as seeds; asynchronous
startup and observed-map goal admission can otherwise change the encounters.
Promote through contract checks → targeted native trials → the full 18-trial
matrix → held-out seeds/repeats → loaded transport gates. Report response tails
and task outcomes together, with thresholds unchanged and failed attempts retained.
Mock tests do not establish physical behavior. Smoothing must stay within measured
stopping/avoidance constraints. A few successful examples are not reliability
estimates: approximately 299 independent representative zero-failure trials are
needed for a one-sided 95% upper failure-probability bound below 1%.

## Implementation ledger

### R3 asynchronous escape search — 2026-10-05

[Implementation, fault tests and native evidence](ASYNC_ESCAPE_SUPERVISOR.md):
pure snapshot search in a single-flight worker; expiry/epoch rejection; fresh
control-side revalidation; idle result drainage and best-effort stop-before-shutdown.
Configured 50 Hz was exercised with 3.2–5.6 ms crowd callback p99, but timer jitter
and strict crowd failures remain. One-person/three-goal handoffs passed. Full R3
timing qualification, R2 ownership and S2/S6 acceptance are still open.

### R2/R3 command handoff and supervisor work — 2026-10-05

[Native evidence and contract](COMMAND_HANDOFF_AND_SUPERVISOR.md): atomic
planner command/state/continuation envelopes, restart epochs, full-pose goal
fences, startup single-writer checks and offline lineage joins. Prediction
preparation and static-query batching preserve existing risk predicates while
reducing repeated work. Final one-person handoff test: 3/3 goals, zero contacts;
four-person acceptance remains unmet. The supervisor remains at 20 Hz pending
separation of alternative search from immediate validation.

### Updated-plan implementation: S0.2 and R1–R3 — 2026-10-05

[Detailed report and artifacts](PHASE_PROGRESS_20261005.md): live resolved-parameter
capture and cross-node motion checks; inference generation/sequence timing and
overrun reporting; bounded/fair priority reservations; stop-generation invalidation
and stale-result rejection. Two native transport deadline diagnostics, six crowd
pilot cases and an isolated forward/stop check were retained. Partial work-package
progress only: S0/R1/R2/R3 and full social-navigation/transport acceptance remain open.

### Native iteration and goal-facing follow-up — 2026-10-05

- Bounded action cleanup now awaits terminal results, including goals accepted
  after mission timeout. Cleanup services callbacks independently of the expired
  mission deadline. Navigation observes terminal failure; dock/scan failures
  clear active intent.
- Continuous evaluation health latches outages, rejects replayed/regressing
  robot/people timestamps, checks population and final freshness. Stable
  placement requires at least one second of coverage, not a message burst.
  Native object acquisition timestamps and actual articulated contacts remain open.
- Bounded workspace recovery rechecks freshness after IK and refuses retries
  with unknown action termination or significant base movement. Pickup can
  re-dock after a sensed unreachable target without enlarging capture tolerance.
- Opt-in `--carry-posture payload_v1` shares the cycle-01 posture bank with
  training, reducing redundant IK branch variation. Historical `ik` stays the
  default for explicit comparison. Arm-transition/loaded-turn failures persist.
- Release retracts and retreats before requiring a fresh visual placement
  observation, responding to a retained correct physical placement that was
  invisible to the close camera. Precise docking now stops/settles/rechecks fresh
  pose under the same deadline. Clearance-based retreat avoids reversing toward
  the table just to hit an exact observation endpoint.
- Each transport run now captures source/config/model hashes, source-text archive,
  imported runtime module hashes, dependencies, launch assets and native binary
  identity. A Python source/install mismatch blocks launch. This is not complete
  native-build attestation or clean-machine reproducibility.
- [Grove-G1 review](GROVE_G1_REVIEW.md) informs ownership, measured docking,
  camera-aware perception, persistent skills and an isolated AGILE comparison.
- [Goal-facing travel](GOAL_FACING_NAVIGATION.md) uses a 0.75 m finishing region
  with hysteresis instead of a permanently latched 3 m region. Lateral escapes
  remain available. Faster yaw was measured but not adopted: the 0.75 rad/s trial
  also produced 0.64 m in-turn translation.

**Native outcomes:** nine zero-person development attempts were retained, with
two complete passes and seven failures under evolving implementations. The latest
pass (`20261005T110801Z`) verified 0.210 m lift and 3.496 m maximum displacement,
but needed repeated precise-retreat corrections. The next clearance-retreat
variant (`20261005T111141Z`) cleared pickup without endpoint chasing but failed
delivery docking. Other failures include stale workspace, a loaded-turn fall,
close-view verification, out-of-reach drift, tilt and odometry loss. These are
development outcomes, not a frozen-cohort reliability estimate.

All attempts: [portable transport ledger](results/transport_lifecycle_20261005.json).
The four-person goal-facing smoke failed with 2/3 goals and one contact:
[portable navigation result](results/avoidance_goal_facing_20261005.json).
S2 and crowd-transport promotion remain open. Next priorities are calibrated
docking deadband/coasting, loaded-turn drift and locomotion-policy compatibility.

Final checks for this iteration: **342 bridge tests, 83 benchmark tests, 18
bringup tests, eight focused script tests and three Torch/MJLab training-helper
tests passed**. Documentation links, whitespace and modified shell syntax passed.
The Torch tests ran with unrelated ROS pytest plugin autoload disabled in the
MJLab environment; no training run or physical-hardware trial was performed.
These software passes do not supersede the failed native acceptance results.

### Earlier publication and initial slice

- 2026-10-05 publication `3a7ceac`: source, simulator patch snapshot, navigation
  corrections, transport action/health helpers, provenance tools and retained
  datasets published together. [Publication verification](DEVELOPMENT.md#publication-verification--5-october-2026)
  records build/regression results. This does not close S0/S1 or add a new native
  acceptance cohort. Subsequent local work must be evaluated under its own identity.

- 2026-10-05: roadmap adopted for BrainCo-equipped indoor delivery. First S1
  attachment/placement slice implemented in `transport_demo.py`: explicit
  `empty`/`uncertain`/`held` states, conservative carrying/payload publication on
  ambiguous service outcomes, attachment state in result/heartbeat telemetry,
  and fresh post-release-acknowledgement visual placement evidence.
- Verification: **55 focused tests passed**, comprising 19 new lifecycle
  regressions plus transport completion, geometry and manipulation IO tests.
  Tests ran against workspace source in the ROS Docker image with the existing
  `.verification/install` overlay for generated messages. Host collection failed
  because those container-built message packages were unavailable at host paths;
  the container run resolved that environment issue. Documentation link checks
  and `git diff --check` passed. No new native simulation or hardware run was
  performed; S1 as a whole remains open.

Reproduce the focused tests from the repository root:

```bash
docker compose run --rm --no-deps unitree-g1-sim bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /workspace/ros2_ws/.verification/install/setup.bash &&
  export PYTHONDONTWRITEBYTECODE=1 &&
  export PYTHONPATH="/workspace/ros2_ws/src/unitree_g1_mujoco_bridge:/workspace/ros2_ws/src/dynamic_obstacle_benchmarks:$PYTHONPATH" &&
  python3 -m pytest -p no:cacheprovider \
    src/unitree_g1_mujoco_bridge/test/test_transport_lifecycle.py \
    src/unitree_g1_mujoco_bridge/test/test_transport_completion.py \
    src/unitree_g1_mujoco_bridge/test/test_transport_geometry.py \
    src/unitree_g1_mujoco_bridge/test/test_sim_manipulation_io.py'
```

Prerequisite: the ROS verification overlay has been built, including the
`GraspObject` service. This command verifies source-level behavior; it does not
rebuild or qualify an installed native transport executable.
