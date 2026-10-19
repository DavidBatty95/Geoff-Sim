# Project status and progression

<!-- geoff-nav:start -->
<p align="center">
  <a href="../README.md"><img src="media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../README.md#quick-start"><img src="media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="DEMO_GUIDE.md"><img src="media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="INDEX.md"><img src="media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="REPOSITORY_GUIDE.md"><img src="media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="PROJECT_STATUS.md"><img src="media/navigation/progress-current.svg" width="132" height="44" alt="Progress (current page)"></a>
  <a href="PROJECT_PLAN.md"><img src="media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="RESULTS_REVIEW.md"><img src="media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="literature_review/README.md"><img src="media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="DEVELOPMENT.md"><img src="media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

**Last reviewed: 19 October 2026.** This page retains the historical development
baseline [`3a7ceac`](https://github.com/DavidBatty95/Geoff-Sim/commit/3a7ceac)
alongside the dated follow-ups below. It is a manually maintained
evidence snapshot, not an acceptance release. Later experiments need their own checks.

## At a glance

**Broader reference corpus:** 51/51 qualifying sources reproduced; two new
training runs and 406 matched evaluations completed. Broad80 fixes four parent
failures without parent-relative regressions (51/58 vs47/58); broader coverage
still loses fresh passes against narrow rehearsal in both seeds. No promotion.
[Paired changes and complete-run recordings](BROAD_REFERENCE_20261019.md).

**Actor-reference rehearsal:** implemented optional native successful-state
rehearsal during PPO. Matched seeds80/81 and 290 evaluation probes yield 52/58
passes for reference80 versus parent's 51/58, both with no falls; reference81
regresses to 42/58 with three falls. No repeatable benefit or promotion yet.
[Implementation, corpus exposure and outcomes](ACTOR_REFERENCE_20261019.md).

**Latest factor-isolation comparison:** six 100-iteration runs isolate stopping
reward and transition commands across two continuation seeds. Across 464 probes,
neither factor consistently beats its matched controls. Retained braking79 has
48/58 passes with no falls on this round's conditions; no new policy exceeds
that count. [Results and changed validation set](BRAKING_ISOLATION_20261019.md).

**Braking follow-up:** targeted transition/stopping training across two new
continuation seeds completed with matched controls and 242 probes. Seed 79
reduces failures from 16/58 to 6/58 versus the previous candidate, with no falls;
seed 78 has eight falls despite passing its fresh validation set. Neither
qualifies for promotion. [Full comparison](BRAKING_FOLLOWUP_20261019.md).

**Latest failure-reduction experiment:** three matched 400-iteration candidates
each pass 5/10 development conditions with zero falls, versus cycle-05's 0/10
and eight falls. Selected `timing_fixed` passes 18/24 held-out conditions versus
6/24, with zero versus seven falls. Six held-out failures remain, including one
regression; deployment qualification is not met. See
[longer-budget stopping comparison](FAILURE_REDUCTION_20261019.md).

**Latest native-timing training checkpoint:** latched post-substep sensor packets,
explicit grouped motor PD and per-episode load-offset variation are implemented.
Actual GPU checks verify encoder/actor timing, gains and partial-reset isolation.
The candidate passes **0/10** dense cases and **1/4** offset controls; it remains
unpromoted. **866 ROS results, 205 orchestration tests, 12 native CTests,
19 training-contract tests and two payload/weld tests pass.**
[Full implementation scope and retained outcomes](NATIVE_TIMING_TRAINING_20261018.md).

**18 October compiled-model/feedback checkpoint:** final scene compilation
exposes wrist-roll friction loss 0.1 versus native 0.2; an opt-in correction
matches all 29 joint friction values. Diagnostic load placement further reduces
two-state joint-velocity gaps; CPU/Warp differences are small at one step.
All **16 feedback-source stopping probes fail**. Exact continuation passes three
cases, including native-sensor feedback. **866 ROS results, 204 orchestration
tests, 12 native CTests, 17 training-contract tests and 6 separate native Python
tests pass.** [Measurements, scope and remaining work](COMPILED_FEEDBACK_20261018.md).

**Latest 17 October actuation checkpoint:** identical-state tests isolate
built-in-PD/implicit integration differences, and recorded actions confirm active
deployment soft-limit clipping absent from historical training. Euler and target-
limit options are implemented and tested. Euler-only passes **1/10** dense cases;
Euler plus clipping passes **0/10 distinct conditions**, staying upright but
failing stopping gates. The interrupted comparison and follow-up remain separate.
**866 ROS results, 202 orchestration tests, 12 native CTests, 16 training-contract
tests and 3 separate native actuator tests pass.**
[Measured scope, residual mismatches and results](ACTUATION_PARITY_20261017.md).

**17 October dense-transition checkpoint:** an opt-in curriculum audits 1,266
move-to-stop requests and all 101 movement-duration ticks. The new candidate
passes **0/10** dense 60 ms conditions; earlier long77 passes **1/10**, cycle-05
**0/10**. None passes an entire five-offset neighborhood. Both fine-tunes pass
3/4 separate lower-delay controls; the new candidate is not promoted.
**866 ROS results, 201 orchestration tests, 12 native CTests and 14 separate
training-contract tests pass.** [Evidence and next priority](DENSE_TRANSITIONS_20261017.md).

**Latest 16 October stop-branch study:** preserved-state stop branches expose a
one-observation timing sensitivity. All 14 broad branches fail; an adjacent-update
follow-up passes 1/4. Fresh-start clock controls pass 1/4 with legacy timing and
0/4 with explicit integer steps. New support diagnostics and actual-transition
scoring improve reproducibility; no policy is promoted. **866 ROS results,
200 orchestration tests and 12 native CTests pass**, plus eight separate native
checkpoint/support tests and both exact continuation regressions.
[Conditions, failures and timing limits](STOP_BRANCHES_20261016.md).

**16 October checkpoint/delay study:** synchronous native-probe checkpoints pass
exact continuation in two scenarios, including reproduction of an outage-case
fall. A 40-probe matrix across seeds 76/77 and short/long delay training yields
baseline **2/8**, short76 **3/8**, long76 **3/8**, short77 **2/8**, long77 **3/8**.
Four longer-command follow-ups all fail. All candidates remain experimental.
**866 ROS results, 192 orchestration tests, 12 native CTests and eight separate
native Python tests pass.** [Implementation, all outcomes and limits](CHECKPOINT_DELAY_20261016.md).

**Latest matched ablation:** a predeclared 24-probe matrix covers both lateral
signs, 40/60 ms delays and loaded/held-empty states. Baseline and stopping-penalty
candidate pass **2/8** each; the matched no-penalty fine-tune passes **3/8**.
Four longer-command follow-ups expose the opposite tradeoff; neither candidate
is promoted. All 60 ms conditions fail, including five falls across the matrix.
**866 ROS results, 191 orchestration tests and 12 native CTests pass.**
[Complete matrix, setup failure and follow-ups](STOPPING_ABLATION_20261015.md).

**Latest stopping-training experiment:** cycle-07 adds an explicit zero-command
residual-motion penalty and trains with published-action feedback and 0–40 ms
application delay. Native loaded gates pass **2/4 versus cycle-05's 1/4**;
both negative-lateral cases still fail. Cycle-07 remains unpromoted. Two
command-only replays both contact scene geometry, showing the need for faithful
initial-state capture. **866 ROS results, 183 orchestration tests, 12 native
CTests and 12 separate training-contract tests pass.**
[Complete comparison and provenance](STOPPING_CANDIDATE_20261015.md).

**Latest loaded follow-up:** native probes now use the actual 150 g object and
audit published/applied action history. **3/4** pass; negative-lateral published
feedback with 40 ms delay still fails (0.738 m post-stop travel). The final-rollout
zero-person mission completes in **154.44 s**; the one-person mission fails while
braking before pickup. **866 ROS results, 182 orchestration tests and 12 native
CTests pass**, plus the isolated native fixture test.
[Full conditions, provenance and limits](LOADED_STOPPING_20261015.md).

**Latest 15 October stopping checkpoint:** delayed command transitions now agree
across scalar rollouts, vectorized checks and sequence search. Held-empty stop
probes pass **4/8**; all four negative published-feedback cases fail. Integrated
follow-ups complete **1/3** (zero-person repeat: **169.04 s**); they precede the
final vectorized correction. No reliability or speedup claim follows. **866 ROS
results (670 bridge), 172 orchestration tests and 12 native CTests pass.**
[Stopping criteria, failures and next qualification step](NATIVE_STOPPING_20261015.md).

**Latest 15 October workspace checkpoint:** support inspection can reobserve and
replan after a settled displaced scan; docking withdraws nominal intent when its
predicted approach corridor is occupied. The zero-person control completed in
**149.10 s**, with a native payload correction from **3.79 to 2.73 cm** before
release. **All six one-person attempts failed**, including contact/fall cases;
the final two people trials stayed upright/contact-free but did not complete.
Predictive refuge remains opt-in. **834 ROS results (638 bridge), 159 orchestration
tests and 12 native CTests pass.** [All outcomes and limits](CROWDED_DOCKING_20261015.md).

**Latest 15 October cadence checkpoint:** minimum-spaced device observations
remove compressed catch-up frames. The unchanged compact exploration cohort is
**4/4 complete**; a separate compact pilot and medium-building red → blue run also
completed. The alignment acceptance path and restoration were exercised natively.
Four recording attempts added three completions and one fall; two completed runs
had unusable footage, and the final inspected recording now shows the full delivery.
These ten attempts are not a pooled reliability cohort. **807 ROS results (611
bridge), 155 orchestration tests and 12 native CTests pass**.
[Timing diagnosis, all outcomes and limitations](POLICY_CADENCE_20261015.md).

**Later 15 October placement follow-up:** the prior red → blue failure exposed
about 32° of base rotation during a pelvis-accurate lower action. A bounded
world-relative payload-alignment guard now precedes release; route blockage
snapshots and an opt-in numerical-worker limit improve diagnosis. **All five
new native attempts fell before placement**, including the one-worker experiment.
The new guard remains natively unqualified; the earlier 2/4 cohort is not a result
for this later candidate. **803 ROS results (607 bridge), 152 orchestration tests
and 12 native CTests pass.** [Detailed scope and all outcomes](PLACEMENT_ALIGNMENT_20261015.md).

**15 October navigation-integrity checkpoint:** checked path-goal transforms
prevent Humble's reproduced false-arrival path; acquisition-time motion bounds
reject abrupt primary/fallback scan matches. A scripted red → red delivery
completed in **126.52 s**. The unchanged-code exploration cohort completed
**2/4** colour tasks on one zero-person layout: red → red in **181.58 s**, blue →
blue in **207.88 s**. All four stayed upright with valid evaluator data; red →
blue failed placement verification and blue → red failed route admission. Two
earlier development falls remain in the eight-attempt record. **791 ROS results,
142 orchestration tests and 12 native CTests pass**.
[Diagnosis, source changes and full outcomes](NAVIGATION_INTEGRITY_20261015.md).

**14 October locomanipulation media and posture follow-up:** the README now shows
fresh pickup/lift and loaded-motion recordings with an updated workflow banner.
**All seven capture attempts were incomplete**; the manifest retains settling,
localization and body-instability failures. New clips do not establish placement
or restoration. Fixed arm carry/rest checks now use the same current-waist
reference and reject missing/nonfinite measured joints. Braking can use up to
five seconds of the existing operation deadline after captures showed measured
settling beyond 1.5 seconds. **588 bridge tests pass.**
[Media/provenance](MEDIA.md#locomanipulation-recordings--14-october-2026) ·
[Control changes and qualification limits](PERFORMANCE_FOUNDATION.md#fixed-arm-posture-completion-and-native-capture-follow-up).

**14 October observation-freshness follow-up:** live DDS traffic can no longer
mask a frozen device tick when `use_device_tick` is enabled. Duplicate/reordered
packets are rejected before accepted-state/upright updates. The existing
LowState timeout now also bounds lack of measurement progress; fault handling
invalidates inference and publishes zero motion feedback before the existing
sensor-state hold. **567 bridge tests pass**, including eight new freshness
regressions. This does not detect an advancing but consistently delayed stream
or qualify physical fault recovery. [Contract and remaining gaps](PERFORMANCE_FOUNDATION.md#device-observation-progress-watchdog).

**14 October integrated reliability checkpoint:** the experiment runner now
retains planned attempts, explicit run identities and strict outcomes through
malformed results and bounded outer timeouts. Bootstrap revisions match patch
bases. Compositor selection/publication is serialized with action state;
navigation and inspection braking reject missing measured settling rather than
silently advancing. Full verification passed **744 ROS test results** (including
**559 bridge tests**), **142 orchestration tests**, **12 native CTests**, the mock
benchmark and the MJLab environment check. Pristine patch reconstruction passed.
These are implementation checks; native delivery qualification of the new
settling contract remains open. See [the checkpoint and next steps](PERFORMANCE_FOUNDATION.md).

**14 October delay-robust candidate:** added coherent sensor-lag training and
explicit applied/published action-feedback modes. Cycle-06 completed 500 iterations
and passed **12/12** matched native delay cases versus cycle-05's **6/12**, while
still moving. A broader turn sweep found a low-speed clockwise stop regression;
two earlier checkpoints did not pass all selection probes. **The candidate is
not promoted.** All 52 investigation probes, including failures and contact-
confounded trials, are retained in the [training/qualification report](PERFORMANCE_IMPLEMENTATION_20261012.md#sensor-delay-training-candidate-and-qualification).

**14 October actuation experiments:** arm execution is now asynchronous, with
single-threaded action/watchdog regressions. The one-thread native comparison
did not improve timing, so two threads remain the default. Opt-in 20 ms duplicate
coalescing reduced recorded LowCmd traffic from 98.4 to 50.7/s and policy-to-application
p95 from 35.3 to 30.3 ms in a development pair. Both missions still fell; coalescing
remains disabled by default. A compact run completed an arm trajectory but fell
during docking. **549 bridge tests and 26 diagnostic/orchestration tests pass.**
[All five attempts and limitations are retained](PERFORMANCE_IMPLEMENTATION_20261012.md#14-october-actuation-pipeline-experiments).

**13 October stability follow-up:** collision-checked scan/brake requests now take
priority over obsolete escapes when clearance permits. The bridge suite passes
**532 tests**. A mirrored medium-building blue → red mission completed in
**389.19 seconds** with no fall/drop/recorded person contact, exercising the
outward-shoulder IK fix through pickup and placement. The other layout stayed
upright but failed with no reachable useful viewpoint before pickup; loaded
retreat repeatability and all-layout completion remain open. See the
[follow-up and both retained outcomes](PERFORMANCE_IMPLEMENTATION_20261012.md#13-october-navigation-stability-follow-up).

**Later 13 October investigation:** the search dead end followed a lethal-start
Nav2 failure and substantial inspection drift toward a wall, rather than full
coverage. Added fresh-odometry turn-displacement detection and partial-inspection
handling; **535 bridge tests pass**. Two native retries both fell before pickup
and did not exercise the new guard. End-to-end recovery remains unverified; see
the [retained failure analysis](PERFORMANCE_IMPLEMENTATION_20261012.md#search-dead-end-investigation-and-turn-displacement-guard).

**Turn-to-rest diagnostics:** 25 isolated native probes (extended stopping,
brief outages, irregular cadence, and an incident-command window) stayed upright.
Added bounded, timestamped recovery-event telemetry; **536 bridge tests pass**.
Instrumented layout-0 blue → red run `20261013T144107Z` completed in **254.09 s**,
including loaded retreat, with no fall/drop/recorded person contact. Two
stale-result holds were observed and recovered. Earlier falls remain unexplained
by the isolated hypotheses; see the [probe report](PERFORMANCE_IMPLEMENTATION_20261012.md#controlled-turn-to-rest-probes-and-recovery-observability).

**Inference-dropout continuity:** a subsequent instrumented failure exposed a
1.055 rad recovery target jump. Brief inference faults now retain validated
targets and policy history, with escalation after two deadline budgets and
unchanged stale-result rejection. **539 bridge tests pass**. Two native follow-ups
confirmed zero target jumps during those holds but still fell before pickup;
this fixes a discontinuity, not overall stability. See the
[retained comparison](PERFORMANCE_IMPLEMENTATION_20261012.md#inference-dropout-target-continuity).

**Combined-motion delay investigation:** diagonal commands and a recorded
command-window replay stayed upright in isolation, but adding 40/60 ms joint/IMU
measurement delay to 40 ms action delay reproduced falls without sampled
non-floor scene contacts. Added Humble-compatible DDS metadata timing; the
instrumented mission still fell. DDS-publication-to-target-publication p95 was
32.3 ms, with downstream actuation still unmeasured. **542 bridge tests pass**;
[all probe and native outcomes are retained](PERFORMANCE_IMPLEMENTATION_20261012.md#combined-translation-and-measurement-delay-investigation).

**Downstream timing:** opt-in native control-application sampling now closes the
previously unmeasured downstream segment. A corrected transition-only run gave
128 content-matched observation/application pairs: device-time age was 40 ms
median, 90.6 ms p95 and 122 ms maximum. This overlaps the isolated delay-sensitive
failure range, but does not prove causation. Both instrumented missions fell;
all failures and matching ambiguities are retained. **543 bridge tests and 12
native CTests pass**. See the [application-timing report](PERFORMANCE_IMPLEMENTATION_20261012.md#native-command-application-timing).

**12 October performance follow-up:** continuous turn/walk transitions, checked
nominal braking, full-route execution and camera-aware search have produced
complete zero-person medium-building red → blue deliveries. A later development
trial completed in **202.48 seconds** with six inspection stops, no fall/drop and
zero recorded contacts. The colour/layout matrix retains failures, and crowded
operation is still unqualified. Full CI passed with 704 ROS test results, 124
orchestration tests and 11 native CTests. See the [implementation and evidence
report](PERFORMANCE_IMPLEMENTATION_20261012.md).

**Wider-table/people follow-up:** the current development layout widens the
central aisle to 3.9 m and the gap between rows to 2.0 m. Added continuous
workspace prediction and recovery/balance corrections pass regression tests,
but the new native trials have **no complete delivery pass** and include
contacts/falls. Earlier successful transport cohorts used the earlier layout.
See [the retained clearance report](TRANSPORT_PEOPLE_CLEARANCE.md).

**6 October fidelity/publication follow-up:** [full report](HARDWARE_FIDELITY.md).
Canonical G1 CRC/machine-mode handling, measured sensor-site IMUs, raw measured
sensor TF and driver-layout-aware point clouds are validated. Full CI passed,
including 624 ROS test results and 11 native CTests. The final two zero-person
dual-delivery runs passed on unchanged code. Hardware calibration, complete
contact coverage and strict crowd acceptance remain open.
The updated native one-person/three-goal smoke passed with score 86.752 and
0.646 m minimum body clearance; it does not replace the full crowd matrix.

| Area | Current state | What remains |
| --- | --- | --- |
| Simulation and demos | Integrated ROS 2/MuJoCo stack, onboard visual sessions, six representative worlds and results GUI | Repeated clean-checkout/native reproduction and longer uninterrupted evidence |
| Onboard navigation | Sensor-derived localization, mapping, person tracking and planning implemented | All-pass full matrix and held-out acceptance; contacts still occur |
| Payload transport | Experimental sensor-driven pickup/carry/place; two consecutive zero-person fixture passes | Repeatability, broader conditions, actual finger grasp and crowd integration |
| Mission correctness | Attachment-state, action-termination, health and provenance helpers published | Complete lifecycle/coverage qualification and integrated recovery |
| BrainCo hands | Description, interfaces and experimental contact-bench work | Repeated mobile contact-only grasp validation |
| Research | 50-work literature-review package, published trials and retained failure ledger | Controlled comparisons, held-out experiments and stronger reliability estimates |
| Hardware | Physical G1 is the target | Calibrated deployment and hardware acceptance remain future work |

**Planned research delivery:** [P0–P7](RESEARCH_PROGRAMME.md) maps the five review
gaps to calibration, causal diagnosis, method comparisons and publication evidence.
All phases are planned; first priority is P0 followed by P1/P2 and the G1-focused
Paper A candidate. These research plans do not change the acceptance status above.

## Current evidence

| Snapshot | Recorded outcome | Scope |
| --- | --- | --- |
| [Published v12 cohort](DEMO_RESULTS.md) | 4/18 strict passes; 19/54 goals; 12 contacts | Six representative worlds, four people and three requested goals per trial |
| [Consolidated v82](results/avoidance_v82_matrix.json) | 14/18 strict passes; 49/54 goals; four contacts | Investigation configuration; not a release acceptance pass |
| [Later full v84 matrix](results/avoidance_v84_matrix.json) | 10/18 strict passes; 46/54 goals; six contacts | Latest retained complete matrix in the assessed baseline |
| [Targeted v85 audit](results/avoidance_v85_alignment_audit.json) | Geometric overlap confirmed in both audited courtyard failures | Alignment diagnostic, not a new full matrix |
| [Payload cycle 01](PAYLOAD_POLICY_TRAINING.md) | Two consecutive complete zero-person native runs; 2 complete passes among 11 recorded candidate attempts | Earlier attempts used evolving integration code; magnetic attachment, not finger-force closure |
| [Publication checks](DEVELOPMENT.md#publication-verification--5-october-2026) | 15 ROS packages built; 333 bridge, 83 benchmark, 18 bringup, 79 script, 7 training tests; 10 simulator CTests passed | Software/build evidence from the development environment |

The best observed complete navigation matrices reached 16/18 strict passes, but
no all-pass release matrix or held-out acceptance is established. The later v84
regression is retained rather than replacing it with a best-case snapshot.
Matched seeds do not guarantee identical timing or observed-free goal selection.

The published default dashboard dataset remains the v12 publication snapshot.
Use the selector or explicit dataset paths to inspect later iterations. A higher
filename version does not mean a better or qualified controller. See
[results review](RESULTS_REVIEW.md) for denominators and provenance.

## Project progression

### Local native-contract follow-up — 6 October 2026

[Native payload contract work](NATIVE_PAYLOAD_CONTRACT.md) produced two consecutive
complete zero-person **two-colour/four-table** deliveries using the cycle-03
native-inertial/delay-trained policy. Both runs verified red and blue placement;
the second exercised cancel/wait/relocalize/resume. No falls or drops were recorded.
This is 2/2 on the final repeated configuration, and 2/5 across its evolving
integration attempts; it remains a simulated-weld fixture, not contact-only grasp.

Matched delayed/native-inertial loaded probes retained 32/32 cycle-03 environments
versus 23/32 baseline, but left-pivot tracking regressed, so fluid-carry acceptance
is still open. A subsequent four-person navigation pilot with the original
navigation policy passed **0/2 strict trials**: small-open reached 3/3 goals but
scored 67.933; medium-doorway reached 2/3 and recorded one collision. The full
S0–S7 plan is not complete. Latest checks: 417 bridge, 95 benchmark, 18 bringup,
19 focused script and 9 Torch/training tests passed.

The subsequent acquisition/approach slice adds native object acquisition stamps,
replay-safe placement evidence, closer observation viewpoints and RGB-D support-edge
pickup directions. Two further complete dual-delivery runs passed after retained
failures (perception optimization changed between those successes). Cycle 03 now
has **4 complete passes among 9 evolving integration attempts**. Latest checks:
426 bridge tests and 10 native CTests passed; benchmark/bringup/script/training
checks above also passed. Full phase acceptance remains open.

### Local phase follow-up — 5 October 2026

Latest [dual-cube perception diagnostics](DUAL_TRANSPORT_PERCEPTION.md) identified
multi-second camera delivery latency at 384×216. At 192×108, both cubes and both
destinations acquired visual confirmation; one 120-second diagnostic run verified
the red delivery. Full-budget baseline and fluid-policy runs still fell, so no
complete two-cube success or native fluid-policy qualification is established.
Current checks: 412 bridge, 95 benchmark, 18 bringup, 8 focused script and 8
Torch/training tests passed. All five native attempts remain in the evidence ledger.

Latest [asynchronous supervisor work](ASYNC_ESCAPE_SUPERVISOR.md) separates escape
search from immediate validation. Crowd callback p99 measured 3.2–5.6 ms at
configured 50 Hz, with ~35 ms p99 actual tick intervals. One-person handoff testing
passed 3/3 goals; the two four-person cases still failed strict acceptance (one
completed without contact but scored below 70). Faster callbacks are not a
collision-free or hard-real-time qualification.

Earlier [handoff/supervisor iteration](COMMAND_HANDOFF_AND_SUPERVISOR.md): the
full-pose atomic command path completed a one-person, three-goal native test
with zero contacts and 507/507 matched planner-to-supervisor records. Prediction
preparation and batched static checks reduced observed supervisor work in pilots,
but the final four-person pair failed both cases. No crowd-acceptance promotion.

After the published baseline, runtime parameter capture, inference-publication
timing, bounded priority scheduling and stop-generation invalidation were implemented
and tested. In a courtyard pilot, active-planning p99 was 82 ms versus 154 ms in
the instrumented baseline; that seed completed all three goals, but other trials
still collided. The interruption follow-up passed its isolated stopping check
and failed both crowd cases. No stage acceptance is claimed.

See the [phase implementation and native evidence report](PHASE_PROGRESS_20261005.md)
for all conditions, limitations, failed runs and remaining gates. These local
results supplement rather than replace the dated publication table above.

![Development milestones from simulation foundation to the October source publication](media/project-progression.svg)

Publication dates and experiment dates are different. Commits identify published
source; dated reports identify measurements performed on their recorded setup.

| Date / phase | Progress | Evidence to follow |
| --- | --- | --- |
| July 2026 — integration | Simulator/ROS integration and verification foundation | [Integration checkpoint `3dd33d4`](https://github.com/DavidBatty95/Geoff-Sim/commit/3dd33d4) |
| August 2026 — runnable workflows | Locomotion/social demonstrations and manipulation workflows; oracle modes explicitly separated | [Workflow publication `1e98ba4`](https://github.com/DavidBatty95/Geoff-Sim/commit/1e98ba4), [suite reference](DEMO_SUITE.md) |
| 22 September 2026 — inspectable experiments | Six representative worlds, dashboard, portable benchmark evidence and transport plan | [Worlds/dashboard `7c4cd3e`](https://github.com/DavidBatty95/Geoff-Sim/commit/7c4cd3e), [evidence publication `886abb2`](https://github.com/DavidBatty95/Geoff-Sim/commit/886abb2) |
| September development records — reliability investigation | Tracking, response, candidate-search and supervision corrections; failed pilots retained | [Investigation ledger](NAVIGATION_RELIABILITY_INVESTIGATION.md), [transport-plan history](TRANSPORT_DEMO_PLAN.md) |
| 25 September experiment report — payload fixture | Payload-aware fine-tuning and two consecutive unpinned fixture completions | [Training cycle, interventions and attempts](PAYLOAD_POLICY_TRAINING.md) |
| 5 October 2026 — publication baseline | Expanded documentation, source/patch publication, transport helpers and regression verification | [Documentation `94c72be`](https://github.com/DavidBatty95/Geoff-Sim/commit/94c72be), [source `3a7ceac`](https://github.com/DavidBatty95/Geoff-Sim/commit/3a7ceac) |

## Current priorities

1. **Reproduce the baseline (S0):** freeze source, patches, policy and runtime
   identity; repeat a clean build and a native baseline with complete artifacts.
2. **Qualify mission correctness (S1):** verify uncertain possession, terminal
   cancellation, continuous evaluator coverage, contact accounting and authority.
3. **Close navigation failures (S2):** reproduce audited encounters, isolate the
   sensing/model/planning/execution contribution, then validate full and held-out cohorts.
4. **Develop physical grasping (S3):** use contact-only hand experiments with
   declared object geometry and perturbations; keep magnetic fixtures labelled.
5. **Integrate recovery and loaded motion (S4/S5):** consolidate mission paths,
   verify swept geometry and measure loaded tracking/stopping before crowd promotion.

The [forward plan](PROJECT_PLAN.md) gives deliverables and exit criteria. Individual
components may progress in parallel; the end-to-end acceptance dependencies still apply.

## What the current publication does not establish

- Reliable collision-free navigation across development and unseen conditions.
- Crowd transport, contact-only mobile finger grasping or physical G1 deployment.
- An uninterrupted 30-minute endurance run from short GIFs or restarting sessions.
- A causal improvement from RL alone: initialization and integration changes also
  contributed to the documented payload experiment.
- A fresh acceptance result for every later local source change.

These are evidence boundaries that drive the next experiments, not reasons to
hide failed trials. Follow [the plan's promotion gates](PROJECT_PLAN.md#stage-deliverables-and-exit-evidence).

## Following updates

- Read this page for the current assessed snapshot and changed blockers.
- Read [the project plan](PROJECT_PLAN.md) for the next deliverables.
- Use [GitHub commits](https://github.com/DavidBatty95/Geoff-Sim/commits/main)
  for publication history and [Actions](https://github.com/DavidBatty95/Geoff-Sim/actions/workflows/verification.yml)
  for actual CI outcomes; the test counts above are not a claim about every CI run.
- Use [issues](https://github.com/DavidBatty95/Geoff-Sim/issues) to discuss or take
  ownership of a work package. Planning items here are not automatically created issues.

Update this page when a published milestone or measured outcome changes the
assessment. Keep old reports intact and link new evidence rather than rewriting
historical results. The [update checklist](PROJECT_PLAN.md#keeping-the-project-current)
defines the maintenance process.
