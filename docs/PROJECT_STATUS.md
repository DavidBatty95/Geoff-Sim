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

**Last reviewed: 6 October 2026.** This page retains the historical development
baseline [`3a7ceac`](https://github.com/DavidBatty95/Geoff-Sim/commit/3a7ceac)
alongside the dated follow-ups below. It is a manually maintained
evidence snapshot, not an acceptance release. Later experiments need their own checks.

## At a glance

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
