# Changelog

<!-- geoff-nav:start -->
<p align="center">
  <a href="README.md"><img src="docs/media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="README.md#quick-start"><img src="docs/media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="docs/DEMO_GUIDE.md"><img src="docs/media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="docs/INDEX.md"><img src="docs/media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="docs/REPOSITORY_GUIDE.md"><img src="docs/media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="docs/PROJECT_STATUS.md"><img src="docs/media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="docs/PROJECT_PLAN.md"><img src="docs/media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="docs/RESULTS_REVIEW.md"><img src="docs/media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="docs/literature_review/README.md"><img src="docs/media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="docs/DEVELOPMENT.md"><img src="docs/media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

User-facing changes and significant development publications are summarized here.
Entries identified by commit are **development snapshots**, not numbered stable
releases or new acceptance claims. See [project status](docs/PROJECT_STATUS.md)
for measured outcomes and [release policy](docs/RELEASES.md) for promotion criteria.

## Unreleased

### Added

- Opt-in transition curriculum with tick-spaced movement durations, longer stop
  dwells, signed lateral endpoint coverage and retained request/duration audits.
- Timing-neighborhood summaries require every declared offset to be present and
  pass. A 30-probe dense sweep rejects the new candidate; eight lower-delay
  controls retain narrower successes. [Full evidence](docs/DENSE_TRANSITIONS_20261017.md).

- Explicit checkpoint stop branches with preserved-prefix/physical-state checks,
  offline foot-support diagnostics and branch-specific actual-transition scoring.
- Opt-in integer-physics-step command clock avoids floating-point boundary
  postponement; legacy timing remains reproducible. Native comparisons expose
  one-update stopping sensitivity and retain all falls/failures.
  [Evidence and verification](docs/STOP_BRANCHES_20261016.md).

- Synchronous native-probe JSON checkpoint capture/restore with full integration
  state, policy history, queues, delayed measurements, arm targets and scheduler
  state. Dtype preservation fixes a reproduced continuation mismatch; both
  end-to-end parity cases pass. Fresh-start matrices reject resumed prefixes.
- Two-seed comparison of 0–40 ms versus 0–60 ms training exposure: 40 matched
  probes and four duration follow-ups retain all failures. Candidates remain
  unpromoted. [Results and scope](docs/CHECKPOINT_DELAY_20261016.md).

- Serial native stopping matrix runner with predeclared paired conditions,
  policy/protocol and measured-delay checks, retained process failures and
  incomplete-plan host export. Eight regressions cover orchestration integrity.
- Matched stopping-reward ablation: 24 broader probes and four longer-command
  follow-ups show duration-dependent tradeoffs and 60 ms failures. Both candidates
  remain unpromoted. [Full evidence](docs/STOPPING_ABLATION_20261015.md).

- Opt-in stopping training objective and signed lateral stop evaluation schedule,
  retaining the deployed actor-input contract. Cycle-07 improves measured travel
  but remains unpromoted: 2/4 strict native gates versus baseline 1/4.
- Replay rejects command-coverage gaps over 250 ms. Both retained incident-command
  replays contact scene geometry; neutral-state replay is not incident recreation.
  [Training and comparison evidence](docs/STOPPING_CANDIDATE_20261015.md).

- Native loaded-start probe fixture with the original 150 g object, attachment
  tracking and separate payload/scene contact evidence; recorded action vectors
  and publication/application IDs are audited by the stopping scorer.
- Four loaded probes (3/4 passing) and corrected-rollout integrated repeats:
  zero-person delivery completes in 154.44 s; one-person braking still fails.
  [Conditions, setup failure and results](docs/LOADED_STOPPING_20261015.md).

- Fail-closed native stopping scorer with dwell, travel, complete-window and
  sample-coverage checks, retained failures and source hashes. Eight held-empty
  probes pass 4/8; three integrated attempts complete 1/3.
- Correct delayed handoffs and boundary integration in scalar/vectorized
  rollouts and sequence search. Final CI passes 866 ROS results and 172
  orchestration tests. [Evidence and limits](docs/NATIVE_STOPPING_20261015.md).

- Explicit settled-view displacement recovery with post-stop acquisitions,
  invalidated viewpoint caches and the original mission deadline.
- Docking workspace yield/resume and age-aware predicted approach-corridor
  admission, including compensated endpoints and finite/fresh robot-pose checks.
- Prefix-selected evidence exports retain failed trials and recovery events.
  Seven workspace trials include a complete zero-person control; all six
  one-person attempts failed. Predictive refuge remains opt-in.

- Policy observation-interval telemetry and catch-up/history regressions. Native
  follow-ups include a 4/4 complete compact colour-task cohort and a medium-building
  delivery; ten candidate attempts retain nine completions and one recording-run fall.
- Complete recorded red-cube delivery GIFs, including placement verification and
  restoration. Recorder window ownership and unusable-footage checks prevent
  mislabelling unrelated/obstructed captures; all capture outcomes are retained.

- World-relative pre-release payload alignment with bounded reprojection/correction
  after base motion; attachment remains held on failed/stale alignment.
- Blocked-route footprint categories and hashed occupancy/query snapshots for
  diagnosing loaded navigation without weakening clearance.
- Opt-in `--numeric-threads` experiment, forwarded to simulation/mission containers
  and recorded in source/runtime evidence. Defaults remain unchanged.
- Supplemental 15 October outcomes: five follow-ups fell before placement; no new
  native pass or thread-performance improvement is claimed. Full verification
  passes 803 ROS results, 152 orchestration tests and 12 native CTests.

- Checked DWB/goal-checker plugin pair reproduces and blocks Humble false arrival
  after failed path-endpoint transforms; nine real-TF/plugin regressions retain
  arrival tolerances and verify recovery.
- 15 October evidence: eight retained native attempts, including a complete
  scripted delivery and 2/4 complete tasks in the unchanged-code exploration
  cohort. All four cohort runs remained upright; cross-colour failures remain.

- Fresh locomanipulation window recordings, time-lapse overview, pickup/lift and
  loaded-transport GIFs with still-image alternatives. Source manifests retain
  all seven failed capture attempts; no complete-delivery footage is claimed.
- Reproducible window-capture/GIF-export tools and a locomanipulation-led README
  banner and gallery; earlier navigation recordings remain available separately.

- Shared strict performance-outcome validation, explicit matrix run IDs,
  pre-recorded cohorts, atomic summaries, bounded outer execution and concurrent
  matrix exclusion. Malformed/startup-failed attempts remain visible.
- Deterministic concurrent compositor publication and measured-braking regressions,
  including frozen/future/nonfinite state, clock regression and action cleanup.

- Coherent joint/IMU sensor-delay training, explicit published-action feedback,
  and parity-checked intermediate actor export. Retained cycle-06 qualification
  gains and its low-speed turn/stop regression; deployment defaults are not promoted.
- Opt-in native command-application sampling, LowCmd publication brackets, and
  content-matched downstream timing with duplicate ambiguity and device-time ages.
- Independent measurement-delay and scene-contact diagnostics for native probes;
  Humble-compatible DDS message timing for matched policy observations.
- Extended native turn-to-rest, inference-outage and observation-cadence probes;
  bounded timestamped policy-recovery telemetry and deduplicated evidence export.
- Native turn-calibration and serial exploration colour/layout validation tools,
  with retained failed attempts and policy/source identities.
- Heading-aware camera search, independent support-surface proposals, calibrated
  camera coverage and stable overlapping region identities.
- Reactive payload training with missed-update exposure and an explicit
  native-foot-contact fidelity profile; candidates require native qualification.

### Changed

- Workspace interruptions revoke nominal docking intent and reset coasting
  compensation without extending the docking deadline. Full verification passes
  834 ROS results, 159 orchestration tests and 12 native CTests.

- Device-tick sampling now enforces a full policy period after each accepted
  observation, avoiding 2–6 ms catch-up samples after late callbacks. Actual device
  gaps remain distinct from bounded filter integration time in diagnostics.
- Integrated verification passes 807 ROS results, 155 orchestration tests and
  12 native CTests. Zero-person passes do not establish general/crowd reliability.

- Scan-registration displacement is bounded by time since the last accepted
  acquisition, shared by primary/fallback matching and capped at the original
  30 cm limit. Rejection diagnostics preserve pose/reference state.
- Transport builds include the native navigation plugin package. Full integrated
  verification passes 791 ROS results, 142 orchestration tests and 12 native CTests.

- Fixed carry/rest postures share current-waist measured completion validation;
  incomplete/nonfinite arm or waist measurements cannot pass via FK defaults.
  Restoration retains arm authority until the existing 3.5 cm tolerance passes.
- Native capture traces exposed stops longer than the initial 1.5-second braking
  allowance. Braking now has up to five seconds within the original operation
  deadline, retaining freshness, velocity and dwell requirements. The complete
  bridge suite passes **588 tests**; native capture failures remain documented.

- Device-clock policy execution now requires LowState tick progression within
  the existing sensor timeout. Duplicate/reordered packets cannot replace
  accepted measurements or renew upright evidence. Frozen streams invalidate
  pending inference and enter the sensor-fault hold with zero motion feedback;
  diagnostics distinguish receipt age, progress age and rejected tick counts.
  The complete bridge suite passes 567 tests.

- Compositor target selection and publication share the control-state lock.
  Normal navigation/inspection braking records `settled`, `timeout` or
  `stale_state`; unsuccessful settling raises instead of silently advancing.
- Bootstrap simulator pins match reviewed patch bases; MJLab fallback reads its
  revision from the patch manifest. Integrated verification passed 744 ROS test
  results, 142 orchestration tests and 12 native CTests on 14 October.

- Arm actions use asynchronous completion so command/watchdog callbacks remain
  serviceable with one executor thread; two threads remain the default. Added
  opt-in duplicate-target coalescing with immediate changed-target forwarding.
- Brief inference dropouts retain validated joint targets and policy history;
  sustained faults escalate to state hold without extending inference deadlines.
- Inspection turns detect excessive fresh measured displacement and request
  braking; optional search can retain the partial view and replan.
- Collision-checked scan/brake requests supersede obsolete escape plans when
  nominal clearance permits, avoiding unnecessary inspection/braking translation.
- Tabletop IK uses an outward-shoulder workspace to avoid the diagnosed inward
  torso-contact branch; native geometry and mission follow-ups are retained.
- Continuous nominal alignment, bounded braking, whole-route navigation, prompt
  arm-compositor forwarding, static-field reuse and vectorized tracker decoding.
- Discovery evidence survives managed navigation termination; retreat settling,
  carry-posture reference and bounded search-reserve recovery are improved.
- Recorded zero-person medium-building delivery progress and unresolved crowd
  failures in the [performance implementation report](docs/PERFORMANCE_IMPLEMENTATION_20261012.md).

### Repository maintenance

- Software citation metadata, community conduct guidance and actionable security-reporting routes.
- Maintainer/release runbooks and a contributor test-selection guide.
- Focused repository-quality checks, generated-document validation and PDF build provenance.
- Change-aware CI with a stable final verification gate and conditional review-PDF rebuilds.
- Optional pre-commit checks for malformed files, conflict markers, private keys and large additions.

## 2026-10-06 — Hardware-facing development snapshot

[`d1d54ff`](https://github.com/DavidBatty95/Geoff-Sim/commit/d1d54ff)

- Published G1 command CRC/machine-mode handling, sensor-site IMUs, measured TF
  and hardware point-cloud layout support.
- Added BrainCo interface adapters, asynchronous supervision, native-delay payload
  training and recoverable RGB-D transport work.
- Retained native transport and verification evidence with explicit limitations.
- Added gap-aligned research phases and candidate-paper planning.

See [hardware fidelity](docs/HARDWARE_FIDELITY.md) and
[native payload contract](docs/NATIVE_PAYLOAD_CONTRACT.md). These are simulation
development results; crowd and physical-hardware qualification remain separate.

## 2026-10-05 — Research review and repository navigation

[`95f6994`](https://github.com/DavidBatty95/Geoff-Sim/commit/95f6994) ·
[`b2a29ff`](https://github.com/DavidBatty95/Geoff-Sim/commit/b2a29ff) ·
[`7fc09a5`](https://github.com/DavidBatty95/Geoff-Sim/commit/7fc09a5)

- Expanded the literature review into a single 74-page PDF with 50 critical
  appraisals, a linked bibliography and a novelty roadmap.
- Put intended research gaps first on the Research page.
- Added a topic-clustered resource library, online-reference directory and
  generated animated navigation shared across project pages.

## 2026-10-05 — Navigation and payload workflows

[`3a7ceac`](https://github.com/DavidBatty95/Geoff-Sim/commit/3a7ceac)

- Published navigation corrections, payload-training and experimental transport
  tools, regression tests and retained experiment records.
- Refreshed reviewed simulator patches to reproduce the maintained source snapshot.

## 2026-09-22 — Worlds and inspectable evidence

[`7c4cd3e`](https://github.com/DavidBatty95/Geoff-Sim/commit/7c4cd3e) ·
[`886abb2`](https://github.com/DavidBatty95/Geoff-Sim/commit/886abb2)

- Added representative navigation worlds, the browser results dashboard and
  portable benchmark evidence.
- Published the initial transport-development plan.

Earlier milestones are indexed in [project history](docs/resources/HISTORY.md).
The [Git history](https://github.com/DavidBatty95/Geoff-Sim/commits/main) is the
complete change record; historical results retain their original configuration.
