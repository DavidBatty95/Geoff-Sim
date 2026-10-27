# Demo testing suite

`--heading-history` enables [publication-stamped attitude propagation](HEADING_HISTORY_20261027.md)
and requires both `--forecast-change-guard` and `--strategic-avoidance`. It uses
DDS source publication time, not certified sensor acquisition time, and remains
an opt-in research experiment. Both retained survey runs complete; see the report
for geometric overlaps and estimator availability.

The [forecast-invalidation follow-up](FORECAST_INVALIDATION_20261027.md) adds
`--forecast-change-guard` and `--record-invalidation-audit`, both requiring
`--strategic-avoidance` and off by default. The former enables coherent forecast
epochs, a velocity-change reserve and stricter primitive pose age; the latter
captures up to four admitted/rejected input pairs for offline diagnosis.
Guarded seed 42 still fails with overlap; these options are experimental.

The [continuation and resumption update](SUSTAINED_SURVEY_RECOVERY_20261027.md)
retains timed plans across computational misses without bypassing revalidation.
Reports now separate task-time primitive publications from post-completion
observation/shutdown activity; the latter cannot establish a survey benefit.

The [latest admission refinement](EARLY_ESCAPE_ADMISSION_20261026.md) starts
strategic planning before full enclosure and checks worker proposals with extra
clearance at sampled admission delays. It uses the same opt-in
`--strategic-avoidance` flag. Final command admission now occurs in live surveys,
but crowd avoidance and delivery remain unqualified.

The strategic option now uses [process-isolated escape search](ESCAPE_PRIMITIVES_20261026.md)
with copied forecasts, expiry/context rejection and fresh full-plan validation.
It remains experimental; completed worker plans are not counted as executed or
successful escapes.

`--strategic-avoidance` enables the experimental multi-person pincer predictor
and all-person refuge selection. It is off by default; see the
[strategic avoidance results](STRATEGIC_AVOIDANCE_20261025.md) before interpreting
its survey runs as improvement or qualification.

The [25 October recovery follow-up](SURVEY_RECOVERY_IMPLEMENTATION_20261025.md)
adds `--test-stage survey|pickup|delivery` and `--contact-force-capture` to the
transport runner, with all seven layered validation outcomes retained. Stage
completion is explicitly distinct from delivery; both full crowd trials failed.

The shared runner is `./scripts/run_demo_suite.sh`. Use `--list` to inspect all
presets and individual cases. Run one suite at a time: live cases share the
simulator and generated scenes.

## Layers and prerequisites

| Preset | Cases | What a pass establishes |
| --- | --- | --- |
| `quick` | Mock person avoidance | Deterministic benchmark mechanics |
| `validation` | `workspace-ci`, `dex3-contact` | ROS/native regression checks and prescribed-wrist contact physics |
| `manipulation` | `dex3-contact`, `dex3-delivery` | Component checks followed by one experimental mobile task |
| `navigation` | Mock, fixed-forward, waypoint-live | Existing scored locomotion/navigation checks |

Install the repository and Docker image first. `workspace-ci` builds the workspace
and native simulator; the pinned contact case uses MuJoCo 3.3.6 and pytest 8.3.5
through `run_mjlab.sh`. Dependency installation may require network access.
Live cases require the normal simulator/display setup; `--headless` suppresses
RViz, not necessarily the native viewer. `--fast-start` reuses a completed build.

The mobile case requires the local checkpoint
`vendor/mjlab/logs/payload_training/native_contact_cycle_05_20261012/policy.onnx`,
SHA256 `4582897893b9730781b2e2ddd682eb312479184b636582efb4f67d59fc338c6c`.
It fails preflight if unavailable or mismatched rather than substituting a policy.
A dry run can inspect commands without that checkpoint.

## Run and inspect

```bash
./scripts/run_demo_suite.sh --suite validation --headless --seed 42
./scripts/run_demo_suite.sh --suite manipulation --dry-run --headless --seed 42
./scripts/run_demo_suite.sh --case dex3-delivery --headless --seed 42 --fast-start
```

Results live under
`ros2_ws/src/dynamic_obstacle_benchmarks/results/demo_suite/suite_<UTC timestamp>/`:

- `manifest.json`: exact commands, seed, commit, dirty-tree indicator and preflight.
- `<case>/suite.log`: child output, including failed tests.
- `<case>/suite_status.json`: exit status, timeout, elapsed time and evidence scope.
- `status.json`: aggregate status and attempted cases. With default fail-fast,
  later manifest cases may be unattempted; `--continue-on-failure` runs them too.

Mobile artifacts remain in the transport runner's unique `transport_demo/dex3_suite_*`
directory; the suite status links and hashes its `result.json`. That runner also
retains source snapshots and runtime parameters. High-rate control recording is
not enabled by default to limit disk growth. Preserve failure evidence and check
free space before collecting long live sessions.

## Acceptance and interpretation

Process exit zero alone cannot pass the mobile case. The result must report
`success`, `mission_complete` and `data_valid` as true, with a verified red-object
to red-pad delivery, no fall, no drops and zero contact samples. Missing or
malformed evidence fails. A placement flag while still grasping cannot pass
because mission completion is also required.

Dry runs report `planned` cases and `dry-run` overall. Visual sessions remain
unscored. A passing regression/component preset never implies mobile qualification;
one successful mobile task would not establish a multi-condition robustness
matrix. The [latest retained performance](DEX3_ROTATIONAL_RETENTION_20261023.md)
has zero completed deliveries in seven 23 October trials.

The runner enforces per-case wall-clock timeouts, terminates the child process
group, preserves output, and returns nonzero on failed or interrupted suites.
Regression tests include partial-line output at timeout and rejection of
held-object, invalid, missing and malformed delivery evidence.

## Latest suite execution — 23 October 2026

`suite_20261023T214838610371Z` passed both validation cases:

- `workspace-ci`: 65.801 seconds; 903 ROS results, 234 orchestration tests and
  12 native CTests passed.
- `dex3-contact`: 28.798 seconds including environment setup; 16 pinned-MuJoCo
  Python tests passed.

`suite_20261023T215025724336Z` validated the manipulation command plan as a dry
run. No new live mobile delivery was executed for this suite publication.

## Walk-then-reach follow-up

Pickup now keeps the arms under locomotion-policy control through staging and
final docking. Before taking arm authority or raising the hand, it requires a
fresh cylinder observation, fresh odometry, arrival within the existing 7 cm /
0.15 rad docking bounds, and 0.3 seconds of advancing low-velocity samples
(planar speed below 0.08 m/s and yaw rate below 0.15 rad/s). Pickup recovery clears
the fingers, restores the rest posture and releases arm authority before walking
to re-dock. The loaded carry posture is still used after grasping.

Live run `dex3_walk_then_reach_20261023` verified the new command order: final
docking before arm preparation, then rest-posture restoration before re-docking.
The arm transition still caused backward base displacement and a reach recovery;
the second docking stopped because the fresh observed-range/settling gate was not
satisfied. The result was valid but unsuccessful, with no fall, dropped object or
collision contact sample. Full logs, source snapshot and evaluator history are
retained under that run ID in the local transport results directory.

This adds one failed attempt to the earlier seven-run 23 October cohort: **0/8
completed deliveries**. The sequencing improvement is live-observed, while smooth
arm-transition balance and repeatable pickup remain development priorities.
Follow-up verification passed 61 focused recovery/docking/posture tests, then
the complete workspace CI: 909 ROS results, 234 orchestration tests and 12 native
CTests. Six new pickup-gating cases cover successful settled arrival and refusal
while moving, distant, stale, misaligned or missing the observed cylinder.
