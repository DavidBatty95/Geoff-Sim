# Script entry points

<!-- geoff-nav:start -->
<p align="center">
  <a href="../README.md"><img src="../docs/media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../README.md#quick-start"><img src="../docs/media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="../docs/DEMO_GUIDE.md"><img src="../docs/media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="../docs/README.md"><img src="../docs/media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="../docs/REPOSITORY_GUIDE.md"><img src="../docs/media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="../docs/PROJECT_STATUS.md"><img src="../docs/media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="../docs/PROJECT_PLAN.md"><img src="../docs/media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="../docs/RESULTS_REVIEW.md"><img src="../docs/media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="../docs/literature_review/README.md"><img src="../docs/media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="../docs/DEVELOPMENT.md"><img src="../docs/media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

For normal use, run `./geoff` from the repository root:

```bash
./geoff worlds
./geoff demo plaza
./geoff demo cafe --people 4 --duration 300 --fast-start
./geoff demo clinic --people 4 --duration 300 --fast-start
./geoff benchmark --worlds small_clutter medium_doorway --seeds 42 43 44
./geoff results
```

Transport now defaults to Dex3 contact-only manipulation with small cylindrical
objects. `setup_dex3_assets.py` verifies pinned official assets; `dex3_hand_model.py`
generates the robot/MoveIt descriptions; `measure_dex3_contact_grasp.py` measures
complete physical hand sequences; `verify_dex3_services.py` checks the native
DDS/ROS grasp and release path. [Commands and qualification scope](../docs/DEX3_CONTACT_MANIPULATION_20261020.md).

`probe_dex3_arm_stability.py` isolates free-base standing, carry and extension
with native joint sensors and optional IMU heading feedback. Its results are
diagnostics, not delivery acceptance. `run_transport_demo.py
--manipulation-heading-hold` enables the experimental planner integration;
`--pickup-standoff` retains 0.43 m as its default.
[Protocol and retained outcomes](../docs/DEX3_ARM_STABILITY_20261022.md).

`run_transport_demo.py --arm-stability-probe --carry-posture payload_v1` runs an
isolated ROS arm protocol and enables control-stream capture. It cannot report a
successful delivery. `analyze_arm_control_trace.py RUN --output NEW.json` checks
capture coverage; `--record-control-state` also enables capture during ordinary
transport trials. `--grasp-height-offset` is an explicit experimental offset,
with zero as the default. [Tabletop-clearance findings](../docs/DEX3_TABLE_CLEARANCE_20261023.md).
`sweep_dex3_retention.py` retains declared closure and wrist-roll studies;
the contact bench supports tool-referenced rotation with `--pitch-about-tool`.
[Rotational-retention findings and reference caveats](../docs/DEX3_ROTATIONAL_RETENTION_20261023.md).

`run_demo_suite.sh --suite validation` builds/tests the workspace and runs the
pinned Dex3 component suite. `--suite manipulation` adds a strictly checked
experimental mobile task; `--dry-run` records its plan. See the
[demo testing guide](../docs/DEMO_TESTING.md) for artifacts and acceptance rules.

For offline stopping qualification, `score_native_stopping.py INPUT... --output
NEW_REPORT.json --require-pass` retains failed/unreadable probes and returns
nonzero when any selected probe fails. It refuses report overwrite and requires
the full requested observation window. [Criteria and measured outcomes](../docs/NATIVE_STOPPING_20261015.md).

For experimental successful-state rehearsal, record native probes with
`--record-observations --record-actions`, build a passing-only corpus with
`build_actor_reference.py INPUT... --output NEW.json`, and train with
`g1_payload_training.py --actor-reference CORPUS --actor-reference-weight 10`.
Run `test_payload_actor_reference.py` alongside `test_g1_payload_training.py` in
the MJLab pytest environment. [Implementation and mixed results](../docs/ACTOR_REFERENCE_20261019.md).

`run_cumulative_stopping.py` fills missing matched conditions using provenance-linked
earlier evidence; `interpolate_payload_actor.py` exports synthetic actors with parity
checks. `iterate_native_layers.py` searches serially, retaining early-rejected plans
as incomplete and accepting only fully evaluated no-fall improvements. Its default
proposal budget is unlimited; interruption, source changes and resource/setup blockers
are recorded in `status.json`. It never changes deployment automatically.
[Protocol and retained failures](../docs/CUMULATIVE_INTERPOLATION_20261019.md).

`probe_native_policy.py --payload --record-actions` adds the native 150 g loaded-start
fixture and auditable published/applied action vectors. Scoring checks retained
attachment, payload/scene contacts and action-publication consistency.
`test_native_probe_payload.py` runs separately in pinned MuJoCo 3.3.6 with pytest
plugin autoload disabled. [Commands and loaded outcomes](../docs/LOADED_STOPPING_20261015.md).

`g1_payload_training.py --objective stopping --action-feedback published
--max-delay-steps 2` enables experimental explicit-stop training. Evaluation
`--probe signed_stops` covers both lateral signs and two command durations.
Command replay rejects post-motion coverage gaps exceeding 250 ms.
[Candidate results and reproduction](../docs/STOPPING_CANDIDATE_20261015.md).

`run_native_stop_matrix.py --policy NAME=PATH ... --output NEW_DIRECTORY` runs
paired serial native probes in the pinned probe environment, retaining failures
and checking policy/protocol identity and measured delay. Defaults cover both
lateral signs, loaded/held-empty states and 40/60 ms delays. Export on the host
with `--export-summary ARTIFACT_SUMMARY --report NEW_REPORT`.
[Commands, matched ablation and limits](../docs/STOPPING_ABLATION_20261015.md).

`probe_native_policy.py --checkpoint-at TIME --checkpoint-output NEW.json` captures
physics and controller state before a step. Restore with identical probe options
and `--resume-checkpoint NEW.json`. `verify_native_probe_checkpoint.py --policy
POLICY --output NEW_DIRECTORY` checks exact continuation in two scenarios; host
export uses `--output ARTIFACT_DIRECTORY --export-report NEW_REPORT`.
Checkpoint tests run in the pinned MuJoCo environment with pytest plugin autoload
disabled. [Commands, parity results and limitations](../docs/CHECKPOINT_DELAY_20261016.md).

`run_checkpoint_stop_branches.py --policy NAME=PATH --output NEW_DIRECTORY`
compares explicitly marked stop interventions while checking parent trajectory,
prefix and physical-state identity. `probe_native_policy.py --record-support`
adds offline foot forces/heights and leg state. Both the probe and native matrix
runner accept `--command-clock physics_step`; `legacy_float` remains the default.
[Stop-time findings and reproduction](../docs/STOP_BRANCHES_20261016.md).

`g1_payload_training.py --command-profile transitions` enables the experimental
move/stop curriculum and writes `transition_audit.json` with sampled request and
duration counts. The matrix runner now reports complete timing neighborhoods;
an isolated passing offset does not make its neighborhood pass.
[Dense sweep, controls and reproduction](../docs/DENSE_TRANSITIONS_20261017.md).

`compare_native_actuation.py --checkpoint PATH... --output NEW.json` isolates
actuator/integrator behavior on captured native states in the pinned MuJoCo
environment. `audit_native_targets.py INPUT... --output NEW.json` reconstructs
deployment target clipping from recorded actions, excluding composed arms.
Training flags `--integrator euler --target-soft-limit-factor .9` opt into the
measured integration/target contract; historical defaults remain available.
[Tests, candidate results and remaining gaps](../docs/ACTUATION_PARITY_20261017.md).

`compare_compiled_plants.py --states PORTABLE_STATES --output NEW.json` audits
final scene models. Optional solver/payload controls and `--warp` isolate further
differences. Training `--native-joint-friction` requires `--dynamics native_contact`.
Native probes/matrices accept independent `--joint-feedback state|sensors` and
`--pd-feedback state|sensors` selections; both default to historical `state`.
[Commands and measured outcomes](../docs/COMPILED_FEEDBACK_20261018.md).

Training `--native-timing` requires native-contact dynamics and Euler integration;
its per-substep packet-capture hook is required, not optional metric logging.
`--payload-offset-range XMIN XMAX` samples the diagnostic load placement per episode.
`verify_training_timing.py --output NEW.json` verifies the actual GPU timing/reset
contract. Native loaded probes/matrices accept `--payload-offset-x X`.
[Commands, tests and candidate results](../docs/NATIVE_TIMING_TRAINING_20261018.md).

Every generated world and the original plaza can run as a continuous demo. The
demo launcher handles goal handoff, session duration, episode restart and logs.
Use `./geoff demo --help`, `./geoff benchmark --help` or `./geoff results --help`
for the underlying tool options.

## Choose a workflow

| Goal | Recommended entry point | Guide |
| --- | --- | --- |
| Visual exploration | `./geoff demo WORLD` | [Demo walkthrough](../docs/DEMO_GUIDE.md) |
| Scored navigation | `./scripts/run_demo_suite.sh --case onboard-waypoints` | [Suite reference](../docs/DEMO_SUITE.md) |
| Repeated world/seed trials | `./geoff benchmark` | [Experiment methodology](../docs/AVOIDANCE_EXPERIMENTS.md) |
| Existing-result inspection | `./geoff results` | [Results review](../docs/RESULTS_REVIEW.md) |
| Experimental pickup/carry | `./geoff transport --help` | [Payload policy and fixture](../docs/PAYLOAD_POLICY_TRAINING.md) |
| Experimental unknown-environment search | `python3 scripts/run_exploration_transport_demo.py --help` | [Medium rooms, random colour tasks and visual approach sides](../docs/ROOM_SEARCH_AND_TASKS.md) |
| Training/calibration | Explicit checkpoint and measurement tools | [Training guide](../docs/TRAINING_AND_CALIBRATION.md) |
| Hardware-facing interfaces | `python3 scripts/run_hardware_parity_probe.py --output vendor/mjlab/logs/probe/interfaces.json` | [Fidelity and performance](../docs/HARDWARE_FIDELITY.md) |

Run host commands from the repository root. The launchers arrange the supported
container environment; individual ROS modules require its sourced workspace.
Use the exact policy and configuration recorded by the relevant experiment.

## Maintained internal tools

| Area | Entry points |
| --- | --- |
| Installation/build | `setup_sources.sh`, `build_ws.sh`, `build_unitree_mujoco.sh` |
| Demo orchestration | `onboard_visual_session.py`, `demo_suite.py` |
| Worlds and experiments | `avoidance_worlds.py`, `world_appearance.py`, `run_avoidance_matrix.py` |
| Results | `avoidance_results_gui.py`, `analyze_avoidance_dataset.py`, `collect_demo_results.py` |
| Diagnostics | `summarize_social_run.py`, `inspect_social_trace.py`, `evaluate_onboard_perception.py`, `analyze_response_trace.py` |
| Failure review | `review_transport_failures.py` — offline chronology, phase accounting and aligned perception diagnostics |
| Native response probes | `probe_native_policy.py`, `summarize_native_probes.py` — version-checked dynamics diagnostics, independent measurement/action delay, outage/cadence injection, command-window replay and compact export |
| Motion/exploration performance | `calibrate_native_turns.py`, `summarize_motion_incident.py`, `run_performance_matrix.py` — [implementation, measured outcomes and serial colour/layout validation](../docs/PERFORMANCE_IMPLEMENTATION_20261012.md) |
| Reliable matrix execution | `run_performance_matrix.py --startup-cleanup-budget 900` — unique run IDs, atomic planned-cohort accounting, strict outcomes and timeout handling; [current contracts and limitations](../docs/PERFORMANCE_FOUNDATION.md) |
| Selected evidence windows | `publish_performance_evidence.py --date YYYYMMDD --run-prefix YYYYMMDDThh --output <report.json>` — retain all matching mission/matrix attempts with explicit selection metadata; omit unrelated probes/training |
| Numerical worker experiments | `--numeric-threads 1` on transport/matrix runners — explicit BLAS/OpenMP limits in both containers, recorded in provenance; [unqualified experiment and outcomes](../docs/PLACEMENT_ALIGNMENT_20261015.md#optional-numerical-library-worker-experiment) |
| Locomanipulation media | `record_transport_preview.py`, `export_transport_gifs.py --placement-run <run>` — owned X11 window captures, complete-workflow GIFs and all-attempt provenance; [capture/export guide](../docs/MEDIA.md#locomanipulation-recordings--15-october-2026) |
| Native command application | `run_transport_demo.py --application-timing` and `analyze_command_application.py` — sampled content-transition timing with explicit lineage ambiguity and wall/device-clock separation |
| Actuation experiments | `--arm-executor-threads {1,2}` and `--arm-repeat-period .02` on transport runners — optional scheduler/duplicate-refresh comparisons; defaults remain 2 threads and no coalescing |
| Sensor-delay policy training | `g1_payload_training.py --max-sensor-delay-steps N --action-feedback published`, `payload_sensor_delay.py`, `export_payload_actor.py` — coherent sensor lag and parity-checked local candidate exports; use the MJLab uv environment and qualify before deployment |
| Command replay | `policy_command_replay.py` — source-timed empty-hand command ingestion for the native probe |
| General research batches | `benchmark_batch.py`, `aggregate_benchmark_batch.py` |
| Response calibration | `measure_g1_fixed_command.py`, `process_g1_response_trial.py`, `assemble_g1_response_dataset.py` |
| Transport | `run_transport_demo.py`, `transport_worlds.py`, `transport_evidence.py`, `summarize_payload_results.py`, `summarize_transport_progress.py` |
| Payload/hand research | `g1_payload_training.py`, `dex3_hand_model.py`, `measure_dex3_contact_grasp.py`, `verify_dex3_services.py` |
| Hardware fidelity | `audit_robot_model.py`, `audit_sim_interfaces.py`, `run_hardware_parity_probe.py`, `summarize_hardware_audit.py` |
| Payload dynamics/perception diagnostics | `probe_native_policy.py`, `audit_payload_dynamics.py`, `profile_transport_perception.py`, `analyze_transport_perception.py` |
| Evaluation alignment | `audit_evaluation_alignment.py`, `test_evaluation_audit.py` |
| Source/docs maintenance | `source_patches.py`, `check_docs.py`, `generate_readme_graphics.py` |
| Resource library | `resource_library.py`: topic-card artwork and the complete bibliography index |
| Repository quality | `check_repository.py`, `ci_scope.py`, `install_quality_tool.py`, `test_check_docs.py`, `test_repository_quality.py`, `test_pdf_provenance.py` |
| Verification | `test_workspace.sh`, `test_workspace_inside.sh`, `test_*.py` |

The low-level shell wrappers remain because the supported orchestration uses
them. MJLab training/play/evaluation helpers are optional development tools, not
the demo interface; see [training](../docs/TRAINING_AND_CALIBRATION.md).

Transport and avoidance matrices save `response_diagnostics.json` with timing
tails and explicit unmeasured boundaries. Matrices also request the native
wrapper's `--capture-parameters` option, which retains resolved ROS values and
checks motion-setting consistency before goals. See the
[phase report](../docs/PHASE_PROGRESS_20261005.md) for reproduction and results.

## Outputs and maintenance

The dual-transport scene has wider table aisles. People-present transport runs
save a derived clearance profile, configurable with `--people-clearance-extra`.
See [the clearance follow-up](../docs/TRANSPORT_PEOPLE_CLEARANCE.md) for commands,
regression checks and retained native failures; full people-present delivery is
not yet validated by these changes.

Demo/suite/matrix launchers save artifacts under the benchmark package's ignored
`results/` tree. Training uses `vendor/mjlab/logs/`; selected portable evidence
is published under `docs/results/`. See the
[artifact map](../docs/REPOSITORY_GUIDE.md#where-results-go).

Scripts named `test_*.py` provide regression coverage. Training-specific tests
need MJLab/Torch rather than the ROS Python environment. The
[development guide](../docs/DEVELOPMENT.md) records the current invocation and
verification scope. Changes to public CLI flags should also update the demo
guide, affected package README and reproducibility examples.

README diagrams and shared navigation are generated by `generate_readme_graphics.py`;
use `--check` to detect stale SVGs or menu blocks. `readme_navigation.py` defines
button labels, icons and destinations once for every page. `check_docs.py` checks
maintained documentation links and SVG syntax, including nested button assets.

The same generator maintains the [resource library](../docs/resources/README.md)
artwork and [paper catalogue](../docs/resources/RESEARCH.md) from
`resource_library.py` and the existing verified bibliography. Add new links to
the appropriate topic collection and keep historical/upstream evidence scope explicit.

## Historical tools

Removed unused legacy tools:

- `mjlab_continue_g1_from_5597.sh`: tied to one private numbered checkpoint;
  use explicit checkpoint arguments with the maintained training helpers.
- `auto_tune_human_aware_navigation.py` and `tune_extreme_lab_survival.py`:
  old ad hoc Extreme Lab parameter sweeps; use versioned benchmark matrices and
  current source/provenance checks instead.

Their prior contents remain available in Git history.
