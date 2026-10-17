# Published experiment records

<!-- geoff-nav:start -->
<p align="center">
  <a href="../../README.md"><img src="../media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../../README.md#quick-start"><img src="../media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="../DEMO_GUIDE.md"><img src="../media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="../INDEX.md"><img src="../media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="../REPOSITORY_GUIDE.md"><img src="../media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="../PROJECT_STATUS.md"><img src="../media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="../PROJECT_PLAN.md"><img src="../media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="../RESULTS_REVIEW.md"><img src="../media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="../literature_review/README.md"><img src="../media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="../DEVELOPMENT.md"><img src="../media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

This directory holds portable measurements and provenance for inspection without
a running simulator. It includes failures and invalid attempts, as well as
successful runs. Raw logs and local policies live in the artifact locations
described by the [repository guide](../REPOSITORY_GUIDE.md#where-results-go).

## Start with a dataset

Actuation alignment: [four matched states](native_actuation_20261017.json),
[initial target reconstruction](target_clipping_20261017.json),
[phase-split target audit](target_clipping_phases_20261017.json),
[Euler candidate matrix](euler_dense_20261017.json),
[interrupted clipping matrix](euler_clipped_dense_20261017.json), and
[separate follow-up](euler_clipped_followup_20261017.json).
[Interpretation and limits](../ACTUATION_PARITY_20261017.md).

17 October: [30-probe dense timing sweep and training audit](dense_transitions_20261017.json)
and [eight lower-delay controls](transition_delay_controls_20261017.json).
[Interpretation and unpromoted status](../DENSE_TRANSITIONS_20261017.md).

Stop-state investigation: [14 broad branches](stop_branches_20261016.json),
[four adjacent-update branches](stop_branch_tick_20261016.json),
[integer-clock probes](step_clock_20261016.json),
[legacy-clock controls](legacy_clock_control_20261016.json), and
[continuation regression](checkpoint_branch_regression_20261016.json).
[Interpretation](../STOP_BRANCHES_20261016.md).

16 October: [40-probe delay/seed matrix](delay_seeds_20261016.json),
[four duration follow-ups](delay_duration3_20261016.json),
[exact continuation results](checkpoint_parity_20261016.json) and
[initial dtype mismatches](checkpoint_initial_20261016.json).
[Interpretation](../CHECKPOINT_DELAY_20261016.md).

Matched reward ablation: [24-probe matrix](stop_ablation_20261015.json),
[four duration follow-ups](stop_ablation_duration3_20261015.json), and
[zero-probe setup failure](stop_ablation_setup_failure_20261015.json).
[Interpretation](../STOPPING_ABLATION_20261015.md).

Stopping-trained candidate: [eight matched probes](stop_candidate_20261015.json),
[training provenance](stop_candidate_training_20261015.json) and
[two command-only replays](stop_replay_20261015.json).
[Interpretation and unpromoted status](../STOPPING_CANDIDATE_20261015.md).

Loaded follow-up: [four loaded probes](loaded_stopping_20261015.json) (3/4 pass)
and [two corrected-rollout missions](loaded_followups_20261015.json) (1/2 complete).
[Conditions and limits](../LOADED_STOPPING_20261015.md).

Latest stopping records: [eight scored probes](native_stopping_20261015.json)
(4/8 pass) and [three integrated follow-ups](rollout_timing_20261015.json)
(1/3 complete). [Interpretation and scope](../NATIVE_STOPPING_20261015.md).

| Record | Purpose |
| --- | --- |
| [avoidance_dataset.json](avoidance_dataset.json) | Published v12 representative-world cohort used by the default dashboard |
| [performance_development_20261015.json](performance_development_20261015.json) | Eight native navigation-integrity attempts; scripted delivery and 2/4 unchanged-code exploration tasks complete, all failures retained |
| [placement_alignment_20261015.json](placement_alignment_20261015.json) | Cumulative 15 October export: earlier eight attempts plus five later pre-placement falls; earlier passes are not new alignment qualifications |
| [policy_cadence_20261015.json](policy_cadence_20261015.json) | Cumulative 23-attempt export; ten cadence follow-ups include a 4/4 compact cohort, medium delivery, three recording completions and one recording-run fall |
| [crowd_workspace_20261015.json](crowd_workspace_20261015.json) | Seven prefix-selected workspace attempts: zero-person control complete, all six one-person attempts incomplete |
| [crowd_workspace_diagnostics_20261015.json](crowd_workspace_diagnostics_20261015.json) | Receipt-aligned chronology and perception diagnostics for the contact/fall docking follow-up |
| [cadence_baseline_failure_20261015.json](cadence_baseline_failure_20261015.json) / [cadence_baseline_success_20261015.json](cadence_baseline_success_20261015.json) | Full-trace reanalysis: both earlier outcomes contained 2 ms observation intervals; hashed source traces, not causal proof |
| [demo_results.json](demo_results.json) | Earlier visual, mock, oracle and locomotion records |
| [avoidance_v82_matrix.json](avoidance_v82_matrix.json) | Consolidated investigation snapshot with recorded replay data |
| [avoidance_v84_matrix.json](avoidance_v84_matrix.json) | Later complete matrix: 10/18 strict passes, six contacts |
| [avoidance_v85_alignment_audit.json](avoidance_v85_alignment_audit.json) | Targeted evaluation-alignment diagnostic, not a complete release matrix |
| [payload_training_cycle_01.json](payload_training_cycle_01.json) | Payload-training evaluation and native candidate attempts |
| [hardware_fidelity_20261006.json](hardware_fidelity_20261006.json) | Nominal model and live articulated-hand interface audits; no physical calibration |
| [avoidance_hardware_fidelity_smoke_20261006.json](avoidance_hardware_fidelity_smoke_20261006.json) | Updated-interface native one-person smoke: 3/3 goals, zero contacts, strict pass |
| [transport_hardware_fidelity_20261006.json](transport_hardware_fidelity_20261006.json) | Two unchanged-code dual-delivery passes following interface/sensor changes |
| [transport_people_clearance_20261006.json](transport_people_clearance_20261006.json) | Wider-layout/people-avoidance development attempts; no complete pass, failures retained |
| [exploration_search_20261006.json](exploration_search_20261006.json) | Five evolving unknown-environment search attempts, timing/source evidence and failures; no complete delivery |
| [room_search_and_tasks_20261007.json](room_search_and_tasks_20261007.json) | Room/task/approach-side follow-up, all 20 attempts; compact red-to-blue delivery passed, medium task still unqualified |
| [exploration_crowd_20261007.json](exploration_crowd_20261007.json) | Three people-present room-search diagnostics, measured walker motion and planner timing; all failed, no crowded delivery pass |
| [failure_review_20261007.json](failure_review_20261007.json) | Five detailed trace reviews: overlap/fall chronology, candidate rejection reasons, phase times and frame-aligned perception diagnostics |
| [tracking_followup_20261007.json](tracking_followup_20261007.json) | Six wall-rejection/body-fault development attempts; failures and timeouts retained |
| [tracking_diagnostics_20261007.json](tracking_diagnostics_20261007.json) | Track-load and timing evidence for those attempts, including the enabled/disabled pilot |
| [native_response_probes_20261007.json](native_response_probes_20261007.json) | Short translation/stop probes, engine versions and raw-artifact hashes; not a fitted response model |
| [overtaking_followup_20261007.json](overtaking_followup_20261007.json) | Three earlier-avoidance/settling attempts; all failed, experimental guard remains opt-in |
| [overtaking_diagnostics_20261007.json](overtaking_diagnostics_20261007.json) | Incident and phase analysis for those attempts |
| [turn_transition_probes_20261007.json](turn_transition_probes_20261007.json) | Isolated turns and filtered-command replays; not exact incident reconstructions |
| [transport_native_contract_20261006.json](transport_native_contract_20261006.json) | Native-inertial/delay development attempts and earlier dual-delivery passes |
| [transport_acquisition_approach_20261006.json](transport_acquisition_approach_20261006.json) | Acquisition-stamp/viewpoint/support-edge follow-up, including failures |
| [visual_goal_handoffs.json](visual_goal_handoffs.json) | Unscored goal-cycling observations |
| [avoidance_r1_baseline_20261005.json](avoidance_r1_baseline_20261005.json) | Instrumented synchronized courtyard baseline; failed trial retained |
| [avoidance_r3_profile_20261005.json](avoidance_r3_profile_20261005.json) | Scoring-phase profiling pilot; failed trial retained |
| [avoidance_r3_bounded_20261005.json](avoidance_r3_bounded_20261005.json) | Bounded priority pilot: one of two strict passes |
| [avoidance_r2_stop_generation_20261005.json](avoidance_r2_stop_generation_20261005.json) | Stop-generation follow-up: both crowd cases failed |
| [transport_runtime_capture_20261005.json](transport_runtime_capture_20261005.json) | Two deliberately short startup/deadline diagnostics, not delivery acceptance |
| [avoidance_r2_atomic_20261005.json](avoidance_r2_atomic_20261005.json) | Atomic planner-command handoff pilot; seed-44 failure retained |
| [avoidance_r2_alignment_20261005.json](avoidance_r2_alignment_20261005.json) | Checked alignment-priority pilot; one of two strict passes |
| [avoidance_r3_supervisor_timing_20261005.json](avoidance_r3_supervisor_timing_20261005.json) | Supervisor timing baseline; failed crowd case |
| [avoidance_r3_prepared_20261005.json](avoidance_r3_prepared_20261005.json) | Immutable prediction preparation; one of two strict passes |
| [avoidance_r3_batched_20261005.json](avoidance_r3_batched_20261005.json) | Batched static checks; both crowd cases failed |
| [avoidance_r2_protocol_smoke_20261005.json](avoidance_r2_protocol_smoke_20261005.json) | One-person, one-goal native protocol smoke passed |
| [avoidance_r2_protocol_handoffs_20261005.json](avoidance_r2_protocol_handoffs_20261005.json) | Full-pose envelope: one person, three goals, strict pass; includes lineage summary |
| [avoidance_r3_async20_20261005.json](avoidance_r3_async20_20261005.json) | Asynchronous search at 20 Hz: both crowd cases failed |
| [avoidance_r3_async50_20261005.json](avoidance_r3_async50_20261005.json) | 50 Hz candidate: no strict passes; seed 42 completed without contact but below score threshold |
| [avoidance_r3_async_handoffs_20261005.json](avoidance_r3_async_handoffs_20261005.json) | Idle-drain follow-up: one person, three goals, strict pass |

Version numbers identify development datasets, not software releases or rankings.
Files named `pilot`, `validation`, `smoke` and `matrix` may contain different
worlds and trial counts. Consult the [investigation ledger](../NAVIGATION_RELIABILITY_INVESTIGATION.md)
and [transport history](../TRANSPORT_DEMO_PLAN.md) before comparing them.

## Inspect and export

From the repository root:

```bash
python3 scripts/avoidance_results_gui.py
# Open http://127.0.0.1:8765
python3 scripts/avoidance_results_gui.py --export docs/results/avoidance_dataset.csv
```

Use the dataset selector to keep cohorts separate. The dashboard supports the
avoidance/demo datasets; specialized payload and audit records have their own
report structures and should be read with their corresponding runbooks.

## Publishing another result

Use a fresh output name for a new experiment. Include its question, exact command,
source/configuration/policy identity, world version, seed, budget, data validity
and validator outcome. Retain all attempts and state how the record was selected.
Update the relevant experiment report and [status page](../PROJECT_STATUS.md) when
the evidence changes the project assessment. The default dataset is a deliberate
publication snapshot, not automatically the newest local experiment.
