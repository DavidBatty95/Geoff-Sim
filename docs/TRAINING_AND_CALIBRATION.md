# Training and response calibration

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

The onboard benchmark uses the checksum-verified official Unitree G1 29-DoF
velocity policy (`unitree_rl_lab_g1_29dof`, 480 observations, 29 actions). The
default demo does not depend on an unpublished locally trained checkpoint.

MJLab remains optional. See the scripts' `--help` for the current CLI:

- `scripts/mjlab_train_g1_step_foundation.sh`
- `scripts/mjlab_train_g1_navigation_finetune.sh`
- `scripts/mjlab_evaluate_g1_stability.sh`
- `scripts/write_mjlab_training_manifest.py`

Response measurements and dataset assembly use:

- `scripts/run_g1_fixed_command_test.sh`
- `scripts/extract_g1_response_metrics.py`
- `scripts/process_g1_response_trial.py`
- `scripts/assemble_g1_response_dataset.py`
- `scripts/summarize_g1_response_dataset.py`

The calibration machine contract is
`docs/schemas/g1_response_calibration.schema.json`. Preserve policy hashes,
command order, timing, initial state and failure labels in new datasets. Fixed
command response measures locomotion behavior, not pedestrian avoidance.

Prior training proposals and envelope-fitting designs are historical material
under [archive/research](../archive/research/); they are not completion claims.
