# Benchmark Experiment Batches

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

The benchmark batch tools run and aggregate a frozen matrix of methods,
scenarios, seeds, and repetitions. They preserve failed, invalid, missing, and
timed-out trials instead of selecting only successful result files.

These tools are experiment infrastructure. The included mock specification
validates orchestration and repeatability; it does not compare social-navigation
algorithms or provide physical avoidance evidence.

## Commands

Inspect the expanded mock matrix without launching Docker:

```bash
./scripts/run_benchmark_batch.sh \
  --spec ros2_ws/src/dynamic_obstacle_benchmarks/config/mock_experiment_v1.yaml \
  --dry-run
```

Run the complete matrix:

```bash
./scripts/run_benchmark_batch.sh \
  --spec ros2_ws/src/dynamic_obstacle_benchmarks/config/mock_experiment_v1.yaml
```

The command prints the unique batch directory. Aggregate that directory with:

```bash
python3 scripts/aggregate_benchmark_batch.py \
  ros2_ws/src/dynamic_obstacle_benchmarks/results/batches/<batch-id> \
  --strict
```

`--strict` fails only when expected trials are not evaluable. Algorithmic
failures remain evaluable observations and do not make aggregation incomplete.

Live wrappers accept the exact output path supplied by the batch runner:

```bash
./scripts/run_extreme_lab_person_avoidance_benchmark.sh \
  --output ros2_ws/src/dynamic_obstacle_benchmarks/results/example/result.json
```

`--output` and `--result-dir` are mutually exclusive. Both modes must resolve
under the writable `ros2_ws` bind mount. Unsupported external paths fail instead
of being silently redirected inside the container. Reports, simulation logs,
JSONL, bags, and benchmark companion files derive from the selected JSON stem.

Development filters are available without changing the specification:

```bash
./scripts/run_benchmark_batch.sh \
  --spec ros2_ws/src/dynamic_obstacle_benchmarks/config/mock_experiment_v1.yaml \
  --method mock_reference \
  --scenario crossing_smoke \
  --max-trials 1
```

Do not use filtered runs as the final paper dataset unless the filtering was
declared in the experimental protocol before execution.

## Specification Contract

Experiment specifications are versioned YAML mappings. The batch runner owns
`--seed` and `--output`; method and scenario arguments may not provide them.
Commands are argv token lists and are never passed through `shell=True`.

```yaml
schema_version: 1

experiment:
  id: experiment_id
  seeds: [1, 2, 3]
  repetitions: 2
  timeout_s: 300
  output_root: ros2_ws/src/dynamic_obstacle_benchmarks/results/batches
  require_repository_output: true

aggregation:
  confidence_level: 0.95
  bootstrap_samples: 2000
  bootstrap_seed: 91357
  reference_method: baseline
  continuous_metrics: [final_score, minimum_body_clearance_m]
  binary_metrics: [success, collisions, fallen, no_movement]

provenance:
  files:
    - id: benchmark_config
      path: path/to/config.yaml
  repositories:
    - id: simulator
      path: ros2_ws/src/unitree_mujoco
  container_services: [unitree-g1-sim]

methods:
  - id: baseline
    command: [scripts/run_person_avoidance_benchmark.sh]
    args: [--backend, mock, --minimum-score, "0"]

scenarios:
  - id: smoke
    args: [--mode, smoke, --people, "2", --goals, "2"]
```

Complete-token substitutions are supported for:

- `{repo}`
- `{run_dir}`
- `{trial_dir}`
- `{attempt_dir}`
- `{result_json}`
- `{method_id}`
- `{scenario_id}`
- `{seed}`
- `{repetition}`

Methods are expanded in listed order, then scenarios, seeds, and repetitions.
The resolved trial list is written to `manifest.json` before execution.
Required provenance files are hashed with SHA-256 before execution. Repository
entries record commit and dirty state. Container services record the local image
ID when Docker makes it available. Final specifications must explicitly list the
policy, scene, social-navigation configuration, benchmark configuration, runtime
wrapper, simulator repository, and container service used by the experiment.

## Artifact Layout

```text
<batch-dir>/
  manifest.json
  resolved_spec.yaml
  status.json
  trials/
    method=<id>/scenario=<id>/seed=<seed>/repetition=<n>/
      trial_manifest.json
      trial_status.json
      attempt=001/
        stdout.log
        stderr.log
        result.json
        result_goals.csv
        result_events.csv
        result_scenario.yaml
  aggregate/
    aggregate.json
    runs.csv
    groups.csv
    paired_differences.csv
```

`runs.csv` always has one row for every trial in the manifest, including trials
that were not run or did not produce valid artifacts.

## Outcome Classification

Execution state and benchmark outcome are separate. A wrapper may exit nonzero
after producing a valid algorithmic-failure result, while a zero exit without a
supported result is not a successful observation.

Current outcome values are:

- `valid_success`
- `valid_algorithm_failure`
- `validation_failed`
- `benchmark_invalid_infrastructure`
- `benchmark_invalid_algorithm`
- `artifact_missing`
- `artifact_malformed`
- `schema_unsupported`
- `not_run`

Infrastructure-invalid reasons must be declared in the specification. Collisions,
falls, deadlocks, no movement, and algorithmic timeouts must not be classified as
infrastructure exclusions.

## Statistical Semantics

Binary outcomes use two-sided 95% Wilson score intervals by default. The output
reports the observed denominator for each rate. Success also reports a
conservative all-expected-trials rate.

Continuous metrics include count, mean, median, sample standard deviation,
minimum, quartiles, maximum, and a deterministic percentile-bootstrap interval
for the mean. Bootstrap sample count and seed are recorded in `aggregate.json`.

Paired comparisons:

1. Average repetitions within each method/scenario/seed cell.
2. Match each candidate method against the configured reference on scenario and
   seed.
3. Report candidate minus reference.
4. List seeds excluded because either side lacked an evaluable value.

Re-running aggregation from unchanged artifacts produces byte-identical JSON and
CSV output.

## Current Limitations

- Batch execution is sequential; live MuJoCo wrappers share global container
  resources and must not run concurrently.
- Resume and immutable retry attempts are not implemented yet.
- The initial oracle baseline specification is development-only and must not be
  used as final paper evidence or as a perception-only result.
- Provenance is explicit rather than inferred. An omitted policy, scene, or
  configuration is not automatically discovered, so final specifications need
  review before data collection.
- Statistical output does not replace a predeclared experimental protocol or
  justify treating repeated runs as independent observations.

## Development Baselines

`oracle_method_development_v1.yaml` defines an initial, non-final matrix:

- `nav2_only`
- `reactive_geometric`
- `deterministic_temporal`
- `full_planner_supervisor`

It uses oracle pedestrian state to isolate method behavior before introducing
perception errors. Reactive and deterministic filters run supervisor-only by
routing `/cmd_vel` directly into the supervisor. Method-specific supervisor YAML
is applied after the oracle/perception sensing profile so method and sensing
identity remain independent.

Inspect it without launching live trials:

```bash
./scripts/run_benchmark_batch.sh \
  --spec ros2_ws/src/dynamic_obstacle_benchmarks/config/oracle_method_development_v1.yaml \
  --dry-run
```

Do not open final evaluation seeds or interpret the proposed method until the G1
response envelope and pedestrian uncertainty calibration are implemented.
