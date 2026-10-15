# Loaded stopping and corrected-rollout mission follow-up

## Result

Four new native probes with a **150 g payload** pass the sampled stopping gate
**3/4**. The negative-lateral published-feedback probe still fails, travelling
**0.738 m** after the stop command and completing its stationary dwell only at
**5.42 s**. A zero-person integrated delivery completes in **154.44 s**; the
one-person mission fails during braking before pickup. This is diagnostic
progress, not a demonstrated improvement in locomotion reliability.

[Scored probes and hashes](results/loaded_stopping_20261015.json) ·
[Both mission outcomes](results/loaded_followups_20261015.json) ·
[Previous held-empty investigation](NATIVE_STOPPING_20261015.md)

## Implemented improvements

- `probe_native_policy.py --payload` uses the native red object's mass and the
  existing right-wrist magnetic weld. At initialization, the object is positioned
  at the calibrated hand offset with a consistent weld transform. Carry arms
  then follow the same prescribed ramp used by held-empty probes. This is an
  offline loaded-start fixture, not a pickup or physical finger-grasp test.
- Payload attachment and position error are sampled throughout the probe.
  Robot/payload contact is reported separately and also contributes to the strict
  robot nonfloor-contact gate; payload contact with the scene, including the
  floor, remains a failure.
- `--record-actions` retains published and applied policy-action vectors,
  publication/application sequence IDs, and composed joint targets. The scorer
  checks that applied actions match their recorded publications and reports
  application age. These are simulator queue observations, not hardware receipts.
- Loaded scoring also requires positive declared mass, retained attachment,
  ≤3 cm attachment-position error throughout the samples, and complete
  payload/scene contact coverage. Missing evidence fails. Existing travel,
  dwell, motion-resumption and observation-window criteria are unchanged.

The native fixture regression checks actual MuJoCo equality residuals in all six
weld dimensions and confirms the original 0.15 kg mass. Scorer regressions reject
detached/drifting loads, missing mass/attachment/contact evidence, missing action
publications, mismatched applied vectors and nonfinite action values.

## Controlled diagnostic conditions

All four use cycle-05, MuJoCo **3.3.6**, 20 ms observation intervals, 15 s requested
duration, and a two-second lateral command starting at 5 s. They run serially
without ROS or cameras. Only direction, feedback convention and declared delay
differ. Each condition has one run; these are not a statistical reliability cohort.

| Command | Feedback | Application age | Post-stop travel | Dwell completed | Gate |
| ---: | --- | ---: | ---: | ---: | --- |
| +0.45 | Published | 40 ms | 0.135 m | 2.56 s | Pass |
| −0.45 | Published | 40 ms | 0.738 m | 5.42 s | **Fail** |
| −0.45 | Applied | 40 ms | 0.158 m | 1.62 s | Pass |
| −0.45 | Published | 0 ms | 0.118 m | 2.98 s | Pass |

All four remained upright with no sampled robot/scene or payload/scene contacts.
Maximum attachment-position error was below **0.70 mm**. Action-history audits
matched all recorded applied vectors to their publications and confirmed the
configured 40 ms or zero delay.

Final review removed an exemption for robot/payload contacts from the probe's
robot nonfloor counter. None of these four probes recorded such contacts, so
their outcomes are unaffected; the retained probe hashes identify the measured
version, and the final probe uses the stricter counting rule.

The matched negative-lateral cases support investigating delay/previous-action
feedback sensitivity with an actual load. They do not establish that deployment
can reconstruct the applied action, nor that the same behavior holds across
initial gait states, payload masses, delays or directions. The zero-delay case
passes the 3 s dwell deadline by only about 20 ms. No response gain, stopping
constant, deployed feedback convention or policy was promoted.

## Integrated missions

Both missions include the final scalar/vectorized timing correction from
`f104172`, use cycle-05 and `transport_dual`, and have valid evaluator streams.
The probe/scorer additions do not alter the onboard executive.

| Run suffix | People / seed / task | Result |
| --- | --- | --- |
| `202100Z-loaded-check-control` | 0 / 48 / red → blue | **Complete, 154.44 s**; verified placement and arm restoration |
| `202400Z-loaded-check-person` | 1 / 46 / red → red | Braking timeout after a displaced pickup-support scan; last event **89.54 s**, no pickup |

Both recorded zero contact samples and no fall. The control corrected estimated
payload alignment from **3.73 cm to 2.40 cm** before release. The one-person run
recorded minimum geometric person clearance **0.801 m**, scan displacement
**0.385 m**, and a five-second braking timeout. The failure was retained with an
empty attachment state. Completing the zero-person case does not qualify the
one-person case or demonstrate a speedup over asynchronous earlier runs.

## Reproduction and verification

```bash
docker compose run --rm --no-deps -T unitree-g1-sim \
  /workspace/scripts/run_mjlab.sh --with onnxruntime==1.23.2 --with mujoco==3.3.6 \
  python /workspace/scripts/probe_native_policy.py \
  --policy /workspace/vendor/mjlab/logs/payload_training/native_contact_cycle_05_20261012/policy.onnx \
  --output /workspace/ros2_ws/src/dynamic_obstacle_benchmarks/results/loaded_review/negative.json \
  --duration 15 --delay-steps 2 --payload --record-actions \
  --require-mujoco-version 3.3.6 --linear-y -.45 --command-duration 2 \
  --action-feedback published

docker compose run --rm --no-deps -T -e PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 unitree-g1-sim \
  /workspace/scripts/run_mjlab.sh --with mujoco==3.3.6 --with pytest==8.3.5 \
  python -m pytest -q -p no:cacheprovider /workspace/scripts/test_native_probe_payload.py

python3 scripts/score_native_stopping.py \
  ros2_ws/src/dynamic_obstacle_benchmarks/results/loaded_review \
  --output /tmp/loaded_review.json --require-pass
```

The first isolated fixture-test launch failed during automatic ROS pytest-plugin
loading because `lark` was absent. Disabling unrelated plugin autoload in the
pinned probe test environment resolved this setup failure; the native fixture
test then passed. All four physics probes produced retained artifacts.

Full workspace CI passed **866 ROS results (670 bridge), 182 orchestration tests,
12 native CTests**, the deterministic mock benchmark and MJLab environment check.
The native Python fixture test passed separately; it requires the pinned probe
environment. Focused scorer/evidence tests passed **31/31**.

## Next work

Use the new action histories to test a delay-randomized training candidate under
the deployed published-action observation convention, across both directions,
multiple command durations and actual loads. Keep cycle-05 as the baseline until
unchanged strict gates and integrated repeats justify promotion. Separately,
replay the one-person braking encounter with synchronized command/state history;
extending the timeout alone would not establish a controlled stop.
