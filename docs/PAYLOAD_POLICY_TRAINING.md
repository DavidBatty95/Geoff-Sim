# Payload-aware locomotion and authentic hand positioning

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

This report documents a dated development snapshot. The repository includes the
transport and training implementation; reproduction commands also require the
explicitly identified local checkpoint. Later source changes do not retroactively
revalidate the recorded experiments.

## Status

This is the cycle-01 historical report. The later
[native-inertial/delay-aware cycle 03](NATIVE_PAYLOAD_CONTRACT.md) and
[hardware-facing fidelity pass](HARDWARE_FIDELITY.md) record the current dual-cube
command, new acquisition/approach checks, retained failures and publication validation.

**Two consecutive, unchanged native ROS/MuJoCo runs completed the full mission**
in the zero-person `transport_lab` fixture: sensed pickup, loaded turn/carry,
placement, retreat, measured arm restoration and authority release.

| Run (UTC directory) | Seed | Completion | Lift | Carry | Result |
| --- | --- | --- | --- | --- | --- |
| `20260925T111629Z` | 42 | 94.06 s | 0.201 m | 3.546 m | Verified placement, no fall/drop |
| `20260925T111841Z` | 43 | 88.97 s | 0.207 m | 3.513 m | Verified placement, no fall/drop |

Both finished with valid evaluator data, restored arms and no attached object.
This establishes an isolated demonstration, not robust crowd transport. There
were **11 recorded native attempts with this candidate, 2 complete passes**;
the earlier attempts used evolving integration code. Their failures are retained
in the [complete candidate evidence](results/payload_training_cycle_01.json).
Raw logs are under
`ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/`.

The hand is positioned through URDF inverse kinematics, joint trajectories and
measured joint feedback. Cartesian servo corrections target a measured residual
of at most 2.5 cm. The hand point is `(0.12, 0.01, 0)` m in the right-wrist frame,
inside the native rubber-hand mesh. The simulator only activates the magnetic
weld within 9 cm of that point and preserves the existing object/hand transform;
it does not move the box into the hand. This is a simulated magnetic attachment,
not finger-force closure. The base remains unpinned.

The camera now publishes optical-axis depth rather than ray range. Destination
markers are filled colour squares, with their decorative geometry separated
from the region reported to the evaluator. Evaluation requires fresh robot,
object and destination data. Earlier prototype results did not require the
destination record for their `data_valid` flag; retain those failures as legacy
diagnostics, not complete transport evidence.

The final fixture uses 384×216 RGB-D at 2 Hz with reliable, keep-last-one
registered-image topics. A stationary tabletop box can retain its last sensed
pose for up to 20 s through hand/near-field occlusion; attachment is still
distance-gated, and lift verification requires an image acquired after grasp.
Numerical IK runs off the ROS callback thread. Search turns request heading
without chasing centimetre-level position drift, and the robot retreats 1.1 m
from pickup before scanning. Retraction uses the tested carrying workspace,
followed by return to the measured pre-manipulation joint pose.

## First RL cycle

`scripts/g1_payload_training.py` imports the deployed official actor directly
from ONNX into a trainable 512/256/128 ELU network. Conversion is checked against
the ONNX reference implementation (observed maximum difference approximately
`1.3e-6`). The deployment contract remains **480 observations / 29 actions**:
five samples per IMU, command, joint-position, joint-velocity and action term,
using the existing policy joint order. The actor sees measured arm state.
Privileged root velocity is available only to the training critic and scoring.

The training environment uses MJLab's G1 model, native-deployment joint gains,
50 Hz control / 500 Hz physics, bounded physical arm trajectories, and a rigid
11 cm cube attached 19 cm forward of the right wrist. Payload mass and cube
inertia are updated together. Episode sampling uses:

- 20% natural-arm, nominally unloaded episodes;
- 20% externally held arms without a payload;
- 60% externally held arms with 0.05–0.5 kg payloads;
- three representative reach/carry poses with small joint variations;
- forward, reverse, lateral, diagonal, turning and stop commands.

Arm PD gains switch between the original locomotion values and the manipulation
values according to authority, matching the demo's compositor arrangement.
Payload and posture samples are selected before writing the reset physics.

The first cycle used seed 42, 1,024 environments and 300 PPO iterations:
**7,372,800 environment steps**. The pretrained actor uses learning rate `1e-5`;
the fresh critic uses `3e-4`. Artifacts are under:

```text
vendor/mjlab/logs/payload_training/cycle_01/
  manifest.json
  env.yaml
  agent.yaml
  final.pt
  policy.onnx
```

### Initialization is a separate intervention

The unmodified actor failed when the arm was driven into the carry pose, even
without added mass. A nominal-arm warm start zeros the first layer's arm-input
columns. These columns remain trainable and receive gradients from the actual
arm measurements during PPO. This is distinct from masking observations at
runtime. The original actor, warm-start-only actor and trained actor must be
compared separately; improved survival from the initialization is not an RL gain.

### Initial evaluation

The seed-71 probe uses 32 different initial-yaw environments per profile and a
28-second sequence: settle, forward, stop, left/right turns, reverse, lateral,
diagonal and final stop. It uses one fixed carry pose. Survival does not mean
perfect tracking or crowded-navigation acceptance.

The first completed candidate evaluation is retained at
`vendor/mjlab/logs/payload_training/cycle_01_eval71_retry1/evaluation.json`.
Its initial load attempt is retained separately as an aborted API-integration
attempt. The candidate stayed upright in all 32 environments in each of the
unloaded, held-empty, 150 g and 350 g profiles.

For 150 g, mean planar errors were 0.076 m/s forward, 0.083 m/s lateral and
0.096 m/s diagonal. Mean left/right yaw-rate errors were 0.222 / 0.157 rad/s.
Reverse tracking still had 0.116 m/s error. This is an experimental checkpoint;
two native fixture passes are recorded above. It remains experimental outside
that fixture, especially for reverse motion and crowded navigation.

The persisted original and initialization-only probes are in
`baseline_original_eval71/` and `baseline_zero_eval71/` beside the cycle.
At 150 g, initialization alone had lateral error 0.173 m/s and right-turn error
0.480 rad/s, versus 0.083 m/s and 0.157 rad/s after training. Reverse error
increased from 0.087 to 0.116 m/s. All four initialization-only groups already
survived; the additional RL gains are in tracking, not survival on this probe.

An additional pregrasp-pose probe also survived 32/32 sequences. The optional
`--pose-bank reach --arm-transitions` settings prepare broader IK-derived pose
coverage for a future cycle; **they were not used to train cycle 01**. Native
completion also needed the integration fixes above, so it is not attributable
to RL alone.

## Reproduce

Use the existing GPU Docker/uv wrapper from the repository root:

```bash
MJLAB_WITH_GPUS=1 bash scripts/run_mjlab_docker.sh --no-sync python \
  /workspace/scripts/g1_payload_training.py \
  --mode train --num-envs 1024 --iterations 300 --seed 42 --arm-init zero
```

Evaluate a checkpoint, preserving all four profiles:

```bash
MJLAB_WITH_GPUS=1 bash scripts/run_mjlab_docker.sh --no-sync python \
  /workspace/scripts/g1_payload_training.py \
  --mode evaluate --num-envs 32 --seed 71 \
  --checkpoint /workspace/vendor/mjlab/logs/payload_training/cycle_01/final.pt
```

Omit `--checkpoint` to evaluate the original actor; add `--arm-init zero` to
evaluate the initialization alone. Default outputs and the Warp cache use the
writable, host-mounted `vendor/mjlab/logs/` directory. Do not put persistent
artifacts in the container-only `/workspace/logs` directory.

Run an experimental native transport trial with the exported actor:

```bash
./geoff transport --world transport_lab --people 0 \
  --policy vendor/mjlab/logs/payload_training/cycle_01/policy.onnx --no-rviz
```

The explicit `--policy` option turns off the old runtime arm-observation mask.
It does not replace the published navigation policy. `transport_lab` is the
tested return route with a loaded turn. `transport_aisle` remains a separate
experimental forward-transfer fixture. Each run records settings, implementation/policy
hashes, trace and evaluator result under the benchmark results directory.

## Checks and remaining work

Final regression checks passed: 284 bridge tests, 18 bringup tests, 83 benchmark
tests, 77 general script tests, 7 Torch/MJLab script tests and all 10 simulator
CTest targets. The 3 payload tests skipped in the ROS-only environment passed
in the separate MJLab environment. Source patch export/check/pristine replay,
maintained-document link checks and whitespace checks also passed.

Focused training tests cover ONNX conversion, trainable arm-input initialization,
physical payload inertia, reset ordering and authority-dependent gains. Native
tests cover rotated hand capture without repositioning the box, rejection of
distant attachments, and off-axis optical-depth reconstruction. The bridge suite
also checks hand-point Jacobians against finite differences of the actual URDF.

Before promoting this actor beyond the isolated fixture: quantify stopping
drift and reverse/yaw regressions, and expand to additional arm poses, worlds,
people and longer runs. The present transport evaluator's person-gap
counter does not certify absence of arm/table or other static contacts.
