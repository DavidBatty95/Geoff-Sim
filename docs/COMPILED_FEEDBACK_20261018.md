# Compiled plant, Warp backend and sensor-boundary audit

## Outcome

The audit now compares the **final compiled training plant**, not only its source
MJCF, and separates model differences from backend and feedback timing effects.
An opt-in native joint-friction correction fixes a measured wrist-roll mismatch.
Diagnostic payload-offset alignment reduces the remaining one-step joint-velocity
gap further. CPU/Warp differences are much smaller in the two tested states.

Independent policy-observation and motor-PD feedback selectors now model the
native sensor boundary explicitly. All **16 stopping probes fail** the unchanged
gate; no policy is promoted. Exact checkpoint continuation passes in **three
scenarios**, including native-sensor feedback.

## 1. Compiled-model comparison

`compare_native_actuation.py` now exports portable root, joint and payload states
from pinned MuJoCo 3.3.6 checkpoints. `compare_compiled_plants.py` builds the actual
MJLab `Scene`, including actuator/collision edits, and applies the simulation
configuration before comparing it with the native scene recompiled in the same
engine. Root/joint state and composed targets are mapped by joint name. The
payload follows each model's attachment representation.

These are two moving states at approximately **6.992 s**, captured under cycle-05
and long77. The compiled-plant audit uses **MuJoCo 3.8.1 on CPU**. This portable
state mapping is not a cross-version binary checkpoint restore or mission replay.

Evidence, in measurement order:

- [Portable native states](results/portable_native_states_20261018.json)
- [Initial compiled comparison](results/compiled_plants_20261018.json)
- [Native joint-friction correction](results/compiled_plants_friction_20261018.json)
- [Solver-budget control](results/compiled_plants_solver_20261018.json)
- [Expanded body/mass inventory](results/compiled_plants_final_20261018.json)
- [Payload-offset control](results/compiled_plants_payload_20261018.json)
- [CPU/Warp comparison](results/compiled_plants_warp_20261018.json)

These reports revisit the same two states; they are not additional independent
trials. All audits use Euler integration and the same supplied targets, separating
this work from the earlier [actuator/integrator experiment](ACTUATION_PARITY_20261017.md).

### Measured correction: wrist-roll friction

Historical training sets friction loss to 0.1 for every joint containing `wrist`
in its name. The native model instead specifies **0.2 for wrist roll**, with
0.1 only for wrist pitch/yaw. The new training flag
`--native-joint-friction` copies friction loss by native joint identity and requires
`--dynamics native_contact`. A regression checks all 29 values after final scene
compilation; historical behavior remains reproducible without the flag.

Maximum absolute joint-velocity difference from the native CPU plant after one
2 ms step:

| Diagnostic configuration | Cycle-05 captured state | Long77 captured state |
| --- | ---: | ---: |
| Historical native-contact training model | 0.03494 rad/s | 0.01482 rad/s |
| Native joint friction | 0.02090 rad/s | 0.002193 rad/s |
| Native friction + solver budgets | 0.02090 rad/s | 0.002193 rad/s |
| Above + native probe payload offset | **0.002059 rad/s** | **0.0006544 rad/s** |

The final joint metric is about 94% and 96% lower in these two states. This is
one-step model agreement, not a stopping or mission reliability improvement.
Individual root-velocity components do not improve monotonically at every stage.

### What the controls show

- Actuator generalized forces match to roughly 2×10⁻¹⁴ in CPU comparisons.
- Active foot contact positions, penetration distances and dimensions match in
  both states (six contacts in the first, two in the second).
- Native and training solver limits differ (100 versus 10 main iterations;
  50 versus 20 line-search iterations), but both solves complete in **4 or 5
  iterations** here. Raising the budgets changes neither result.
- The native robot subtree weighs **35.112142 kg**, with a separate 0.15 kg free
  payload. The training subtree weighs **35.262142 kg**, including its rigid load.
  The mass difference is therefore accounted for by attachment representation.
- Moving the rigid diagnostic training load from **0.19 m to 0.12 m** along the
  wrist-local x axis reduces the residual joint error. This matches the probe
  fixture only; it is not a default for all real or simulated grasps. Native
  weld compliance and captured payload velocity still differ from a rigid child.

The friction correction is available in training. Solver-budget and payload-offset
changes are explicit audit controls, not promoted training defaults.

### Backend comparison

With the matched diagnostic training model, CUDA Warp and CPU integration differ
by **1.42×10⁻⁶ and 7.23×10⁻⁶ rad/s** in maximum joint velocity after one step.
Joint-position differences are **2.77×10⁻⁸ and 4.11×10⁻⁸ rad**. The runtime reported
Warp 1.12.0. These errors are substantially smaller than the initial model gaps,
but two one-step states do not establish long-horizon or all-contact backend parity.

## 2. Native sensor/control boundary

In the normal running path, `StepOrHold` applies commands, calls `mj_step`, then
captures state. The bridge reads joint positions/velocities from `sensordata` for
both LowState and motor PD. MuJoCo's post-step sensor values describe the state
used during that step, while `qpos/qvel` have already advanced. A pinned-engine
regression reproduces the **one-physics-step (2 ms) epoch difference**.

The legacy Python probe uses current `qpos/qvel` for joints and sensor arrays for
IMU values. New independent options allow the effects to be isolated:

- `--joint-feedback state|sensors`: policy joint-observation source.
- `--pd-feedback state|sensors`: motor PD position/velocity source.

Both default to `state` for historical probe reproduction. `sensors/sensors`
models the bridge's normal local sensor boundary; it does not recreate DDS
scheduling, variable actuator acknowledgement or an entire ROS mission. The
helper validates unique scalar joint sensors rather than assuming array offsets.
Checkpoint identity and matrix protocol checks include both selections.

## 3. Feedback-factorial native probes

Sixteen serial probes compare the four source combinations using cycle-05 and
the retained Euler-plus-clipping candidate. All use a native 150 g load,
published-action feedback, 60 ms action delay, the integer-step clock and ±0.45
lateral commands for two seconds. No new policy is trained in this audit.

Cells show post-stop travel and outcome. Every condition fails at least one
unchanged stopping requirement.

| Policy joints / PD source | Cycle-05 positive | Cycle-05 negative | Clipped77 positive | Clipped77 negative |
| --- | --- | --- | --- | --- |
| [state / state](results/feedback_state_state_20261018.json) | 0.321 m; late | 1.813 m; fall | 0.248 m; late | 0.954 m; late |
| [state / sensors](results/feedback_state_sensors_20261018.json) | 1.652 m; fall | 1.236 m; moving at end | 0.283 m; late | 2.788 m; moving at end |
| [sensors / state](results/feedback_sensors_state_20261018.json) | 0.314 m; late | 2.458 m; fall | 0.247 m; late | 0.994 m; fall |
| [sensors / sensors](results/feedback_sensors_sensors_20261018.json) | 1.223 m; fall | 1.778 m; fall | 0.284 m; late | 0.781 m; fall |

All processes and protocol audits completed. Seven probes record falls. The
source combinations alter the complete trajectory from startup, so this is not
an isolated stop-instant intervention. The outcome differences support explicit
feedback-boundary modelling, not selecting a source because one condition looks
better or relaxing the stopping gate.

[Exact continuation regression](results/feedback_checkpoint_20261018.json) passes
all three cases: legacy loaded, native-sensor loaded, and delayed-sensor/cadence/
outage continuation. The tests recompute 400, 400 and 254 samples respectively.
The outage case reproduces a fall; continuation parity is not locomotion success.

## Reproduction and verification

```bash
docker compose -f docker-compose.yml -f docker-compose.gpu.yml run --rm --no-deps -T unitree-g1-sim \
  /workspace/scripts/run_mjlab.sh python /workspace/scripts/compare_compiled_plants.py \
  --states /workspace/docs/results/portable_native_states_20261018.json \
  --native-joint-friction --match-native-solver --match-payload-offset --warp \
  --output /workspace/ros2_ws/src/dynamic_obstacle_benchmarks/results/compiled_review.json
```

Omit the diagnostic flags individually for the corresponding controls; omit
`--warp` and the GPU compose file for CPU-only audits. Use new output paths.
The native stopping matrix accepts the two feedback flags independently alongside
`--command-clock physics_step`. Its probe environment remains pinned to MuJoCo
3.3.6; full compiled-model/backend comparisons use the MJLab environment's 3.8.1.

Full CI passes **866 ROS results (670 bridge), 204 orchestration tests and
12 native CTests**, the deterministic mock benchmark and MJLab environment check.
Separate environments pass **17 training-contract tests** and **6 native Python
feedback/actuator tests**. All three end-to-end continuation cases pass.

Next: model the measured sensor/PD timing during training, evaluate payload
attachment distributions rather than a single offset, and expand paired-state
backend checks to additional contacts and longer horizons. The compiled-friction
option is verified at model level; policies remain unpromoted.
