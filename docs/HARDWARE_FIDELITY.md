# Hardware-facing simulation fidelity and performance

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

Updated 20 October 2026 with the [Dex3 contact-only platform](DEX3_CONTACT_MANIPULATION_20261020.md).
The earlier 6 October BrainCo audit remains below as historical comparison.
This is a **nominal simulation model**;
there are no measurements from a physical G1 in this project.

## Interface contract

ROS automatically maps `/lowcmd` to DDS `rt/lowcmd`. Publishing a ROS topic named
`/rt/lowcmd` adds another prefix on the wire and is not the standard robot topic.
Legacy ROS `rt/` aliases are now opt-in through `legacy_rt_aliases` on the bridge,
policy controller and SLAM reset monitor; defaults use the canonical names.

| Application topic | ROS type | Semantics |
|---|---|---|
| `/lowcmd` | `unitree_hg/msg/LowCmd` | 35 slots, 29 body motors used; radians, rad/s, N·m; PR mode; observed machine mode; SDK-layout CRC |
| `/lowstate` | `unitree_hg/msg/LowState` | Measured simulated motor position/velocity/effort, primary IMU and tick; nominal SDK loop 500 Hz |
| `/sportmodestate` | `unitree_go/msg/SportModeState` | SDK high-state compatibility; not an onboard localization input |
| `/dex3/{left,right}/cmd` | `unitree_hg/msg/HandCmd` | Default: seven independent radian/rad/s/torque motor commands |
| `/lf/dex3/{left,right}/state` | `unitree_hg/msg/HandState` | Default: measured simulated finger joints and labelled tactile-force proxy |
| `/brainco/{left,right}/cmd` | `unitree_go/msg/MotorCmds` | Six normalized position/speed channels |
| `/brainco/{left,right}/state` | `unitree_go/msg/MotorStates` | Measured normalized finger position/speed; simulated current is unavailable (`NaN`) |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | `rgb8`, acquisition-stamped |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | `16UC1`, millimetres; ideal co-located depth approximation |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | Registered `16UC1`, same acquisition as colour |
| Corresponding `camera_info` topics | `sensor_msgs/msg/CameraInfo` | Matching dimensions/intrinsics, identity rectification, ideal zero distortion |
| `/camera/camera/{imu,accel/sample,gyro/sample}` | `sensor_msgs/msg/Imu` | Actual simulated sensor-site angular velocity and specific force; no fabricated attitude solution |
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | Local XYZ in metres and synthetic intensity; fields describe the actual layout |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Actual simulated Livox-site angular velocity/specific force |

`/cmd_vel`, joint-state/arm interfaces and the onboard navigation topics are
application interfaces rather than promises about every vendor firmware service.
`/evaluation/*`, `/mujoco/*` and `/g1/sim_sensor_status` are simulation diagnostics.
The complete simulator graph is intentionally not identical to a robot graph.

### G1 command correctness

The bridge now echoes `LowState.mode_machine` and waits for it before issuing body
commands. CRC covers the SDK's 1,000-byte, little-endian, zero-padded HG structure
before its CRC word, not ROS CDR serialization. The implementation is checked
against the pinned Unitree bitwise algorithm; all unused/reserve slots remain
zero for generated commands. A bounded two-entry payload cache avoids recomputing
the CRC for identical hold packets.

Reference: pinned
[`g1_low_level_example.cpp`](../ros2_ws/src/unitree_ros2/example/src/src/g1/lowlevel/g1_low_level_example.cpp)
and [`motor_crc_hg.cpp`](../ros2_ws/src/unitree_ros2/example/src/src/common/motor_crc_hg.cpp).
Battery, temperature, fault injection and other unused vendor state fields are
not a calibrated electrical/firmware simulation.

### Dex3 contact model

The current Dex3 model has14 finger DOFs,29 body actuators and nominal mass
36.1652352 kg. Finger actuation does not change the body DDS motor order.
Nine tactile slots carry summed simulated normal force in `pressure[0]`, with
marker `0x44583343` in `reserve[1]`; this is not a calibrated hardware pressure
conversion. Both-hand ROS/DDS tests confirm physical lift and release without
object constraints. Hardware tactile calibration and complete autonomous
delivery qualification remain separate work.

### Legacy BrainCo driver boundary

The stock [BrainCo G1 Revo2 driver](https://github.com/BrainCoTech/unitree-g1-brainco-hand)
uses `ros2_stark_msgs`, not the normalized `MotorCmds` application topics above.
The optional `brainco_stark_adapter` preserves the application topics while
translating to `/set_motor_multi_{slave_id}` and `/motor_status_{slave_id}`.
It uses vendor mode 6, six 0–1000 position/speed values in finger-channel order,
and requires the vendor SDK's normalized unit mode. Zero-speed requests hold
fresh measured positions; unsupported/malformed values are rejected.

Install/source the vendor `ros2_stark_msgs` package before using that adapter:

```bash
ros2 run unitree_g1_mujoco_bridge brainco_stark_adapter --ros-args \
  -p left_slave_id:=1 -p right_slave_id:=2
```

Do not run it alongside the simulation's hand-state publishers. This adapter has
conversion tests but no serial-hardware validation. Current units depend on SDK
configuration: `current_scale_to_amp` defaults to zero (unavailable/NaN); set it
only for a known conversion, such as `0.001` for mA. Tactile/PWM/current-control
services are not emulated by this position/speed adapter.

## Robot model audit

The audit verifies all 29 named body joints, joint limits against the maintained
URDF, one actuator per body joint, and native IMU sensor dimensions.

| Model | Mass | Body actuators | Proximal fingers |
|---|---:|---:|---:|
| Body model with proxy hands | 35.112 kg | 29 | 0 |
| Articulated BrainCo model | 35.588 kg | 29 | 10 |

The legacy BrainCo model has 22 finger DOFs, six driven/coupled channels per hand,
mesh contact geometry and URDF inertials. It is generated by
`scripts/brainco_hand_model.py` and built automatically into
`scene_29dof_brainco_hardware.xml`. The six-channel hand states are published at a
nominal 50 Hz. Hand friction, torque limits and coupling compliance are model
assumptions, not calibrated actuator/current measurements.

The legacy BrainCo URDF sums to 33.817 kg and differs in torso/waist revision from the native
model. That discrepancy is retained explicitly rather than asserting exact
hardware equivalence. Cycle-03 training uses the native inertials deliberately;
see [the dynamics audit](NATIVE_PAYLOAD_CONTRACT.md). The validated transport
fixture still uses proxy hands and a capture-radius weld. Its success does not
qualify articulated contact-only mobile grasping.

## Sensors, frames and performance changes

- Added physical MuJoCo gyro/accelerometer sensors at the camera and Livox sites.
  Their raw data includes mount rotation, gravity-specific force and rigid-body
  motion. Raw IMU orientation covariance is `-1`; separate accel/gyro messages
  also mark their unavailable component. Samples are acquisition-stamped, and
  stale/replayed packets cannot be republished as new measurements.
- The onboard estimator publishes raw sensor TF from measured IMU and encoders,
  not simulator root pose. Simulation frame IDs remain
  `mujoco_d435i_color_optical_frame` and `mujoco_livox_frame`; camera/LiDAR names
  are configurable. These are not claims about a particular driver's default
  frame names. Physical deployments need their calibrated TF tree. The
  dual-transport fixture's explicit 25° camera pitch is passed to measured TF.
- Odometry decodes `PointCloud2` field offsets/types, byte order, organized rows
  and padding. It no longer assumes every real-driver cloud is packed XYZI/16
  bytes. Its normalized output declares its new layout correctly.
- Native RGB-D receive polling is decoupled from the acquisition rate. It polls
  every 10 ms but publishes each acquisition only once, avoiding a further
  0–500 ms timer wait at the 2 Hz transport setting. Missing native camera data
  is not replaced with procedural images.
- Native IMU and hand schedules retain their phase, avoiding floating-point
  period accumulation that reduced nominal 100/50 Hz to ~83/47 Hz. The SDK IO
  loop now targets the reference G1's 500 Hz instead of 1 kHz.

The raycast camera is an ideal pinhole, not a full D435i stereo/exposure/noise
model. Default transport resolution/rate is **192×108 at 2 Hz**, below real-camera
capability. The default LiDAR is deliberately sparse and does not reproduce the
MID-360's full point rate, reflectivity, rolling acquisition, packet loss or
noise distribution. No calibrated noise distribution is invented in the absence
of hardware data. These bandwidth/fidelity limits are part of the declared
simulation profile.

## Validation and retained outcomes

Live articulated-hand audit `live_hands_03.json` passed topic/type, CRC/machine
mode, IMU validity, camera calibration and measured-TF checks. Observed rates
were ~325 Hz LowState, ~92 Hz IMU, 50 Hz hands, 10 Hz LiDAR and 2 Hz RGB-D.
Configured frequencies are not guaranteed wall-clock rates under ROS/viewer
load. Camera age p95 was ~310 ms; raw IMU age p95 ~17–18 ms. Maximum measured
pelvis tilt in the 20 s standing audit was 0.017 rad.

The earlier `live_hands_02` audit passed without the new TF check, with ~47 Hz hand
states and camera age p95 ~423 ms. The first audit attempt failed to serialize a
NumPy scalar; its simulation log is retained and no passing report is claimed.

Two subsequent **unchanged-code zero-person dual-delivery trials passed**:

| Run | Nominal seed | Verified | Fall/drop | Processed RGB-D age p95 |
|---|---:|---|---|---:|
| `20261006T145814Z` | 46 | both cubes | none recorded | 1.270 s |
| `20261006T150306Z` | 47 | both cubes | none recorded | 1.261 s |

The preceding seed-45 run measured 1.668 s p95. This comparison is a retained
native observation, not a controlled hardware timing experiment. Seeds do not
change the zero-person layout. Crowd-navigation acceptance remains open; the
earlier four-person pilot passed 0/2 strict trials.

A new synchronized native **one-person, three-goal** small-open smoke trial passed
strict validation: **86.752 points**, 3/3 goals, zero collisions/near misses and
0.646 m minimum body clearance. This validates the updated onboard command/sensor
path in a limited encounter, not the full crowd matrix. Its
[portable record](results/avoidance_hardware_fidelity_smoke_20261006.json) retains
the runtime settings and source identities.

Publication validation:

- Full `test_workspace_inside.sh ci` passed: **15 packages built**, **624 ROS
  test results**, no errors/failures/skips; **99 script tests**; **11 native CTests**.
- Strict deterministic mock benchmark passed: **99.818**, 2/2 goals, no collisions.
  This is test infrastructure evidence, not native crowd evidence.
- **13 isolated training/payload/schedule tests** passed in MJLab.
- Source patch export/check/pristine replay passed for simulator and MJLab.
- CI found a dashboard failure on unrelated JSON report arrays and an optional
  SciPy ABI incompatibility during development; the dashboard now filters report
  schemas, and measured TF uses dependency-free quaternion conversion.

Portable evidence: [model/live audits](results/hardware_fidelity_20261006.json) ·
[native transport pair](results/transport_hardware_fidelity_20261006.json).

## Reproduce

```bash
docker compose run --rm unitree-g1-sim /workspace/scripts/test_workspace_inside.sh ci
python3 scripts/run_hardware_parity_probe.py \
  --output vendor/mjlab/logs/my_hardware_probe/interfaces.json --duration 20
```

For a running onboard simulator, source the ROS overlay and run
`python3 scripts/audit_sim_interfaces.py --duration 20 --output REPORT.json`;
add `--require-hands` only for the articulated model. The static model audit
requires the optional MJLab/MuJoCo Python environment:
`python scripts/audit_robot_model.py --output REPORT.json`.

The reproducible transport command and local checkpoint requirements are in
[NATIVE_PAYLOAD_CONTRACT.md](NATIVE_PAYLOAD_CONTRACT.md). Generated policies and
raw run directories are not included in the source publication.
