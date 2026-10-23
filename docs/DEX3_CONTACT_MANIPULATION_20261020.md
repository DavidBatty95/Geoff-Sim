# Dex3 contact-only manipulation — 20 October 2026

Follow-up: [21 October endurance and mobile qualification attempts](DEX3_MOBILE_QUALIFICATION_20261021.md).
The results below describe the original short-hold implementation; the follow-up
records revised closure targets and the unresolved long-hold/mobile failures.

The current simulation defaults now use **two articulated Dex3 hands** and
**physical contact grasping**. Transport objects are **40 mm diameter, 90 mm tall,
80 g cylinders**, replacing the wider cubes in the default fixtures.

Both hands pass tabletop grasp/lift/hold/lower/release tests. ROS grasp/release
services also pass through the real native DDS hand backend, including an
empty-grasp negative control. A full unpinned G1 trial physically picked up and
lifted the cylinder **16.6 cm**. Full autonomous delivery remains unqualified:
the retained mobile attempts stopped at startup, lift visibility, docking or
hand alignment. Those failures are included in the evidence, not omitted.

## Models and control

- Official Unitree seven-joint, three-finger geometry is pinned to
  `unitreerobotics/unitree_ros` revision
  `5994d4faef0a9cadd3287f8de0199a67eeb2a259`.
  [Source files, hashes and upstream license](../ros2_ws/src/unitree_g1_description/dex3_sources/manifest.json)
  are retained with the hand meshes.
- `scripts/dex3_hand_model.py` composes those hands onto the established body,
  generating the Dex3 URDF, MJCF, camera/scene variants and arms-only MoveIt SRDF.
  The model has 29 body actuators, 14 additional finger DOFs, 50 generalized
  positions and a nominal total mass of 36.1652352 kg.
- The native `Dex3Hands` backend drives named finger DOFs with bounded motor
  torques. The seven channels are thumb0/1/2, middle0/1, index0/1. Commands use
  radians, rad/s and torque through `unitree_hg/HandCmd`; feedback uses
  `unitree_hg/HandState` on `/lf/dex3/{left,right}/state`.
- Transport scenes have **zero equality constraints**: no object welds and no
  base pin. Native attachment/base-pin commands are rejected whenever Dex3
  joints are present, even if a stale custom scene includes a magnetic weld.
- Historical magnetic experiments remain available only through explicit
  `--grasp-model legacy_magnetic` fixtures. Archived stopping probes select those
  labelled legacy scenes; they are not Dex3 qualification evidence.

Hand gains, damping, friction and rotor inertia are simulation parameters, not
calibrated hardware measurements. The native torque limits use 2.45 N·m for
thumb0 and 1.4 N·m for the other channels. Targets are rate limited to 1.5 rad/s.

## Grasp feedback and workflow

`dex3_grasp.py` closes fingers and waits for opposing thumb/finger contact plus
loaded joint tracking error for a continuous 0.3 s. An empty closing hand cannot
qualify merely by reaching its target. Release requires measured open fingers
and unloaded finger contacts. Duplicate/reset/stale or invalid packets cannot
confirm a grasp. Holding targets continue after a successful service response.

The simulator publishes an explicitly marked tactile proxy: summed normal
contact force in `PressSensorState.pressure[0]`, grouped across nine sensor slots,
with marker `0x44583343` in `HandState.reserve[1]`. Self-contact is excluded.
This is a simulation force signal, not calibrated Dex3 hardware pressure.

The mobile workflow uses the Dex3 palm offset and a 12 mm measured pregrasp
alignment gate. RGB-D detection supports the small cylinder and fits its visible
side surface without consulting scene coordinates. During carrying, lost or
stale hand contact stops the mission with conservative uncertain-payload state.

Small objects can be occluded by the hand after pickup. Lift confirmation now
accepts camera evidence or **more than 8 cm of measured hand rise with fresh
joint/IMU/odometry data and persistent opposed contacts**. This uses onboard
feedback, not evaluator object positions. The MoveIt pick action independently
checks physical grasp feedback after lifting; planning-scene attachment is only
a collision-planning representation, not a physics attachment.

The stable task IDs remain `box_red`, `box_blue`, etc. for compatibility; their
default physical geometry is now cylindrical. Hand/object placement and locomotion
need further tuning under the new hand mass and contact dynamics.

## Measured evidence

| Check | Outcome |
| --- | --- |
| Right hand, Euler/2 ms contact rig | 14.27 cm minimum held lift; opposing contacts; successful release |
| Left hand, Euler/2 ms contact rig | 13.95 cm minimum held lift; opposing contacts; successful release |
| ROS → DDS right-hand grasp/release | Both services pass; 14.15 cm held lift |
| ROS → DDS left-hand grasp/release | Both services pass; 13.71 cm held lift |
| Empty-grasp negative control | Grasp correctly times out; no lift; release succeeds |
| Small position/radius perturbations | Physics regression tests pass |
| Full G1 `dex3_contact_delivery_retry_20261020` | Physical lift 16.6 cm, no fall; mission fails old camera-only lift verification |
| Later mobile attempts | Docking/alignment failures retained; no completed delivery claimed |

[All calibration results, DDS checks and five mobile attempts](results/dex3_contact_20261020.json).
Early mocap-wrist calibration failures are retained. The corrected test rig uses
actuated wrist slides so wrist velocity reaches the friction solver. No object
force, pose update or equality constraint supplies the grip. These mechanical
hand tests are distinct from whole-body autonomous delivery.

The evidence JSON's `source_sha256` records the working tree when that report
was published, rather than the source revision of each retained run. Later
feedback guards and verifier changes postdate that snapshot. The retained
results are historical evidence; the final regression checks below separately
validate the current implementation.

## Run it

Normal transport now selects Dex3 contact grasping by default:

```bash
./geoff transport --world transport_dual --task red:red --people 0
```

Rebuild after changing hand/backend code; `--fast-start` is intended for an
already-built workspace. The first retained pilot's startup failed when a late
native rebuild exhausted DDS participant slots after ROS nodes had started.

A short reproducible contact-only test:

```bash
docker compose run --rm --no-deps -T unitree-g1-sim \
  /workspace/scripts/run_mjlab.sh --with mujoco==3.3.6 python \
  /workspace/scripts/measure_dex3_contact_grasp.py --side right \
  --output /workspace/ros2_ws/src/dynamic_obstacle_benchmarks/results/my_dex3_grasp.json
```

Use `--side left` for the other hand. `--fixture-output NEW.xml` also writes a
fixture for `verify_dex3_services.py`, which exercises the ROS/DDS service path.
The latter runs in the sourced ROS workspace and uses an isolated loopback domain.

When the hand has been positioned by the arm controller, the service interface is:

```bash
ros2 service call /g1_sim_manipulation_io/grasp_object g1_msgs/srv/GraspObject \
  "{arm: right, object_id: box_red}"
ros2 service call /g1_sim_manipulation_io/release_right std_srvs/srv/Trigger "{}"
```

## Verification and media

Full workspace CI passes: **876 ROS results (680 bridge tests), 219 orchestration
tests and 12 native CTests**. The separate native suite passes eight tests,
covering physical grasp/release, empty closing, body/hand model contracts and
archived legacy payload reproduction. MoveIt configuration and manipulation
package tests also pass.

The [complete-run GIFs](media/dex3/README.md) show both ten-second hand sequences
from initial approach through final release. They do not show an incomplete
pickup excerpt or claim full mobile delivery success. The prior locomotion
search is stopped because its proxy-hand/magnetic-load dynamics are no longer
the current platform configuration.
