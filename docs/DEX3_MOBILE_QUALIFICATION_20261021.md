# Dex3 grasp endurance and mobile qualification — 21 October 2026

Later follow-up: [both-hand retention, loaded retreat and placement trials](DEX3_RETENTION_AND_PLACEMENT_20261021.md).
The nine-trial results below describe the earlier development session.

**Mobile delivery remains unqualified: zero of nine retained attempts completed
delivery.** Right-hand grasp endurance improved, including one full-robot pickup
that retained its object throughout a 45-second retreat attempt. That attempt
failed the retreat deadline, not the grasp monitor. No trial reported a fall;
all nine reported valid evaluator data. These are development trials, not a
fixed-configuration reliability estimate.

## Changes and measured evidence

- Dex3 reaching now reprojects the observed object into the current pelvis frame
  after each arm trajectory and compensates measured tracking error. The generic
  arm controller's 25 mm completion band was too loose for the unchanged 12 mm
  grasp alignment gate. There are at most six attempts and an 80 mm correction
  bound; unreachable targets still require redocking.
- Docking now also estimates bounded yaw coast compensation (at most 0.12 rad).
  Final position/yaw acceptance and the original deadline are unchanged.
- Finger stiffness increased from 2 to 3 N·m/rad, with damping 0.08 and the
  existing native torque/rate limits. Stiffness alone did **not** fix retention:
  the original closure failed both static and swaying 40-second hold tests.
- A retained 27-case right-hand closure sweep tested 40-second holds with
  10 mm wrist sway at 1.5 Hz. **Two of 27 passed** lift, opposed contact and
  release gates. The selected right-hand targets are
  `(0, -1.0, -0.7, 1.5, 1.5, 1.5, 1.5)` radians. Its minimum held lift was
  **14.18 cm** throughout the extended hold, followed by successful release.
- The mirrored left-hand extended-hold test **failed**. The short both-hand
  regression remains a separate check; long-duration robustness is not
  established for both hands.
- A mobile trial with the selected closure lifted **18.09 cm**, retained contact
  throughout a 45-second retreat attempt and reported no dropped object.
  Trace evidence showed repeated `-0.30 m/s` commands reaching policy inference
  while the robot was nearly stationary. Loaded reverse references now use up
  to the existing 0.45 m/s limit when more than 0.35 m from the endpoint, with
  normal rollout/clearance/slew checks. Two subsequent trials failed before
  retreat, so this change has **not** demonstrated successful loaded retreat.

## All mobile attempts

Run IDs share prefix `dex3_contact_delivery_` and suffix `_20261021`.

| Run middle | Outcome |
| --- | --- |
| `servo` | Default upstream policy: search-turn displacement limit |
| `servo_candidate` | 19.83 cm lift; object dropped during retreat |
| `grip3` | Pickup docking deadline |
| `grip3_cycle05` | Native cycle05 policy: pickup docking deadline |
| `yawcomp` | Body-tilt limit during IK preparation; arm cancelled |
| `fixedcarry` | 15.54 cm lift; object dropped during retreat |
| `longhold` | 18.09 cm lift; object retained; retreat clearance deadline |
| `retreat45` | Contact lost during initial lift; arm cancelled |
| `retreat45_repeat` | Measured hand alignment outside 12 mm gate |

Except the explicitly labelled policy checks, trials used the experimental 95%
actor blend from 19 October. The final four used the existing `payload_v1` fixed
carry posture to avoid arbitrary redundant IK branches. No locomotion checkpoint
was promoted. Physical contact sensing still cannot establish object identity
by itself; RGB-D alignment and post-lift verification remain necessary.

## Reproduction and provenance

[Retained evidence](results/dex3_contact_20261021.json) includes all four bench
reports (including every sweep condition) and all nine mobile attempts. Bulk
bench observations remain in the raw results directory. Each mobile run retains
its source snapshot, runtime parameters, logs and trace. Report-level source
hashes describe publication-time sources, not each earlier trial's revision.

```bash
docker compose run --rm --no-deps -T unitree-g1-sim \
  /workspace/scripts/run_mjlab.sh --with mujoco==3.3.6 python \
  /workspace/scripts/measure_dex3_contact_grasp.py --side right \
  --hold-seconds 40 --sway-m .01 \
  --output /workspace/ros2_ws/src/dynamic_obstacle_benchmarks/results/NEW_hold.json
```

The existing README GIFs are historical complete ten-second bench sequences
from 20 October. They do not show this endurance sweep or a completed delivery.

## Remaining qualification work

Final code checks passed: **879 ROS test results**, **219 orchestration tests**,
and **nine pinned-MuJoCo Python tests**, including the new 40-second right-hand
sway regression. Documentation, source-patch, repository hygiene and historical
README GIF checks also passed. Passing these checks does not qualify delivery.

1. Repeat sustained holding across small alignment errors, wrist orientations
   and both hands; the right-hand sweep winner is selected calibration evidence.
2. Prevent object displacement during approach and reobserve after disturbed
   contact rather than relying on an occluded, previously stationary estimate.
3. Establish loaded reverse progress and stable docking with the Dex3 dynamics.
4. Complete delivery, release and supported-placement verification, then repeat
   a predeclared fixed-configuration matrix across objects and layouts.
