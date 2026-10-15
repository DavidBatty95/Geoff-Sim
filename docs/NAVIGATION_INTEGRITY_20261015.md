# Checked navigation goals and motion-bounded localization

## Outcome

The 15 October candidate completed a scripted red → red delivery and **2/4** tasks
in a subsequent unchanged-code, zero-person exploration matrix. All four matrix
attempts stayed upright with valid evaluator data; cross-colour tasks failed
placement verification and route admission. These are development results on one
layout and seed, not crowd acceptance or a controlled speedup.

The [portable evidence](results/performance_development_20261015.json) retains all
eight attempts, including two development falls, source identities, policy hashes
and the complete matrix.

## Checked path-goal transforms

In retained run `20261014T164700Z-readme-delay`, delivery navigation started about
1.1 m from staging. Each of three attempts logged **Transform data too old when
converting from map to odom**, immediately followed by **Reached the goal**.

Humble's `ControllerServer::isGoalReached` calls `nav_2d_utils::transformPose`
without checking its boolean return. Failed transformation leaves a default pose
at the origin. ROS initializes its quaternion to a valid identity, so checking
geometry validity alone does not solve the problem. A real-library regression
reproduces false arrival with the stock checker and a stale TF buffer.

Two local plugins now work together:

- **`ValidatedDWBLocalPlanner`** retains the actual path endpoint. Before normal
  DWB computation, it explicitly checks transformation to the local-costmap frame
  using the existing transform tolerance.
- **`ValidatedGoalChecker`** evaluates only that checked endpoint. Missing or
  invalid transformation evidence clears its arrival state; the unchecked pose
  argument from Humble's controller cannot become arrival evidence.
- TF failure uses Nav2's existing controller failure-tolerance path, which
  publishes zero commands and retries or fails the action.
- Position/yaw tolerances remain **0.45 m / 0.75 rad**. The mission's independent
  staging check, docking tolerances and placement verification remain active.

The plugins live in `unitree_social_costmap`; `nav2_livox.yaml` selects the pair.
Transport builds now explicitly include that package. Rebuild before using
`--fast-start` after this update. Custom configurations must keep the pair together;
the wrapper rejects an incompatible checker. Normal DWB and goal-tolerance
behavior delegates to the installed libraries; system Nav2 packages are unmodified.

References:

- [Humble controller implementation](https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/src/controller_server.cpp)
- [Humble transform helper](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/nav_2d_utils/src/tf_help.cpp)
- [Local controller/checker](../ros2_ws/src/unitree_social_costmap/src/validated_goal_checker.cpp)
- [Real-TF/plugin regressions](../ros2_ws/src/unitree_social_costmap/test/test_validated_goal_checker.cpp)

## Motion-bounded scan registration

The first cycle-05 follow-up carried the cube to delivery, but destination
geometry shifted with odometry during docking. Offline inspection found short
scan-to-scan corrections of roughly 13–19 cm over 40–90 ms, larger than nearby
evaluation motion. Evaluator truth was used only for diagnosis.

The estimator now gates translation using:

```text
translation limit = min(0.30 m, 0.03 m + 1.5 m/s × elapsed acquisition time)
```

- Time is since the **last accepted** scan, not the last received/rejected scan.
- Primary registration and full-band fallback share the bound.
- Rejected matches cannot overwrite accepted pose/reference state or renew
  odometry freshness. Diagnostics include proposed translation, limit and reason.
- The original 30 cm cap remains after longer gaps; this is not unrestricted
  reacquisition.
- Parameters: `registration_max_speed` (1.5 m/s) and
  `registration_translation_slack` (0.03 m). These are engineering reserves above
  current walking commands, not measured hardware limits.

This rejects abrupt corrections, not slow drift. It does not establish calibrated
covariance or decouple tracking from scan acceptance. More rejections can reduce
downstream sensing availability; held-out testing remains necessary.

## Retained native attempts

Raw artifacts are under the ignored
`ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/` directory.
Full run names, configuration and hashes are in the portable report.

| Attempt suffix | Configuration | Outcome |
| --- | --- | --- |
| `071200Z-checked-goal` | Cycle-03; checked goal; scripted red → red | Fell after repeated pickup docking corrections |
| `071600Z-checked-goal-contact` | Cycle-05; checked goal; scripted red → red | Carried to delivery; destination observation expired during reach refinement |
| `072300Z-motion-bounded-registration` | Cycle-05; both fixes; scripted red → red | Body instability during initial discovery |
| `072500Z-motion-bound-repeat` | Same candidate, scripted red → red | **Complete, 126.52 s**, including placement verification and restoration |

The matrix used the **exploration executive**, not the scripted survey executive
above. All four tasks used the same candidate, `transport_dual`, layout 0, seed 48,
zero people and 600-second mission budgets:

| Task | Complete | Fall | Outcome |
| --- | --- | --- | --- |
| Red → red | Yes | No | **181.58 s**, verified delivery and restoration |
| Red → blue | No | No | Post-release placement unverified; evaluator also rejected delivery |
| Blue → red | No | No | No observed-clear route after pickup/loaded motion |
| Blue → blue | Yes | No | **207.88 s**, verified delivery and restoration |

Matrix: `performance_matrix/20261015T072614Z/summary.json`. All four have valid
evaluator streams and no recorded dropped objects. Zero person-contact counts in
zero-person worlds are not crowd-safety evidence. The eight development attempts
are not one frozen cohort: policy and code changed before the matrix.

Reproduce after building:

```bash
python3 scripts/run_performance_matrix.py \
  --policy vendor/mjlab/logs/payload_training/native_contact_cycle_05_20261012/policy.onnx \
  --world transport_dual --tasks red:red red:blue blue:red blue:blue \
  --layouts 0 --seeds 48 --duration 600 --fast-start

python3 scripts/publish_performance_evidence.py --date 20261015 \
  --output docs/results/performance_development_20261015.json
```

Cycle-05 is a local artifact with SHA-256 recorded in the report. Cycle-06 remains
unpromoted.

## Verification and next priorities

Full `test_workspace_inside.sh ci` passed: **791 ROS results**, zero failures,
errors or skips, including **595 bridge tests** and nine new checked-goal C++
regressions; **142 orchestration tests**, **12 native CTests**, the deterministic
mock benchmark and MJLab environment check also passed. Pristine simulator patch
reconstruction passed separately.

Next priorities: cross-colour placement alignment/reobservation, observed-route
recovery with a held payload, docking oscillation, startup/turn stability and
held-out layouts/seeds. Complete deliveries demonstrate integration, not a
measured speedup or reliable general autonomy.
