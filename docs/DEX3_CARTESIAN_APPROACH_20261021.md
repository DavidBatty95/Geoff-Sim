# Dex3 Cartesian approach and moving-base limits — 21 October 2026

Follow-up: [22 October isolated stability and heading-feedback experiments](DEX3_ARM_STABILITY_20261022.md).

**Eight new mobile attempts completed no pickup or delivery.** One candidate-policy
trial fell during navigation, before arm reaching. Every attempt and that fall
are retained. The changes below fix specific planning/perception defects; they
do not establish improved end-to-end reliability.

## Findings and changes

- Reconstructing a recorded staging trajectory from its endpoint joint angles
  showed about **40 mm tool-path bow** relative to a straight Cartesian segment.
  A clear endpoint does not imply clearance during a joint-linear trajectory.
- Dex3 Cartesian requests now produce waypoints spaced at most 25 mm apart,
  continue local IK branches, and check intermediate joint interpolation against
  an 8 mm planned tool-path corridor. Large joint jumps or infeasible segments
  fail before dispatch. The event explicitly labels this as a **planned
  pelvis-frame path**, not a measured-world collision-clearance guarantee.
- Small Cartesian corrections no longer incur a two-second minimum duration;
  segment timing retains the **0.35 rad/s joint limit**, with a 0.4-second minimum
  total duration and shorter post-action observation delay.
- Reach correction now integrates the current observed world-relative residual,
  including base movement during the trajectory, within the existing 80 mm bound.
- Pickup recovery withdraws the open hand and restores carry posture before
  moving the base. Heading drift beyond 0.15 rad triggers this recovery; at most
  two redocks are allowed inside the existing six-attempt reach budget.
- A retained perception frame contained a red component only **1 mm high** that
  was classified as the 90 mm cylinder. Dex3 cylinder candidates now require
  more than 60 mm visible height; clipped caps/short fragments cannot replace a
  complete-object estimate. A synthetic occlusion regression exercises this.
- A docking trace acknowledged approximately **0.254 m/s mean forward command**
  over its last 20 seconds while sensed displacement was only **16 mm**. Forward
  docking now uses up to 0.45 m/s within the configured forward limit, as reverse
  docking already did. The final trial passed docking but failed reaching.

An independent model check found the URDF and native right-hand tool positions
agreed within **0.57 mm** for a representative nonzero-waist posture. New native
regressions check both sides within 1 mm. A large static wrist-frame mismatch
therefore does not explain the several-centimetre mobile alignment errors in
that checked configuration.

## Retained mobile outcomes

IDs share prefix `dex3_contact_delivery_` and suffix `_20261021`.

| Run middle | Policy | Outcome |
| --- | --- | --- |
| `cartesian` | Experimental 95% blend | Pickup docking deadline |
| `cartesian_cycle05` | Native cycle05 | Alignment failure; base yaw drifted about 0.9 rad during correction sequence |
| `cartesian_repeat` | Experimental 95% blend | Navigation fall; measured-instability stop and cancellation |
| `shortservo` | Native cycle05 | Object displaced during recovery; observation expired |
| `guardedredock` | Native cycle05 | Object fell during approach; withdrawal/reobservation stopped on expired evidence |
| `fullcylinder` | Experimental 95% blend | Object movement 7.7 mm; final hand error 17.8 mm exceeded unchanged 12 mm gate |
| `worldservo` | Experimental 95% blend | Pickup docking deadline; command/progress mismatch retained |
| `forward45` | Experimental 95% blend | Docking passed; reaching exceeded 80 mm correction bound as the base moved backward; object essentially stationary |

All eight reported valid evaluator data. Configurations evolved between trials,
so these results are not a fixed-policy reliability matrix. No policy was
promoted and no grasp, placement, fall or freshness acceptance threshold was
relaxed. Release changes from the preceding session remain untested in a complete
mobile placement sequence.

## Verification and next priority

Final workspace CI: **886 ROS test results** (690 bridge tests), **219 orchestration
tests**, zero failures. Pinned native Python suite: **14 passed**, including
both-hand kinematic parity, sustained grasping and upright slow release.

[Cumulative evidence](results/dex3_cartesian_development_20261021.json) retains
all 26 mobile attempts from this date alongside prior bench/service results and
trajectory hashes. Report-level source hashes are publication-time snapshots;
individual mobile runs retain their own source snapshots.

The principal remaining integration issue is whole-body stabilization with the
Dex3 mass and extended arm: the free base can yaw or translate away faster than
discrete reach corrections converge. The next experiment should measure that
response in a repeatable arm-extension protocol, then evaluate coordinated base
and arm control before spending another broad set of full delivery retries.
The checked nominal tool path is not a substitute for that control or for full
robot/object swept-volume collision checking.
