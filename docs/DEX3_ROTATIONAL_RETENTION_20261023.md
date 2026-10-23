# Dex3 rotational retention follow-up — 23 October 2026

**Mobile delivery remains unqualified.** One 15 mm raised-grasp trial retained
the cylinder through travel, destination docking and lowering, but stopped when
the pad became unconfirmed. Its evaluator placement flag occurred while the hand
still held the cylinder; this is not a completed release or delivery.

## What the contact loss revealed

Offline reconstruction of the retained object-axis history showed approximately
20 degrees of cylinder tilt after pickup. The index finger was nearly fully
closed and unloaded, leaving the thumb and middle finger to carry the load.
The cylinder slipped down relative to the wrist before opposing contact was lost.
These evaluator measurements are diagnostic evidence, not mission-control inputs.

The clearance/retention tradeoff from the
[morning experiments](DEX3_TABLE_CLEARANCE_20261023.md) therefore remains active:
raising the tool clears the table but can produce a tilted, poorly supported grasp.

## Rotational bench and reference correction

The contact bench now supports actuated wrist yaw, pitch and roll, lateral motion,
closure waiting, lift retraction and measured slow opening. The rotational study
uses 40-second holds, 10 mm sway at 1.5 Hz and repeated 0–pi–0 yaw cycles with a
20-second period. Objects remain unconstrained and receive no external forces;
the prescribed wrist fixture is component evidence, not a mobile robot test.

An important reference correction separates the retained studies:

- Earlier pitch experiments rotated about the wrist origin, moving the tool
  point vertically by about 8 mm. Their successful holds cannot be interpreted
  as tests at a constant tool height.
- `--pitch-about-tool` compensates wrist translation by
  `tool_offset - R @ tool_offset`, preserving the commanded tool reference.
  The corrected regression checks actual yaw above 3 radians and measured tool
  height during the hold, as well as sampled pre-lift hand/table contact.

All initial successes and subsequent failures are retained. The original
12-case thumb-opposition/distal-thumb sweep and eight-case baseline/candidate
comparison did not establish a better closure. The corrected 12-case tool-pivot
sweep at the marginal 30 mm offset and nine wrist-roll cases failed. Current
side-specific closure targets have therefore not been replaced.

The corrected nominal 15 mm tool-offset case passed hold and upright release
with no recorded pre-lift table force. Smaller-cylinder/opposite-pitch conditions
retained the object during holding but failed upright release, including with
slow opening. Early lifting and backward lift retraction alone did not reproduce
all mobile failures. The bench is still not robust across these perturbations.
Some dropped objects in box-only fixtures fall without a ground plane; their
large negative lift values indicate failure, not a physical fall-distance claim.

## Controller changes and actual DDS checks

Closing now requires a fresh starting joint sample. Contact dwell cannot begin
until the native device tick has advanced through the commanded reference-ramp
interval: maximum joint travel divided by 1.5 rad/s, plus 0.1 seconds. Existing
freshness, opposing-contact, reset, 0.3-second dwell and six-second timeout checks
remain in force. Status includes joint positions and the closure-ready tick.

Actual ROS/DDS fixtures passed grasp and release on both hands after this change:
minimum held lift was **14.73 cm right** and **14.48 cm left**. These isolated
checks do not establish transport retention.

Placement now has bounded reobservation recovery when pad confirmation is revoked
after lowering: withdraw to carry while retaining the object, then require a
confirmed observation no older than two seconds and newer than the withdrawal
timestamp. Retries remain within the original 30-second/three-attempt budget.
Tests cover new observations, absent observations and rejection of cached
pre-withdrawal evidence. Successful execution of this recovery in a mobile
delivery remains unverified because the subsequent trials failed earlier.

## All four follow-up mobile attempts

IDs share prefix `dex3_contact_delivery_` and suffix `_20261023`. They use the
native cycle05 policy, `payload_v1` and experimental manipulation heading hold.

| Run middle | Height offset | Outcome |
| --- | ---: | --- |
| `closedramp` | 20 mm | Lost opposing contact after transport despite the longer closure interval |
| `turngrip15` | 15 mm | Retained through docking/lowering; pad confirmation revoked; still holding when evaluator placement flag became true |
| `turngrip15_reobserve` | 15 mm | Pickup alignment failure |
| `turngrip15_reobserve_repeat` | 15 mm | Contact lost before destination docking |

None completed delivery. No policy or new closure was promoted.

## Evidence, checks and next priority

The new [cumulative evidence snapshot](results/dex3_rotational_retention_20261023.json)
contains **15 bench files** (including multi-case sweeps), **two ROS/DDS checks**,
**four native stability probes**, **two ROS arm diagnostics**, and **seven mobile
attempts** for 23 October. It includes the morning results and their retained
instrumentation failure. Source hashes at the report root are publication-time
snapshots, not per-run provenance; mobile run directories retain source archives.

Latest workspace CI completed with **903 ROS results**, including **707 bridge
tests**, **223 orchestration tests**, and **12 native CTests**, with no failures.
The separate pinned-MuJoCo Python suite passed **16 tests**.

The next functional priority is a matched wrist/object-orientation fixture that
reproduces the tilted thumb/middle-only pinch, followed by supported release.
A candidate must beat the baseline across the retained perturbations before a
fixed-configuration mobile delivery matrix is attempted. Available disk space
was only about 3.4 GB at this reporting checkpoint; preserve existing evidence
when planning further high-rate captures.
