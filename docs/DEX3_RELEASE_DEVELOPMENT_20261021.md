# Dex3 measured opening and release diagnostics — 21 October 2026

Later session: [Cartesian approach and moving-base limits](DEX3_CARTESIAN_APPROACH_20261021.md).

**Slow, measured opening passes both-hand physics and ROS/DDS rig checks.
Mobile delivery remains unqualified.** Four new mobile attempts all stopped
before destination release, so these runs do not establish whether the change
fixes the tipping observed in the [previous session](DEX3_RETENTION_AND_PLACEMENT_20261021.md).

## Release experiments

Three nine-case right-hand sweeps varied wrist release gap (0, 15 and 30 mm)
and finger opening rate (0.3, 0.75 and 1.5 rad/s):

1. Original plane support, centred cylinder.
2. Box support matching the transport tabletop dimensions/friction.
3. Box support with the cylinder initially offset laterally by 15 mm.

All 27 cases released upright in this rig. **They did not reproduce the mobile
tipping failure**, and therefore do not identify opening speed as its cause.
The new bench records object-axis upright cosine. Final bench acceptance now
also requires upright cosine above 0.98 and less than 10 mm position variation
over the final second. Existing lift, opposed-contact and release gates remain.

## Controller changes and integration checks

- Opening starts from fresh measured finger positions when available, then moves
  toward zero at **0.3 rad/s**. Missed timer catch-up is capped at 40 ms, avoiding
  a large opening jump after a scheduling delay.
- The existing six-second operation deadline, open-joint/contact-unloading
  thresholds and continuous 0.3-second confirmation dwell still apply.
- The native DDS rig now observes 18 seconds instead of 14, allowing the complete
  six-second release service window after its 11.5-second release request.
- Both-hand physics tests exercise measured-start opening onto a box support
  with a 30 mm release gap and require upright final placement.

| ROS → native DDS check | Outcome |
| --- | --- |
| Left hand, current side-specific closure | Grasp/release pass; minimum held lift 14.49 cm |
| Right hand, current closure | Grasp/release pass; minimum held lift 14.75 cm |
| Empty-grasp negative control | Grasp correctly times out; no lift; release succeeds |

These are complete mechanical wrist-rig tests, not mobile delivery qualification.

## All four new mobile attempts

IDs share prefix `dex3_contact_delivery_` and suffix `_20261021`.

| Run middle | Outcome |
| --- | --- |
| `slowopen` | 17.49 cm lift; object retained; retreat-clearance deadline |
| `slowopen_thread1` | Object disturbed during approach; redocking deadline |
| `slowopen_repeat` | Object fell during pregrasp staging; fresh observation unavailable |
| `fingertipclear` | Object fell during final approach; withdrew to reobserve, then stopped when observation expired |

All four reported valid evaluator data and no robot fall. None reached the new
release sequence. One run used `--numeric-threads 1`; all used the experimental
95% policy blend and `payload_v1` posture. Host load was elevated during this
session, but no causal attribution or deadline relaxation is made from that.

Checking open-hand geometry found the fingertips extend about **82 mm beyond
the palm tool reference**, while the previous staging waypoint was only 80 mm
behind the cylinder. Staging/occlusion withdrawal now uses 150 mm to include
finger reach, object radius and tracking clearance. The final attempt shows
this geometric correction alone does **not** establish reliable pickup.

Independent evaluator logs now include each object's local z-axis in world
coordinates (`object_axes`) alongside position history. These diagnostics are
saved after the run and never used by mission control.

## Evidence and verification

[Cumulative evidence](results/dex3_release_development_20261021.json) retains
18 mobile attempts from this date, 11 bench reports (including all sweep cases),
three new ROS/DDS checks and evaluator trajectory hashes. Historical reports
remain intact; source hashes describe publisher-time sources, not every run.

Final checks passed: **884 ROS results** (688 bridge tests), **219 orchestration
tests**, and **12 pinned-MuJoCo Python tests**. The first CI run exposed a test
fixture missing the new orientation-history field; the fixture was corrected
and the full CI rerun passed.

Next work needs to reproduce the mobile wrist/object orientation and motion
history in the release rig, and improve the final approach's swept clearance
and moving-base compensation. More upright stationary-wrist successes alone
will not qualify placement.
