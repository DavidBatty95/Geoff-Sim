# Dex3 tabletop clearance and control capture — 23 October 2026

**Tabletop interference explains a substantial part of the previously unexplained
backward motion.** A raised approach with measured sag compensation achieved two
pickups and more than 2.5 m object displacement in consecutive development trials.
Both stopped on lost opposing finger contact before placement; **mobile delivery
remains unqualified**.

## Isolating the ROS/native difference

The new `--arm-stability-probe` runs standing, carry, staging, extension and hold
through the real ROS arm compositor, without navigation or object interaction.
It explicitly reports `mission_kind=arm_stability_diagnostic`,
`mission_complete=false` and `success=false`, even when its diagnostic protocol
completes. It cannot be counted as a delivery.

Optional `--record-control-state` records actor targets, composed body targets,
observed LowCmd publications, LowState joint/IMU data, arm authority and Dex3
contact feedback. These are observed publications, **not motor-application
acknowledgements**. `analyze_arm_control_trace.py` checks stream coverage and
advancing native ticks, then summarizes each diagnostic phase offline.

The first recorder failed on NumPy float32 serialization. That run is retained
with `capture_valid=false`. After fixing and testing serialization with actual
ROS message types, the valid capture showed:

- **7.8 mm** evaluator-measured base translation during extension away from the
  table, broadly consistent with the native diagnostic's millimetre-scale motion.
- The expected **160/4 arm PD gains** in observed LowCmd packets.
- Native device ticks progressing with elapsed time.

This did not support a large command-gain discrepancy as the explanation for the
near-table retreat. Timing remains observable rather than assumed equivalent.

## Near-table physical counterfactual

The native probe now supports a declared near-table initial position and a
carry-start fixture. The carry-start comparison avoids the confound of raising
a hanging hand into the table. All fixtures remain free-base, with no equality
constraints or externally applied root/object forces.

| Requested tool z relative to pelvis | Table contact | Maximum base translation during extension |
| --- | --- | ---: |
| 0.20 m | Wrist/fingers, up to **13.5 N** | **42.9 mm** |
| 0.22 m | Residual wrist/finger contact | **9.5 mm** |
| 0.24 m | No recorded non-floor contact | **4.9 mm** |

The lower pose also maintained wrist-table contact during the hold. These
experiments provide physical evidence for tabletop interference, rather than
attributing every retreat to the locomotion policy or ROS timing. Raising the
target changes posture as well as clearance, so this is not a complete causal
decomposition of all body motion.

## Clearance versus grasp retention

A separate box-support hand bench tested tool-height offsets above the observed
cylinder centre:

- **20 mm:** passed 40-second hold with 10 mm wrist sway, minimum held lift
  **15.13 cm**, followed by upright stable release.
- **30 and 40 mm:** failed sustained grasp/release; both failures are retained.

Thus the largest collision-free native-probe height cannot simply be adopted as
a grasp setting. `--grasp-height-offset` exposes the experiment explicitly and
retains a zero default. The object dimensions and masses are unchanged.

The reach controller now estimates a bounded tracking correction at the clear
staging waypoint, before advancing over the table. This compensates observed arm
sag without adding simulator truth. The 12 mm alignment tolerance, opposing
contact checks, correction bound and mission deadlines remain unchanged. The
reported object centre remains the measured centre; the tool offset is stored
in the existing hand/object transform used for placement.

## All mobile attempts

IDs share prefix `dex3_contact_delivery_` and suffix `_20261023`. All use native
cycle05, `payload_v1`, heading hold and a 20 mm grasp-height offset.

| Run middle | Outcome |
| --- | --- |
| `raised20` | Alignment failure; no pickup |
| `raised20_sag` | 3.3 mm alignment, **13.38 cm lift**, **1.16 m loaded retreat**, maximum object displacement **2.66 m**; contact lost during destination inspection |
| `raised20_repeat` | 7.2 mm alignment, **13.35 cm lift**, **1.29 m loaded retreat**, maximum object displacement **2.57 m**; contact lost during destination inspection |

No robot fall was recorded. The carry attempts stopped with uncertain payload
state; neither is a placement success. In the repeat, feedback remained fresh
while thumb force fell from roughly 39 N to zero and the middle finger remained
loaded. This confirms loss of opposed contact, not merely stale packets. These
are simulated force proxies, not calibrated hardware tactile measurements.

## Evidence and checks

[Retained evidence](results/dex3_table_clearance_20261023.json) includes four
near-table native probes, three height-offset bench results, both ROS diagnostic
attempts (including the recorder failure), and all three mobile trials. Full
control traces and evaluator histories remain in their run directories. Capture
hashes identify a recorded byte prefix when analysis ran while logging was live;
the later verification report hashes the closed trace.

Final checks: **899 ROS results** (703 bridge tests), **223 orchestration tests**,
and **15 pinned-MuJoCo Python tests** passed. The new capture tests reject missing
command streams, long receipt gaps and frozen device ticks. The native bench
regression includes the 20 mm raised grasp with slow measured opening.

Next priorities are a grasp geometry/orientation that both clears the table and
retains opposed contact through turning, followed by supported release and a
fixed-configuration delivery matrix. The higher-offset failures prevent claiming
that increasing height alone solves the task.

The [rotational-retention follow-up](DEX3_ROTATIONAL_RETENTION_20261023.md)
records the subsequent reference-corrected bench studies, native-tick closure
guard and four further mobile attempts, with a separate cumulative snapshot.
