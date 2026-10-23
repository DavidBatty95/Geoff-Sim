# Dex3 retention, retreat and placement follow-up — 21 October 2026

Later session: [measured opening, release diagnostics and four more trials](DEX3_RELEASE_DEVELOPMENT_20261021.md).

**Both hands now pass sustained contact-only holding tests. Mobile delivery is
still unqualified.** Five additional full-robot trials all achieved pickup;
three carried the cylinder to the destination and released it. None completed
verified placement. All five reported valid evaluator data and no robot fall.

## Left-hand retention

The mirrored right-hand closure failed all 27 cases of the left-hand long-hold
closure sweep. A subsequent 12-case thumb-opposition sweep produced three passes.
The selected left-hand command is `(0.05, 1, 1, -1.5, -1.5, -1.5, -1.5)` radians;
the right-hand command remains `(0, -1, -0.7, 1.5, 1.5, 1.5, 1.5)`.

The selected left closure then passed:

- **60-second hold**, 10 mm wrist sway at 1.5 Hz: minimum held lift **13.46 cm**,
  persistent opposed contacts and successful release.
- **40-second hold** with a 19 mm-radius cylinder and initial x/y shifted by
  -3/-2 mm: minimum held lift **13.46 cm**, successful release.

These are simulation calibration/bench results, not hardware qualification or a
complete robustness matrix. Native motor limits, contact requirements and the
empty-grasp rejection remain in effect. Bench defaults now use the same
side-specific closure targets as the ROS controller.

## Approach and retreat changes

- Lower to a pregrasp waypoint 80 mm behind the observed cylinder before
  advancing, rather than sweeping diagonally from carry height into the object.
- Require a fresh (at most two-second-old) observation after staging. Limit the
  final-reach estimate to five seconds instead of twenty. If it expires, withdraw
  from the current measured hand pose and reobserve, with at most two withdrawals
  inside the existing six-attempt budget.
- Use the existing reverse-speed envelope, capped at 0.45 m/s, through the
  positional stopping band. Returning to 0.30 m/s near the endpoint reproduced a
  stall, including after release with the arm still in its carry posture.
- Compensate measured forward coast after a failed retreat-settling check by
  commanding bounded extra clearance (at most 120 mm). The original settled
  clearance, timeout, collision checks and command slew limits remain unchanged.

## Placement work and retained failures

Lowering now uses the 90 mm cylinder's half-height plus a 5 mm nominal gap
(50 mm), replacing the old 75 mm cube offset. After release, the open hand
withdraws horizontally 100 mm before returning to carry height.

The latter change followed an initial hypothesis that diagonal retraction was
catching the cylinder. The next trial's evaluator-only trajectory shows tipping
**during finger opening, before withdrawal**. Retraction alone therefore does
not resolve placement. The actual held-object pose and release mechanics need
further measurement; a small kinematic alignment error is not sufficient evidence
that the cylinder is supported upright.

All runs below use prefix `dex3_contact_delivery_`, suffix `_20261021`, the
experimental 95% actor blend and `payload_v1` fixed carry posture.

| Run middle | Outcome |
| --- | --- |
| `staged` | Pickup, 1.03 m loaded retreat, destination docking and release; post-release retreat stalled; cylinder eventually fell off table |
| `supported` | 18.82 cm lift and retained grasp; retreat deadline before sufficient settled clearance |
| `reverseband` | 19.29 cm lift and retained grasp; repeated forward coast defeated retreat clearance |
| `retreatcoast` | 1.07 m loaded retreat, navigation, release and 0.49 m post-release retreat; cylinder tipped/rolled outside pad; no floor drop |
| `clearrelease` | 1.07 m loaded retreat, navigation, release and 0.47 m post-release retreat; cylinder tipped during opening and later rolled off table |

In `clearrelease`, offline evaluator samples put the cylinder centre near
`(-2.124, 0.011, 1.025)` immediately before opening, and near
`(-2.171, -0.054, 0.960)` about 1.17 seconds later. Horizontal arm withdrawal
started after the release acknowledgement, about 1.35 seconds after opening
began. This evidence contradicts attributing the entire failure to retraction.

## Evidence and next qualification step

[Cumulative retained evidence](results/dex3_contact_followup_20261021.json)
preserves all 14 mobile attempts from this date, all eight bench reports including
failed sweep conditions, and hashes of the new evaluator-only trajectory files.
The [earlier nine-trial report](DEX3_MOBILE_QUALIFICATION_20261021.md) remains an
unchanged historical evidence snapshot. No locomotion policy was promoted.

The next focused experiment should measure object orientation and support during
opening, reproduce tipping in a hand-level release fixture, and establish a
supported-release sequence before a fixed-configuration delivery matrix. Runtime
control must continue to use onboard observations, finger feedback and encoders;
the evaluator trajectories are solely for offline diagnosis.

## Verification

Final workspace CI passed **883 ROS test results** (687 bridge tests) and
**219 orchestration tests**. The pinned native Python suite passed **10 tests**,
including 40-second sway holding for both hands, short perturbation cases and
empty-grasp rejection. The new left-hand closure has not yet repeated the separate
native DDS service bench; historical service results remain labelled separately.
