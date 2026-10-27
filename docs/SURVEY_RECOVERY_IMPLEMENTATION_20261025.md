# Survey recovery implementation and validation — 25 October 2026

**The requested recovery, transition checks, safe-hold search, contact diagnostics
and layered validation are implemented. The crowd demo remains unqualified.**
Two survey diagnostics completed, but neither full three-person delivery passed;
both full attempts recorded falls before pickup. All outcomes are retained.

## 1. Bounded survey yielding and resumption

Basic discovery now catches `ViewpointDisplaced` through `survey_rotate()`.
It requests checked braking/avoidance, waits for a clear measured viewpoint,
reprojects any sensed viewing hint from the new pose and resumes surveying.
At most two recoveries share the original 45-second turn budget. Other failures
are not silently retried. View hints cannot substitute for confirmed detections.

Yield completion requires advancing fresh odometry, low measured velocity,
clear predicted person workspace, a fresh near-zero policy command and fresh
roll/pitch below 0.15 rad. In particular, a stationary robot still receiving an
escape command is not treated as safely stopped. Intent is revoked on exit.

## 2. Validate the complete turn/brake transition

Turns now recheck displacement and heading after checked braking. The 0.35 m
viewpoint reserve remains unchanged. A post-braking heading error above 0.15 rad
also requires viewpoint recovery unless a fresh discovery already ended the
turn. Every basic-survey turn requests measured settling first, rather than
assuming a previous brake or fixed observation pause is sufficient.

Tests cover recovery limits, original deadline preservation, hint reprojection,
advancing-state dwell, blocked workspace, nonzero commands despite stillness,
and displacement/heading changes during braking.

## 3. Search for an escape with a safe stopping endpoint

Survey/braking conflicts now invoke the bounded control-sequence search even
when the general navigation beam-search experiment is disabled. Refuge searches
target separation from the closest moving person's lane rather than the old
survey point. They use a 4.5-second minimum horizon, width 16 and a 20 ms maximum
search budget. The existing planner/supervisor rechecks remain active.

The search requires low terminal speed and yaw rate, then checks a further
two-second zero-command hold with swept pedestrian and static clearance.
Terminal collisions are rejected rather than merely assigned a cost. Future
pedestrian motion during that hold uses the final forecast velocity; this is a
prediction, not knowledge of future simulator trajectories.

The vectorized response also now matches the scalar native-cycle05 pivot-yaw
model. Sixty randomized profile/delay cases test shared rollout equivalence.

The [initial three-second replay](results/survey_refuge_replay_20261025.json)
retained five clear plans and rejected three endpoint-only plans that conflicted
during the hold. The [extended replay](results/survey_refuge_extended_replay_20261025.json)
found safe-hold plans in eight of ten retained sensed snapshots. These isolate
constant-velocity crowd forecasts, not full static geometry or physical success.
They are not historical-executable counterfactuals. Live failures below prevent
claiming that these model-level improvements solve crowd avoidance.

## 4. Actual native contact-force capture

`--contact-force-capture` enables control-state recording and the optional native
`UNITREE_MUJOCO_CONTACT_TRACE` sink. Every physics substep contributes to summed
normal-force peaks and impulse, reported in roughly 50 ms intervals, including
zero-contact intervals. Only robot/person contacts are included. Capture stops
after 35 simulator seconds and refuses to overwrite an existing trace.

The diagnostic does not feed evaluator or force data into mission control.
A new native CTest exercises actual colliding and separated robot/person geoms.
`analyze_contact_trace.py` matches device ticks to received policy commands and
rejects missing streams, frozen clocks and incomplete native/state coverage.

The [verified capture](results/survey_contact_capture_verified_20261025.json)
covers simulator time 0.002–33.350 s with maximum interval gap 0.052 s. It records
42 force-bearing windows, a **572.23 N peak summed normal force** and **167.22 N·s
summed normal impulse**. All 42 force windows associate with zero translational
policy commands. This establishes actual simulated contact during the stopped
command condition; it is stronger evidence than geometric overlap alone.
The receive-time association is not motor-application acknowledgement and does
not attribute every component of base movement solely to the external force.
The verified capture is the canonical record. A byte-identical initial-analysis
copy was removed during repository cleanup; its history remains in Git.

## 5. All seven live attempts

Run IDs below have suffix `_20261025`. All used native cycle05, red-to-red in
`transport_dual`, `payload_v1`, heading hold and 15 mm grasp offset. The targeted
capture had RViz enabled; the later validation trials disabled RViz. No run
disabled physical contact, used base pins or relaxed completion gates.

| Run prefix | Stage / people / seed | Outcome |
| --- | --- | --- |
| `survey_contact_capture` | Survey / 3 / 42 | Valid force capture; yielding timed out; 179 overlap samples; no fall |
| `survey_empty` | Survey / 0 / 42 | Diagnostic complete; no overlap or fall |
| `pickup_empty` | Pickup / 0 / 42 | 180-second deadline after arm-transition drift and repeated re-docking; no pickup, overlap or fall |
| `survey_crowd42` | Survey / 3 / 42 | Yield/reobserve/resume exercised; 184 overlap samples and a fall |
| `survey_crowd43` | Survey / 3 / 43 | Diagnostic complete; zero overlaps, no fall; minimum reported clearance 1.306 m |
| `delivery_crowd42` | Full delivery / 3 / 42 | 54 overlap samples; fall during survey recovery |
| `delivery_crowd43` | Full delivery / 3 / 43 | Survey completed, then 40 overlap samples and fall during pickup navigation |

[Portable validation evidence](results/survey_recovery_validation_20261025.json)
retains every attempt, source snapshot hashes, selected planner outcomes and
full logger-tail overlap statistics separately from mission-result statistics.
Overlap counts are samples, not unique contacts or force measurements.

Development proceeded between runs: the initial capture used the shorter search
horizon; the later tests used the extended search. The stronger near-zero-command
and body-tilt yield guard was added after `survey_crowd42`. Each run retains its
own source archive. This is development evidence, not a fixed-build acceptance
matrix or proof of improvement over the historical failure rate.

## Reproduction and checks

Use `--test-stage survey` or `--test-stage pickup` for a bounded diagnostic.
These explicitly return `mission_complete=false` and `success=false`; a completed
stage is recorded separately as `diagnostic_complete`. Delivery remains default.

```bash
ROS_DOMAIN_ID=92 python3 scripts/run_transport_demo.py \
  --world transport_dual --task red:red --people 3 --seed 43 \
  --duration 120 --fast-start --no-rviz --test-stage survey \
  --carry-posture payload_v1 --manipulation-heading-hold \
  --grasp-height-offset 0.015 \
  --policy vendor/mjlab/logs/payload_training/native_contact_cycle_05_20261012/policy.onnx
```

Add `--contact-force-capture` for targeted force/control evidence. The binary must
have been rebuilt after the native patch change. High-rate control traces require
more space; approximately 2.1 GB remained after these retained runs.

Final workspace CI passed **945 ROS results** (749 bridge tests), **238
orchestration tests**, and **13 native CTests**, including native force capture.
Source patches were exported and checked. No new policy was promoted.

The remaining priorities are earlier, physically executable avoidance before
walkers converge; evaluating search feasibility/timing with full live static
geometry; and the separate arm-transition balance/re-docking problem. The falls
are acceptance failures, despite the passing code-level regression checks.
