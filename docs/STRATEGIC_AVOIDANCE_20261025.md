# Strategic multi-person avoidance — 25 October 2026

**An opt-in strategic layer is implemented and tested. It improves forecast-based
refuge selection, but it has not solved the difficult live crowd encounter.**
Enable it with `--strategic-avoidance`; it is disabled by default.

The [26 October follow-up](ESCAPE_PRIMITIVES_20261026.md) implements reusable
motion programs and process-isolated search with fresh-state revalidation,
retaining every intermediate attempt and remaining admission failure.

## What was missing

The previous refuge target was perpendicular to the closest moving person's
walking lane. With converging walkers, this can send the robot away from one
person and toward another. The planner has an enclosure cost, but its default
weight is zero, and a short-horizon nominal turn can take priority over planning
an earlier escape.

## Implemented strategy

1. **Predict enclosure before short-horizon collision.** Forecast the sensed crowd
   and nominal robot route over seven seconds. Flag pairs that become close,
   opposed and simultaneously closer to the robot. This is a geometric pincer
   hypothesis, not an inference that people intentionally coordinate.
2. **Compare all-person refuges.** Consider a ring of candidate points, the current
   position, previous refuge and both perpendicular escape choices for each
   moving person. Rank each coarse route against every person, including its
   stopping interval, rather than maximizing distance from the nearest person.
3. **Respect observed obstacles.** Reject coarse paths blocked by the available
   static-distance field. The strategic route is only a hint; the existing
   response-aware sequence search and supervisor still determine executability.
4. **Retain a clear choice.** Keep the previous refuge when its predicted route
   remains clear and within 12 cm of the best clearance score. Reevaluate it each
   tick and discard blocked choices. This reduces direction flipping while the
   pincer condition remains active.
5. **Trigger planning early.** An active pincer bypasses short-horizon precision
   priority, requests the bounded sequence search and enables its enclosure cost.
   Manipulation mode is excluded from this new strategic activation path.

Forecasts use tracked positions and velocities. They do not read simulator truth
or future pedestrian script paths. The strategic approximation assumes straight
travel at 0.35 m/s after 0.3 seconds and includes a two-second hold; actual
execution must pass the existing dynamics-aware checks. A negative coarse margin
remains negative and cannot authorize motion. If no candidate passes execution
checks, the existing checked fallback behavior remains in control.

Planner diagnostics expose predicted time to enclosure, affected track pairs and
the three highest-ranked refuges, including minimum/terminal margins and whether
the previous choice was retained.

## Retained scenario comparison

The [final candidate comparison](results/strategic_avoidance_candidates_20261025.json)
uses six sensed snapshots from each of the prior four- and three-person failures.
Eight snapshots have a moving-person baseline to compare. All-person selection
improves the forecast minimum margin in **six**, matches it in **two**, and is
worse in **zero**. This comparison has no static-map replay and is not a physical
or historical-executable counterfactual.

Examples:

| Snapshot | Closest-person hint margin | All-person hint margin |
| --- | ---: | ---: |
| Three-person run, first translation onset | −0.045 m | +0.386 m |
| Four-person run, one second after translation onset | −0.606 m | +0.037 m |
| Three-person run, 0.5 s after onset | −0.922 m | −0.164 m |

The last example is an important limit: a better direction can still have no
clear straight-line escape. Two late snapshots remain negative. At the first
translation onset, the predictor identifies enclosure 3.0 s ahead in the
three-person trace and 4.2 s ahead in the four-person trace. The retained samples
do **not** establish detection earlier than the historical avoidance onset.

The [initial ring-only comparison](results/strategic_avoidance_replay_20261025.json)
is also retained. It exposed a small regression in one snapshot; retaining the
original per-person escape candidates removed that regression in the declared
comparison.

## Live validation

[Both live attempts](results/strategic_avoidance_live_20261025.json) use three
people, native cycle05, the same red-to-red survey, 120-second budget and no RViz.
IDs have suffix `_20261025`.

| Run | Outcome |
| --- | --- |
| `strategic_survey42` | Yield timeout; 163 geometric overlap samples; no fall; valid data |
| `strategic_survey43` | Survey diagnostic completed; no overlap or fall; minimum reported clearance 1.064 m |

In seed 42, pincer prediction activated on **42 planner ticks** and found a clear
coarse refuge on **eight**. It produced **zero complete executable sequence
proposals**. The measured strategic/kinodynamic preparation-and-search phase
averaged **49.3 ms**, peaking at **73.1 ms**. This phase includes work outside the
inner 20 ms beam-search budget. These measurements identify a real-time planning
and feasibility bottleneck; the current status does not distinguish every
timeout from geometric infeasibility.

In seed 43, the pincer condition never activated. Its successful survey therefore
does **not** demonstrate a benefit from the new strategy. Neither run is delivery
qualification. Full logger-tail overlap statistics remain separate from mission
evaluation in the published evidence.

## Tests and next work

Ten geometric tests cover opposed approaches, parallel/diverging motion, distant
walkers, third-person blockage, no-clear-refuge cases, static barriers, retained
direction and malformed forecasts. Two planner integration cases verify that
the experiment can preempt short-horizon precision priority while the disabled
configuration retains its previous behavior.

Focused planner/search tests: **207 passed**. Complete workspace CI: **957 ROS
results** (761 bridge tests), **238 orchestration tests**, **13 native CTests**.

The next step is making the strategic plan executable within timing constraints:
distinguish deadline exhaustion from infeasibility, prepare reusable bounded
escape primitives or asynchronous plans, and revalidate each returned plan
against current tracks, static geometry and braking state. Multi-person velocity
uncertainty and changes of direction also need explicit treatment. The present
constant-velocity forecast is useful early-warning evidence, not a guarantee.

## Run the experiment

```bash
ROS_DOMAIN_ID=92 python3 scripts/run_transport_demo.py \
  --world transport_dual --task red:red --people 3 --seed 42 \
  --duration 120 --fast-start --no-rviz --test-stage survey \
  --strategic-avoidance --carry-posture payload_v1 \
  --manipulation-heading-hold --grasp-height-offset 0.015 \
  --policy vendor/mjlab/logs/payload_training/native_contact_cycle_05_20261012/policy.onnx
```

Implementation: `strategic_avoidance.py` and `human_aware_motion_planner.py` in
`ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/`.
Offline reproduction: `scripts/replay_strategic_avoidance.py`; publishers refuse
existing output names so earlier evidence remains intact.
