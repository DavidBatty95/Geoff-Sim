# Fresh-check invalidation and forecast guard — 27 October 2026

Follow-up: the [heading decomposition](HEADING_INVALIDATION_20261027.md) now
isolates the first guarded rejection's coupled heading/yaw-rate effect and
quantifies the contribution from delayed-pose extrapolation.

**The invalidations have multiple causes; the experimental guard does not
establish reliable crowd avoidance.** The guarded seed-42 survey still fails
with 324 geometric overlap samples. Seed 43 completes without overlap or a fall,
with 13 primitive publications before task end. No delivery is qualified.

## Exact-input diagnosis

`--record-invalidation-audit` retains up to four previously admitted/currently
rejected input pairs. The offline analyzer swaps predicted/measured robot state
and old/new occupancy forecasts to isolate which changes alter admission.
It replays human occupancy and response checks, **not static geometry or motor
application**. Evaluator truth is used only for offline comparisons, with
timestamp-matched planar frame alignment; it is not an input to mission control.

In the [initial audit](results/primitive_invalidation_factorial_20261027.json),
the first invalidation occurs after 40.3 ms with only 0.52 mm robot XY residual.
The old forecast passes with either robot state; the new forecast fails with
either state. Its rejecting track shifts approximately 0.44 m over the forecast.
The [offline truth comparison](results/primitive_invalidation_truth_20261027.json)
finds the updated forecast more accurate. Smoothing away that update would hide
valid evidence. A second pair also exposes a larger projected robot-state
discrepancy and mismatched track/prediction epochs.

## Experimental correction

`--forecast-change-guard` requires `--strategic-avoidance` and is off by default.
The guarded planner:

- Rejects duplicate/regressing track and prediction epochs and uses typed
  predictions only when their epoch matches the track epoch within 1 microsecond;
  otherwise it uses fresh-track constant-velocity prediction.
- Adds a per-identity velocity-change reserve over a 0.5-second history, with a
  0.03 m/s deadband and 0.35 m cap. The reserve covers forecast time plus the
  two-second terminal hold in strategic and primitive checks. This is an
  engineering reserve, not a calibrated confidence bound.
- Requires odometry age at most 0.3 seconds at the beginning and end of long
  primitive admission.

Typed forecast extension now continues the final endpoint velocity and grows
covariance beyond the supplied horizon rather than jumping to an unrelated
constant-velocity path. Full-path, terminal-hold, deadline and supervisor checks
remain required. No policy or closure replacement was promoted.

## All three live attempts

[Retained validation report](results/forecast_guard_validation_20261027.json).
These are three-person, native-cycle05, red-to-red survey diagnostics; completion
is distinct from mission/delivery success. Counts below use the mission result
for overlaps and logger receipt times for publication scope.

| Run | Survey outcome | Overlap samples / fall | Primitive publications before / after task end |
| --- | --- | --- | --- |
| `invalidation_audit42_20261027` | Failed | 39 / yes | 3 / 0 |
| `forecast_guard42_20261027` | Braking timeout | 324 / no | 65 / 0 |
| `forecast_guard43_20261027` | Diagnostic complete | 0 / no | 13 / 8 |

Overlap samples are geometric envelope overlaps, not unique collisions or force
measurements. Different run durations prevent a naive comparison of raw counts.
Publications do not prove actuator application or causal survey benefit. Unlike
the earlier sustained seed-43 run, this guarded run does contain task-time
primitive publications; the earlier post-task-only finding remains valid for
that earlier run.

## Guarded replays

- [Seed 42, four captured pairs](results/primitive_invalidation_guarded_20261027.json):
  the first pair passes with predicted robot state under either forecast and fails
  with measured state under either forecast. XY residual is 10.8 mm and yaw
  residual 0.116 rad after 49.4 ms; this is a projected-state discrepancy, not proof
  of that much physical rotation. The second pair fails only with the newer
  forecast, whose rejecting track shifts up to 2.03 m and agrees better with
  offline truth. Its radius is unchanged. The last two recorded rejections are
  static-geometry rejections and are not reproduced by this occupancy-only replay.
- [Seed 43, two captured pairs](results/primitive_invalidation_guarded43_20261027.json):
  both pass with the old occupancy forecast and fail with the new one under either
  robot state. Rejecting-track forecast shifts reach 0.61 m and 0.45 m; radius
  changes reach 0.159 m and 0.006 m respectively. Thus these swaps include both
  center and radius changes, not isolated center-prediction effects. At the
  available offline truth offsets, the newer rejecting-track prediction is less
  accurate. Those short comparisons do not justify rejecting all fresh updates.

All six guarded pairs have matched track/prediction epochs. Coherence fixes one
identified input defect, but neither coherence nor the reserve prevents all
invalidations. The next targeted investigation should separate source odometry,
pose extrapolation and actual heading response in the first guarded seed-42
pair, and extend replay to captured static geometry before attributing its later
rejections. The present evidence does not identify a specific IMU or gait defect.

## Verification and provenance

Full workspace CI passes **1,010 ROS results**, including **814 bridge tests**,
**242 orchestration tests**, and **13 native CTests**. Tests cover coherent and
duplicate epochs, reserve history, endpoint continuity, aged-pose rejection and
synthetic forecast-versus-state factorial diagnostics. The deterministic mock
benchmark also passes; it is not a physical crowd validation.

Per-run source snapshots and effective parameters are retained with the raw
artifacts. Publisher-time hashes are not historical run provenance. The analyzer
reports its script hash and replay scope. Earlier reports and failed attempts
are retained. This development work remains local and uncommitted.
