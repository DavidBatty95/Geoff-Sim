# Heading invalidation decomposition — 27 October 2026

**The first guarded seed-42 rejection involves an interaction between extrapolated
heading and yaw-rate state. Most of its apparent heading jump comes from pose
extrapolation, rather than physical rotation or a source-odometry heading jump.**
This completes the targeted diagnosis proposed in the
[forecast-invalidation report](FORECAST_INVALIDATION_20261027.md).

## Reproducible method

`scripts/analyze_invalidation_heading.py` matches each captured state to its exact
logged odometry source epoch, reconstructs the constant-body-twist pose advance,
and compares heading increments with offline evaluator yaw interpolated at the
same acquisition epochs. It also substitutes individual and combined robot-state
components into the response-aware occupancy check.

```bash
python3 scripts/analyze_invalidation_heading.py \
  ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/forecast_guard42_20261027 \
  --output /tmp/opencode/heading42_new.json
```

Choose a new output filename; existing evidence is not overwritten. Reports retain
trace and analyzer hashes. Evaluator truth is exclusively offline, never a control
input. Logged odometry is rounded; forecast epoch can differ slightly from the
pose-query epoch. Truth interpolation refuses extrapolation or gaps above 150 ms.
Static geometry and motor application are outside the replay scope.

## First guarded seed-42 pair

The pair spans **49.446 ms**. Source odometry ages are **153.120 ms** and
**137.954 ms**, both within the experimental 300 ms admission bound.

| Quantity | Heading increment (rad) |
| --- | ---: |
| Source odometry heading change between source samples | +0.018427 |
| Change in extrapolation increments | +0.095118 |
| Captured planner heading change | +0.113563 |
| Offline physical heading change between planning epochs | +0.017627 |
| Response-model heading change from the old captured state | −0.002474 |

Reconstruction agrees with the two captured headings within **0.000016 rad**;
XY residuals are below **0.06 mm**. The source-frame heading offset changes by
only **−0.000211 rad**. This is strong evidence against a source-heading jump as
the dominant explanation for this pair.

The yaw-rate sample changes from **−0.05899 to +0.62402 rad/s**. Holding each
sample constant across its odometry age changes the extrapolation correction
from **−0.00903 to +0.08609 rad**. Actual rotation over those age intervals is
approximately **+0.07072 and +0.06971 rad**. The first extrapolation underestimates
current heading; the next slightly overestimates it. The difference looks like
a much larger short-time rotation to the planner.

The model-versus-physical heading increment differs by **−0.02010 rad** as well.
Because the rollout assumes the admitted program and there is no replay of actual
motor application, this is not independently a policy-response calibration result.

## Which components flip admission?

Using the current occupancy forecast, substituting measured state into the
predicted state gives:

| Substitution | Occupancy check |
| --- | --- |
| XY only | Pass |
| Heading only | Pass; minimum margin 0.138 m |
| Linear velocity only | Pass |
| Yaw rate only | Pass; minimum margin 0.030 m |
| Heading and yaw rate together | **Reject** |
| Filtered command only | Pass |
| Entire measured state | **Reject** |

Conversely, replacing either heading or yaw rate in the measured state with its
predicted value restores a pass. These are diagnostic counterfactuals, not safe
commands or a recommendation to override sensed motion. They establish a coupled
heading/rate effect in this captured rejection, rather than a heading-only cause.

## Cross-checks and retained evidence

- [Verified seed-42 decomposition and component checks](results/invalidation_heading_verified42_20261027.json):
  all four pairs. The second remains forecast-driven after heading/rate replacement.
  The last two remain outside static-geometry replay scope.
- [Verified seed-43 decomposition and component checks](results/invalidation_heading_verified43_20261027.json):
  both pairs remain rejected after heading/rate replacement. The second has an
  apparent heading change of −0.08214 rad versus physical +0.01080 rad, showing
  another extrapolation discrepancy without making it the rejection's cause.
- [Initial decomposition](results/invalidation_heading42_20261027.json) and
  [individual-component development report](results/invalidation_heading_components42_20261027.json)
  are retained. The verified reports add combined and reverse substitutions.

## Correction direction and limits

The evidence supports developing **time-consistent current heading and yaw-rate
estimation** rather than extrapolating a roughly 150 ms old instantaneous angular
rate across the full pose age. It does not support simply smoothing away new
measurements or substituting predicted state whenever admission fails.

The inspected sensor path matters: `onboard_lidar_odometry.py` stores LowState IMU
samples at receipt time, selects a nearby sample for each acquisition-stamped
cloud, and puts that sample's gyro Z into odometry angular velocity.
`human_aware_motion_planner.py::_robot_pose()` then advances the delayed pose with
that twist. `/onboard/measured_rpy` also explicitly uses receipt time, since
LowState has a device tick rather than a synchronized ROS acquisition epoch.
Treating that topic as acquisition-synchronized would introduce a new timing error.

The next implementation should preserve the IMU device-tick/receipt provenance,
capture a synchronized heading/rate history, and compare time-aligned propagation
against this retained baseline before live promotion. A whole-body gyro Z sample
also should not be assumed to equal Euler yaw derivative under arbitrary tilt.
The current evidence identifies the extrapolation mechanism, not a defective IMU
or a specific gait oscillation frequency.

This session adds the offline diagnostic and tests; it does not introduce a new
runtime estimator or new live trial. Physical crowd avoidance and delivery remain
unqualified, and the previous live outcomes remain the applicable evidence.

## Verification

Four new tests cover wrapped interpolation, missing brackets/gaps, missing exact
source odometry, and separating rate-induced projection changes from physical
rotation, including absent truth. They are included in workspace CI.

Full CI passes **1,010 ROS results** (814 bridge tests), **246 orchestration
tests**, and **13 native CTests**. The deterministic mock benchmark passes.
Development remains local and uncommitted.
