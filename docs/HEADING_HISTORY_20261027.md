# Publication-stamped attitude propagation — 27 October 2026

The opt-in `--heading-history` experiment replaces long angular extrapolation
from delayed odometry with recent onboard attitude increments. It requires
`--forecast-change-guard` and `--strategic-avoidance`. It remains off by default.

## Timestamp contract and implementation

The existing Humble DDS metadata executor supplies LowState **source publication
timestamps**, explicitly not sensor acquisition timestamps. History retains source
time, receipt time and advancing device ticks. This removes callback queue delay
from sample placement while leaving sensor-to-publication latency uncalibrated.
The implementation does not relabel `/onboard/measured_rpy` receipt time as acquisition.

`heading_history.py` brackets the odometry source epoch in that attitude history,
applies the measured yaw increment to the odometry heading, and extrapolates only
the latest sample's short age. Current Euler yaw rate is computed from body gyro
Y/Z and measured roll/pitch. The planner uses that rate for response prediction.
Existing XY projection retains the original odometry twist separately.

Samples must have advancing source timestamps and device ticks, finite values,
publication-to-callback delay at most 50 ms, and roll/pitch at most 0.65 rad.
Propagation requires a bracketed source epoch, no history gap above 50 ms and a
latest sample at most 50 ms old. Missing data returns no planner pose, without a
TF fallback bypass. Audit/debug records expose the clock, brackets, latest sample,
query epoch, increment, rate and rejection counts. Simulator truth is offline only.

## Live validation

Both three-person survey attempts use the same world, seeds, policy, posture and
120-second cap as the prior guarded runs. See the
[complete stage report](results/heading_history_validation_20261027.json).

| Run | Diagnostic | Fall | Geometric overlap samples |
| --- | --- | --- | ---: |
| `heading_history42_20261027` | Survey complete | No | 115 |
| `heading_history43_20261027` | Survey complete | No | 77 |

These are survey diagnostics, not safe-delivery qualification. Different durations
and closed-loop trajectories prevent comparing raw overlap counts as a reliability
improvement. No new policy was promoted.

`scripts/assess_heading_history.py` compares each valid recorded increment with
offline acquisition-stamped evaluator heading and the old constant-rate projection
on the **same samples**. The [accuracy report](results/heading_history_accuracy_20261027.json)
covers the entire logger interval, including teardown, rather than task-only data.

| Seed | Matched samples | History mean / p95 absolute error (rad) | Old projection mean / p95 (rad) |
| --- | ---: | --- | --- |
| 42 | 852 | 0.00132 / 0.00624 | 0.01022 / 0.04507 |
| 43 | 772 | 0.00238 / 0.01031 | 0.02604 / 0.08844 |

This supports improved heading accuracy when history is available, not improved
crowd safety. Availability is a substantial limitation: seed 42 records 869
complete estimates, 751 gap refusals and 262 stale-attitude states; seed 43 records
800 complete estimates, 293 gap refusals and 81 stale-attitude states. Each also
contains startup/stale-odometry states. Thus valid-only accuracy is selection-
conditioned and cannot establish continuous control quality.

## Verification and disposition

Five history unit tests and one planner integration regression cover time brackets,
wraparound, gyro-to-Euler conversion, advancing ticks, stale/late/missing metadata,
gaps, invalid samples, current-rate use and fail-closed behavior. Full CI passes
**1,016 ROS results (820 bridge tests), 246 orchestration tests and 13 native CTests**.
The estimator is retained as an experimental research option with its complete
evidence. Source snapshots and effective parameters accompany both raw runs.
