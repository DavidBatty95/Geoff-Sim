# Perception and motion-response follow-up

Next implementation: [earlier overtaking avoidance and turn-transition diagnostics](EARLY_AVOIDANCE_FOLLOWUP.md).

2026-10-07 follow-up to the [failure review](DEMO_FAILURE_REVIEW.md).

**Implemented:** sensor-only wall-continuation rejection, bounded structural
memory, an explicit A/B switch, terminal onboard body-instability handling, and
translation/stop diagnostics for the native policy. **Still open:** persistent
false tracks, collision-free people-present delivery, and the 45 ms planner
timing target.

Verification: **484 bridge tests** and **22 targeted script tests** passed,
along with documentation, whitespace and shell-syntax checks. Native outcomes
below remain distinct from those software checks.

## Tracking changes

The active object-tracking branch does not call `_update_static_background()`.
Simply changing the static-background thresholds would therefore not address
that configuration. The new `wall_evidence.py` supplies independent structural
evidence instead:

- Keep upper LiDAR returns separately from the normal person-height-filtered
  point cloud. They never become person clusters themselves.
- Use transformed, finite, non-noise returns within the existing tracking range.
  The upper band is above `max_person_height + 0.15 m`, capped at
  `max_height + 1.5 m`.
- Retain at most eight scans, at most 0.25 seconds, in one frame, with the existing
  accumulated-point bound. Replayed scans cannot replace newer evidence.
- Fit up to 32 deterministic local line proposals. Require an extended thin
  vertical sheet: at least six inliers, 0.55 m horizontal extent and 0.4 m
  vertical extent. An overhead beam alone is insufficient.
- Reject structural person hypotheses, including wall-supported partial crops.
  Independent semantic person detections above the existing confirmation
  threshold take precedence. Stationarity by itself never identifies a wall.
- Run the structural check after the cheap person-size gate to avoid fitting
  large wall clusters that would already be rejected.

The parameter is `dynamic_obstacle_tracker.reject_wall_supported_clusters`.
The transport launcher exposes `--tracker-wall-evidence` (default) and
`--no-tracker-wall-evidence`, and records the selection in the generated YAML and
manifest. Ordinary static-obstacle sensing and clearance checking remain active.
This is a geometry-based rejection cue, not a general human recognition model.

## Native evidence

[All six outcomes](results/tracking_followup_20261007.json) and
[tracking/incident diagnostics](results/tracking_diagnostics_20261007.json)
retain the unsuccessful prototypes as well as the later pilots.

| Run (UTC, 2026-10-07) | People | Version / purpose | Outcome |
| --- | --- | --- | --- |
| `123254Z` | 0 | Initial narrow upper-band evidence | Pickup verified; later fall/drop; peak 6 false tracks |
| `124414Z` | 0 | Retain unclipped upper returns | Early fall; fitting before cheap rejection added unnecessary work |
| `125652Z` | 1 | Cheap-gate-first fitting and body-fault handling | Upright for 120 s; 9 overlap samples; deadline |
| `130654Z` | 0 | Final bounded-history wall evidence enabled | Upright for 90 s; peak 3 tracks, mean 0.65 |
| `130915Z` | 0 | Same-code wall evidence disabled | Upright for 90 s; peak 7 tracks, mean 2.44 |
| `131448Z` | 1 | Final wall evidence enabled | Upright for 120 s; 16 overlap samples; deadline |

The enabled/disabled pilots used the same world, seed, task, duration and code,
but their real-time trajectories differed. They support continued investigation
of reduced false-track load, not a statistically established improvement.
Neither empty-room pilot achieved zero false tracks. The enabled pilot's planner
p95 was **46.46 ms**, versus **29.65 ms** disabled, so there is no demonstrated
overall speed win from this comparison.

The last one-person pilot had planner p95 **51.06 ms**, a peak of six tracks and
mean reported track count **1.44**. The existing offline nearest-track diagnostic
matched all 115 samples within 2 m under its 0.5 m gate; this is not an identity
metric or visibility-conditioned recall. At the first geometric overlap, all
24 evaluated commands were still rejected for `hard_human_collision` and the
planner stopped. Earlier feasible escape remains an unresolved issue.

The two one-person pilots stayed upright longer than the preceding crowd
failures, but they used evolving code and different trajectories. They are not
unchanged-code successful-delivery repeats. None of these six trials completed
a delivery.

### Reproduce the short comparison

```bash
python3 scripts/run_exploration_transport_demo.py --people 0 --seed 48 \
  --task-seed 48 --duration 90 --tracker-wall-evidence \
  --policy vendor/mjlab/logs/payload_training/native_delay_cycle_03_20261006/policy.onnx \
  --fast-start
```

Repeat with `--no-tracker-wall-evidence`. For the one-person diagnostic, use
`--people 1 --duration 120`. These are bounded diagnostics, not delivery passes.

## Body-instability handling

`TransportMission` now timestamps `/onboard/measured_rpy` and recognizes sustained
roll or pitch above 0.7 rad for at least 0.2 seconds. Evidence must be fresh
(at most 0.3 seconds old), come from increasing acquisition stamps, and have no
inter-sample gap greater than 0.25 seconds.

The mission stops its intent and raises `BodyInstabilityError`. Managed action
cleanup still establishes termination, and exploration propagates the fault
instead of dispatching another viewpoint or retrying localization. Attachment
state is preserved, including uncertain possession. This uses onboard attitude,
not evaluator truth. Unit regressions verify transient/stale/replayed rejection
and navigation cleanup without retry. The later native pilots did not fall, so
they do not establish a measured native reduction in failure-reporting delay.

## Motion-response measurements

`probe_native_policy.py` now supports linear commands, finite command duration,
post-stop displacement measurement and `--require-mujoco-version`. The initial
environment used MuJoCo 3.8.1; deployment-matched probes were then run explicitly
with **3.3.6**. Both sets are retained in
[probe summaries](results/native_response_probes_20261007.json), with raw artifact
hashes. Raw samples are in `vendor/mjlab/logs/native_response_*_20261007.json`.

Each deployment-matched probe stood for 5 seconds, requested motion for 2 seconds,
then requested zero for 2 seconds. Policy application delay was two 20 ms periods.

| Command `(vx, vy)` m/s | Arm condition | Mean body `(vx, vy)` m/s after first 0.5 s | Maximum additional displacement after stop request |
| --- | --- | --- | --- |
| `(0.45, 0)` | Policy-controlled | `(0.511, -0.122)` | 0.191 m |
| `(0.37, -0.256)` | Policy-controlled | `(0.337, -0.236)` | 0.245 m |
| `(0, -0.45)` | Policy-controlled | `(-0.076, -0.368)` | 0.180 m |
| `(0.37, -0.256)` | Prescribed carry posture, **no attached load** | `(0.323, -0.276)` | 0.289 m |

All four probes remained upright. They use native geometry with synchronous
Python MuJoCo and step commands, without ROS scheduling, sensor processing or
the deployed command-filter chain. These are short response diagnostics, not
fitted steady-state gains, loaded transport acceptance or a stopping guarantee.
In particular, the diagonal command did produce substantial lateral motion:
the results do not support disabling all low-amplitude diagonal escapes.
Cross-axis motion and post-stop displacement merit further calibration.

```bash
bash scripts/run_mjlab_docker.sh --no-sync --with mujoco==3.3.6 python \
  /workspace/scripts/probe_native_policy.py \
  --policy /workspace/vendor/mjlab/logs/payload_training/native_delay_cycle_03_20261006/policy.onnx \
  --output /workspace/vendor/mjlab/logs/native_response_diagonal_336_20261007.json \
  --linear-x .37 --linear-y -.256 --command-duration 2 --duration 9 \
  --delay-steps 2 --require-mujoco-version 3.3.6
```

## Next acceptance gates

1. Eliminate persistent empty-room false tracks without losing stationary or
   near-wall people; measure the structural check's cost on fixed sensor replays.
2. Calibrate braking and cross-axis response through the deployed ROS command
   chain, with actual payloads, before changing rollout gains or clearance.
3. Isolate crossing/overtaking cases and intervene while a checked escape is
   still feasible. Preserve hard collision rejection and action ownership.
4. Re-run the complete medium task only after those isolated cases are reliable.

No collision threshold, pose freshness rule or placement acceptance threshold
was relaxed in this follow-up.
