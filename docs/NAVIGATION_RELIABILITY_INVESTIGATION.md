# Navigation reliability investigation — 24 September 2026

<!-- geoff-nav:start -->
<p align="center">
  <a href="../README.md"><img src="media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../README.md#quick-start"><img src="media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="DEMO_GUIDE.md"><img src="media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="README.md"><img src="media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="REPOSITORY_GUIDE.md"><img src="media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="PROJECT_STATUS.md"><img src="media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="PROJECT_PLAN.md"><img src="media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="RESULTS_REVIEW.md"><img src="media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="literature_review/README.md"><img src="media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="DEVELOPMENT.md"><img src="media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

This is a dated development report. The repository includes the subsequent
implementation and reviewed source patches. For subsequent transport
results, see [payload training](PAYLOAD_POLICY_TRAINING.md); the transport-gate
statement at the end records the state on this report's date.

For later complete matrices, publication history and the active forward plan,
see [project status](PROJECT_STATUS.md) and [the project plan](PROJECT_PLAN.md).

## Acceptance status

**Navigation acceptance has not been achieved. Locomanipulation is not qualified.**

The latest consolidated working configuration is recorded in
[v82](results/avoidance_v82_matrix.json): **14/18 strict passes, 49/54 goals,
four contacts, no invalid trials**. The best complete matrices observed during
this investigation reached **16/18 strict passes**. None was an all-pass release
matrix, and held-out-seed acceptance has not been demonstrated.

| Latest v82 world | Strict passes | Goals | Contacts |
| --- | --- | --- | --- |
| Courtyard | 1/3 | 7/9 | 2 |
| Café | 2/3 | 7/9 | 1 |
| Station | 3/3 | 9/9 | 0 |
| Clinic | 3/3 | 9/9 | 0 |
| Concourse | 2/3 | 8/9 | 1 |
| Office | 3/3 | 9/9 | 0 |

Complete standard matrices used four independently controlled actors, three requested
goals per trial, seeds 42/43/44, and a 90-second deadline. Contact thresholds,
the 0.50 m goal tolerance, 0.90 m/s measured-speed limit, minimum score of 70,
and onboard-source restrictions were retained. Failures remain in the datasets.
Targeted subsets are identified by their smaller record counts. The startup-gated
v61 diagnostic is a separate condition, not a substitute pass.

Observed-map admission can select different goals across repetitions of the same
seed. Startup timing and asynchronous sensing/control also vary. Consequently,
isolated pass-rate changes do not establish causation or reproducible reliability.
The six-case v48 pass did **not** generalize to the subsequent complete matrix.

## Retained corrections

- Recovery considers future conflict timing rather than preferring a retreat
  that a faster person can catch. A later wall conflict does not automatically
  outrank an earlier person collision.
- A short recovery is evaluated as the single command it actually publishes;
  it cannot claim safety from an unexecuted later phase.
- Existing clearance-margin overlaps can be exited with checked non-worsening
  motion. Static recovery must increase clearance and cannot cross physical body
  geometry. Ambiguous conservative-grid distances receive exact swept checks.
- The final supervisor preserves an independently checked planner recovery before
  searching alternatives. Alternatives require fresh scan data, static clearance
  and time-aligned people checks.
- Timed commands carry a short-lived sequence matched to the current command,
  robot frame, goal and active mission. Stale/mismatched sequences are discarded.
- Goal identity uses the requested pose/frame, not changing `map` → `odom`
  coordinates. SLAM corrections and same-goal timestamp refreshes no longer
  restart a committed escape. Short-goal travel mode survives avoidance detours.
- A zero forward-turning-arc floor no longer silently removes reverse motion
  when yaw is requested. Explicit positive arc floors still apply.
- Partial person observations are shape-gated, cannot reuse the same crop points
  for multiple identities, and cannot indefinitely refresh a track without full
  person evidence. Weak observations have higher measurement noise and do not
  reinforce confidence merely because the inferred track moves.
- A standing-person birth-height gate rejects low furniture. This is configured
  for the standing actors in these worlds; broader person-type coverage has not
  been established.
- LiDAR registration prefers upper structural returns when sufficiently
  observable, with a full-band fallback. Point-to-plane updates reduce tangential
  resampling bias, retaining a point-to-point fallback for degenerate geometry.
- The runtime graph audit now requires odometry and applied-command feedback
  subscriptions on both motion-planning and safety nodes.

### An integration regression was retained

During v66, two supervisor feedback subscriptions were accidentally moved out of
the constructor. The supervisor continuously stopped for stale odometry; all
18 trials failed and reported contacts. The cohort is retained unchanged.
A constructor regression test reproduces the fault, and the strengthened graph
audit rejects that wiring before future scored missions start.

## Consolidated operating configuration

The defaults retain the lower-cost 2,400-ray / 15 Hz sensing configuration.
Translational command ramps are 2.40 m/s² in the onboard demo catalog, with a
0.65 m/s combined command cap. This command cap is not a certificate of actual
peak body speed. Person/static geometry remains checked independently.

The working profile uses a 3-second forecast, a 0.25 m prediction buffer and
0.40 m preferred-clearance margin in addition to body radii. Goal urgency can
increase after sustained stagnation, but does not bypass collision rejection.
The response model distinguishes positive/negative lateral gains and the
measured difference between walking turns and stationary pivots.

The following experiments remain available in code but are **disabled by default**:
multi-step kinodynamic search, translation refinement, goal-region capture,
timed-wait proposals, nonzero goal-cost radius, extra covariance-based body
inflation, heading alignment while translating, torso-only centroids, selection
hysteresis, and enclosure cost. Their numerical/regression tests do not establish
live navigation acceptance.

## Calibration corrections and evidence

The fixed-command measurement tool previously derived velocity from receipt times
while processing duplicate acquisition stamps. It also included pre-stop motion
in stopping drift when the latest received pose predated the stop request.
It now uses unique odometry source stamps and interpolated transition poses.
Older measurements retain their original values and should not be treated as
equivalent to the corrected time basis.

Corrected six-second, single-axis tests with 2.40 m/s² command ramps reported:

| Command | Steady response magnitude | Onset | Stop drift | Validation |
| --- | --- | --- | --- | --- |
| Lateral +0.45 m/s | 0.357 m/s | 0.312 s | 0.109 m | Pass |
| Lateral −0.45 m/s | 0.402 m/s | 0.300 s | 0.152 m | Pass |
| Forward +0.55 m/s | 0.528 m/s | 0.248 s | 0.206 m | **Fail:** exceeds 0.20 m stopping-drift limit |

The failed forward result was not relaxed into a pass. Higher reverse speed was
also tested and not adopted as a default. Mixed translation/yaw measurements are
explicit diagnostics; both the wrapper and validator prevent their being claimed
as single-axis acceptance trials.

## Complete iteration ledger

Counts below include every scored record. Reported contacts on invalid attempts
would also be retained. Dataset files contain commands, source/policy/image hashes
and result provenance. Intermediate working-tree variants were not committed as
individual source revisions; hashes alone do not reconstruct those variants.
The current configuration can be rerun from the current working tree.

| Dataset | Strict passes | Goals | Reported contacts | Invalid | Startup |
| --- | --- | --- | --- | --- | --- |
| [v38](results/avoidance_v38_validation.json) | 6/8 | 21/24 | 2 | 0 | standard |
| [v39](results/avoidance_v39_validation.json) | 6/8 | 22/24 | 2 | 0 | standard |
| [v40](results/avoidance_v40_matrix.json) | 13/18 | 49/54 | 4 | 0 | standard |
| [v41](results/avoidance_v41_validation.json) | 3/8 | 18/24 | 5 | 0 | standard |
| [v42](results/avoidance_v42_matrix.json) | 12/18 | 46/54 | 5 | 0 | standard |
| [v43](results/avoidance_v43_validation.json) | 6/8 | 22/24 | 2 | 0 | standard |
| [v44](results/avoidance_v44_matrix.json) | 15/18 | 50/54 | 3 | 0 | standard |
| [v45](results/avoidance_v45_validation.json) | 3/6 | 14/18 | 3 | 0 | standard |
| [v46](results/avoidance_v46_validation.json) | 3/6 | 13/18 | 2 | 0 | standard |
| [v47](results/avoidance_v47_validation.json) | 4/6 | 14/18 | 2 | 0 | standard |
| [v48](results/avoidance_v48_validation.json) | 6/6 | 18/18 | 0 | 0 | standard |
| [v49](results/avoidance_v49_matrix.json) | 13/18 | 48/54 | 5 | 0 | standard |
| [v50](results/avoidance_v50_matrix.json) | 16/18 | 52/54 | 2 | 0 | standard |
| [v51](results/avoidance_v51_matrix.json) | 12/18 | 48/54 | 5 | 0 | standard |
| [v52](results/avoidance_v52_matrix.json) | 16/18 | 52/54 | 1 | 0 | standard |
| [v53](results/avoidance_v53_matrix.json) | 12/18 | 48/54 | 5 | 0 | standard |
| [v54](results/avoidance_v54_matrix.json) | 13/18 | 48/54 | 5 | 0 | standard |
| [v55](results/avoidance_v55_validation.json) | 11/12 | 35/36 | 1 | 0 | standard |
| [v56](results/avoidance_v56_matrix.json) | 15/18 | 48/54 | 3 | 0 | standard |
| [v57](results/avoidance_v57_matrix.json) | 14/18 | 49/54 | 3 | 0 | standard |
| [v58](results/avoidance_v58_validation.json) | 10/12 | 35/36 | 2 | 0 | standard |
| [v59](results/avoidance_v59_matrix.json) | 14/18 | 50/54 | 3 | 0 | standard |
| [v60](results/avoidance_v60_matrix.json) | 13/18 | 49/54 | 4 | 0 | standard |
| [v61 diagnostic](results/avoidance_sync_v61_diagnostic.json) | 7/12 | 26/36 | 5 | 0 | gated |
| [v62](results/avoidance_v62_matrix.json) | 14/18 | 48/54 | 4 | 0 | standard |
| [v63](results/avoidance_v63_validation.json) | 10/12 | 34/36 | 2 | 0 | standard |
| [v64](results/avoidance_v64_matrix.json) | 14/18 | 50/54 | 1 | 0 | standard |
| [v65](results/avoidance_v65_matrix.json) | 16/18 | 52/54 | 2 | 0 | standard |
| [v66 integration regression](results/avoidance_v66_matrix.json) | 0/18 | 0/54 | 18 | 0 | standard |
| [v67](results/avoidance_v67_smoke.json) | 2/2 | 6/6 | 0 | 0 | standard |
| [v68](results/avoidance_v68_matrix.json) | 14/18 | 47/54 | 2 | 0 | standard |
| [v69](results/avoidance_v69_validation.json) | 3/4 | 10/12 | 0 | 0 | standard |
| [v70](results/avoidance_v70_matrix.json) | 14/18 | 49/54 | 4 | 0 | standard |
| [v71](results/avoidance_v71_matrix.json) | 9/18 | 46/54 | 4 | 0 | standard |
| [v72](results/avoidance_v72_matrix.json) | 13/18 | 48/54 | 5 | 0 | standard |
| [v73](results/avoidance_v73_matrix.json) | 12/18 | 48/54 | 5 | 0 | standard |
| [v74](results/avoidance_v74_smoke.json) | 1/2 | 5/6 | 1 | 0 | standard |
| [v75](results/avoidance_v75_validation.json) | 10/12 | 34/36 | 2 | 0 | standard |
| [v76](results/avoidance_v76_matrix.json) | 16/18 | 52/54 | 2 | 0 | standard |
| [v77](results/avoidance_v77_smoke.json) | 1/2 | 5/6 | 1 | 0 | standard |
| [v78](results/avoidance_v78_smoke.json) | 0/2 | 4/6 | 2 | 0 | standard |
| [v79](results/avoidance_v79_matrix.json) | 14/18 | 50/54 | 4 | 0 | standard |
| [v80](results/avoidance_v80_validation.json) | 1/6 | 13/18 | 4 | 0 | standard |
| [v81](results/avoidance_v81_validation.json) | 2/6 | 11/18 | 3 | 0 | standard |
| [v82 consolidated](results/avoidance_v82_matrix.json) | 14/18 | 49/54 | 4 | 0 | standard |

Regenerate this ledger's counts with:

```bash
python3 scripts/analyze_avoidance_dataset.py docs/results --cohort-table --minimum-version 38
```

## Verification and rerunning

The latest v82 dataset includes downsampled evaluation traces for dashboard
replay. These traces are offline scoring data, not navigation inputs.

Checks passed: 272 bridge tests, 83 benchmark tests, 18 bringup tests, and 75
script tests. The four Torch-dependent
schedule tests passed separately in the MJLab environment. Source-patch checks
and pristine patch application reproduced both reviewed submodule snapshots.

```bash
python3 scripts/run_avoidance_matrix.py \
  --worlds small_open small_clutter medium_open medium_doorway large_pillars large_office \
  --seeds 42 43 44 --people 4 --goals 3 --trial-seconds 90 \
  --label current-working-tree \
  --publish docs/results/avoidance_next_matrix.json
./geoff results
```

Use fresh publication names when preserving multiple runs. A full pass must be
repeated and verified on held-out seeds before opening the transport gate.

## Transport gate

No end-to-end unpinned, sensor-driven box delivery has been demonstrated.
The existing transport server still contains base-pin phases, and measured box/
pad perception is required for onboard manipulation. The approved sequence remains
navigation acceptance, then measured pickup/docking, balanced arm control,
payload-aware social navigation, physical release/settling verification, and
scored multi-delivery missions. See the [transport plan](TRANSPORT_DEMO_PLAN.md).
