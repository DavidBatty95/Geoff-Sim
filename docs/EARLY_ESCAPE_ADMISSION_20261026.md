# Earlier, delay-aware escape admission — 26 October 2026

**Strategic primitive commands now pass fresh admission and the downstream
supervisor in live runs.** The latest seed-43 survey completed without overlap or
a fall. Seed 42 still failed after overlap, so this is progress in admission—not
qualification of reliable crowd avoidance or delivery.

**27 October scope correction:** the eleven seed-43 primitive publications below
occurred after `survey_complete`, during the observation/shutdown interval. They
are not evidence of helping complete the survey. See the
[verified phase audit and continuation follow-up](SUSTAINED_SURVEY_RECOVERY_20261027.md).

## Why the previous plans were rejected

Inspection of the retained worker/revalidation records found proposals with only
about 6 mm and 32 mm spare clearance, returned roughly 115 ms after submission.
Other, much better-cleared proposals exhausted their foreground validation
allowance. This required both earlier opportunities and a cheaper way to check
plans with reserve; relaxing collision or freshness gates was not appropriate.

## Implemented changes

### Earlier planning trigger

The strategic option now also watches a **closing corridor** before an opposed
pincer forms. Over the existing seven-second forecast it looks for meaningful
closing motion that brings a person within 0.8 m of the existing inflated
envelope. At least two tracks must be present. Stationary jitter, distant crowds
and diverging motion do not independently trigger it. This is a planning trigger,
not a reduced collision-clearance requirement.

Diagnostics distinguish `closing_corridor` from `pincer` and identify the tracks
and predicted time to reserve loss. A robot approaching stationary people is
also considered; meaningful relative motion need not come only from the walkers.

### Clearance reserve across admission delay

Worker proposals require **0.10 m extra human clearance** and are checked at
additional admission delays of **0, 100 and 200 ms**. Every sampled delay variant
must pass. The forecast window extends from 4.5 to 4.7 seconds for the longest
delay, retaining the full braking phase and subsequent two-second hold.

These are sampled delay checks under the motion/forecast model, not a proof for
every possible delay or future pedestrian acceleration. Returned plans still
face the original fresh-state admission and supervisor checks. The worker's
80 ms budget, 200 ms result expiry and mission-context rejection remain in force.

### Make reserved proposals affordable to validate

- Rank a diverse eight-program shortlist toward the sensed refuge before
  searching the full 129-program library. If the shortlist fails nominal or
  delay checks, the full library can use the remaining original budget.
- Use scalar arithmetic for small response/clearance batches and vectorized
  arithmetic for larger ones. Randomized and geometric tests compare both paths.
- Batch the identical conservative static-segment checks against the immutable
  static snapshot instead of issuing hundreds of small grid queries.
- For a ready proposal, prepare fresh static geometry without first scoring an
  unused zero-command anchor against every person. Full proposed-path checks
  remain mandatory.
- Give ready-plan validation priority within the **existing overall planner
  budget**, retaining its 12 ms cap and reserving 10 ms for final scoring and
  publication. This replaces an unnecessarily restrictive local sub-budget;
  it does not increase the configured planner budget or freshness limits.

## Replay evidence and its limits

The [initial replay](results/early_admission_replay_20261026.json) and
[final replay](results/early_admission_final_replay_20261026.json) retain 18 sensed
snapshots across three prior runs. They compare nominal and reserved proposals
against sensed state recorded 150 ms later, including mission-context changes.

The closing-corridor watch activates at the −250 ms sample relative to historical
translation onset in two encounters where the pincer-only trigger is still false.
This establishes earlier sampled opportunities, not a guaranteed live lead time.
Nominal proposals pass the later check in 13/18 snapshots versus 12/18 for the
reserved variant. The reserve rejects one plan that happens to pass the nominal
later check, and some later plans remain infeasible. No aggregate replay-success
improvement is claimed. Computation is unlimited in this diagnostic and full
static geometry is not replayed.

## All seven development attempts

Run IDs below have suffix `_20261026`, use three people and the standard cycle05
red-to-red survey with a 120-second deadline, heading hold, 15 mm grasp offset
and no RViz. Each retains its own source snapshot; changes between runs make
this a development series, not a fixed-build acceptance matrix.

| Run prefix | Seed / implementation checkpoint | Outcome |
| --- | --- | --- |
| `early_admission_survey42` | 42 / initial reserve | 32 overlap samples; fall |
| `early_admission_fast42` | 42 / scalar response | Braking timeout; zero overlap; no fall |
| `early_admission_priority42` | 42 / shortlist | 49 overlap samples; fall |
| `early_admission_ready42` | 42 / omit unused anchor score | 35 overlap samples; fall |
| `early_admission_budget42` | 42 / prioritize admission budget | 20 overlap samples; fall |
| `early_admission_batched42` | 42 / batched clearance checks | Recovery/resumption exercised; braking timeout; 190 overlap samples; no fall |
| `early_admission_batched43` | 43 / final implementation | Survey complete; zero overlap; no fall; minimum reported clearance 1.245 m |

[Cumulative evidence](results/early_admission_validation_20261026.json) includes
all attempts, source hashes, full logger-tail overlap scope, worker decisions and
matched supervisor publications. Earlier single-run reports remain intact.
Overlap counts are geometric samples, not unique collisions or force events.
Runs terminate on their checks; a zero-overlap early timeout is not an avoidance
success. The four intermediate falls remain explicit failures.

### New live admission checkpoint

| Final run | Delay-checked worker proposals observed | Primitive translation commands selected | Matched supervisor samples preserving translation |
| --- | ---: | ---: | ---: |
| Seed 42 | 4 | 3 | 9 |
| Seed 43 | 2 | 11 | 28 |

None of those matched supervisor samples zeroed the translation. These are
planner/supervisor publications, not motor-application acknowledgements or counts
of independent completed escapes. The smallest returned forecast margins were
0.113 m and 0.458 m respectively. Seed 43 exercises the strategic layer after its
task-completion milestone; that activity does not demonstrate a survey benefit.

Seed 42 still accumulated overlap and failed to complete its survey. More time
upright or a lower count in a shorter failed run cannot establish robustness.
No pickup or full delivery was qualified here.

## Verification and use

Complete workspace CI passes **985 ROS results** (789 bridge tests), **238
orchestration tests**, and **13 native CTests**. Focused tests cover early triggers,
sampled-delay rejection, retained braking/hold time, reserve-only rejection,
full-library fallback, response equivalence, equivalent clearance decisions and
fresh static validation without the unused anchor score.

Use the existing `--strategic-avoidance --test-stage survey` options. The strategic
experiment remains off by default; no new flag is required for this refinement.

The remaining task is sustaining clearance after an admitted command while
pedestrian estimates and intent change, then returning to the observation task.
The current constant-velocity forecasts and short horizon can still fail as
walkers accelerate or change direction. That limitation must be addressed without
treating a successful admission as proof of a successful physical encounter.
