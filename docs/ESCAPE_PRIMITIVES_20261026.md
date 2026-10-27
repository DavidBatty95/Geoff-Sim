# Process-isolated escape planning — 26 October 2026

**The next strategic-planning priority is implemented: reusable escape motions,
bounded background search and fresh-state revalidation.** The worker now returns
complete checked snapshot plans in the difficult live encounter. None survived
the final admission checks in that encounter, so this is not yet a successful
physical escape or a qualified crowd demo.

The [later admission follow-up](EARLY_ESCAPE_ADMISSION_20261026.md) adds an earlier
trigger and sampled-delay reserve, and records the first primitive translations
that pass both fresh admission and the supervisor. Seed 42 remains unsuccessful.

## Implementation

- A library of **129 command programs** covers standing, straight translations
  of different durations and two-direction dogleg escapes, all ending in braking.
  Commands respect asymmetric forward/reverse/lateral bounds and the planar cap.
- Only command normalization and phase times are cached. Every request recomputes
  response-aware trajectories, swept human/static clearance, terminal settling
  and a two-second zero-command hold from current state.
- Coarse refuge queries and native pivot-yaw interpolation are batched rather
  than repeated in Python per candidate.
- The strategic option uses a **single process worker**, prewarmed before the
  first request. No stale request queue is maintained. The numeric worker receives
  copied read-only forecasts and static-field data; it never accesses a ROS node.
- Background template search has an **80 ms** wall-clock budget. The owning
  controller polls without waiting. Results older than **200 ms**, or with changed
  goal/mission/mode/payload/model context, cannot authorize a command.
- A returned or continuing program undergoes full current-state revalidation
  with a **12 ms maximum**, bounded further by the remaining foreground search
  allowance. Freshness is checked again after validation. Existing candidate and
  supervisor checks still apply; no partially checked plan is returned on timeout.
- Remaining program durations advance rather than restarting at every tick.
  Reusing a program still invokes full new clearance checks. Scoring now includes
  a primitive's known braking phase instead of truncating it at the ordinary
  three-second horizon and inventing a moving terminal state.

The change remains behind `--strategic-avoidance`, disabled by default. The
supervisor's existing short-escape worker retains its thread executor; only the
new strategic primitive search uses the process executor.

## Diagnostics distinguish the failures

`kinodynamic_details` records preprocessing time, worker outcomes and current-state
revalidation outcomes separately. Reasons include deadline exhaustion, no feasible
prefix, unsafe terminal state/hold, changed context and state expiring during
search. Worker counters distinguish failures, expired results and actual proposals.

The [initial bounded replay](results/escape_primitives_replay_20261026.json)
showed that the synchronous prototype exhausted its 12 ms allowance in nine of
ten retained snapshots; the remaining snapshot had no feasible prefix. The
[unrestricted diagnostic](results/escape_primitives_unbounded_diagnostic_20261026.json)
found complete template plans in eight of ten snapshots, with substantial timing
variation. Beam search also found eight, but not exactly the same eight. These
results isolate sensed constant-velocity forecasts without replaying full static
geometry or physical execution. The unrestricted study is not a timing pass.

The first live implementation used a thread. It reduced foreground work but
still returned no plans within its budget. Process isolation subsequently
produced complete plans, and polling before optional refuge reranking reserved
more foreground time for mandatory revalidation. All intermediate attempts and
their source snapshots remain retained.

## Every live attempt

All IDs have suffix `_20261026`, use three people and the same declared transport
survey protocol, and retain source snapshots and runtime parameters.

| Run prefix | Seed / implementation | Outcome |
| --- | --- | --- |
| `primitive_survey42` | 42 / thread worker | Survey failed at settling check after recovery; 39 overlap samples; no fall |
| `primitive_survey43` | 43 / thread worker | Survey complete; zero overlap; no fall |
| `primitive_process_survey42` | 42 / initial process worker | Four worker proposals; none admitted; 47 overlap samples and a fall |
| `primitive_process_recheck42` | 42 / prioritize revalidation | Four worker proposals; none admitted; 53 overlap samples and a fall |
| `primitive_process_survey43` | 43 / final process version | Survey complete; zero overlap; no fall; minimum reported clearance 0.845 m |

[Cumulative evidence](results/escape_primitives_validation_20261026.json) includes
all five attempts and full logger-tail overlap scope. Intermediate
[thread](results/escape_primitives_live_20261026.json),
[initial process](results/escape_primitives_process_initial_20261026.json) and
[revalidation-priority](results/escape_primitives_process_recheck_20261026.json)
reports are preserved. Overlap counts are geometric samples, not unique
collisions or measured force events.

### What the final seed-42 run establishes

The worker returned four complete snapshot plans. One was rejected because the
mission context changed, two failed fresh prefix-clearance checks, and one
exceeded the remaining foreground validation budget. There were **zero selected
primitive commands**. The planner did not force an old plan through after its
conditions changed.

The measured foreground strategic preparation/search phase averaged **15.19 ms**
and peaked at **39.17 ms**, compared with 49.30 ms average in the earlier strategic
development run. These are different live runs, not a controlled timing-speedup
benchmark. The worker's time is separate from foreground time.

Seed 43's final survey did not activate the pincer condition. Its pass therefore
does not demonstrate a primitive-avoidance benefit. No delivery was attempted or
qualified in this session, and the two recorded falls remain acceptance failures.

## Verification

Tests cover shared-response equivalence at non-grid command handoffs, asymmetric
limits, collisions during the terminal hold, rejection after new obstacles,
deadline behavior, immutable snapshot copies, actual process serialization with
static geometry, fresh continuation and complete braking-phase scoring. Existing
single-flight worker tests cover stale results and invalidation.

Complete workspace CI passes **968 ROS results** (772 bridge tests), **238
orchestration tests**, and **13 native CTests**.

## Running and next limitation

Use the existing transport command with `--strategic-avoidance --test-stage survey`.
No additional flag is needed for process isolation. The native cycle05 checkpoint
and completed build are still required.

The remaining limitation is **validity at execution time**, not just whether a
worker can construct a plan. Plans need more clearance reserve and/or earlier
initiation so they survive sensing, planning and admission delay. The fresh
rejections and deadline must remain visible; relaxing those checks would turn
stale snapshot feasibility into an unjustified live-safety claim.
