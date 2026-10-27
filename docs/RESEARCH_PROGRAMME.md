# Research programme: from identified gaps to defensible papers

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

[Research gaps](literature_review/README.md#research-gaps-and-intended-contributions) ·
[Engineering plan](PROJECT_PLAN.md) · [Technical roadmap](SOCIAL_LOCOMANIPULATION_ROADMAP.md) ·
[Complete literature review](literature_review/literature_review.pdf)

**Proposed programme — 6 October 2026.** This plan turns the review's five gaps
into research phases with defined questions, methods, comparisons and evidence.
All P0–P7 phases are **planned** until an owner, frozen protocol and phase evidence
are recorded. Existing implementations and development pilots are starting
material; their existence does not close the scientific gates below.

**Recommended first paper:** a measured observation-to-execution model for
humanoid social navigation (**G1**), supported by controlled diagnosis of the
oracle-to-onboard gap (**G2**). Coordination, articulated risk and human validation
are later candidates when they establish distinct results.

![Research phases: protocol, identification, causal diagnosis, calibrated planning, coordination, body and payload risk, human evaluation, and reproducible publication](media/research-programme.svg)

## How the plans fit together

| Identifier | Meaning | Authority |
| --- | --- | --- |
| **G1–G5** | Scientific questions and intended contribution boundaries | [Gap-first Research page](literature_review/README.md) |
| **S0–S7** | Engineering capability and operating qualification | [Project plan](PROJECT_PLAN.md#stage-deliverables-and-exit-evidence) |
| **R1–R6** | Implementation work on timing, ownership, response and persistent skills | [Technical workstream](SOCIAL_LOCOMANIPULATION_ROADMAP.md#reactive-simulation-workstream-s1-s2-s4-and-s5) |
| **P0–P7** | Research delivery: protocol, experiments, attribution and manuscript evidence | This page |
| **N1–N3 / E0–E6** | Contribution targets and evidence ladder in the published review | [PDF novelty roadmap, p. 30](literature_review/literature_review.pdf#page=30) |

Scientific studies can examine failing controllers once their measurements are
trustworthy. They do not need to wait for physical deployment or an all-pass
navigation release. Conversely, publishing an improvement does not relax S2's
strict navigation criteria or qualify S6/S7 crowd transport and hardware use.
A negative or inconclusive study can complete its protocol while leaving a
proposed positive contribution unsupported.

## Phase and gap crosswalk

| Phase | Gap / review target | Engineering support | Principal deliverable |
| --- | --- | --- | --- |
| [P0 Protocol and novelty audit](#p0--protocol-and-novelty-audit) | All gaps; E0/E2 | S0/S1 | Prior-art differentiation, frozen study contract and analysis plan |
| [P1 Response and uncertainty identification](#p1--response-and-uncertainty-identification) | G1; N1.1/N1.2, E1 | S1/S5; R1/R4 | Held-out response models and occupancy-coverage report |
| [P2 Causal oracle-to-onboard analysis](#p2--causal-oracle-to-onboard-analysis) | G2; N2, E3/E5 | S0/S1/S2; R1/R5 | Controlled interventions, interaction effects and failure taxonomy |
| [P3 Age-aware response-calibrated planning](#p3--age-aware-response-calibrated-planning) | G1; N1.3, E2–E4 | S2; R1/R3/R4 | Explicit method and matched collision–completion comparisons |
| [P4 Safety–liveness coordination](#p4--safety-liveness-coordination) | G3; N3 | S1/S2/S4; R2/R3/R6 | Coordination rule with measured liveness and risk effects |
| [P5 Articulated and payload-aware risk](#p5--articulated-and-payload-aware-risk) | G4; body-risk extension | S3/S5/S6; R4 | Abstraction-validity and fidelity–latency study |
| [P6 Contextual human evaluation](#p6--contextual-human-evaluation) | G5; comfort extension | S4/S6/S7 as applicable | Human-centered evidence with controlled behavior comparisons |
| [P7 Replication and paper release](#p7--replication-and-paper-release) | Every candidate; E4–E6 | S0 plus the claimed operating scope | Reproducible manuscript, artifacts and reviewed claim boundary |

## Dependencies and delivery order

```text
S0/S1 measurement readiness → P0 protocol and novelty audit
                                 ├── P1 identification and calibration ──┐
                                 └── P2 causal diagnosis ────────────────┤
                                                                       ↓
                                                    P3 core method → P7 → Paper A
                                                       ├── P4 coordination → P7 → Paper B, if distinct
                                                       └── P5 body/load risk → P7 → Paper C, if distinct

Qualified selected behaviors + P0 human-study protocol → P6 → P7 → Paper D
```

P1 and P2 can run in parallel after the relevant P0 protocol is frozen. P5 can
begin with isolated geometry experiments before P3 finishes. P6 can start with a
properly scoped video/simulation study; physical-participant experiments require
the corresponding operating qualification. **P7 is repeated for each paper**,
not postponed until every extension is complete. Plans and pilot work may proceed
in parallel with engineering; confirmatory evidence uses a frozen candidate.

## P0 — Protocol and novelty audit

**Purpose:** define what could be new and make the experiment capable of testing it.
**Entry:** a declared operating scope and an instrumented candidate whose source,
configuration, policy and evaluator identity can be recovered.

### Work packages

- **P0.1 Nearest-prior-work search.** Extend the review around uncertainty-aware
  biped MPC, delayed/sampled-data CBFs, learned reachable sets, decision-focused
  prediction and uncertainty calibration. Retain query dates, candidate records,
  version choices, deduplication and exclusion reasons. Resolve the review's four
  inaccessible originals where possible; disclose any remaining restrictions.
- **P0.2 Differentiation matrix.** For each candidate claim, compare the actual
  mathematical object, inputs, action interface, assumptions, guarantee and
  evaluation against the closest methods. Obtain a critical second reading
  where available. A new robot or software combination alone is insufficient.
- **P0.3 Study contract.** Freeze scenarios, actor behavior/schedules, goals,
  policy, command shaping, sensing, compute budget and event definitions.
  Declare which elements are interventions and which must stay fixed.
- **P0.4 Analysis plan.** Specify primary outcomes, meaningful effect sizes,
  risk/non-inferiority margins where applicable, independent units, sample-size
  justification, uncertainty estimates, missing-data treatment and stopping rules
  before confirmatory results are inspected.
- **P0.5 Baseline readiness.** Check implemented adapters against their published
  contracts; label adaptations and reproduction limitations. Give comparators
  comparable tuning resources on development conditions only.

**Exit evidence:** a versioned protocol, prior-art/claim matrix, comparator
implementation records and executable pilot confirming that required measurements
are available. All items need an artifact or explicit unresolved blocker.

**Decision:** if the proposed mechanism is already established under equivalent
assumptions, refine the method or adopt a clearly bounded empirical/replication
question. Do not rename an existing method and retain the original novelty claim.

## P1 — Response and uncertainty identification

**Gap:** G1. **Hypothesis:** a compact measured model predicts the deployed
humanoid's short-horizon motion and uncertainty more usefully than instantaneous
or hand-tuned models on held-out transitions.

**Prior-art boundary:** [DWA](https://doi.org/10.1109/100.580977) already reasons
about reachable/stoppable velocities; [biped MPC](https://doi.org/10.1109/humanoids.2016.7803304)
already combines feasibility and uncertainty; [ASAP](https://arxiv.org/abs/2502.01143v3)
already learns dynamics corrections. The target is a specific, validated
**policy-level navigation interface**, not generic dynamics awareness.

### Work packages

- **P1.1 Measurement campaign:** collect forward/lateral steps, ramps, reversals,
  pivots, combined-axis commands and stops across initial speeds and headings.
  For loaded extensions, vary arm pose, payload and transitions explicitly.
- **P1.2 Time and command lineage:** retain acquisition time, estimator output,
  planned command, normalized/supervised command, policy-consumed command,
  low-level publication and measured response. Distinguish actual acknowledgements
  from publication timestamps and simulation time from wall time.
- **P1.3 Model identification:** compare instantaneous, generic bounded-dynamics,
  hand-tuned, fitted deterministic and fitted residual-envelope models. Test
  deadbands, asymmetric response, stopping tails and cross-axis coupling.
- **P1.4 Coverage and sharpness:** evaluate horizon-dependent position/heading
  error, stopping-distance error, interval/occupancy coverage and envelope width.
  Check conditional performance by maneuver, load and observation quality.
- **P1.5 Leakage control:** split by complete trajectories and operating conditions.
  Separate fit, calibration, development and held-out test sets; adjacent telemetry
  samples from one run must not be split across those roles.

**Exit evidence:** identified model, parameter/data manifests, held-out prediction
and coverage results, failure examples and inference cost. Numerical targets are
chosen in P0 from the intended decision tolerance and pilot variability, then frozen.

**Falsification / stop condition:** if a generic envelope achieves equivalent
coverage and useful sharpness, or the learned fit fails outside calibration
conditions, simplify or reject the additional model. Better open-loop prediction
alone does not satisfy G1; P3 must test downstream decision value.

## P2 — Causal oracle-to-onboard analysis

**Gap:** G2. **Hypothesis:** particular interactions among observation age, tracking
error and execution response explain failures more strongly than aggregate
perception accuracy or map quality alone.

### Work packages

- **P2.1 Source-intervention matrix:** independently substitute person tracks,
  robot pose and map information with their evaluation counterparts. Oracle
  substitutions are labelled experimental ablations, never onboard acceptance.
- **P2.2 Perturbation design:** inject declared acquisition-age delays, dropouts,
  center/velocity bias, identity errors and pose drift. Include combined conditions
  to identify interactions; isolate information loss from clock/transform bugs.
- **P2.3 Encounter matching:** freeze actor schedules and goal definitions for
  controlled replay. Record deviations caused by asynchronous execution; treat
  responsive actors as a separate experiment from nonreactive replay.
- **P2.4 Failure attribution:** link measurements to decisions, supervisor vetoes,
  policy input and physical outcomes. Classify contact, fall, deadline, deadlock
  and invalid evaluation without deleting unsuccessful trials.
- **P2.5 Generality checks:** repeat the important interventions on held-out
  layouts/encounter families and assess whether the ranking of causes persists.

**Suggested initial design:** source mode × age correction × response model,
with pose/map interventions in additional blocks if a full factorial is too
costly. Choose the precise levels and repetitions in P0; retain compute-load
measurements so a larger model's scheduling cost is part of the treatment.

**Exit evidence:** source-audited datasets, interaction-effect plots and a
reproducible failure taxonomy. A causal claim requires an intervention and a
credible matching argument; a correlation in one trace is diagnostic evidence.

**Falsification:** if an apparent effect disappears after matching goals,
exposure or policy settings, report the confound. If oracle tracks do not help
but changing the low-level controller does, revise the proposed perception-centric
explanation. P2 can support a standalone empirical paper only if the findings
are generalizable and materially extend existing benchmark analysis.

## P3 — Age-aware response-calibrated planning

**Gap:** G1, informed by G2. **Primary paper candidate: A.**
**Hypothesis:** a jointly time-aligned person-occupancy and executable robot-motion
model improves the collision–completion frontier under matched conditions.

### Method to specify

Define the robot's predicted occupancy from a measured policy response and its
residual envelope. Align each person's occupancy from the measurement's
acquisition time, preserving that time when tracks are republished. Define how
the decision rule treats uncertainty dependence, out-of-calibration inputs,
model expiry and the absence of a feasible candidate. State whether the result
is an empirical model, a probabilistic coverage claim or a formally justified
guarantee; these require different evidence.

### Required comparison matrix

| Variant | What it isolates |
| --- | --- |
| Instantaneous motion + current observations | Cost of ignoring execution response |
| Generic bounded dynamics / current hand-tuned envelope | Value beyond established feasible-motion modeling |
| Fitted deterministic response, age correction off/on | Contribution of calibration and time alignment |
| Fitted response + residual uncertainty, age correction off/on | Value and interaction of uncertainty treatment |
| Equally conservative fixed spatial/time margins | Whether a simpler buffer explains the benefit |
| A documented classical planner adapter | External methodological context, e.g. a suitable DWA/VO/TEB family baseline |
| A compatible learned comparator, where reproducible | Context against interaction/memory learning; match source and action contracts or report a separate transfer study |

The [2016 biped formulation](https://doi.org/10.1109/humanoids.2016.7803304)
must appear in the differentiation analysis. If its original controller cannot
be reproduced on the same interface, document that limitation and the precise
adaptation used; do not present a loosely inspired implementation as a faithful
replication. Raw-sensor and state-vector policies require separate information
conditions rather than an unfair pooled ranking.

### Experiments and evidence gate

Use people-free response controls followed by crossings, head-on encounters,
overtaking, doorways, occluded emergence, sudden stops/turns and sustained crowds.
Sweep declared observation age, density and human/robot speed ratio. Include
natural as well as injected timing variation and reserve scene/encounter families
for held-out evaluation. Freeze tuning before that evaluation.

**Exit evidence:** reproducible algorithm specification, calibrated model tests,
matched comparisons, interaction ablations and complete-mission outcomes with
uncertainty. Support an improvement only if it meets the predeclared practically
meaningful effect criterion and risk constraints; otherwise label it negative
or inconclusive. Faster callbacks or a better fit alone do not satisfy this gate.

**Expected paper figures:** pipeline/source contract; held-out response and
coverage plots; collision–completion frontier; factor-interaction plot; timing
tails; representative successes and failures with applied-command lineage.

## P4 — Safety-liveness coordination

**Gap:** G3. **Candidate paper: B, only with a distinct mechanism.**
**Entry:** trustworthy action/command lineage and a fixed response/prediction
contract, using P3 if its model proves useful.

**Hypothesis:** a defined coordination rule reduces prolonged veto loops,
oscillation and task timeouts without worsening contact/fall risk relative to
independently designed planning and supervision.

- **P4.1 Formalize the handoff:** specify which trajectory/command was checked,
  the validity window, mission generation, authority and fallback semantics.
  Correcting missing subscriptions or stale-result bugs is baseline hygiene.
- **P4.2 Define the new decision rule:** explain why and when to yield, resume,
  wait, detour or replace a route. Identify any invariant/guarantee assumptions;
  otherwise make a bounded empirical claim.
- **P4.3 Isolate coordination:** hold sensing, predictor, policy and response
  model fixed. Compare independent versus shared supervision, simple hysteresis,
  fixed-time waiting and the proposed rule under matched compute budgets.
- **P4.4 Stress liveness:** test prolonged blockage, ambiguous crossings,
  repeated replanning, late results, cancellation and new-goal handoffs. Separate
  ordinary disturbance recovery from missing or invalid evaluation.

**Exit evidence:** reduced veto duration/reversals and improved resumption or
completion meeting P0 criteria, with contact/fall outcomes and latency reported
together. Show where the mechanism fails. If the effect is explained by the P3
model alone or by a simple debounce, integrate the result as an ablation in Paper A.

**Prior-art boundary:** freezing, reciprocal negotiation and hierarchical
navigation are established. Novelty needs the specific coordination mechanism
and evidence, not a new set of state-machine names or a higher requested rate.

## P5 — Articulated and payload-aware risk

**Gap:** G4. **Candidate paper: C.**
**Hypothesis:** posture/load-conditioned executable occupancy identifies important
risks missed by base-only models at a better fidelity–latency tradeoff than
unconditionally applying full-body checking.

- **P5.1 Establish reference measurements:** compute articulated link/payload
  clearance and native contacts with validated frame/time alignment. Keep
  physical contact, conservative geometry overlap and proxemic intrusion distinct.
- **P5.2 Characterize abstraction failure:** compare base-only, inflated-base,
  posture-conditioned and articulated swept checks during turns, lateral motion,
  reach/carry/rest transitions and recovery. Vary payload geometry/mass and
  initial motion on declared development/held-out splits.
- **P5.3 Design selective fidelity:** if base abstraction errors change decisions,
  specify when a more detailed check is invoked and how its runtime fits the
  response budget. Test load-conditioned dynamics separately from geometry.
- **P5.4 Validate in missions:** begin with empty-room and scripted-interaction
  fixtures, then qualify broader delivery conditions through the S-stage gates.
  Label magnetic attachment experiments separately from contact-only grasping.

**Exit evidence:** a reproducible abstraction-error map, decision-level false
accept/reject rates, contact/fall/drop outcomes and computational cost. A more
detailed mesh is not sufficient; show an improved decision or a valuable finding
about when the simpler approximation is adequate.

**Prior-art boundary:** whole-body control, capability-aware navigation and
dynamics correction already exist in ExBody, Parkour, wheeled-legged navigation
and ASAP. The claim must concern a specific risk/representation method or
generalizable abstraction-validity result. A pure loaded-policy training gain
needs its own baselines, including initialization-only and held-posture controls.

## P6 — Contextual human evaluation

**Gap:** G5. **Candidate paper: D.**
**Hypothesis:** the selected response-aware behavior improves perceived
predictability or comfort in a defined humanoid task beyond effects explained
by speed, clearance and morphology alone.

- **P6.1 Define constructs and setting:** distinguish comfort, legibility,
  perceived safety and task appropriateness. Choose the population, task and
  instrument; obtain the applicable participant-study review/consent before data
  collection. Set sample size from the study design and target precision/effect.
- **P6.2 Design controlled stimuli:** counterbalance condition order and match
  speed/clearance where they are not the treatment. Include relevant arm/payload
  and interaction contexts without changing everything simultaneously.
- **P6.3 Select an evidence tier:** video or interactive simulation can test a
  bounded perception question; physical encounters require appropriate S6/S7
  qualification. Do not generalize video ratings directly to in-person behavior.
- **P6.4 Analyze participants, not frames:** account for repeated measures,
  participant/scene effects and multiple outcomes. Report effect sizes,
  uncertainty, missing responses and predefined exclusions.
- **P6.5 Relate judgments to objective behavior:** pair ratings with motion,
  clearance, yielding, task completion and risk evidence. Test whether the
  behavioral mechanism explains a benefit once simple physical differences are controlled.

**Exit evidence:** a transparent study report with valid instruments and bounded
claims. A larger clearance or lower jerk is a proxy, not a demonstrated comfort
gain. If judgments do not improve, retain that finding and revise the social claim.

**Prior-art boundary:** SCAND already includes human evaluation; Habitat 3.0,
the evaluation guidelines and proxemics taxonomy supply established approaches.
The new question must be specific to the proposed mechanism and humanoid/task
context, not simply the addition of a questionnaire.

## P7 — Replication and paper release

**Applies to every candidate paper.** This is a repeatable release gate, not a
requirement to finish all five gaps before writing the first manuscript.

1. **Freeze the scientific claim:** state scope, comparator, conditions and
   supported outcomes in one paragraph. Refresh nearest-prior-work checks and
   distinguish the new result from earlier project publications.
2. **Repeat the evidence:** rerun from a clean source/configuration snapshot;
   use held-out conditions and an independent rerun/review where practicable.
   Hardware or cross-simulator transfer is required only for claims that assert it.
3. **Audit analysis:** planned/executed/valid/successful denominators, missingness,
   confidence intervals, effect sizes, timing boundaries and failures all remain visible.
4. **Package artifacts:** source and policy identity, protocol, manifests,
   processed data/schema, analysis commands, plotting code and representative traces.
   Record access restrictions rather than claiming inaccessible assets are reproducible.
5. **Write the paper around evidence:** problem → nearest prior work → precise
   method/question → protocol → results → ablations → failures → limitations.
   A demonstration video supports this account but does not replace it.
6. **Review the manuscript:** verify every numerical claim against an artifact,
   check citation and figure provenance, and distinguish empirical robustness
   from formal guarantees and simulation from physical validation.

**Exit evidence:** reviewed claim-to-artifact matrix, repeatable analysis and a
complete submission package. Submission is an owner/research-team decision;
passing this gate does not guarantee acceptance. A rigorous negative finding
can support a paper if it is informative and genuinely extends existing evidence.

## Shared experimental contract

These requirements apply across phases and should become part of each P0 protocol.

| Element | Required decision |
| --- | --- |
| Primary outcome | Prefer complete mission without prohibited contact/fall, with valid evaluation; define goal/stop/task predicates and budgets exactly |
| Separate outcomes | Contact incidence/count, falls, drops, task timeout/deadlock and invalid attempts; do not hide them inside one weighted score |
| Time and exposure | State wall/simulation/acquisition time, trial duration, distance and people exposure; avoid success-only efficiency rankings |
| Experimental unit | Declare scenario/seed/repeat and clustering of shared scenes or repeated measurements; telemetry samples are not independent trials |
| Split discipline | Separate model fitting, residual calibration, tuning and final testing by complete runs/conditions |
| Tuning parity | Record comparator configuration, search effort and compute/hardware; distinguish original implementations from adapted baselines |
| Sample size | Use pilot variance or target rate precision to justify repetitions before final evaluation; do not stop at the first all-pass streak |
| Uncertainty | Rate intervals and paired/cluster-aware differences where justified; distinguish non-inferiority margins from failure to find significance |
| Missingness | Retain every attempted trial; report method-induced overload/data loss rather than quietly excluding it |
| Generalization | Hold out scene/encounter families or operating conditions appropriate to the claim, not merely neighboring frames |

For context, zero failures in 100 **independent representative** trials still
permits an approximately 3% one-sided 95% upper failure-probability bound.
Repeated correlated encounters do not justify the independent-trial calculation.
Precision targets and operating scope must be chosen before calling a result reliable.

## Candidate paper portfolio

| Candidate | Proposed central question | Minimum evidence | Decision boundary |
| --- | --- | --- | --- |
| **A — Observation-age-aware, response-calibrated humanoid navigation** | Does the specified measured model improve the collision–completion frontier? | P0–P3 and P7; held-out model tests, external context, fixed-margin controls and causal ablations | First priority; do not claim general SOTA from unmatched comparisons |
| **B — Consistent supervision with bounded liveness** | Does a new coordination rule resolve planner/supervisor disagreement without shifting risk? | P0/P2/P4/P7 with a fixed sensing/response model | Separate only if the contribution is distinct from A; otherwise include as its ablation/extension |
| **C — Articulated/payload risk at a bounded compute cost** | When do base abstractions fail, and does selective detailed checking improve decisions? | P0/P1/P5/P7 and the relevant geometry/contact qualification | Distinct representation/decision result, not only more accurate geometry or a new payload demo |
| **D — Human validity of response-aware humanoid behavior** | Do controlled behavior changes improve participant judgments in a specified context? | P0/P6/P7 and the chosen simulation/physical evidence tier | Distinct human-centered finding; acknowledge objective/subjective disagreement |

P2 may also support an independent empirical/evaluation contribution if it
produces robust new findings and reusable methodology beyond the support needed
for A. Decide this after analysis; do not divide one result into nominally
different papers with overlapping claims. Venue choice follows the contribution
and evidence—methods, systems/evaluation or HRI—not a promised publication count.

## First actionable research cycle

1. Nominate an owner for P0 and draft Paper A's one-paragraph claim and nearest
   comparator matrix. Review it against the [74-page review](literature_review/literature_review.pdf).
2. Inventory S0/S1 measurement gaps that prevent P1/P2 experiments, distinguishing
   engineering correctness from proposed method changes.
3. Specify a small people-free response pilot and a matched crossing/occlusion
   pilot. Use them to validate instrumentation and estimate design parameters.
4. Freeze development/calibration/held-out splits, analysis choices and the
   confirmatory protocol before tuning or comparing final outcomes.
5. Execute P1/P2, review failures and choose the simplest justified P3 model.
   Open P4–P6 only when their dependencies and distinct contribution are clear.

## Phase tracking and claim ledger

Use **Planned → Protocol frozen → Pilot → Confirmatory → Analyzed → Claim reviewed**
to track scientific delivery. Record the outcome as **supported, refuted or
inconclusive** separately from whether the experiment was completed. All phases
begin as planned here; no owner, deadline, positive effect or publication is inferred.

```text
Phase / gap / candidate paper:
Owner and linked issue:
Nearest prior method and precise intended difference:
Hypothesis and falsification criterion:
Prerequisite measurements / engineering gate:
Protocol version and source/policy/configuration identity:
Baselines, splits, sample-size and analysis decisions:
Artifact index and measured outcomes, including failed/invalid attempts:
Status / claim outcome / remaining threat to validity:
Next decision: proceed, simplify, revise, stop or prepare P7:
```

Keep this programme aligned with the [gap-first Research page](literature_review/README.md),
[project status](PROJECT_STATUS.md) and [engineering plan](PROJECT_PLAN.md).
When a result changes the intended claim, update the ledger and relevant review
discussion with its date and source; do not rewrite old experiments as if they
used the new method.
