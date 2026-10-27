# Research gaps and literature review

<!-- geoff-nav:start -->
<p align="center">
  <a href="../../README.md"><img src="../media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../../README.md#quick-start"><img src="../media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="../DEMO_GUIDE.md"><img src="../media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="../README.md"><img src="../media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="../REPOSITORY_GUIDE.md"><img src="../media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="../PROJECT_STATUS.md"><img src="../media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="../PROJECT_PLAN.md"><img src="../media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="../RESULTS_REVIEW.md"><img src="../media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="README.md"><img src="../media/navigation/research-current.svg" width="132" height="44" alt="Research (current page)"></a>
  <a href="../DEVELOPMENT.md"><img src="../media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

**Research objective:** determine whether a measured model of what a walking
humanoid can actually execute, combined with the age and uncertainty of its
observations, can improve collision-free mission completion around people.

**Latest review — expanded 5 October 2026:**
[Download the complete 74-page PDF](literature_review.pdf) ·
[Research gaps, p. 26](literature_review.pdf#page=26) ·
[Novelty roadmap, p. 30](literature_review.pdf#page=30).

## Research gaps and intended contributions

These are **proposed research directions**, not claims that the capabilities are
already accepted or that no related method exists. The review distinguishes
project deficits from under-tested interfaces in its corpus and from contributions
that still need a precise method, comparison and evidence.

| Priority / gap | Intended contribution | What would demonstrate it? |
| --- | --- | --- |
| **G1 — Calibrated avoidance under stale observations** | A policy-conditioned, measured motion envelope combined with acquisition-age-aware person occupancy | Held-out response/coverage accuracy **and** better collision–completion tradeoffs than generic dynamics and equally conservative fixed margins |
| **G2 — Explain the oracle-to-onboard gap** | A source-audited causal account of how perception, time alignment and execution interact to cause failures | Matched interventions on tracks, pose, maps, delay and response; reproducible effects across held-out encounters, including negative results |
| **G3 — Coordinate safety and liveness** | A defined planner/supervisor handoff or decision rule that checks the motion actually executed | Fewer prolonged veto loops, oscillations and timeouts without increasing contacts or falls, under matched sensing and compute budgets |
| **G4 — Account for body, arms and payload** | A validated boundary for base-only approximations, followed by an efficient posture/load-aware model where needed | Link/payload contact or clearance reference data showing when the approximation changes decisions, plus a measured fidelity–latency tradeoff |
| **G5 — Validate contextual humanoid comfort** | Evidence that a response-aware behavior is more predictable or comfortable in a defined humanoid interaction | Controlled human evaluation separating the method from speed, clearance, morphology and task context; objective outcomes reported alongside judgments |

**Recommended first-paper focus:** G1, supported by the controlled diagnosis in
G2. Include G3 if a specific coordination mechanism is necessary and independently
testable. G4 and G5 are subsequent studies with additional measurement and
validation requirements. The [PDF roadmap](literature_review.pdf#page=30) gives
the concrete N1–N3 goals and extension targets; the
[engineering project plan](../PROJECT_PLAN.md) tracks the implementation work.

**Turn these gaps into experiments:** the new
[P0–P7 research and publication programme](../RESEARCH_PROGRAMME.md) specifies
dependencies, method work, baselines, falsification criteria and candidate-paper
boundaries for every gap.

| Intended gap | Delivery phases | Candidate result |
| --- | --- | --- |
| G1 — Calibrated avoidance under stale observations | [P1 identification](../RESEARCH_PROGRAMME.md#p1--response-and-uncertainty-identification) → [P3 planning](../RESEARCH_PROGRAMME.md#p3--age-aware-response-calibrated-planning) | Paper A, supported by G2 diagnosis |
| G2 — Explain the oracle-to-onboard gap | [P2 causal analysis](../RESEARCH_PROGRAMME.md#p2--causal-oracle-to-onboard-analysis) | Controlled attribution; standalone empirical contribution only if distinct |
| G3 — Coordinate safety and liveness | [P4 coordination](../RESEARCH_PROGRAMME.md#p4--safety-liveness-coordination) | Paper B if a new rule has independent value |
| G4 — Body, arms and payload | [P5 articulated risk](../RESEARCH_PROGRAMME.md#p5--articulated-and-payload-aware-risk) | Paper C: abstraction validity and decision value |
| G5 — Contextual humanoid comfort | [P6 human evaluation](../RESEARCH_PROGRAMME.md#p6--contextual-human-evaluation) | Paper D: controlled human-centered findings |

Every candidate starts with [P0's novelty/protocol audit](../RESEARCH_PROGRAMME.md#p0--protocol-and-novelty-audit)
and uses [P7's replication and paper-release gate](../RESEARCH_PROGRAMME.md#p7--replication-and-paper-release).
These phases extend the review's N1–N3/E0–E6 roadmap into an actionable living
plan. They are proposed work, not a promise of publication or a record of completed studies.

### What prior work already establishes

The novelty claim must begin where the closest prior work ends:

- **Dynamics and stopping are established:** [DWA](https://doi.org/10.1109/100.580977)
  already searches reachable/stoppable velocities. The
  [2016 biped crowd paper](https://doi.org/10.1109/humanoids.2016.7803304)
  already combines balance, uncertainty, stopping and nonreactive crowds.
- **Learned interaction and partial observability are established:**
  [SARL](https://doi.org/10.1109/icra.2019.8794134),
  [RGL](https://doi.org/10.1109/iros45743.2020.9340705) and
  [DS-RNN](https://doi.org/10.1109/icra48506.2021.9561595)
  provide substantive attention, graph and temporal baselines.
- **Sensor-driven humanoid avoidance already exists:**
  [safe humanoid locomotion](https://arxiv.org/abs/2508.07611v1),
  [Omni-Perception](https://arxiv.org/abs/2505.19214) and
  [FLUX](https://arxiv.org/abs/2603.12806v1) constrain broad integration claims.
- **Human evaluation is not absent from the field:**
  [SCAND](https://doi.org/10.1109/lra.2022.3184025) includes participant evaluation;
  [evaluation guidelines](https://doi.org/10.1145/3700599) and
  [contextual proxemics](https://doi.org/10.3389/frobt.2026.1800762)
  provide an established methodological foundation.

The intended novelty is therefore a **specific, validated coupling of observation,
execution and decision-making**, or new controlled findings about that coupling—not
the first use of a humanoid, a safety filter, uncertainty or a social cost.

### Evidence gates for a defensible contribution

1. **Freeze the baseline:** source, policy, configuration, scenarios and evaluator semantics.
2. **Measure the interface:** stopping, turns, reversals, delay and residual error on held-out trajectories.
3. **Specify the new method:** equations, assumptions, uncertainty treatment, timing budget and fallback.
4. **Challenge it fairly:** matched baselines, fixed-margin alternatives and ablations isolating each claimed mechanism.
5. **Test generalization:** complete missions, held-out conditions, repeats, uncertainty and retained failures/invalid attempts.
6. **Explain and bound the claim:** publish causal traces, counterexamples and the conditions where the method does not help.

Current onboard collisions and incomplete missions remain visible in the
[project evidence](../PROJECT_STATUS.md). Passing implementation tests is progress
toward a trustworthy experiment, not completion of these research gates.

## Latest review: one comprehensive PDF

The **74-page expanded edition** tells the story from navigation fundamentals to
candidate novel work. It includes:

- A systematic extraction method for the defined **50-work corpus**, with
  **46 selected full-text appraisals and four explicitly record-limited originals**.
- The progression from geometry and reciprocity to interaction learning,
  prediction, onboard perception, embodied control and complete-mission evaluation.
- Detailed comparisons of methods, sensor/action contracts, metrics, timing,
  physical validity, human evaluation and contradictory tradeoffs.
- Five ranked research gaps, falsifiable hypotheses, a staged experimental
  programme and a concrete novelty roadmap.
- **All 50 critical paper appraisals**, followed by a complete linked bibliography.
- Clickable citations, publication URLs, table of contents, bookmarks and
  bibliography back-links to citing pages.

| Read for… | PDF entry point |
| --- | --- |
| The fundamentals-to-state-of-the-art story | [Fundamentals, p. 9](literature_review.pdf#page=9) |
| Cross-paper interpretation and methodological differences | [Deeper thematic synthesis, p. 22](literature_review.pdf#page=22) |
| The intended research gaps | [Ranked gaps, p. 26](literature_review.pdf#page=26) |
| Goals required to support novelty | [Novelty roadmap, p. 30](literature_review.pdf#page=30) |
| Individual methods, evidence and limitations | [Paper-by-paper appraisals, p. 36](literature_review.pdf#page=36) |
| Full citations and source links | [Linked bibliography, p. 70](literature_review.pdf#page=70) |

Page links work in PDF viewers that support page fragments; the embedded
bookmarks and table of contents provide navigation within the downloaded PDF.
[PDF validation record](pdf_validation.json): 50 linked references, 648 resolved
internal links and 92 bookmarks in this edition.

## Review scope and evidence transparency

The original literature-discovery cutoff remains **22 September 2026**. The
5 October expansion deepens inspection and applies consistent extraction to that
corpus; it is **not an exhaustive new database search or an experimental replication**.
The review records its purposive initial selection, single AI-assisted workflow,
copy/version differences and lack of independent dual screening.

[Full-text audit](FULLTEXT_AUDIT.md) · [Structured appraisals](paper_appraisals.json)
· [Search and expansion record](SEARCH_LOG.md) · [Original September audit](READING_AUDIT.md).

The four original full texts still unavailable are MuJoCo (2012), sparse
trajectory optimization (2013), velocity obstacles (1998) and dense-crowd
cooperation (2013). Their limited appraisal is explicit. The previously inaccessible
biped paper was recovered and materially sharpened the novelty assessment.

**Browse the references:** [50-paper topic archive](../resources/RESEARCH.md).
**Find implementation and upstream context:** [resource library](../resources/README.md).

## Build the PDF

From the repository root:

```bash
python3 docs/literature_review/generate_review_appendix.py
bash docs/literature_review/build.sh
python3 docs/literature_review/verify_pdf.py --write-report
```

The script works from any current directory when invoked by its correct path.
Compilation intermediates go into `docs/literature_review/build/`, which is
locally ignored by Git. The checked PDF is also published beside this README so
the GitHub link works without a local build. It checks bibliography consistency before
compilation and rejects unresolved references, missing glyphs, and overfull boxes.

The main TeX file includes the fundamentals, systematic method, thematic synthesis,
novelty roadmap and generated paper appraisals into this **single PDF**. Edit
`paper_appraisals.json` for the structured records, then regenerate their TeX/audit
outputs. The build checks those generated outputs before compiling.
Install the [isolated quality requirements](../../tools/requirements-quality.txt)
for the `pypdf` validation command. The successful build also writes a provenance
receipt; validation refuses to bless a PDF if its recorded inputs have changed.

Supported toolchains, in selection order:

1. **Tectonic** (tested with 0.17.0; automatically runs BibTeX and required passes).
2. **latexmk + LuaLaTeX**.
3. **LuaLaTeX + BibTeX**, with explicit repeated passes.

For Ubuntu/Debian, the TeX Live option can be installed with:

```bash
sudo apt-get install latexmk texlive-luatex texlive-latex-extra texlive-fonts-recommended
```

Alternatively install [Tectonic](https://tectonic-typesetting.github.io/), or
point the script at an existing binary:

```bash
TECTONIC=/path/to/tectonic bash docs/literature_review/build.sh
```

Tectonic's first build needs network access to fetch its TeX bundle; subsequent
builds reuse its cache. The build does **not** re-query paper databases. Python 3
is required; bibliographic verification uses only the Python standard library,
while PDF inspection uses the pinned quality-tool dependencies.
The document uses Unicode-capable fonts, so plain pdfLaTeX is not a supported
fallback.

Published artifacts are checked against the CI-pinned Tectonic build. On the
supported Linux x86-64 host, after creating the quality environment described in
[Development](../DEVELOPMENT.md), install that same verified binary with:

```bash
python3 scripts/install_quality_tool.py tectonic --directory .venv-quality/bin
TECTONIC="$PWD/.venv-quality/bin/tectonic" bash docs/literature_review/build.sh
.venv-quality/bin/python docs/literature_review/verify_pdf.py --write-report
```

## Verification and updating

Check the delivered snapshot and all citation keys offline:

```bash
bash docs/literature_review/build.sh --check-only
```

Re-fetch every selected identity record, validate identifiers/titles, retain new
retrieval evidence, and regenerate BibTeX:

```bash
python3 docs/literature_review/verify_sources.py --refresh --export
```

The refresh contacts Crossref and arXiv sequentially with retry/backoff. It fails
on unavailable/mismatched records rather than marking them verified. Review the
resulting diff: records and unversioned arXiv pages can change. For adding papers,
edit `sources.json`, refresh, inspect the new content, update the reading audit
and synthesis, then rebuild. Metadata refresh alone does not update the research
assessment or establish that a new full text has been read.

Project-evidence hashes describe the inspection snapshot. Later controller
iterations and measured outcomes have their own provenance in
[the avoidance datasets](../AVOIDANCE_EXPERIMENTS.md).

To repeat PDF link/bookmark checks, install the pinned
[`tools/requirements-quality.txt`](../../tools/requirements-quality.txt) in your
document-tooling environment and run:

```bash
python3 docs/literature_review/verify_pdf.py --write-report
python3 docs/literature_review/verify_pdf.py --check-report
```

Optional primary-text retrieval is explicit and network-dependent:

```bash
python3 docs/literature_review/collect_fulltexts.py --locate
python3 docs/literature_review/collect_fulltexts.py --fetch
```

This caches accessible PDFs/text under ignored `build/fulltexts/` and records
failures. Retrieval does not mark a paper as read; update its appraisal and audit
only after inspecting the material. Publisher PDFs are not bundled in Git.

## Evidence files

| File | Purpose |
| --- | --- |
| `sources.json` | Selected DOI/arXiv identities and expected titles |
| `verified_sources.json` | Retrieved titles/authors/years/venues, abstracts where available, URLs, timestamps, response hashes |
| `references.bib` | Generated BibTeX for the verified snapshot |
| `READING_AUDIT.md` | Per-paper distinction between identity verification and content inspection |
| `supplemental_reading.json` | Additional arXiv abstract records inspected for DOI-cited papers |
| `SEARCH_LOG.md` | Search families, screening decisions, cutoff, limitations, and unresolved follow-up |
| `project_evidence.json` | Hashes and revision context for inspected project files |
| `paper_appraisals.json` | All 50 structured method/evidence/critique/novelty appraisals and inspection anchors |
| `FULLTEXT_AUDIT.md` | Expanded inspection coverage, source copies and access limitations |
| `fulltext_retrieval.json` | Locator/retrieval URLs, timestamps, failed attempts and PDF/text fingerprints |
| `paper_appraisals.tex` | Generated appraisal appendix included in the single PDF |
| `fundamentals.tex`, `systematic_method.tex`, `thematic_synthesis.tex`, `novelty_roadmap.tex` | Expanded narrative and research-goal chapters |
| `pdf_validation.json` | Delivered PDF hash, page count, citation/link checks and bookmark/page map |
| `build_provenance.json` | Successful-build receipt tying the PDF to TeX/BibTeX and declared data/build inputs |

**Verification boundary:** bibliography identity, primary-material appraisal,
document integrity and experimental replication are different checks. The
expanded review strengthens the first three; it does not claim the fourth.
