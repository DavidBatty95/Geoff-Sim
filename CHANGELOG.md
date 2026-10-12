# Changelog

<!-- geoff-nav:start -->
<p align="center">
  <a href="README.md"><img src="docs/media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="README.md#quick-start"><img src="docs/media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="docs/DEMO_GUIDE.md"><img src="docs/media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="docs/INDEX.md"><img src="docs/media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="docs/REPOSITORY_GUIDE.md"><img src="docs/media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="docs/PROJECT_STATUS.md"><img src="docs/media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="docs/PROJECT_PLAN.md"><img src="docs/media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="docs/RESULTS_REVIEW.md"><img src="docs/media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="docs/literature_review/README.md"><img src="docs/media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="docs/DEVELOPMENT.md"><img src="docs/media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

User-facing changes and significant development publications are summarized here.
Entries identified by commit are **development snapshots**, not numbered stable
releases or new acceptance claims. See [project status](docs/PROJECT_STATUS.md)
for measured outcomes and [release policy](docs/RELEASES.md) for promotion criteria.

## Unreleased

### Added

- Software citation metadata, community conduct guidance and actionable security-reporting routes.
- Maintainer/release runbooks and a contributor test-selection guide.
- Focused repository-quality checks, generated-document validation and PDF build provenance.
- Change-aware CI with a stable final verification gate and conditional review-PDF rebuilds.
- Optional pre-commit checks for malformed files, conflict markers, private keys and large additions.

## 2026-10-06 — Hardware-facing development snapshot

[`d1d54ff`](https://github.com/DavidBatty95/Geoff-Sim/commit/d1d54ff)

- Published G1 command CRC/machine-mode handling, sensor-site IMUs, measured TF
  and hardware point-cloud layout support.
- Added BrainCo interface adapters, asynchronous supervision, native-delay payload
  training and recoverable RGB-D transport work.
- Retained native transport and verification evidence with explicit limitations.
- Added gap-aligned research phases and candidate-paper planning.

See [hardware fidelity](docs/HARDWARE_FIDELITY.md) and
[native payload contract](docs/NATIVE_PAYLOAD_CONTRACT.md). These are simulation
development results; crowd and physical-hardware qualification remain separate.

## 2026-10-05 — Research review and repository navigation

[`95f6994`](https://github.com/DavidBatty95/Geoff-Sim/commit/95f6994) ·
[`b2a29ff`](https://github.com/DavidBatty95/Geoff-Sim/commit/b2a29ff) ·
[`7fc09a5`](https://github.com/DavidBatty95/Geoff-Sim/commit/7fc09a5)

- Expanded the literature review into a single 74-page PDF with 50 critical
  appraisals, a linked bibliography and a novelty roadmap.
- Put intended research gaps first on the Research page.
- Added a topic-clustered resource library, online-reference directory and
  generated animated navigation shared across project pages.

## 2026-10-05 — Navigation and payload workflows

[`3a7ceac`](https://github.com/DavidBatty95/Geoff-Sim/commit/3a7ceac)

- Published navigation corrections, payload-training and experimental transport
  tools, regression tests and retained experiment records.
- Refreshed reviewed simulator patches to reproduce the maintained source snapshot.

## 2026-09-22 — Worlds and inspectable evidence

[`7c4cd3e`](https://github.com/DavidBatty95/Geoff-Sim/commit/7c4cd3e) ·
[`886abb2`](https://github.com/DavidBatty95/Geoff-Sim/commit/886abb2)

- Added representative navigation worlds, the browser results dashboard and
  portable benchmark evidence.
- Published the initial transport-development plan.

Earlier milestones are indexed in [project history](docs/resources/HISTORY.md).
The [Git history](https://github.com/DavidBatty95/Geoff-Sim/commits/main) is the
complete change record; historical results retain their original configuration.
