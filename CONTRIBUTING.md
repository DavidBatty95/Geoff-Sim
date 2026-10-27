# Contributing

<!-- geoff-nav:start -->
<p align="center">
  <a href="README.md"><img src="docs/media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="README.md#quick-start"><img src="docs/media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="docs/DEMO_GUIDE.md"><img src="docs/media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="docs/README.md"><img src="docs/media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="docs/REPOSITORY_GUIDE.md"><img src="docs/media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="docs/PROJECT_STATUS.md"><img src="docs/media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="docs/PROJECT_PLAN.md"><img src="docs/media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="docs/RESULTS_REVIEW.md"><img src="docs/media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="docs/literature_review/README.md"><img src="docs/media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="docs/DEVELOPMENT.md"><img src="docs/media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

Start with [the handoff](docs/HANDOFF.md), [architecture](docs/ARCHITECTURE.md)
and [development guide](docs/DEVELOPMENT.md). This is a simulation research
prototype; reproducible failures are useful evidence.

Use the [repository guide](docs/REPOSITORY_GUIDE.md) to locate components and the
[project plan](docs/PROJECT_PLAN.md) to choose a bounded work package. Link its
S0–S7, R1–R6 or [P0–P7 research-phase](docs/RESEARCH_PROGRAMME.md) identifier in the issue/PR, state the expected evidence, and assign
an owner when work begins. Update [project status](docs/PROJECT_STATUS.md) when a
published milestone changes the assessment; keep historical experiment reports dated.

1. Work on a focused branch and describe the acceptance criterion.
2. Keep simulator truth out of onboard localization, tracking, planning and goal
   selection. Visualization/scoring may use explicitly isolated diagnostics.
3. Run the checks appropriate to the changed component. Record exact live
   commands, seed, policy, scene and artifacts. Report failures as well as passes.
4. If changing patched sources under a submodule, review/export the patch set
   and verify pristine application before including it in the parent change.
5. Update maintained documentation and generated diagrams. Put research drafts
   in `archive/research/`, clearly marked historical, rather than `docs/`.
6. Stage only intended files. Keep local `.env`, build outputs, checkpoints,
   generated walk meshes and raw logs out of commits.

Do not weaken validators, redefine success, or switch to oracle input to make a
sensor-only experiment pass. Visual sessions are explicitly unscored. Avoid
changing locomotion limits as a side effect of visualization work.

## A focused contribution workflow

1. Start a topic branch from the intended base revision. Keep unrelated runtime,
   experiment and presentation changes in separate reviewable contributions.
2. Link an issue or phase and describe the observable acceptance criterion.
3. Run the relevant checks below and retain their output, including failures.
4. Update affected runbooks, generated assets and the **Unreleased** changelog
   for user-facing changes. Preserve historical experiment records.
5. Open a pull request using the template. Explain source/score/behavior changes
   and identify any evidence that needs a graphical or GPU environment.
6. Address review and merge only after applicable checks pass. A maintainer can
   request additional native evidence when the change affects physical behavior.

## Choose the right checks

| Change | Minimum useful checks |
| --- | --- |
| Markdown, menus or SVGs | `python3 scripts/check_docs.py`; `python3 scripts/generate_readme_graphics.py --check` |
| Research review | Appraisal/bibliography checks, PDF rebuild and PDF validation/report checks |
| Repository tooling or CI | Repository-quality tests, workflow syntax/lint and the applicable verification jobs |
| Python/ROS behavior | Relevant package regressions in the supported container; build consumers when interfaces change |
| Simulator source | Native build/CTest, patch export/check and pristine replay |
| Planner, perception or policy behavior | Above checks plus a declared native experiment and retained failed/invalid attempts |

Use [Development](docs/DEVELOPMENT.md) for exact environments and commands. The
workflow's path selection changes CI cost, not the evidence needed for a scientific
or behavioral claim.

Optional staged-file hooks:

```bash
python3 -m venv .venv-quality
.venv-quality/bin/python -m pip install -r tools/requirements-quality.txt
.venv-quality/bin/pre-commit install
```

## Review, conduct and licensing

The [code of conduct](CODE_OF_CONDUCT.md) applies to project collaboration.
Use [private security reporting](SECURITY.md#reporting-a-vulnerability) for
sensitive issues. `.github/CODEOWNERS` identifies the current repository owner;
add appointed reviewers as the team grows before enforcing independent approval.

The existing package/submodule licences remain authoritative; see
[LICENSES.md](LICENSES.md). A contribution must preserve source attribution and
must not imply a new blanket licence for third-party code or assets.
See [Release policy](docs/RELEASES.md) for artifacts and publication requirements.
