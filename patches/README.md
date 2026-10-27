# Reviewed source patches

<!-- geoff-nav:start -->
<p align="center">
  <a href="../README.md"><img src="../docs/media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../README.md#quick-start"><img src="../docs/media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="../docs/DEMO_GUIDE.md"><img src="../docs/media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="../docs/README.md"><img src="../docs/media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="../docs/REPOSITORY_GUIDE.md"><img src="../docs/media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="../docs/PROJECT_STATUS.md"><img src="../docs/media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="../docs/PROJECT_PLAN.md"><img src="../docs/media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="../docs/RESULTS_REVIEW.md"><img src="../docs/media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="../docs/literature_review/README.md"><img src="../docs/media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="../docs/DEVELOPMENT.md"><img src="../docs/media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

The parent repository pins upstream/fork revisions through Git submodules.
These patches carry the local simulator and MJLab modifications needed by the
current workspace, including new files that a parent-repository commit alone
would otherwise omit. Generated walk meshes and machine-specific
`simulate/config.yaml` changes are intentionally excluded.

`manifest.json` records each base commit, patch checksum and resulting source
file checksum. `scripts/setup_sources.sh` applies them after source checkout.

## Snapshot contents

| File | Purpose |
| --- | --- |
| [`manifest.json`](manifest.json) | Exact upstream bases and checksums for every exported source file |
| [`unitree_mujoco.patch`](unitree_mujoco.patch) | Simulator changes: sensors, actors, viewer overlays, manipulation and native tests |
| [`mjlab.patch`](mjlab.patch) | Training-environment changes on top of the pinned MJLab source |

## Verify or apply

Run from the repository root:

```bash
python3 scripts/source_patches.py apply             # Fresh checkout; idempotent
python3 scripts/source_patches.py check             # Detect snapshot drift
python3 scripts/source_patches.py verify-pristine   # Reproduce files from pinned base
```

## Update a snapshot

After intentionally changing patched sources, review the submodule diff, run its
tests, then regenerate and commit the patch snapshot with the parent changes:

```bash
python3 scripts/source_patches.py export
python3 scripts/source_patches.py verify-pristine
```

The exporter uses an explicit source allowlist. Add new source files to it when
needed. It never commits, stages, resets or discards working-tree changes.
Application fails on a conflicting checkout rather than overwriting local work.
When updating submodule base revisions, review/rebase the patch set explicitly.

After application, submodules can appear dirty by design: the pinned base plus
the published patch is the maintained source. A clean parent status is not a
substitute for `check` and `verify-pristine`. The October publication refreshed
the simulator snapshot to include the BrainCo helper and changed manipulation
source/tests. See [publication verification](../docs/DEVELOPMENT.md#publication-verification--5-october-2026)
and the [S0 reproducibility plan](../docs/PROJECT_PLAN.md).
