# Development and verification

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
  <a href="DEVELOPMENT.md"><img src="media/navigation/develop-current.svg" width="132" height="44" alt="Develop (current page)"></a>
</p>
<!-- geoff-nav:end -->

Use the Docker ROS Humble environment for ROS tests. Host-only tooling such as
the results GUI, world generator and source-patch manager uses Python 3.

## Fast repository-quality checks

Use an isolated host environment for documentation and metadata tooling:

```bash
python3 -m venv .venv-quality
.venv-quality/bin/python -m pip install -r tools/requirements-quality.txt
.venv-quality/bin/python scripts/check_repository.py
.venv-quality/bin/cffconvert --infile CITATION.cff --validate
.venv-quality/bin/python scripts/check_docs.py
.venv-quality/bin/python scripts/generate_readme_graphics.py --check
bash docs/literature_review/build.sh --check-only
.venv-quality/bin/python docs/literature_review/verify_pdf.py --check-report
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv-quality/bin/python -m pytest -q \
  scripts/test_readme_navigation.py scripts/test_check_docs.py \
  scripts/test_repository_quality.py scripts/test_pdf_provenance.py
```

If a ROS shell has exported `PYTHONPATH`, clear it for these isolated commands
to avoid loading unrelated ROS Python packages. In a documentation-only clone
without initialized submodules, `check_docs.py --allow-uninitialized-submodules`
reports deferred upstream links; ordinary project links remain required.
The native CI job checks those upstream targets after recursive checkout.

The workflow runs lightweight repository checks on every change, rebuilds the
review PDF when its sources/tooling change, and runs full native verification for
runtime, configuration, result-data, workflow or unknown changes. The stable
**Verification gate** checks the applicable job outcomes. Manual runs exercise
all jobs. See [the maintainer guide](MAINTAINER_GUIDE.md) before configuring branch rules.

## ROS and simulator checks

```bash
python3 scripts/check_docs.py
python3 scripts/generate_readme_graphics.py --check
python3 scripts/source_patches.py check
python3 scripts/source_patches.py verify-pristine
python3 -m pytest scripts/test_avoidance_dashboard.py scripts/test_avoidance_worlds.py
docker compose run --rm unitree-g1-sim /workspace/scripts/test_workspace_inside.sh fast
```

After changing C++ simulator code:

```bash
docker compose run --rm unitree-g1-sim /workspace/scripts/build_unitree_mujoco.sh
docker compose run --rm unitree-g1-sim ctest --test-dir /workspace/ros2_ws/src/unitree_mujoco/simulate/build --output-on-failure
python3 scripts/source_patches.py export
python3 scripts/source_patches.py verify-pristine
```

Do not run Compose tests concurrently with a live matrix: benchmark cleanup uses
the simulator service's one-off container names. The full `ci` test mode builds
the ROS workspace and runs package tests plus a strict mock benchmark gate.
That benchmark's acceptance failure must remain visible.

Patched submodules intentionally appear dirty. Their pinned gitlinks plus
`patches/manifest.json` and the reviewed patches reproduce the maintained sources;
machine-specific `simulate/config.yaml` changes are not exported. Do not commit
generated meshes, build trees, local environments, policies or raw logs.

The Docker image copies only `docker/` runtime files; sources are mounted at
runtime. Direct Python runtime packages are pinned in `docker/requirements.txt`,
and the verified ROS base image is pinned by digest. Apt repositories and
transitive Python dependencies are not bit-for-bit frozen. Cohort manifests
record the actual Docker image ID used for new runs.

The optional real-browser check uses installed Firefox/geckodriver:

```bash
./geoff results
# In another terminal:
python3 scripts/check_dashboard_browser.py
```

It verifies dataset selection, filtering, full run details and recorded-trajectory
replay. Torch-specific training tests use the MJLab environment rather than the
ROS Python interpreter; disable ROS pytest plugin autoload for that isolated test.

## Review PDF integrity

A successful review build writes `build_provenance.json` linking the published
PDF to its exact TeX/BibTeX and declared data/build inputs. PDF verification
checks citations, source URLs, bookmarks, internal links and the saved report.
If review inputs change, rebuild before updating the report:

```bash
python3 docs/literature_review/generate_review_appendix.py
python3 scripts/install_quality_tool.py tectonic --directory .venv-quality/bin
TECTONIC="$PWD/.venv-quality/bin/tectonic" bash docs/literature_review/build.sh
.venv-quality/bin/python docs/literature_review/verify_pdf.py --write-report
.venv-quality/bin/python docs/literature_review/verify_pdf.py --check-report
```

The review CI job uses a checksum-pinned Tectonic binary and compares rebuilt
text, page/bookmark structure and link signatures with the published report.
Creation timestamps can differ; scientific content and navigation must match.
The existing `TECTONIC` override remains available for local builds.

## Publication verification — 5 October 2026

For the newer 6 October verification, see [hardware fidelity](HARDWARE_FIDELITY.md):
full CI passed (15 packages, 624 ROS test results, 99 script tests, 11 simulator
CTests and the strict mock gate); 13 isolated MJLab tests also passed. Two new
native dual-delivery trials and an articulated-hand interface audit passed.
The table below remains the earlier publication snapshot.

The source publication was checked with the existing Docker environment:

| Check | Result |
| --- | --- |
| ROS workspace build, `.verification` overlay | All 15 packages built |
| MuJoCo build and CTest | Build succeeded; 10/10 tests passed |
| Bridge tests | 333 passed |
| Benchmark package tests | 83 passed |
| Bringup tests | 18 passed |
| General script tests in ROS environment | 79 passed; 3 payload tests skipped for missing training dependencies |
| Payload and stability-schedule tests in MJLab | All 7 passed, including the 3 payload tests skipped above |
| Source patch snapshot and pristine replay | Both simulator and MJLab snapshots reproduced |

Run package test directories in separate pytest invocations: their shared `test`
package names collide when collected together by ROS launch-testing. Run
`scripts/test_g1_stability_schedule.py` in MJLab rather than the ROS interpreter.
For the isolated training tests:

```bash
docker compose run --rm -e PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 unitree-g1-sim \
  /workspace/scripts/run_mjlab.sh --no-sync python -m pytest -q \
  /workspace/scripts/test_g1_payload_training.py \
  /workspace/scripts/test_g1_stability_schedule.py
```

This verifies builds and regression coverage. No new live navigation matrix,
transport mission or hardware acceptance run was collected for this publication.
