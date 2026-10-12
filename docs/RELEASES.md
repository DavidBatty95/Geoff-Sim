# Releases, versions and reproducible artifacts

<!-- geoff-nav:start -->
<p align="center">
  <a href="../README.md"><img src="media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../README.md#quick-start"><img src="media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="DEMO_GUIDE.md"><img src="media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="INDEX.md"><img src="media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="REPOSITORY_GUIDE.md"><img src="media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="PROJECT_STATUS.md"><img src="media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="PROJECT_PLAN.md"><img src="media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="RESULTS_REVIEW.md"><img src="media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="literature_review/README.md"><img src="media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="DEVELOPMENT.md"><img src="media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

[Changelog](../CHANGELOG.md) · [Maintainer guide](MAINTAINER_GUIDE.md) · [Project status](PROJECT_STATUS.md)

Geoff-Sim currently publishes development snapshots. A Git commit, CI pass and
isolated native demonstration have different meanings from a qualified release.
This page defines the process for creating a release when the owner chooses to
promote a candidate; it does not create a tag or claim that one already exists.

## Versioning policy

- Use annotated `vMAJOR.MINOR.PATCH` tags for numbered releases.
- Use a prerelease suffix, such as `v0.1.0-rc.1`, for a release candidate. The
  example is a naming convention, not an existing release.
- During `0.x`, document interface/configuration changes explicitly; experimental
  behavior can change between minor versions.
- Patch releases should identify the fixes and any changed evidence. A new result
  dataset must retain its own provenance even when software versioning is unchanged.
- Keep development entries under **Unreleased** in `CHANGELOG.md`. On release,
  add the real version/date and update `CITATION.cff` accordingly.
- Never move an already published tag to hide a different implementation or failed
  result. Publish a corrective release and explain the change.

## Release evidence checklist

1. **Scope:** define the supported host, ROS distribution, worlds, policies,
   operating conditions and which demos are qualified versus experimental.
2. **Identity:** freeze the parent revision, submodule pins, reviewed patches,
   dependency/image identity, launch parameters and model/policy hashes.
3. **Clean reproduction:** clone into a new location/account without local build
   caches or training outputs. Verify source access, installation and the documented
   commands on the intended environment.
4. **Verification:** run the full `Verification` workflow, including the strict
   mock gate and applicable native checks. Include failed/invalid native attempts
   in the evidence rather than substituting CI for behavioral acceptance.
5. **Documentation:** make README, package guides, changelog, known limitations
   and research/evaluation claims match the frozen candidate.
6. **Licensing:** resolve the original-content licence and redistribution terms
   for each included asset. Existing package/upstream licences remain authoritative.
7. **Artifacts:** provide reproducible model access or a complete training route,
   checksums, source manifests, evidence summaries and instructions for verification.
8. **Publication:** have the owner create the annotated tag and GitHub release,
   attach the reviewed artifacts, and test every release download/link.

## Model and policy artifact contract

Policies remain outside ordinary source commits. A release using a model should
either distribute an authorized immutable artifact or document how to reproduce
it from accessible inputs. Include:

| Field | Required information |
| --- | --- |
| Identity | Filename, size, SHA-256 and immutable release/download location |
| Origin | Upstream project or training source revision and applicable terms |
| Interface | Observation/action dimensions, joint ordering, history/reset semantics and expected gains |
| Training | Initialization, command/posture/load distribution, seeds, iterations and environment/tool versions |
| Deployment | Compatible source/configuration, required sensors and exact launch command |
| Evaluation | Validated conditions, complete attempted-run ledger and known regressions |
| Scope | Whether evidence is mock, simulation, fixture-only, crowd, or physical hardware |

The latest local transport candidate is documented in
[the native payload report](NATIVE_PAYLOAD_CONTRACT.md). A checkpoint path under
`vendor/mjlab/logs/` describes a local artifact; it is not a public download. Until
an authorized release asset is available, readers need the specified checkpoint
or its full training/reproduction workflow. Do not imply that cloning the code
also downloads local training output.

## Suggested release contents

- Source tag and a manifest linking pins, patches and configuration.
- Release notes explaining changes, supported commands and limitations.
- Portable result JSON/CSV and links to the relevant experiment reports.
- Authorized policy/model artifacts plus a checksum file, if the release depends on them.
- The reviewed literature PDF when appropriate, with its source and validation records.
- A clean-reproduction record identifying the host/image and the commands exercised.

Large logs, recordings and authorized model binaries belong in release/artifact
storage rather than repeated Git-history blobs. Keep essential small README
previews and the reviewed PDF in the repository where they support browsing.

## Cite a development snapshot or release

For a research release, consider depositing the reviewed source/artifact snapshot
in Zenodo or an institutional archive that issues persistent identifiers. Record
the actual minted DOI, version and release date in `CITATION.cff` after deposit.
Keep a software-release DOI distinct from the DOI of a paper or dataset, and
confirm the deposit's redistribution terms against the licence inventory.

GitHub reads [`CITATION.cff`](../CITATION.cff) to offer a **Cite this repository**
entry. For reproducible research, also identify the exact commit/tag, policy hash,
dataset and experiment protocol. Cite relevant upstream methods separately.
A generated software citation does not replace the literature review's paper
bibliography or the provenance of a specific experiment.
