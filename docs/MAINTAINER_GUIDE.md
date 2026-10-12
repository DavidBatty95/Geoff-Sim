# Repository maintenance and GitHub settings

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

[Contributing](../CONTRIBUTING.md) · [Release policy](RELEASES.md) · [Development](DEVELOPMENT.md)

This guide connects the checked-in project workflow to the settings a GitHub
repository owner must configure. It is a setup record, not a claim that those
settings have already been enabled.

## Required verification

The `Verification` workflow exposes a stable **Verification gate** job. Use that
as the required status check for `main`; it aggregates the applicable jobs:

| Check | When it runs | Coverage |
| --- | --- | --- |
| Classify changes | Every workflow run | Fail-closed routing of runtime and review changes |
| Repository quality | Every run | Metadata, documentation/anchors, generated assets, bibliography, checked-in PDF and focused tooling tests |
| Review PDF rebuild | Review sources/tooling or workflow changes; manual runs | Checksum-pinned TeX engine, clean document build, semantic comparison and PDF artifacts |
| Build, test, and benchmark | Runtime/configuration/data/workflow changes; manual runs | Pinned sources/patch replay, Docker/ROS/MuJoCo build, tests and strict mock gate |
| Verification gate | Always, after the above jobs | Fails on failed/cancelled prerequisites or an unexpectedly skipped required job |

Documentation-only changes skip the ROS image/workspace build. Unknown files
default to full verification. Fork pull requests run without repository secrets;
private submodule access can therefore prevent runtime verification. Document that
failure and rerun a reviewed candidate in an authorized context rather than using
`pull_request_target` to execute untrusted code with credentials.

## Owner-controlled GitHub configuration

In **Settings → Rules → Rulesets** (or branch protection on older interfaces),
configure the default branch deliberately:

- Target `main`; require pull requests for routine changes.
- Require the **Verification gate** status check. Select its exact reported name
  after the new workflow has run once. Remove obsolete required job names when
  migrating settings, so a retired check does not block every PR.
- Block force-pushing and branch deletion.
- Require conversations to be resolved before merging.
- Require review/Code Owner approval once another appropriate reviewer is assigned.
  GitHub does not let an author approve their own PR, so a sole-maintainer project
  should appoint a reviewer before enforcing that requirement.
- Keep administrative bypass narrow and document exceptional use.
- If a merge queue is used, the workflow supports the `merge_group` event.

In **Settings → General**:

- Confirm the default branch and repository description/topics.
- Enable automatic deletion of merged head branches if it fits the team's workflow.
- Choose a merge strategy and keep commit messages descriptive and focused.
- Assign maintainers to the actual responsibilities in `.github/CODEOWNERS`.

In **Settings → Security / Code security** (names vary by GitHub plan):

- Enable the dependency graph and Dependabot alerts/security updates where available.
- Enable secret scanning and push protection where supported.
- Enable private vulnerability reporting and test the route from `SECURITY.md`.
- Consider code scanning for Python and C++; C++ analysis must use the ROS build
  environment rather than assume a generic autobuild works.

Feature availability depends on repository visibility and the owner's GitHub
plan. Record which features are actually enabled; do not describe an unavailable
service as an enforced repository control.

## Credentials and external sources

`SUBMODULE_TOKEN`, if needed, should be a narrowly scoped **read-only** credential
for the declared source repositories. The workflow does not persist checkout
credentials. Test access from a clean checkout and the intended contributor/CI
account; an owner's working clone can hide permission and cached-source problems.

SSH Git push access does not imply authenticated GitHub API administration. Owner
settings need the GitHub UI or an authenticated `gh` session with the appropriate
permissions. Keep tokens in GitHub secrets or a credential manager, not repository
files, issue reports or logs.

## Routine maintenance

| Trigger | Action |
| --- | --- |
| A contribution arrives | Check scope, linked phase/issue, evidence, documentation and reviewer ownership |
| Dependabot opens a PR | Inspect release notes; run the applicable verification before merging |
| Hook/tool pins need refreshing | Review new versions and checksums; `pre-commit autoupdate --freeze` preserves immutable hook pins |
| Source submodule changes | Review the diff, export the patch snapshot and verify pristine replay |
| A research result is published | Retain source/policy/data identity, update the dated report and claim ledger |
| A model/checkpoint is promoted | Apply the artifact/reproduction requirements in the release guide |
| Review text changes | Regenerate appraisals, rebuild the PDF and update its validation report |
| A release candidate is prepared | Run full workflow, clean-checkout reproduction and the release checklist |

The local pre-commit configuration provides fast staged-file checks. CI remains
the shared verification authority; installing a local hook is not proof it ran on
another contributor's machine.

## Decisions still requiring ownership

1. **Original-content licence:** `LICENSES.md` intentionally preserves existing
   terms and leaves a root licence decision open. Confirm the intended grant for
   original code/docs; preserve upstream notices and asset-specific terms.
2. **Artifact redistribution:** determine which policies and assets can be
   distributed, and who will host/support them.
3. **Review capacity:** name additional reviewers before requiring independent
   approvals or publishing a support policy.
4. **GitHub settings:** enable and verify the rules/security features above using
   an owner-authorized account.

Use [project status](PROJECT_STATUS.md) for capability claims and
[the changelog](../CHANGELOG.md) for user-facing publication history.
