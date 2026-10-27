# Security and deployment scope

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

The supported target is the documented local MuJoCo simulation environment.
No hardware deployment or independent safety certification is claimed.

- Keep the simulation DDS interface on the configured local interface/domain.
  Do not connect tutorial command publishers to a live robot network.
- `.env`, credentials and private checkpoints are local files. Do not attach
  them to public issues or logs.
- The development container uses host networking/IPC for ROS integration and
  explicitly mapped graphics devices. It does not require `privileged: true`.
  Review host mounts and X11 access before using shared machines.

Only the current maintained branch is supported. Third-party vulnerabilities
should also be reported to the corresponding upstream project.

## Reporting a vulnerability

1. If GitHub offers **Security → Report a vulnerability**, use the
   [private advisory form](https://github.com/DavidBatty95/Geoff-Sim/security/advisories/new).
   Availability depends on the repository's owner-controlled settings.
2. If that route is unavailable, use the maintainer's contact link in the
   [project profile](profile/README.md) to request a confidential reporting channel.
3. Include the affected commit, component, prerequisites, impact and a minimal
   reproduction. Share credentials, exploit details and sensitive logs only through
   the agreed private channel.

Ordinary installation or simulation failures can use the public bug form.
Sensitive vulnerabilities should not be reported through public issues.
Maintainers should agree on disclosure and remediation with the reporter and
notify affected upstream projects when relevant.

## Maintainer controls

The [maintainer guide](docs/MAINTAINER_GUIDE.md) lists the settings needed for
private reporting, protected branches, required checks and dependency/security
alerts. Checked-in workflow configuration alone does not enable these GitHub
administrative features.
