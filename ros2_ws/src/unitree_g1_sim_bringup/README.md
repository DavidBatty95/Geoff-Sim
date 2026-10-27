# G1 simulation bringup

<!-- geoff-nav:start -->
<p align="center">
  <a href="../../../README.md"><img src="../../../docs/media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../../../README.md#quick-start"><img src="../../../docs/media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="../../../docs/DEMO_GUIDE.md"><img src="../../../docs/media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="../../../docs/README.md"><img src="../../../docs/media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="../../../docs/REPOSITORY_GUIDE.md"><img src="../../../docs/media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="../../../docs/PROJECT_STATUS.md"><img src="../../../docs/media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="../../../docs/PROJECT_PLAN.md"><img src="../../../docs/media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="../../../docs/RESULTS_REVIEW.md"><img src="../../../docs/media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="../../../docs/literature_review/README.md"><img src="../../../docs/media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="../../../docs/DEVELOPMENT.md"><img src="../../../docs/media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

[Workspace map](../README.md) · [Demo walkthrough](../../../docs/DEMO_GUIDE.md) · [Architecture](../../../docs/ARCHITECTURE.md)

Bringup connects the simulator, robot description, bridge, sensors, localization,
navigation and visualization into selectable launch configurations.

## Folder guide

| Path | Purpose |
| --- | --- |
| [`launch/`](launch/) | Launch descriptions; `g1_sim.launch.py` is the main composition point |
| [`config/`](config/) | Navigation, controller and social-navigation parameter profiles |
| [`rviz/`](rviz/) | Viewer layouts for onboard and diagnostic modes |
| [`test/`](test/) | Launch-description and manipulation-control contract checks |

The onboard profile is `config/social_navigation_onboard_v1.yaml`. The demo
catalogue in `scripts/demo_suite.py` supplies case-specific launch overrides;
the effective configuration is therefore the profile **plus launch arguments**.
Record both when comparing experiments.

## Recommended entry points

From the repository root:

```bash
./geoff demo plaza
./geoff demo clinic --people 4 --duration 300 --fast-start
./scripts/run_demo_suite.sh --list
```

For launch-argument inspection inside a sourced ROS container:

```bash
ros2 launch unitree_g1_sim_bringup g1_sim.launch.py --show-args
```

Use `--fast-start` only with current builds. `--headless` suppresses RViz; the
standalone MuJoCo viewer may still require the display environment. Only one
simulator and intended command source should own the runtime at once.

## Modes and boundaries

Current defaults select `hand_model:=dex3` and `grasp_backend:=dex3_contact`, with
Dex3 URDF and native scene variants. [Contact-only manipulation](../../../docs/DEX3_CONTACT_MANIPULATION_20261020.md)
documents the new object geometry and tested grasp/release path.

The native build also retains `scene_29dof_brainco_hardware.xml` for legacy articulated
hand/interface inspection. Use `scripts/run_hardware_parity_probe.py` to launch
that profile with a passive topic/CRC/sensor/TF audit. The
[fidelity guide](../../../docs/HARDWARE_FIDELITY.md) records supported fields and
nominal versus measured rates. `camera_mount_pitch_deg` must match the selected
model; the dual-transport launcher sets its explicit 25° mounting assumption.

- Onboard cases enable sensor-derived localization, online mapping and observed
  person tracking; they do not receive a static scene map or truth localization.
- Historical oracle cases intentionally use different sources and must stay
  labelled separately in reports.
- Arm composition, stationary holding and walking-policy authority require
  deliberate configuration; see the manipulation package guides before combining them.
- Timer-based node startup and wrapper-level readiness checks serve different
  roles. Reliable startup/reset orchestration remains part of the forward plan.

For verification use [development checks](../../../docs/DEVELOPMENT.md).
For planned lifecycle improvements see [S0/S1](../../../docs/PROJECT_PLAN.md).
