# Container environment

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

The supported runtime puts ROS 2 Humble, MuJoCo dependencies and project tooling
inside the `unitree-g1-sim` Compose service. The host provides Docker, display
access, graphics devices and the source checkout.

## Configuration map

| File | Role |
| --- | --- |
| [`Dockerfile`](Dockerfile) | Image build and runtime dependencies |
| [`requirements.txt`](requirements.txt) | Pinned direct Python runtime dependencies |
| [`scripts/`](scripts/) | Container-side setup/entrypoint helpers |
| [`../docker-compose.yml`](../docker-compose.yml) | Default service, source mounts, DDS and display settings |
| [`../docker-compose.gpu.yml`](../docker-compose.gpu.yml) | Optional NVIDIA configuration used by GPU workflows |
| [`../.env.example`](../.env.example) | Documented local environment defaults |

The default working directory is `/workspace/ros2_ws`. Sources and artifacts are
mounted under `/workspace`; the image is not a frozen copy of the current code.
Many tooling/documentation mounts are read-only, while ROS build and training
artifact locations are writable.

## Common commands

From the repository root after installation:

```bash
docker compose config --quiet
docker compose run --rm unitree-g1-sim /workspace/scripts/build_ws.sh
docker compose run --rm unitree-g1-sim /workspace/scripts/build_unitree_mujoco.sh
docker compose run --rm unitree-g1-sim /workspace/scripts/test_workspace_inside.sh fast
```

Use `./geoff` for ordinary demos. GPU training uses the
[MJLab wrapper](../docs/PAYLOAD_POLICY_TRAINING.md#reproduce). Host networking and
the shared DDS domain mean independent containers can still share robot topics;
follow the one-stack-at-a-time runbooks.

## Reproducibility

The ROS base image and direct Python dependencies are pinned. Apt/transitive
dependency resolution is not a bit-for-bit freeze. Experiment records should
retain the actual image ID, source/patch hashes, policy identity and effective
configuration. See [S0 of the project plan](../docs/PROJECT_PLAN.md).
