#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/workspace/ros2_ws}"
REPO_ROOT="${REPO_ROOT:-/workspace}"
MODE="${1:-fast}"
INSTALL_BASE="${COLCON_INSTALL_BASE:-${WORKSPACE}/install}"

case "${MODE}" in
  fast|precommit|smoke|ci)
    ;;
  *)
    echo "Usage: $0 [fast|precommit|smoke|ci]"
    exit 2
    ;;
esac

set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u

if [ "${MODE}" = "ci" ]; then
  export COLCON_BUILD_BASE="${COLCON_BUILD_BASE:-${WORKSPACE}/.verification/build}"
  export COLCON_INSTALL_BASE="${COLCON_INSTALL_BASE:-${WORKSPACE}/.verification/install}"
  export COLCON_LOG_BASE="${COLCON_LOG_BASE:-${WORKSPACE}/.verification/log}"
  INSTALL_BASE="${COLCON_INSTALL_BASE}"
  echo "[test] ROS workspace build"
  "${REPO_ROOT}/scripts/build_ws.sh"
  "${REPO_ROOT}/scripts/build_unitree_mujoco.sh"
  ctest --test-dir "${WORKSPACE}/src/unitree_mujoco/simulate/build" --output-on-failure
fi

set +u
if [ -f "${INSTALL_BASE}/setup.bash" ]; then
  source "${INSTALL_BASE}/setup.bash"
fi
set -u

cd "${REPO_ROOT}"

echo "[test] Python syntax for local ROS packages"
PYTHONPYCACHEPREFIX=/tmp/unitree_g1_pycache python3 -m compileall -q \
  "${REPO_ROOT}/scripts" \
  "${WORKSPACE}/src/dynamic_obstacle_benchmarks/dynamic_obstacle_benchmarks" \
  "${WORKSPACE}/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge" \
  "${WORKSPACE}/src/unitree_g1_sim_bringup/launch"

echo "[test] Optional Grove-G1 source profile"
python3 "${REPO_ROOT}/integrations/grove_g1/test/test_profile.py"

echo "[test] XML/package/scene files parse"
python3 - <<'PY'
import pathlib
import xml.etree.ElementTree as ET

root = pathlib.Path('/workspace/ros2_ws')
paths = [
    root / 'src/g1_locomotion/package.xml',
    root / 'src/g1_msgs/package.xml',
    root / 'src/unitree_g1_description/package.xml',
    root / 'src/unitree_g1_mujoco_bridge/package.xml',
    root / 'src/unitree_g1_sim_bringup/package.xml',
    root / 'src/dynamic_obstacle_benchmarks/package.xml',
    root / 'src/unitree_social_navigation_msgs/package.xml',
    root / 'src/unitree_social_costmap/package.xml',
    root / 'src/unitree_g1_description/urdf/g1_29dof_mode_15_brainco_hand.urdf',
]
paths.extend(
    sorted((root / 'src/unitree_mujoco/unitree_robots/g1').glob('scene_29dof*.xml'))
)

missing = [str(path) for path in paths if not path.exists()]
if missing:
    raise SystemExit('Missing required files:\n' + '\n'.join(missing))

for path in paths:
    ET.parse(path)
    print(f'  parsed {path.relative_to(root)}')
PY

echo "[test] Robot description expands with xacro"
xacro "${WORKSPACE}/src/unitree_g1_description/urdf/g1_29dof_mode_15_brainco_hand.urdf" >/tmp/g1_robot_description.urdf

echo "[test] Required sim assets exist"
test -f "${WORKSPACE}/src/unitree_g1_description/meshes/pelvis.STL"
test -f "${WORKSPACE}/src/unitree_g1_sim_bringup/rviz/g1_sim.rviz"
test -f "${WORKSPACE}/config/cyclonedds.xml"

echo "[test] ROS package launch arguments are loadable"
if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix unitree_g1_sim_bringup >/dev/null 2>&1; then
  ros2 launch unitree_g1_sim_bringup g1_sim.launch.py --show-args >/tmp/g1_launch_args.txt
  grep -q "mujoco_scene" /tmp/g1_launch_args.txt
else
  echo "[test] Skipping launch argument check because workspace is not built"
fi

if [ "${MODE}" != "fast" ]; then
  echo "[test] Host-side benchmark orchestration tests"
  python3 -m pytest \
    "${REPO_ROOT}/scripts/test_avoidance_dashboard.py" \
    "${REPO_ROOT}/scripts/test_avoidance_worlds.py" \
    "${REPO_ROOT}/scripts/test_benchmark_batch.py" \
    "${REPO_ROOT}/scripts/test_benchmark_aggregation.py" \
    "${REPO_ROOT}/scripts/test_live_benchmark_output.py" \
    "${REPO_ROOT}/scripts/test_fixed_command_measurement.py" \
    "${REPO_ROOT}/scripts/test_g1_response_extraction.py" \
    "${REPO_ROOT}/scripts/test_g1_response_fitting_spec.py" \
    "${REPO_ROOT}/scripts/test_process_g1_response_trial.py" \
    "${REPO_ROOT}/scripts/test_benchmark_validator.py" \
    "${REPO_ROOT}/scripts/test_demo_suite.py" \
    "${REPO_ROOT}/scripts/test_readme_navigation.py" \
    "${REPO_ROOT}/scripts/test_transport_evidence.py" \
    "${REPO_ROOT}/scripts/test_transport_progress.py" \
    "${REPO_ROOT}/scripts/test_response_trace.py" \
    "${REPO_ROOT}/scripts/test_performance_evidence.py" \
    "${REPO_ROOT}/scripts/test_performance_matrix.py" \
    "${REPO_ROOT}/scripts/test_numeric_threads.py" \
    "${REPO_ROOT}/scripts/test_transport_capture.py" \
    "${REPO_ROOT}/scripts/test_arm_control_trace.py" \
    "${REPO_ROOT}/scripts/test_native_stopping.py" \
    "${REPO_ROOT}/scripts/test_native_stop_matrix.py" \
    "${REPO_ROOT}/scripts/test_checkpoint_stop_branches.py" \
    "${REPO_ROOT}/scripts/test_native_targets.py" \
    "${REPO_ROOT}/scripts/test_transport_worlds.py" \
    "${REPO_ROOT}/scripts/test_transport_people_profile.py" \
    "${REPO_ROOT}/scripts/test_search_evidence.py" \
    "${REPO_ROOT}/scripts/test_transport_objectives.py" \
    "${REPO_ROOT}/scripts/test_transport_crowd.py" \
    "${REPO_ROOT}/scripts/test_review_transport_failures.py" \
    "${REPO_ROOT}/scripts/test_policy_command_replay.py" \
    "${REPO_ROOT}/scripts/test_evaluation_audit.py" \
    "${REPO_ROOT}/scripts/test_mjlab_training_manifest.py"

  echo "[test] ROS package unit and integration tests"
  cd "${WORKSPACE}"
  colcon --log-base "${COLCON_LOG_BASE:-${WORKSPACE}/log}" test \
    --build-base "${COLCON_BUILD_BASE:-${WORKSPACE}/build}" \
    --install-base "${INSTALL_BASE}" \
    --merge-install \
    --return-code-on-test-failure \
    --python-testing pytest \
    --packages-select \
      dynamic_obstacle_benchmarks \
      g1_locomotion \
      g1_msgs \
      unitree_g1_description \
      unitree_g1_mujoco_bridge \
      unitree_g1_sim_bringup \
      unitree_social_navigation_msgs \
      unitree_social_costmap \
    --event-handlers console_direct+
  colcon test-result \
    --test-result-base "${COLCON_BUILD_BASE:-${WORKSPACE}/build}" \
    --verbose

  echo "[test] Deterministic person-avoidance regression benchmark"
  benchmark_output="/tmp/person_avoidance_smoke.json"
  rm -f "${benchmark_output}" \
    "/tmp/person_avoidance_smoke_goals.csv" \
    "/tmp/person_avoidance_smoke_events.csv" \
    "/tmp/person_avoidance_smoke_scenario.yaml"
  ros2 run dynamic_obstacle_benchmarks person_avoidance_benchmark \
    --config "${WORKSPACE}/src/dynamic_obstacle_benchmarks/config/person_avoidance_benchmark.yaml" \
    --mode smoke \
    --backend mock \
    --seed 1 \
    --headless \
    --output "${benchmark_output}"
  python3 "${REPO_ROOT}/scripts/validate_benchmark_result.py" \
    "${benchmark_output}" \
    --minimum-score "${BENCHMARK_MINIMUM_SCORE:-95.0}"
  cd "${REPO_ROOT}"
fi

echo "[test] MJ Lab helper is usable when dependencies are installed"
if [ -d "/workspace/vendor/mjlab/.venv" ]; then
  /workspace/scripts/run_mjlab.sh list-envs >/tmp/mjlab_envs.txt || true
  grep -q "Mjlab-Velocity-Flat-Unitree-G1" /tmp/mjlab_envs.txt
else
  echo "[test] Skipping MJ Lab check because /workspace/vendor/mjlab/.venv is missing"
fi

if [ "${MODE}" = "smoke" ]; then
  echo "[test] GPU visibility, if NVIDIA runtime is available"
  if [ -d "/workspace/vendor/mjlab/.venv" ]; then
    /workspace/scripts/run_mjlab.sh python - <<'PY'
import torch

print(f'torch_cuda_available={torch.cuda.is_available()}')
print(f'torch_cuda_device_count={torch.cuda.device_count()}')
if torch.cuda.is_available():
    print(f'torch_cuda_device_name={torch.cuda.get_device_name(0)}')
PY
  else
    echo "[test] Skipping Torch CUDA check because /workspace/vendor/mjlab/.venv is missing"
  fi
fi

echo "[test] Workspace checks passed (${MODE})"
