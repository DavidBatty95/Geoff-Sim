#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export UNITREE_MUJOCO_DOMAIN_ID="${UNITREE_MUJOCO_DOMAIN_ID:-${ROS_DOMAIN_ID}}"

printf 'Starting Docker G1 simulation on ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID}"
printf 'Starting Unitree Mujoco DDS on domain_id=%s\n' "${UNITREE_MUJOCO_DOMAIN_ID}"

ARGS=()
for arg in "$@"; do
  case "${arg}" in
    policy_path:="${REPO_ROOT}"/*)
      ARGS+=("policy_path:=/workspace/${arg#policy_path:=${REPO_ROOT}/}")
      ;;
    *)
      ARGS+=("${arg}")
      ;;
  esac
done

RUN_OPTIONS=(--rm)
if [ -n "${GEOFF_NUMERIC_THREADS:-}" ]; then
  if [[ ! "${GEOFF_NUMERIC_THREADS}" =~ ^([1-9]|[12][0-9]|3[0-2])$ ]]; then
    echo "GEOFF_NUMERIC_THREADS must be an integer from 1 to 32" >&2
    exit 2
  fi
  for variable in OPENBLAS_NUM_THREADS OMP_NUM_THREADS MKL_NUM_THREADS NUMEXPR_NUM_THREADS; do
    RUN_OPTIONS+=(-e "${variable}=${GEOFF_NUMERIC_THREADS}")
  done
fi
if [ -n "${UNITREE_G1_CONTAINER_NAME:-}" ]; then
  RUN_OPTIONS+=(--name "${UNITREE_G1_CONTAINER_NAME}")
fi

docker compose run "${RUN_OPTIONS[@]}" \
  -e COLCON_INSTALL_BASE="${COLCON_INSTALL_BASE:-/workspace/ros2_ws/install}" \
  unitree-g1-sim \
  bash -lc 'exec /workspace/scripts/bringup_g1_ros2.sh "$@"' bash "${ARGS[@]}"
