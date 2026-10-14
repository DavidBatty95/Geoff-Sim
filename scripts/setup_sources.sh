#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/workspace/ros2_ws}"

mkdir -p "${WORKSPACE}/src" "/workspace/vendor"

if [ ! -f "${WORKSPACE}/unitree.repos" ]; then
  echo "Missing ${WORKSPACE}/unitree.repos"
  exit 1
fi

unitree_repositories=(unitree_mujoco unitree_ros2 unitree_sdk2)
existing_repositories=0
for repository in "${unitree_repositories[@]}"; do
  if git -C "${WORKSPACE}/src/${repository}" rev-parse --git-dir >/dev/null 2>&1; then
    existing_repositories=$((existing_repositories + 1))
  fi
done

if [ "${existing_repositories}" -eq 0 ]; then
  vcs import "${WORKSPACE}/src" < "${WORKSPACE}/unitree.repos"
elif [ "${existing_repositories}" -ne "${#unitree_repositories[@]}" ]; then
  echo "Unitree sources are partially populated. Run git submodule update --init --recursive from the repository root." >&2
  exit 1
fi

if ! git -C "/workspace/vendor/mjlab" rev-parse --git-dir >/dev/null 2>&1; then
  git clone https://github.com/DavidBatty95/Geoff-Sim-mjlab.git "/workspace/vendor/mjlab"
  mjlab_revision="$(python3 -c 'import json; print(next(s["base_commit"] for s in json.load(open("/workspace/patches/manifest.json"))["sources"] if s["path"] == "vendor/mjlab"))')"
  git -C "/workspace/vendor/mjlab" checkout --detach "${mjlab_revision}"
fi

python3 /workspace/scripts/source_patches.py apply

for ignore_dir in \
  "${WORKSPACE}/src/unitree_mujoco/example" \
  "${WORKSPACE}/src/unitree_mujoco/simulate" \
  "${WORKSPACE}/src/unitree_mujoco/simulate_python" \
  "${WORKSPACE}/src/unitree_sdk2"; do
  if [ -d "${ignore_dir}" ]; then
    touch "${ignore_dir}/COLCON_IGNORE"
  fi
done

rosdep update || true
rosdep install \
  --from-paths "${WORKSPACE}/src" \
  --ignore-src \
  --rosdistro "${ROS_DISTRO:-humble}" \
  -y \
  --skip-keys "unitree_sdk2 ament_python"

echo "Sources are available in ${WORKSPACE}/src"
