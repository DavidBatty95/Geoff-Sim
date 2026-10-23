#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/workspace/ros2_ws}"
MUJOCO_SRC="${WORKSPACE}/src/unitree_mujoco"
MUJOCO_SIM="${MUJOCO_SRC}/simulate"
MUJOCO_BUILD="${MUJOCO_SIM}/build"

set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
  source "${WORKSPACE}/install/setup.bash"
fi
set -u

export LD_LIBRARY_PATH="/workspace/vendor/unitree_sdk2_install/lib:${LD_LIBRARY_PATH:-}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${WORKSPACE}/config/cyclonedds.xml}"

default_bin="${MUJOCO_BUILD}/unitree_mujoco"
if [ -z "${UNITREE_MUJOCO_BIN:-}" ]; then
  rebuild=0
  if [ ! -x "${default_bin}" ]; then
    rebuild=1
  elif [ "${MUJOCO_SIM}/CMakeLists.txt" -nt "${default_bin}" ]; then
    rebuild=1
  else
    while IFS= read -r source; do
      if [ "${source}" -nt "${default_bin}" ]; then
        rebuild=1
        break
      fi
    done < <(find "${MUJOCO_SIM}/src" -type f \( -name '*.cc' -o -name '*.cpp' -o -name '*.h' \))
  fi

  if [ "${rebuild}" = "1" ]; then
    echo "Unitree Mujoco sources changed; rebuilding the simulator before launch."
    /workspace/scripts/build_unitree_mujoco.sh
  fi
fi

candidate_bins=(
  "${UNITREE_MUJOCO_BIN:-}"
  "${default_bin}"
  "${MUJOCO_BUILD}/bin/unitree_mujoco"
)

for bin in "${candidate_bins[@]}"; do
  if [ -n "${bin}" ] && [ -x "${bin}" ]; then
    if [ -f "${MUJOCO_SIM}/config.yaml" ]; then
      cp "${MUJOCO_SIM}/config.yaml" "${MUJOCO_SIM}/config.yaml.bak"
      UNITREE_MUJOCO_CONFIG="${MUJOCO_SIM}/config.yaml" UNITREE_MUJOCO_INTERFACE="${UNITREE_MUJOCO_INTERFACE:-lo}" UNITREE_MUJOCO_DOMAIN_ID="${UNITREE_MUJOCO_DOMAIN_ID:-${ROS_DOMAIN_ID:-42}}" python3 - <<'PY'
import os
from pathlib import Path

path = Path(os.environ['UNITREE_MUJOCO_CONFIG'])
lines = path.read_text().splitlines()
scene = os.environ.get("UNITREE_MUJOCO_SCENE", "scene_29dof_extreme_lab_dex3.xml")
settings = {
    'robot': 'robot: "g1"  # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1"',
    'robot_scene': f'robot_scene: "{scene}" # Robot scene, /unitree_robots/[robot]/scene.xml',
    'domain_id': f'domain_id: {os.environ.get("UNITREE_MUJOCO_DOMAIN_ID", "42")}  # Domain id',
    'interface': f'interface: "{os.environ.get("UNITREE_MUJOCO_INTERFACE", "lo")}" # Interface',
    'use_joystick': 'use_joystick: 0 # Simulate Unitree WirelessController using a gamepad',
    'print_scene_information': 'print_scene_information: 0 # Runtime scene dump can trigger upstream simulator memory corruption',
    'enable_elastic_band': 'enable_elastic_band: 0 # Keep off by default; upstream G1 runtime is unstable with the band enabled here',
}

seen = set()
updated = []
for line in lines:
    key = line.split(':', 1)[0].strip() if ':' in line else ''
    if key in settings:
        updated.append(settings[key])
        seen.add(key)
    else:
        updated.append(line)

for key, value in settings.items():
    if key not in seen:
        updated.append(value)

path.write_text('\n'.join(updated) + '\n')
PY
    fi
    exec "${bin}"
  fi
done

echo "Could not find a Unitree Mujoco executable."
echo "Run /workspace/scripts/build_unitree_mujoco.sh, or set UNITREE_MUJOCO_BIN to the built simulator binary."
exit 1
