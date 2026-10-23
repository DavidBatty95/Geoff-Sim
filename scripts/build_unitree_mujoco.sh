#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/workspace/ros2_ws}"
SRC_DIR="${WORKSPACE}/src/unitree_mujoco"
SIM_DIR="${SRC_DIR}/simulate"
BUILD_DIR="${SIM_DIR}/build"
INSTALL_DIR="/workspace/vendor/unitree_mujoco_install"
SDK_INSTALL_DIR="/workspace/vendor/unitree_sdk2_install"
MUJOCO_VERSION="${MUJOCO_VERSION:-3.3.6}"
MUJOCO_DIR="/workspace/vendor/mujoco-${MUJOCO_VERSION}"
MUJOCO_ARCHIVE="/workspace/vendor/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz"
MUJOCO_URL="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz"

if [ ! -d "${SRC_DIR}" ]; then
  echo "Missing ${SRC_DIR}. Run /workspace/scripts/setup_sources.sh first."
  exit 1
fi

if [ ! -d "${WORKSPACE}/src/unitree_sdk2" ]; then
  echo "Missing ${WORKSPACE}/src/unitree_sdk2. Run /workspace/scripts/setup_sources.sh first."
  exit 1
fi

if [ ! -f "${SDK_INSTALL_DIR}/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake" ]; then
  cmake -S "${WORKSPACE}/src/unitree_sdk2" -B "/workspace/vendor/unitree_sdk2_build" \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_INSTALL_PREFIX="${SDK_INSTALL_DIR}"
  cmake --build "/workspace/vendor/unitree_sdk2_build" --parallel "$(nproc)"
  cmake --install "/workspace/vendor/unitree_sdk2_build"
fi

if [ -f "${SDK_INSTALL_DIR}/lib/libddsc.so" ] && [ ! -e "${SDK_INSTALL_DIR}/lib/libddsc.so.0" ]; then
  ln -s "libddsc.so" "${SDK_INSTALL_DIR}/lib/libddsc.so.0"
fi

if [ -f "${SDK_INSTALL_DIR}/lib/libddscxx.so" ] && [ ! -e "${SDK_INSTALL_DIR}/lib/libddscxx.so.0" ]; then
  ln -s "libddscxx.so" "${SDK_INSTALL_DIR}/lib/libddscxx.so.0"
fi

if [ ! -d "${MUJOCO_DIR}" ]; then
  curl -L "${MUJOCO_URL}" -o "${MUJOCO_ARCHIVE}"
  tar -xzf "${MUJOCO_ARCHIVE}" -C "/workspace/vendor"
fi

if [ ! -e "${SIM_DIR}/mujoco" ]; then
  ln -s "${MUJOCO_DIR}" "${SIM_DIR}/mujoco"
fi

python3 "${SRC_DIR}/tools/generate_natural_pedestrians.py" --robot-dir "${SRC_DIR}/unitree_robots/g1"
python3 /workspace/scripts/avoidance_worlds.py
python3 /workspace/scripts/setup_dex3_assets.py
python3 /workspace/scripts/transport_worlds.py --grasp-model legacy_magnetic
python3 /workspace/scripts/transport_worlds.py
python3 /workspace/scripts/brainco_hand_model.py

cmake -S "${SIM_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_PREFIX_PATH="${SDK_INSTALL_DIR}" \
  -DCMAKE_BUILD_RPATH="${SDK_INSTALL_DIR}/lib" \
  -DCMAKE_INSTALL_RPATH="${SDK_INSTALL_DIR}/lib" \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}"

cmake --build "${BUILD_DIR}" --parallel "$(nproc)"

if cmake --install "${BUILD_DIR}"; then
  echo "Installed Unitree Mujoco to ${INSTALL_DIR}"
else
  echo "Install target unavailable; build output remains in ${BUILD_DIR}"
fi
