#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${REPO_ROOT}/rc_arm_stack"

if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "[run_rc_arm_mujoco_bridge] missing workspace setup: ${WORKSPACE_DIR}/install/setup.bash"
  echo "[run_rc_arm_mujoco_bridge] please build first: cd ${WORKSPACE_DIR} && colcon build"
  exit 1
fi

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

HARDWARE_CONFIG_FILE="${HARDWARE_CONFIG_FILE:-${WORKSPACE_DIR}/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.mujoco.yaml}"

echo "[run_rc_arm_mujoco_bridge] workspace: ${WORKSPACE_DIR}"
echo "[run_rc_arm_mujoco_bridge] hardware_config_file=${HARDWARE_CONFIG_FILE}"
echo "[run_rc_arm_mujoco_bridge] payload and unloaded defaults are read directly from ${HARDWARE_CONFIG_FILE}"

cd "${REPO_ROOT}"
exec python3 demo/rc_robotarm_demo.py --hardware-config-file "${HARDWARE_CONFIG_FILE}" "$@"
