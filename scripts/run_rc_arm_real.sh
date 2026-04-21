#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${REPO_ROOT}/rc_moveit"

if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "[run_rc_arm_real] missing workspace setup: ${WORKSPACE_DIR}/install/setup.bash"
  echo "[run_rc_arm_real] please build first: cd ${WORKSPACE_DIR} && colcon build"
  exit 1
fi

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

HARDWARE_CONFIG_FILE="${HARDWARE_CONFIG_FILE:-${WORKSPACE_DIR}/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.real.yaml}"
CONTROLLERS_FILE="${CONTROLLERS_FILE:-${WORKSPACE_DIR}/rc_arm_description/config/rc_arm_2/rc_arm_2_controllers.yaml}"
USE_RVIZ="${USE_RVIZ:-true}"
USE_TF_TARGET_BRIDGE="${USE_TF_TARGET_BRIDGE:-true}"
USE_TARGET_POSE_MOVEIT_EXECUTOR="${USE_TARGET_POSE_MOVEIT_EXECUTOR:-true}"

echo "[run_rc_arm_real] workspace: ${WORKSPACE_DIR}"
echo "[run_rc_arm_real] hardware_config_file=${HARDWARE_CONFIG_FILE}"
echo "[run_rc_arm_real] controllers_file=${CONTROLLERS_FILE}"

exec ros2 launch rc_arm_moveit_config rc_arm_2_robot.launch.py \
  hardware_config_file:="${HARDWARE_CONFIG_FILE}" \
  controllers_file:="${CONTROLLERS_FILE}" \
  use_rviz:="${USE_RVIZ}" \
  use_tf_target_bridge:="${USE_TF_TARGET_BRIDGE}" \
  use_target_pose_moveit_executor:="${USE_TARGET_POSE_MOVEIT_EXECUTOR}" \
  "$@"
