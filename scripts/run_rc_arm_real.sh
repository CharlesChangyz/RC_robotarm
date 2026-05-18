#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${REPO_ROOT}/rc_arm_stack"

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
USE_RVIZ="${USE_RVIZ:-false}"
USE_TF_TARGET_BRIDGE="${USE_TF_TARGET_BRIDGE:-true}"
USE_TARGET_POSE_EXECUTOR="${USE_TARGET_POSE_EXECUTOR:-true}"
JOINT_LIMITS_FILE="${JOINT_LIMITS_FILE:-${WORKSPACE_DIR}/rc_arm_motion_config/config/rc_arm_2/ruckig_joint_limits.yaml}"

echo "[run_rc_arm_real] workspace: ${WORKSPACE_DIR}"
echo "[run_rc_arm_real] hardware_config_file=${HARDWARE_CONFIG_FILE}"
echo "[run_rc_arm_real] controllers_file=${CONTROLLERS_FILE}"
echo "[run_rc_arm_real] use_rviz=${USE_RVIZ}"
echo "[run_rc_arm_real] payload and unloaded defaults are read directly from ${HARDWARE_CONFIG_FILE}"

exec ros2 launch rc_arm_motion_config rc_arm_2_robot.launch.py \
  hardware_config_file:="${HARDWARE_CONFIG_FILE}" \
  controllers_file:="${CONTROLLERS_FILE}" \
  joint_limits_file:="${JOINT_LIMITS_FILE}" \
  use_rviz:="${USE_RVIZ}" \
  use_tf_target_bridge:="${USE_TF_TARGET_BRIDGE}" \
  use_target_pose_executor:="${USE_TARGET_POSE_EXECUTOR}" \
  "$@"
