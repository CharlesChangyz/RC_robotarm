#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${REPO_ROOT}/rc_ruckig"

if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "[run_rc_arm_mujoco] missing workspace setup: ${WORKSPACE_DIR}/install/setup.bash"
  echo "[run_rc_arm_mujoco] please build first: cd ${WORKSPACE_DIR} && colcon build"
  exit 1
fi

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

HARDWARE_CONFIG_FILE="${HARDWARE_CONFIG_FILE:-${WORKSPACE_DIR}/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.mujoco.yaml}"
CONTROLLERS_FILE="${CONTROLLERS_FILE:-${WORKSPACE_DIR}/rc_arm_description/config/rc_arm_2/rc_arm_2_controllers.yaml}"
USE_RVIZ="${USE_RVIZ:-true}"
USE_TF_TARGET_BRIDGE="${USE_TF_TARGET_BRIDGE:-true}"
USE_TARGET_POSE_RUCKIG_EXECUTOR="${USE_TARGET_POSE_RUCKIG_EXECUTOR:-true}"

echo "[run_rc_arm_mujoco] workspace: ${WORKSPACE_DIR}"
echo "[run_rc_arm_mujoco] hardware_config_file=${HARDWARE_CONFIG_FILE}"
echo "[run_rc_arm_mujoco] controllers_file=${CONTROLLERS_FILE}"
echo "[run_rc_arm_mujoco] use_rviz=${USE_RVIZ}"
echo "[run_rc_arm_mujoco] make sure your MuJoCo side publishes JointState and consumes commands using the topics configured in ${HARDWARE_CONFIG_FILE}"
echo "[run_rc_arm_mujoco] payload and unloaded defaults are read directly from ${HARDWARE_CONFIG_FILE}"

exec ros2 launch rc_arm_ruckig_config rc_arm_2_robot.launch.py \
  hardware_config_file:="${HARDWARE_CONFIG_FILE}" \
  controllers_file:="${CONTROLLERS_FILE}" \
  use_rviz:="${USE_RVIZ}" \
  use_tf_target_bridge:="${USE_TF_TARGET_BRIDGE}" \
  use_target_pose_ruckig_executor:="${USE_TARGET_POSE_RUCKIG_EXECUTOR}" \
  "$@"
