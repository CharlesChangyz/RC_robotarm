#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${REPO_ROOT}/rc_moveit"

if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "[run_rc_arm_mujoco] missing workspace setup: ${WORKSPACE_DIR}/install/setup.bash"
  echo "[run_rc_arm_mujoco] please build first: cd ${WORKSPACE_DIR} && colcon build"
  exit 1
fi

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
source "${REPO_ROOT}/config/ros_domain.env"
set -u

HARDWARE_CONFIG_FILE="${HARDWARE_CONFIG_FILE:-${WORKSPACE_DIR}/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.mujoco.yaml}"
CONTROLLERS_FILE="${CONTROLLERS_FILE:-${WORKSPACE_DIR}/rc_arm_description/config/rc_arm_2/rc_arm_2_controllers.yaml}"
USE_RVIZ="${USE_RVIZ:-false}"
USE_TF_TARGET_BRIDGE="${USE_TF_TARGET_BRIDGE:-true}"
USE_DM_SERIAL_FRAME_BRIDGE="${USE_DM_SERIAL_FRAME_BRIDGE:-true}"
USE_TARGET_POSE_MOVEIT_EXECUTOR="${USE_TARGET_POSE_MOVEIT_EXECUTOR:-true}"
USE_ARM2_MIDDLEWARE="${USE_ARM2_MIDDLEWARE:-true}"
MIDDLEWARE_DM_SERIAL_BRIDGE_ENABLED="${MIDDLEWARE_DM_SERIAL_BRIDGE_ENABLED:-true}"
MIDDLEWARE_DM_SERIAL_ALLOWED_ACTION_SET_IDS="${MIDDLEWARE_DM_SERIAL_ALLOWED_ACTION_SET_IDS:-}"

echo "[run_rc_arm_mujoco] workspace: ${WORKSPACE_DIR}"
echo "[run_rc_arm_mujoco] ros_domain_id=${ROS_DOMAIN_ID}"
echo "[run_rc_arm_mujoco] hardware_config_file=${HARDWARE_CONFIG_FILE}"
echo "[run_rc_arm_mujoco] controllers_file=${CONTROLLERS_FILE}"
echo "[run_rc_arm_mujoco] use_rviz=${USE_RVIZ}"
echo "[run_rc_arm_mujoco] use_dm_serial_frame_bridge=${USE_DM_SERIAL_FRAME_BRIDGE}"
echo "[run_rc_arm_mujoco] use_arm2_middleware=${USE_ARM2_MIDDLEWARE}"
echo "[run_rc_arm_mujoco] middleware_dm_serial_bridge_enabled=${MIDDLEWARE_DM_SERIAL_BRIDGE_ENABLED}"
echo "[run_rc_arm_mujoco] middleware_dm_serial_allowed_action_set_ids=${MIDDLEWARE_DM_SERIAL_ALLOWED_ACTION_SET_IDS}"
echo "[run_rc_arm_mujoco] make sure your MuJoCo side publishes JointState and consumes commands using the topics configured in ${HARDWARE_CONFIG_FILE}"
echo "[run_rc_arm_mujoco] payload and unloaded defaults are read directly from ${HARDWARE_CONFIG_FILE}"

LAUNCH_ARGS=(
  "hardware_config_file:=${HARDWARE_CONFIG_FILE}"
  "controllers_file:=${CONTROLLERS_FILE}"
  "use_rviz:=${USE_RVIZ}"
  "use_tf_target_bridge:=${USE_TF_TARGET_BRIDGE}"
  "use_dm_serial_frame_bridge:=${USE_DM_SERIAL_FRAME_BRIDGE}"
  "use_target_pose_moveit_executor:=${USE_TARGET_POSE_MOVEIT_EXECUTOR}"
  "use_arm2_middleware:=${USE_ARM2_MIDDLEWARE}"
  "middleware_dm_serial_bridge_enabled:=${MIDDLEWARE_DM_SERIAL_BRIDGE_ENABLED}"
)

if [[ -n "${MIDDLEWARE_DM_SERIAL_ALLOWED_ACTION_SET_IDS}" ]]; then
  LAUNCH_ARGS+=("middleware_dm_serial_allowed_action_set_ids:=${MIDDLEWARE_DM_SERIAL_ALLOWED_ACTION_SET_IDS}")
fi

exec ros2 launch rc_arm_moveit_config rc_arm_2_robot.launch.py "${LAUNCH_ARGS[@]}" "$@"
