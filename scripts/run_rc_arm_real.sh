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
source "${REPO_ROOT}/config/ros_domain.env"
set -u

HARDWARE_CONFIG_FILE="${HARDWARE_CONFIG_FILE:-${WORKSPACE_DIR}/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.real.yaml}"
CONTROLLERS_FILE="${CONTROLLERS_FILE:-${WORKSPACE_DIR}/rc_arm_description/config/rc_arm_2/rc_arm_2_controllers.yaml}"
USE_RVIZ="${USE_RVIZ:-false}"
USE_TF_TARGET_BRIDGE="${USE_TF_TARGET_BRIDGE:-true}"
USE_DM_SERIAL_FRAME_BRIDGE="${USE_DM_SERIAL_FRAME_BRIDGE:-false}"
USE_TARGET_POSE_MOVEIT_EXECUTOR="${USE_TARGET_POSE_MOVEIT_EXECUTOR:-true}"
USE_ARM2_MIDDLEWARE="${USE_ARM2_MIDDLEWARE:-true}"
USE_CAMERA_TARGET_POINT_BRIDGE="${USE_CAMERA_TARGET_POINT_BRIDGE:-true}"
MIDDLEWARE_TARGET_POINT_TOPIC="${MIDDLEWARE_TARGET_POINT_TOPIC:-/arm2/middleware/target_point}"
CAMERA_TARGET_POINT_INPUT_TOPIC="${CAMERA_TARGET_POINT_INPUT_TOPIC:-/arm2/camera_raw_dat}"
CAMERA_TARGET_POINT_OUTPUT_TOPIC="${CAMERA_TARGET_POINT_OUTPUT_TOPIC:-${MIDDLEWARE_TARGET_POINT_TOPIC}}"
CAMERA_TARGET_POINT_SOURCE_FRAME="${CAMERA_TARGET_POINT_SOURCE_FRAME:-camera_d435_optical_frame}"
CAMERA_TARGET_POINT_TARGET_FRAME="${CAMERA_TARGET_POINT_TARGET_FRAME:-world}"
CAMERA_TARGET_POINT_TF_TIMEOUT_SEC="${CAMERA_TARGET_POINT_TF_TIMEOUT_SEC:-0.2}"
TARGET_POSE_EXECUTOR_DEFAULT_FRAME="${TARGET_POSE_EXECUTOR_DEFAULT_FRAME:-world}"
MIDDLEWARE_DM_SERIAL_BRIDGE_ENABLED="${MIDDLEWARE_DM_SERIAL_BRIDGE_ENABLED:-true}"
MIDDLEWARE_DM_SERIAL_ALLOWED_ACTION_SET_IDS="${MIDDLEWARE_DM_SERIAL_ALLOWED_ACTION_SET_IDS:-}"

echo "[run_rc_arm_real] workspace: ${WORKSPACE_DIR}"
echo "[run_rc_arm_real] ros_domain_id=${ROS_DOMAIN_ID}"
echo "[run_rc_arm_real] hardware_config_file=${HARDWARE_CONFIG_FILE}"
echo "[run_rc_arm_real] controllers_file=${CONTROLLERS_FILE}"
echo "[run_rc_arm_real] use_rviz=${USE_RVIZ}"
echo "[run_rc_arm_real] use_dm_serial_frame_bridge=${USE_DM_SERIAL_FRAME_BRIDGE}"
echo "[run_rc_arm_real] use_arm2_middleware=${USE_ARM2_MIDDLEWARE}"
echo "[run_rc_arm_real] use_camera_target_point_bridge=${USE_CAMERA_TARGET_POINT_BRIDGE}"
echo "[run_rc_arm_real] middleware_target_point_topic=${MIDDLEWARE_TARGET_POINT_TOPIC}"
echo "[run_rc_arm_real] camera_target_point_input_topic=${CAMERA_TARGET_POINT_INPUT_TOPIC}"
echo "[run_rc_arm_real] camera_target_point_output_topic=${CAMERA_TARGET_POINT_OUTPUT_TOPIC}"
echo "[run_rc_arm_real] camera_target_point_source_frame=${CAMERA_TARGET_POINT_SOURCE_FRAME}"
echo "[run_rc_arm_real] camera_target_point_target_frame=${CAMERA_TARGET_POINT_TARGET_FRAME}"
echo "[run_rc_arm_real] camera_target_point_tf_timeout_sec=${CAMERA_TARGET_POINT_TF_TIMEOUT_SEC}"
echo "[run_rc_arm_real] target_pose_executor_default_frame=${TARGET_POSE_EXECUTOR_DEFAULT_FRAME}"
echo "[run_rc_arm_real] middleware_dm_serial_bridge_enabled=${MIDDLEWARE_DM_SERIAL_BRIDGE_ENABLED}"
echo "[run_rc_arm_real] middleware_dm_serial_allowed_action_set_ids=${MIDDLEWARE_DM_SERIAL_ALLOWED_ACTION_SET_IDS}"
echo "[run_rc_arm_real] payload and unloaded defaults are read directly from ${HARDWARE_CONFIG_FILE}"

LAUNCH_ARGS=(
  "ros_domain_id:=${ROS_DOMAIN_ID}"
  "hardware_config_file:=${HARDWARE_CONFIG_FILE}"
  "controllers_file:=${CONTROLLERS_FILE}"
  "use_rviz:=${USE_RVIZ}"
  "use_tf_target_bridge:=${USE_TF_TARGET_BRIDGE}"
  "use_dm_serial_frame_bridge:=${USE_DM_SERIAL_FRAME_BRIDGE}"
  "use_target_pose_moveit_executor:=${USE_TARGET_POSE_MOVEIT_EXECUTOR}"
  "use_arm2_middleware:=${USE_ARM2_MIDDLEWARE}"
  "use_camera_target_point_bridge:=${USE_CAMERA_TARGET_POINT_BRIDGE}"
  "middleware_target_point_topic:=${MIDDLEWARE_TARGET_POINT_TOPIC}"
  "camera_target_point_input_topic:=${CAMERA_TARGET_POINT_INPUT_TOPIC}"
  "camera_target_point_output_topic:=${CAMERA_TARGET_POINT_OUTPUT_TOPIC}"
  "camera_target_point_source_frame:=${CAMERA_TARGET_POINT_SOURCE_FRAME}"
  "camera_target_point_target_frame:=${CAMERA_TARGET_POINT_TARGET_FRAME}"
  "camera_target_point_tf_timeout_sec:=${CAMERA_TARGET_POINT_TF_TIMEOUT_SEC}"
  "target_pose_executor_default_frame:=${TARGET_POSE_EXECUTOR_DEFAULT_FRAME}"
  "middleware_dm_serial_bridge_enabled:=${MIDDLEWARE_DM_SERIAL_BRIDGE_ENABLED}"
)

if [[ -n "${MIDDLEWARE_DM_SERIAL_ALLOWED_ACTION_SET_IDS}" ]]; then
  LAUNCH_ARGS+=("middleware_dm_serial_allowed_action_set_ids:=${MIDDLEWARE_DM_SERIAL_ALLOWED_ACTION_SET_IDS}")
fi

exec ros2 launch rc_arm_moveit_config rc_arm_2_robot.launch.py "${LAUNCH_ARGS[@]}" "$@"
