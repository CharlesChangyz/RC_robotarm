#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${REPO_ROOT}/rc_moveit"

if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "[run_tf_cli_domain55] missing workspace setup: ${WORKSPACE_DIR}/install/setup.bash"
  echo "[run_tf_cli_domain55] please build first: cd ${WORKSPACE_DIR} && colcon build"
  exit 1
fi

# ROS/ament setup scripts may read optional environment variables before
# defining them, which is incompatible with `set -u`.
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-55}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

DISCOVERY_SERVER_HOST="${DISCOVERY_SERVER_HOST:-192.168.3.83}"
DISCOVERY_SERVER_PORT="${DISCOVERY_SERVER_PORT:-11811}"
export ROS_DISCOVERY_SERVER="${ROS_DISCOVERY_SERVER:-${DISCOVERY_SERVER_HOST}:${DISCOVERY_SERVER_PORT}}"

echo "[run_tf_cli_domain55] ros_domain_id=${ROS_DOMAIN_ID}"
echo "[run_tf_cli_domain55] ros_localhost_only=${ROS_LOCALHOST_ONLY}"
echo "[run_tf_cli_domain55] rmw_implementation=${RMW_IMPLEMENTATION}"
echo "[run_tf_cli_domain55] ros_discovery_server=${ROS_DISCOVERY_SERVER}"
echo "[run_tf_cli_domain55] starting TF target GUI"

exec python3 "${REPO_ROOT}/demo/tf_target_cli_publisher.py" "$@"
