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

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
export ROS_DOMAIN_ID="55"
set -u

echo "[run_tf_cli_domain55] ros_domain_id=${ROS_DOMAIN_ID}"
echo "[run_tf_cli_domain55] starting TF target GUI"

exec python3 "${REPO_ROOT}/demo/tf_target_cli_publisher.py" "$@"
