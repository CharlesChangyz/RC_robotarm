#!/usr/bin/env bash

set -eo pipefail

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[run_fastdds_discovery_server] missing ROS 2 Humble setup: /opt/ros/humble/setup.bash" >&2
  exit 1
fi

# ROS/ament setup scripts may read optional environment variables before
# defining them, which is incompatible with `set -u`.
set +u
source /opt/ros/humble/setup.bash
set -u

SERVER_ID="${FASTDDS_DISCOVERY_SERVER_ID:-0}"
SERVER_HOST="${FASTDDS_DISCOVERY_SERVER_HOST:-192.168.3.83}"
SERVER_PORT="${FASTDDS_DISCOVERY_SERVER_PORT:-11811}"

echo "[run_fastdds_discovery_server] server_id=${SERVER_ID}"
echo "[run_fastdds_discovery_server] listen=${SERVER_HOST}:${SERVER_PORT}"
echo "[run_fastdds_discovery_server] starting Fast DDS discovery server"
exec fastdds discovery -i "${SERVER_ID}" -l "${SERVER_HOST}" -p "${SERVER_PORT}"
