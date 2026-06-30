#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${RC_ROBOTARM_WORKSPACE:-/home/rc2/RC_robotarm}"
KFS_DIR="${KFS_DIR:-/home/rc2/KFS}"
LOG_DIR="${RC_ARM2_AUTOSTART_LOG_DIR:-${HOME}/.local/state/rc-arm2-autostart}"
CONDA_SH="${CONDA_SH:-/home/rc2/miniconda3/etc/profile.d/conda.sh}"
TF_SERVICE="${TF_SERVICE:-/rc_arm_2/remote/start_real}"
START_REAL_DELAY_SEC="${START_REAL_DELAY_SEC:-8}"
START_REAL_RETRIES="${START_REAL_RETRIES:-30}"
START_REAL_RETRY_DELAY_SEC="${START_REAL_RETRY_DELAY_SEC:-2}"
DISPLAY_WAIT_SEC="${DISPLAY_WAIT_SEC:-60}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-55}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/home/rc2/.Xauthority}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

mkdir -p "${LOG_DIR}"

TF_LOG="${LOG_DIR}/tf_gui.log"
KFS_LOG="${LOG_DIR}/kfs_camera.log"
MAIN_LOG="${LOG_DIR}/boot_start.log"

log() {
  printf '[%(%F %T)T] %s\n' -1 "$*" | tee -a "${MAIN_LOG}"
}

children=()

cleanup() {
  log "stopping autostart children"
  for pid in "${children[@]}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
  wait || true
}
trap cleanup INT TERM EXIT

if [[ ! -x "${WORKSPACE_DIR}/scripts/run_tf_cli_domain55.sh" ]]; then
  log "missing executable: ${WORKSPACE_DIR}/scripts/run_tf_cli_domain55.sh"
  exit 1
fi

if [[ ! -f "${WORKSPACE_DIR}/rc_moveit/install/setup.bash" ]]; then
  log "missing ROS workspace setup: ${WORKSPACE_DIR}/rc_moveit/install/setup.bash"
  exit 1
fi

if [[ ! -f "${KFS_DIR}/xinsheng4ros.py" ]]; then
  log "missing KFS script: ${KFS_DIR}/xinsheng4ros.py"
  exit 1
fi

if [[ "${DISPLAY}" == :* ]]; then
  display_num="${DISPLAY#:}"
  display_num="${display_num%%.*}"
  x_socket="/tmp/.X11-unix/X${display_num}"
  for _ in $(seq 1 "${DISPLAY_WAIT_SEC}"); do
    if [[ -S "${x_socket}" && -f "${XAUTHORITY}" ]]; then
      break
    fi
    sleep 1
  done
fi

log "starting TF target GUI"
(
  cd "${WORKSPACE_DIR}/scripts"
  exec ./run_tf_cli_domain55.sh
) >>"${TF_LOG}" 2>&1 &
children+=("$!")

log "starting KFS camera ROS publisher"
(
  cd "${KFS_DIR}"
  if [[ -f "${CONDA_SH}" ]]; then
    # shellcheck disable=SC1090
    source "${CONDA_SH}"
    conda activate camera
    exec python3 xinsheng4ros.py
  else
    exec /home/rc2/miniconda3/condabin/conda run --no-capture-output -n camera python3 xinsheng4ros.py
  fi
) >>"${KFS_LOG}" 2>&1 &
children+=("$!")

log "waiting ${START_REAL_DELAY_SEC}s before requesting start_real"
sleep "${START_REAL_DELAY_SEC}"

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/rc_moveit/install/setup.bash"
set -u

for attempt in $(seq 1 "${START_REAL_RETRIES}"); do
  if timeout 5 ros2 service call "${TF_SERVICE}" std_srvs/srv/Trigger "{}" >>"${MAIN_LOG}" 2>&1; then
    log "start_real request sent via ${TF_SERVICE}"
    break
  fi
  log "start_real service not ready, retry ${attempt}/${START_REAL_RETRIES}"
  sleep "${START_REAL_RETRY_DELAY_SEC}"
done

log "autostart processes are running; waiting for exit"
wait -n "${children[@]}"
status=$?
log "one child exited with status ${status}; stopping the rest"
exit "${status}"
