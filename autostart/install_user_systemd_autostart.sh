#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${DIR}/.." && pwd)"
SERVICE_NAME="rc-arm2-autostart.service"
SOURCE_SERVICE="${DIR}/${SERVICE_NAME}"
USER_SYSTEMD_DIR="${XDG_CONFIG_HOME:-${HOME}/.config}/systemd/user"
TARGET_SERVICE="${USER_SYSTEMD_DIR}/${SERVICE_NAME}"

if [[ ! -f "${SOURCE_SERVICE}" ]]; then
  echo "[rc-arm2-autostart] missing service template: ${SOURCE_SERVICE}" >&2
  exit 1
fi

mkdir -p "${USER_SYSTEMD_DIR}"
sed "s#@RC_ROBOTARM_WORKSPACE@#${WORKSPACE_DIR}#g" "${SOURCE_SERVICE}" > "${TARGET_SERVICE}"
chmod 0644 "${TARGET_SERVICE}"
chmod +x "${DIR}/boot_start.sh"

systemctl --user daemon-reload
systemctl --user enable "${SERVICE_NAME}"

if command -v loginctl >/dev/null 2>&1; then
  if loginctl enable-linger "$(id -un)" >/dev/null 2>&1; then
    echo "[rc-arm2-autostart] enabled linger for user: $(id -un)"
  else
    echo "[rc-arm2-autostart] WARN: could not enable linger for user: $(id -un)" >&2
    echo "[rc-arm2-autostart] WARN: service may wait until user login/session is available." >&2
  fi
fi

echo "[rc-arm2-autostart] installed: ${TARGET_SERVICE}"
echo "[rc-arm2-autostart] enabled user unit: ${SERVICE_NAME}"
echo "[rc-arm2-autostart] start now with:"
echo "  systemctl --user start ${SERVICE_NAME}"
echo "[rc-arm2-autostart] status:"
systemctl --user --no-pager status "${SERVICE_NAME}" || true
