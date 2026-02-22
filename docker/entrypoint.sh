#!/usr/bin/env bash
set -euo pipefail

# For environments without X server, allow running with virtual framebuffer.
if [[ "${HAKO_USE_XVFB:-0}" == "1" && -z "${DISPLAY:-}" ]]; then
  export DISPLAY=:99
  Xvfb :99 -screen 0 1280x720x24 > /tmp/xvfb.log 2>&1 &
  echo "[hako-entry] Started Xvfb on ${DISPLAY}."
else
  echo "[hako-entry] Using DISPLAY=${DISPLAY:-<none>}."
fi

exec "$@"
