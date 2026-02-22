#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.bash"

IMAGE_NAME="$(cat "${SCRIPT_DIR}/image_name.txt")"
IMAGE_TAG="$(cat "${SCRIPT_DIR}/latest_version.txt")"
DOCKER_IMAGE="toppersjp/${IMAGE_NAME}:${IMAGE_TAG}"

RUN_FLAGS=(
  -it --rm
  -w "${DOCKER_DIR}"
  --name "${IMAGE_NAME}"
  -e TZ=Asia/Tokyo
  -v "${HOST_WORKDIR}:${DOCKER_DIR}"
)

ARCH="$(arch)"
OS_TYPE="$("${SCRIPT_DIR}/utils/detect_os_type.bash")"
PORT="${PORT:-8765}"
GUI_MODE="${HAKO_DOCKER_GUI:-auto}" # auto|on|off

echo "[run] ARCH=${ARCH}"
echo "[run] OS_TYPE=${OS_TYPE}"
echo "[run] IMAGE=${DOCKER_IMAGE}"
echo "[run] WORKDIR host=${HOST_WORKDIR} -> container=${DOCKER_DIR}"
echo "[run] GUI_MODE=${GUI_MODE}"

NET_FLAG=()

if [[ "${OS_TYPE}" == "Mac" ]]; then
  # macOS は --net=host が使えない → ポートフォワード
  NET_FLAG=(-p "${PORT}:${PORT}")
else
  # Linux は host ネットワーク
  NET_FLAG=(--net host)
fi

enable_headless() {
  echo "[run] Use headless mode with Xvfb"
  RUN_FLAGS+=(
    -e LIBGL_ALWAYS_SOFTWARE=1
    -e HAKO_USE_XVFB=1
  )
}

enable_gui_linux() {
  local display_value="${DISPLAY:-}"
  if [[ -z "${display_value}" ]]; then
    return 1
  fi
  if [[ ! -d /tmp/.X11-unix ]]; then
    return 1
  fi
  echo "[run] Enable Linux X11 forwarding (DISPLAY=${display_value})"
  RUN_FLAGS+=(
    -e DISPLAY="${display_value}"
    -v /tmp/.X11-unix:/tmp/.X11-unix
  )
  if [[ -d /dev/dri ]]; then
    RUN_FLAGS+=(--device /dev/dri)
  fi
  return 0
}

enable_gui_mac() {
  local display_value="${HAKO_DOCKER_DISPLAY:-${DISPLAY:-}}"
  if [[ -z "${display_value}" ]]; then
    display_value="host.docker.internal:0"
  fi
  # macOS host DISPLAY often looks like /private/tmp/.../org.xquartz:0, which
  # is not reachable from container. Normalize to Docker Desktop host alias.
  if [[ "${display_value}" == /private/tmp/* || "${display_value}" == /tmp/* ]]; then
    display_value="host.docker.internal:0"
  fi
  echo "[run] Enable macOS XQuartz forwarding (DISPLAY=${display_value})"
  echo "[run] Hint: run 'open -a XQuartz' and 'xhost +localhost' on host if window does not appear."
  RUN_FLAGS+=(
    -e DISPLAY="${display_value}"
    -e LIBGL_ALWAYS_INDIRECT=1
  )
  return 0
}

case "${GUI_MODE}" in
  off)
    enable_headless
    ;;
  on)
    if [[ "${OS_TYPE}" == "Mac" ]]; then
      enable_gui_mac
    else
      if ! enable_gui_linux; then
        echo "[run] ERROR: GUI mode 'on' requested, but X11 is not available on host."
        echo "[run] Set DISPLAY and ensure /tmp/.X11-unix exists, or use HAKO_DOCKER_GUI=off."
        exit 1
      fi
    fi
    ;;
  auto)
    if [[ "${OS_TYPE}" == "Mac" ]]; then
      enable_gui_mac
    else
      if ! enable_gui_linux; then
        enable_headless
      fi
    fi
    ;;
  *)
    echo "[run] ERROR: invalid HAKO_DOCKER_GUI='${GUI_MODE}' (expected: auto|on|off)"
    exit 1
    ;;
esac

# Optional platform override, e.g. DOCKER_PLATFORM=linux/amd64
if [[ -n "${DOCKER_PLATFORM:-}" ]]; then
  docker run --platform "${DOCKER_PLATFORM}" "${RUN_FLAGS[@]}" "${NET_FLAG[@]}" "${DOCKER_IMAGE}"
else
  docker run "${RUN_FLAGS[@]}" "${NET_FLAG[@]}" "${DOCKER_IMAGE}"
fi
