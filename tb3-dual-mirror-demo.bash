#!/bin/bash

# Launch the multi-process TurtleBot3 mirror recipe:
#   Process A: real Burger + mirrored Waffle, Conductor owner, Viewer enabled
#   Process B: real Waffle + mirrored Burger, Conductor disabled, Viewer disabled
#   Controller A: drives TB3_BURGER/hako_cmd_game
#   Controller B: drives TB3_WAFFLE/hako_cmd_game

ACTIVATE_MODE=${ACTIVATE_MODE:-"immediate"}
LAUNCH_FILE=${LAUNCH_FILE:-"tb3-dual-mirror-demo-launch.json"}
HAKO_PYTHON=${HAKO_PYTHON:-"python3.12"}

if grep -qi microsoft /proc/version 2>/dev/null || grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null; then
    powershell.exe -Command "${HAKO_PYTHON} -m hakoniwa_pdu.apps.launcher.hako_launcher --mode ${ACTIVATE_MODE} ${LAUNCH_FILE}"
elif [ "$(uname -s)" = "Linux" ]; then
    "${HAKO_PYTHON}" -m hakoniwa_pdu.apps.launcher.hako_launcher --mode "${ACTIVATE_MODE}" "${LAUNCH_FILE}"
elif [ "$(uname -s)" = "Darwin" ]; then
    "${HAKO_PYTHON}" -m hakoniwa_pdu.apps.launcher.hako_launcher --mode "${ACTIVATE_MODE}" "${LAUNCH_FILE}"
else
    echo "Unsupported OS"
    exit 1
fi
