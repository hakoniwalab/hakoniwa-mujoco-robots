#!/bin/bash

# use "activate-only" mode to activate hakoniwa manually
# ACTIVATE_MODE="activate-only" bash tb3-demo.bash
ACTIVATE_MODE=${ACTIVATE_MODE:-"immediate"}
if grep -qi microsoft /proc/version 2>/dev/null || grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null; then
    powershell.exe -Command "python -m hakoniwa_pdu.apps.launcher.hako_launcher --mode ${ACTIVATE_MODE} tb3-demo-launch-win.json"
elif [ "$(uname -s)" = "Linux" ]; then
    python3 -m hakoniwa_pdu.apps.launcher.hako_launcher --mode ${ACTIVATE_MODE} tb3-demo-launch.json
elif [ "$(uname -s)" = "Darwin" ]; then
    python3 -m hakoniwa_pdu.apps.launcher.hako_launcher --mode ${ACTIVATE_MODE} tb3-demo-launch.json
else
    echo "Unsupported OS"
fi