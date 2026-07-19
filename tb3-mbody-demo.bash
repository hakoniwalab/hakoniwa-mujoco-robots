#!/bin/bash

# Launch a joystick-free MBody TurtleBot3 demo with hakoniwa-pdu launcher.
#
# Examples:
#   bash tb3-mbody-demo.bash
#   HAKO_TB3_MODEL=waffle HAKO_TB3_ROUTE_PATTERN=figure8 bash tb3-mbody-demo.bash
#   HAKO_TB3_MODEL=waffle_pi HAKO_TB3_ROUTE_PATTERN=figure8 bash tb3-mbody-demo.bash
#   HAKO_TB3_ROUTE_PATTERN=dance HAKO_TB3_ENABLE_VIEWER=1 bash tb3-mbody-demo.bash
#   HAKO_TB3_ENABLE_VIEWER=0 ACTIVATE_MODE=activate-only bash tb3-mbody-demo.bash

ACTIVATE_MODE=${ACTIVATE_MODE:-"immediate"}
LAUNCH_FILE=${LAUNCH_FILE:-"tb3-mbody-demo-launch.json"}
HAKO_TB3_MODEL=${HAKO_TB3_MODEL:-"burger"}
HAKO_PYTHON=${HAKO_PYTHON:-"python3.12"}

if [ -z "${HAKO_TB3_SIM:-}" ]; then
    case "${HAKO_TB3_MODEL}" in
        burger)
            export HAKO_TB3_SIM="./src/cmake-build/main_for_sample/tb3/tb3_sim_burger"
            ;;
        waffle)
            export HAKO_TB3_SIM="./src/cmake-build/main_for_sample/tb3/tb3_sim_waffle"
            ;;
        waffle_pi)
            export HAKO_TB3_SIM="./src/cmake-build/main_for_sample/tb3/tb3_sim_waffle_pi"
            ;;
        *)
            echo "Unsupported HAKO_TB3_MODEL: ${HAKO_TB3_MODEL}"
            echo "Use burger, waffle, or waffle_pi, or set HAKO_TB3_SIM explicitly."
            exit 1
            ;;
    esac
fi

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
