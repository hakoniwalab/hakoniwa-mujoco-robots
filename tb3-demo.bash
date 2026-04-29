#!/bin/bash

# use "activate-only" mode to activate hakoniwa manually
# ACTIVATE_MODE="activate-only" bash tb3-demo.bash
ACTIVATE_MODE=${ACTIVATE_MODE:-"immediate"}

 python3 -m hakoniwa_pdu.apps.launcher.hako_launcher --mode ${ACTIVATE_MODE} tb3-demo-launch.json
