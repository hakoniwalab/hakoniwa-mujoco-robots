#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "${SCRIPT_DIR}"

CONFIG_PATH="${1:-config/forklift-unit.json}"
FORWARD_DISTANCE="${FORWARD_DISTANCE:-2.0}"
BACKWARD_DISTANCE="${BACKWARD_DISTANCE:-2.0}"
MOVE_SPEED="${MOVE_SPEED:-0.7}"
TURN_DEGREE="${TURN_DEGREE:-0.0}"
START_HEIGHT="${START_HEIGHT:--0.05}"
PAUSE_SEC="${PAUSE_SEC:-0.5}"
CTRL_LOG_FILE="${HAKO_CTRL_LOG_FILE:-./logs/control-run.log}"
FORWARD_GOAL_X="${FORWARD_GOAL_X:-}"
HOME_GOAL_X="${HOME_GOAL_X:-0.0}"
GOAL_TOLERANCE="${GOAL_TOLERANCE:-0.03}"

mkdir -p ./logs

echo "[control] CONFIG_PATH=${CONFIG_PATH}" | tee -a "${CTRL_LOG_FILE}"
echo "[control] FORWARD_DISTANCE=${FORWARD_DISTANCE}" | tee -a "${CTRL_LOG_FILE}"
echo "[control] BACKWARD_DISTANCE=${BACKWARD_DISTANCE}" | tee -a "${CTRL_LOG_FILE}"
echo "[control] MOVE_SPEED=${MOVE_SPEED}" | tee -a "${CTRL_LOG_FILE}"
echo "[control] TURN_DEGREE=${TURN_DEGREE}" | tee -a "${CTRL_LOG_FILE}"
echo "[control] START_HEIGHT=${START_HEIGHT}" | tee -a "${CTRL_LOG_FILE}"
echo "[control] PAUSE_SEC=${PAUSE_SEC}" | tee -a "${CTRL_LOG_FILE}"
if [[ -n "${FORWARD_GOAL_X}" ]]; then
  echo "[control] FORWARD_GOAL_X=${FORWARD_GOAL_X}" | tee -a "${CTRL_LOG_FILE}"
  echo "[control] HOME_GOAL_X=${HOME_GOAL_X}" | tee -a "${CTRL_LOG_FILE}"
  echo "[control] GOAL_TOLERANCE=${GOAL_TOLERANCE}" | tee -a "${CTRL_LOG_FILE}"
fi

if [[ -n "${FORWARD_GOAL_X}" ]]; then
  python -m python.forklift_simple_auto "${CONFIG_PATH}" \
    --forward-goal-x "${FORWARD_GOAL_X}" \
    --home-goal-x "${HOME_GOAL_X}" \
    --goal-tolerance "${GOAL_TOLERANCE}" \
    --move-speed "${MOVE_SPEED}" \
    --start-height "${START_HEIGHT}" \
    --pause-sec "${PAUSE_SEC}" 2>&1 | tee -a "${CTRL_LOG_FILE}"
else
  python -m python.forklift_simple_auto "${CONFIG_PATH}" \
    --forward-distance "${FORWARD_DISTANCE}" \
    --backward-distance "${BACKWARD_DISTANCE}" \
    --move-speed "${MOVE_SPEED}" \
    --turn-degree "${TURN_DEGREE}" \
    --start-height "${START_HEIGHT}" \
    --pause-sec "${PAUSE_SEC}" 2>&1 | tee -a "${CTRL_LOG_FILE}"
fi
exit ${PIPESTATUS[0]}
