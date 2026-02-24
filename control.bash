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
STARTUP_WAIT_SEC="${STARTUP_WAIT_SEC:-0.0}"
PHASE_TIMEOUT_SEC="${PHASE_TIMEOUT_SEC:-0.8}"
PDU_WRITE_LOG="${HAKO_PDU_WRITE_LOG:-1}"
CONTROLLER_MODE="${HAKO_CONTROLLER_MODE:-asset}"
CONTROLLER_ASSET_NAME="${HAKO_CONTROLLER_ASSET_NAME:-forklift-controller}"
CONTROLLER_DELTA_USEC="${HAKO_CONTROLLER_DELTA_USEC:-1000}"
FORWARD_GOAL_X="${FORWARD_GOAL_X:-}"
HOME_GOAL_X="${HOME_GOAL_X:-0.0}"
GOAL_TOLERANCE="${GOAL_TOLERANCE:-0.03}"
MISSION_LOOPS="${MISSION_LOOPS:-0}"
if [[ $# -ge 2 ]]; then
  MISSION_LOOPS="$2"
fi

mkdir -p ./logs

echo "[control] CONFIG_PATH=${CONFIG_PATH}" | tee -a "${CTRL_LOG_FILE}"
echo "[control] FORWARD_DISTANCE=${FORWARD_DISTANCE}" | tee -a "${CTRL_LOG_FILE}"
echo "[control] BACKWARD_DISTANCE=${BACKWARD_DISTANCE}" | tee -a "${CTRL_LOG_FILE}"
echo "[control] MOVE_SPEED=${MOVE_SPEED}" | tee -a "${CTRL_LOG_FILE}"
echo "[control] TURN_DEGREE=${TURN_DEGREE}" | tee -a "${CTRL_LOG_FILE}"
echo "[control] START_HEIGHT=${START_HEIGHT}" | tee -a "${CTRL_LOG_FILE}"
echo "[control] PAUSE_SEC=${PAUSE_SEC}" | tee -a "${CTRL_LOG_FILE}"
echo "[control] STARTUP_WAIT_SEC=${STARTUP_WAIT_SEC}" | tee -a "${CTRL_LOG_FILE}"
echo "[control] PHASE_TIMEOUT_SEC=${PHASE_TIMEOUT_SEC}" | tee -a "${CTRL_LOG_FILE}"
echo "[control] HAKO_PDU_WRITE_LOG=${PDU_WRITE_LOG}" | tee -a "${CTRL_LOG_FILE}"
echo "[control] CONTROLLER_MODE=${CONTROLLER_MODE}" | tee -a "${CTRL_LOG_FILE}"
echo "[control] CONTROLLER_ASSET_NAME=${CONTROLLER_ASSET_NAME}" | tee -a "${CTRL_LOG_FILE}"
echo "[control] CONTROLLER_DELTA_USEC=${CONTROLLER_DELTA_USEC}" | tee -a "${CTRL_LOG_FILE}"
echo "[control] MISSION_LOOPS=${MISSION_LOOPS}" | tee -a "${CTRL_LOG_FILE}"
echo "[control] START_TIME=$(date '+%Y-%m-%d %H:%M:%S')" | tee -a "${CTRL_LOG_FILE}"
if [[ -n "${FORWARD_GOAL_X}" ]]; then
  echo "[control] FORWARD_GOAL_X=${FORWARD_GOAL_X}" | tee -a "${CTRL_LOG_FILE}"
  echo "[control] HOME_GOAL_X=${HOME_GOAL_X}" | tee -a "${CTRL_LOG_FILE}"
  echo "[control] GOAL_TOLERANCE=${GOAL_TOLERANCE}" | tee -a "${CTRL_LOG_FILE}"
fi

if [[ -n "${FORWARD_GOAL_X}" ]]; then
  HAKO_PDU_WRITE_LOG="${PDU_WRITE_LOG}" python -u -m python.forklift_simple_auto "${CONFIG_PATH}" \
    --controller-mode "${CONTROLLER_MODE}" \
    --asset-name "${CONTROLLER_ASSET_NAME}" \
    --controller-delta-usec "${CONTROLLER_DELTA_USEC}" \
    --forward-goal-x "${FORWARD_GOAL_X}" \
    --home-goal-x "${HOME_GOAL_X}" \
    --goal-tolerance "${GOAL_TOLERANCE}" \
    --move-speed "${MOVE_SPEED}" \
    --start-height "${START_HEIGHT}" \
    --pause-sec "${PAUSE_SEC}" \
    --startup-wait-sec "${STARTUP_WAIT_SEC}" \
    --phase-timeout-sec "${PHASE_TIMEOUT_SEC}" \
    --mission-loops "${MISSION_LOOPS}" 2>&1 | tee -a "${CTRL_LOG_FILE}"
else
  HAKO_PDU_WRITE_LOG="${PDU_WRITE_LOG}" python -u -m python.forklift_simple_auto "${CONFIG_PATH}" \
    --controller-mode "${CONTROLLER_MODE}" \
    --asset-name "${CONTROLLER_ASSET_NAME}" \
    --controller-delta-usec "${CONTROLLER_DELTA_USEC}" \
    --forward-distance "${FORWARD_DISTANCE}" \
    --backward-distance "${BACKWARD_DISTANCE}" \
    --move-speed "${MOVE_SPEED}" \
    --turn-degree "${TURN_DEGREE}" \
    --start-height "${START_HEIGHT}" \
    --pause-sec "${PAUSE_SEC}" \
    --startup-wait-sec "${STARTUP_WAIT_SEC}" \
    --phase-timeout-sec "${PHASE_TIMEOUT_SEC}" \
    --mission-loops "${MISSION_LOOPS}" 2>&1 | tee -a "${CTRL_LOG_FILE}"
fi
exit ${PIPESTATUS[0]}
