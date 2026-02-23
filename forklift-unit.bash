#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "${SCRIPT_DIR}"

STATE_FILE="${HAKO_FORKLIFT_STATE_FILE:-./tmp/forklift-it.state}"
AUTOSAVE_STEPS="${HAKO_FORKLIFT_STATE_AUTOSAVE_STEPS:-1000}"
MOTION_GAIN="${HAKO_FORKLIFT_MOTION_GAIN:-0.2}"
RUN_LOG_FILE="${HAKO_RUN_LOG_FILE:-./logs/forklift-unit-run.log}"
TRACE_FILE="${HAKO_FORKLIFT_TRACE_FILE:-./logs/forklift-unit-trace.csv}"
TRACE_EVERY_STEPS="${HAKO_FORKLIFT_TRACE_EVERY_STEPS:-10}"
RESUME_CMD_HOLD_SEC="${HAKO_FORKLIFT_RESUME_CMD_HOLD_SEC:-2.0}"
RESET_ARTIFACTS="${HAKO_RESET_ARTIFACTS:-0}"

mkdir -p ./logs
mkdir -p ./tmp

if [[ "${RESET_ARTIFACTS}" == "1" ]]; then
  rm -f "${STATE_FILE}" \
        "${RUN_LOG_FILE}" \
        "${TRACE_FILE}" \
        ./logs/forklift-unit-recovery.log \
        ./logs/control-run.log
  echo "[forklift-unit] RESET_ARTIFACTS=1 -> cleaned state/log files"
fi

export HAKO_FORKLIFT_STATE_FILE="${STATE_FILE}"
export HAKO_FORKLIFT_STATE_AUTOSAVE_STEPS="${AUTOSAVE_STEPS}"
export HAKO_FORKLIFT_MOTION_GAIN="${MOTION_GAIN}"
export HAKO_FORKLIFT_TRACE_FILE="${TRACE_FILE}"
export HAKO_FORKLIFT_TRACE_EVERY_STEPS="${TRACE_EVERY_STEPS}"
export HAKO_FORKLIFT_RESUME_CMD_HOLD_SEC="${RESUME_CMD_HOLD_SEC}"

echo "[forklift-unit] STATE_FILE=${STATE_FILE}" | tee -a "${RUN_LOG_FILE}"
echo "[forklift-unit] AUTOSAVE_STEPS=${AUTOSAVE_STEPS}" | tee -a "${RUN_LOG_FILE}"
echo "[forklift-unit] MOTION_GAIN=${MOTION_GAIN}" | tee -a "${RUN_LOG_FILE}"
echo "[forklift-unit] TRACE_FILE=${TRACE_FILE}" | tee -a "${RUN_LOG_FILE}"
echo "[forklift-unit] TRACE_EVERY_STEPS=${TRACE_EVERY_STEPS}" | tee -a "${RUN_LOG_FILE}"
echo "[forklift-unit] RESUME_CMD_HOLD_SEC=${RESUME_CMD_HOLD_SEC}" | tee -a "${RUN_LOG_FILE}"
echo "[forklift-unit] START_TIME=$(date '+%Y-%m-%d %H:%M:%S')" | tee -a "${RUN_LOG_FILE}"
./src/cmake-build/main_for_sample/forklift/forklift_unit_sim "$@" 2>&1 | tee -a "${RUN_LOG_FILE}"
exit ${PIPESTATUS[0]}
