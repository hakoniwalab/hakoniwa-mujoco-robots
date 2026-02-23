#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "${SCRIPT_DIR}"

STATE_FILE="${HAKO_FORKLIFT_STATE_FILE:-./tmp/forklift-it.state}"
AUTOSAVE_STEPS="${HAKO_FORKLIFT_STATE_AUTOSAVE_STEPS:-1000}"
MOTION_GAIN="${HAKO_FORKLIFT_MOTION_GAIN:-0.2}"
RUN_LOG_FILE="${HAKO_RUN_LOG_FILE:-./logs/forklift-unit-run.log}"

mkdir -p ./logs
mkdir -p ./tmp

export HAKO_FORKLIFT_STATE_FILE="${STATE_FILE}"
export HAKO_FORKLIFT_STATE_AUTOSAVE_STEPS="${AUTOSAVE_STEPS}"
export HAKO_FORKLIFT_MOTION_GAIN="${MOTION_GAIN}"

echo "[forklift-unit] STATE_FILE=${STATE_FILE}" | tee -a "${RUN_LOG_FILE}"
echo "[forklift-unit] AUTOSAVE_STEPS=${AUTOSAVE_STEPS}" | tee -a "${RUN_LOG_FILE}"
echo "[forklift-unit] MOTION_GAIN=${MOTION_GAIN}" | tee -a "${RUN_LOG_FILE}"
./src/cmake-build/main_for_sample/forklift/forklift_unit_sim "$@" 2>&1 | tee -a "${RUN_LOG_FILE}"
exit ${PIPESTATUS[0]}
