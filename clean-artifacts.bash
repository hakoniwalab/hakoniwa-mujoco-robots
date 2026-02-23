#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "${SCRIPT_DIR}"

mkdir -p ./logs ./tmp

rm -f ./tmp/*
rm -f ./logs/*

echo "[clean] removed files under ./tmp and ./logs"
