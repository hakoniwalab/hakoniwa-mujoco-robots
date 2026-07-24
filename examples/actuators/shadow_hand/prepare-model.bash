#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
MBODY_ROOT="$REPO_ROOT/thirdparty/hakoniwa-mbody-registry"
SOURCE_YAML="$MBODY_ROOT/sources/shadow_hand.yaml"
OUTPUT_DIR="$MBODY_ROOT/bodies/shadow_hand/source"
MODEL_PATH="$OUTPUT_DIR/shadow_hand/scene_right.xml"

if [ ! -f "$SOURCE_YAML" ]; then
    echo "Error: Shadow Hand source definition is missing from hakoniwa-mbody-registry." >&2
    echo "Run: git submodule update --init thirdparty/hakoniwa-mbody-registry" >&2
    exit 1
fi

if [ -x "$MBODY_ROOT/.venv/bin/python" ]; then
    export PATH="$MBODY_ROOT/.venv/bin:$PATH"
fi

"$MBODY_ROOT/tools/forge.sh" "$SOURCE_YAML" "$OUTPUT_DIR"

if [ ! -f "$MODEL_PATH" ]; then
    echo "Error: Shadow Hand model was not materialized at $MODEL_PATH" >&2
    exit 1
fi

echo "Shadow Hand model is ready:"
echo "$MODEL_PATH"
