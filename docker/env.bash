#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export HOST_WORKDIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
export DOCKER_DIR=/root/workspace

