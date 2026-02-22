#!/bin/bash

set -euo pipefail

IMAGE_NAME="$(cat docker/image_name.txt)"
IMAGE_TAG="$(cat docker/latest_version.txt)"
DOCKER_IMAGE="toppersjp/${IMAGE_NAME}:${IMAGE_TAG}"
DOCKER_FILE="docker/Dockerfile"

ARCH="$(uname -m)"
PLATFORM="${DOCKER_PLATFORM:-}"
if [[ -z "${PLATFORM}" ]]; then
  if [[ "${ARCH}" == "arm64" || "${ARCH}" == "aarch64" ]]; then
    PLATFORM="linux/arm64"
  else
    PLATFORM="linux/amd64"
  fi
fi

echo "[create-image] ARCH=${ARCH}"
echo "[create-image] PLATFORM=${PLATFORM}"
HAKO_CORE_PRO_REPO="${HAKO_CORE_PRO_REPO:-https://github.com/hakoniwalab/hakoniwa-core-pro.git}"
HAKO_CORE_PRO_REF="${HAKO_CORE_PRO_REF:-main}"
echo "[create-image] HAKO_CORE_PRO_REPO=${HAKO_CORE_PRO_REPO}"
echo "[create-image] HAKO_CORE_PRO_REF=${HAKO_CORE_PRO_REF}"
docker build --no-cache --platform "${PLATFORM}" \
  --build-arg "HAKO_CORE_PRO_REPO=${HAKO_CORE_PRO_REPO}" \
  --build-arg "HAKO_CORE_PRO_REF=${HAKO_CORE_PRO_REF}" \
  -t "${DOCKER_IMAGE}" -f "${DOCKER_FILE}" .
