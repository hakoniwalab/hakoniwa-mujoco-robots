#!/bin/bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${HAKO_BUILD_DIR:-${PROJECT_ROOT}/src/cmake-build}"

CORE_PREFIX="${HAKONIWA_CORE_ROOT:-/usr/local/hakoniwa}"
ENDPOINT_PREFIX="${HAKONIWA_PDU_ENDPOINT_ROOT:-${CORE_PREFIX}}"
USE_THIRDPARTY_HAKONIWA=0
EXTRA_CMAKE_ARGS=()

print_install_help() {
    cat <<EOF

Install hints:
  hakoniwa-core-pro:
    git clone --recursive https://github.com/hakoniwalab/hakoniwa-core-pro.git
    cd hakoniwa-core-pro
    bash build.bash
    bash install.bash

  glfw:
    macOS:  brew install glfw
    Ubuntu: sudo apt-get update && sudo apt-get install -y libglfw3-dev

  hakoniwa-pdu-endpoint:
    Install it into /usr/local/hakoniwa, or set:
      export HAKONIWA_PDU_ENDPOINT_ROOT=/path/to/hakoniwa-pdu-endpoint/install

If you installed Hakoniwa core outside /usr/local/hakoniwa, set:
  export HAKONIWA_CORE_ROOT=/path/to/hakoniwa-core-pro/install

To use the Hakoniwa submodules in this repository, pass:
  -DHAKO_USE_THIRDPARTY_HAKONIWA=ON

To bypass this preflight check temporarily:
  HAKO_SKIP_PREFLIGHT=1 ./build.bash
EOF
}

fail_preflight() {
    echo "ERROR: build prerequisites are missing:" >&2
    local issue
    for issue in "$@"; do
        echo "  - ${issue}" >&2
    done
    print_install_help >&2
    exit 1
}

has_glfw() {
    if command -v pkg-config >/dev/null 2>&1 && pkg-config --exists glfw3; then
        return 0
    fi

    local prefix
    local old_ifs="${IFS}"
    IFS=':;'
    for prefix in ${CMAKE_PREFIX_PATH:-} /opt/homebrew /usr/local /usr; do
        if [[ -f "${prefix}/lib/cmake/glfw3/glfw3Config.cmake" ]]; then
            IFS="${old_ifs}"
            return 0
        fi
    done
    IFS="${old_ifs}"
    return 1
}

preflight() {
    local issues=()

    command -v cmake >/dev/null 2>&1 || issues+=("cmake is not installed or not in PATH.")

    if [[ "${USE_THIRDPARTY_HAKONIWA}" == "1" ]]; then
        [[ -f "${PROJECT_ROOT}/thirdparty/hakoniwa-core-pro/CMakeLists.txt" ]] || issues+=("vendored hakoniwa-core-pro was not found. Run: git submodule update --init --recursive")
        [[ -f "${PROJECT_ROOT}/thirdparty/hakoniwa-pdu-endpoint/CMakeLists.txt" ]] || issues+=("vendored hakoniwa-pdu-endpoint was not found. Run: git submodule update --init --recursive")
    else
        if [[ ! -d "${CORE_PREFIX}" ]]; then
            issues+=("hakoniwa-core-pro install prefix was not found: ${CORE_PREFIX}")
        elif [[ ! -f "${CORE_PREFIX}/lib/cmake/hakoniwa-core/hakoniwa-coreConfig.cmake" \
           && ! -f "${CORE_PREFIX}/lib/libassets.a" \
           && ! -f "${CORE_PREFIX}/lib/libassets.dylib" \
           && ! -f "${CORE_PREFIX}/lib/libassets.so" ]]; then
            issues+=("hakoniwa-core-pro does not look installed under: ${CORE_PREFIX}")
        fi

        if [[ ! -f "${ENDPOINT_PREFIX}/lib/cmake/hakoniwa_pdu_endpoint/hakoniwa_pdu_endpointConfig.cmake" ]]; then
            issues+=("hakoniwa-pdu-endpoint package config was not found under: ${ENDPOINT_PREFIX}")
        fi
    fi

    if ! has_glfw; then
        issues+=("glfw3 was not found by pkg-config or common CMake package paths.")
    fi

    if [[ ${#issues[@]} -gt 0 ]]; then
        fail_preflight "${issues[@]}"
    fi
}

if [[ $# -eq 1 && "$1" == "clean" ]]; then
    rm -rf "${BUILD_DIR:?}/"*
    exit 0
fi

for arg in "$@"; do
    if [[ "${arg}" == "-DHAKO_USE_THIRDPARTY_HAKONIWA=ON" ]]; then
        USE_THIRDPARTY_HAKONIWA=1
    fi
    EXTRA_CMAKE_ARGS+=("${arg}")
done

if [[ "${HAKO_SKIP_PREFLIGHT:-0}" != "1" ]]; then
    preflight
fi

mkdir -p "${BUILD_DIR}"

CMAKE_ARGS=(
    -S "${PROJECT_ROOT}/src"
    -B "${BUILD_DIR}"
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    -DUSE_VIEWER=ON
)

if [[ "${USE_THIRDPARTY_HAKONIWA}" != "1" ]]; then
    CMAKE_ARGS+=(
        -DHAKONIWA_INSTALL_PREFIX="${CORE_PREFIX}"
        -DHAKONIWA_PDU_ENDPOINT_PREFIX="${ENDPOINT_PREFIX}"
    )
fi

CMAKE_ARGS+=("${EXTRA_CMAKE_ARGS[@]}")

cmake "${CMAKE_ARGS[@]}"
cmake --build "${BUILD_DIR}" --parallel
