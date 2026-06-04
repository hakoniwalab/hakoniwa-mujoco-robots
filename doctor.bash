#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
FAILURES=0
WARNINGS=0

info() {
    printf 'INFO  %s\n' "$*"
}

ok() {
    printf 'OK    %s\n' "$*"
}

warn() {
    WARNINGS=$((WARNINGS + 1))
    printf 'WARN  %s\n' "$*"
}

fail() {
    FAILURES=$((FAILURES + 1))
    printf 'FAIL  %s\n' "$*"
}

have_command() {
    command -v "$1" >/dev/null 2>&1
}

check_command() {
    local name="$1"
    local hint="$2"
    if have_command "$name"; then
        ok "$name found: $(command -v "$name")"
    else
        fail "$name not found. $hint"
    fi
}

check_path_dir() {
    local label="$1"
    local path="$2"
    if [ -d "$path" ]; then
        ok "$label found: $path"
    else
        fail "$label not found: $path"
    fi
}

check_cmake_package() {
    local label="$1"
    local default_root="$2"
    local env_name="$3"
    local rel_config="$4"
    local root="${!env_name:-$default_root}"
    local config_path="$root/$rel_config"

    if [ -f "$config_path" ]; then
        ok "$label package config found: $config_path"
    else
        fail "$label package config not found: $config_path"
        info "Set $env_name to the install prefix if it is not under $default_root."
    fi
}

check_python_package() {
    local python_cmd="$1"
    local package_name="$2"
    local min_version="$3"

    if ! have_command "$python_cmd"; then
        fail "$python_cmd not found. Install Python 3 or use another Python command."
        return
    fi

    info "Checking $package_name with $python_cmd: $(command -v "$python_cmd")"
    if "$python_cmd" - "$package_name" "$min_version" <<'PY'
import importlib.metadata
import sys

name = sys.argv[1]
minimum = tuple(int(x) for x in sys.argv[2].split("."))

try:
    version = importlib.metadata.version(name)
except importlib.metadata.PackageNotFoundError:
    print(f"{name} is not installed")
    sys.exit(2)

parts = []
for item in version.split("."):
    digits = ""
    for ch in item:
        if ch.isdigit():
            digits += ch
        else:
            break
    if digits:
        parts.append(int(digits))
    else:
        break
current = tuple(parts)

print(version)
if current < minimum:
    sys.exit(3)
PY
    then
        ok "$package_name >= $min_version installed for $python_cmd"
    else
        local status=$?
        if [ "$status" -eq 2 ]; then
            fail "$package_name is not installed for $python_cmd. Run: $python_cmd -m pip install --upgrade \"$package_name>=$min_version\""
        elif [ "$status" -eq 3 ]; then
            fail "$package_name is older than $min_version for $python_cmd. Run: $python_cmd -m pip install --upgrade \"$package_name>=$min_version\""
        else
            fail "Could not check $package_name with $python_cmd"
        fi
    fi
}

check_glfw() {
    if have_command pkg-config && pkg-config --exists glfw3; then
        ok "glfw3 found by pkg-config"
        return
    fi

    if [ "$(uname -s)" = "Darwin" ] && have_command brew && brew --prefix glfw >/dev/null 2>&1; then
        ok "glfw found by Homebrew: $(brew --prefix glfw)"
        return
    fi

    warn "glfw3 was not detected. macOS: brew install glfw. Ubuntu: sudo apt-get install -y libglfw3-dev"
}

printf 'Hakoniwa MuJoCo Robots doctor\n'
printf 'root: %s\n\n' "$ROOT_DIR"

check_command cmake "Install CMake 3.20 or newer."
check_command git "Install Git."

if [ -n "${PYTHON_CMD:-}" ]; then
    :
elif have_command python3; then
    PYTHON_CMD=python3
elif have_command python; then
    PYTHON_CMD=python
else
    PYTHON_CMD=python3
fi

check_python_package "$PYTHON_CMD" hakoniwa-pdu 1.6.1

if [ -f "$ROOT_DIR/MUJOCO_VERSION.txt" ]; then
    MUJOCO_VERSION="$(tr -d '[:space:]' < "$ROOT_DIR/MUJOCO_VERSION.txt")"
    if [ -n "$MUJOCO_VERSION" ]; then
        ok "MuJoCo version file: $MUJOCO_VERSION"
    else
        fail "MUJOCO_VERSION.txt is empty"
    fi
else
    fail "MUJOCO_VERSION.txt not found"
fi

check_path_dir "thirdparty/nolman single include" "$ROOT_DIR/thirdparty/nolman/single_include/nlohmann"
check_cmake_package "hakoniwa-core-pro" "${HAKONIWA_CORE_ROOT:-/usr/local/hakoniwa}" HAKONIWA_CORE_ROOT "lib/cmake/hakoniwa-core/hakoniwa-coreConfig.cmake"
check_cmake_package "hakoniwa-pdu-endpoint" "${HAKONIWA_PDU_ENDPOINT_ROOT:-/usr/local/hakoniwa}" HAKONIWA_PDU_ENDPOINT_ROOT "lib/cmake/hakoniwa_pdu_endpoint/hakoniwa_pdu_endpointConfig.cmake"
check_glfw

printf '\nSummary: %d failure(s), %d warning(s)\n' "$FAILURES" "$WARNINGS"

if [ "$FAILURES" -ne 0 ]; then
    printf 'Run the suggested install commands, then re-run ./doctor.bash before ./build.bash.\n'
    exit 1
fi

printf 'Environment looks ready for ./build.bash.\n'
