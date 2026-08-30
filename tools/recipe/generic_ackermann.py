#!/usr/bin/env python3
"""Configure and operate the Launcher-managed generic Ackermann Recipe."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path


PROFILES = {
    "golf-cart": {
        "recipe_id": "generic-ackermann-ps5",
        "body": "generic_ackermann_golf_cart",
        "manifest": "recipes/generic_ackermann/asset-manifest.json",
        "model": "bodies/generic_ackermann_golf_cart/generated/model.minimal_world.xml",
    },
    "hunter": {
        "recipe_id": "hunter-ackermann-ps5",
        "body": "hunter_v2",
        "manifest": "recipes/hunter/asset-manifest.json",
        "model": "bodies/hunter_v2/generated/hunter_v2.minimal_world.xml",
    },
}
selected_profile = "golf-cart"


class RecipeError(RuntimeError):
    pass


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def business_pack_root() -> Path:
    override = os.getenv("HAKONIWA_BUSINESS_PACK_ROOT")
    return Path(override).expanduser().resolve() if override else repo_root().parent / "hakoniwa-business-pack"


def mbody_root() -> Path:
    override = os.getenv("HAKONIWA_MBODY_REGISTRY_ROOT")
    return Path(override).expanduser().resolve() if override else repo_root().parent / "hakoniwa-mbody-registry"


def profile() -> dict[str, str]:
    return PROFILES[selected_profile]


def asset_manifest() -> Path:
    return repo_root() / profile()["manifest"]


def generated_model() -> Path:
    return mbody_root() / profile()["model"]


def foundation_root() -> Path:
    return business_pack_root() / "work/foundation"


def install_root() -> Path:
    return foundation_root() / "install"


def foundation_python() -> Path:
    return install_root() / "python/bin/python3"


def workspace_root() -> Path:
    return business_pack_root() / "work/recipes" / profile()["recipe_id"]


def config_dir() -> Path:
    return workspace_root() / "config"


def logs_dir() -> Path:
    return workspace_root() / "logs"


def runtime_dir() -> Path:
    return workspace_root() / "runtime"


def build_dir() -> Path:
    # Reuse the repository's standard native build tree and its MuJoCo package
    # cache. A fresh build tree would download the same large SDK again.
    return repo_root() / "src/cmake-build"


def plant_binary() -> Path:
    return build_dir() / "examples/actuators/generic_ackermann/generic-ackermann-hakoniwa-asset"


def launcher_path() -> Path:
    return config_dir() / "launcher.json"


def session_path() -> Path:
    return runtime_dir() / "launcher-session.json"


def required(path: Path, label: str) -> Path:
    if not path.is_file():
        raise RecipeError(f"{label} not found: {path}")
    return path


def run(command: list[str], *, capture: bool = False) -> subprocess.CompletedProcess[str]:
    print("+", subprocess.list2cmdline(command), flush=True)
    return subprocess.run(
        command,
        cwd=repo_root(),
        text=True,
        capture_output=capture,
        check=False,
    )


def validate_asset_package() -> int:
    validator = required(
        repo_root() / "tools/validate_ackermann_asset_package.py",
        "Ackermann Asset Package validator",
    )
    python = required(foundation_python(), "Foundation Python")
    return run([str(python), str(validator), str(required(asset_manifest(), "Asset manifest"))]).returncode


def launcher_supports_runtime_cleanup(python: Path) -> bool:
    probe = subprocess.run(
        [
            str(python),
            "-c",
            "from hakoniwa_pdu.apps.launcher.model import LauncherSpec; "
            "raise SystemExit(0 if 'runtime' in LauncherSpec.model_fields else 1)",
        ],
        cwd=repo_root(),
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return probe.returncode == 0


def launcher_command(operation: str) -> list[str]:
    python = required(foundation_python(), "Foundation Python")
    if operation == "start":
        return [
            str(python),
            "-m",
            "hakoniwa_pdu.apps.launcher.hako_launcher",
            str(required(launcher_path(), "generated Launcher")),
            "--background",
            str(session_path()),
        ]
    return [
        str(python),
        "-m",
        "hakoniwa_pdu.apps.launcher.hako_launcher_ctl",
        operation,
        str(session_path()),
    ]


def configure(headless: bool) -> int:
    install = install_root()
    python = required(foundation_python(), "Foundation Python")
    required(foundation_root() / "config/cpp_core_config.json", "Foundation Core config")
    sender = required(repo_root() / "python/ackermann_gamepad.py", "PS5 sender")
    pdu_def = required(repo_root() / "config/ackermann-gamepad-pdudef-compact.json", "PDU definition")
    mapping = required(repo_root() / "recipes/generic_ackermann/ps5-controller-macos.json", "PS5 mapping")
    manifest = required(asset_manifest(), "Ackermann asset manifest")
    required(generated_model(), "MBody-generated Ackermann model; run forge first")
    if validate_asset_package() != 0:
        return 1
    config_dir().mkdir(parents=True, exist_ok=True)
    logs_dir().mkdir(parents=True, exist_ok=True)
    runtime_dir().mkdir(parents=True, exist_ok=True)

    launcher = {
        "version": "0.1",
        "defaults": {
            "cwd": str(repo_root()),
            "stdout": str(logs_dir() / "${asset}.out"),
            "stderr": str(logs_dir() / "${asset}.err"),
            "start_grace_sec": 2,
            "delay_sec": 1,
            "env": {
                "set": {
                    "HAKONIWA_CORE_ROOT": str(install),
                    "HAKONIWA_PDU_ENDPOINT_ROOT": str(install),
                    "HAKO_CONFIG_PATH": str(foundation_root() / "config/cpp_core_config.json"),
                    "PYTHONUNBUFFERED": "1",
                },
                "prepend": {
                    "PATH": [str(python.parent), str(install / "bin")],
                    "DYLD_LIBRARY_PATH": [str(install / "lib")],
                },
            },
        },
        "assets": [
            {
                "name": f"{selected_profile}-ackermann-plant",
                "activation_timing": "before_start",
                "command": str(plant_binary()),
                "args": [
                    "--manifest",
                    str(manifest),
                    *(["--no-viewer"] if headless else []),
                ],
                "delay_sec": 2,
            },
            {
                "name": f"{selected_profile}-ackermann-ps5-controller",
                "activation_timing": "after_start",
                "command": str(python),
                "args": [
                    str(sender),
                    "--pdu-def",
                    str(pdu_def),
                    "--rc-config",
                    str(mapping),
                ],
                "depends_on": [f"{selected_profile}-ackermann-plant"],
                "delay_sec": 1,
            },
        ],
    }
    if launcher_supports_runtime_cleanup(python):
        # This single-host Recipe exclusively owns the selected Foundation
        # runtime while active. The Launcher removes only its configured
        # mmap/lock files before assets start.
        launcher["runtime"] = {"cleanup_mmap_on_start": True}
    else:
        print(
            "[WARN] Installed Launcher has no runtime cleanup contract; "
            "upgrade the Foundation hakoniwa-pdu package before retrying a stale mmap session."
        )
    launcher_path().write_text(json.dumps(launcher, indent=2) + "\n", encoding="utf-8")
    print(f"Recipe workspace: {workspace_root()}")
    print(f"Launcher        : {launcher_path()}")
    print(f"Mode            : {'headless' if headless else 'viewer'}")
    return 0


def build() -> int:
    install = install_root()
    if not (install / "lib").is_dir():
        raise RecipeError(f"Foundation libraries not found: {install / 'lib'}")
    configure_command = [
        "cmake",
        "-S",
        str(repo_root() / "src"),
        "-B",
        str(build_dir()),
        f"-DHAKONIWA_INSTALL_PREFIX={install}",
        f"-DHAKONIWA_PDU_ENDPOINT_PREFIX={install}",
    ]
    if run(configure_command).returncode != 0:
        return 1
    return run([
        "cmake",
        "--build",
        str(build_dir()),
        "--target",
        "generic-ackermann-hakoniwa-asset",
        "-j4",
    ]).returncode


def forge() -> int:
    root = mbody_root()
    forge_tool = required(root / "tools/ackermann/forge.py", "MBody Ackermann Forge")
    python = required(foundation_python(), "Foundation Python")
    return run([
        str(python),
        str(forge_tool),
        profile()["body"],
    ]).returncode


def verify_forge() -> int:
    root = mbody_root()
    forge_tool = required(root / "tools/ackermann/forge.py", "MBody Ackermann Forge")
    python = required(foundation_python(), "Foundation Python")
    return run([
        str(python),
        str(forge_tool),
        profile()["body"],
        "--verify",
    ]).returncode


def validate_model() -> int:
    root = mbody_root()
    validator = required(root / "tools/ackermann/validate.py", "MBody Ackermann validator")
    python = required(foundation_python(), "Foundation Python")
    return run([
        str(python),
        str(validator),
        profile()["body"],
        "--report",
        str(repo_root() / f".hako/work/{selected_profile}/validation-report.json"),
    ]).returncode


def optimize_model(trials: int | None) -> int:
    root = mbody_root()
    optimizer = required(root / "tools/ackermann/optimize.py", "MBody Ackermann optimizer")
    python = required(foundation_python(), "Foundation Python")
    command = [
        str(python),
        str(optimizer),
        profile()["body"],
        "--output",
        str(repo_root() / f".hako/work/{selected_profile}/optimization"),
    ]
    if trials is not None:
        command.extend(["--trials", str(trials)])
    return run(command).returncode


def doctor() -> int:
    checks = [
        ("Ackermann runtime config", asset_manifest().parent / "ackermann-runtime.json"),
        ("Asset manifest", asset_manifest()),
        ("MBody Ackermann Forge", mbody_root() / "tools/ackermann/forge.py"),
        ("Asset Package validator", repo_root() / "tools/validate_ackermann_asset_package.py"),
        ("MBody-generated model", generated_model()),
        ("Foundation Python", foundation_python()),
        ("Foundation Core config", foundation_root() / "config/cpp_core_config.json"),
        ("hako-cmd", install_root() / "bin/hako-cmd"),
        ("PS5 sender", repo_root() / "python/ackermann_gamepad.py"),
        ("Ackermann plant", plant_binary()),
        ("generated Launcher", launcher_path()),
    ]
    failed = False
    for label, path in checks:
        ok = path.is_file()
        print(f"[{'OK' if ok else 'NG'}] {label}: {path}")
        failed = failed or not ok
    if failed:
        return 1
    return validate_asset_package()


def smoke() -> int:
    required(plant_binary(), "Ackermann plant")
    if validate_asset_package() != 0:
        return 1
    return run([
        str(plant_binary()),
        "--manifest",
        str(required(asset_manifest(), "Ackermann asset manifest")),
        "--smoke",
    ]).returncode


def start() -> int:
    required(plant_binary(), "Ackermann plant; run build first")
    session_path().parent.mkdir(parents=True, exist_ok=True)
    result = run(launcher_command("start"))
    if result.returncode == 0:
        print("Generic Ackermann PS5 Recipe is running.")
        print(f"Session: {session_path()}")
        print(f"Logs   : {logs_dir()}")
    return result.returncode


def control(operation: str) -> int:
    if not session_path().is_file():
        raise RecipeError(f"Launcher session not found: {session_path()}")
    return run(launcher_command(operation)).returncode


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description="Operate the generic Ackermann PS5 Recipe")
    result.add_argument(
        "command",
        choices=("forge", "verify-forge", "validate", "optimize", "configure", "build", "doctor", "smoke", "start", "status", "stop"),
    )
    result.add_argument("--headless", action="store_true", help="disable the MuJoCo Viewer")
    result.add_argument("--trials", type=int, help="override Ackermann optimization trial count")
    result.add_argument("--vehicle", choices=tuple(PROFILES), default="golf-cart", help="Ackermann Asset Package profile")
    return result


def main(argv: list[str] | None = None) -> int:
    global selected_profile
    args = parser().parse_args(argv)
    selected_profile = args.vehicle
    if args.command == "forge":
        return forge()
    if args.command == "verify-forge":
        return verify_forge()
    if args.command == "validate":
        return validate_model()
    if args.command == "optimize":
        return optimize_model(args.trials)
    if args.command == "configure":
        return configure(args.headless)
    if args.command == "build":
        return build()
    if args.command == "doctor":
        return doctor()
    if args.command == "smoke":
        return smoke()
    if args.command == "start":
        return start()
    return control("status" if args.command == "status" else "terminate")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RecipeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
