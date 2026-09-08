#!/usr/bin/env python3
"""Operate the model-only Hunter V2 Ackermann Forge Recipe."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import sys


BODY = "hunter_v2"
selected_state_dir: Path | None = None


def state_dir() -> Path:
    return selected_state_dir or repo_root() / ".hako"


class RecipeError(RuntimeError):
    pass


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def mbody_root() -> Path:
    override = os.getenv("HAKONIWA_MBODY_REGISTRY_ROOT")
    return Path(override).expanduser().resolve() if override else repo_root().parent / "hakoniwa-mbody-registry"


def business_pack_root() -> Path:
    override = os.getenv("HAKONIWA_BUSINESS_PACK_ROOT")
    return Path(override).expanduser().resolve() if override else repo_root().parent / "hakoniwa-business-pack"


def foundation_python() -> Path:
    configured = os.getenv("HAKONIWA_HOME", "").strip()
    prefix = Path(configured).expanduser().resolve() if configured else business_pack_root() / "work/foundation/install"
    return prefix / "python/bin/python3"


def required(path: Path, label: str) -> Path:
    if not path.is_file():
        raise RecipeError(f"{label} not found: {path}")
    return path


def command(verify: bool) -> int:
    root = mbody_root()
    python = required(foundation_python(), "Foundation Python")
    forge = required(root / "tools/ackermann/forge.py", "MBody Ackermann Forge")
    args = [str(python), str(forge), BODY]
    if verify:
        args.append("--verify")
    print("+", subprocess.list2cmdline(args), flush=True)
    return subprocess.run(args, cwd=repo_root(), check=False).returncode


def validate_model() -> int:
    root = mbody_root()
    python = required(foundation_python(), "Foundation Python")
    validator = required(root / "tools/ackermann/validate.py", "MBody Ackermann validator")
    report = state_dir() / "work/hunter/validation-report.json"
    args = [str(python), str(validator), BODY, "--report", str(report)]
    print("+", subprocess.list2cmdline(args), flush=True)
    return subprocess.run(args, cwd=repo_root(), check=False).returncode


def optimize_model(trials: int | None) -> int:
    root = mbody_root()
    python = required(foundation_python(), "Foundation Python")
    optimizer = required(root / "tools/ackermann/optimize.py", "MBody Ackermann optimizer")
    output = state_dir() / "work/hunter/optimization"
    args = [str(python), str(optimizer), BODY, "--output", str(output)]
    if trials is not None:
        args.extend(["--trials", str(trials)])
    print("+", subprocess.list2cmdline(args), flush=True)
    return subprocess.run(args, cwd=repo_root(), check=False).returncode


def runtime_command(operation: str, headless: bool) -> int:
    runtime_recipe = required(
        repo_root() / "tools/recipe/generic_ackermann.py",
        "Generic Ackermann runtime Recipe",
    )
    args = [sys.executable, str(runtime_recipe), operation, "--vehicle", "hunter"]
    if selected_state_dir is not None:
        args.extend(["--state-dir", str(selected_state_dir)])
    if headless:
        args.append("--headless")
    print("+", subprocess.list2cmdline(args), flush=True)
    return subprocess.run(args, cwd=repo_root(), check=False).returncode


def doctor() -> int:
    root = mbody_root()
    checks = [
        ("Hunter source definition", root / "sources/hunter_v2.yaml"),
        ("Hunter Ackermann Recipe", root / "bodies/hunter_v2/config/ackermann-forge.yaml"),
        ("MBody Ackermann Forge", root / "tools/ackermann/forge.py"),
        ("Foundation Python", foundation_python()),
        ("generated Hunter model", root / "bodies/hunter_v2/generated/hunter_v2.minimal_world.xml"),
        ("Hunter Asset Manifest", repo_root() / "recipes/hunter/asset-manifest.json"),
        ("Hunter runtime config", repo_root() / "recipes/hunter/ackermann-runtime.json"),
        ("Asset Package validator", repo_root() / "tools/validate_ackermann_asset_package.py"),
    ]
    failed = False
    for label, path in checks:
        ok = path.is_file()
        print(f"[{'OK' if ok else 'NG'}] {label}: {path}")
        failed = failed or not ok
    if failed:
        return 1
    validator = repo_root() / "tools/validate_ackermann_asset_package.py"
    manifest = repo_root() / "recipes/hunter/asset-manifest.json"
    args = [str(foundation_python()), str(validator), str(manifest)]
    print("+", subprocess.list2cmdline(args), flush=True)
    return subprocess.run(args, cwd=repo_root(), check=False).returncode


def main() -> int:
    global selected_state_dir
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state-dir", help="validation/optimization state directory (default: repository root/.hako; relative to cwd)")
    parser.add_argument(
        "command",
        choices=("forge", "verify-forge", "validate", "optimize", "doctor", "configure", "build", "smoke", "start", "status", "stop"),
    )
    parser.add_argument("--trials", type=int, help="override Ackermann optimization trial count")
    parser.add_argument("--headless", action="store_true", help="disable the MuJoCo Viewer")
    args = parser.parse_args()
    selected_state_dir = Path(args.state_dir).expanduser().resolve() if args.state_dir else None
    if args.command == "forge":
        return command(False)
    if args.command == "verify-forge":
        return command(True)
    if args.command == "validate":
        return validate_model()
    if args.command == "optimize":
        return optimize_model(args.trials)
    if args.command == "doctor":
        return doctor()
    return runtime_command(args.command, args.headless)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RecipeError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
