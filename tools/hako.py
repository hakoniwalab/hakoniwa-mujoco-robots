#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


class HakoError(RuntimeError):
    pass


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _native_args(values: list[str]) -> list[str]:
    if values and values[0] == "--":
        return values[1:]
    return values


def _powershell() -> str:
    for name in ("powershell.exe", "pwsh", "powershell"):
        found = shutil.which(name)
        if found:
            return found
    raise HakoError("PowerShell was not found on PATH")


def resolve_command(command: str, native_args: list[str]) -> list[str]:
    root = _repo_root()
    args = _native_args(native_args)

    if sys.platform == "win32":
        script = root / "build-win.ps1"
        cmd = [
            _powershell(),
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(script),
        ]
        if command == "doctor":
            cmd.append("-DoctorOnly")
        cmd.extend(args)
        return cmd

    if command == "doctor":
        script = root / "doctor.bash"
    elif command == "build":
        script = root / "build.bash"
    else:
        raise HakoError(f"unsupported command: {command}")
    return ["bash", str(script), *args]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="OS-independent entry point for Hakoniwa MuJoCo Robots"
    )
    parser.add_argument("--dry-run", action="store_true", help="print the delegated command only")
    parser.add_argument("command", choices=["doctor", "build"])
    parser.add_argument(
        "native_args",
        nargs=argparse.REMAINDER,
        help="arguments passed to the platform-native script; use '--' before them",
    )
    args = parser.parse_args(argv)

    command = resolve_command(args.command, args.native_args)
    print(">", subprocess.list2cmdline(command))
    if args.dry_run:
        return 0

    completed = subprocess.run(command, cwd=_repo_root(), check=False)
    return completed.returncode


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except HakoError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
