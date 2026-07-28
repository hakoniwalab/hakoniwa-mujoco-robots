#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, Mapping


DEFAULT_MANIFEST = "hakoniwa-build.yaml"


class HakoError(RuntimeError):
    pass


class ConfigError(HakoError):
    pass


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _powershell() -> str:
    for name in ("powershell.exe", "pwsh", "powershell"):
        found = shutil.which(name)
        if found:
            return found
    raise HakoError("PowerShell was not found on PATH")


def _parse_scalar(text: str) -> Any:
    value = text.strip()
    if value == "":
        return {}
    if value.startswith(('"', "'")):
        if len(value) < 2 or value[-1] != value[0]:
            raise ConfigError(f"unterminated quoted scalar: {value}")
        if value[0] == '"':
            try:
                return json.loads(value)
            except json.JSONDecodeError as exc:
                raise ConfigError(f"invalid quoted scalar: {value}") from exc
        return value[1:-1].replace("''", "'")
    try:
        return int(value)
    except ValueError:
        return value


def load_simple_yaml(path: Path) -> Dict[str, Any]:
    """Load the tiny mapping/scalar subset used by build manifest v1."""
    root: Dict[str, Any] = {}
    stack: list[tuple[int, Dict[str, Any]]] = [(-1, root)]
    for lineno, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if "\t" in raw:
            raise ConfigError(f"{path}:{lineno}: tabs are not allowed")
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        stripped = line.lstrip(" ")
        indent = len(line) - len(stripped)
        if ":" not in stripped:
            raise ConfigError(f"{path}:{lineno}: expected 'key: value'")
        key, raw_value = stripped.split(":", 1)
        key = key.strip()
        if not key:
            raise ConfigError(f"{path}:{lineno}: empty key")
        while stack and indent <= stack[-1][0]:
            stack.pop()
        if not stack:
            raise ConfigError(f"{path}:{lineno}: invalid indentation")
        parent = stack[-1][1]
        if key in parent:
            raise ConfigError(f"{path}:{lineno}: duplicate key: {key}")
        parsed = _parse_scalar(raw_value)
        parent[key] = parsed
        if isinstance(parsed, dict):
            stack.append((indent, parsed))
    return root


def resolve_config(raw: Mapping[str, Any]) -> Dict[str, Any]:
    expected_root = {"version", "build"}
    unknown = sorted(set(raw) - expected_root)
    missing = sorted(expected_root - set(raw))
    if unknown:
        raise ConfigError(f"unknown key(s) under root: {', '.join(unknown)}")
    if missing:
        raise ConfigError(f"missing required key(s) under root: {', '.join(missing)}")
    if raw["version"] != 1:
        raise ConfigError("version must be 1")

    build = raw["build"]
    if not isinstance(build, Mapping):
        raise ConfigError("build must be a mapping")
    expected_build = {"dir"}
    unknown = sorted(set(build) - expected_build)
    missing = sorted(expected_build - set(build))
    if unknown:
        raise ConfigError(f"unknown key(s) under build: {', '.join(unknown)}")
    if missing:
        raise ConfigError(f"missing required key(s) under build: {', '.join(missing)}")
    build_dir = build["dir"]
    if not isinstance(build_dir, str) or not build_dir.strip():
        raise ConfigError("build.dir must be a non-empty string")
    if build_dir != "auto" and Path(build_dir).expanduser().is_absolute():
        raise ConfigError("build.dir must be 'auto' or a repository-relative path")
    return {"version": 1, "build": {"dir": build_dir}}


def resolve_manifest_path(value: str | None, root: Path) -> Path:
    if value is None:
        path = root / DEFAULT_MANIFEST
    else:
        path = Path(value)
        if not path.is_absolute():
            path = (Path.cwd() / path).resolve()
    if not path.is_file():
        raise ConfigError(f"build manifest not found: {path}")
    return path


def _resolved_build_dir(build_dir: str, root: Path) -> Path | None:
    if build_dir == "auto":
        return None
    return (root / build_dir).resolve()


def resolve_command(
    command: str,
    native_args: list[str],
    *,
    build_dir: str = "auto",
) -> tuple[list[str], dict[str, str]]:
    root = _repo_root()
    env = dict(os.environ)
    resolved_build_dir = _resolved_build_dir(build_dir, root)

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
        if resolved_build_dir is not None:
            cmd.extend(["-BuildDirName", build_dir])
        cmd.extend(native_args)
        return cmd, env

    if command == "doctor":
        script = root / "doctor.bash"
    elif command == "build":
        script = root / "build.bash"
    else:
        raise HakoError(f"unsupported command: {command}")
    if resolved_build_dir is not None:
        env["HAKO_BUILD_DIR"] = str(resolved_build_dir)
    return ["bash", str(script), *native_args], env


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="OS-independent entry point for Hakoniwa MuJoCo Robots"
    )
    parser.add_argument("--dry-run", action="store_true", help="print the delegated command only")
    parser.add_argument("command", choices=["doctor", "build"])
    parser.add_argument(
        "--config",
        default=None,
        help=f"build manifest (default: repository root/{DEFAULT_MANIFEST})",
    )
    return parser


def parse_cli(argv: list[str] | None = None) -> tuple[argparse.Namespace, list[str]]:
    values = list(sys.argv[1:] if argv is None else argv)
    if "--" in values:
        separator = values.index("--")
        own_args = values[:separator]
        native_args = values[separator + 1 :]
    else:
        own_args = values
        native_args = []
    return create_parser().parse_args(own_args), native_args


def main(argv: list[str] | None = None) -> int:
    args, native_args = parse_cli(argv)
    root = _repo_root()
    manifest = resolve_manifest_path(args.config, root)
    cfg = resolve_config(load_simple_yaml(manifest))
    build_dir = cfg["build"]["dir"]

    command, env = resolve_command(args.command, native_args, build_dir=build_dir)
    print(f"Build manifest: {manifest}")
    print(f"Build directory: {build_dir}")
    print(">", subprocess.list2cmdline(command))
    if args.dry_run:
        return 0

    completed = subprocess.run(command, cwd=root, env=env, check=False)
    return completed.returncode


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except HakoError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
