#!/usr/bin/env python3
"""Validate Hakoniwa MuJoCo user-authored assets.

The checks are intentionally lightweight:

- JSON files are always parsed.
- JSON Schema validation is used when the optional `jsonschema` package exists.
- MJCF files are loaded when the optional Python `mujoco` package exists.

This lets beginners run one command before writing C++ integration code, while
keeping the repository usable in environments that do not have Python-side
MuJoCo or jsonschema installed.
"""

from __future__ import annotations

import argparse
import json
import sys
import warnings
from pathlib import Path
from typing import Any
from urllib.parse import urlparse


def _load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _repo_root_for(path: Path) -> Path:
    current = path.resolve()
    if current.is_file():
        current = current.parent
    for candidate in [current, *current.parents]:
        if (candidate / "config").is_dir() and (candidate / "models").is_dir():
            return candidate
    return Path.cwd().resolve()


def _resolve_schema_path(json_path: Path, value: str) -> Path | None:
    if value.startswith("http://") or value.startswith("https://"):
        parsed = urlparse(value)
        schema_name = Path(parsed.path).name
        if parsed.netloc == "hakoniwa.dev" and parsed.path.startswith("/schemas/") and schema_name:
            root = _repo_root_for(json_path)
            for schema_dir in (
                root / "config" / "assets" / "schema",
                root / "config" / "sensors" / "schema",
                root / "config" / "actuator" / "schema",
            ):
                candidate = schema_dir / schema_name
                if candidate.exists():
                    return candidate.resolve()
        return None
    candidate = Path(value)
    if not candidate.is_absolute():
        candidate = json_path.parent / candidate
    return candidate.resolve()


def _load_local_schema_store(root: Path) -> dict[str, Any]:
    store: dict[str, Any] = {}
    for schema_dir in (
        root / "config" / "assets" / "schema",
        root / "config" / "sensors" / "schema",
        root / "config" / "actuator" / "schema",
    ):
        if not schema_dir.is_dir():
            continue
        for schema_path in schema_dir.glob("*.schema.json"):
            try:
                schema = _load_json(schema_path)
            except Exception:
                continue
            store[schema_path.as_uri()] = schema
            store[schema_path.name] = schema
            schema_id = schema.get("$id") if isinstance(schema, dict) else None
            if isinstance(schema_id, str) and schema_id:
                store[schema_id] = schema
    return store


def validate_json(path: Path) -> bool:
    try:
        data = _load_json(path)
    except Exception as exc:
        print(f"FAIL JSON parse: {path}: {exc}")
        return False

    print(f"OK   JSON parse: {path}")

    schema_value = data.get("$schema") if isinstance(data, dict) else None
    if not isinstance(schema_value, str) or schema_value == "":
        print(f"WARN JSON schema: {path}: no $schema field")
        return True

    schema_path = _resolve_schema_path(path, schema_value)
    if schema_path is None:
        print(f"WARN JSON schema: {path}: remote schema is not checked: {schema_value}")
        return True
    if not schema_path.exists():
        print(f"FAIL JSON schema: {path}: schema file not found: {schema_path}")
        return False

    try:
        import jsonschema
    except ImportError:
        print(f"WARN JSON schema: {path}: install jsonschema to validate against {schema_path}")
        return True

    try:
        schema = _load_json(schema_path)
        store = _load_local_schema_store(_repo_root_for(path))
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", DeprecationWarning)
            resolver = jsonschema.validators.RefResolver(
                base_uri=schema_path.parent.as_uri() + "/",
                referrer=schema,
                store=store,
            )
        validator_cls = jsonschema.validators.validator_for(schema)
        validator_cls.check_schema(schema)
        validator = validator_cls(schema, resolver=resolver)
        errors = sorted(validator.iter_errors(data), key=lambda e: list(e.path))
    except Exception as exc:
        print(f"FAIL JSON schema: {path}: failed to prepare schema {schema_path}: {exc}")
        return False

    if errors:
        print(f"FAIL JSON schema: {path}: {len(errors)} error(s)")
        for error in errors[:20]:
            location = ".".join(str(p) for p in error.path) or "<root>"
            print(f"  - {location}: {error.message}")
        if len(errors) > 20:
            print(f"  ... {len(errors) - 20} more error(s)")
        return False

    print(f"OK   JSON schema: {path} -> {schema_path}")
    return True


def _resolve_manifest_path(manifest_path: Path, value: str) -> Path:
    candidate = Path(value)
    if not candidate.is_absolute():
        candidate = manifest_path.parent / candidate
    return candidate.resolve()


def validate_manifest(path: Path) -> bool:
    ok = validate_json(path)
    try:
        manifest = _load_json(path)
    except Exception as exc:
        print(f"FAIL manifest parse: {path}: {exc}")
        return False

    if not isinstance(manifest, dict):
        print(f"FAIL manifest: {path}: root must be an object")
        return False

    model = manifest.get("model")
    if isinstance(model, str) and model:
        model_path = _resolve_manifest_path(path, model)
        if model_path.exists():
            ok = validate_mjcf(model_path) and ok
        else:
            print(f"FAIL manifest MJCF path: {model_path}: not found")
            ok = False

    for key in ("pdu_def", "endpoint"):
        value = manifest.get(key)
        if isinstance(value, str) and value:
            json_path = _resolve_manifest_path(path, value)
            if json_path.exists():
                ok = validate_json(json_path) and ok
            else:
                print(f"FAIL manifest {key} path: {json_path}: not found")
                ok = False

    components = manifest.get("components", [])
    if isinstance(components, list):
        for index, component in enumerate(components):
            if not isinstance(component, dict):
                print(f"FAIL manifest components[{index}]: must be an object")
                ok = False
                continue
            config = component.get("config")
            if not isinstance(config, str) or not config:
                print(f"FAIL manifest components[{index}]: missing config")
                ok = False
                continue
            config_path = _resolve_manifest_path(path, config)
            if config_path.exists():
                ok = validate_json(config_path) and ok
            else:
                component_id = component.get("id", index)
                print(f"FAIL manifest component config: {component_id}: {config_path}: not found")
                ok = False
    return ok


def validate_mjcf(path: Path) -> bool:
    try:
        import mujoco
    except ImportError:
        print(f"WARN MJCF load: {path}: install mujoco Python package to validate MJCF loading")
        return True

    try:
        model = mujoco.MjModel.from_xml_path(str(path))
    except Exception as exc:
        print(f"FAIL MJCF load: {path}: {exc}")
        return False

    print(
        "OK   MJCF load: "
        f"{path} bodies={model.nbody} joints={model.njnt} geoms={model.ngeom} "
        f"actuators={model.nu} cameras={model.ncam} sites={model.nsite}"
    )
    return True


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate Hakoniwa MuJoCo MJCF and JSON assets before C++ integration.",
    )
    parser.add_argument("--manifest", action="append", default=[], help="Hakoniwa asset manifest JSON file")
    parser.add_argument("--mjcf", action="append", default=[], help="MJCF XML file to load")
    parser.add_argument("--json", action="append", default=[], help="JSON config file to parse and validate")
    args = parser.parse_args()

    paths = [Path(p) for p in args.manifest + args.mjcf + args.json]
    if not paths:
        parser.error("pass at least one --manifest, --mjcf, or --json path")

    ok = True
    for raw in args.manifest:
        path = Path(raw)
        if not path.exists():
            print(f"FAIL manifest path: {path}: not found")
            ok = False
            continue
        ok = validate_manifest(path) and ok

    for raw in args.mjcf:
        path = Path(raw)
        if not path.exists():
            print(f"FAIL MJCF path: {path}: not found")
            ok = False
            continue
        ok = validate_mjcf(path) and ok

    for raw in args.json:
        path = Path(raw)
        if not path.exists():
            print(f"FAIL JSON path: {path}: not found")
            ok = False
            continue
        ok = validate_json(path) and ok

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
