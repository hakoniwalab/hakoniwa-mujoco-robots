#!/usr/bin/env python3
"""Validate an Ackermann Asset Manifest, runtime contract, bindings, and MJCF."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import jsonschema
import mujoco


REPO_ROOT = Path(__file__).resolve().parents[1]
ASSET_SCHEMA = REPO_ROOT / "config/assets/schema/hakoniwa-asset-manifest.schema.json"
RUNTIME_SCHEMA = REPO_ROOT / "config/assets/schema/ackermann-runtime.schema.json"
ROLES = ("steering_left", "steering_right", "drive_left", "drive_right")


class PackageError(RuntimeError):
    pass


def load_json(path: Path) -> dict:
    if not path.is_file():
        raise PackageError(f"file not found: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise PackageError(f"JSON root must be an object: {path}")
    return value


def resolve(base: Path, value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else (base / path).resolve()


def model_has(model: mujoco.MjModel, object_type: mujoco.mjtObj, name: str) -> bool:
    return mujoco.mj_name2id(model, object_type, name) >= 0


def validate_package(manifest_path: Path) -> None:
    manifest_path = manifest_path.resolve()
    manifest = load_json(manifest_path)
    jsonschema.Draft202012Validator(load_json(ASSET_SCHEMA)).validate(manifest)
    if "runtime_config" not in manifest:
        raise PackageError("Ackermann Asset Manifest requires runtime_config")
    base = manifest_path.parent
    runtime_path = resolve(base, manifest["runtime_config"])
    runtime = load_json(runtime_path)
    jsonschema.Draft202012Validator(load_json(RUNTIME_SCHEMA)).validate(runtime)

    model_path = resolve(base, manifest["model"])
    pdu_path = resolve(base, manifest["pdu_def"])
    endpoint_path = resolve(base, manifest["endpoint"])
    for label, path in (("model", model_path), ("pdu_def", pdu_path), ("endpoint", endpoint_path)):
        if not path.is_file():
            raise PackageError(f"manifest {label} does not exist: {path}")

    components = {item["id"]: item for item in manifest["components"]}
    role_components = runtime["bindings"]["components"]
    missing_components = sorted(set(role_components.values()) - set(components))
    if missing_components:
        raise PackageError("runtime roles reference missing components: " + ", ".join(missing_components))

    model = mujoco.MjModel.from_xml_path(str(model_path))
    joint_names = runtime["bindings"]["joints"]
    actuator_names = runtime["bindings"]["actuators"]
    required_joints = [runtime["bindings"]["base_freejoint"], *(joint_names[role] for role in ROLES)]
    missing_joints = [name for name in required_joints if not model_has(model, mujoco.mjtObj.mjOBJ_JOINT, name)]
    missing_actuators = [name for name in actuator_names.values() if not model_has(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)]
    if missing_joints or missing_actuators:
        raise PackageError(
            "MJCF semantic bindings missing: joints=" + repr(missing_joints) +
            " actuators=" + repr(missing_actuators)
        )

    for role in ROLES:
        component = components[role_components[role]]
        component_path = resolve(base, component["config"])
        config = load_json(component_path)
        actual_joint = config.get("spec", {}).get("joint_name")
        actual_actuator = config.get("mjcf_binding", {}).get("actuator_name")
        if actual_joint != joint_names[role]:
            raise PackageError(
                f"{role} component joint mismatch: runtime={joint_names[role]} component={actual_joint}"
            )
        if actual_actuator != actuator_names[role]:
            raise PackageError(
                f"{role} component actuator mismatch: expected={actuator_names[role]} component={actual_actuator}"
            )
    print(
        f"[OK] Ackermann Asset Package: {manifest['name']} "
        f"model={model_path.name} nq={model.nq} nv={model.nv} nu={model.nu}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifests", nargs="+", type=Path)
    args = parser.parse_args()
    for manifest in args.manifests:
        validate_package(manifest)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (PackageError, OSError, ValueError, json.JSONDecodeError, jsonschema.ValidationError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
