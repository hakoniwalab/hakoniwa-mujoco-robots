#!/usr/bin/env python3
"""Inspect a Hakoniwa MuJoCo asset manifest.

The manifest is intentionally a thin index. This inspector follows its
references and reports whether component JSON files point to MJCF objects and
PDU channels that exist in the referenced model, PDU definition, and endpoint.
"""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def resolve(base: Path, value: str) -> Path:
    path = Path(value)
    if not path.is_absolute():
        path = base.parent / path
    return path.resolve()


def collect_mjcf_names(path: Path) -> dict[str, set[str]]:
    names: dict[str, set[str]] = {
        "body": set(),
        "camera": set(),
        "joint": set(),
        "site": set(),
        "actuator": set(),
    }
    root = ET.parse(path).getroot()
    for elem in root.iter():
        name = elem.attrib.get("name")
        if not name:
            continue
        if elem.tag in names:
            names[elem.tag].add(name)
        if is_inside_actuator(elem, root):
            names["actuator"].add(name)
    return names


def is_inside_actuator(elem: ET.Element, root: ET.Element) -> bool:
    for actuator_root in root.findall("actuator"):
        if elem is actuator_root:
            return False
        if any(child is elem for child in actuator_root.iter()):
            return True
    return False


def load_pdu_channels(pdu_def_path: Path) -> dict[str, dict[str, str]]:
    pdu_def = load_json(pdu_def_path)
    type_files: dict[str, Path] = {}
    for entry in pdu_def.get("paths", []):
        if isinstance(entry, dict) and isinstance(entry.get("id"), str) and isinstance(entry.get("path"), str):
            type_files[entry["id"]] = resolve(pdu_def_path, entry["path"])

    channels: dict[str, dict[str, str]] = {}
    for robot in pdu_def.get("robots", []):
        if not isinstance(robot, dict):
            continue
        robot_name = robot.get("name")
        pdutypes_id = robot.get("pdutypes_id")
        if not isinstance(robot_name, str) or not isinstance(pdutypes_id, str):
            continue
        pdu_types_path = type_files.get(pdutypes_id)
        if pdu_types_path is None or not pdu_types_path.exists():
            channels[robot_name] = {}
            continue
        pdu_types = load_json(pdu_types_path)
        robot_channels: dict[str, str] = {}
        if isinstance(pdu_types, list):
            for item in pdu_types:
                if isinstance(item, dict) and isinstance(item.get("name"), str):
                    robot_channels[item["name"]] = str(item.get("type", ""))
        channels[robot_name] = robot_channels
    return channels


def load_endpoint_comm_channels(endpoint_path: Path) -> dict[str, set[str]]:
    endpoint = load_json(endpoint_path)
    comm_value = endpoint.get("comm")
    if not isinstance(comm_value, str):
        return {}
    comm_path = resolve(endpoint_path, comm_value)
    comm = load_json(comm_path)
    channels: dict[str, set[str]] = {}
    robots = comm.get("io", {}).get("robots", [])
    if isinstance(robots, list):
        for robot in robots:
            if not isinstance(robot, dict) or not isinstance(robot.get("name"), str):
                continue
            names: set[str] = set()
            for pdu in robot.get("pdu", []):
                if isinstance(pdu, dict) and isinstance(pdu.get("name"), str):
                    names.add(pdu["name"])
            channels[robot["name"]] = names
    return channels


def find_object(root: dict[str, Any], key: str) -> dict[str, Any]:
    value = root.get(key)
    return value if isinstance(value, dict) else {}


def get_pdu_config(config: dict[str, Any]) -> dict[str, Any]:
    return find_object(config, "pdu_config")


def get_mjcf_binding(config: dict[str, Any]) -> dict[str, Any]:
    return find_object(config, "mjcf_binding") or find_object(config, "RuntimeBinding")


def get_spec(config: dict[str, Any]) -> dict[str, Any]:
    return find_object(config, "spec")


def binding_refs(config: dict[str, Any]) -> list[tuple[str, str, str]]:
    refs: list[tuple[str, str, str]] = []
    binding = get_mjcf_binding(config)
    spec = get_spec(config)

    field_types = {
        "camera_name": "camera",
        "body_name": "body",
        "parent_body": "body",
        "source_body": "body",
        "exclude_body": "body",
        "source_site": "site",
        "site_name": "site",
        "actuator_name": "actuator",
        "freejoint_name": "joint",
    }
    for field, object_type in field_types.items():
        value = binding.get(field)
        if isinstance(value, str) and value:
            refs.append((object_type, value, f"mjcf_binding.{field}"))

    spec_joint = spec.get("joint_name")
    if isinstance(spec_joint, str) and spec_joint:
        refs.append(("joint", spec_joint, "spec.joint_name"))

    joints = binding.get("joints")
    if isinstance(joints, list):
        for index, item in enumerate(joints):
            if isinstance(item, dict):
                value = item.get("mjcf_joint")
                if isinstance(value, str) and value:
                    refs.append(("joint", value, f"mjcf_binding.joints[{index}].mjcf_joint"))

    transforms = binding.get("transforms")
    if isinstance(transforms, list):
        for index, item in enumerate(transforms):
            if isinstance(item, dict):
                value = item.get("source_body")
                if isinstance(value, str) and value:
                    refs.append(("body", value, f"mjcf_binding.transforms[{index}].source_body"))
    return refs


def status(ok: bool) -> str:
    return "OK  " if ok else "FAIL"


def inspect_manifest(path: Path) -> bool:
    manifest = load_json(path)
    if not isinstance(manifest, dict):
        print(f"FAIL manifest root must be object: {path}")
        return False

    model_path = resolve(path, manifest["model"])
    pdu_def_path = resolve(path, manifest["pdu_def"])
    endpoint_path = resolve(path, manifest["endpoint"])
    mjcf_names = collect_mjcf_names(model_path)
    pdu_channels = load_pdu_channels(pdu_def_path)
    comm_channels = load_endpoint_comm_channels(endpoint_path)

    print(f"Asset: {manifest.get('name', '<unnamed>')}")
    print(f"  model   : {model_path}")
    print(f"  pdu_def : {pdu_def_path}")
    print(f"  endpoint: {endpoint_path}")
    print()

    ok = True
    for component in manifest.get("components", []):
        if not isinstance(component, dict):
            print("FAIL component entry is not an object")
            ok = False
            continue
        component_id = str(component.get("id", "<no-id>"))
        config_path = resolve(path, str(component.get("config", "")))
        print(f"Component: {component_id} [{component.get('kind', '?')}/{component.get('type', '?')}]")
        print(f"  config: {config_path}")
        try:
            config = load_json(config_path)
        except Exception as exc:
            print(f"  FAIL config load: {exc}")
            ok = False
            continue

        pdu_config = get_pdu_config(config)
        pdu_name = pdu_config.get("pdu_name")
        message_type = pdu_config.get("message_type")
        update_rate = pdu_config.get("update_rate_hz")
        if isinstance(pdu_name, str) and pdu_name:
            print(f"  pdu_config: channel={pdu_name} type={message_type} rate_hz={update_rate}")
            pdu_robot = component.get("pdu_robot")
            if isinstance(pdu_robot, str) and pdu_robot:
                pdu_def_type = pdu_channels.get(pdu_robot, {}).get(pdu_name)
                in_pdu_def = pdu_def_type is not None
                type_matches = (
                    not isinstance(message_type, str) or
                    not message_type or
                    pdu_def_type == message_type
                )
                in_comm = pdu_name in comm_channels.get(pdu_robot, set())
                print(f"  {status(in_pdu_def)}pdu_def: {pdu_robot}/{pdu_name}")
                if in_pdu_def:
                    print(f"  {status(type_matches)}pdu type: {pdu_def_type}")
                print(f"  {status(in_comm)}comm   : {pdu_robot}/{pdu_name}")
                ok = in_pdu_def and type_matches and in_comm and ok
            else:
                print("  INFO pdu_def/comm check skipped: component has no pdu_robot")
        else:
            print("  INFO no pdu_config.pdu_name")

        refs = binding_refs(config)
        if refs:
            for object_type, name, source in refs:
                found = name in mjcf_names.get(object_type, set())
                print(f"  {status(found)}mjcf {object_type}: {name} ({source})")
                ok = found and ok
        else:
            print("  INFO no known mjcf binding references")
        print()
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect a Hakoniwa MuJoCo asset manifest.")
    parser.add_argument("manifest", help="Hakoniwa asset manifest JSON path")
    args = parser.parse_args()
    try:
        return 0 if inspect_manifest(Path(args.manifest).resolve()) else 1
    except Exception as exc:
        print(f"FAIL inspect: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
