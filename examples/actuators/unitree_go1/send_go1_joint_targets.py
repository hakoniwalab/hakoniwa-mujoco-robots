#!/usr/bin/env python3
"""Send 12-joint position targets to the Unitree Go1 Hakoniwa asset."""

from __future__ import annotations

import argparse
import math
import threading
import time
from pathlib import Path

import hakopy
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_JointState import pdu_to_py_JointState
from hakoniwa_pdu.pdu_msgs.std_msgs.pdu_conv_Float64MultiArray import (
    py_to_pdu_Float64MultiArray,
)
from hakoniwa_pdu.pdu_msgs.std_msgs.pdu_pytype_Float64MultiArray import (
    Float64MultiArray,
)
from hakoniwa_pdu_endpoint.c_endpoint import Endpoint, PduKey


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_ENDPOINT_CONFIG = REPO_ROOT / "config/endpoint/go1_joint_endpoint.json"
DEFAULT_PDU_DEF = REPO_ROOT / "config/go1-joint-pdudef-compact.json"

JOINT_ORDER = [
    "FR_hip",
    "FR_thigh",
    "FR_calf",
    "FL_hip",
    "FL_thigh",
    "FL_calf",
    "RR_hip",
    "RR_thigh",
    "RR_calf",
    "RL_hip",
    "RL_thigh",
    "RL_calf",
]
HOME_TARGETS = [0.0, 0.9, -1.8] * 4


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send std_msgs/Float64MultiArray joint targets to Go1JointAsset."
    )
    parser.add_argument("--endpoint-config", default=str(DEFAULT_ENDPOINT_CONFIG))
    parser.add_argument("--pdu-def", default=str(DEFAULT_PDU_DEF))
    parser.add_argument("--sender-asset-name", default="Go1JointSender")
    parser.add_argument("--target-robot-name", default="Go1JointAsset")
    parser.add_argument("--command-pdu-name", default="joint_position_targets")
    parser.add_argument("--joint-state-pdu-name", default="joint_states")
    parser.add_argument("--endpoint-name", default="go1_joint_sender")
    parser.add_argument("--delta-usec", type=int, default=20_000)
    parser.add_argument("--send-rate-hz", type=float, default=20.0)
    parser.add_argument("--duration-sec", type=float, default=4.0)
    parser.add_argument("--amplitude", type=float, default=0.12)
    parser.add_argument("--frequency-hz", type=float, default=0.5)
    parser.add_argument(
        "--no-read-joint-state",
        action="store_true",
        help="Disable reading sensor_msgs/JointState from the C++ asset.",
    )
    return parser.parse_args()


def make_targets(elapsed_sec: float, amplitude: float, frequency_hz: float) -> list[float]:
    targets = list(HOME_TARGETS)
    offset = amplitude * math.sin(2.0 * math.pi * frequency_hz * elapsed_sec)
    targets[JOINT_ORDER.index("FR_thigh")] += offset
    targets[JOINT_ORDER.index("RL_thigh")] += offset
    return targets


def make_payload(values: list[float]) -> bytes:
    msg = Float64MultiArray()
    msg.data = [float(value) for value in values]
    return bytes(py_to_pdu_Float64MultiArray(msg))


def compact_joint_state(joint_state) -> str:
    wanted = {"FR_thigh", "RL_thigh"}
    parts: list[str] = []
    for index, name in enumerate(joint_state.name):
        if name not in wanted:
            continue
        position = joint_state.position[index] if index < len(joint_state.position) else 0.0
        velocity = joint_state.velocity[index] if index < len(joint_state.velocity) else 0.0
        parts.append(f"{name}:pos={position:.3f},vel={velocity:.3f}")
    return " | ".join(parts)


def main() -> int:
    args = parse_args()
    endpoint_config = str(Path(args.endpoint_config).resolve())
    pdu_def = str(Path(args.pdu_def).resolve())
    command_key = PduKey(args.target_robot_name, args.command_pdu_name)
    joint_state_key = PduKey(args.target_robot_name, args.joint_state_pdu_name)
    endpoint = Endpoint(args.endpoint_name, "inout")
    shutdown = threading.Event()
    callback_state = {"result": 0}

    def on_initialize(_context):
        try:
            endpoint.post_start()
        except Exception as exc:
            print(f"[ERROR] endpoint.post_start() failed: {exc}")
            callback_state["result"] = 1
            return 1
        return 0

    def on_reset(_context):
        return 0

    def on_manual_timing_control(_context):
        try:
            send_interval_usec = max(1, int(1_000_000.0 / args.send_rate_hz))
            elapsed_usec = 0
            next_send_usec = 0
            count = 0
            latest_joint_state = None
            joint_state_size = 0
            skipped_invalid_joint_state = 0
            if not args.no_read_joint_state:
                joint_state_size = endpoint.get_pdu_size(joint_state_key)
            print(
                "[INFO] Go1 joint sender callback started: "
                f"robot={args.target_robot_name} command={args.command_pdu_name} "
                f"order={','.join(JOINT_ORDER)}"
            )
            while not shutdown.is_set() and elapsed_usec <= int(args.duration_sec * 1_000_000):
                if joint_state_size > 0:
                    raw_joint_state = endpoint.recv_by_name(joint_state_key, joint_state_size)
                    if raw_joint_state:
                        if isinstance(raw_joint_state, tuple):
                            raw_joint_state = raw_joint_state[0]
                        try:
                            latest_joint_state = pdu_to_py_JointState(raw_joint_state)
                        except Exception as exc:
                            skipped_invalid_joint_state += 1
                            if skipped_invalid_joint_state == 1:
                                print(
                                    "[INFO] Skipping invalid initial joint_states PDU "
                                    f"until publisher writes the first frame: {exc}"
                                )

                if elapsed_usec >= next_send_usec:
                    t = elapsed_usec / 1_000_000.0
                    targets = make_targets(t, args.amplitude, args.frequency_hz)
                    endpoint.send_by_name(command_key, make_payload(targets))
                    if (count % 20) == 0:
                        message = (
                            "sent joint_position_targets "
                            f"FR_thigh={targets[1]:.3f} RL_thigh={targets[10]:.3f}"
                        )
                        if latest_joint_state is not None:
                            message += " | joint_states " + compact_joint_state(latest_joint_state)
                        print(message)
                    count += 1
                    next_send_usec += send_interval_usec

                if hakopy.usleep(args.delta_usec) is False:
                    break
                elapsed_usec += args.delta_usec
                time.sleep(args.delta_usec / 1_000_000.0)

            endpoint.send_by_name(command_key, make_payload(HOME_TARGETS))
            shutdown.set()
        except Exception as exc:
            print(f"[ERROR] Go1 joint sender callback failed: {exc}")
            callback_state["result"] = 1
            shutdown.set()
        return 0

    callback = {
        "on_initialize": on_initialize,
        "on_simulation_step": None,
        "on_manual_timing_control": on_manual_timing_control,
        "on_reset": on_reset,
    }

    endpoint.open(endpoint_config)
    endpoint.start()

    ret = hakopy.asset_register(
        args.sender_asset_name,
        pdu_def,
        callback,
        args.delta_usec,
        hakopy.HAKO_ASSET_MODEL_CONTROLLER,
    )
    if ret is False:
        print("[ERROR] hakopy.asset_register() failed")
        endpoint.stop()
        endpoint.close()
        return 1

    print("[INFO] Go1 joint sender is registered.")
    print("[INFO] Run `hako-cmd start` to begin sending commands.")
    try:
        started = hakopy.start()
        print(f"[INFO] hakopy.start() returns {started}")
    except KeyboardInterrupt:
        print("[INFO] Interrupted. Exiting.")
    finally:
        shutdown.set()
        endpoint.stop()
        endpoint.close()
    return int(callback_state["result"])


if __name__ == "__main__":
    raise SystemExit(main())
