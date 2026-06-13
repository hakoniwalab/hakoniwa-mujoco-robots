#!/usr/bin/env python3
"""Send joint actuator commands through Hakoniwa PDU.

This script is a Hakoniwa controller asset. It writes std_msgs/Float64
commands to the joint actuator PDU robot/channel names used by the C++ example.
"""

from __future__ import annotations

import argparse
import math
import threading
from pathlib import Path

import time
import hakopy
from hakoniwa_pdu.pdu_msgs.std_msgs.pdu_conv_Float64 import py_to_pdu_Float64
from hakoniwa_pdu.pdu_msgs.std_msgs.pdu_pytype_Float64 import Float64
from hakoniwa_pdu_endpoint.c_endpoint import Endpoint, PduKey


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_ENDPOINT_CONFIG = REPO_ROOT / "config/endpoint/joint_actuator_endpoint.json"
DEFAULT_PDU_DEF = REPO_ROOT / "config/joint-actuator-pdudef-compact.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send std_msgs/Float64 joint commands to the Hakoniwa actuator example."
    )
    parser.add_argument(
        "--endpoint-config",
        default=str(DEFAULT_ENDPOINT_CONFIG),
        help="Endpoint config JSON path.",
    )
    parser.add_argument(
        "--pdu-def",
        default=str(DEFAULT_PDU_DEF),
        help="PDU definition JSON path passed to hakopy.asset_register().",
    )
    parser.add_argument(
        "--sender-asset-name",
        default="JointActuatorSender",
        help="Hakoniwa asset name for this Python sender.",
    )
    parser.add_argument(
        "--target-robot-name",
        default="JointActuatorAsset",
        help="PDU robot name that owns the actuator command channels.",
    )
    parser.add_argument(
        "--position-pdu-name",
        default="position_target",
        help="Position command PDU channel name.",
    )
    parser.add_argument(
        "--velocity-pdu-name",
        default="velocity_target",
        help="Velocity command PDU channel name.",
    )
    parser.add_argument(
        "--endpoint-name",
        default="joint_actuator_sender",
        help="Endpoint instance name for this Python process.",
    )
    parser.add_argument(
        "--delta-usec",
        type=int,
        default=20_000,
        help="Hakoniwa callback period in usec.",
    )
    parser.add_argument(
        "--send-rate-hz",
        type=float,
        default=20.0,
        help="Command publish rate.",
    )
    parser.add_argument(
        "--position-amplitude",
        type=float,
        default=0.8,
        help="Sine-wave position target amplitude in rad.",
    )
    parser.add_argument(
        "--position-frequency-hz",
        type=float,
        default=0.25,
        help="Sine-wave position target frequency.",
    )
    parser.add_argument(
        "--velocity-target",
        type=float,
        default=1.5,
        help="Velocity target magnitude in rad/s.",
    )
    return parser.parse_args()


def make_float64_payload(value: float) -> bytes:
    msg = Float64()
    msg.data = float(value)
    return bytes(py_to_pdu_Float64(msg))


def main() -> int:
    args = parse_args()
    endpoint_config = str(Path(args.endpoint_config).resolve())
    pdu_def = str(Path(args.pdu_def).resolve())
    position_key = PduKey(args.target_robot_name, args.position_pdu_name)
    velocity_key = PduKey(args.target_robot_name, args.velocity_pdu_name)
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
            print(
                "[INFO] Joint actuator sender callback started: "
                f"robot={args.target_robot_name} "
                f"position={args.position_pdu_name} velocity={args.velocity_pdu_name}"
            )
            while not shutdown.is_set():
                if elapsed_usec >= next_send_usec:
                    t = elapsed_usec / 1_000_000.0
                    position_target = args.position_amplitude * math.sin(
                        2.0 * math.pi * args.position_frequency_hz * t
                    )
                    phase = int(t // 2.0) % 3
                    if phase == 0:
                        velocity_target = args.velocity_target
                    elif phase == 1:
                        velocity_target = -args.velocity_target
                    else:
                        velocity_target = 0.0

                    endpoint.send_by_name(
                        position_key,
                        make_float64_payload(position_target),
                    )
                    endpoint.send_by_name(
                        velocity_key,
                        make_float64_payload(velocity_target),
                    )

                    if (count % 20) == 0:
                        print(
                            "sent "
                            f"position_target={position_target:.3f} "
                            f"velocity_target={velocity_target:.3f}"
                        )
                    count += 1
                    next_send_usec += send_interval_usec

                if hakopy.usleep(args.delta_usec) is False:
                    break
                elapsed_usec += args.delta_usec
                time.sleep(args.delta_usec / 1_000_000.0)
        except Exception as exc:
            print(f"[ERROR] joint actuator sender callback failed: {exc}")
            callback_state["result"] = 1
        finally:
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

    print("[INFO] Joint actuator command sender is registered.")
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
