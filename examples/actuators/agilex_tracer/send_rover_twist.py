#!/usr/bin/env python3
"""Send geometry_msgs/Twist commands to the generic rover Twist asset."""

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path

import hakopy
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Twist import py_to_pdu_Twist
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_pytype_Twist import Twist
from hakoniwa_pdu_endpoint.c_endpoint import Endpoint, PduKey


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_ENDPOINT_CONFIG = REPO_ROOT / "config/endpoint/rover_twist_endpoint.json"
DEFAULT_PDU_DEF = REPO_ROOT / "config/rover-twist-pdudef-compact.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send geometry_msgs/Twist cmd_vel commands to RoverTwistAsset."
    )
    parser.add_argument("--endpoint-config", default=str(DEFAULT_ENDPOINT_CONFIG))
    parser.add_argument("--pdu-def", default=str(DEFAULT_PDU_DEF))
    parser.add_argument("--sender-asset-name", default="RoverTwistSender")
    parser.add_argument("--target-robot-name", default="RoverTwistAsset")
    parser.add_argument("--pdu-name", default="cmd_vel")
    parser.add_argument("--endpoint-name", default="rover_twist_sender")
    parser.add_argument("--delta-usec", type=int, default=20_000)
    parser.add_argument("--send-rate-hz", type=float, default=20.0)
    parser.add_argument("--linear-x", type=float, default=0.2)
    parser.add_argument("--angular-z", type=float, default=0.0)
    parser.add_argument("--duration-sec", type=float, default=5.0)
    return parser.parse_args()


def make_twist_payload(linear_x: float, angular_z: float) -> bytes:
    msg = Twist()
    msg.linear.x = float(linear_x)
    msg.angular.z = float(angular_z)
    return bytes(py_to_pdu_Twist(msg))


def main() -> int:
    args = parse_args()
    endpoint_config = str(Path(args.endpoint_config).resolve())
    pdu_def = str(Path(args.pdu_def).resolve())
    cmd_key = PduKey(args.target_robot_name, args.pdu_name)
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
            payload = make_twist_payload(args.linear_x, args.angular_z)
            print(
                "[INFO] Rover Twist sender callback started: "
                f"robot={args.target_robot_name} pdu={args.pdu_name} "
                f"linear.x={args.linear_x:.3f} angular.z={args.angular_z:.3f}"
            )
            while not shutdown.is_set() and elapsed_usec <= int(args.duration_sec * 1_000_000):
                if elapsed_usec >= next_send_usec:
                    endpoint.send_by_name(cmd_key, payload)
                    if (count % 20) == 0:
                        print(
                            "sent cmd_vel "
                            f"linear.x={args.linear_x:.3f} "
                            f"angular.z={args.angular_z:.3f}"
                        )
                    count += 1
                    next_send_usec += send_interval_usec

                if hakopy.usleep(args.delta_usec) is False:
                    break
                elapsed_usec += args.delta_usec
                time.sleep(args.delta_usec / 1_000_000.0)

            endpoint.send_by_name(cmd_key, make_twist_payload(0.0, 0.0))
            shutdown.set()
        except Exception as exc:
            print(f"[ERROR] rover twist sender callback failed: {exc}")
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

    print("[INFO] Rover Twist sender is registered.")
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
