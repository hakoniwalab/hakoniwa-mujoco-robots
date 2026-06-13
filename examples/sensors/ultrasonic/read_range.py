#!/usr/bin/env python3
"""Hakoniwa ultrasonic range reader asset."""

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path

import hakopy
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_Range import pdu_to_py_Range
from hakoniwa_pdu_endpoint.c_endpoint import Endpoint, PduKey


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_ENDPOINT_CONFIG = REPO_ROOT / "config/endpoint/ultrasonic_endpoint.json"
DEFAULT_PDU_DEF = REPO_ROOT / "config/ultrasonic-pdudef-compact.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Receive a Hakoniwa sensor_msgs/Range PDU and print it."
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
        "--reader-asset-name",
        default="UltrasonicReader",
        help="Hakoniwa asset name for this Python reader.",
    )
    parser.add_argument(
        "--producer-asset-name",
        default="UltrasonicAsset",
        help="Hakoniwa asset name that publishes the range PDU.",
    )
    parser.add_argument(
        "--pdu-name",
        default="range",
        help="Range PDU channel name.",
    )
    parser.add_argument(
        "--endpoint-name",
        default="ultrasonic_reader",
        help="Endpoint instance name for this Python process.",
    )
    parser.add_argument(
        "--delta-usec",
        type=int,
        default=30_000,
        help="Hakoniwa callback period in usec.",
    )
    parser.add_argument(
        "--print-period-sec",
        type=float,
        default=0.2,
        help="Minimum interval between printed range values.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    endpoint_config = str(Path(args.endpoint_config).resolve())
    pdu_def = str(Path(args.pdu_def).resolve())
    range_key = PduKey(args.producer_asset_name, args.pdu_name)
    shutdown = threading.Event()
    callback_state = {"result": 0}
    skipped_invalid = {"count": 0}
    endpoint = Endpoint(args.endpoint_name, "inout")

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
            pdu_size = endpoint.get_pdu_size(range_key)
            print(
                "[INFO] Ultrasonic reader callback started: "
                f"producer={args.producer_asset_name} pdu={args.pdu_name} size={pdu_size}"
            )
            last_print_time = 0.0
            while not shutdown.is_set():
                raw = endpoint.recv_by_name(range_key, pdu_size)
                if raw:
                    try:
                        msg = pdu_to_py_Range(raw)
                    except Exception as exc:
                        skipped_invalid["count"] += 1
                        if skipped_invalid["count"] == 1:
                            print(
                                "[INFO] Skipping invalid initial range PDU "
                                f"until publisher writes the first value: {exc}"
                            )
                        elif skipped_invalid["count"] % 100 == 0:
                            print(
                                "[INFO] Still waiting for first valid range PDU "
                                f"(skipped {skipped_invalid['count']} invalid reads)"
                            )
                        if hakopy.usleep(args.delta_usec) is False:
                            break
                        continue

                    now = time.monotonic()
                    if now - last_print_time >= args.print_period_sec:
                        if skipped_invalid["count"] > 0:
                            print(
                                "[INFO] First valid range PDU received after "
                                f"{skipped_invalid['count']} skipped reads"
                            )
                            skipped_invalid["count"] = 0
                        print(
                            "range="
                            f"{float(msg.range):.3f} m "
                            f"min={float(msg.min_range):.3f} "
                            f"max={float(msg.max_range):.3f} "
                            f"fov={float(msg.field_of_view):.3f} "
                            f"radiation_type={int(msg.radiation_type)}"
                        )
                        last_print_time = now

                if hakopy.usleep(args.delta_usec) is False:
                    break
        except Exception as exc:
            print(f"[ERROR] ultrasonic reader callback failed: {exc}")
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
        args.reader_asset_name,
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

    def run_hakopy_start() -> None:
        try:
            start_ret = hakopy.start()
            print(f"[INFO] hakopy.start() returns {start_ret}")
        except Exception as exc:
            print(f"[ERROR] hakopy.start() failed: {exc}")
            callback_state["result"] = 1
        finally:
            shutdown.set()

    worker = threading.Thread(
        target=run_hakopy_start,
        name="hakoniwa-ultrasonic-reader",
        daemon=True,
    )
    worker.start()

    print("[INFO] Waiting for range PDU. Press Ctrl+C to quit.")
    try:
        while not shutdown.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        shutdown.set()
    finally:
        try:
            endpoint.stop()
        finally:
            endpoint.close()
        worker.join(timeout=1.0)

    return int(callback_state["result"])


if __name__ == "__main__":
    raise SystemExit(main())
