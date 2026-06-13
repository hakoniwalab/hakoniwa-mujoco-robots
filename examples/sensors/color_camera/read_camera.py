#!/usr/bin/env python3
"""Hakoniwa camera image reader asset.

OpenCV window operations intentionally run on the main thread. hakopy.start()
is launched on a worker thread so the Hakoniwa callback loop can receive PDU
data without owning the GUI thread.
"""

from __future__ import annotations

import argparse
import queue
import threading
import time
from pathlib import Path

import cv2
import hakopy
import numpy as np
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_Image import pdu_to_py_Image
from hakoniwa_pdu_endpoint.c_endpoint import Endpoint, PduKey


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_ENDPOINT_CONFIG = REPO_ROOT / "config/endpoint/camera_endpoint.json"
DEFAULT_PDU_DEF = REPO_ROOT / "config/camera-pdudef-compact.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Receive a Hakoniwa sensor_msgs/Image PDU and show it with OpenCV."
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
        default="CameraReader",
        help="Hakoniwa asset name for this Python reader.",
    )
    parser.add_argument(
        "--producer-asset-name",
        default="CameraAsset",
        help="Hakoniwa asset name that publishes the image PDU.",
    )
    parser.add_argument(
        "--pdu-name",
        default="camera_image",
        help="Image PDU channel name.",
    )
    parser.add_argument(
        "--endpoint-name",
        default="camera_reader",
        help="Endpoint instance name for this Python process.",
    )
    parser.add_argument(
        "--delta-usec",
        type=int,
        default=30_000,
        help="Hakoniwa callback period in usec.",
    )
    return parser.parse_args()


def image_to_bgr(image_msg) -> np.ndarray | None:
    if isinstance(image_msg.data, (bytes, bytearray, memoryview)):
        data = np.frombuffer(image_msg.data, dtype=np.uint8)
    else:
        data = np.asarray(image_msg.data, dtype=np.uint8)
    height = int(image_msg.height)
    width = int(image_msg.width)
    encoding = str(image_msg.encoding)

    if encoding == "rgb8":
        expected = height * width * 3
        if len(data) != expected:
            return None
        rgb = data.reshape((height, width, 3))
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    if encoding == "bgr8":
        expected = height * width * 3
        if len(data) != expected:
            return None
        return data.reshape((height, width, 3)).copy()

    if encoding == "mono8":
        expected = height * width
        if len(data) != expected:
            return None
        mono = data.reshape((height, width))
        return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

    print(f"[WARN] Unsupported image encoding: {encoding}")
    return None


def put_latest(frame_queue: "queue.Queue[np.ndarray]", frame: np.ndarray) -> None:
    try:
        while True:
            frame_queue.get_nowait()
    except queue.Empty:
        pass
    try:
        frame_queue.put_nowait(frame)
    except queue.Full:
        pass


def make_status_frame(message: str) -> np.ndarray:
    frame = np.zeros((240, 640, 3), dtype=np.uint8)
    cv2.putText(
        frame,
        "Hakoniwa Camera Reader",
        (24, 64),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (220, 220, 220),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        message,
        (24, 128),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (120, 220, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        "Press q or Esc to quit",
        (24, 190),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (180, 180, 180),
        1,
        cv2.LINE_AA,
    )
    return frame


def main() -> int:
    args = parse_args()
    endpoint_config = str(Path(args.endpoint_config).resolve())
    pdu_def = str(Path(args.pdu_def).resolve())
    image_key = PduKey(args.producer_asset_name, args.pdu_name)
    frame_queue: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=1)
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
            pdu_size = endpoint.get_pdu_size(image_key)
            print(
                "[INFO] Camera reader callback started: "
                f"producer={args.producer_asset_name} pdu={args.pdu_name} size={pdu_size}"
            )
            while not shutdown.is_set():
                raw = endpoint.recv_by_name(image_key, pdu_size)
                if raw:
                    try:
                        image_msg = pdu_to_py_Image(raw)
                    except Exception as exc:
                        skipped_invalid["count"] += 1
                        if skipped_invalid["count"] == 1:
                            print(
                                "[INFO] Skipping invalid initial image PDU "
                                f"until publisher writes the first frame: {exc}"
                            )
                        elif skipped_invalid["count"] % 100 == 0:
                            print(
                                "[INFO] Still waiting for first valid image PDU "
                                f"(skipped {skipped_invalid['count']} invalid reads)"
                            )
                        if hakopy.usleep(args.delta_usec) is False:
                            break
                        continue

                    frame = image_to_bgr(image_msg)
                    if frame is not None:
                        if skipped_invalid["count"] > 0:
                            print(
                                "[INFO] First valid image PDU received after "
                                f"{skipped_invalid['count']} skipped reads"
                            )
                            skipped_invalid["count"] = 0
                        put_latest(frame_queue, frame)

                if hakopy.usleep(args.delta_usec) is False:
                    break
        except Exception as exc:
            print(f"[ERROR] camera reader callback failed: {exc}")
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
        name="hakoniwa-camera-reader",
        daemon=True,
    )
    worker.start()

    print("[INFO] OpenCV viewer runs on the main thread. Press q or Esc to quit.")
    last_frame: np.ndarray | None = make_status_frame("Waiting for camera_image PDU...")
    last_status_time = time.monotonic()
    try:
        while not shutdown.is_set():
            try:
                last_frame = frame_queue.get(timeout=0.03)
            except queue.Empty:
                pass

            if last_frame is not None:
                cv2.imshow("Hakoniwa Camera Data", last_frame)
            if time.monotonic() - last_status_time >= 2.0:
                print(
                    "[INFO] Waiting for image PDU: "
                    f"producer={args.producer_asset_name} pdu={args.pdu_name}"
                )
                last_status_time = time.monotonic()
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                shutdown.set()
                break
            time.sleep(0.001)
    except KeyboardInterrupt:
        shutdown.set()
    finally:
        cv2.destroyAllWindows()
        try:
            endpoint.stop()
        finally:
            endpoint.close()
        worker.join(timeout=1.0)

    return int(callback_state["result"])


if __name__ == "__main__":
    raise SystemExit(main())
