#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import time
from pathlib import Path

from hakoniwa_pdu._optional_hakopy import hakopy
from hakoniwa_pdu.impl.pdu_channel_config import PduChannelConfig
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Twist import py_to_pdu_Twist
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_pytype_Twist import Twist


DEFAULT_SERVICE_CONFIG_PATH = (
    Path(__file__).resolve().parents[1] / "models" / "drone" / "drone_ball_service.json"
)
DEFAULT_ROBOT_NAME = "Ball-1"
DEFAULT_WARMUP_READ_COUNT = 1
DEFAULT_WRITE_RETRY_COUNT = 20
DEFAULT_WRITE_RETRY_INTERVAL_SEC = 0.05


class BallController:
    def __init__(
        self,
        service_config_path: Path,
        robot_name: str,
        warmup_read_count: int,
        write_retry_count: int,
        write_retry_interval_sec: float,
    ):
        self.service_config_path = service_config_path
        self.robot_name = robot_name
        self.warmup_read_count = warmup_read_count
        self.write_retry_count = write_retry_count
        self.write_retry_interval_sec = write_retry_interval_sec
        service_config = self._load_json(self.service_config_path)
        if "pdu_config_path" not in service_config:
            raise RuntimeError(f"pdu_config_path is not set in {self.service_config_path}")
        self.pdu_config_path = Path(service_config["pdu_config_path"])
        if not self.pdu_config_path.is_absolute():
            self.pdu_config_path = (Path.cwd() / self.pdu_config_path).resolve()
        self.pdu_config = PduChannelConfig(str(self.pdu_config_path))

        if not hakopy.init_for_external():
            raise RuntimeError("hakopy.init_for_external() failed")
        self.warmup()

    @staticmethod
    def _load_json(path: Path) -> dict:
        import json

        with path.open("r", encoding="utf-8") as f:
            return json.load(f)

    def warmup(self) -> None:
        pos_channel_id = self.pdu_config.get_pdu_channel_id(self.robot_name, "pos")
        pos_pdu_size = self.pdu_config.get_pdu_size(self.robot_name, "pos")
        if pos_channel_id < 0 or pos_pdu_size <= 0:
            raise RuntimeError(f"Ball warmup PDU is not defined: {self.robot_name}/pos")
        for _ in range(self.warmup_read_count):
            try:
                hakopy.pdu_read(self.robot_name, pos_channel_id, pos_pdu_size)
            except Exception:
                pass
            time.sleep(self.write_retry_interval_sec)

    def send_twist(self, pdu_name: str, linear: tuple[float, float, float], angular: tuple[float, float, float]) -> None:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = linear
        msg.angular.x, msg.angular.y, msg.angular.z = angular
        raw = py_to_pdu_Twist(msg)
        channel_id = self.pdu_config.get_pdu_channel_id(self.robot_name, pdu_name)
        if channel_id < 0:
            raise RuntimeError(f"Unknown PDU channel: {self.robot_name}/{pdu_name}")
        for attempt in range(self.write_retry_count):
            if hakopy.pdu_write(self.robot_name, channel_id, raw, len(raw)):
                return
            if attempt + 1 < self.write_retry_count:
                time.sleep(self.write_retry_interval_sec)
        raise RuntimeError(
            f"Failed to write {self.robot_name}/{pdu_name} "
            f"after {self.write_retry_count} attempts"
        )

    def set_pos(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> None:
        self.send_twist("set_pos", (x, y, z), (roll, pitch, yaw))

    def add_force(
        self,
        fx: float,
        fy: float,
        fz: float,
        tx: float = 0.0,
        ty: float = 0.0,
        tz: float = 0.0,
    ) -> None:
        self.send_twist("add_force", (fx, fy, fz), (tx, ty, tz))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send Ball-1 set_pos and add_force events to drone_ball_sim over SHM."
    )
    parser.add_argument(
        "--service-config-path",
        default=str(DEFAULT_SERVICE_CONFIG_PATH),
        help="Path to the external SHM service config JSON.",
    )
    parser.add_argument(
        "--robot-name",
        default=DEFAULT_ROBOT_NAME,
        help="Target robot name. Default: Ball-1",
    )
    parser.add_argument(
        "--set-pos",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="Send Ball-1.set_pos with position and Euler angles.",
    )
    parser.add_argument(
        "--add-force",
        nargs=3,
        type=float,
        metavar=("FX", "FY", "FZ"),
        help="Send Ball-1.add_force with force vector in world coordinates.",
    )
    parser.add_argument(
        "--add-torque",
        nargs=3,
        type=float,
        default=(0.0, 0.0, 0.0),
        metavar=("TX", "TY", "TZ"),
        help="Optional torque vector paired with --add-force.",
    )
    parser.add_argument(
        "--sleep-after-set-pos",
        type=float,
        default=0.1,
        help="Seconds to wait between set_pos and add_force. Default: 0.1",
    )
    parser.add_argument(
        "--warmup-read-count",
        type=int,
        default=DEFAULT_WARMUP_READ_COUNT,
        help=f"How many times to call run_nowait() before first write. Default: {DEFAULT_WARMUP_READ_COUNT}",
    )
    parser.add_argument(
        "--write-retry-count",
        type=int,
        default=DEFAULT_WRITE_RETRY_COUNT,
        help=f"Retry count for SHM write during startup races. Default: {DEFAULT_WRITE_RETRY_COUNT}",
    )
    parser.add_argument(
        "--write-retry-interval",
        type=float,
        default=DEFAULT_WRITE_RETRY_INTERVAL_SEC,
        help=f"Retry interval seconds for SHM write. Default: {DEFAULT_WRITE_RETRY_INTERVAL_SEC}",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.set_pos is None and args.add_force is None:
        parser.error("At least one of --set-pos or --add-force is required.")

    controller = BallController(
        Path(args.service_config_path),
        args.robot_name,
        args.warmup_read_count,
        args.write_retry_count,
        args.write_retry_interval,
    )

    if args.set_pos is not None:
        controller.set_pos(*args.set_pos)
        print(
            "sent set_pos:",
            f"pos=({args.set_pos[0]}, {args.set_pos[1]}, {args.set_pos[2]})",
            f"euler=({args.set_pos[3]}, {args.set_pos[4]}, {args.set_pos[5]})",
        )
        if args.add_force is not None and args.sleep_after_set_pos > 0:
            time.sleep(args.sleep_after_set_pos)

    if args.add_force is not None:
        tx, ty, tz = args.add_torque
        controller.add_force(*args.add_force, tx, ty, tz)
        print(
            "sent add_force:",
            f"force=({args.add_force[0]}, {args.add_force[1]}, {args.add_force[2]})",
            f"torque=({tx}, {ty}, {tz})",
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
