#!/usr/bin/env python3

import argparse
import sys
import time
from dataclasses import dataclass


AXIS_COUNT = 6
BUTTON_COUNT = 15
DEFAULT_CONFIG_PATH = "config/tb3-pdudef-compact.json"
DEFAULT_ROBOT = "TB3"
DEFAULT_PDU = "hako_cmd_game"
RUNTIME = None


@dataclass(frozen=True)
class RoutePhase:
    name: str
    duration_sec: float
    linear_axis: float
    yaw_axis: float


def load_runtime():
    global RUNTIME
    if RUNTIME is not None:
        return RUNTIME

    try:
        import hakopy
        from hakoniwa_pdu.impl.shm_communication_service import ShmCommunicationService
        from hakoniwa_pdu.pdu_manager import PduManager
        from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_conv_GameControllerOperation import (
            py_to_pdu_GameControllerOperation,
        )
        from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_pytype_GameControllerOperation import (
            GameControllerOperation,
        )
    except ModuleNotFoundError as exc:
        print(
            "[ERROR] Missing Python runtime module. Install hakoniwa-pdu and make "
            "hakopy available before running tb3_route_demo.py.",
            file=sys.stderr,
        )
        print(f"[ERROR] {exc}", file=sys.stderr)
        raise SystemExit(1) from exc

    RUNTIME = {
        "hakopy": hakopy,
        "ShmCommunicationService": ShmCommunicationService,
        "PduManager": PduManager,
        "py_to_pdu_GameControllerOperation": py_to_pdu_GameControllerOperation,
        "GameControllerOperation": GameControllerOperation,
    }
    return RUNTIME


def clamp_axis(value: float) -> float:
    return max(-1.0, min(1.0, value))


def build_gamepad(linear_axis: float, yaw_axis: float):
    runtime = load_runtime()
    data = runtime["GameControllerOperation"]()
    data.axis = [0.0] * AXIS_COUNT
    data.button = [False] * BUTTON_COUNT

    # tb3_sim maps axis[3] through a sign inversion to forward velocity and
    # axis[0] through a sign inversion to yaw rate.
    data.axis[0] = -clamp_axis(yaw_axis)
    data.axis[3] = -clamp_axis(linear_axis)
    return data


def build_route(args: argparse.Namespace) -> list[RoutePhase]:
    linear = clamp_axis(args.linear_axis)
    yaw = clamp_axis(args.yaw_axis)
    phases: list[RoutePhase] = []

    if args.pattern == "straight":
        phases.append(RoutePhase("forward", args.duration_sec, linear, 0.0))
    elif args.pattern == "spin":
        phases.append(RoutePhase("spin", args.duration_sec, 0.0, yaw))
    elif args.pattern == "square":
        for index in range(args.loops * 4):
            side = (index % 4) + 1
            phases.append(RoutePhase(f"side-{side}", args.forward_sec, linear, 0.0))
            phases.append(RoutePhase(f"turn-{side}", args.turn_sec, 0.0, yaw))
    elif args.pattern == "showcase":
        phases.extend(
            [
                RoutePhase("forward", args.forward_sec, linear, 0.0),
                RoutePhase("arc-left", args.forward_sec, linear * 0.8, yaw * 0.6),
                RoutePhase("spin-left", args.turn_sec, 0.0, yaw),
                RoutePhase("arc-right", args.forward_sec, linear * 0.8, -yaw * 0.6),
                RoutePhase("settle", args.stop_sec, 0.0, 0.0),
            ]
        )
    elif args.pattern == "figure8":
        for index in range(args.loops):
            loop = index + 1
            phases.extend(
                [
                    RoutePhase(f"loop-{loop}-arc-left", args.forward_sec, linear, yaw * 0.7),
                    RoutePhase(f"loop-{loop}-arc-right", args.forward_sec, linear, -yaw * 0.7),
                ]
            )
    elif args.pattern == "dance":
        for index in range(args.loops):
            loop = index + 1
            phases.extend(
                [
                    RoutePhase(f"loop-{loop}-dash", args.forward_sec * 0.7, linear, 0.0),
                    RoutePhase(f"loop-{loop}-arc-left", args.forward_sec, linear * 0.8, yaw * 0.8),
                    RoutePhase(f"loop-{loop}-spin", args.turn_sec, 0.0, yaw),
                    RoutePhase(f"loop-{loop}-reverse-arc", args.forward_sec * 0.8, -linear * 0.6, -yaw * 0.6),
                    RoutePhase(f"loop-{loop}-arc-right", args.forward_sec, linear * 0.8, -yaw * 0.8),
                    RoutePhase(f"loop-{loop}-pause", args.stop_sec, 0.0, 0.0),
                ]
            )
    else:
        raise ValueError(f"unsupported pattern: {args.pattern}")

    phases.append(RoutePhase("stop", args.stop_sec, 0.0, 0.0))
    return phases


def send_command(
    pdu_manager,
    robot: str,
    pdu: str,
    command,
) -> None:
    runtime = load_runtime()
    payload = runtime["py_to_pdu_GameControllerOperation"](command)
    pdu_manager.run_nowait()
    pdu_manager.flush_pdu_raw_data_nowait(robot, pdu, payload)


def run_route(args: argparse.Namespace) -> None:
    runtime = load_runtime()
    pdu_manager = runtime["PduManager"]()
    pdu_manager.initialize(
        config_path=args.config_path,
        comm_service=runtime["ShmCommunicationService"](),
    )
    pdu_manager.start_service_nowait()

    if not runtime["hakopy"].init_for_external():
        raise RuntimeError("hakopy.init_for_external() failed")

    route = build_route(args)
    interval_sec = 1.0 / args.rate_hz
    print(
        f"[INFO] TB3 route demo start: pattern={args.pattern}, phases={len(route)}, "
        f"rate_hz={args.rate_hz:.1f}, target={args.robot}/{args.pdu}"
    )

    try:
        for phase in route:
            command = build_gamepad(phase.linear_axis, phase.yaw_axis)
            deadline = time.monotonic() + phase.duration_sec
            print(
                f"[INFO] phase={phase.name} duration={phase.duration_sec:.2f}s "
                f"linear_axis={phase.linear_axis:.2f} yaw_axis={phase.yaw_axis:.2f}"
            )
            while time.monotonic() < deadline:
                send_command(pdu_manager, args.robot, args.pdu, command)
                time.sleep(interval_sec)
    finally:
        stop = build_gamepad(0.0, 0.0)
        for _ in range(max(1, int(args.rate_hz * args.stop_sec))):
            send_command(pdu_manager, args.robot, args.pdu, stop)
            time.sleep(interval_sec)
        print("[INFO] TB3 route demo finished")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Joystick-free TurtleBot3 route demo that writes GameControllerOperation PDU commands."
    )
    parser.add_argument(
        "--config-path",
        default=DEFAULT_CONFIG_PATH,
        help=f"PDU config JSON path (default: {DEFAULT_CONFIG_PATH})",
    )
    parser.add_argument("--robot", default=DEFAULT_ROBOT, help=f"Robot name (default: {DEFAULT_ROBOT})")
    parser.add_argument("--pdu", default=DEFAULT_PDU, help=f"Command PDU name (default: {DEFAULT_PDU})")
    parser.add_argument(
        "--pattern",
        choices=("straight", "spin", "square", "showcase", "figure8", "dance"),
        default="square",
        help="Route pattern to send (default: square)",
    )
    parser.add_argument(
        "--loops",
        type=int,
        default=1,
        help="Number of repeated loops for square, figure8, or dance patterns (default: 1)",
    )
    parser.add_argument(
        "--linear-axis",
        type=float,
        default=0.5,
        help="Normalized forward command axis in [-1, 1] (default: 0.5)",
    )
    parser.add_argument(
        "--yaw-axis",
        type=float,
        default=0.6,
        help="Normalized left-turn command axis in [-1, 1] (default: 0.6)",
    )
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=5.0,
        help="Duration for straight/spin patterns in seconds (default: 5.0)",
    )
    parser.add_argument(
        "--forward-sec",
        type=float,
        default=2.0,
        help="Forward phase duration for square pattern in seconds (default: 2.0)",
    )
    parser.add_argument(
        "--turn-sec",
        type=float,
        default=1.2,
        help="Turn phase duration for square pattern in seconds (default: 1.2)",
    )
    parser.add_argument(
        "--stop-sec",
        type=float,
        default=0.5,
        help="Stop command duration at the end and during cleanup (default: 0.5)",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=50.0,
        help="Command publish rate in Hz (default: 50.0)",
    )
    args = parser.parse_args()

    if args.loops < 1:
        parser.error("--loops must be >= 1")
    if args.duration_sec <= 0.0:
        parser.error("--duration-sec must be > 0")
    if args.forward_sec <= 0.0:
        parser.error("--forward-sec must be > 0")
    if args.turn_sec <= 0.0:
        parser.error("--turn-sec must be > 0")
    if args.stop_sec < 0.0:
        parser.error("--stop-sec must be >= 0")
    if args.rate_hz <= 0.0:
        parser.error("--rate-hz must be > 0")
    return args


def main() -> int:
    try:
        run_route(parse_args())
        return 0
    except KeyboardInterrupt:
        print("[INFO] interrupted")
        return 130
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
