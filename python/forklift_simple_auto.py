import argparse
import os
import sys
import time

import hakopy
from hakoniwa_pdu.impl.shm_communication_service import ShmCommunicationService
from hakoniwa_pdu.pdu_manager import PduManager

from python.api.forklift_api import ForkliftAPI


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Simple automatic forklift mission (forward and back)."
    )
    parser.add_argument("config_path", help="Path to PDU config JSON")
    parser.add_argument(
        "--forward-distance",
        type=float,
        default=1.0,
        help="Forward distance in meters (default: 1.0)",
    )
    parser.add_argument(
        "--backward-distance",
        type=float,
        default=None,
        help="Backward distance in meters (default: same as --forward-distance)",
    )
    parser.add_argument(
        "--turn-degree",
        type=float,
        default=0.0,
        help="Optional turn angle after forward move (default: 0.0)",
    )
    parser.add_argument(
        "--start-height",
        type=float,
        default=-0.05,
        help="Fork height at mission start/end (default: -0.05)",
    )
    parser.add_argument(
        "--pause-sec",
        type=float,
        default=0.5,
        help="Pause time between actions in seconds (default: 0.5)",
    )
    parser.add_argument(
        "--move-speed",
        type=float,
        default=0.5,
        help="Forward/backward speed in gamepad axis range (0.0, 1.0] (default: 0.5)",
    )
    return parser.parse_args()


def print_status(forklift: ForkliftAPI, label: str) -> None:
    pos = forklift.get_position()
    yaw = forklift.get_yaw_degree()
    height = forklift.get_height()
    print(
        f"[INFO] {label}: pos=({pos.linear.x:.3f}, {pos.linear.y:.3f}, {pos.linear.z:.3f}), "
        f"yaw={yaw:.3f} deg, height={height:.3f} m"
    )


def run_simple_mission(
    forklift: ForkliftAPI,
    forward_distance: float,
    backward_distance: float,
    turn_degree: float,
    start_height: float,
    pause_sec: float,
    move_speed: float,
) -> None:
    print("[INFO] Mission start: simple auto control")
    forklift.set_move_speed(move_speed)
    print(
        f"[INFO] Parameters: forward={forward_distance:.3f}m, backward={backward_distance:.3f}m, "
        f"turn={turn_degree:.3f}deg, start_height={start_height:.3f}m, move_speed={forklift.move_speed:.3f}"
    )
    print_status(forklift, "Initial state")

    print(f"[INFO] Set fork height to {start_height:.3f}m")
    forklift.lift_move(start_height)
    time.sleep(pause_sec)

    print(f"[INFO] Move forward {forward_distance:.3f}m")
    forklift.move(forward_distance)
    time.sleep(pause_sec)

    if abs(turn_degree) > 1e-6:
        print(f"[INFO] Turn to {turn_degree:.3f} deg")
        forklift.set_yaw_degree(turn_degree)
        time.sleep(pause_sec)

    print(f"[INFO] Move backward {backward_distance:.3f}m")
    forklift.move(-backward_distance)
    time.sleep(pause_sec)

    if abs(turn_degree) > 1e-6:
        print("[INFO] Return heading to 0 deg")
        forklift.set_yaw_degree(0.0)
        time.sleep(pause_sec)

    print(f"[INFO] Return fork height to {start_height:.3f}m")
    forklift.lift_move(start_height)
    time.sleep(pause_sec)

    print_status(forklift, "Mission done")


def main() -> int:
    args = parse_args()
    config_path = args.config_path
    if not os.path.exists(config_path):
        print(f"[ERROR] Config file not found: {config_path}")
        return 1

    forward_distance = abs(args.forward_distance)
    backward_distance = abs(args.backward_distance) if args.backward_distance is not None else forward_distance
    pause_sec = max(0.0, args.pause_sec)

    pdu_manager = PduManager()
    forklift = None
    try:
        pdu_manager.initialize(config_path=config_path, comm_service=ShmCommunicationService())
        pdu_manager.start_service_nowait()
        if not hakopy.init_for_external():
            print("[ERROR] hakopy.init_for_external() failed")
            return 1

        forklift = ForkliftAPI(pdu_manager)
        run_simple_mission(
            forklift=forklift,
            forward_distance=forward_distance,
            backward_distance=backward_distance,
            turn_degree=args.turn_degree,
            start_height=args.start_height,
            pause_sec=pause_sec,
            move_speed=args.move_speed,
        )
        return 0
    except KeyboardInterrupt:
        print("[INFO] Interrupted by user")
        return 0
    finally:
        try:
            if forklift is not None:
                forklift.stop()
        except Exception:
            pass
        try:
            pdu_manager.stop_service()
        except Exception:
            pass
        try:
            hakopy.fin()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
