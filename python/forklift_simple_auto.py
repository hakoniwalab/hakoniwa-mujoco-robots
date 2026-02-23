import argparse
import asyncio
import inspect
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
    parser.add_argument(
        "--forward-goal-x",
        type=float,
        default=None,
        help="Global X target for forward phase. If set, absolute-goal mission is used.",
    )
    parser.add_argument(
        "--home-goal-x",
        type=float,
        default=0.0,
        help="Global X target for return-home phase in absolute-goal mission (default: 0.0)",
    )
    parser.add_argument(
        "--goal-tolerance",
        type=float,
        default=0.03,
        help="Tolerance for absolute-goal convergence in meters (default: 0.03)",
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

def move_to_global_x(
    forklift: ForkliftAPI,
    target_x: float,
    tolerance: float,
    max_iter: int = 8,
) -> bool:
    for i in range(max_iter):
        pos = forklift.get_position()
        err = target_x - pos.linear.x
        print(f"[INFO] Global target_x={target_x:.3f}, current_x={pos.linear.x:.3f}, err={err:.3f}")
        if abs(err) <= tolerance:
            forklift.stop()
            return True
        forklift.move(err)
    print(f"[WARN] Failed to reach global X target {target_x:.3f} within {max_iter} iterations")
    return False

def stabilize_at_global_x(
    forklift: ForkliftAPI,
    target_x: float,
    tolerance: float,
    settle_sec: float = 0.3,
    max_rounds: int = 5,
) -> bool:
    for _ in range(max_rounds):
        forklift.stop()
        time.sleep(settle_sec)
        pos = forklift.get_position()
        err = target_x - pos.linear.x
        print(f"[INFO] Stabilize target_x={target_x:.3f}, current_x={pos.linear.x:.3f}, err={err:.3f}")
        if abs(err) <= tolerance:
            return True
        forklift.move(err)
    print(f"[WARN] Failed to stabilize at global X target {target_x:.3f}")
    return False

def wait_for_sim_phase(
    forklift: ForkliftAPI,
    timeout_sec: float = 3.0,
    poll_interval_sec: float = 0.05,
) -> int:
    deadline = time.time() + max(0.1, timeout_sec)
    last_phase = 0
    while time.time() < deadline:
        ok, phase = forklift.get_phase_status()
        if ok:
            last_phase = phase
            if phase in (1, 2):
                return phase
        time.sleep(max(0.001, poll_interval_sec))
    return last_phase

def run_global_goal_mission(
    forklift: ForkliftAPI,
    forward_goal_x: float,
    home_goal_x: float,
    goal_tolerance: float,
    start_height: float,
    pause_sec: float,
    move_speed: float,
) -> None:
    print("[INFO] Mission start: absolute-goal control")
    forklift.set_move_speed(move_speed)
    print(
        f"[INFO] Parameters: forward_goal_x={forward_goal_x:.3f}, home_goal_x={home_goal_x:.3f}, "
        f"goal_tolerance={goal_tolerance:.3f}, start_height={start_height:.3f}, move_speed={forklift.move_speed:.3f}"
    )
    print_status(forklift, "Initial state")
    # Prime command PDU before phase polling to avoid startup null-conversion noise.
    forklift.stop()
    current_phase = wait_for_sim_phase(forklift)
    print(f"[INFO] Current sim phase from PDU: {current_phase}")

    print(f"[INFO] Set fork height to {start_height:.3f}m")
    forklift.lift_move(start_height)
    time.sleep(pause_sec)

    if current_phase == 2:
        print("[INFO] Phase indicates return leg. Skip forward phase and go home.")
    else:
        print(f"[INFO] Move to global forward goal X={forward_goal_x:.3f}")
        reached_forward = move_to_global_x(forklift, forward_goal_x, goal_tolerance)
        stable_forward = stabilize_at_global_x(forklift, forward_goal_x, goal_tolerance)
        if not (reached_forward and stable_forward):
            print("[WARN] Forward phase did not fully converge; continue to return-home phase.")
        time.sleep(pause_sec)

    print(f"[INFO] Return to global home X={home_goal_x:.3f}")
    reached_home = move_to_global_x(forklift, home_goal_x, goal_tolerance)
    stable_home = stabilize_at_global_x(forklift, home_goal_x, goal_tolerance)
    if not (reached_home and stable_home):
        print("[WARN] Home phase did not fully converge.")
    time.sleep(pause_sec)

    print(f"[INFO] Return fork height to {start_height:.3f}m")
    forklift.lift_move(start_height)
    _ = stabilize_at_global_x(forklift, home_goal_x, goal_tolerance)
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
        if args.forward_goal_x is not None:
            run_global_goal_mission(
                forklift=forklift,
                forward_goal_x=args.forward_goal_x,
                home_goal_x=args.home_goal_x,
                goal_tolerance=max(0.001, args.goal_tolerance),
                start_height=args.start_height,
                pause_sec=pause_sec,
                move_speed=args.move_speed,
            )
        else:
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
            stop_result = pdu_manager.stop_service()
            if inspect.isawaitable(stop_result):
                asyncio.run(stop_result)
        except Exception:
            pass
        try:
            hakopy.fin()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
