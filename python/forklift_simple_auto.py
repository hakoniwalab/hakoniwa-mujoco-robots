import argparse
import asyncio
import inspect
import json
import os
import sys
import time
from typing import Optional
import importlib.metadata

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
    parser.add_argument(
        "--startup-wait-sec",
        type=float,
        default=0.0,
        help="Wait time before phase check/command start to let simulator publish restored state (default: 0.0)",
    )
    parser.add_argument(
        "--phase-timeout-sec",
        type=float,
        default=float(os.getenv("HAKO_PHASE_TIMEOUT_SEC", "0.8")),
        help="Timeout for phase read before fallback (default: 0.8, env: HAKO_PHASE_TIMEOUT_SEC)",
    )
    parser.add_argument(
        "--mission-loops",
        type=int,
        default=1,
        help="Number of round trips to run (default: 1). Set 0 for infinite loop until Ctrl+C.",
    )
    parser.add_argument(
        "--controller-mode",
        choices=("external", "asset"),
        default=os.getenv("HAKO_CONTROLLER_MODE", "external"),
        help="Controller execution mode (default: external, env: HAKO_CONTROLLER_MODE)",
    )
    parser.add_argument(
        "--asset-name",
        default=os.getenv("HAKO_CONTROLLER_ASSET_NAME", "forklift_controller"),
        help="Hakoniwa asset name used in --controller-mode asset",
    )
    parser.add_argument(
        "--controller-delta-usec",
        type=int,
        default=int(os.getenv("HAKO_CONTROLLER_DELTA_USEC", "1000")),
        help="Hakoniwa controller delta time in usec for --controller-mode asset (default: 1000)",
    )
    parser.add_argument(
        "--pdu-diagnose",
        action="store_true",
        help="Print PDU diagnostics (hakoniwa-pdu version, config format, required PDU channel/size).",
    )
    parser.add_argument(
        "--pdu-diagnose-strict",
        action="store_true",
        help="Exit with error if any required PDU cannot be resolved (use with --pdu-diagnose).",
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


def infer_phase_from_pose(forklift: ForkliftAPI, origin_threshold: float = 0.3) -> int:
    pos = forklift.get_position()
    dist_xy = (pos.linear.x ** 2 + pos.linear.y ** 2) ** 0.5
    if dist_xy > origin_threshold:
        print(
            f"[WARN] Phase read as 0, but pose indicates resumed run "
            f"(x={pos.linear.x:.3f}, y={pos.linear.y:.3f}, dist={dist_xy:.3f}). "
            "Use phase=1 fallback."
        )
        return 1
    return 0


def run_simple_mission(
    forklift: ForkliftAPI,
    forward_distance: float,
    backward_distance: float,
    turn_degree: float,
    start_height: float,
    pause_sec: float,
    move_speed: float,
    current_phase: int = 0,
    sleep_fn=time.sleep,
) -> None:
    print("[INFO] Mission start: simple auto control")
    forklift.set_move_speed(move_speed)
    print(
        f"[INFO] Parameters: forward={forward_distance:.3f}m, backward={backward_distance:.3f}m, "
        f"turn={turn_degree:.3f}deg, start_height={start_height:.3f}m, move_speed={forklift.move_speed:.3f}"
    )
    print_status(forklift, "Initial state")

    print(f"[INFO] Current sim phase from PDU: {current_phase}")
    if current_phase == 0:
        print(f"[INFO] Set fork height to {start_height:.3f}m")
        forklift.lift_move(start_height)
        sleep_fn(pause_sec)
    else:
        print("[INFO] Resume phase detected. Skip initial lift command to preserve restored drive target.")

    if current_phase == 2:
        print("[INFO] Phase indicates return leg. Skip forward move.")
    else:
        print(f"[INFO] Move forward {forward_distance:.3f}m")
        forklift.move(forward_distance)
        sleep_fn(pause_sec)

    if abs(turn_degree) > 1e-6:
        print(f"[INFO] Turn to {turn_degree:.3f} deg")
        forklift.set_yaw_degree(turn_degree)
        sleep_fn(pause_sec)

    print(f"[INFO] Move backward {backward_distance:.3f}m")
    forklift.move(-backward_distance)
    sleep_fn(pause_sec)

    if abs(turn_degree) > 1e-6:
        print("[INFO] Return heading to 0 deg")
        forklift.set_yaw_degree(0.0)
        sleep_fn(pause_sec)

    print(f"[INFO] Return fork height to {start_height:.3f}m")
    forklift.lift_move(start_height)
    sleep_fn(pause_sec)

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
    sleep_fn=time.sleep,
) -> bool:
    for _ in range(max_rounds):
        sleep_fn(settle_sec)
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
    sleep_fn=time.sleep,
) -> int:
    deadline = time.time() + max(0.1, timeout_sec)
    last_phase = 0
    start = time.time()
    while time.time() < deadline:
        ok, phase = forklift.get_phase_status()
        if ok:
            last_phase = phase
            if phase in (1, 2):
                elapsed = time.time() - start
                print(f"[INFO] Phase resolved: phase={phase} elapsed={elapsed:.3f}s")
                return phase
        sleep_fn(max(0.001, poll_interval_sec))
    elapsed = time.time() - start
    print(f"[WARN] Phase resolve timeout: use last_phase={last_phase} elapsed={elapsed:.3f}s")
    return last_phase

def run_global_goal_mission(
    forklift: ForkliftAPI,
    forward_goal_x: float,
    home_goal_x: float,
    goal_tolerance: float,
    start_height: float,
    pause_sec: float,
    move_speed: float,
    current_phase: int,
    sleep_fn=time.sleep,
) -> None:
    print("[INFO] Mission start: absolute-goal control")
    forklift.set_move_speed(move_speed)
    print(
        f"[INFO] Parameters: forward_goal_x={forward_goal_x:.3f}, home_goal_x={home_goal_x:.3f}, "
        f"goal_tolerance={goal_tolerance:.3f}, start_height={start_height:.3f}, move_speed={forklift.move_speed:.3f}"
    )
    print_status(forklift, "Initial state")
    print(f"[INFO] Current sim phase from PDU: {current_phase}")

    if current_phase == 0:
        print(f"[INFO] Set fork height to {start_height:.3f}m")
        forklift.lift_move(start_height)
        sleep_fn(pause_sec)
    else:
        print("[INFO] Resume phase detected. Skip initial lift command to avoid overriding restored drive target.")

    if current_phase == 2:
        print("[INFO] Phase indicates return leg. Skip forward phase and go home.")
    else:
        print(f"[INFO] Move to global forward goal X={forward_goal_x:.3f}")
        reached_forward = move_to_global_x(forklift, forward_goal_x, goal_tolerance)
        stable_forward = stabilize_at_global_x(
            forklift, forward_goal_x, goal_tolerance, sleep_fn=sleep_fn
        )
        if not (reached_forward and stable_forward):
            print("[WARN] Forward phase did not fully converge; continue to return-home phase.")
        sleep_fn(pause_sec)

    print(f"[INFO] Return to global home X={home_goal_x:.3f}")
    reached_home = move_to_global_x(forklift, home_goal_x, goal_tolerance)
    stable_home = stabilize_at_global_x(
        forklift, home_goal_x, goal_tolerance, sleep_fn=sleep_fn
    )
    if not (reached_home and stable_home):
        print("[WARN] Home phase did not fully converge.")
    sleep_fn(pause_sec)

    print(f"[INFO] Return fork height to {start_height:.3f}m")
    forklift.lift_move(start_height)
    _ = stabilize_at_global_x(forklift, home_goal_x, goal_tolerance, sleep_fn=sleep_fn)
    forklift.stop()
    sleep_fn(pause_sec)

    print_status(forklift, "Mission done")


class ControllerInterrupted(Exception):
    pass


def _detect_pdudef_format(config_path: str) -> str:
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        return "unknown"
    if isinstance(data, dict) and "paths" in data:
        return "compact"
    robots = data.get("robots", []) if isinstance(data, dict) else []
    if robots and isinstance(robots, list):
        first = robots[0]
        if isinstance(first, dict) and (
            "shm_pdu_readers" in first or "shm_pdu_writers" in first
        ):
            return "legacy"
    return "unknown"


def log_pdu_diagnostics(
    pdu_manager: PduManager,
    config_path: str,
    robot_name: str = "forklift",
    required_pdus=None,
) -> bool:
    if required_pdus is None:
        required_pdus = ["hako_cmd_game", "pos", "height", "phase"]
    try:
        pdu_version = importlib.metadata.version("hakoniwa-pdu")
    except Exception:
        pdu_version = "unknown"
    fmt = _detect_pdudef_format(config_path)
    print(f"[DIAG] hakoniwa-pdu version: {pdu_version}")
    print(f"[DIAG] pdudef path: {config_path}")
    print(f"[DIAG] pdudef format: {fmt}")

    unresolved = []
    for pdu_name in required_pdus:
        ch = pdu_manager.get_pdu_channel_id(robot_name, pdu_name)
        sz = pdu_manager.get_pdu_size(robot_name, pdu_name)
        ok = (ch >= 0 and sz > 0)
        state = "OK" if ok else "NG"
        print(f"[DIAG] resolve {robot_name}/{pdu_name}: channel={ch} size={sz} => {state}")
        if not ok:
            unresolved.append(pdu_name)

    if unresolved:
        print(f"[WARN] unresolved pdus: {', '.join(unresolved)}")
        if fmt == "compact":
            print(
                "[WARN] compact format detected but required PDUs were unresolved. "
                "Check hakoniwa-pdu runtime version and compact support path."
            )
        return False
    return True


def _build_tick_sleep():
    # Use hakopy.usleep() for tick-synchronized controller execution.
    def _sleep(sec: float):
        usec = int(max(0.0, sec) * 1_000_000)
        if usec <= 0:
            return
        ok = hakopy.usleep(usec)
        if not ok:
            raise ControllerInterrupted("hakopy.usleep returned false")
    return _sleep


def run_controller(args: argparse.Namespace, pdu_manager: PduManager, use_tick_sleep: bool) -> int:
    forward_distance = abs(args.forward_distance)
    backward_distance = abs(args.backward_distance) if args.backward_distance is not None else forward_distance
    pause_sec = max(0.0, args.pause_sec)
    startup_wait_sec = max(0.0, args.startup_wait_sec)
    sleep_fn = _build_tick_sleep() if use_tick_sleep else time.sleep

    forklift = ForkliftAPI(pdu_manager)
    forklift.set_sleep_func(sleep_fn)
    if startup_wait_sec > 0.0:
        print(f"[INFO] Startup wait: {startup_wait_sec:.3f}s")
        sleep_fn(startup_wait_sec)
    phase_timeout_sec = max(0.0, float(args.phase_timeout_sec))
    current_phase = wait_for_sim_phase(
        forklift,
        timeout_sec=phase_timeout_sec,
        sleep_fn=sleep_fn,
    )
    if current_phase == 0:
        current_phase = infer_phase_from_pose(forklift)
    print(f"[INFO] Phase selected for mission: {current_phase}")
    mission_loops = max(0, args.mission_loops)
    loop_index = 0
    while True:
        phase_for_this_loop = current_phase if loop_index == 0 else 0
        print(
            f"[INFO] Mission loop {loop_index + 1}"
            + (" (infinite)" if mission_loops == 0 else f"/{mission_loops}")
            + f" phase={phase_for_this_loop}"
        )
        if args.forward_goal_x is not None:
            run_global_goal_mission(
                forklift=forklift,
                forward_goal_x=args.forward_goal_x,
                home_goal_x=args.home_goal_x,
                goal_tolerance=max(0.001, args.goal_tolerance),
                start_height=args.start_height,
                pause_sec=pause_sec,
                move_speed=args.move_speed,
                current_phase=phase_for_this_loop,
                sleep_fn=sleep_fn,
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
                current_phase=phase_for_this_loop,
                sleep_fn=sleep_fn,
            )
        loop_index += 1
        if mission_loops > 0 and loop_index >= mission_loops:
            break
    return 0


def _main_external(args: argparse.Namespace, pdu_manager: PduManager) -> int:
    if not hakopy.init_for_external():
        print("[ERROR] hakopy.init_for_external() failed")
        return 1
    return run_controller(args, pdu_manager, use_tick_sleep=False)


def _main_asset(args: argparse.Namespace, pdu_manager: PduManager) -> int:
    callback_state = {"result": 0}

    def on_initialize(_ctx):
        return 0

    def on_reset(_ctx):
        return 0

    def on_manual_timing_control(_ctx):
        try:
            callback_state["result"] = run_controller(args, pdu_manager, use_tick_sleep=True)
        except ControllerInterrupted:
            print("[INFO] Controller stopped by hakopy.usleep(false)")
            callback_state["result"] = 0
        except KeyboardInterrupt:
            print("[INFO] Interrupted by user")
            callback_state["result"] = 0
        except Exception as e:
            print(f"[ERROR] controller loop failed: {e}")
            callback_state["result"] = 1
        return 0

    callback = {
        "on_initialize": on_initialize,
        "on_simulation_step": None,
        "on_manual_timing_control": on_manual_timing_control,
        "on_reset": on_reset,
    }

    delta_usec = max(100, int(args.controller_delta_usec))
    model = hakopy.HAKO_ASSET_MODEL_CONTROLLER
    ret = hakopy.asset_register(args.asset_name, args.config_path, callback, delta_usec, model)
    if not ret:
        print("[ERROR] hakopy.asset_register() failed")
        return 1
    print(
        f"[INFO] Controller asset mode: name={args.asset_name} delta_usec={delta_usec} model=CONTROLLER"
    )
    start_ret = hakopy.start()
    print(f"[INFO] hako_asset_start() returns {start_ret}")
    return int(callback_state["result"])


def main() -> int:
    args = parse_args()
    config_path = args.config_path
    if not os.path.exists(config_path):
        print(f"[ERROR] Config file not found: {config_path}")
        return 1

    pdu_manager = PduManager()
    interrupted = False
    try:
        pdu_manager.initialize(config_path=config_path, comm_service=ShmCommunicationService())
        pdu_manager.start_service_nowait()
        if args.pdu_diagnose:
            ok = log_pdu_diagnostics(pdu_manager, config_path)
            if args.pdu_diagnose_strict and not ok:
                print("[ERROR] PDU diagnose strict mode failed")
                return 2
        mode = args.controller_mode
        print(f"[INFO] Controller mode: {mode}")
        if mode == "asset":
            return _main_asset(args, pdu_manager)
        return _main_external(args, pdu_manager)
    except KeyboardInterrupt:
        interrupted = True
        print("[INFO] Interrupted by user")
        return 0
    finally:
        try:
            # For resume experiments, avoid overwriting restored target on Ctrl+C.
            if (not interrupted):
                forklift = ForkliftAPI(pdu_manager)
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
