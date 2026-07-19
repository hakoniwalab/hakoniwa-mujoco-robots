#!/usr/bin/env python3

import argparse
import sys
import time
import numpy as np
from dataclasses import dataclass, field

# --- PDU Communication Boilerplate (from tb3_route_demo.py) ---
AXIS_COUNT = 6
BUTTON_COUNT = 15
DEFAULT_CONFIG_PATH = "config/tb3-pdudef-compact.json"
DEFAULT_ROBOT = "TB3"
DEFAULT_CMD_PDU = "hako_cmd_game"
DEFAULT_SCAN_PDU = "laser_scan"
RUNTIME = None

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
        from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_LaserScan import pdu_to_py_LaserScan
    except ModuleNotFoundError as exc:
        print(f"[ERROR] Missing Python runtime module. Install hakoniwa-pdu and hakopy.", file=sys.stderr)
        raise SystemExit(1) from exc

    RUNTIME = {
        "hakopy": hakopy,
        "ShmCommunicationService": ShmCommunicationService,
        "PduManager": PduManager,
        "py_to_pdu_GameControllerOperation": py_to_pdu_GameControllerOperation,
        "GameControllerOperation": GameControllerOperation,
        "pdu_to_py_LaserScan": pdu_to_py_LaserScan,
    }
    return RUNTIME

def clamp_axis(value: float) -> float:
    return max(-1.0, min(1.0, value))

def build_gamepad(linear_axis: float, yaw_axis: float):
    runtime = load_runtime()
    data = runtime["GameControllerOperation"]()
    data.axis = [0.0] * AXIS_COUNT
    data.button = [False] * BUTTON_COUNT
    data.axis[0] = -clamp_axis(yaw_axis)
    data.axis[3] = -clamp_axis(linear_axis)
    return data

def send_command(pdu_manager, robot: str, pdu: str, command):
    runtime = load_runtime()
    payload = runtime["py_to_pdu_GameControllerOperation"](command)
    pdu_manager.flush_pdu_raw_data_nowait(robot, pdu, payload)

# --- Wall Following Logic ---

@dataclass
class SmoothedValue:
    value: float = np.inf

    def update(self, sample: float, alpha: float) -> float:
        if not np.isfinite(sample):
            return self.value
        if not np.isfinite(self.value):
            self.value = sample
        else:
            self.value = self.value * (1.0 - alpha) + sample * alpha
        return self.value


@dataclass
class WallFollowerState:
    mode: str = "SEARCHING"
    yaw_axis: float = 0.0
    linear_axis: float = 0.0
    corner_until: float = 0.0
    last_log_at: float = 0.0
    front: SmoothedValue = field(default_factory=SmoothedValue)
    front_left: SmoothedValue = field(default_factory=SmoothedValue)
    front_right: SmoothedValue = field(default_factory=SmoothedValue)
    right_front: SmoothedValue = field(default_factory=SmoothedValue)
    right: SmoothedValue = field(default_factory=SmoothedValue)


def _angle_wrap_pi(angle: np.ndarray) -> np.ndarray:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def _sector_distance(
    ranges: np.ndarray,
    angles: np.ndarray,
    start_deg: float,
    end_deg: float,
    range_min: float,
    range_max: float,
    percentile: float,
) -> float:
    start = np.deg2rad(start_deg)
    end = np.deg2rad(end_deg)
    wrapped = _angle_wrap_pi(angles)
    if start <= end:
        mask = (wrapped >= start) & (wrapped <= end)
    else:
        mask = (wrapped >= start) | (wrapped <= end)

    sector = ranges[mask]
    valid = sector[np.isfinite(sector) & (sector >= range_min) & (sector <= range_max)]
    if valid.size == 0:
        return np.inf
    return float(np.percentile(valid, percentile))


def read_sectors(scan, percentile: float) -> dict[str, float]:
    ranges = np.array(scan.ranges, dtype=np.float32)
    angles = scan.angle_min + np.arange(ranges.size, dtype=np.float32) * scan.angle_increment
    range_min = max(float(scan.range_min), 0.05)
    range_max = float(scan.range_max)
    return {
        "front": _sector_distance(ranges, angles, -18, 18, range_min, range_max, percentile),
        "front_left": _sector_distance(ranges, angles, 18, 55, range_min, range_max, percentile),
        "front_right": _sector_distance(ranges, angles, -55, -18, range_min, range_max, percentile),
        "right_front": _sector_distance(ranges, angles, -75, -35, range_min, range_max, percentile),
        "right": _sector_distance(ranges, angles, -115, -70, range_min, range_max, percentile),
    }


def smooth_sectors(state: WallFollowerState, sectors: dict[str, float], alpha: float) -> dict[str, float]:
    return {
        "front": state.front.update(sectors["front"], alpha),
        "front_left": state.front_left.update(sectors["front_left"], alpha),
        "front_right": state.front_right.update(sectors["front_right"], alpha),
        "right_front": state.right_front.update(sectors["right_front"], alpha),
        "right": state.right.update(sectors["right"], alpha),
    }


def slew(current: float, target: float, max_delta: float) -> float:
    delta = clamp_axis(target - current)
    if delta > max_delta:
        delta = max_delta
    elif delta < -max_delta:
        delta = -max_delta
    return current + delta


def decide_command(args: argparse.Namespace, state: WallFollowerState, sectors: dict[str, float], now: float) -> tuple[float, float]:
    desired = args.desired_dist_m
    front = sectors["front"]
    front_left = sectors["front_left"]
    front_right = sectors["front_right"]
    right_front = sectors["right_front"]
    right = sectors["right"]

    obstacle_ahead = front < args.front_clearance_m
    near_front = front < args.slowdown_dist_m
    wall_visible = min(right, right_front) < args.wall_detect_m

    if state.mode == "SEARCHING" and wall_visible:
        state.mode = "FOLLOWING"
    elif state.mode == "FOLLOWING" and not wall_visible and front > args.slowdown_dist_m:
        state.mode = "SEARCHING"

    if obstacle_ahead:
        state.mode = "TURNING_CORNER"
        state.corner_until = max(state.corner_until, now + args.corner_hold_sec)

    if state.mode == "TURNING_CORNER" and now >= state.corner_until and front > args.front_release_m:
        state.mode = "FOLLOWING" if wall_visible else "SEARCHING"

    if state.mode == "TURNING_CORNER":
        linear_target = args.linear_speed * args.corner_linear_scale
        # Prefer the more open front side while keeping a left-turn bias for right-wall following.
        openness_bias = clamp_axis((front_left - front_right) * args.open_space_gain)
        yaw_target = args.corner_turn_speed + max(0.0, openness_bias * 0.25)
    elif state.mode == "SEARCHING":
        linear_target = args.linear_speed * args.search_linear_scale
        yaw_target = -args.search_turn_axis
    else:
        # Right-wall following: positive error means too close, so turn left.
        if np.isfinite(right):
            distance_error = desired - right
        else:
            distance_error = -desired

        angle_error = 0.0
        if (
            np.isfinite(right)
            and np.isfinite(right_front)
            and right < args.wall_detect_m * 1.8
            and right_front < args.wall_detect_m * 1.8
        ):
            angle_error = right_front - right

        yaw_target = (
            distance_error * args.distance_gain
            + angle_error * args.angle_gain
        )

        if near_front:
            front_scale = np.clip(
                (front - args.front_clearance_m) / max(args.slowdown_dist_m - args.front_clearance_m, 1.0e-6),
                0.0,
                1.0,
            )
            linear_target = args.linear_speed * (args.min_linear_scale + (1.0 - args.min_linear_scale) * front_scale)
            yaw_target += args.front_bias_gain * (1.0 - front_scale)
        else:
            linear_target = args.linear_speed

        yaw_target = max(-args.max_follow_yaw_axis, min(args.max_follow_yaw_axis, yaw_target))

    yaw_target = clamp_axis(yaw_target)
    linear_target = clamp_axis(linear_target)
    state.yaw_axis = slew(state.yaw_axis, yaw_target, args.max_yaw_step)
    state.linear_axis = slew(state.linear_axis, linear_target, args.max_linear_step)
    return state.linear_axis, state.yaw_axis

def run_wall_follower(args: argparse.Namespace) -> None:
    runtime = load_runtime()
    pdu_manager = runtime["PduManager"]()
    pdu_manager.initialize(config_path=args.config_path, comm_service=runtime["ShmCommunicationService"]())
    pdu_manager.start_service_nowait()

    if not runtime["hakopy"].init_for_external():
        raise RuntimeError("hakopy.init_for_external() failed")

    interval_sec = 1.0 / args.rate_hz
    state = WallFollowerState()
    started_at = time.monotonic()
    
    print(
        f"[INFO] Wall follower started. rate={args.rate_hz:.1f}Hz "
        f"desired_dist={args.desired_dist_m:.2f}m duration={args.duration_sec:.1f}s"
    )

    try:
        while args.duration_sec <= 0.0 or (time.monotonic() - started_at) < args.duration_sec:
            pdu_manager.run_nowait()
            
            raw_scan = pdu_manager.read_pdu_raw_data(args.robot, args.scan_pdu)
            if raw_scan is None:
                print("[WARN] No LiDAR data. Sending stop command.", file=sys.stderr)
                command = build_gamepad(0.0, 0.0)
                send_command(pdu_manager, args.robot, args.cmd_pdu, command)
                time.sleep(interval_sec)
                continue

            scan = runtime["pdu_to_py_LaserScan"](raw_scan)
            raw_sectors = read_sectors(scan, args.sector_percentile)
            sectors = smooth_sectors(state, raw_sectors, args.sensor_alpha)
            linear_axis, yaw_axis = decide_command(args, state, sectors, time.monotonic())

            now = time.monotonic()
            if now - state.last_log_at >= args.log_interval_sec:
                state.last_log_at = now
                print(
                    f"[INFO] mode={state.mode} linear={linear_axis:.2f} yaw={yaw_axis:.2f} "
                    f"front={sectors['front']:.2f} right_front={sectors['right_front']:.2f} "
                    f"right={sectors['right']:.2f}",
                    flush=True,
                )

            command = build_gamepad(linear_axis, yaw_axis)
            send_command(pdu_manager, args.robot, args.cmd_pdu, command)
            
            time.sleep(interval_sec)

    finally:
        print("[INFO] Stopping robot.")
        stop = build_gamepad(0.0, 0.0)
        send_command(pdu_manager, args.robot, args.cmd_pdu, stop)
        time.sleep(0.5)

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A wall-following controller for TurtleBot3.")
    parser.add_argument("--config-path", default=DEFAULT_CONFIG_PATH, help="PDU config JSON path.")
    parser.add_argument("--robot", default=DEFAULT_ROBOT, help="Robot name.")
    parser.add_argument("--cmd-pdu", default=DEFAULT_CMD_PDU, help="Command PDU name.")
    parser.add_argument("--scan-pdu", default=DEFAULT_SCAN_PDU, help="LaserScan PDU name.")
    parser.add_argument("--rate-hz", type=float, default=20.0, help="Controller loop rate in Hz.")
    parser.add_argument("--duration-sec", type=float, default=30.0, help="Run duration. Use 0 for endless.")
    parser.add_argument("--desired-dist-m", type=float, default=0.75, help="Desired right-wall distance in meters.")
    parser.add_argument("--linear-speed", type=float, default=0.34, help="Normal forward command axis.")
    parser.add_argument("--front-clearance-m", type=float, default=0.65, help="Start corner handling below this front distance.")
    parser.add_argument("--front-release-m", type=float, default=0.90, help="Exit corner handling above this front distance.")
    parser.add_argument("--slowdown-dist-m", type=float, default=1.20, help="Start slowing down below this front distance.")
    parser.add_argument("--wall-detect-m", type=float, default=1.25, help="Right wall detection distance.")
    parser.add_argument("--corner-turn-speed", type=float, default=0.62, help="Left turn command axis while clearing corners.")
    parser.add_argument("--corner-hold-sec", type=float, default=0.75, help="Minimum time to keep corner-turn mode.")
    parser.add_argument("--corner-linear-scale", type=float, default=0.20, help="Forward speed scale while cornering.")
    parser.add_argument("--search-linear-scale", type=float, default=0.65, help="Forward speed scale while searching.")
    parser.add_argument("--search-turn-axis", type=float, default=0.20, help="Right turn command while searching for a wall.")
    parser.add_argument("--distance-gain", type=float, default=1.15, help="Gain for right-wall distance error.")
    parser.add_argument("--angle-gain", type=float, default=0.70, help="Gain for right-front minus right distance.")
    parser.add_argument("--front-bias-gain", type=float, default=0.35, help="Extra left turn as front clearance shrinks.")
    parser.add_argument("--open-space-gain", type=float, default=0.7, help="Corner turn bias toward the more open front side.")
    parser.add_argument("--min-linear-scale", type=float, default=0.42, help="Minimum speed scale near front obstacles.")
    parser.add_argument("--sensor-alpha", type=float, default=0.28, help="Low-pass alpha for sector distances.")
    parser.add_argument("--sector-percentile", type=float, default=20.0, help="Distance percentile used per sector.")
    parser.add_argument("--max-yaw-step", type=float, default=0.08, help="Max yaw axis change per control cycle.")
    parser.add_argument("--max-follow-yaw-axis", type=float, default=0.36, help="Max yaw axis during normal wall following.")
    parser.add_argument("--max-linear-step", type=float, default=0.04, help="Max linear axis change per control cycle.")
    parser.add_argument("--log-interval-sec", type=float, default=0.5, help="Status log interval.")
    return parser.parse_args()

def main() -> int:
    try:
        run_wall_follower(parse_args())
        return 0
    except KeyboardInterrupt:
        print("[INFO] Interrupted by user.")
        return 130
    except Exception as exc:
        print(f"[ERROR] An unexpected error occurred: {exc}", file=sys.stderr)
        return 1

if __name__ == "__main__":
    raise SystemExit(main())
