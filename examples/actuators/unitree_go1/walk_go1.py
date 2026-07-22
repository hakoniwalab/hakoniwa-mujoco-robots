#!/usr/bin/env python3
"""Send simple open-loop joint target patterns to the Unitree Go1 Hakoniwa asset.

These are experimental open-loop gait demos. They are not verified walking
controllers.
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from pathlib import Path

import hakopy
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_JointState import pdu_to_py_JointState
from hakoniwa_pdu.pdu_msgs.std_msgs.pdu_conv_Float64MultiArray import (
    py_to_pdu_Float64MultiArray,
)
from hakoniwa_pdu.pdu_msgs.std_msgs.pdu_pytype_Float64MultiArray import (
    Float64MultiArray,
)
from hakoniwa_pdu_endpoint.c_endpoint import Endpoint, PduKey


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_ENDPOINT_CONFIG = REPO_ROOT / "config/endpoint/go1_joint_endpoint.json"
DEFAULT_PDU_DEF = REPO_ROOT / "config/go1-joint-pdudef-compact.json"

JOINT_ORDER = [
    "FR_hip",
    "FR_thigh",
    "FR_calf",
    "FL_hip",
    "FL_thigh",
    "FL_calf",
    "RR_hip",
    "RR_thigh",
    "RR_calf",
    "RL_hip",
    "RL_thigh",
    "RL_calf",
]
HOME = [0.0, 0.9, -1.8] * 4
LEG_OFFSETS = {
    "FR": 0,
    "FL": 3,
    "RR": 6,
    "RL": 9,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send an experimental trot-like open-loop pattern to Go1JointAsset."
    )
    parser.add_argument("--endpoint-config", default=str(DEFAULT_ENDPOINT_CONFIG))
    parser.add_argument("--pdu-def", default=str(DEFAULT_PDU_DEF))
    parser.add_argument("--sender-asset-name", default="Go1WalkSender")
    parser.add_argument("--target-robot-name", default="Go1JointAsset")
    parser.add_argument("--command-pdu-name", default="joint_position_targets")
    parser.add_argument("--joint-state-pdu-name", default="joint_states")
    parser.add_argument("--endpoint-name", default="go1_walk_sender")
    parser.add_argument("--delta-usec", type=int, default=20_000)
    parser.add_argument("--send-rate-hz", type=float, default=40.0)
    parser.add_argument("--duration-sec", type=float, default=6.0)
    parser.add_argument(
        "--profile",
        choices=["creep", "trot"],
        default="creep",
        help="creep is slower and more stable; trot is showier and easier to tip.",
    )
    parser.add_argument("--frequency-hz", type=float, default=0.55)
    parser.add_argument("--thigh-amp", type=float, default=0.10)
    parser.add_argument("--calf-lift", type=float, default=0.12)
    parser.add_argument("--hip-sway", type=float, default=0.01)
    parser.add_argument(
        "--reverse",
        action="store_true",
        help="Deprecated alias kept for experiments; the stable backward sweep is now the default.",
    )
    parser.add_argument(
        "--forward",
        action="store_true",
        help="Try the opposite fore/aft thigh sweep. This may step in place or tip depending on the model.",
    )
    parser.add_argument("--warmup-sec", type=float, default=0.8)
    parser.add_argument("--cooldown-sec", type=float, default=0.6)
    parser.add_argument(
        "--no-read-joint-state",
        action="store_true",
        help="Disable reading sensor_msgs/JointState from the C++ asset.",
    )
    return parser.parse_args()


def smoothstep(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def make_payload(values: list[float]) -> bytes:
    msg = Float64MultiArray()
    msg.data = [float(value) for value in values]
    return bytes(py_to_pdu_Float64MultiArray(msg))


def set_leg(
    targets: list[float],
    leg: str,
    phase: float,
    thigh_amp: float,
    calf_lift: float,
    hip_sway: float,
    blend: float,
) -> None:
    base = LEG_OFFSETS[leg]
    theta = 2.0 * math.pi * phase

    diagonal_sign = 1.0 if leg in {"FR", "RL"} else -1.0
    swing = max(0.0, math.sin(theta))
    stance = -max(0.0, -math.sin(theta))

    hip = diagonal_sign * hip_sway * math.sin(theta)
    thigh = thigh_amp * math.cos(theta)
    calf = calf_lift * swing + 0.08 * stance

    desired = [
        HOME[base + 0] + hip,
        HOME[base + 1] + thigh,
        HOME[base + 2] + calf,
    ]
    for i in range(3):
        targets[base + i] = HOME[base + i] + (desired[i] - HOME[base + i]) * blend


def set_creep_leg(
    targets: list[float],
    leg: str,
    phase: float,
    thigh_amp: float,
    calf_lift: float,
    hip_sway: float,
    direction: float,
    blend: float,
) -> None:
    base = LEG_OFFSETS[leg]
    swing_ratio = 0.25
    diagonal_sign = 1.0 if leg in {"FR", "RL"} else -1.0

    if phase < swing_ratio:
        swing_t = phase / swing_ratio
        lift = math.sin(math.pi * swing_t)
        fore_aft = -thigh_amp + 2.0 * thigh_amp * smoothstep(swing_t)
        calf = calf_lift * lift
        phase_hip_sway = math.sin(math.pi * swing_t)
    else:
        stance_t = (phase - swing_ratio) / (1.0 - swing_ratio)
        fore_aft = thigh_amp - 2.0 * thigh_amp * stance_t
        calf = -0.02
        phase_hip_sway = 0.0

    desired = [
        HOME[base + 0] + diagonal_sign * hip_sway * phase_hip_sway,
        HOME[base + 1] + direction * fore_aft,
        HOME[base + 2] + calf,
    ]
    for i in range(3):
        targets[base + i] = HOME[base + i] + (desired[i] - HOME[base + i]) * blend


def targets_at_time(elapsed_sec: float, args: argparse.Namespace) -> tuple[str, list[float], bool]:
    total_sec = args.warmup_sec + args.duration_sec + args.cooldown_sec
    if elapsed_sec >= total_sec:
        return "done", list(HOME), True

    if elapsed_sec < args.warmup_sec:
        blend = smoothstep(elapsed_sec / max(args.warmup_sec, 1.0e-6))
        gait_time = 0.0
        phase_name = "warmup"
    elif elapsed_sec < args.warmup_sec + args.duration_sec:
        blend = 1.0
        gait_time = elapsed_sec - args.warmup_sec
        phase_name = "trot"
    else:
        cooldown_t = elapsed_sec - args.warmup_sec - args.duration_sec
        blend = 1.0 - smoothstep(cooldown_t / max(args.cooldown_sec, 1.0e-6))
        gait_time = args.duration_sec
        phase_name = "cooldown"

    phase_a = (gait_time * args.frequency_hz) % 1.0
    targets = list(HOME)
    if args.profile == "trot":
        phase_b = (phase_a + 0.5) % 1.0
        set_leg(targets, "FR", phase_a, args.thigh_amp, args.calf_lift, args.hip_sway, blend)
        set_leg(targets, "RL", phase_a, args.thigh_amp, args.calf_lift, args.hip_sway, blend)
        set_leg(targets, "FL", phase_b, args.thigh_amp, args.calf_lift, args.hip_sway, blend)
        set_leg(targets, "RR", phase_b, args.thigh_amp, args.calf_lift, args.hip_sway, blend)
    else:
        direction = -1.0 if args.forward else 1.0
        leg_phase_offsets = {
            "FR": 0.00,
            "RL": 0.25,
            "FL": 0.50,
            "RR": 0.75,
        }
        phase_name = "creep" if phase_name == "trot" else phase_name
        for leg, offset in leg_phase_offsets.items():
            set_creep_leg(
                targets,
                leg,
                (phase_a + offset) % 1.0,
                args.thigh_amp,
                args.calf_lift,
                args.hip_sway,
                direction,
                blend,
            )
    return phase_name, targets, False


def compact_joint_state(joint_state) -> str:
    wanted = {"FR_thigh", "FL_thigh", "RR_thigh", "RL_thigh"}
    parts: list[str] = []
    for index, name in enumerate(joint_state.name):
        if name not in wanted:
            continue
        position = joint_state.position[index] if index < len(joint_state.position) else 0.0
        velocity = joint_state.velocity[index] if index < len(joint_state.velocity) else 0.0
        parts.append(f"{name}:pos={position:.3f},vel={velocity:.3f}")
    return " | ".join(parts)


def main() -> int:
    args = parse_args()
    endpoint_config = str(Path(args.endpoint_config).resolve())
    pdu_def = str(Path(args.pdu_def).resolve())
    command_key = PduKey(args.target_robot_name, args.command_pdu_name)
    joint_state_key = PduKey(args.target_robot_name, args.joint_state_pdu_name)
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
            latest_joint_state = None
            joint_state_size = 0
            skipped_invalid_joint_state = 0
            if not args.no_read_joint_state:
                joint_state_size = endpoint.get_pdu_size(joint_state_key)
            print(
                "[INFO] Go1 walk sender callback started: "
                f"robot={args.target_robot_name} command={args.command_pdu_name} "
                f"profile={args.profile} freq={args.frequency_hz:.2f}Hz order={','.join(JOINT_ORDER)}"
            )
            while not shutdown.is_set():
                if joint_state_size > 0:
                    raw_joint_state = endpoint.recv_by_name(joint_state_key, joint_state_size)
                    if raw_joint_state:
                        if isinstance(raw_joint_state, tuple):
                            raw_joint_state = raw_joint_state[0]
                        try:
                            latest_joint_state = pdu_to_py_JointState(raw_joint_state)
                        except Exception as exc:
                            skipped_invalid_joint_state += 1
                            if skipped_invalid_joint_state == 1:
                                print(
                                    "[INFO] Skipping invalid initial joint_states PDU "
                                    f"until publisher writes the first frame: {exc}"
                                )

                if elapsed_usec >= next_send_usec:
                    elapsed_sec = elapsed_usec / 1_000_000.0
                    phase, targets, done = targets_at_time(elapsed_sec, args)
                    endpoint.send_by_name(command_key, make_payload(targets))
                    if (count % 20) == 0 or done:
                        message = (
                            f"sent walk phase={phase} "
                            f"FR_thigh={targets[1]:.3f} FL_thigh={targets[4]:.3f} "
                            f"RR_thigh={targets[7]:.3f} RL_thigh={targets[10]:.3f}"
                        )
                        if latest_joint_state is not None:
                            message += " | joint_states " + compact_joint_state(latest_joint_state)
                        print(message)
                    count += 1
                    next_send_usec += send_interval_usec
                    if done:
                        shutdown.set()

                if hakopy.usleep(args.delta_usec) is False:
                    break
                elapsed_usec += args.delta_usec
                time.sleep(args.delta_usec / 1_000_000.0)

            endpoint.send_by_name(command_key, make_payload(HOME))
        except Exception as exc:
            print(f"[ERROR] Go1 walk sender callback failed: {exc}")
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

    print("[INFO] Go1 walk sender is registered.")
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
