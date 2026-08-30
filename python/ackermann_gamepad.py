#!/usr/bin/env python3
"""Publish PS5 gamepad state to the generic Ackermann vehicle."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import hakopy
import pygame
from hakoniwa_pdu.impl.shm_communication_service import ShmCommunicationService
from hakoniwa_pdu.pdu_manager import PduManager
from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_conv_GameControllerOperation import (
    py_to_pdu_GameControllerOperation,
)
from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_pytype_GameControllerOperation import (
    GameControllerOperation,
)

from rc_utils.rc_utils import RcConfig, StickMonitor


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PDU_DEF = REPO_ROOT / "config/ackermann-gamepad-pdudef-compact.json"
DEFAULT_RC_CONFIG = REPO_ROOT / "python/rc_config/ps4-control.json"
TARGET_NAMES = ("Wireless Controller", "DualSense")
AXIS_COUNT = 6
BUTTON_COUNT = 15
DEADZONE = 0.05


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Control AckermannVehicle with a PS5 controller.")
    parser.add_argument("--pdu-def", default=str(DEFAULT_PDU_DEF))
    parser.add_argument("--rc-config", default=str(DEFAULT_RC_CONFIG))
    parser.add_argument("--robot", default="AckermannVehicle")
    parser.add_argument("--pdu", default="hako_cmd_game")
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument(
        "--check-controller",
        action="store_true",
        help="detect the PS5 controller and exit without touching Hakoniwa",
    )
    return parser.parse_args()


def select_joystick():
    for index in range(pygame.joystick.get_count()):
        joystick = pygame.joystick.Joystick(index)
        joystick.init()
        name = joystick.get_name()
        print(f"[INFO] Detected Joystick {index}: {name}")
        if any(candidate.lower() in name.lower() for candidate in TARGET_NAMES):
            print(f"[INFO] Selected Joystick {index}: {name}")
            return joystick
    return None


def main() -> int:
    args = parse_args()
    pygame.init()
    pygame.joystick.init()
    joystick = select_joystick()
    if joystick is None:
        print("[ERROR] PS5/DualSense controller was not found.")
        return 1
    print(f"[INFO] Axis count: {joystick.get_numaxes()}")
    print(f"[INFO] Button count: {joystick.get_numbuttons()}")
    if args.check_controller:
        pygame.joystick.quit()
        pygame.quit()
        return 0

    monitor = StickMonitor(RcConfig(args.rc_config))
    gamepad = GameControllerOperation()
    gamepad.axis = [0.0] * AXIS_COUNT
    gamepad.button = [False] * BUTTON_COUNT

    manager = PduManager()
    manager.initialize(
        config_path=str(Path(args.pdu_def).resolve()),
        comm_service=ShmCommunicationService(),
    )
    manager.start_service_nowait()
    if hakopy.init_for_external() is False:
        print("[ERROR] hakopy.init_for_external() failed")
        return 1

    period = 1.0 / max(args.rate_hz, 1.0)
    print("[INFO] Left stick: steering; right stick vertical: throttle/reverse")
    try:
        while True:
            manager.run_nowait()
            pygame.event.pump()
            for event in pygame.event.get():
                if hasattr(event, "instance_id") and event.instance_id != joystick.get_instance_id():
                    continue
                if event.type == pygame.JOYAXISMOTION and event.axis < joystick.get_numaxes():
                    output_index = monitor.rc_config.get_op_index(event.axis)
                    if output_index is None or output_index >= len(gamepad.axis):
                        continue
                    value = monitor.stick_value(event.axis, event.value)
                    gamepad.axis[output_index] = 0.0 if abs(value) < DEADZONE else value
                elif event.type in (pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP):
                    if event.button < len(gamepad.button):
                        gamepad.button[event.button] = event.type == pygame.JOYBUTTONDOWN

            payload = py_to_pdu_GameControllerOperation(gamepad)
            manager.flush_pdu_raw_data_nowait(args.robot, args.pdu, payload)
            time.sleep(period)
    except KeyboardInterrupt:
        print("[INFO] Ackermann gamepad sender stopped.")
    finally:
        pygame.joystick.quit()
        pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
