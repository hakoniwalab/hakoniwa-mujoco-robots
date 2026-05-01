#!/usr/bin/env python3

import json
import os
import sys
from pathlib import Path

import hakopy
import pygame
import time

from rc_utils.rc_utils import RcConfig, StickMonitor
from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_conv_GameControllerOperation import py_to_pdu_GameControllerOperation
from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_pytype_GameControllerOperation import GameControllerOperation
from hakoniwa_pdu.impl.shm_communication_service import ShmCommunicationService
from hakoniwa_pdu.pdu_manager import PduManager

DEFAULT_RC_CONFIG = "python/rc_config/ps4-control.json"
DEFAULT_ASSET_NAME = "tb3_gamepad"
TARGET_KEYWORDS = ["Wireless Controller"]
DEADZONE = 0.05
AXIS_COUNT = 6
BUTTON_COUNT = 15
STEP_USEC = 20000

REPO_ROOT = Path(__file__).resolve().parents[1]

_joystick = None
_stick_monitor = None
_gamepad = None
pdu_manager = None


def select_joystick():
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("[ERROR] No joystick connected.")
        return None

    for i in range(joystick_count):
        js = pygame.joystick.Joystick(i)
        js.init()
        name = js.get_name()
        print(f"[INFO] Detected Joystick {i}: {name}")
        if all(keyword.lower() in name.lower() for keyword in TARGET_KEYWORDS):
            print(f"[INFO] Selected Joystick {i}: {name}")
            return js

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"[WARN] Fallback to Joystick 0: {js.get_name()}")
    return js


def build_empty_gamepad() -> GameControllerOperation:
    data = GameControllerOperation()
    data.axis = [0.0] * AXIS_COUNT
    data.button = [False] * BUTTON_COUNT
    return data


def update_gamepad_state():
    pygame.event.pump()
    for event in pygame.event.get():
        if hasattr(event, "instance_id") and event.instance_id != _joystick.get_instance_id():
            continue
        if hasattr(event, "joy") and event.joy != _joystick.get_id():
            continue

        if event.type == pygame.JOYAXISMOTION:
            axis_id = event.axis
            if axis_id >= _joystick.get_numaxes():
                continue
            op_index = _stick_monitor.rc_config.get_op_index(axis_id)
            if op_index is None or op_index >= len(_gamepad.axis):
                continue
            stick_value = _stick_monitor.stick_value(axis_id, event.value)
            if abs(stick_value) < DEADZONE:
                stick_value = 0.0
            _gamepad.axis[op_index] = stick_value
        elif event.type in (pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP):
            button_id = event.button
            if button_id >= min(_joystick.get_numbuttons(), len(_gamepad.button)):
                continue
            _gamepad.button[button_id] = event.type == pygame.JOYBUTTONDOWN


def my_on_initialize(_context):
    return 0


def my_on_reset(_context):
    print("TB3_GAMEPAD_RESET_CALLBACK")
    return 0


def my_on_manual_timing_control(_context):
    global pdu_manager
    robot="TB3"
    pdu = "hako_cmd_game"

    while True:
        pdu_manager.run_nowait()
        update_gamepad_state()
        payload = py_to_pdu_GameControllerOperation(_gamepad)
        try:
            #print(f"gamepad axis: {_gamepad.axis}, button: {_gamepad.button}")
            pdu_manager.flush_pdu_raw_data_nowait(robot, pdu, payload)
        except Exception as exc:
            print(f"TB3_GAMEPAD_WRITE_FAILED:{exc}")
        
        time.sleep(STEP_USEC / 1.0e6)
    return 0


MY_CALLBACK = {
    "on_initialize": my_on_initialize,
    "on_simulation_step": None,
    "on_manual_timing_control": my_on_manual_timing_control,
    "on_reset": my_on_reset,
}


def main() -> int:
    global pdu_manager, _joystick, _stick_monitor, _gamepad

    pdu_config_path = sys.argv[1] if len(sys.argv) >= 2 else os.getenv("PDU_CONFIG_PATH", "config/tb3-pdudef-compact.json")
    rc_config_path = sys.argv[2] if len(sys.argv) >= 3 else os.getenv("RC_CONFIG_PATH", DEFAULT_RC_CONFIG)
    asset_name = sys.argv[3] if len(sys.argv) >= 4 else os.getenv("TB3_GAMEPAD_ASSET_NAME", DEFAULT_ASSET_NAME)

    rc_config_path = str((REPO_ROOT / rc_config_path).resolve()) if not os.path.isabs(rc_config_path) else rc_config_path

    pdu_manager = PduManager()
    pdu_manager.initialize(config_path=pdu_config_path, comm_service=ShmCommunicationService())
    pdu_manager.start_service_nowait()

    pygame.init()
    pygame.joystick.init()
    _joystick = select_joystick()
    if _joystick is None:
        return 1

    print(f"[INFO] Axis count: {_joystick.get_numaxes()}")
    print(f"[INFO] Button count: {_joystick.get_numbuttons()}")

    _stick_monitor = StickMonitor(RcConfig(rc_config_path))
    _gamepad = build_empty_gamepad()


    ret = hakopy.init_for_external()
    if ret is False:
        print("TB3_GAMEPAD_REGISTER_FAILED")
        return 1

    try:
        my_on_manual_timing_control(None)  # 直接呼び出してループを開始
    finally:
        pygame.joystick.quit()
        pygame.quit()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
