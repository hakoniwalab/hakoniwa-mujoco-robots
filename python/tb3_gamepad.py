#!/usr/bin/env python3

import json
import os
import sys
from pathlib import Path

import hakopy
import pygame

from rc_utils.rc_utils import RcConfig, StickMonitor
from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_conv_GameControllerOperation import py_to_pdu_GameControllerOperation
from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_pytype_GameControllerOperation import GameControllerOperation
from hakoniwa_pdu_endpoint.c_endpoint import Endpoint, PduKey

DEFAULT_ENDPOINT_CONFIG = "config/endpoint/tb3_gamepad_endpoint.json"
DEFAULT_RC_CONFIG = "python/rc_config/ps4-control.json"
DEFAULT_ASSET_NAME = "tb3_gamepad"
TARGET_KEYWORDS = ["Wireless Controller"]
DEADZONE = 0.05
AXIS_COUNT = 6
BUTTON_COUNT = 15
STEP_USEC = 20000

REPO_ROOT = Path(__file__).resolve().parents[1]

_endpoint = None
_joystick = None
_stick_monitor = None
_gamepad = None


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
    try:
        _endpoint.post_start()
        print("TB3_GAMEPAD_ENDPOINT_POST_START_OK")
    except Exception as exc:
        print(f"TB3_GAMEPAD_ENDPOINT_POST_START_FAILED:{exc}")
        return -1
    return 0


def my_on_reset(_context):
    print("TB3_GAMEPAD_RESET_CALLBACK")
    return 0


def my_on_manual_timing_control(_context):
    key = PduKey(robot="TB3", pdu="hako_cmd_game")

    while True:
        update_gamepad_state()
        payload = bytes(py_to_pdu_GameControllerOperation(_gamepad))
        try:
            _endpoint.send_by_name(key, payload)
        except Exception as exc:
            print(f"TB3_GAMEPAD_WRITE_FAILED:{exc}")
        result = hakopy.usleep(STEP_USEC)
        if not result:
            break
    return 0


MY_CALLBACK = {
    "on_initialize": my_on_initialize,
    "on_simulation_step": None,
    "on_manual_timing_control": my_on_manual_timing_control,
    "on_reset": my_on_reset,
}


def main() -> int:
    global _endpoint, _joystick, _stick_monitor, _gamepad

    endpoint_config_path = sys.argv[1] if len(sys.argv) >= 2 else os.getenv("TB3_ENDPOINT_CONFIG", DEFAULT_ENDPOINT_CONFIG)
    rc_config_path = sys.argv[2] if len(sys.argv) >= 3 else os.getenv("RC_CONFIG_PATH", DEFAULT_RC_CONFIG)
    asset_name = sys.argv[3] if len(sys.argv) >= 4 else os.getenv("TB3_GAMEPAD_ASSET_NAME", DEFAULT_ASSET_NAME)

    endpoint_config_path = str((REPO_ROOT / endpoint_config_path).resolve()) if not os.path.isabs(endpoint_config_path) else endpoint_config_path
    rc_config_path = str((REPO_ROOT / rc_config_path).resolve()) if not os.path.isabs(rc_config_path) else rc_config_path

    if not os.path.exists(endpoint_config_path):
        print(f"[ERROR] Endpoint config file not found at '{endpoint_config_path}'")
        return 1
    if not os.path.exists(rc_config_path):
        print(f"[ERROR] RC config file not found at '{rc_config_path}'")
        return 1

    pygame.init()
    pygame.joystick.init()
    _joystick = select_joystick()
    if _joystick is None:
        return 1

    print(f"[INFO] Axis count: {_joystick.get_numaxes()}")
    print(f"[INFO] Button count: {_joystick.get_numbuttons()}")

    _stick_monitor = StickMonitor(RcConfig(rc_config_path))
    _gamepad = build_empty_gamepad()

    with open(endpoint_config_path, "r", encoding="utf-8") as f:
        ep_config = json.load(f)
    pdu_def_relpath = ep_config.get("pdu_def_path")
    if pdu_def_relpath is None:
        print("TB3_GAMEPAD_NO_PDU_CONFIG_PATH")
        return 1
    pdu_config_path = str((Path(endpoint_config_path).parent / pdu_def_relpath).resolve())

    _endpoint = Endpoint("tb3_gamepad_endpoint", "inout")
    try:
        _endpoint.open(endpoint_config_path)
        _endpoint.start()
        print("TB3_GAMEPAD_ENDPOINT_READY")
    except Exception as exc:
        print(f"TB3_GAMEPAD_ENDPOINT_OPEN_FAILED:{exc}")
        return 1

    ret = hakopy.asset_register(
        asset_name,
        pdu_config_path,
        MY_CALLBACK,
        STEP_USEC,
        hakopy.HAKO_ASSET_MODEL_CONTROLLER,
    )
    if ret is False:
        print("TB3_GAMEPAD_REGISTER_FAILED")
        return 1

    try:
        ret = hakopy.start()
        print(f"TB3_GAMEPAD_START_RETURN:{ret}")
    finally:
        try:
            zero = build_empty_gamepad()
            _endpoint.send_by_name(PduKey(robot="TB3", pdu="hako_cmd_game"), bytes(py_to_pdu_GameControllerOperation(zero)))
        except Exception:
            pass
        try:
            _endpoint.stop()
        except Exception:
            pass
        try:
            _endpoint.close()
        except Exception:
            pass
        pygame.joystick.quit()
        pygame.quit()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
