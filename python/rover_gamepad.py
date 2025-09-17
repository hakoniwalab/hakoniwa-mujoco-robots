#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import pygame
import time
import os
import hakoniwa_pdu.apps.drone.hakosim as hakosim
from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_pytype_GameControllerOperation import GameControllerOperation
from rc_utils.rc_utils import RcConfig, StickMonitor

DEFAULT_CONFIG_PATH = "rc_config/ps4-control.json"
TARGET_KEYWORDS = ["Xbox", "360"]  # 柔軟な名称判定

DEADZONE = 0.05

def joystick_control(config_path, robot_name, channel_id, stick_monitor: StickMonitor, joystick):
    try:
        client = hakosim.MultirotorClient(config_path)
        client.confirmConnection()
        client.enableApiControl(True)
        client.armDisarm(True)

        axis_count = joystick.get_numaxes()
        button_count = joystick.get_numbuttons()

        while True:
            data = client.getGameJoystickData()
            data.axis = list(data.axis)
            data.button = list(data.button)
            client.run_nowait()

            pygame.event.pump()

            for event in pygame.event.get():
                # 指定ジョイスティック以外のイベントは無視
                if hasattr(event, 'instance_id') and event.instance_id != joystick.get_instance_id():
                    continue
                if hasattr(event, 'joy') and event.joy != joystick.get_id():
                    continue

                if event.type == pygame.JOYAXISMOTION:
                    axis_id = event.axis
                    if axis_id < axis_count:
                        op_index = stick_monitor.rc_config.get_op_index(axis_id)
                        stick_value = stick_monitor.stick_value(axis_id, event.value)
                        if abs(stick_value) > 0.1:
                            pass
                            #print(f"stick event: stick_index={axis_id} op_index={op_index} event.value={event.value:.3f} stick_value={stick_value:.3f}")
                        if op_index < len(data.axis):
                            data.axis[op_index] = stick_value
                    else:
                        print(f'[WARN] Unsupported axis index: {axis_id}')
                elif event.type in (pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP):
                    button_id = event.button
                    if button_id < button_count:
                        is_pressed = (event.type == pygame.JOYBUTTONDOWN)
                        data.button[button_id] = is_pressed
                        if is_pressed:
                            print(f'[INFO] Button {button_id} pressed')
                    else:
                        print(f'[WARN] Unsupported button index: {button_id}')

            client.putGameJoystickData(data)
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        pygame.joystick.quit()
        pygame.quit()

def main():
    if len(sys.argv) != 3 and len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <robot_name> <config_path> [rc_config_path]")
        return 1

    robot_name = sys.argv[1]
    config_path = sys.argv[2]
    rc_config_path = sys.argv[3] if len(sys.argv) == 4 else os.getenv("RC_CONFIG_PATH", DEFAULT_CONFIG_PATH)

    if not os.path.exists(config_path):
        print(f"[ERROR] Config file not found at '{config_path}'")
        return 1
    if not os.path.exists(rc_config_path):
        print(f"[ERROR] RC config file not found at '{rc_config_path}'")
        return 1

    rc_config = RcConfig(rc_config_path)
    print("Controller: ", rc_config_path)
    print("Mode: ", rc_config.config['mode'])
    stick_monitor = StickMonitor(rc_config)


    pygame.init()
    pygame.joystick.init()

    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("[ERROR] No joystick connected.")
        return 1

    joystick = None
    for i in range(joystick_count):
        js = pygame.joystick.Joystick(i)
        js.init()
        name = js.get_name()
        print(f"[INFO] Detected Joystick {i}: {name}")
        if all(k.lower() in name.lower() for k in TARGET_KEYWORDS):
            joystick = js
            print(f"[INFO] Selected Joystick {i}: {name}")
            break

    if joystick is None:
        print(f"[ERROR] Joystick with keywords {TARGET_KEYWORDS} not found.")
        return 1

    print(f"[INFO] Axis count: {joystick.get_numaxes()}")
    print(f"[INFO] Button count: {joystick.get_numbuttons()}")

    joystick_control(config_path, robot_name=robot_name, channel_id=0, stick_monitor=stick_monitor, joystick=joystick)
    return 0

if __name__ == "__main__":
    sys.exit(main())
