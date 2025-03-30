#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import pygame
import time
import os
import hako_pdu
import hakopy
from pdu_data import PduData

DEFAULT_CONFIG_PATH = "rc_config/ps4-control.json"
AXIS_COUNT = 6
BUTTON_COUNT = 15
DEADZONE = 0.05

def joystick_control(config_path, robot_name, channel_id):
    try:
        hako_binary_path = os.getenv('HAKO_BINARY_PATH', '/usr/local/lib/hakoniwa/hako_binary/offset')
        pdu_manager = hako_pdu.HakoPduManager(hako_binary_path, config_path)

        gamepad_pdu = PduData(pdu_manager, robot_name, channel_id)

        while True:
            gamepad_data = gamepad_pdu.read()
            if gamepad_data is None:
                print("[ERROR] Failed to read gamepad data.")
                break
            pygame.event.pump()
            gamepad_data['axis'] = list(gamepad_data['axis']) 
            gamepad_data['button'] = list(gamepad_data['button'])
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    axis_id = event.axis
                    if axis_id < AXIS_COUNT:
                        value = event.value
                        if abs(value) < DEADZONE:
                            value = 0.0
                        gamepad_data['axis'][axis_id] = value
                        #if value != 0.0:
                        #    print(f'[INFO] Axis {axis_id}: {value:.2f}')
                    else:
                        print(f'[WARN] Unsupported axis index: {axis_id}')
                elif event.type in (pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP):
                    button_id = event.button
                    if button_id < BUTTON_COUNT:
                        is_pressed = (event.type == pygame.JOYBUTTONDOWN)
                        gamepad_data['button'][button_id] = is_pressed
                        if is_pressed:
                            print(f'[INFO] Button {button_id} pressed')
                    else:
                        print(f'[WARN] Unsupported button index: {button_id}')

            gamepad_pdu.write(gamepad_data)
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        pygame.joystick.quit()
        pygame.quit()

def main():
    if len(sys.argv) != 2 and len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <config_path> [rc_config_path]")
        return 1

    config_path = sys.argv[1]
    rc_config_path = sys.argv[2] if len(sys.argv) == 3 else os.getenv("RC_CONFIG_PATH", DEFAULT_CONFIG_PATH)

    if not os.path.exists(config_path):
        print(f"[ERROR] Config file not found at '{config_path}'")
        return 1
    #if not os.path.exists(rc_config_path):
    #    print(f"[ERROR] RC config file not found at '{rc_config_path}'")
    #    return 1

    if not hakopy.init_for_external():
        raise RuntimeError("ERROR: hakopy.init_for_external() failed.")

    pygame.init()
    pygame.joystick.init()

    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("[ERROR] No joystick connected.")
        return 1

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f'[INFO] Joystick: {joystick.get_name()}')
    print(f'[INFO] Buttons: {joystick.get_numbuttons()}')

    joystick_control(config_path, robot_name="forklift", channel_id=0)
    return 0

if __name__ == "__main__":
    sys.exit(main())
