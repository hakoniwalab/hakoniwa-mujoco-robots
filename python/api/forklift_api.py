# api/forklift.py

from pdu.pdu_data import PduData
import time

class ForkliftAPI:
    # Axis/Button mappings (consistent with GamePad PDU mapping)
    AXIS_YAW       = 2  # Left stick LR (yaw control)
    AXIS_LIFT      = 1  # Left stick UD (lift control)
    AXIS_FORWARD   = 3  # Right stick UD (move forward/backward)
    BUTTON_ESTOP   = 0  # Cross button (emergency stop)

    SLEEP_INTERVAL = 0.001  # Sleep interval for control loop
    MOVE_SPEED = 0.5  # Speed for moving forward/backward

    def __init__(self, pdu_manager, robot_name="forklift"):
        self.forklift_gamepad = PduData(pdu_manager, robot_name, 0)
        self.forklift_pos = PduData(pdu_manager, robot_name, 1)
        self.forklift_height = PduData(pdu_manager, robot_name, 2)

    def get_yaw_degree(self):
        # Convert radians to degrees for the gamepad axis
        pos = self.forklift_pos.read()
        #print(f"[INFO] Forklift position data: {pos}")
        yaw_rad = pos['angular']['z']
        yaw_degree = yaw_rad * (180.0 / 3.14159)
        return yaw_degree
    
    def stop(self):
        gamepad_data = self.forklift_gamepad.read()
        gamepad_data['axis'] = list(gamepad_data['axis'])
        gamepad_data['axis'][self.AXIS_YAW] = 0.0
        gamepad_data['axis'][self.AXIS_LIFT] = 0.0
        gamepad_data['axis'][self.AXIS_FORWARD] = 0.0
        gamepad_data['button'] = list(gamepad_data['button'])
        gamepad_data['button'][self.BUTTON_ESTOP] = False
        self.forklift_gamepad.write(gamepad_data)
        time.sleep(self.SLEEP_INTERVAL)
    
    def _turn_to_yaw_degree(self, target_yaw_degree):
        target_yaw_degree = self._normalize_angle(target_yaw_degree)
        while True:
            current_yaw = self._normalize_angle(self.get_yaw_degree())
            print(f"[INFO] Current yaw degree: {current_yaw:.2f}, Target yaw degree: {target_yaw_degree:.2f}")
            error = target_yaw_degree - current_yaw
            if abs(error) < 0.1:
                self.stop()
                print(f"[INFO] Reached target yaw degree: {target_yaw_degree:.2f}")
                break
            gamepad_data = self.forklift_gamepad.read()
            gamepad_data['axis'] = list(gamepad_data['axis'])
            gamepad_data['axis'][self.AXIS_YAW] = -1.0 if error > 0 else 1.0
            self.forklift_gamepad.write(gamepad_data)
            time.sleep(self.SLEEP_INTERVAL)

    def _normalize_angle(self, degree):
        # -180〜180 に正規化
        while degree > 180:
            degree -= 360
        while degree < -180:
            degree += 360
        return degree

    def lift_move(self, target_height):
        while True:
            current_height = self.get_height()
            print(f"[INFO] Current height: {current_height:.2f}, Target height: {target_height:.2f}")
            error = target_height - current_height
            if abs(current_height - target_height) < 0.01:
                self.stop()
                print(f"[INFO] Reached target height: {current_height:.2f}")
                break
            # Move the lift
            gamepad_data = self.forklift_gamepad.read()
            gamepad_data['axis'] = list(gamepad_data['axis'])
            gamepad_data['axis'][self.AXIS_LIFT] = -1.0 if error > 0 else 1.0
            self.forklift_gamepad.write(gamepad_data)
            time.sleep(self.SLEEP_INTERVAL)

    def move_forward(self, distance):
        move_distance = 0.0
        initial_pos = (self.get_position()['linear']['x'], self.get_position()['linear']['y'])
        while True:
            current_pos =  (self.get_position()['linear']['x'], self.get_position()['linear']['y'])
            move_distance = ((current_pos[0] - initial_pos[0]) ** 2 + (current_pos[1] - initial_pos[1]) ** 2) ** 0.5
            print(f"[INFO] Current position: {current_pos}, Target distance: {distance}")
            error = distance - move_distance
            if abs(error) < 0.01:
                self.stop()
                print(f"[INFO] Reached target distance: {distance}")
                break
            # Move the forklift forward
            gamepad_data = self.forklift_gamepad.read()
            gamepad_data['axis'] = list(gamepad_data['axis'])
            gamepad_data['axis'][self.AXIS_FORWARD] = -self.MOVE_SPEED if error > 0 else self.MOVE_SPEED
            self.forklift_gamepad.write(gamepad_data)
            time.sleep(self.SLEEP_INTERVAL)

    def move_backward(self, distance):
        move_distance = 0.0
        initial_pos = (self.get_position()['linear']['x'], self.get_position()['linear']['y'])
        while True:
            current_pos =  (self.get_position()['linear']['x'], self.get_position()['linear']['y'])
            move_distance = ((current_pos[0] - initial_pos[0]) ** 2 + (current_pos[1] - initial_pos[1]) ** 2) ** 0.5
            print(f"[INFO] Current position: {current_pos}, Target distance: {distance}")
            error = distance - move_distance
            if abs(error) < 0.01:
                self.stop()
                print(f"[INFO] Reached target distance: {distance}")
                break
            # Move the forklift forward
            gamepad_data = self.forklift_gamepad.read()
            gamepad_data['axis'] = list(gamepad_data['axis'])
            gamepad_data['axis'][self.AXIS_FORWARD] = self.MOVE_SPEED if error > 0 else -self.MOVE_SPEED
            self.forklift_gamepad.write(gamepad_data)
            time.sleep(self.SLEEP_INTERVAL)

    def move(self, distance):
        if distance >= 0:
            self.move_forward(distance)
        else:
            self.move_backward(-distance)

    def turn(self, relative_degree):
        # Convert degrees to radians for the gamepad axis
        yaw_degree = self.get_yaw_degree()
        self._turn_to_yaw_degree(yaw_degree + relative_degree)

    def get_position(self):
        pos = self.forklift_pos.read()
        if pos is None:
            print("[ERROR] Failed to read forklift position data.")
            return None
        return pos
    
    def get_height(self):
        height = self.forklift_height.read()
        if height is None:
            print("[ERROR] Failed to read forklift height data.")
            return None
        return height['data']
    