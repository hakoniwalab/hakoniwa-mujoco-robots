# api/forklift.py

from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_pytype_GameControllerOperation import GameControllerOperation
from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_conv_GameControllerOperation import pdu_to_py_GameControllerOperation, py_to_pdu_GameControllerOperation
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Twist import pdu_to_py_Twist
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_pytype_Twist import Twist
from hakoniwa_pdu.pdu_msgs.std_msgs.pdu_pytype_Float64 import Float64
from hakoniwa_pdu.pdu_msgs.std_msgs.pdu_conv_Float64 import pdu_to_py_Float64
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
        self.robot_name = robot_name
        self.pdu_manager = pdu_manager

    def get_gamepad(self) -> GameControllerOperation:
        self.pdu_manager.run_nowait()  # Ensure PDU data is updated
        raw_data = self.pdu_manager.read_pdu_raw_data(self.robot_name, "hako_cmd_game")
        try:
            return pdu_to_py_GameControllerOperation(raw_data)
        except Exception as e:
            value = GameControllerOperation()
            value.axis = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            value.button = [False] * 16
            return value  # Return empty data on error
            
    def put_gamepad(self, data: GameControllerOperation):
        return self.pdu_manager.flush_pdu_raw_data_nowait(self.robot_name, "hako_cmd_game", py_to_pdu_GameControllerOperation(data))

    def get_position(self) -> Twist:
        self.pdu_manager.run_nowait()  # Ensure PDU data is updated
        raw_data = self.pdu_manager.read_pdu_raw_data(self.robot_name, "pos")
        try :
            return pdu_to_py_Twist(raw_data)
        except Exception as e:
            return Twist()  # Return empty data on error

    def get_height(self) -> float:
        self.pdu_manager.run_nowait()  # Ensure PDU data is updated        
        raw_data = self.pdu_manager.read_pdu_raw_data(self.robot_name, "height")
        try:
            return pdu_to_py_Float64(raw_data).data
        except Exception as e:
            return 0  # Return empty data on error

    def get_yaw_degree(self):
        # Convert radians to degrees for the gamepad axis
        pos: Twist = self.get_position()
        #print(f"[INFO] Forklift position data: {pos}")
        yaw_rad = pos.angular.z
        yaw_degree = yaw_rad * (180.0 / 3.14159)
        return yaw_degree
    
    def stop(self):
        gamepad_data: GameControllerOperation = self.get_gamepad()
        gamepad_data.axis = list(gamepad_data.axis)
        gamepad_data.axis[self.AXIS_YAW] = 0.0
        gamepad_data.axis[self.AXIS_LIFT] = 0.0
        gamepad_data.axis[self.AXIS_FORWARD] = 0.0
        gamepad_data.button = list(gamepad_data.button)
        gamepad_data.button[self.BUTTON_ESTOP] = False
        self.put_gamepad(gamepad_data)
        time.sleep(self.SLEEP_INTERVAL)

    def calc_yaw_degree_error(self, target_yaw_degree):
        current_yaw = self.get_yaw_degree()
        error = target_yaw_degree - current_yaw
        # 誤差を -180～180 に正規化
        error = (error + 180) % 360 - 180
        return error

    def _turn_to_yaw_degree(self, target_yaw_degree):
        target_yaw_degree = self._normalize_angle(target_yaw_degree)

        Kp = 0.5  # 比例ゲイン（P）
        Ki = 0.2  # 積分ゲイン（I）←追加
        Kd = 1.0   # 微分ゲイン（D）

        prev_error = 0
        integral = 0
        dt = 0.001  # 制御周期（秒）

        while True:
            error = self.calc_yaw_degree_error(target_yaw_degree)
            if abs(error) < 0.25:
                self.stop()
                break
            
            #print(f"[DEBUG] Current yaw: {self.get_yaw_degree()}, Target yaw: {target_yaw_degree}, Error: {error}")

            # 積分項の計算
            integral += error * dt

            # 微分項の計算
            derivative = (error - prev_error) / dt
            prev_error = error

            # PID出力
            control = Kp * error + Ki * integral + Kd * derivative

            # 飽和（-1.0～1.0にクランプ）
            control = -max(min(control, 1.0), -1.0)

            # 書き込み
            gamepad_data = self.get_gamepad()
            gamepad_data.axis = list(gamepad_data.axis)
            gamepad_data.axis[self.AXIS_YAW] = control
            #print(f"[DEBUG] Control output: {control}")
            self.put_gamepad(gamepad_data)

            time.sleep(dt)

    def set_yaw_degree(self, target_yaw_degree):
        print(f"[INFO] Forklift turn {target_yaw_degree} [deg].")
        while True:
            error = self.calc_yaw_degree_error(target_yaw_degree)
            print(f"[INFO] Current yaw: {self.get_yaw_degree()}, Target yaw: {target_yaw_degree}")
            print(f"[INFO] Yaw error: {error}")
            if abs(error) <= 0.5:
                print("[INFO] Target angle reached.")
                break
            # 誤差分だけ回転させる
            self.relative_turn(error)
            time.sleep(0.5)

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
            #print(f"[INFO] Current height: {current_height:.2f}, Target height: {target_height:.2f}")
            error = target_height - current_height
            if abs(current_height - target_height) < 0.01:
                self.stop()
                #print(f"[INFO] Reached target height: {current_height:.2f}")
                break
            # Move the lift
            gamepad_data = self.get_gamepad()
            gamepad_data.axis = list(gamepad_data.axis)
            gamepad_data.axis[self.AXIS_LIFT] = -1.0 if error > 0 else 1.0
            self.put_gamepad(gamepad_data)
            time.sleep(self.SLEEP_INTERVAL)

    def move_forward(self, distance):
        move_distance = 0.0
        p = self.get_position()
        initial_pos = (p.linear.x, p.linear.y)
        while True:
            p = self.get_position()
            current_pos =  (p.linear.x, p.linear.y)
            move_distance = ((current_pos[0] - initial_pos[0]) ** 2 + (current_pos[1] - initial_pos[1]) ** 2) ** 0.5
            #print(f"[INFO] Current position: {current_pos}, Target distance: {distance}")
            error = distance - move_distance
            if abs(error) < 0.01:
                self.stop()
                #print(f"[INFO] Reached target distance: {distance}")
                break
            # Move the forklift forward
            gamepad_data = self.get_gamepad()
            gamepad_data.axis = list(gamepad_data.axis)
            gamepad_data.axis[self.AXIS_FORWARD] = -self.MOVE_SPEED if error > 0 else self.MOVE_SPEED
            self.put_gamepad(gamepad_data)
            time.sleep(self.SLEEP_INTERVAL)

    def move_backward(self, distance):
        move_distance = 0.0
        initial_pos = (self.get_position().linear.x, self.get_position().linear.y)
        while True:
            current_pos =  (self.get_position().linear.x, self.get_position().linear.y)
            move_distance = ((current_pos[0] - initial_pos[0]) ** 2 + (current_pos[1] - initial_pos[1]) ** 2) ** 0.5
            #print(f"[INFO] Current position: {current_pos}, Target distance: {distance}")
            error = distance - move_distance
            if abs(error) < 0.01:
                self.stop()
                #print(f"[INFO] Reached target distance: {distance}")
                break
            # Move the forklift forward
            gamepad_data = self.get_gamepad()
            gamepad_data.axis = list(gamepad_data.axis)
            gamepad_data.axis[self.AXIS_FORWARD] = self.MOVE_SPEED if error > 0 else -self.MOVE_SPEED
            self.put_gamepad(gamepad_data)
            time.sleep(self.SLEEP_INTERVAL)

    def move(self, distance):
        if distance >= 0:
            self.move_forward(distance)
        else:
            self.move_backward(-distance)

    def relative_turn(self, relative_degree):
        # Convert degrees to radians for the gamepad axis
        yaw_degree = self.get_yaw_degree()
        self._turn_to_yaw_degree(yaw_degree + relative_degree)


    