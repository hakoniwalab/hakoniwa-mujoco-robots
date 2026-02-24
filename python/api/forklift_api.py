# api/forklift.py

from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_pytype_GameControllerOperation import GameControllerOperation
from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_conv_GameControllerOperation import pdu_to_py_GameControllerOperation, py_to_pdu_GameControllerOperation
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Twist import pdu_to_py_Twist
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_pytype_Twist import Twist
from hakoniwa_pdu.pdu_msgs.std_msgs.pdu_pytype_Float64 import Float64
from hakoniwa_pdu.pdu_msgs.std_msgs.pdu_conv_Float64 import pdu_to_py_Float64
from hakoniwa_pdu.pdu_msgs.std_msgs.pdu_conv_Int32 import pdu_to_py_Int32
import os
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
        self.move_speed = self.MOVE_SPEED
        self._sleep_fn = time.sleep
        self.write_log_enabled = os.getenv("HAKO_PDU_WRITE_LOG", "0") in ("1", "true", "TRUE", "on", "ON")
        self.write_seq = 0

    def set_sleep_func(self, sleep_fn):
        # sleep_fn: Callable[[float], None]
        self._sleep_fn = sleep_fn if sleep_fn is not None else time.sleep

    def _sleep(self, sec: float):
        self._sleep_fn(max(0.0, float(sec)))

    def set_move_speed(self, speed: float):
        # Gamepad axis valid range is [-1.0, 1.0], so clamp to (0, 1].
        value = abs(float(speed))
        if value <= 0.0:
            value = 0.1
        if value > 1.0:
            value = 1.0
        self.move_speed = value

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
            
    def _log_pdu_write(self, data: GameControllerOperation, reason: str):
        if not self.write_log_enabled:
            return
        self.write_seq += 1
        axis = list(data.axis)
        ts = time.time()
        print(
            f"[PDU-WRITE] ts={ts:.6f} seq={self.write_seq} reason={reason} "
            f"axis[yaw]={axis[self.AXIS_YAW]:.3f} axis[lift]={axis[self.AXIS_LIFT]:.3f} "
            f"axis[fwd]={axis[self.AXIS_FORWARD]:.3f}"
        )

    def put_gamepad(self, data: GameControllerOperation, reason: str = "unknown"):
        self._log_pdu_write(data, reason)
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

    def get_phase(self) -> int:
        self.pdu_manager.run_nowait()
        raw_data = self.pdu_manager.read_pdu_raw_data(self.robot_name, "phase")
        try:
            return int(pdu_to_py_Int32(raw_data).data)
        except Exception:
            return 0

    def get_phase_status(self):
        self.pdu_manager.run_nowait()
        raw_data = self.pdu_manager.read_pdu_raw_data(self.robot_name, "phase")
        try:
            return True, int(pdu_to_py_Int32(raw_data).data)
        except Exception:
            return False, 0

    def get_yaw_degree(self):
        # Convert radians to degrees for the gamepad axis
        pos: Twist = self.get_position()
        #print(f"[INFO] Forklift position data: {pos}")
        yaw_rad = pos.angular.z
        yaw_degree = yaw_rad * (180.0 / 3.14159)
        return yaw_degree
    
    def stop(self, reason: str = "stop"):
        gamepad_data: GameControllerOperation = self.get_gamepad()
        gamepad_data.axis = list(gamepad_data.axis)
        gamepad_data.axis[self.AXIS_YAW] = 0.0
        gamepad_data.axis[self.AXIS_LIFT] = 0.0
        gamepad_data.axis[self.AXIS_FORWARD] = 0.0
        gamepad_data.button = list(gamepad_data.button)
        gamepad_data.button[self.BUTTON_ESTOP] = False
        self.put_gamepad(gamepad_data, reason=reason)
        self._sleep(self.SLEEP_INTERVAL)

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
                self.stop(reason="turn_done")
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
            self.put_gamepad(gamepad_data, reason="turn_pid")

            self._sleep(dt)

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
            self._sleep(0.5)

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
                self.stop(reason="lift_done")
                #print(f"[INFO] Reached target height: {current_height:.2f}")
                break
            # Move the lift
            gamepad_data = self.get_gamepad()
            gamepad_data.axis = list(gamepad_data.axis)
            gamepad_data.axis[self.AXIS_LIFT] = -1.0 if error > 0 else 1.0
            self.put_gamepad(gamepad_data, reason="lift_move")
            self._sleep(self.SLEEP_INTERVAL)

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
                self.stop(reason="move_forward_done")
                #print(f"[INFO] Reached target distance: {distance}")
                break
            # Move the forklift forward
            gamepad_data = self.get_gamepad()
            gamepad_data.axis = list(gamepad_data.axis)
            gamepad_data.axis[self.AXIS_FORWARD] = -self.move_speed if error > 0 else self.move_speed
            self.put_gamepad(gamepad_data, reason="move_forward")
            self._sleep(self.SLEEP_INTERVAL)

    def move_backward(self, distance):
        move_distance = 0.0
        initial_pos = (self.get_position().linear.x, self.get_position().linear.y)
        while True:
            current_pos =  (self.get_position().linear.x, self.get_position().linear.y)
            move_distance = ((current_pos[0] - initial_pos[0]) ** 2 + (current_pos[1] - initial_pos[1]) ** 2) ** 0.5
            #print(f"[INFO] Current position: {current_pos}, Target distance: {distance}")
            error = distance - move_distance
            if abs(error) < 0.01:
                self.stop(reason="move_backward_done")
                #print(f"[INFO] Reached target distance: {distance}")
                break
            # Move the forklift forward
            gamepad_data = self.get_gamepad()
            gamepad_data.axis = list(gamepad_data.axis)
            gamepad_data.axis[self.AXIS_FORWARD] = self.move_speed if error > 0 else -self.move_speed
            self.put_gamepad(gamepad_data, reason="move_backward")
            self._sleep(self.SLEEP_INTERVAL)

    def move(self, distance):
        if distance >= 0:
            self.move_forward(distance)
        else:
            self.move_backward(-distance)

    def relative_turn(self, relative_degree):
        # Convert degrees to radians for the gamepad axis
        yaw_degree = self.get_yaw_degree()
        self._turn_to_yaw_degree(yaw_degree + relative_degree)


    
