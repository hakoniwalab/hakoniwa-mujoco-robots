import sys
import os
import time
import hakopy
from hakoniwa_pdu.pdu_manager import PduManager
from hakoniwa_pdu.impl.shm_communication_service import ShmCommunicationService

from python.api.forklift_api import ForkliftAPI
from python.api.monitor_camera import MonitorCameraManager


def forklift_test1(forklift):
    # move forward
    forklift.lift_move(-0.05)
    time.sleep(2)
    print("[INFO] Forklift moving forward 1m.")
    forklift.move(1.0)
    time.sleep(2)
    print("[INFO] Forklift lifting up 0.3m.")
    forklift.lift_move(0.3)
    time.sleep(2)
    print("[INFO] Forklift move back 1m.")
    forklift.move(-1.0)
    time.sleep(5)
    print("[INFO] Forklift turn -90deg.")
    forklift.set_yaw_degree(-90)

    print("[INFO] Forklift move forward 1m.")
    forklift.move(1.0)
    time.sleep(2)
    print("[INFO] Forklift lift down.")
    forklift.lift_move(-0.05)
    time.sleep(2)
    print("[INFO] Forklift move back 1m.")
    forklift.move(-1.0)
    time.sleep(2)
    print("[INFO] Forklift turn 90deg.")
    forklift.set_yaw_degree(0)
    time.sleep(2)
    # info position
    pos = forklift.get_position()
    print(f"[INFO] Forklift position: {pos}")
    # info yaw
    yaw = forklift.get_yaw_degree()
    print(f"[INFO] Forklift yaw degree: {yaw}")
    # info height
    height = forklift.get_height()
    print(f"[INFO] Forklift height: {height}")

def forklift_turn_test(forklift, yaw_degree):
    # info position
    pos = forklift.get_position()
    print(f"[INFO] Forklift position: {pos}")
    # info yaw
    yaw = forklift.get_yaw_degree()
    print(f"[INFO] Forklift yaw degree: {yaw}")

    forklift.set_yaw_degree(yaw_degree)

    # info position
    pos = forklift.get_position()
    print(f"[INFO] Forklift position: {pos}")
    # info yaw
    yaw = forklift.get_yaw_degree()
    print(f"[INFO] Forklift yaw degree: {yaw}")

SAVE_DIR_PATH="./images"
def saveMonitorCameraImage(monitor: MonitorCameraManager, log_id:str):
    global SAVE_DIR_PATH
    for camera_name in monitor.get_camera_names():
        try:
            png_image = monitor.get_image(camera_name)
            if png_image:
                with open(f"{SAVE_DIR_PATH}/SIM_{camera_name}_{log_id}.png", "wb") as f:
                    f.write(png_image)
        except Exception as e:
            pass

class ForkliftController:
    def __init__(self, forklift, monitor_camera_manager=None):
        self.monitor_camera_manager = monitor_camera_manager
        self.forklift = forklift
        self.mission_id = 0
    
    def rotate_to_pickup_pos(self):
        self.forklift.set_yaw_degree(-180)

    def rotate_to_shelf_pos(self):
        self.forklift.set_yaw_degree(-270)

    def grab_cargo(self):
        print("[INFO] Forklift lifting up 0.2m.")
        self.forklift.lift_move(0.2)

    def release_cargo(self):
        print("[INFO] Forklift lifting down 0.05m.")
        self.forklift.lift_move(0.05)

    def lift_down(self):
        print("[INFO] Forklift lifting down.")
        self.forklift.lift_move(-0.05)
        time.sleep(1)
    
    def approach_target(self, distance):
        print(f"[INFO] Forklift move forward {distance} m.")
        self.forklift.move(distance)
        time.sleep(2)

    def retreat(self, distance):
        print(f"[INFO] Forklift move back {distance} m.")
        self.forklift.move(-distance)
        time.sleep(2)

    def rotate_to(self, yaw_degree):
        print(f"[INFO] Forklift turning to {yaw_degree} deg.")
        self.forklift.set_yaw_degree(yaw_degree)


    def pickup_sequence(self, approach_dist, back_dist):
        self.lift_down()
        saveMonitorCameraImage(self.monitor_camera_manager, f"{self.mission_id}_pickup_liftdown")
        self.approach_target(approach_dist)
        saveMonitorCameraImage(self.monitor_camera_manager, f"{self.mission_id}_pickup_approach")
        self.grab_cargo()
        saveMonitorCameraImage(self.monitor_camera_manager, f"{self.mission_id}_pickup_grab")
        self.retreat(back_dist)
        saveMonitorCameraImage(self.monitor_camera_manager, f"{self.mission_id}_pickup_retreat")

    def dropoff_sequence(self, approach_dist, back_dist):
        self.approach_target(approach_dist)
        saveMonitorCameraImage(self.monitor_camera_manager, f"{self.mission_id}_drop_approach")
        self.release_cargo()
        saveMonitorCameraImage(self.monitor_camera_manager, f"{self.mission_id}_drop_release")
        self.retreat(back_dist)
        saveMonitorCameraImage(self.monitor_camera_manager, f"{self.mission_id}_drop_retreat")

    def mission(self, pickup_x, dropoff_x, dropoff_y, next_pickup_y):
        print(f"[INFO] Starting pickup sequence.")
        self.rotate_to_pickup_pos()

        print(f"[INFO] Moving to pickup point.")
        self.pickup_sequence(pickup_x, dropoff_x)

        self.rotate_to_shelf_pos()

        print(f"[INFO] Moving to dropoff point.")
        self.dropoff_sequence(dropoff_y, next_pickup_y)

        self.mission_id += 1


    def full_mission(self):
        print(f"[INFO] Starting 1st mission")
        self.mission(
            pickup_x=1.65,
            dropoff_x=1.0,
            dropoff_y=1.1,
            next_pickup_y=1.95
        )
        print(f"[INFO] Starting 2nd mission")
        self.mission(
            pickup_x=1.1,
            dropoff_x=2.1,
            dropoff_y=2.2,
            next_pickup_y=1.0
        )

def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <config_path> <monitor_config_path>")
        return 1

    config_path = sys.argv[1]
    if not os.path.exists(config_path):
        print(f"[ERROR] Config file not found at '{config_path}'")
        return 1

    monitor_config_path = sys.argv[2]
    if not os.path.exists(monitor_config_path):
        print(f"[ERROR] Monitor config file not found at '{monitor_config_path}'")
        return 1

    pdu_manager = PduManager()
    pdu_manager.initialize(config_path=config_path, comm_service=ShmCommunicationService())
    pdu_manager.start_service_nowait()
    ret = hakopy.init_for_external()
    if ret == False:
        print(f"ERROR: init_for_external() returns {ret}.")
        return False

    monitor = MonitorCameraManager(pdu_manager, monitor_config_path)
    print(f"[INFO] Monitor camera config loaded from '{monitor_config_path}'")

    forklift = ForkliftAPI(pdu_manager)
    print("[INFO] Forklift API test started.")
    controller = ForkliftController(forklift, monitor)
    controller.full_mission()
    print("[INFO] Forklift API test completed.")

    return 0

if __name__ == "__main__":
    sys.exit(main())
