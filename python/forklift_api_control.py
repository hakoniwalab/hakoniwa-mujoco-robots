import sys
import os
import time
import hakopy
import hako_pdu
from api.forklift_api import ForkliftAPI

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


class ForkliftController:
    def __init__(self, forklift):
        self.forklift = forklift
    
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
        self.approach_target(approach_dist)
        self.grab_cargo()
        self.retreat(back_dist)

    def dropoff_sequence(self, approach_dist, back_dist):
        self.approach_target(approach_dist)
        self.release_cargo()
        self.retreat(back_dist)

    def mission(self, pickup_x, dropoff_x, dropoff_y, next_pickup_y):
        print(f"[INFO] Starting pickup sequence.")
        self.rotate_to_pickup_pos()
        self.pickup_sequence(pickup_x, dropoff_x)
        self.rotate_to_shelf_pos()
        print(f"[INFO] Moving to dropoff point.")
        self.dropoff_sequence(dropoff_y, next_pickup_y)


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
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <config_path>")
        return 1

    config_path = sys.argv[1]
    if not os.path.exists(config_path):
        print(f"[ERROR] Config file not found at '{config_path}'")
        return 1

    if not hakopy.init_for_external():
        raise RuntimeError("ERROR: hakopy.init_for_external() failed.")

    hako_binary_path = os.getenv('HAKO_BINARY_PATH', '/usr/local/lib/hakoniwa/hako_binary/offset')
    pdu_manager = hako_pdu.HakoPduManager(hako_binary_path, config_path)

    forklift = ForkliftAPI(pdu_manager)
    print("[INFO] Forklift API test started.")
    controller = ForkliftController(forklift)
    controller.full_mission()
    print("[INFO] Forklift API test completed.")

    return 0

if __name__ == "__main__":
    sys.exit(main())
