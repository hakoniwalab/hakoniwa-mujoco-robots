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

def forklift_demo(forklift):
    # info position
    pos = forklift.get_position()
    print(f"[INFO] Forklift position: {pos}")
    # info yaw
    yaw = forklift.get_yaw_degree()
    print(f"[INFO] Forklift yaw degree: {yaw}")

    forklift.set_yaw_degree(-180)

    # info position
    pos = forklift.get_position()
    print(f"[INFO] Forklift position: {pos}")
    # info yaw
    yaw = forklift.get_yaw_degree()
    print(f"[INFO] Forklift yaw degree: {yaw}")

    move_distance = 1.65
    print(f"[INFO] Forklift move forward {move_distance} m.")
    forklift.move(move_distance)
    forklift.lift_move(-0.05)
    time.sleep(2)

    print("[INFO] Forklift lifting up 0.2m.")
    forklift.lift_move(0.2)
    time.sleep(2)
    move_distance = 1.0
    print("[INFO] Forklift move back  {move_distance} m.")
    forklift.move(-move_distance)
    time.sleep(2)

    forklift.set_yaw_degree(-270)

    move_distance = 1.1
    print(f"[INFO] Forklift move forward {move_distance} m.")
    forklift.move(move_distance)
    time.sleep(1)
    print("[INFO] Forklift lifting down 0.05 m.")
    forklift.lift_move(0.05)
    time.sleep(1)

    move_distance = 1.95
    print("[INFO] Forklift move back  {move_distance} m.")
    forklift.move(-move_distance)
    time.sleep(2)

    forklift.set_yaw_degree(-180)

    move_distance = 1.1
    forklift.lift_move(-0.05)
    print(f"[INFO] Forklift move forward {move_distance} m.")
    forklift.move(move_distance)
    time.sleep(2)

    print("[INFO] Forklift lifting up 0.2m.")
    forklift.lift_move(0.2)
    time.sleep(2)
    move_distance = 2.1
    print("[INFO] Forklift move back  {move_distance} m.")
    forklift.move(-move_distance)
    time.sleep(2)

    forklift.set_yaw_degree(-270)

    move_distance = 1.95
    print(f"[INFO] Forklift move forward {move_distance} m.")
    forklift.move(move_distance)
    time.sleep(1)
    print("[INFO] Forklift lifting down 0.05 m.")
    forklift.lift_move(0.05)
    time.sleep(1)

    move_distance = 1.0
    print("[INFO] Forklift move back  {move_distance} m.")
    forklift.move(-move_distance)
    time.sleep(2)

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
    forklift_demo(forklift)
    print("[INFO] Forklift API test completed.")

    return 0

if __name__ == "__main__":
    sys.exit(main())
