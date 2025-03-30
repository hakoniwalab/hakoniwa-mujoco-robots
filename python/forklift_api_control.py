import sys
import os
import hakopy
import hako_pdu
from api.forklift_api import ForkliftAPI

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
    print("[INFO] Forklift API initialized.")


    forklift.lift_move(-0.05)
    forklift.move(1.0)
    forklift.lift_move(0.3)
    forklift.move(-1.0)
    forklift.turn(-80)
    forklift.move_forward(1.0)
    forklift.lift_move(-0.05)
    forklift.move(-1.0)
    forklift.turn(80)
    # info position
    pos = forklift.get_position()
    print(f"[INFO] Forklift position: {pos}")
    # info yaw
    yaw = forklift.get_yaw_degree()
    print(f"[INFO] Forklift yaw degree: {yaw}")
    # info height
    height = forklift.get_height()
    print(f"[INFO] Forklift height: {height}")

    return 0

if __name__ == "__main__":
    sys.exit(main())
