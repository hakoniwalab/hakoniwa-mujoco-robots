import sys
import json
import os
import fnmatch
from typing import Dict, Any, List

class MonitorCameraConfig:
    def __init__(self, config_path: str, monitor_camera_pattern="Monitor_*"):
        self.config_path = config_path
        self.data = self.load_from_json(self.config_path)
        self.validate()

        # pdu_pathの存在確認
        if not os.path.exists(self.data["pdu_path"]):
            print(f"WARNING: pdu_path is not found: {self.data['pdu_path']}")
            self.pdu_def = None
        else:
            self.pdu_def = self.load_from_json(self.data["pdu_path"])
            print(f"INFO: PDU definition loaded from {self.data['pdu_path']}")

        self.monitor_camera_pattern = monitor_camera_pattern

    def get_monitor_cameras(self) -> List[Dict[str, Any]]:
        """ 監視カメラ情報を取得 """
        if not self.pdu_def or "robots" not in self.pdu_def:
            print("WARNING: PDU definition is not available.")
            return []

        entries = [
            entry for entry in self.pdu_def["robots"]
            if fnmatch.fnmatch(entry["name"], self.monitor_camera_pattern)
        ]
        return entries

    def load_from_json(self, path: str) -> Dict[str, Any]:
        """ JSONファイルを読み込む """
        try:
            with open(path, "r", encoding="utf-8") as file:
                return json.load(file)
        except Exception as e:
            raise RuntimeError(f"JSONファイルの読み込みに失敗: {e}")

    def validate(self):
        """ JSONの構造が正しいかバリデーション """
        required_keys = ["pdu_path", "monitor_cameras"]
        for key in required_keys:
            if key not in self.data:
                raise ValueError(f"JSONに '{key}' キーが存在しません")

        for camera in self.data["monitor_cameras"]:
            required_camera_keys = [
                "pdu_info", "coordinate_system", "fov", "resolution", "camera_type", "trigger_type"
            ]
            if "pdu_info" not in camera or "robot_name" not in camera["pdu_info"]:
                raise ValueError("カメラ設定に 'pdu_info' または 'robot_name' が不足しています")
            for key in required_camera_keys:
                if key not in camera:
                    raise ValueError(f"カメラ設定に '{key}' が不足しています")

    def get_monitor_camera_settings(self) -> List[Dict[str, Any]]:
        """ 監視カメラ設定情報を取得 """
        return self.data["monitor_cameras"]

# 使用例
if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <config_path>")
        sys.exit(1)

    config = MonitorCameraConfig(sys.argv[1])
    camera_settings = config.get_monitor_camera_settings()
    print(json.dumps(camera_settings, indent=2, ensure_ascii=False))
