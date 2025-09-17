import os
import time
from hakoniwa_pdu.pdu_manager import PduManager
from camera.api.monitor_camera_config import MonitorCameraConfig
from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_pytype_MonitorCameraCmd import MonitorCameraCmd
from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_pytype_MonitorCameraData import MonitorCameraData
from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_conv_MonitorCameraCmd import pdu_to_py_MonitorCameraCmd, py_to_pdu_MonitorCameraCmd
from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_conv_MonitorCameraData import pdu_to_py_MonitorCameraData, py_to_pdu_MonitorCameraData

class MonitorCamera:
    def __init__(self, entry):
        self.name = entry.get('name', 'Unknown')
        self.cmd_request_id = 1
        self.cmd_pdu_name, self.cmd_pdu_channel_id, self.cmd_pdu_size = self._find_pdu_entry(entry, 'hako_msgs/MonitorCameraCmd')
        self.image_pdu_name, self.image_pdu_channel_id, self.image_pdu_size = self._find_pdu_entry(entry, 'hako_msgs/MonitorCameraData')

    def _find_pdu_entry(self, entry, target_type):
        """ shm_pdu_readers と shm_pdu_writers の両方を確認し、該当するPDUを探す """
        for key in ['shm_pdu_readers', 'shm_pdu_writers']:
            for e in entry.get(key, []):
                if e.get('type') == target_type:
                    return e.get('org_name'), e.get('channel_id'), e.get('pdu_size')
        return None, None, None  # 該当なしの場合

class MonitorCameraManager:
    def __init__(self, pdu_manager: PduManager, camera_config_path: str, monitor_camera_pattern = "Monitor_*"):
        self.monitor_camera_config = MonitorCameraConfig(camera_config_path, monitor_camera_pattern)
        self.pdu_manager = pdu_manager

        self.cameras = []
        for entry in self.monitor_camera_config.get_monitor_cameras():
            camera = MonitorCamera(entry)
            self.cameras.append(camera)

        #ret = hakopy.init_for_external()
        #if ret == False:
        #    print(f"ERROR: MonitorCamera.init_for_external() returns {ret}.")
        #    return False

    def get_camera(self, camera_name: str) -> MonitorCamera:
        for camera in self.cameras:
            if camera.name == camera_name:
                return camera
        return None
    def get_camera_names(self):
        return [camera.name for camera in self.cameras]

    def get_image(self, camera_name, timeout=5.0):
        camera = self.get_camera(camera_name)
        pdu_cmd = self._get_packet()
        pdu_cmd.request_id = camera.cmd_request_id
        pdu_cmd.encode_type = 0
        self.pdu_manager.flush_pdu_raw_data_nowait(camera.name, camera.cmd_pdu_name, py_to_pdu_MonitorCameraCmd(pdu_cmd))
        img = self._get_camera_data(camera, timeout)
        pdu_cmd.header.request = 0
        pdu_cmd.header.result = 0
        self.pdu_manager.flush_pdu_raw_data_nowait(camera.name, camera.cmd_pdu_name, py_to_pdu_MonitorCameraCmd(pdu_cmd))
        camera.cmd_request_id = camera.cmd_request_id + 1
        if img is None:
            return None
        else:
            return bytes(img)

    def _get_camera_data(self, camera: MonitorCamera, timeout=5.0):
        """ カメラデータを取得（タイムアウト付き）"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            #print(f"INFO: Waiting for camera data: {camera.name} channel_id: {camera.image_pdu_channel_id}")
            raw = self.pdu_manager.read_pdu_raw_data(camera.name, camera.image_pdu_name)
            pdu_data = pdu_to_py_MonitorCameraData(raw)
            if pdu_data.request_id == camera.cmd_request_id:
                return pdu_data.image.data
        print(f"WARNING: Timeout while waiting for camera data: {camera.name}")
        return None  # タイムアウト時は None を返す

    def _get_packet(self) -> MonitorCameraCmd:
        cmd = MonitorCameraCmd()
        cmd.header.request = 1
        cmd.header.result = 0
        cmd.header.result_code = 0
        return cmd

