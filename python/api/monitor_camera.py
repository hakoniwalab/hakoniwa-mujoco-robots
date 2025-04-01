import hakopy
import hako_pdu
import os
import time
from camera.api.monitor_camera_config import MonitorCameraConfig

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
    def __init__(self, camera_config_path: str, monitor_camera_pattern = "Monitor_*"):
        self.monitor_camera_config = MonitorCameraConfig(camera_config_path, monitor_camera_pattern)
        hako_binary_path = os.getenv('HAKO_BINARY_PATH', '/usr/local/lib/hakoniwa/hako_binary/offset')
        self.pdu_manager = hako_pdu.HakoPduManager(hako_binary_path, self.monitor_camera_config.data["pdu_path"])

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
        command, pdu_cmd = self._get_packet(camera.cmd_pdu_channel_id, camera.name)
        pdu_cmd['request_id'] = camera.cmd_request_id
        pdu_cmd['encode_type'] = 0
        command.write()
        img = self._get_camera_data(camera, timeout)
        pdu_cmd['header']['request'] = 0
        pdu_cmd['header']['result'] = 0
        command.write()
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
            command = self.pdu_manager.get_pdu(camera.name, camera.image_pdu_channel_id)
            pdu_data = command.read()
            if pdu_data['request_id'] == camera.cmd_request_id:
                return pdu_data['image']['data']
        print(f"WARNING: Timeout while waiting for camera data: {camera.name}")
        return None  # タイムアウト時は None を返す

    def _get_packet(self, channel, camera_name):
        command = self.pdu_manager.get_pdu(camera_name, channel)
        cmd = command.get()
        cmd['header']['request'] = 1
        cmd['header']['result'] = 0
        cmd['header']['result_code'] = 0
        return command, cmd

