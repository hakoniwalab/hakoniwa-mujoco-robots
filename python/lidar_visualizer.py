#!/usr/bin/env python3

import atexit
import os
import signal
import struct
import sys
import time
from pathlib import Path

import hakopy
import numpy as np
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Circle

from hakoniwa_pdu.impl.shm_communication_service import ShmCommunicationService
from hakoniwa_pdu.pdu_manager import PduManager
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Twist import pdu_to_py_Twist
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_LaserScan import pdu_to_py_LaserScan


DEFAULT_CONFIG = "config/tb3-pdudef-compact.json"
DEFAULT_ROBOT = "TB3"
DEFAULT_SCAN_PDU = "laser_scan"
DEFAULT_POSE_PDU = "base_scan_pos"
REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_POINT_SIZE = 4.0
DEFAULT_LOCAL_VIEW_SIZE = 1.0
DEFAULT_FOLLOW_GAIN = 0.08


def _to_float_list(values):
    if isinstance(values, (bytes, bytearray)):
        if len(values) % 4 != 0:
            return []
        return list(struct.unpack("<" + "f" * (len(values) // 4), values))
    return list(values)


def _get_env_float(name: str, default_value: float) -> float:
    value = os.getenv(name)
    if value is None or value == "":
        return default_value
    try:
        return float(value)
    except ValueError:
        return default_value


class LiDARWindow(QMainWindow):
    def __init__(self, pdu_manager: PduManager, robot_name: str, scan_pdu_name: str, pose_pdu_name: str):
        super().__init__()
        self.pdu_manager = pdu_manager
        self.robot_name = robot_name
        self.scan_pdu_name = scan_pdu_name
        self.pose_pdu_name = pose_pdu_name

        self.last_hit_x = np.array([], dtype=np.float32)
        self.last_hit_y = np.array([], dtype=np.float32)
        self.last_hit_time = 0.0
        self.last_sensor_x = 0.0
        self.last_sensor_y = 0.0
        self.last_sensor_yaw = 0.0
        self.view_center_x = _get_env_float("LIDAR_VIEW_CENTER_X", 0.0)
        self.view_center_y = _get_env_float("LIDAR_VIEW_CENTER_Y", 0.0)
        self.follow_gain = _get_env_float("LIDAR_FOLLOW_GAIN", DEFAULT_FOLLOW_GAIN)

        self._closing = False
        self._shutdown_done = False
        self.fixed_plot_range = _get_env_float("LIDAR_PLOT_RANGE", 0.0)
        self.min_plot_range = _get_env_float("LIDAR_MIN_PLOT_RANGE", 1.0)
        self.point_size = _get_env_float("LIDAR_POINT_SIZE", DEFAULT_POINT_SIZE)
        self.local_view_size = _get_env_float("LIDAR_LOCAL_VIEW_SIZE", DEFAULT_LOCAL_VIEW_SIZE)

        self.setWindowTitle(f"{robot_name} LiDAR")
        self.setGeometry(50, 50, 800, 800)

        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)

        central_widget = QWidget()
        layout = QVBoxLayout(central_widget)
        layout.addWidget(self.canvas)
        self.setCentralWidget(central_widget)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)

    def shutdown(self):
        if self._shutdown_done:
            return
        self._shutdown_done = True
        self._closing = True
        try:
            self.timer.stop()
        except Exception:
            pass
        try:
            if self.pdu_manager is not None:
                self.pdu_manager.stop_service_nowait()
        except Exception:
            pass
        self.pdu_manager = None

    def closeEvent(self, event):
        self.shutdown()
        event.accept()

    def _read_pose(self):
        raw_pose = self.pdu_manager.read_pdu_raw_data(self.robot_name, self.pose_pdu_name)
        if raw_pose is None:
            return None
        try:
            pose = pdu_to_py_Twist(raw_pose)
        except Exception:
            return None
        return float(pose.linear.x), float(pose.linear.y), float(pose.angular.z)

    def _update_view_center(self, sensor_x: float, sensor_y: float):
        gain = min(max(self.follow_gain, 0.0), 1.0)
        self.view_center_x += (sensor_x - self.view_center_x) * gain
        self.view_center_y += (sensor_y - self.view_center_y) * gain

    def update_plot(self):
        if self._closing or self.pdu_manager is None:
            return

        try:
            self.pdu_manager.run_nowait()
            raw_scan = self.pdu_manager.read_pdu_raw_data(self.robot_name, self.scan_pdu_name)
        except Exception:
            return

        if raw_scan is None:
            return

        try:
            scan = pdu_to_py_LaserScan(raw_scan)
        except Exception:
            return

        ranges = np.array(_to_float_list(scan.ranges), dtype=np.float32)
        if ranges.size == 0:
            return

        pose = self._read_pose()
        if pose is not None:
            sensor_x, sensor_y, sensor_yaw = pose
            self.last_sensor_x = sensor_x
            self.last_sensor_y = sensor_y
            self.last_sensor_yaw = sensor_yaw
        else:
            sensor_x = self.last_sensor_x
            sensor_y = self.last_sensor_y
            sensor_yaw = self.last_sensor_yaw

        self._update_view_center(sensor_x, sensor_y)

        angles = scan.angle_min + np.arange(ranges.size, dtype=np.float32) * scan.angle_increment
        valid = (
            np.isfinite(ranges)
            & (ranges >= scan.range_min)
            & (ranges < (scan.range_max - 1.0e-2))
        )

        local_x = ranges[valid] * np.cos(angles[valid])
        local_y = -ranges[valid] * np.sin(angles[valid])
        valid_ranges = ranges[valid]

        cos_yaw = np.cos(sensor_yaw)
        sin_yaw = np.sin(sensor_yaw)
        world_x = sensor_x + local_x * cos_yaw - local_y * sin_yaw
        world_y = sensor_y + local_x * sin_yaw + local_y * cos_yaw

        half_view = self.local_view_size * 0.5
        view_window = (
            (np.abs(world_x - self.view_center_x) <= half_view)
            & (np.abs(world_y - self.view_center_y) <= half_view)
        )
        world_x = world_x[view_window]
        world_y = world_y[view_window]
        valid_ranges = valid_ranges[view_window]

        if world_x.size > 0:
            self.last_hit_x = world_x
            self.last_hit_y = world_y
            self.last_hit_time = time.time()

        self.ax.cla()
        self.ax.set_xlim(self.view_center_x - half_view, self.view_center_x + half_view)
        self.ax.set_ylim(self.view_center_y - half_view, self.view_center_y + half_view)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_title(f"{self.robot_name} LiDAR (World View)")
        self.ax.set_xlabel("World X [m]")
        self.ax.set_ylabel("World Y [m]")
        self.ax.grid(True)

        self.ax.scatter([sensor_x], [sensor_y], s=50, marker="x")
        arrow_len = max(0.08, half_view * 0.18)
        self.ax.arrow(
            sensor_x,
            sensor_y,
            arrow_len * np.cos(sensor_yaw),
            arrow_len * np.sin(sensor_yaw),
            head_width=max(0.015, half_view * 0.04),
            head_length=max(0.025, half_view * 0.06),
            length_includes_head=True,
        )

        if self.local_view_size >= 1.0:
            self.ax.add_patch(
                Circle((self.view_center_x, self.view_center_y), 1.0, fill=False, linestyle="--", linewidth=0.8, alpha=0.5)
            )

        if world_x.size > 0:
            color_values = np.clip(
                (valid_ranges - scan.range_min) / max(scan.range_max - scan.range_min, 1.0e-6),
                0.0,
                1.0,
            )
            self.ax.scatter(world_x, world_y, s=self.point_size, c=color_values, cmap="turbo", vmin=0.0, vmax=1.0)
            nearest = float(np.min(valid_ranges)) if np.any(valid) else float("nan")
            status = f"valid hits: {world_x.size}/{ranges.size}  nearest: {nearest:.3f} m"
        elif (time.time() - self.last_hit_time) < 1.0 and self.last_hit_x.size > 0:
            self.ax.scatter(self.last_hit_x, self.last_hit_y, s=self.point_size, alpha=0.6)
            status = f"valid hits: 0/{ranges.size}  (showing last hit cloud)"
        else:
            status = f"valid hits: 0/{ranges.size}"

        self.ax.text(
            0.02,
            0.98,
            status + f"\nrobot: ({sensor_x:.2f}, {sensor_y:.2f})  view center: ({self.view_center_x:.2f}, {self.view_center_y:.2f})  window: {self.local_view_size:.2f} m square",
            transform=self.ax.transAxes,
            va="top",
            ha="left",
            fontsize=10,
            bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none"},
        )

        try:
            self.canvas.draw_idle()
        except RuntimeError:
            pass


def main() -> int:
    config_path = sys.argv[1] if len(sys.argv) >= 2 else os.getenv("HAKO_CONFIG_PATH", DEFAULT_CONFIG)
    robot_name = sys.argv[2] if len(sys.argv) >= 3 else os.getenv("HAKO_LIDAR_ROBOT_NAME", DEFAULT_ROBOT)
    scan_pdu_name = sys.argv[3] if len(sys.argv) >= 4 else os.getenv("HAKO_LIDAR_SCAN_PDU_NAME", DEFAULT_SCAN_PDU)
    pose_pdu_name = sys.argv[4] if len(sys.argv) >= 5 else os.getenv("HAKO_LIDAR_POSE_PDU_NAME", DEFAULT_POSE_PDU)
    config_path = str((REPO_ROOT / config_path).resolve()) if not os.path.isabs(config_path) else config_path

    if not os.path.exists(config_path):
        print(f"[ERROR] Config file not found at '{config_path}'")
        return 1

    pdu_manager = PduManager()
    pdu_manager.initialize(config_path=config_path, comm_service=ShmCommunicationService())
    pdu_manager.start_service_nowait()

    if not hakopy.init_for_external():
        print("[ERROR] hakopy.init_for_external() failed")
        try:
            pdu_manager.stop_service_nowait()
        except Exception:
            pass
        return 1

    app = QApplication(sys.argv)
    window = LiDARWindow(pdu_manager, robot_name, scan_pdu_name, pose_pdu_name)
    window.show()

    def _shutdown(*_args):
        try:
            window.shutdown()
        finally:
            app.quit()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)
    atexit.register(window.shutdown)

    try:
        return app.exec_()
    finally:
        window.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
