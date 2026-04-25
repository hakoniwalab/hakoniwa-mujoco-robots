#!/usr/bin/env python3

import os
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

from hakoniwa_pdu.impl.shm_communication_service import ShmCommunicationService
from hakoniwa_pdu.pdu_manager import PduManager
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_LaserScan import pdu_to_py_LaserScan

DEFAULT_CONFIG = "config/tb3-pdudef-compact.json"
REPO_ROOT = Path(__file__).resolve().parents[1]


def _to_float_list(values):
    if isinstance(values, (bytes, bytearray)):
        if len(values) % 4 != 0:
            return []
        return list(struct.unpack("<" + "f" * (len(values) // 4), values))
    return list(values)


class Tb3LiDARWindow(QMainWindow):
    def __init__(self, pdu_manager: PduManager):
        super().__init__()
        self.pdu_manager = pdu_manager
        self.last_hit_x = np.array([], dtype=np.float32)
        self.last_hit_y = np.array([], dtype=np.float32)
        self.last_hit_time = 0.0
        self.last_range_max = 5.0
        self.setWindowTitle("TB3 2D LiDAR")
        self.setGeometry(50, 50, 600, 600)

        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)

        central_widget = QWidget()
        layout = QVBoxLayout(central_widget)
        layout.addWidget(self.canvas)
        self.setCentralWidget(central_widget)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)

    def update_plot(self):
        self.pdu_manager.run_nowait()
        raw_data = self.pdu_manager.read_pdu_raw_data("TB3", "laser_scan")
        if raw_data is None:
            return

        try:
            scan = pdu_to_py_LaserScan(raw_data)
        except Exception:
            return

        ranges = np.array(_to_float_list(scan.ranges), dtype=np.float32)
        if ranges.size == 0:
            return

        angles = scan.angle_min + np.arange(ranges.size, dtype=np.float32) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges < (scan.range_max - 1.0e-3))
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        self.last_range_max = float(scan.range_max)
        if x.size > 0:
            self.last_hit_x = x
            self.last_hit_y = y
            self.last_hit_time = time.time()

        self.ax.cla()
        self.ax.set_xlim(-scan.range_max, scan.range_max)
        self.ax.set_ylim(-scan.range_max, scan.range_max)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_title("TB3 2D LiDAR")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.grid(True)
        if x.size > 0:
            self.ax.scatter(x, y, s=6, c="tab:blue")
            status = f"valid hits: {x.size}/{ranges.size}"
        elif (time.time() - self.last_hit_time) < 1.0 and self.last_hit_x.size > 0:
            self.ax.scatter(self.last_hit_x, self.last_hit_y, s=6, c="tab:orange", alpha=0.65)
            status = f"valid hits: 0/{ranges.size}  (showing last hit cloud)"
        else:
            status = f"valid hits: 0/{ranges.size}"
        self.ax.text(
            0.02,
            0.98,
            status,
            transform=self.ax.transAxes,
            va="top",
            ha="left",
            fontsize=10,
            bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none"},
        )
        self.canvas.draw()


def main() -> int:
    config_path = sys.argv[1] if len(sys.argv) >= 2 else os.getenv("TB3_CONFIG_PATH", DEFAULT_CONFIG)
    config_path = str((REPO_ROOT / config_path).resolve()) if not os.path.isabs(config_path) else config_path

    if not os.path.exists(config_path):
        print(f"[ERROR] Config file not found at '{config_path}'")
        return 1

    pdu_manager = PduManager()
    pdu_manager.initialize(config_path=config_path, comm_service=ShmCommunicationService())
    pdu_manager.start_service_nowait()

    if not hakopy.init_for_external():
        print("[ERROR] hakopy.init_for_external() failed")
        return 1

    app = QApplication(sys.argv)
    window = Tb3LiDARWindow(pdu_manager)
    window.show()
    try:
        return app.exec_()
    finally:
        try:
            pdu_manager.stop_service_nowait()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
