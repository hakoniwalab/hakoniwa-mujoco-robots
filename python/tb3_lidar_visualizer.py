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
from PyQt5.QtCore import QTimer, Qt
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


def _get_env_float(name: str, default_value: float) -> float:
    value = os.getenv(name)
    if value is None or value == "":
        return default_value
    try:
        return float(value)
    except ValueError:
        return default_value


class Tb3LiDARWindow(QMainWindow):
    def __init__(self, pdu_manager: PduManager):
        super().__init__()
        self.pdu_manager = pdu_manager

        self.last_hit_x = np.array([], dtype=np.float32)
        self.last_hit_y = np.array([], dtype=np.float32)
        self.last_hit_time = 0.0

        self._closing = False
        self._shutdown_done = False

        # 表示範囲。
        # 0以下なら自動縮尺。
        # 例: TB3_LIDAR_PLOT_RANGE=1.5 で ±1.5m 固定。
        self.fixed_plot_range = _get_env_float("TB3_LIDAR_PLOT_RANGE", 0.0)

        # 自動縮尺時の最小表示範囲。
        self.min_plot_range = _get_env_float("TB3_LIDAR_MIN_PLOT_RANGE", 0.8)

        self.setWindowTitle("TB3 2D LiDAR")
        self.setGeometry(50, 50, 700, 700)

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
            if hasattr(self, "timer") and self.timer is not None:
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

    def _calc_plot_range(self, x, y, scan_range_max: float) -> float:
        if self.fixed_plot_range > 0.0:
            return min(self.fixed_plot_range, scan_range_max)

        if x.size > 0:
            r = np.sqrt(x * x + y * y)
            if r.size > 0:
                # 最大値そのものだと外れ値で広がるため、95パーセンタイルを使う
                view = float(np.percentile(r, 95)) * 1.35
                return max(self.min_plot_range, min(scan_range_max, view))

        if self.last_hit_x.size > 0:
            r = np.sqrt(self.last_hit_x * self.last_hit_x + self.last_hit_y * self.last_hit_y)
            if r.size > 0:
                view = float(np.percentile(r, 95)) * 1.35
                return max(self.min_plot_range, min(scan_range_max, view))

        return min(scan_range_max, self.min_plot_range)

    def update_plot(self):
        if self._closing or self.pdu_manager is None:
            return

        try:
            self.pdu_manager.run_nowait()
            raw_data = self.pdu_manager.read_pdu_raw_data("TB3", "laser_scan")
        except Exception:
            return

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

        valid = (
            np.isfinite(ranges)
            & (ranges >= scan.range_min)
            & (ranges < (scan.range_max - 1.0e-3))
        )

        # LaserScan標準のローカル座標:
        #   forward = range * cos(angle)
        #   left    = range * sin(angle)
        #
        # 表示では「前方を上」にしたいので:
        #   plot_x = -left
        #   plot_y = forward
        #
        # これで:
        #   前方: 上
        #   左:   左
        #   右:   右
        forward = ranges[valid] * np.cos(angles[valid])
        left = ranges[valid] * np.sin(angles[valid])

        x = -left
        y = forward

        if x.size > 0:
            self.last_hit_x = x
            self.last_hit_y = y
            self.last_hit_time = time.time()

        plot_range = self._calc_plot_range(x, y, float(scan.range_max))

        self.ax.cla()
        self.ax.set_xlim(-plot_range, plot_range)
        self.ax.set_ylim(-plot_range, plot_range)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_title("TB3 2D LiDAR")
        self.ax.set_xlabel("Right / Left [m]")
        self.ax.set_ylabel("Forward [m]")
        self.ax.grid(True)

        # ロボット中心
        self.ax.scatter([0.0], [0.0], s=40, marker="x")

        # 前方方向
        arrow_len = plot_range * 0.18
        self.ax.arrow(
            0.0,
            0.0,
            0.0,
            arrow_len,
            head_width=plot_range * 0.035,
            head_length=plot_range * 0.05,
            length_includes_head=True,
        )
        self.ax.text(0.0, arrow_len * 1.15, "front", ha="center", va="bottom")

        # 1m目盛りの補助円。表示範囲が小さい時は省略。
        if plot_range >= 1.0:
            circle = self.figure.gca().add_patch(
                self.ax.add_artist(
                    __import__("matplotlib").patches.Circle(
                        (0.0, 0.0),
                        1.0,
                        fill=False,
                        linestyle="--",
                        linewidth=0.8,
                        alpha=0.5,
                    )
                )
            )

        if x.size > 0:
            self.ax.scatter(x, y, s=8)
            nearest = float(np.min(ranges[valid])) if np.any(valid) else float("nan")
            status = f"valid hits: {x.size}/{ranges.size}  nearest: {nearest:.3f} m"
        elif (time.time() - self.last_hit_time) < 1.0 and self.last_hit_x.size > 0:
            self.ax.scatter(self.last_hit_x, self.last_hit_y, s=8, alpha=0.6)
            status = f"valid hits: 0/{ranges.size}  (showing last hit cloud)"
        else:
            status = f"valid hits: 0/{ranges.size}"

        self.ax.text(
            0.02,
            0.98,
            status + f"\nview: ±{plot_range:.2f} m",
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
        try:
            pdu_manager.stop_service_nowait()
        except Exception:
            pass
        return 1

    app = QApplication(sys.argv)
    window = Tb3LiDARWindow(pdu_manager)

    # Python が SIGINT を処理できるようにするための空タイマー。
    # Qt の event loop だけだと Ctrl+C が遅延または無視されることがある。
    sigint_timer = QTimer()
    sigint_timer.start(200)
    sigint_timer.timeout.connect(lambda: None)

    def request_shutdown(*_args):
        window.shutdown()
        window.close()
        app.quit()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    app.aboutToQuit.connect(window.shutdown)
    atexit.register(window.shutdown)

    window.show()

    try:
        ret = app.exec_()
    except KeyboardInterrupt:
        request_shutdown()
        ret = 130
    finally:
        try:
            sigint_timer.stop()
        except Exception:
            pass

        window.shutdown()

        try:
            window.close()
            window.deleteLater()
            app.processEvents()
        except Exception:
            pass

    return int(ret)


if __name__ == "__main__":
    raise SystemExit(main())