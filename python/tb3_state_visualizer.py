#!/usr/bin/env python3

import atexit
import math
import os
import signal
import struct
import sys
import time
from collections import deque
from pathlib import Path

import hakopy
import numpy as np
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.gridspec import GridSpec

from hakoniwa_pdu.impl.shm_communication_service import ShmCommunicationService
from hakoniwa_pdu.pdu_manager import PduManager
from hakoniwa_pdu.pdu_msgs import binary_io
from hakoniwa_pdu.pdu_msgs.nav_msgs.pdu_conv_Odometry import pdu_to_py_Odometry
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_Imu import pdu_to_py_Imu
from hakoniwa_pdu.pdu_msgs.tf2_msgs.pdu_conv_TFMessage import pdu_to_py_TFMessage


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = "config/tb3-pdudef-compact.json"
DEFAULT_ROBOT = "TB3"

DEFAULT_TRAIL_LENGTH = 400
DEFAULT_VIEW_SIZE = 4.0

DEFAULT_ENABLE_JOINTS = True
DEFAULT_JOINT_VEL_DEADBAND = 0.1

DEFAULT_UPDATE_INTERVAL_MS = 100
DEFAULT_JOINT_HISTORY_SEC = 10.0


def _get_env_float(name: str, default_value: float) -> float:
    value = os.getenv(name)
    if value is None or value == "":
        return default_value
    try:
        return float(value)
    except ValueError:
        return default_value


def _get_env_int(name: str, default_value: int) -> int:
    value = os.getenv(name)
    if value is None or value == "":
        return default_value
    try:
        return int(value)
    except ValueError:
        return default_value


def _quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _read_i32(raw: bytearray, offset: int) -> int:
    return int.from_bytes(raw[offset:offset + 4], byteorder="little", signed=True)


def _decode_string_slot(chunk: bytes) -> str:
    end = chunk.find(b"\0")
    if end < 0:
        end = len(chunk)
    return chunk[:end].decode("utf-8", errors="replace")


def _decode_float64_varray(raw: bytearray, heap_off: int, array_len: int, array_off: int) -> list[float]:
    if array_len <= 0:
        return []

    start = heap_off + array_off
    end = start + array_len * 8
    blob = raw[start:end]

    if len(blob) != array_len * 8:
        return []

    return list(struct.unpack("<" + "d" * array_len, blob))


def _decode_joint_state_raw(raw: bytearray):
    meta = binary_io.PduMetaDataParser().load_pdu_meta(raw)
    if meta is None:
        return None

    base_off = meta.base_off
    heap_off = meta.heap_off

    name_len = _read_i32(raw, base_off + 136)
    name_off = _read_i32(raw, base_off + 140)
    pos_len = _read_i32(raw, base_off + 144)
    pos_off = _read_i32(raw, base_off + 148)
    vel_len = _read_i32(raw, base_off + 152)
    vel_off = _read_i32(raw, base_off + 156)
    eff_len = _read_i32(raw, base_off + 160)
    eff_off = _read_i32(raw, base_off + 164)

    names = []
    if name_len > 0:
        start = heap_off + name_off
        end = start + name_len * 128
        blob = raw[start:end]

        if len(blob) == name_len * 128:
            names = [
                _decode_string_slot(blob[i * 128:(i + 1) * 128])
                for i in range(name_len)
            ]

    positions = _decode_float64_varray(raw, heap_off, pos_len, pos_off)
    velocities = _decode_float64_varray(raw, heap_off, vel_len, vel_off)
    efforts = _decode_float64_varray(raw, heap_off, eff_len, eff_off)

    inferred_count = max(len(names), len(positions), len(velocities), len(efforts))
    if inferred_count <= 0:
        return None

    if not names:
        names = [f"joint_{i}" for i in range(inferred_count)]

    count = min(len(names), len(positions), len(velocities))
    if count <= 0:
        return None

    return {
        "names": [str(name) for name in names[:count]],
        "position": [float(v) for v in positions[:count]],
        "velocity": [float(v) for v in velocities[:count]],
        "effort": [float(v) for v in efforts[:count]] if efforts else [],
    }


class Tb3StateWindow(QMainWindow):
    def __init__(self, pdu_manager: PduManager, robot_name: str):
        super().__init__()

        self.pdu_manager = pdu_manager
        self.robot_name = robot_name

        self._closing = False
        self._shutdown_done = False

        self.view_size = _get_env_float("TB3_STATE_VIEW_SIZE", DEFAULT_VIEW_SIZE)
        self.trail = deque(maxlen=int(_get_env_float("TB3_STATE_TRAIL_LENGTH", DEFAULT_TRAIL_LENGTH)))

        self.last_imu = None
        self.last_joint_state = None
        self.last_odom = None
        self.last_tf = None

        self.enable_joints = os.getenv(
            "TB3_STATE_ENABLE_JOINTS",
            "1" if DEFAULT_ENABLE_JOINTS else "0",
        ) not in ("0", "false", "False")

        self.joint_decode_failed = False
        self.joint_vel_deadband = _get_env_float("TB3_STATE_JOINT_VEL_DEADBAND", DEFAULT_JOINT_VEL_DEADBAND)

        self.update_interval_ms = _get_env_int("TB3_STATE_UPDATE_INTERVAL_MS", DEFAULT_UPDATE_INTERVAL_MS)
        self.joint_history_sec = _get_env_float(
            "TB3_STATE_JOINT_HISTORY_SEC",
            DEFAULT_JOINT_HISTORY_SEC,
        )

        max_history_len = max(
            2,
            int((self.joint_history_sec * 1000.0) / max(1, self.update_interval_ms)) + 5,
        )
        self.joint_history = deque(maxlen=max_history_len)

        self.start_time = time.monotonic()

        self.setWindowTitle(f"{robot_name} State Visualizer")
        self.setGeometry(100, 100, 1200, 900)

        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)

        gs = GridSpec(
            3,
            2,
            figure=self.figure,
            height_ratios=[1.0, 1.0, 1.0],
            width_ratios=[1.0, 1.25],
        )

        self.ax_world = self.figure.add_subplot(gs[0:2, 0])
        self.ax_joint_vel = self.figure.add_subplot(gs[0, 1])
        self.ax_joint_pos = self.figure.add_subplot(gs[1, 1])
        self.ax_imu = self.figure.add_subplot(gs[2, 0])
        self.ax_tf = self.figure.add_subplot(gs[2, 1])

        central_widget = QWidget()
        layout = QVBoxLayout(central_widget)
        layout.addWidget(self.canvas)
        self.setCentralWidget(central_widget)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_view)
        self.timer.start(self.update_interval_ms)

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

    def _read(self, pdu_name, conv):
        raw = self.pdu_manager.read_pdu_raw_data(self.robot_name, pdu_name)
        if raw is None:
            return None

        try:
            return conv(raw)
        except Exception:
            return None

    def _read_joint_state(self):
        if not self.enable_joints or self.joint_decode_failed:
            return None

        raw = self.pdu_manager.read_pdu_raw_data(self.robot_name, "joint_states")
        if raw is None:
            return None

        try:
            return _decode_joint_state_raw(raw)
        except Exception:
            self.joint_decode_failed = True
            return None

    def _append_joint_history(self, joint_state):
        now = time.monotonic() - self.start_time

        self.joint_history.append(
            {
                "t": now,
                "names": list(joint_state["names"]),
                "positions": list(joint_state["position"]),
                "velocities": list(joint_state["velocity"]),
            }
        )

    def update_view(self):
        if self._closing or self.pdu_manager is None:
            return

        try:
            self.pdu_manager.run_nowait()
        except Exception:
            return

        imu = self._read("imu", pdu_to_py_Imu)
        joint_state = self._read_joint_state()
        odom = self._read("odom", pdu_to_py_Odometry)
        tf = self._read("tf", pdu_to_py_TFMessage)

        if imu is not None:
            self.last_imu = imu

        if joint_state is not None:
            self.last_joint_state = joint_state
            self._append_joint_history(joint_state)

        if odom is not None:
            self.last_odom = odom
            self.trail.append((odom.pose.pose.position.x, odom.pose.pose.position.y))

        if tf is not None:
            self.last_tf = tf

        self._draw_world()
        self._draw_joint()
        self._draw_imu()
        self._draw_tf()

        self.figure.tight_layout(pad=2.0)
        self.canvas.draw_idle()

    def _draw_world(self):
        ax = self.ax_world
        ax.cla()

        ax.set_title("Odometry / Heading")
        ax.set_xlabel("World X [m]")
        ax.set_ylabel("World Y [m]")
        ax.grid(True)
        ax.set_aspect("equal", adjustable="box")

        if self.last_odom is None:
            ax.text(
                0.5,
                0.5,
                "waiting for odom",
                ha="center",
                va="center",
                transform=ax.transAxes,
            )
            return

        x = self.last_odom.pose.pose.position.x
        y = self.last_odom.pose.pose.position.y
        yaw = _quat_to_yaw(self.last_odom.pose.pose.orientation)

        if self.trail:
            pts = np.array(self.trail, dtype=np.float64)
            ax.plot(pts[:, 0], pts[:, 1], linewidth=1.2, alpha=0.7)

        half = self.view_size * 0.5
        ax.set_xlim(x - half, x + half)
        ax.set_ylim(y - half, y + half)

        ax.scatter([x], [y], s=50, marker="x")

        arrow_len = max(0.12, self.view_size * 0.12)
        ax.arrow(
            x,
            y,
            arrow_len * math.cos(yaw),
            arrow_len * math.sin(yaw),
            head_width=max(0.03, self.view_size * 0.03),
            head_length=max(0.05, self.view_size * 0.05),
            length_includes_head=True,
        )

        speed = math.sqrt(
            self.last_odom.twist.twist.linear.x ** 2
            + self.last_odom.twist.twist.linear.y ** 2
        )
        vx = self.last_odom.twist.twist.linear.x
        vy = self.last_odom.twist.twist.linear.y
        wz = self.last_odom.twist.twist.angular.z

        ax.text(
            0.02,
            0.98,
            "pos=("
            f"{x:.2f}, {y:.2f}) yaw={math.degrees(yaw):.1f} deg\n"
            f"speed={speed:.3f} m/s  vx={vx:.3f}  vy={vy:.3f}  wz={wz:.3f}",
            transform=ax.transAxes,
            va="top",
            ha="left",
            bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none"},
        )

    def _draw_joint(self):
        self._draw_joint_velocity_history()
        self._draw_joint_position_history()

    def _draw_joint_velocity_history(self):
        ax = self.ax_joint_vel
        ax.cla()

        ax.set_title(f"Joint Velocity History ({self.joint_history_sec:.1f}s window)")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("velocity [rad/s]")
        ax.grid(True)

        if not self.enable_joints:
            ax.text(
                0.5,
                0.5,
                "joint_states disabled",
                ha="center",
                va="center",
                transform=ax.transAxes,
            )
            return

        if self.joint_decode_failed:
            ax.text(
                0.5,
                0.5,
                "joint_states decode failed",
                ha="center",
                va="center",
                transform=ax.transAxes,
            )
            return

        if not self.joint_history:
            ax.text(
                0.5,
                0.5,
                "waiting for joint velocity history",
                ha="center",
                va="center",
                transform=ax.transAxes,
            )
            return

        latest = self.joint_history[-1]
        latest_t = latest["t"]
        latest_names = latest["names"]
        t_min = latest_t - self.joint_history_sec

        visible_values = []

        for joint_index, joint_name in enumerate(latest_names):
            ts = []
            values = []

            for sample in self.joint_history:
                t = sample["t"]
                if t < t_min:
                    continue

                velocities = sample["velocities"]
                if joint_index >= len(velocities):
                    continue

                value = velocities[joint_index]
                if abs(value) < self.joint_vel_deadband:
                    value = 0.0

                ts.append(t - latest_t)
                values.append(value)
                visible_values.append(value)

            if ts:
                ax.plot(
                    ts,
                    values,
                    marker=".",
                    linewidth=1.2,
                    label=joint_name,
                )

        ax.set_xlim(-self.joint_history_sec, 0.0)

        if visible_values:
            vel_min = min(visible_values)
            vel_max = max(visible_values)

            if math.isclose(vel_min, vel_max):
                margin = max(1.0, abs(vel_min) * 0.1)
                ax.set_ylim(vel_min - margin, vel_max + margin)
            else:
                margin = max(0.1, (vel_max - vel_min) * 0.15)
                ax.set_ylim(vel_min - margin, vel_max + margin)

        ax.legend(loc="upper left", fontsize=8)

        lines = []
        for name, velocity in zip(latest_names, latest["velocities"]):
            if abs(velocity) < self.joint_vel_deadband:
                velocity = 0.0
            lines.append(f"{name}: vel={velocity:.3f} rad/s")

        ax.text(
            0.02,
            0.98,
            "\n".join(lines),
            transform=ax.transAxes,
            va="top",
            ha="left",
            fontsize=9,
            family="monospace",
            bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none"},
        )

    def _draw_joint_position_history(self):
        ax = self.ax_joint_pos
        ax.cla()

        ax.set_title(f"Joint Position History ({self.joint_history_sec:.1f}s window)")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("position [rad]")
        ax.grid(True)

        if not self.enable_joints:
            ax.text(
                0.5,
                0.5,
                "joint_states disabled",
                ha="center",
                va="center",
                transform=ax.transAxes,
            )
            return

        if self.joint_decode_failed:
            ax.text(
                0.5,
                0.5,
                "joint_states decode failed",
                ha="center",
                va="center",
                transform=ax.transAxes,
            )
            return

        if not self.joint_history:
            ax.text(
                0.5,
                0.5,
                "waiting for joint position history",
                ha="center",
                va="center",
                transform=ax.transAxes,
            )
            return

        latest = self.joint_history[-1]
        latest_t = latest["t"]
        latest_names = latest["names"]
        t_min = latest_t - self.joint_history_sec

        visible_values = []

        for joint_index, joint_name in enumerate(latest_names):
            ts = []
            values = []

            for sample in self.joint_history:
                t = sample["t"]
                if t < t_min:
                    continue

                positions = sample["positions"]
                if joint_index >= len(positions):
                    continue

                value = positions[joint_index]

                ts.append(t - latest_t)
                values.append(value)
                visible_values.append(value)

            if ts:
                ax.plot(
                    ts,
                    values,
                    marker=".",
                    linewidth=1.2,
                    label=joint_name,
                )

        ax.set_xlim(-self.joint_history_sec, 0.0)

        if visible_values:
            pos_min = min(visible_values)
            pos_max = max(visible_values)

            if math.isclose(pos_min, pos_max):
                margin = max(1.0, abs(pos_min) * 0.1)
                ax.set_ylim(pos_min - margin, pos_max + margin)
            else:
                margin = max(0.1, (pos_max - pos_min) * 0.15)
                ax.set_ylim(pos_min - margin, pos_max + margin)

        ax.legend(loc="upper left", fontsize=8)

        lines = []
        for name, position in zip(latest_names, latest["positions"]):
            phase = math.atan2(math.sin(position), math.cos(position))
            lines.append(
                f"{name}: pos={position:.3f} rad  phase={phase:.3f} rad"
            )

        ax.text(
            0.02,
            0.98,
            "\n".join(lines),
            transform=ax.transAxes,
            va="top",
            ha="left",
            fontsize=9,
            family="monospace",
            bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none"},
        )

    def _draw_imu(self):
        ax = self.ax_imu
        ax.cla()

        ax.set_title("IMU")
        ax.axis("off")

        if self.last_imu is None:
            ax.text(
                0.5,
                0.5,
                "waiting for imu",
                ha="center",
                va="center",
                transform=ax.transAxes,
            )
            return

        yaw = _quat_to_yaw(self.last_imu.orientation)

        lines = [
            f"orientation yaw: {math.degrees(yaw):.2f} deg",
            f"ang vel: ({self.last_imu.angular_velocity.x:.3f}, {self.last_imu.angular_velocity.y:.3f}, {self.last_imu.angular_velocity.z:.3f})",
            f"lin acc: ({self.last_imu.linear_acceleration.x:.3f}, {self.last_imu.linear_acceleration.y:.3f}, {self.last_imu.linear_acceleration.z:.3f})",
        ]

        ax.text(
            0.03,
            0.95,
            "\n".join(lines),
            va="top",
            ha="left",
            family="monospace",
        )

    def _draw_tf(self):
        ax = self.ax_tf
        ax.cla()

        ax.set_title("TF Frames")
        ax.axis("off")

        if self.last_tf is None or len(self.last_tf.transforms) == 0:
            ax.text(
                0.5,
                0.5,
                "waiting for tf",
                ha="center",
                va="center",
                transform=ax.transAxes,
            )
            return

        lines = []
        for tr in self.last_tf.transforms:
            lines.append(
                f"{tr.header.frame_id} -> {tr.child_frame_id}: "
                f"({tr.transform.translation.x:.3f}, "
                f"{tr.transform.translation.y:.3f}, "
                f"{tr.transform.translation.z:.3f})"
            )

        ax.text(
            0.03,
            0.95,
            "\n".join(lines),
            va="top",
            ha="left",
            family="monospace",
        )


def main() -> int:
    config_path = sys.argv[1] if len(sys.argv) >= 2 else os.getenv("HAKO_CONFIG_PATH", DEFAULT_CONFIG)
    robot_name = sys.argv[2] if len(sys.argv) >= 3 else os.getenv("HAKO_TB3_STATE_ROBOT_NAME", DEFAULT_ROBOT)

    config_path = str((REPO_ROOT / config_path).resolve()) if not os.path.isabs(config_path) else config_path
    joint_env = os.getenv("TB3_STATE_ENABLE_JOINTS")

    if not os.path.exists(config_path):
        print(f"[ERROR] Config file not found at '{config_path}'")
        return 1

    print(
        "[INFO] tb3_state_visualizer config="
        f"{config_path} robot={robot_name} "
        f"TB3_STATE_ENABLE_JOINTS={joint_env!r} default={DEFAULT_ENABLE_JOINTS} "
        f"TB3_STATE_JOINT_HISTORY_SEC={os.getenv('TB3_STATE_JOINT_HISTORY_SEC')!r} "
        f"TB3_STATE_UPDATE_INTERVAL_MS={os.getenv('TB3_STATE_UPDATE_INTERVAL_MS')!r} "
        f"TB3_STATE_JOINT_VEL_DEADBAND={os.getenv('TB3_STATE_JOINT_VEL_DEADBAND')!r}"
    )

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
    window = Tb3StateWindow(pdu_manager, robot_name)
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