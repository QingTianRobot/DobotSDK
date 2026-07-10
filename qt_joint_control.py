import sys
import threading
import time
from typing import Callable, List, Optional

try:
    from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal as Signal
    from PyQt5.QtWidgets import (
        QApplication,
        QCheckBox,
        QDoubleSpinBox,
        QFormLayout,
        QFrame,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QSlider,
        QSpinBox,
        QVBoxLayout,
        QWidget,
    )

    APP_EXEC = "exec_"
except ImportError:
    from PySide6.QtCore import QThread, QTimer, Qt, Signal
    from PySide6.QtWidgets import (
        QApplication,
        QCheckBox,
        QDoubleSpinBox,
        QFormLayout,
        QFrame,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QSlider,
        QSpinBox,
        QVBoxLayout,
        QWidget,
    )

    APP_EXEC = "exec"

from dobot_api import DobotApiDashboard, DobotApiFeedBack, DobotApiMove


FEEDBACK_MAGIC = 0x123456789ABCDEF
JOINT_COUNT = 6
SCALE = 100

ROBOT_MODE_TEXT = {
    1: "INIT",
    2: "BRAKE_OPEN",
    3: "POWER_DISABLED",
    4: "NOT_ENABLE",
    5: "ENABLE",
    6: "BACKDRIVE",
    7: "RUNNING",
    8: "RECORDING",
    9: "ERROR",
    10: "PAUSE",
    11: "JOG",
}


class FeedbackThread(QThread):
    joint_update = Signal(list, dict)
    connected = Signal()
    error = Signal(str)
    disconnected = Signal()

    def __init__(self, ip: str, port: int, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ip = ip
        self.port = port
        self._running = False
        self._client: Optional[DobotApiFeedBack] = None

    def run(self) -> None:
        self._running = True
        try:
            self._client = DobotApiFeedBack(self.ip, self.port)
            self.connected.emit()
            while self._running:
                packet = self._client.feedBackData()
                if packet is None or len(packet) == 0:
                    continue
                if int(packet["test_value"][0]) != FEEDBACK_MAGIC:
                    continue

                joints = [float(value) for value in packet["q_actual"][0]]
                status = {
                    "robot_mode": int(packet["robot_mode"][0]),
                    "speed_scaling": float(packet["speed_scaling"][0]),
                    "enable_status": int(packet["enable_status"][0]),
                    "running_status": int(packet["running_status"][0]),
                    "error_status": int(packet["error_status"][0]),
                }
                self.joint_update.emit(joints, status)
        except Exception as exc:
            if self._running:
                self.error.emit(str(exc))
        finally:
            self._running = False
            if self._client is not None:
                try:
                    self._client.close()
                except Exception:
                    pass
                self._client = None
            self.disconnected.emit()

    def stop(self) -> None:
        self._running = False
        if self._client is not None:
            try:
                self._client.close()
            except Exception:
                pass


class JointRow(QWidget):
    target_changed = Signal()
    slider_released = Signal()
    drag_state_changed = Signal(bool)

    def __init__(self, index: int, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.index = index
        self._syncing = False

        self.name_label = QLabel(f"J{index + 1}")
        self.actual_label = QLabel("--.-- deg")
        self.actual_label.setMinimumWidth(88)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(-360 * SCALE, 360 * SCALE)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(100)

        self.target_spin = QDoubleSpinBox()
        self.target_spin.setRange(-360.0, 360.0)
        self.target_spin.setDecimals(2)
        self.target_spin.setSingleStep(0.1)
        self.target_spin.setSuffix(" deg")
        self.target_spin.setMinimumWidth(110)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 4, 0, 4)
        layout.addWidget(self.name_label)
        layout.addWidget(QLabel("actual"))
        layout.addWidget(self.actual_label)
        layout.addWidget(self.slider, 1)
        layout.addWidget(QLabel("target"))
        layout.addWidget(self.target_spin)

        self.slider.valueChanged.connect(self._on_slider_changed)
        self.slider.sliderPressed.connect(lambda: self.drag_state_changed.emit(True))
        self.slider.sliderReleased.connect(self._on_slider_released)
        self.target_spin.valueChanged.connect(self._on_spin_changed)

    def set_actual(self, value: float) -> None:
        self.actual_label.setText(f"{value:.2f} deg")

    def set_target(self, value: float) -> None:
        clamped = max(self.target_spin.minimum(), min(self.target_spin.maximum(), value))
        scaled = int(round(clamped * SCALE))
        self._syncing = True
        self.slider.setValue(scaled)
        self.target_spin.setValue(scaled / SCALE)
        self._syncing = False

    def target(self) -> float:
        return float(self.target_spin.value())

    def _on_slider_changed(self, value: int) -> None:
        if self._syncing:
            return
        self._syncing = True
        self.target_spin.setValue(value / SCALE)
        self._syncing = False
        self.target_changed.emit()

    def _on_spin_changed(self, value: float) -> None:
        if self._syncing:
            return
        self._syncing = True
        self.slider.setValue(int(round(value * SCALE)))
        self._syncing = False
        self.target_changed.emit()

    def _on_slider_released(self) -> None:
        self.drag_state_changed.emit(False)
        self.slider_released.emit()


class JointControlWindow(QMainWindow):
    command_result = Signal(str)
    command_error = Signal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dobot Joint Angle Monitor")
        self.resize(960, 520)

        self.dashboard: Optional[DobotApiDashboard] = None
        self.move: Optional[DobotApiMove] = None
        self.feedback_thread: Optional[FeedbackThread] = None
        self.command_lock = threading.Lock()
        self.command_busy = False

        self.latest_actual: Optional[List[float]] = None
        self.targets_initialized = False
        self.dragging_count = 0
        self.pending_servo = False
        self.last_servo_time = 0.0

        self._build_ui()
        self._set_connected_ui(False)

        self.command_result.connect(self._append_log)
        self.command_error.connect(self._show_command_error)

        self.servo_timer = QTimer(self)
        self.servo_timer.setInterval(80)
        self.servo_timer.timeout.connect(self._maybe_send_servo)
        self.servo_timer.start()

    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(12)
        self.setCentralWidget(central)

        connection_group = QGroupBox("Connection")
        form = QFormLayout(connection_group)
        form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)

        self.ip_edit = QLineEdit("192.168.5.1")
        self.dashboard_port = QSpinBox()
        self.dashboard_port.setRange(1, 65535)
        self.dashboard_port.setValue(29999)
        self.move_port = QSpinBox()
        self.move_port.setRange(1, 65535)
        self.move_port.setValue(30003)
        self.feedback_port = QSpinBox()
        self.feedback_port.setRange(1, 65535)
        self.feedback_port.setValue(30004)

        endpoint_layout = QHBoxLayout()
        endpoint_layout.addWidget(QLabel("IP"))
        endpoint_layout.addWidget(self.ip_edit, 1)
        endpoint_layout.addWidget(QLabel("Dashboard"))
        endpoint_layout.addWidget(self.dashboard_port)
        endpoint_layout.addWidget(QLabel("Move"))
        endpoint_layout.addWidget(self.move_port)
        endpoint_layout.addWidget(QLabel("Feedback"))
        endpoint_layout.addWidget(self.feedback_port)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self._toggle_connection)
        endpoint_layout.addWidget(self.connect_button)
        form.addRow(endpoint_layout)

        control_layout = QHBoxLayout()
        self.enable_button = QPushButton("Enable")
        self.enable_button.clicked.connect(self._toggle_enable)
        self.clear_button = QPushButton("Clear Error")
        self.clear_button.clicked.connect(lambda: self._run_command("ClearError", self.dashboard.ClearError))
        self.stop_button = QPushButton("Stop")
        self.stop_button.clicked.connect(lambda: self._run_command("ResetRobot", self.dashboard.ResetRobot))

        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(1, 100)
        self.speed_spin.setValue(20)
        self.speed_button = QPushButton("Set Speed")
        self.speed_button.clicked.connect(self._set_speed)

        control_layout.addWidget(self.enable_button)
        control_layout.addWidget(self.clear_button)
        control_layout.addWidget(self.stop_button)
        control_layout.addSpacing(16)
        control_layout.addWidget(QLabel("Speed %"))
        control_layout.addWidget(self.speed_spin)
        control_layout.addWidget(self.speed_button)
        control_layout.addStretch(1)
        form.addRow(control_layout)
        root.addWidget(connection_group)

        status_group = QGroupBox("Status")
        status_layout = QGridLayout(status_group)
        self.connection_label = QLabel("Disconnected")
        self.mode_label = QLabel("--")
        self.speed_label = QLabel("--")
        self.enable_label = QLabel("--")
        self.running_label = QLabel("--")
        self.error_label = QLabel("--")
        status_layout.addWidget(QLabel("Connection"), 0, 0)
        status_layout.addWidget(self.connection_label, 0, 1)
        status_layout.addWidget(QLabel("Robot Mode"), 0, 2)
        status_layout.addWidget(self.mode_label, 0, 3)
        status_layout.addWidget(QLabel("Speed Scaling"), 0, 4)
        status_layout.addWidget(self.speed_label, 0, 5)
        status_layout.addWidget(QLabel("Enabled"), 1, 0)
        status_layout.addWidget(self.enable_label, 1, 1)
        status_layout.addWidget(QLabel("Running"), 1, 2)
        status_layout.addWidget(self.running_label, 1, 3)
        status_layout.addWidget(QLabel("Error"), 1, 4)
        status_layout.addWidget(self.error_label, 1, 5)
        root.addWidget(status_group)

        joint_group = QGroupBox("Joint Angles")
        joint_layout = QVBoxLayout(joint_group)
        self.rows = [JointRow(i) for i in range(JOINT_COUNT)]
        for row in self.rows:
            row.target_changed.connect(self._on_target_changed)
            row.slider_released.connect(self._on_slider_released)
            row.drag_state_changed.connect(self._on_drag_state_changed)
            joint_layout.addWidget(row)
        root.addWidget(joint_group, 1)

        action_layout = QHBoxLayout()
        self.apply_button = QPushButton("Send JointMovJ")
        self.apply_button.clicked.connect(self._send_joint_movj)
        self.sync_button = QPushButton("Sync Sliders To Actual")
        self.sync_button.clicked.connect(self._sync_targets_to_actual)
        self.realtime_servo = QCheckBox("Realtime ServoJ while dragging")
        self.realtime_servo.setChecked(False)
        self.send_on_release = QCheckBox("Send JointMovJ on slider release")
        self.send_on_release.setChecked(True)

        action_layout.addWidget(self.apply_button)
        action_layout.addWidget(self.sync_button)
        action_layout.addWidget(self.send_on_release)
        action_layout.addWidget(self.realtime_servo)
        action_layout.addStretch(1)
        root.addLayout(action_layout)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        root.addWidget(line)

        self.log_label = QLabel("Ready")
        self.log_label.setWordWrap(True)
        root.addWidget(self.log_label)

    def _toggle_connection(self) -> None:
        if self.dashboard is not None or self.move is not None or self.feedback_thread is not None:
            self._disconnect_robot()
            return

        ip = self.ip_edit.text().strip()
        if not ip:
            QMessageBox.warning(self, "Connection", "IP address is empty.")
            return

        try:
            self.dashboard = DobotApiDashboard(ip, int(self.dashboard_port.value()))
            self.move = DobotApiMove(ip, int(self.move_port.value()))
        except Exception as exc:
            self.dashboard = None
            self.move = None
            QMessageBox.critical(self, "Connection Failed", str(exc))
            return

        self.feedback_thread = FeedbackThread(ip, int(self.feedback_port.value()), self)
        self.feedback_thread.connected.connect(lambda: self._append_log("Feedback connected."))
        self.feedback_thread.joint_update.connect(self._on_joint_update)
        self.feedback_thread.error.connect(self._on_feedback_error)
        self.feedback_thread.disconnected.connect(lambda: self._append_log("Feedback disconnected."))
        self.feedback_thread.start()

        self.targets_initialized = False
        self._set_connected_ui(True)
        self._append_log("Dashboard and move ports connected.")

    def _disconnect_robot(self) -> None:
        if self.feedback_thread is not None:
            self.feedback_thread.stop()
            self.feedback_thread.wait(1500)
            self.feedback_thread = None
        for client in (self.dashboard, self.move):
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass
        self.dashboard = None
        self.move = None
        self.targets_initialized = False
        self._set_connected_ui(False)
        self._append_log("Disconnected.")

    def _set_connected_ui(self, connected: bool) -> None:
        self.connect_button.setText("Disconnect" if connected else "Connect")
        self.connection_label.setText("Connected" if connected else "Disconnected")
        for widget in (
            self.enable_button,
            self.clear_button,
            self.stop_button,
            self.speed_button,
            self.apply_button,
            self.sync_button,
            self.realtime_servo,
            self.send_on_release,
        ):
            widget.setEnabled(connected)

    def _on_joint_update(self, joints: list, status: dict) -> None:
        self.latest_actual = joints
        for row, value in zip(self.rows, joints):
            row.set_actual(value)
        if not self.targets_initialized:
            self._sync_targets_to_actual()
            self.targets_initialized = True

        mode = status.get("robot_mode", 0)
        self.mode_label.setText(f"{mode} {ROBOT_MODE_TEXT.get(mode, '')}".strip())
        self.speed_label.setText(f"{status.get('speed_scaling', 0.0):.2f}")
        self.enable_label.setText("Yes" if status.get("enable_status") else "No")
        self.running_label.setText("Yes" if status.get("running_status") else "No")
        self.error_label.setText("Yes" if status.get("error_status") else "No")
        self.enable_button.setText("Disable" if status.get("enable_status") else "Enable")

    def _on_feedback_error(self, message: str) -> None:
        self._append_log(f"Feedback error: {message}")

    def _sync_targets_to_actual(self) -> None:
        if self.latest_actual is None:
            return
        for row, value in zip(self.rows, self.latest_actual):
            row.set_target(value)
        self.pending_servo = False
        self._append_log("Targets synced to actual joint angles.")

    def _targets(self) -> List[float]:
        return [row.target() for row in self.rows]

    def _on_target_changed(self) -> None:
        self.pending_servo = True

    def _on_drag_state_changed(self, dragging: bool) -> None:
        self.dragging_count += 1 if dragging else -1
        self.dragging_count = max(0, self.dragging_count)

    def _on_slider_released(self) -> None:
        if self.send_on_release.isChecked():
            self._send_joint_movj()

    def _toggle_enable(self) -> None:
        if self.dashboard is None:
            return
        if self.enable_button.text() == "Disable":
            self._run_command("DisableRobot", self.dashboard.DisableRobot)
        else:
            self._run_command("EnableRobot", self.dashboard.EnableRobot)

    def _set_speed(self) -> None:
        if self.dashboard is None:
            return
        speed = int(self.speed_spin.value())
        self._run_command(f"SpeedFactor({speed})", lambda: self.dashboard.SpeedFactor(speed))

    def _send_joint_movj(self) -> None:
        if self.move is None:
            return
        joints = self._targets()
        self._run_command(
            "JointMovJ({})".format(", ".join(f"{value:.2f}" for value in joints)),
            lambda: self.move.JointMovJ(*joints),
        )

    def _maybe_send_servo(self) -> None:
        if not self.realtime_servo.isChecked():
            return
        if self.move is None or self.dragging_count <= 0 or not self.pending_servo:
            return
        now = time.monotonic()
        if now - self.last_servo_time < 0.12:
            return
        self.last_servo_time = now
        self.pending_servo = False
        joints = self._targets()
        self._run_command(
            "ServoJ({})".format(", ".join(f"{value:.2f}" for value in joints)),
            lambda: self.move.ServoJ(*joints, t=0.12),
            skip_if_busy=True,
        )

    def _run_command(
        self,
        label: str,
        func: Callable[[], str],
        skip_if_busy: bool = False,
    ) -> None:
        if func is None:
            return
        if skip_if_busy and self.command_busy:
            return

        def worker() -> None:
            with self.command_lock:
                self.command_busy = True
                try:
                    reply = func()
                    self.command_result.emit(f"{label} -> {reply}")
                except Exception as exc:
                    self.command_error.emit(f"{label}: {exc}")
                finally:
                    self.command_busy = False

        threading.Thread(target=worker, daemon=True).start()

    def _append_log(self, text: str) -> None:
        self.log_label.setText(text)

    def _show_command_error(self, text: str) -> None:
        self._append_log(f"Command error: {text}")
        QMessageBox.warning(self, "Command Error", text)

    def closeEvent(self, event) -> None:
        self._disconnect_robot()
        event.accept()


def main() -> int:
    app = QApplication(sys.argv)
    window = JointControlWindow()
    window.show()
    return getattr(app, APP_EXEC)()


if __name__ == "__main__":
    sys.exit(main())
