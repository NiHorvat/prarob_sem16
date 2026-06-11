#!/usr/bin/env python3
"""PyQt5 GUI for the PRAROB 3-DOF drawing robot.

Setup:
  source /opt/ros/jazzy/setup.bash
  source ~/Desktop/robo/prarob_sem16/ros2_ws/install/setup.bash

Run in parallel with robot.launch.py or sim.launch.py:
  python3 GUI/robot_gui.py
"""

import json
import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float32MultiArray, String

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QImage, QPalette, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSlider,
    QStatusBar,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


AUTONOMOUS_COMMAND_TOPIC = "/autonomous_draw_node/command"
AUTONOMOUS_STATUS_TOPIC = "/autonomous_draw_node/status"
AUTONOMOUS_DEBUG_IMAGE_TOPIC = "/autonomous_draw_node/debug_image"


D1 = 0.0579
D2 = 0.0209
L1 = 0.224
L2 = 0.125
JOINT_LIMIT = math.pi / 2.0
JOINT_NAMES = ["joint1", "joint2", "joint3"]
HOME_JOINTS = [math.pi / 2.0, 0.0, -math.pi / 2.0]
STOP_JOINTS = [0.0, 0.0, 0.0]


def get_dk(q):
    if len(q) < 3:
        raise ValueError("get_dk expects [q1, q2, q3]")

    q1, q2, q3 = float(q[0]), float(q[1]), float(q[2])
    r = D2 + L1 * math.cos(q2) + L2 * math.cos(q2 + q3)
    z = D1 + L1 * math.sin(q2) + L2 * math.sin(q2 + q3)
    x = r * math.cos(q1)
    y = r * math.sin(q1)
    return [float(x), float(y), float(z)]


def get_ik(w):
    if len(w) < 3:
        raise ValueError("get_ik expects [x, y, z]")

    x, y, z = float(w[0]), float(w[1]), float(w[2])
    q1 = math.atan2(y, x)
    r = math.sqrt(x**2 + y**2) - D2
    zz = z - D1
    dist = math.sqrt(r**2 + zz**2)

    if dist > L1 + L2 or dist < abs(L1 - L2):
        return []

    cos_q3 = (dist**2 - L1**2 - L2**2) / (2.0 * L1 * L2)
    cos_q3 = max(-1.0, min(1.0, cos_q3))

    solutions = []
    for q3 in (math.acos(cos_q3), -math.acos(cos_q3)):
        beta = math.atan2(L2 * math.sin(q3), L1 + L2 * math.cos(q3))
        alpha = math.atan2(zz, r)
        q2 = alpha - beta
        candidate = [q1, q2, q3]
        if all(abs(angle) <= JOINT_LIMIT for angle in candidate):
            solutions.append([float(angle) for angle in candidate])

    return solutions


def get_closest_ik(q_all, q0):
    if not q_all:
        return None

    q0 = q0 if q0 is not None else [0.0, 0.0, 0.0]
    return min(
        q_all,
        key=lambda q: math.sqrt(sum((float(q[i]) - float(q0[i])) ** 2 for i in range(3))),
    )


def extract_joint_positions(msg):
    if msg.name:
        by_name = dict(zip(msg.name, msg.position))
        if all(name in by_name for name in JOINT_NAMES):
            return [float(by_name[name]) for name in JOINT_NAMES]

    if len(msg.position) >= 3:
        return [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]

    return None


class RobotStateWidget(QGroupBox):
    def __init__(self, parent, title="Current Robot State (live)", show_buttons=True):
        super().__init__(title)
        self.parent_window = parent

        self.joints_label = QLabel("Joint 1: --- rad  |  Joint 2: --- rad  |  Joint 3: --- rad")
        self.ee_label = QLabel("End effector: X=---  Y=---  Z=--- m")

        layout = QVBoxLayout()
        layout.addWidget(self.joints_label)
        layout.addWidget(self.ee_label)

        if show_buttons:
            buttons = QHBoxLayout()

            reset_button = QPushButton("Reset to Home")
            reset_button.clicked.connect(lambda: self.parent_window.publish_angles(HOME_JOINTS, "Reset to home sent"))

            stop_button = QPushButton("Emergency Stop")
            stop_button.setStyleSheet(
                "QPushButton { background-color: #b00020; color: white; font-weight: bold; }"
                "QPushButton:hover { background-color: #8a0018; }"
            )
            stop_button.clicked.connect(lambda: self.parent_window.publish_angles(STOP_JOINTS, "Emergency stop command sent"))

            buttons.addWidget(reset_button)
            buttons.addWidget(stop_button)
            buttons.addStretch(1)
            layout.addLayout(buttons)

        self.setLayout(layout)

    def refresh(self, joints):
        if joints is None:
            self.joints_label.setText("Joint 1: --- rad  |  Joint 2: --- rad  |  Joint 3: --- rad")
            self.ee_label.setText("End effector: X=---  Y=---  Z=--- m")
            return

        xyz = get_dk(joints)
        self.joints_label.setText(
            f"Joint 1: {joints[0]:.4f} rad  |  "
            f"Joint 2: {joints[1]:.4f} rad  |  "
            f"Joint 3: {joints[2]:.4f} rad"
        )
        self.ee_label.setText(
            f"End effector: X={xyz[0]:.4f}  Y={xyz[1]:.4f}  Z={xyz[2]:.4f} m"
        )


class RobotGui(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.current_joints = None
        self.joint_controls = []
        self.state_widgets = []

        self.angles_pub = self.node.create_publisher(
            Float32MultiArray, "/move_servos_node/angles", 10
        )
        self.xyz_pub = self.node.create_publisher(Point, "/ik_node/xyz", 10)
        self.joint_sub = self.node.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )

        # Autonomous backend wiring.
        self.command_pub = self.node.create_publisher(
            String, AUTONOMOUS_COMMAND_TOPIC, 10
        )
        self.status_sub = self.node.create_subscription(
            String, AUTONOMOUS_STATUS_TOPIC, self._autonomous_status_callback, 10
        )
        self.debug_image_sub = self.node.create_subscription(
            Image, AUTONOMOUS_DEBUG_IMAGE_TOPIC, self._debug_image_callback, 1
        )
        self._debug_qimage = None  # keep a reference so Qt does not free the buffer

        self.setWindowTitle("PRAROB Robot Controller")
        self.resize(1000, 700)

        self.status = QStatusBar()
        self.setStatusBar(self.status)
        self.show_status("ROS GUI ready")

        tabs = QTabWidget()
        tabs.addTab(self._build_manual_tab(), "Manual Control")
        tabs.addTab(self._build_autonomous_tab(), "Autonomous Mode")
        self.setCentralWidget(tabs)

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self._spin_ros_once)
        self.ros_timer.start(50)

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._refresh_state_widgets)
        self.ui_timer.start(100)

        self._update_fk_preview()

    def _joint_state_callback(self, msg):
        joints = extract_joint_positions(msg)
        if joints is not None:
            self.current_joints = joints

    def _spin_ros_once(self):
        try:
            rclpy.spin_once(self.node, timeout_sec=0)
        except Exception as exc:
            self.show_status(f"ROS spin error: {exc}", error=True)

    def _refresh_state_widgets(self):
        for widget in self.state_widgets:
            widget.refresh(self.current_joints)

    def show_status(self, message, error=False):
        if error:
            self.status.setStyleSheet("QStatusBar { color: #b00020; }")
        else:
            self.status.setStyleSheet("")
        self.status.showMessage(message)

    def publish_angles(self, angles, status_message="Joint command sent"):
        msg = Float32MultiArray()
        msg.data = [float(angle) for angle in angles]
        self.angles_pub.publish(msg)
        self.show_status(status_message)

    def _build_manual_tab(self):
        tab = QWidget()
        root = QVBoxLayout()

        columns = QHBoxLayout()
        columns.addWidget(self._build_direct_kinematics_group(), stretch=1)
        columns.addWidget(self._build_inverse_kinematics_group(), stretch=1)
        root.addLayout(columns, stretch=1)

        state = RobotStateWidget(self, show_buttons=True)
        self.state_widgets.append(state)
        root.addWidget(state)

        tab.setLayout(root)
        return tab

    def _build_direct_kinematics_group(self):
        group = QGroupBox("Direct Kinematics")
        layout = QVBoxLayout()

        grid = QGridLayout()
        grid.addWidget(QLabel("Joint"), 0, 0)
        grid.addWidget(QLabel("Slider"), 0, 1)
        grid.addWidget(QLabel("Radians"), 0, 2)

        for index in range(3):
            label = QLabel(f"Joint {index + 1}  [-90 deg .. +90 deg]")

            slider = QSlider(Qt.Horizontal)
            slider.setRange(-90, 90)
            slider.setValue(0)
            slider.setTickInterval(15)
            slider.setTickPosition(QSlider.TicksBelow)

            spin = QDoubleSpinBox()
            spin.setRange(-JOINT_LIMIT, JOINT_LIMIT)
            spin.setDecimals(4)
            spin.setSingleStep(0.001)
            spin.setValue(0.0)

            slider.valueChanged.connect(
                lambda value, spinbox=spin: self._slider_to_spinbox(value, spinbox)
            )
            spin.valueChanged.connect(
                lambda value, slider_widget=slider: self._spinbox_to_slider(value, slider_widget)
            )
            slider.valueChanged.connect(lambda _value: self._update_fk_preview())
            spin.valueChanged.connect(lambda _value: self._update_fk_preview())

            self.joint_controls.append((slider, spin))

            grid.addWidget(label, index + 1, 0)
            grid.addWidget(slider, index + 1, 1)
            grid.addWidget(spin, index + 1, 2)

        send_button = QPushButton("Send to Robot")
        send_button.clicked.connect(self._send_joint_controls)

        self.fk_x_label = QLabel("X: 0.0000 m")
        self.fk_y_label = QLabel("Y: 0.0000 m")
        self.fk_z_label = QLabel("Z: 0.0000 m")

        fk_group = QGroupBox("FK Result")
        fk_layout = QVBoxLayout()
        fk_layout.addWidget(self.fk_x_label)
        fk_layout.addWidget(self.fk_y_label)
        fk_layout.addWidget(self.fk_z_label)
        fk_group.setLayout(fk_layout)

        layout.addLayout(grid)
        layout.addWidget(send_button)
        layout.addWidget(fk_group)
        layout.addStretch(1)
        group.setLayout(layout)
        return group

    def _build_inverse_kinematics_group(self):
        group = QGroupBox("Inverse Kinematics")
        layout = QVBoxLayout()

        form = QFormLayout()
        self.x_spin = self._make_pose_spin(-0.4, 0.4, 0.1500)
        self.y_spin = self._make_pose_spin(-0.4, 0.4, 0.0000)
        self.z_spin = self._make_pose_spin(-0.1, 0.5, 0.0500)
        form.addRow("X (m):", self.x_spin)
        form.addRow("Y (m):", self.y_spin)
        form.addRow("Z (m):", self.z_spin)

        solve_button = QPushButton("Solve IK & Send")
        solve_button.clicked.connect(self._solve_ik_and_send)

        self.ik_joint_labels = [
            QLabel("Joint 1: --- rad"),
            QLabel("Joint 2: --- rad"),
            QLabel("Joint 3: --- rad"),
        ]
        ik_group = QGroupBox("IK Result")
        ik_layout = QVBoxLayout()
        for label in self.ik_joint_labels:
            ik_layout.addWidget(label)
        ik_group.setLayout(ik_layout)

        layout.addLayout(form)
        layout.addWidget(solve_button)
        layout.addWidget(ik_group)
        layout.addStretch(1)
        group.setLayout(layout)
        return group

    def _build_autonomous_tab(self):
        tab = QWidget()
        layout = QHBoxLayout()

        # ----- left column: command + status ------------------------------
        left = QVBoxLayout()

        command_group = QGroupBox("Natural Language Command")
        command_layout = QVBoxLayout()
        self.command_edit = QTextEdit()
        self.command_edit.setPlaceholderText("e.g. connect car and plane, avoid football")
        self.command_edit.setFixedHeight(56)

        buttons = QHBoxLayout()
        send_button = QPushButton("Send Command")
        send_button.clicked.connect(self._send_autonomous_command)
        stop_button = QPushButton("Stop")
        stop_button.setStyleSheet(
            "QPushButton { background-color: #b00020; color: white; font-weight: bold; }"
            "QPushButton:hover { background-color: #8a0018; }"
        )
        stop_button.clicked.connect(self._stop_autonomous_command)
        buttons.addWidget(send_button)
        buttons.addWidget(stop_button)

        command_layout.addWidget(self.command_edit)
        command_layout.addLayout(buttons)
        command_group.setLayout(command_layout)

        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout()
        self.autonomous_state_label = QLabel("State: idle")
        self.autonomous_state_label.setStyleSheet("font-weight: bold;")
        self.autonomous_status_label = QLabel("Waiting for command...")
        self.autonomous_status_label.setWordWrap(True)
        self.autonomous_parsed_label = QLabel("Connect: ---   Avoid: ---")
        self.autonomous_parsed_label.setWordWrap(True)
        self.autonomous_timing_label = QLabel("Timing: ---")
        status_layout.addWidget(self.autonomous_state_label)
        status_layout.addWidget(self.autonomous_status_label)
        status_layout.addWidget(self.autonomous_parsed_label)
        status_layout.addWidget(self.autonomous_timing_label)
        status_group.setLayout(status_layout)

        state = RobotStateWidget(self, show_buttons=True)
        self.state_widgets.append(state)

        left.addWidget(command_group)
        left.addWidget(status_group)
        left.addStretch(1)
        left.addWidget(state)

        # ----- right column: live debug overlay ---------------------------
        debug_group = QGroupBox("Detections / Obstacles / Planned Path")
        debug_layout = QVBoxLayout()
        self.debug_image_label = QLabel("No debug image yet")
        self.debug_image_label.setAlignment(Qt.AlignCenter)
        self.debug_image_label.setMinimumSize(480, 360)
        self.debug_image_label.setStyleSheet("background-color: #202020; color: #aaaaaa;")
        debug_layout.addWidget(self.debug_image_label)
        debug_group.setLayout(debug_layout)

        layout.addLayout(left, stretch=1)
        layout.addWidget(debug_group, stretch=1)
        tab.setLayout(layout)
        return tab

    def _make_pose_spin(self, minimum, maximum, value):
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setSingleStep(0.001)
        spin.setDecimals(4)
        spin.setValue(value)
        return spin

    def _slider_to_spinbox(self, degrees, spinbox):
        radians = math.radians(float(degrees))
        spinbox.blockSignals(True)
        spinbox.setValue(radians)
        spinbox.blockSignals(False)

    def _spinbox_to_slider(self, radians, slider):
        degrees = int(round(math.degrees(float(radians))))
        slider.blockSignals(True)
        slider.setValue(degrees)
        slider.blockSignals(False)

    def _joint_values_from_controls(self):
        return [float(spin.value()) for _slider, spin in self.joint_controls]

    def _update_fk_preview(self):
        xyz = get_dk(self._joint_values_from_controls())
        self.fk_x_label.setText(f"X: {xyz[0]:.4f} m")
        self.fk_y_label.setText(f"Y: {xyz[1]:.4f} m")
        self.fk_z_label.setText(f"Z: {xyz[2]:.4f} m")

    def _send_joint_controls(self):
        self.publish_angles(self._joint_values_from_controls(), "Joint command sent")

    def _solve_ik_and_send(self):
        target = [self.x_spin.value(), self.y_spin.value(), self.z_spin.value()]
        solutions = get_ik(target)
        if not solutions:
            self.show_status("IK: No solution", error=True)
            for label in self.ik_joint_labels:
                label.setText(label.text().split(":")[0] + ": --- rad")
            return

        q = get_closest_ik(solutions, self.current_joints)
        for index, value in enumerate(q):
            self.ik_joint_labels[index].setText(f"Joint {index + 1}: {value:.4f} rad")

        self.publish_angles(q, "IK solution sent")

    def _send_autonomous_command(self):
        command = self.command_edit.toPlainText().strip()
        if not command:
            self.show_status("Autonomous mode: empty command", error=True)
            return

        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.autonomous_state_label.setText("State: sent")
        self.autonomous_status_label.setText(f"Command sent: {command}")
        self.show_status(f"Autonomous command sent to {AUTONOMOUS_COMMAND_TOPIC}")

    def _stop_autonomous_command(self):
        msg = String()
        msg.data = "stop"
        self.command_pub.publish(msg)
        self.show_status("Autonomous stop requested")

    def _autonomous_status_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            self.autonomous_status_label.setText(msg.data)
            return

        state = payload.get("state", "?")
        message = payload.get("message", "")
        self.autonomous_state_label.setText(f"State: {state}")
        self.autonomous_status_label.setText(message)

        parsed = payload.get("parsed") or {}
        connect = parsed.get("connect")
        avoid = parsed.get("avoid")
        if connect is not None or avoid is not None:
            self.autonomous_parsed_label.setText(
                f"Connect: {connect or '---'}   Avoid: {avoid or '---'}"
            )

        timing = payload.get("timing") or {}
        if timing:
            parts = [f"{k}={v}s" for k, v in timing.items()]
            self.autonomous_timing_label.setText("Timing: " + "  ".join(parts))

        error = state in ("failed", "aborted")
        self.show_status(f"Autonomous [{state}]: {message}", error=error)

    def _debug_image_callback(self, msg):
        try:
            channels = 3
            height, width = msg.height, msg.width
            buffer = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            buffer = buffer.reshape(height, msg.step)[:, : width * channels]
            frame = buffer.reshape(height, width, channels)
            if msg.encoding in ("bgr8", "8UC3", ""):
                frame = frame[:, :, ::-1]  # BGR -> RGB
            frame = np.ascontiguousarray(frame)
            image = QImage(frame.data, width, height, width * channels,
                           QImage.Format_RGB888)
            self._debug_qimage = image  # retain buffer
            pixmap = QPixmap.fromImage(image).scaled(
                self.debug_image_label.width(),
                self.debug_image_label.height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
            self.debug_image_label.setPixmap(pixmap)
        except Exception as exc:
            self.show_status(f"Debug image error: {exc}", error=True)


def main():
    rclpy.init()
    app = QApplication(sys.argv)

    node = rclpy.create_node("robot_gui_node")
    window = RobotGui(node)
    window.show()

    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
