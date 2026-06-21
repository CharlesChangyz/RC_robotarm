#!/usr/bin/env python3
"""PySide6 GUI TF target publisher for rc_arm_2."""

import argparse
import collections
import json
import math
import signal
import shlex
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from PySide6.QtCore import QObject, QProcess, Qt, QTimer, Signal, Slot
from PySide6.QtGui import QColor, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QDial,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Int32, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
import tf2_ros

ROOT_DIR = Path(__file__).resolve().parent.parent
RC_MOVEIT_DIR = ROOT_DIR / "rc_moveit"
sys.path.insert(0, str(ROOT_DIR / "rc_moveit" / "rc_arm_moveit_config" / "launch"))

from rc_arm_world_pitch_kinematics import RcArmWorldPitchKinematics  # noqa: E402


SCRIPT_RUN_MUJOCO = ROOT_DIR / "scripts" / "run_rc_arm_mujoco.sh"
SCRIPT_RUN_MUJOCO_BRIDGE = ROOT_DIR / "scripts" / "run_rc_arm_mujoco_bridge.sh"
SCRIPT_RUN_REAL = ROOT_DIR / "scripts" / "run_rc_arm_real.sh"
AUTO_CLEANUP_WAIT_SEC = 1.0
WHEEL_CONTINUOUS_INTERVAL_MS = 50
PROJECT_ROS_CLEANUP_PATTERNS = (
    ("middleware", "arm2_middleware"),
    ("executor", "target_pose_moveit_executor.py"),
    ("tf bridge", "tf_target_pose_bridge.py"),
    ("payload sync", "payload_scene_sync.py"),
    ("move_group", "move_group"),
    ("robot_state_publisher", "robot_state_publisher"),
    ("static_transform_publisher", "static_transform_publisher"),
    ("ros2_control_node", "ros2_control_node"),
    ("real launch", "ros2 launch rc_arm_moveit_config rc_arm_2_robot.launch.py"),
)
REMOTE_SERVICE_ACTIONS = {
    "/rc_arm_2/remote/start_mujoco": "start_mujoco",
    "/rc_arm_2/remote/stop_mujoco": "stop_mujoco",
    "/rc_arm_2/remote/start_real": "start_real",
    "/rc_arm_2/remote/stop_real": "stop_real",
    "/rc_arm_2/remote/start_middleware": "start_middleware",
    "/rc_arm_2/remote/stop_middleware": "stop_middleware",
}
REMOTE_LOG_TOPIC = "/rc_arm_2/remote/log"
REMOTE_PROCESS_STATUS_TOPIC = "/rc_arm_2/remote/process_status"
REMOTE_REACHABILITY_REQUEST_TOPIC = "/rc_arm_2/remote/reachability_request"
REMOTE_REACHABILITY_RESULT_TOPIC = "/rc_arm_2/remote/reachability_result"
NEON_CONSOLE_STYLESHEET = """
QMainWindow {
    background: #030712;
}
QScrollArea {
    background: #030712;
    border: none;
}
QWidget#contentRoot {
    background: #030712;
}
QGroupBox {
    color: #72f8ff;
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 rgba(7, 18, 34, 245),
        stop:1 rgba(3, 10, 20, 250));
    border: 1px solid rgba(0, 234, 255, 95);
    border-radius: 4px;
    margin-top: 18px;
    padding: 12px;
    font: 800 14px "Cascadia Code", "Liberation Mono", monospace;
    text-transform: uppercase;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 8px;
    color: #72f8ff;
    background: #030712;
}
QLabel {
    color: #eafcff;
    font-size: 15px;
}
QFormLayout QLabel {
    color: #9ab8c6;
}
QDoubleSpinBox, QSpinBox {
    color: #f2fdff;
    background: #020812;
    border: 1px solid rgba(0, 234, 255, 110);
    border-radius: 3px;
    min-height: 34px;
    padding: 4px 8px;
    font: 800 17px "Cascadia Code", "Liberation Mono", monospace;
}
QDoubleSpinBox:focus, QSpinBox:focus {
    border: 1px solid #ff2bf3;
}
QCheckBox {
    color: #9ab8c6;
    font-size: 15px;
    spacing: 8px;
}
QDial {
    background: rgba(255, 43, 243, 18);
}
QPushButton {
    min-height: 38px;
    color: #f2fdff;
    background: rgba(0, 234, 255, 22);
    border: 1px solid rgba(0, 234, 255, 110);
    border-radius: 3px;
    padding: 7px 10px;
    font: 800 13px "Segoe UI", Arial, sans-serif;
    text-transform: uppercase;
}
QPushButton:hover {
    border: 1px solid #00eaff;
    background: rgba(0, 234, 255, 42);
}
QPushButton:disabled {
    color: #536978;
    border-color: rgba(83, 105, 120, 80);
    background: rgba(83, 105, 120, 25);
}
QPushButton[role="primary"] {
    border: 1px solid #00eaff;
    background: rgba(0, 234, 255, 58);
}
QPushButton[role="safe"] {
    color: #deffe9;
    border: 1px solid #39ff88;
    background: rgba(57, 255, 136, 34);
}
QPushButton[role="danger"] {
    color: #ffe1e8;
    border: 1px solid #ff2f5f;
    background: rgba(255, 47, 95, 36);
}
QPushButton[role="warn"] {
    color: #fff0b6;
    border: 1px solid #ffd23f;
    background: rgba(255, 210, 63, 34);
}
QPushButton[role="magenta"] {
    border: 1px solid #ff2bf3;
    background: rgba(255, 43, 243, 34);
}
QPlainTextEdit#logView {
    color: #c8fbff;
    background: #02060c;
    border: 1px solid rgba(0, 234, 255, 115);
    border-radius: 3px;
    padding: 10px;
    font: 15px "Cascadia Code", "Liberation Mono", monospace;
}
"""


def set_button_role(button: QPushButton, role: str) -> None:
    button.setProperty("role", role)


def middleware_command() -> List[str]:
    action_sets_file = RC_MOVEIT_DIR / "rc_arm2_middleware" / "config" / "action_sets.yaml"
    command = (
        f"cd {shlex.quote(str(RC_MOVEIT_DIR))} && "
        "source /opt/ros/humble/setup.bash && "
        "source install/setup.bash && "
        "ros2 run rc_arm2_middleware arm2_middleware --ros-args "
        f"-p action_sets_file:={shlex.quote(str(action_sets_file))}"
    )
    return ["bash", "-lc", command]


@dataclass
class TargetState:
    x: float
    y: float
    z: float
    j4_rad: float

    def to_display(self) -> str:
        return "x={:.4f} y={:.4f} z={:.4f} j4 world={:.2f} deg ({:.4f} rad)".format(
            self.x, self.y, self.z, math.degrees(self.j4_rad), self.j4_rad
        )

    def almost_equal(self, other: "TargetState", tol: float = 1.0e-6) -> bool:
        return (
            abs(self.x - other.x) <= tol
            and abs(self.y - other.y) <= tol
            and abs(self.z - other.z) <= tol
            and abs(self.j4_rad - other.j4_rad) <= tol
        )


@dataclass
class ActualPose:
    x: float
    y: float
    z: float
    world_pitch_rad: Optional[float]


def normalize_frame_id(frame_id: str) -> str:
    return (frame_id or "").strip().lstrip("/")

class RosBackend(QObject):
    actual_pose_updated = Signal(object)
    reachability_updated = Signal(object)
    payload_state_updated = Signal(bool)
    j5_position_updated = Signal(object)
    last_sent_updated = Signal(object)
    last_send_status = Signal(str)
    last_vacuum_status = Signal(str)
    last_j5_status = Signal(str)
    payload_command_status = Signal(str)
    last_middleware_status = Signal(str)
    backend_error = Signal(str)
    remote_control_requested = Signal(str)

    def __init__(self, args) -> None:
        super().__init__()
        self._args = args
        self._kinematics = RcArmWorldPitchKinematics(
            urdf_path=args.urdf_path or None,
            j4_axis=args.j4_axis,
        )
        self._node: Optional[Node] = None
        self._tf_pub = None
        self._vacuum_pub = None
        self._payload_command_pub = None
        self._middleware_run_pub = None
        self._j5_command_pub = None
        self._payload_sub = None
        self._j5_position_sub = None
        self._joint_state_sub = None
        self._remote_reachability_sub = None
        self._remote_log_pub = None
        self._remote_process_status_pub = None
        self._remote_reachability_result_pub = None
        self._remote_services = []
        self._tf_buffer = None
        self._tf_listener = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_sent: Optional[TargetState] = None
        self._pending_send: Optional[Tuple[TargetState, bool]] = None
        self._pending_vacuum: Optional[bool] = None
        self._pending_payload_active: Optional[bool] = None
        self._pending_middleware_command: Optional[int] = None
        self._pending_j5_command: Optional[float] = None
        self._pending_reachability: Optional[TargetState] = None
        self._last_actual_emit = 0.0
        self._last_middleware_sent: Optional[int] = None
        self._last_middleware_sent_time = 0.0
        self._latest_joint_map: Dict[str, float] = {}
        self._last_solver_solution: Optional[Dict[str, float]] = None

    def start(self) -> None:
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = Node("rc_arm_tf_target_gui_publisher")
        self._tf_pub = self._node.create_publisher(TFMessage, self._args.tf_topic, 10)
        self._vacuum_pub = self._node.create_publisher(Bool, self._args.vacuum_topic, 10)
        self._payload_command_pub = self._node.create_publisher(Bool, self._args.payload_command_topic, 10)
        self._j5_command_pub = self._node.create_publisher(Float64, self._args.j5_command_topic, 10)
        self._middleware_run_pub = self._node.create_publisher(
            Int32, self._args.middleware_run_action_set_topic, 10
        )
        self._payload_sub = self._node.create_subscription(
            Bool, self._args.payload_active_topic, self._on_payload_state, 10
        )
        self._j5_position_sub = self._node.create_subscription(
            Float64, self._args.j5_position_topic, self._on_j5_position, 20
        )
        self._joint_state_sub = self._node.create_subscription(
            JointState, self._args.joint_state_topic, self._on_joint_state, 20
        )
        self._remote_log_pub = self._node.create_publisher(String, REMOTE_LOG_TOPIC, 50)
        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._remote_process_status_pub = self._node.create_publisher(
            String, REMOTE_PROCESS_STATUS_TOPIC, status_qos
        )
        self._remote_reachability_result_pub = self._node.create_publisher(
            String, REMOTE_REACHABILITY_RESULT_TOPIC, 10
        )
        self._remote_reachability_sub = self._node.create_subscription(String, REMOTE_REACHABILITY_REQUEST_TOPIC, self._on_remote_reachability_request, 10)
        self._remote_services = [
            self._node.create_service(Trigger, service_name, self._make_remote_trigger_handler(action))
            for service_name, action in REMOTE_SERVICE_ACTIONS.items()
        ]
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node, spin_thread=False)
        self._thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if rclpy.ok():
            rclpy.shutdown()

    @Slot(object, bool)
    def queue_send_target(self, state: object, changed_only: bool) -> None:
        with self._lock:
            self._pending_send = (state, changed_only)

    @Slot(bool)
    def queue_vacuum(self, enabled: bool) -> None:
        with self._lock:
            self._pending_vacuum = enabled

    @Slot(bool)
    def queue_payload_active(self, enabled: bool) -> None:
        with self._lock:
            self._pending_payload_active = enabled

    @Slot(float)
    def queue_j5_command(self, position_m: float) -> None:
        with self._lock:
            self._pending_j5_command = float(position_m)

    @Slot(object)
    def queue_reachability(self, state: object) -> None:
        with self._lock:
            self._pending_reachability = state

    @Slot(int)
    def queue_run_action_set(self, action_set_id: int) -> None:
        with self._lock:
            self._pending_middleware_command = int(action_set_id)

    def _on_payload_state(self, msg: Bool) -> None:
        self.payload_state_updated.emit(bool(msg.data))

    def _on_j5_position(self, msg: Float64) -> None:
        self.j5_position_updated.emit(float(msg.data))

    def _on_joint_state(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        mapping = self._latest_joint_map.copy()
        for idx, name in enumerate(msg.name):
            if idx < len(msg.position):
                mapping[name] = float(msg.position[idx])
        self._latest_joint_map = mapping

    def _make_remote_trigger_handler(
        self, action: str
    ) -> Callable[[object, object], object]:
        def handler(_request, response):
            self.remote_control_requested.emit(action)
            response.success = True
            response.message = "queued {}".format(action)
            self.publish_remote_log(
                "remote_service",
                "info",
                "queued {}".format(action),
            )
            return response

        return handler

    def publish_remote_log(self, source: str, level: str, text: str) -> None:
        if self._remote_log_pub is None:
            return
        msg = String()
        msg.data = json.dumps(
            {
                "stamp": time.time(),
                "source": str(source),
                "level": str(level),
                "text": str(text),
            },
            sort_keys=True,
        )
        self._remote_log_pub.publish(msg)

    def publish_process_status(self, status: Dict[str, str]) -> None:
        if self._remote_process_status_pub is None:
            return
        msg = String()
        payload = {"stamp": time.time()}
        payload.update(status)
        msg.data = json.dumps(payload, sort_keys=True)
        self._remote_process_status_pub.publish(msg)

    def _on_remote_reachability_request(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            request_id = str(payload.get("request_id", ""))
            state = TargetState(
                x=float(payload["x"]),
                y=float(payload["y"]),
                z=float(payload["z"]),
                j4_rad=float(payload["j4_rad"]),
            )
            report = self._compute_reachability(state)
            result = {
                "request_id": request_id,
                "reachable": bool(report["reachable"]),
                "status": str(report["status"]),
                "ranges": report["ranges"],
            }
        except Exception as exc:
            result = {
                "request_id": "",
                "reachable": False,
                "status": "Error",
                "ranges": {},
                "error": str(exc),
            }
        self._publish_remote_reachability_result(result)

    def _publish_remote_reachability_result(self, payload: Dict[str, object]) -> None:
        if self._remote_reachability_result_pub is None:
            return
        msg = String()
        result = {"stamp": time.time()}
        result.update(payload)
        msg.data = json.dumps(result, sort_keys=True)
        self._remote_reachability_result_pub.publish(msg)

    def _spin_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                rclpy.spin_once(self._node, timeout_sec=0.05)
                self._refresh_actual_pose()
                self._flush_send_request()
                self._flush_vacuum_request()
                self._flush_payload_request()
                self._flush_j5_request()
                self._flush_middleware_request()
                self._flush_reachability_request()
            except Exception as exc:  # pragma: no cover
                self.backend_error.emit(str(exc))
                time.sleep(0.1)

    def _refresh_actual_pose(self) -> None:
        if self._tf_buffer is None or self._node is None:
            return
        now = time.time()
        if now - self._last_actual_emit < 0.2:
            return
        self._last_actual_emit = now
        try:
            trans = self._tf_buffer.lookup_transform(
                normalize_frame_id(self._args.current_pose_parent_frame),
                normalize_frame_id(self._args.current_pose_child_frame),
                rclpy.time.Time(),
            )
        except Exception:
            return

        world_pitch = None
        if all(name in self._latest_joint_map for name in self._kinematics.zero_home_joint_map()):
            try:
                world_pitch = self._kinematics.forward_world_pitch(self._latest_joint_map)
            except Exception:
                world_pitch = None

        actual = ActualPose(
            x=float(trans.transform.translation.x),
            y=float(trans.transform.translation.y),
            z=float(trans.transform.translation.z),
            world_pitch_rad=world_pitch,
        )
        self.actual_pose_updated.emit(actual)

    def _flush_send_request(self) -> None:
        with self._lock:
            pending = self._pending_send
            self._pending_send = None
        if pending is None or self._node is None:
            return

        state, changed_only = pending
        if changed_only and self._last_sent is not None and state.almost_equal(self._last_sent):
            self.last_send_status.emit("skipped: unchanged target")
            return

        qx, qy, qz, qw = self._kinematics.quaternion_from_world_pitch(state.j4_rad)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self._node.get_clock().now().to_msg()
        tf_msg.header.frame_id = normalize_frame_id(self._args.parent_frame)
        tf_msg.child_frame_id = normalize_frame_id(self._args.child_frame)
        tf_msg.transform.translation.x = state.x
        tf_msg.transform.translation.y = state.y
        tf_msg.transform.translation.z = state.z
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._tf_pub.publish(TFMessage(transforms=[tf_msg]))
        self._last_sent = TargetState(state.x, state.y, state.z, state.j4_rad)
        self.last_sent_updated.emit(self._last_sent)
        self.last_send_status.emit(
            "published {} -> {} at {:.0f}".format(
                normalize_frame_id(self._args.parent_frame),
                normalize_frame_id(self._args.child_frame),
                time.time(),
            )
        )

    def _flush_vacuum_request(self) -> None:
        with self._lock:
            pending = self._pending_vacuum
            self._pending_vacuum = None
        if pending is None:
            return
        msg = Bool()
        msg.data = bool(pending)
        self._vacuum_pub.publish(msg)
        self.last_vacuum_status.emit("ON" if pending else "OFF")

    def _flush_payload_request(self) -> None:
        with self._lock:
            pending = self._pending_payload_active
            self._pending_payload_active = None
        if pending is None or self._payload_command_pub is None:
            return
        msg = Bool()
        msg.data = bool(pending)
        self._payload_command_pub.publish(msg)
        self.payload_command_status.emit("payload command: " + ("ON" if pending else "OFF"))

    def _flush_j5_request(self) -> None:
        with self._lock:
            pending = self._pending_j5_command
            self._pending_j5_command = None
        if pending is None or self._j5_command_pub is None:
            return
        msg = Float64()
        msg.data = float(pending)
        self._j5_command_pub.publish(msg)
        self.last_j5_status.emit(
            "published J5 target={:.4f} m to {}".format(
                float(pending),
                self._args.j5_command_topic,
            )
        )

    def _flush_middleware_request(self) -> None:
        with self._lock:
            pending = self._pending_middleware_command
            self._pending_middleware_command = None
        if pending is None or self._middleware_run_pub is None:
            return

        action_set_id = pending
        now = time.monotonic()
        if (
            self._last_middleware_sent is not None
            and self._last_middleware_sent == action_set_id
            and now - self._last_middleware_sent_time < 0.75
        ):
            self.last_middleware_status.emit(
                "skipped duplicate middleware command action_set={} within 0.75s".format(
                    action_set_id
                )
            )
            return

        run_msg = Int32()
        run_msg.data = int(action_set_id)
        self._middleware_run_pub.publish(run_msg)
        self._last_middleware_sent = action_set_id
        self._last_middleware_sent_time = now

        # A middleware run can leave the manual ghost target unchanged but still
        # semantically "new" from the user's perspective. Clearing the manual
        # dedupe allows the next manual send to re-publish the same ghost pose.
        self._last_sent = None
        self.last_sent_updated.emit(None)

        self.last_middleware_status.emit(
            "published action_set={} to {}".format(
                action_set_id,
                self._args.middleware_run_action_set_topic,
            )
        )

    def _flush_reachability_request(self) -> None:
        with self._lock:
            pending = self._pending_reachability
            self._pending_reachability = None
        if pending is None:
            return
        report = self._compute_reachability(pending)
        self.reachability_updated.emit(report)

    def _current_seed_joints(self) -> Optional[Dict[str, float]]:
        needed = self._kinematics.zero_home_joint_map().keys()
        if not all(name in self._latest_joint_map for name in needed):
            return self._last_solver_solution
        return {name: self._latest_joint_map[name] for name in needed}

    def _solve_state(
        self,
        state: TargetState,
        seed_joints: Optional[Dict[str, float]] = None,
    ) -> Optional[Dict[str, float]]:
        seed = seed_joints or self._current_seed_joints()
        solution = self._kinematics.solve_xyz_pitch(
            state.x,
            state.y,
            state.z,
            state.j4_rad,
            seed_joints=seed,
        )
        if solution is not None:
            self._last_solver_solution = dict(solution)
        return solution

    def _estimate_axis_range(
        self,
        state: TargetState,
        axis: str,
        coarse_step: float,
        search_limit: float,
        center_solution: Optional[Dict[str, float]],
    ) -> Tuple[float, float]:
        def with_axis(value: float) -> TargetState:
            if axis == "x":
                return TargetState(value, state.y, state.z, state.j4_rad)
            if axis == "y":
                return TargetState(state.x, value, state.z, state.j4_rad)
            return TargetState(state.x, state.y, value, state.j4_rad)

        center = getattr(state, axis)
        lo = center
        hi = center

        def search_direction(direction: float) -> Tuple[float, Optional[Dict[str, float]]]:
            success_value = center
            success_solution = dict(center_solution) if center_solution is not None else None
            failure_value: Optional[float] = None
            probe = center
            while abs(probe - center) <= search_limit:
                candidate = probe + direction * coarse_step
                if abs(candidate - center) > search_limit:
                    break
                candidate_state = with_axis(candidate)
                candidate_solution = self._solve_state(candidate_state, success_solution)
                if candidate_solution is None:
                    failure_value = candidate
                    break
                probe = candidate
                success_value = candidate
                success_solution = candidate_solution

            if failure_value is None:
                return success_value, success_solution

            best_value = success_value
            best_solution = success_solution
            if direction > 0.0:
                reachable = success_value
                unreachable = failure_value
                for _ in range(8):
                    mid = 0.5 * (reachable + unreachable)
                    mid_state = with_axis(mid)
                    mid_solution = self._solve_state(mid_state, best_solution)
                    if mid_solution is not None:
                        best_value = mid
                        best_solution = mid_solution
                        reachable = mid
                    else:
                        unreachable = mid
            else:
                unreachable = failure_value
                reachable = success_value
                for _ in range(8):
                    mid = 0.5 * (unreachable + reachable)
                    mid_state = with_axis(mid)
                    mid_solution = self._solve_state(mid_state, best_solution)
                    if mid_solution is not None:
                        best_value = mid
                        best_solution = mid_solution
                        reachable = mid
                    else:
                        unreachable = mid
            return best_value, best_solution

        neg_bound, _ = search_direction(-1.0)
        pos_bound, _ = search_direction(1.0)
        lo = neg_bound
        hi = pos_bound

        return (lo, hi)

    def _compute_reachability(self, state: TargetState) -> dict:
        center_solution = self._solve_state(state)
        reachable = center_solution is not None
        coarse_step = max(0.005, min(0.03, float(self._args.reachability_step)))
        ranges = {
            "x": self._estimate_axis_range(state, "x", coarse_step, 0.30, center_solution),
            "y": self._estimate_axis_range(state, "y", coarse_step, 0.30, center_solution),
            "z": self._estimate_axis_range(state, "z", coarse_step, 0.30, center_solution),
        }

        min_margin = min(
            state.x - ranges["x"][0],
            ranges["x"][1] - state.x,
            state.y - ranges["y"][0],
            ranges["y"][1] - state.y,
            state.z - ranges["z"][0],
            ranges["z"][1] - state.z,
        )
        if not reachable:
            status = "Unreachable"
        elif min_margin < coarse_step * 1.2:
            status = "Near limit"
        else:
            status = "Reachable"
        return {"status": status, "ranges": ranges, "reachable": reachable}


class ControlProcess(QObject):
    log_line = Signal(str)
    state_changed = Signal(str)

    def __init__(self, command: List[str], label: str) -> None:
        super().__init__()
        self._command = command
        self._label = label
        self._process = QProcess()
        self._process.setProcessChannelMode(QProcess.MergedChannels)
        self._process.readyReadStandardOutput.connect(self._read_output)
        self._process.stateChanged.connect(self._on_state_changed)
        self._process.finished.connect(self._on_finished)

    def is_running(self) -> bool:
        return self._process.state() != QProcess.NotRunning

    def start(self) -> None:
        if self.is_running():
            return
        self.log_line.emit("{} start: {}".format(self._label, " ".join(self._command)))
        self._process.start(self._command[0], self._command[1:])

    def stop(self) -> None:
        if not self.is_running():
            return
        self.log_line.emit("{} stop requested".format(self._label))
        self._process.terminate()
        if not self._process.waitForFinished(3000):
            self._process.kill()

    def _read_output(self) -> None:
        text = bytes(self._process.readAllStandardOutput()).decode("utf-8", errors="replace")
        if text.strip():
            self.log_line.emit("[{}] {}".format(self._label, text.rstrip()))

    def _on_state_changed(self, state) -> None:
        mapping = {
            QProcess.NotRunning: "stopped",
            QProcess.Starting: "starting",
            QProcess.Running: "running",
        }
        self.state_changed.emit("{}: {}".format(self._label, mapping.get(state, "unknown")))

    def _on_finished(self, exit_code: int, exit_status) -> None:
        self.log_line.emit("{} exited code={} status={}".format(self._label, exit_code, exit_status.value))


class TargetPublisherWindow(QMainWindow):
    def __init__(self, args) -> None:
        super().__init__()
        self._args = args
        self._kinematics = RcArmWorldPitchKinematics(
            urdf_path=args.urdf_path or None,
            j4_axis=args.j4_axis,
        )
        j4_lower, j4_upper = self._kinematics.joint_limit("j4_joint")
        self._j4_world_min_deg = math.degrees(j4_lower)
        self._j4_world_max_deg = math.degrees(j4_upper)
        home_x, home_y, home_z, home_pitch = self._kinematics.zero_home_pose()
        self._editing_target = TargetState(home_x, home_y, home_z, home_pitch)
        self._last_sent: Optional[TargetState] = None
        self._actual_pose: Optional[ActualPose] = None
        self._home_target: Optional[TargetState] = TargetState(home_x, home_y, home_z, home_pitch)
        self._payload_active = False
        self._latest_j5_position: Optional[float] = None
        self._reachability = None
        self._editing_dirty = False
        self._actual_pose_ready = False
        self._syncing_editor = False
        self._wheel_drag_origin: Dict[str, float] = {}
        self._shutdown_started = False

        self.setWindowTitle("RC Arm TF Target Publisher")
        self.setMinimumSize(1280, 760)
        self.resize(1600, 920)
        self.setStyleSheet(NEON_CONSOLE_STYLESHEET)

        self._backend = RosBackend(args)
        self._backend.actual_pose_updated.connect(self._on_actual_pose)
        self._backend.reachability_updated.connect(self._on_reachability)
        self._backend.payload_state_updated.connect(self._on_payload_state)
        self._backend.j5_position_updated.connect(self._on_j5_position)
        self._backend.last_sent_updated.connect(self._on_last_sent)
        self._backend.last_send_status.connect(self._set_send_status)
        self._backend.last_vacuum_status.connect(self._set_vacuum_status)
        self._backend.last_j5_status.connect(self._set_j5_status)
        self._backend.payload_command_status.connect(self._append_log)
        self._backend.last_middleware_status.connect(self._set_middleware_status)
        self._backend.backend_error.connect(self._append_log)
        self._backend.remote_control_requested.connect(self._on_remote_control_requested)

        self._mujoco_stack = ControlProcess(["bash", str(SCRIPT_RUN_MUJOCO)], "MuJoCo stack")
        self._mujoco_bridge = ControlProcess(["bash", str(SCRIPT_RUN_MUJOCO_BRIDGE)], "MuJoCo bridge")
        self._real_stack = ControlProcess(["bash", str(SCRIPT_RUN_REAL)], "Real stack")
        self._middleware_stack = ControlProcess(middleware_command(), "Middleware")
        for proc in (self._mujoco_stack, self._mujoco_bridge, self._real_stack, self._middleware_stack):
            proc.log_line.connect(self._append_log)
            proc.state_changed.connect(self._on_process_state_changed)

        self._reachability_timer = QTimer(self)
        self._reachability_timer.setInterval(300)
        self._reachability_timer.setSingleShot(True)
        self._reachability_timer.timeout.connect(self._request_reachability)
        self._wheel_send_timer = QTimer(self)
        self._wheel_send_timer.setInterval(WHEEL_CONTINUOUS_INTERVAL_MS)
        self._wheel_send_timer.timeout.connect(self._on_wheel_send_timer)

        self._build_ui()
        self._install_shortcuts()
        self._sync_editing_widgets()
        self._update_status_labels()
        self._startup_cleanup_project_ros_processes()
        self._backend.start()
        self._refresh_process_buttons()
        self._request_reachability()

    def _ros2_env_command(self, ros2_args: List[str]) -> List[str]:
        command = (
            "source /opt/ros/humble/setup.bash && "
            f"source {shlex.quote(str(RC_MOVEIT_DIR / 'install' / 'setup.bash'))} && "
            + " ".join(shlex.quote(part) for part in ros2_args)
        )
        return ["bash", "-lc", command]

    def _list_ros_nodes(self) -> Optional[List[str]]:
        try:
            completed = subprocess.run(
                self._ros2_env_command(["ros2", "node", "list"]),
                check=False,
                capture_output=True,
                text=True,
                timeout=5.0,
            )
        except Exception as exc:
            self._append_log(f"ros2 node list failed: {exc}")
            return None

        if completed.returncode != 0:
            stderr = completed.stderr.strip() or completed.stdout.strip() or "unknown error"
            self._append_log(f"ros2 node list failed: {stderr}")
            return None

        return [line.strip() for line in completed.stdout.splitlines() if line.strip().startswith("/")]

    def _get_node_counts(self) -> Optional[collections.Counter]:
        nodes = self._list_ros_nodes()
        if nodes is None:
            return None
        return collections.Counter(nodes)

    def _find_duplicate_nodes(self, node_names: List[str]) -> Optional[List[Tuple[str, int]]]:
        counts = self._get_node_counts()
        if counts is None:
            return None
        return [(name, counts[name]) for name in node_names if counts[name] > 1]

    def _show_duplicate_nodes_warning(
        self,
        title: str,
        duplicates: List[Tuple[str, int]],
        auto_cleanup_attempted: bool,
    ) -> None:
        lines = ["Detected duplicate ROS nodes:"]
        for name, count in sorted(duplicates):
            lines.append(f"{name} x{count}")
        if auto_cleanup_attempted:
            lines.append("Auto cleanup ran, but duplicate ROS nodes remain.")
        else:
            lines.append("Stop old stacks before starting or running a new one.")
        message = "\n".join(lines)
        QMessageBox.warning(self, title, message)
        self._append_log(message)

    def _check_required_single_nodes(
        self,
        node_names: List[str],
        title: str,
        blocked_status: Optional[QLabel] = None,
    ) -> bool:
        counts = self._get_node_counts()
        if counts is None:
            if blocked_status is not None:
                blocked_status.setText("blocked: unable to inspect ROS nodes")
            return False

        missing = [name for name in node_names if counts[name] != 1]
        if not missing:
            return True

        message = "Required ROS nodes are not ready:\n" + "\n".join(missing)
        QMessageBox.warning(self, title, message)
        self._append_log(message)
        if blocked_status is not None:
            blocked_status.setText("blocked: required ROS nodes not ready")
        return False

    def _cleanup_project_ros_processes(
        self,
        prefix: str = "auto cleanup",
        announce: Optional[str] = "duplicate ROS nodes detected",
    ) -> bool:
        if announce:
            self._append_log(f"{prefix}: {announce}")
        overall_ok = True
        for label, pattern in PROJECT_ROS_CLEANUP_PATTERNS:
            try:
                completed = subprocess.run(
                    ["pkill", "-f", pattern],
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=5.0,
                )
            except Exception as exc:
                self._append_log(f"{prefix}: failed to stop {label}: {exc}")
                overall_ok = False
                continue

            if completed.returncode == 0:
                self._append_log(f"{prefix}: stopped {label}")
            elif completed.returncode == 1:
                continue
            else:
                stderr = completed.stderr.strip() or completed.stdout.strip() or "unknown error"
                self._append_log(
                    f"{prefix}: failed to stop {label} (code={completed.returncode}): {stderr}"
                )
                overall_ok = False

        time.sleep(AUTO_CLEANUP_WAIT_SEC)
        return overall_ok

    def _startup_cleanup_project_ros_processes(self) -> None:
        self._cleanup_project_ros_processes(
            prefix="startup cleanup",
            announce="clearing project ROS processes",
        )

    def _auto_cleanup_ros_duplicates(
        self,
        node_names: List[str],
        title: str,
        blocked_status: Optional[QLabel] = None,
        required_nodes: Optional[List[str]] = None,
    ) -> bool:
        duplicates = self._find_duplicate_nodes(node_names)
        if duplicates is None:
            if blocked_status is not None:
                blocked_status.setText("blocked: unable to inspect ROS nodes")
            return False
        if not duplicates:
            return True

        self._cleanup_project_ros_processes()
        duplicates_after = self._find_duplicate_nodes(node_names)
        if duplicates_after is None:
            if blocked_status is not None:
                blocked_status.setText("blocked: unable to inspect ROS nodes after cleanup")
            return False
        if duplicates_after:
            if blocked_status is not None:
                blocked_status.setText("blocked: duplicate ROS nodes remain after cleanup")
            self._append_log("auto cleanup: duplicate ROS nodes remain")
            self._show_duplicate_nodes_warning(title, duplicates_after, auto_cleanup_attempted=True)
            return False

        if required_nodes:
            counts = self._get_node_counts()
            if counts is None:
                if blocked_status is not None:
                    blocked_status.setText("blocked: unable to inspect ROS nodes after cleanup")
                return False
            missing = [name for name in required_nodes if counts[name] != 1]
            if missing:
                message = (
                    "Auto cleanup cleared duplicate nodes, but required ROS nodes are not ready:\n"
                    + "\n".join(missing)
                )
                QMessageBox.warning(self, title, message)
                self._append_log(message)
                if blocked_status is not None:
                    blocked_status.setText("blocked: required ROS nodes not ready after cleanup")
                return False

        self._append_log("auto cleanup: cleared project ROS processes")
        return True

    def _build_ui(self) -> None:
        central = QWidget()
        central.setObjectName("contentRoot")
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)
        top = QHBoxLayout()
        bottom = QHBoxLayout()
        top.setSpacing(12)
        bottom.setSpacing(12)
        root.addLayout(top, stretch=3)
        root.addLayout(bottom, stretch=2)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setWidget(central)
        self.setCentralWidget(scroll)

        top.addWidget(self._build_target_editor(), stretch=5)
        top.addWidget(self._build_reachability_panel(), stretch=2)
        top.addWidget(self._build_system_panel(), stretch=3)
        bottom.addWidget(self._build_status_panel(), stretch=4)
        bottom.addWidget(self._build_log_panel(), stretch=6)

    def _build_target_editor(self) -> QWidget:
        box = QGroupBox("Target Editor")
        layout = QGridLayout(box)

        self._xyz_step_spin = QDoubleSpinBox()
        self._xyz_step_spin.setDecimals(3)
        self._xyz_step_spin.setRange(0.001, 1.0)
        self._xyz_step_spin.setValue(0.1)
        self._wheel_send_rate_spin = QDoubleSpinBox()
        self._wheel_send_rate_spin.setDecimals(1)
        self._wheel_send_rate_spin.setRange(1.0, 60.0)
        self._wheel_send_rate_spin.setValue(1000.0 / WHEEL_CONTINUOUS_INTERVAL_MS)
        self._wheel_send_rate_spin.setSuffix(" Hz")
        self._wheel_send_rate_spin.valueChanged.connect(self._on_wheel_send_rate_changed)
        self._wheel_continuous_send_check = QCheckBox("Continuous send")
        self._wheel_continuous_send_check.setChecked(True)
        self._wheel_continuous_send_check.toggled.connect(self._on_wheel_continuous_toggled)
        self._j4_step_spin = QDoubleSpinBox()
        self._j4_step_spin.setDecimals(1)
        self._j4_step_spin.setRange(0.1, 180.0)
        self._j4_step_spin.setValue(5.0)
        self._j5_target_spin = QDoubleSpinBox()
        self._j5_target_spin.setDecimals(4)
        self._j5_target_spin.setRange(-10.0, 10.0)
        self._j5_target_spin.setSingleStep(0.001)
        self._j5_target_spin.setSuffix(" m")
        self._send_j5_btn = QPushButton("Send J5")
        self._j5_use_actual_btn = QPushButton("Use actual")
        set_button_role(self._send_j5_btn, "magenta")
        self._send_j5_btn.clicked.connect(self._send_j5_command)
        self._j5_use_actual_btn.clicked.connect(self._use_actual_j5)

        self._field_spins = {}
        self._axis_wheels = {}
        self._axis_wheel_delta_labels = {}
        rows = [
            ("x", "x", "m"),
            ("y", "y", "m"),
            ("z", "z", "m"),
            ("j4", "j4 world", "deg"),
        ]
        for row, (field_key, display_name, unit) in enumerate(rows):
            label = QLabel(f"{display_name} ({unit})")
            spin = QDoubleSpinBox()
            spin.setDecimals(4 if field_key != "j4" else 2)
            if field_key == "j4":
                spin.setRange(self._j4_world_min_deg, self._j4_world_max_deg)
            else:
                spin.setRange(-10.0, 10.0)
            spin.valueChanged.connect(self._on_editing_changed)
            minus = QPushButton("-")
            plus = QPushButton("+")
            minus.clicked.connect(lambda _=False, axis=field_key: self._step_axis(axis, -1.0))
            plus.clicked.connect(lambda _=False, axis=field_key: self._step_axis(axis, 1.0))
            layout.addWidget(label, row, 0)
            layout.addWidget(minus, row, 1)
            layout.addWidget(spin, row, 2)
            layout.addWidget(plus, row, 3)
            if field_key in ("x", "y", "z"):
                dial = QDial()
                dial.setRange(-20, 20)
                dial.setValue(0)
                dial.setNotchesVisible(True)
                dial.setWrapping(False)
                dial.sliderPressed.connect(lambda axis=field_key: self._on_axis_wheel_pressed(axis))
                dial.valueChanged.connect(
                    lambda value, axis=field_key: self._on_axis_wheel_changed(axis, value)
                )
                dial.sliderReleased.connect(lambda axis=field_key: self._on_axis_wheel_released(axis))
                delta_label = QLabel("0.0000 m")
                layout.addWidget(dial, row, 4)
                layout.addWidget(delta_label, row, 5)
                self._axis_wheels[field_key] = dial
                self._axis_wheel_delta_labels[field_key] = delta_label
            self._field_spins[field_key] = spin

        layout.addWidget(QLabel("xyz step"), 4, 0)
        layout.addWidget(self._xyz_step_spin, 4, 2)
        layout.addWidget(QLabel("xyz wheel"), 4, 4)
        layout.addWidget(QLabel("j4 step"), 5, 0)
        layout.addWidget(self._j4_step_spin, 5, 2)
        layout.addWidget(QLabel("send rate"), 5, 4)
        layout.addWidget(self._wheel_send_rate_spin, 5, 5)
        layout.addWidget(self._wheel_continuous_send_check, 6, 4, 1, 2)
        layout.addWidget(QLabel("hold for continuous move"), 7, 4, 1, 2)
        layout.addWidget(QLabel("J5 target (m)"), 6, 0)
        layout.addWidget(self._j5_target_spin, 6, 2)
        layout.addWidget(self._send_j5_btn, 6, 3)
        layout.addWidget(self._j5_use_actual_btn, 7, 2, 1, 2)

        self._send_if_changed = QCheckBox("Send if changed only")
        self._send_if_changed.setChecked(True)

        send_btn = QPushButton("Send")
        set_button_role(send_btn, "primary")
        send_btn.clicked.connect(self._send_target)
        self._send_btn = send_btn
        reset_btn = QPushButton("Reset to current")
        reset_btn.clicked.connect(self._reset_to_current)
        self._reset_btn = reset_btn
        home_btn = QPushButton("Home")
        home_btn.clicked.connect(self._reset_to_home)
        self._home_btn = home_btn

        layout.addWidget(self._send_if_changed, 8, 0, 1, 4)
        layout.addWidget(send_btn, 9, 0, 1, 2)
        layout.addWidget(reset_btn, 9, 2)
        layout.addWidget(home_btn, 9, 3)
        return box

    def _build_reachability_panel(self) -> QWidget:
        box = QGroupBox("Reachability")
        layout = QFormLayout(box)
        self._reachability_label = QLabel("Unknown")
        self._range_labels = {
            "x": QLabel("NA"),
            "y": QLabel("NA"),
            "z": QLabel("NA"),
        }
        layout.addRow("Status", self._reachability_label)
        layout.addRow("x range", self._range_labels["x"])
        layout.addRow("y range", self._range_labels["y"])
        layout.addRow("z range", self._range_labels["z"])
        return box

    def _build_system_panel(self) -> QWidget:
        box = QGroupBox("System Control")
        layout = QVBoxLayout(box)
        self._start_mujoco_btn = QPushButton("Start MuJoCo")
        self._stop_mujoco_btn = QPushButton("Stop MuJoCo")
        self._start_real_btn = QPushButton("Start Real")
        self._stop_real_btn = QPushButton("Stop Real")
        self._start_middleware_btn = QPushButton("Start Middleware")
        self._stop_middleware_btn = QPushButton("Stop Middleware")
        self._action_set_spin = QSpinBox()
        self._action_set_spin.setRange(1, 999)
        self._action_set_spin.setValue(1)
        self._run_action_set_btn = QPushButton("Run Action Set")
        vacuum_on = QPushButton("Vacuum ON")
        vacuum_off = QPushButton("Vacuum OFF")
        payload_on = QPushButton("Payload ON")
        payload_off = QPushButton("Payload OFF")
        set_button_role(self._start_mujoco_btn, "safe")
        set_button_role(self._stop_mujoco_btn, "danger")
        set_button_role(self._start_real_btn, "safe")
        set_button_role(self._stop_real_btn, "danger")
        set_button_role(self._start_middleware_btn, "safe")
        set_button_role(self._stop_middleware_btn, "danger")
        set_button_role(self._run_action_set_btn, "primary")
        set_button_role(vacuum_on, "warn")
        set_button_role(payload_on, "warn")

        self._start_mujoco_btn.clicked.connect(self._start_mujoco)
        self._stop_mujoco_btn.clicked.connect(self._stop_mujoco)
        self._start_real_btn.clicked.connect(self._start_real)
        self._stop_real_btn.clicked.connect(self._stop_real)
        self._start_middleware_btn.clicked.connect(self._start_middleware)
        self._stop_middleware_btn.clicked.connect(self._stop_middleware)
        self._run_action_set_btn.clicked.connect(self._run_action_set)
        vacuum_on.clicked.connect(lambda: self._backend.queue_vacuum(True))
        vacuum_off.clicked.connect(lambda: self._backend.queue_vacuum(False))
        payload_on.clicked.connect(lambda: self._backend.queue_payload_active(True))
        payload_off.clicked.connect(lambda: self._backend.queue_payload_active(False))

        for widget in (
            self._start_mujoco_btn,
            self._stop_mujoco_btn,
            self._start_real_btn,
            self._stop_real_btn,
            self._start_middleware_btn,
            self._stop_middleware_btn,
            vacuum_on,
            vacuum_off,
            payload_on,
            payload_off,
        ):
            layout.addWidget(widget)
        action_row = QHBoxLayout()
        action_row.addWidget(QLabel("Action set id"))
        action_row.addWidget(self._action_set_spin, stretch=1)
        layout.addLayout(action_row)
        layout.addWidget(self._run_action_set_btn)
        layout.addStretch(1)
        return box

    def _build_status_panel(self) -> QWidget:
        box = QGroupBox("Status")
        layout = QFormLayout(box)
        self._actual_label = QLabel("NA")
        self._editing_label = QLabel("NA")
        self._last_sent_label = QLabel("NA")
        self._send_status_label = QLabel("idle")
        self._wheel_status_label = QLabel("idle")
        self._vacuum_status_label = QLabel("unknown")
        self._j5_actual_label = QLabel("waiting")
        self._j5_command_status_label = QLabel("idle")
        self._middleware_status_label = QLabel("idle")
        self._payload_status_label = QLabel("false")
        self._process_status_label = QLabel("all stopped")

        layout.addRow("Actual current pose", self._actual_label)
        layout.addRow("Editing target", self._editing_label)
        layout.addRow("Last sent target", self._last_sent_label)
        layout.addRow("Last send result", self._send_status_label)
        layout.addRow("XYZ wheel", self._wheel_status_label)
        layout.addRow("Last vacuum command", self._vacuum_status_label)
        layout.addRow("J5 actual (m)", self._j5_actual_label)
        layout.addRow("Last J5 command (m)", self._j5_command_status_label)
        layout.addRow("Last middleware command", self._middleware_status_label)
        layout.addRow("Payload active", self._payload_status_label)
        layout.addRow("Process status", self._process_status_label)
        return box

    def _build_log_panel(self) -> QWidget:
        box = QGroupBox("Log")
        layout = QVBoxLayout(box)
        self._log_view = QPlainTextEdit()
        self._log_view.setObjectName("logView")
        self._log_view.setReadOnly(True)
        self._log_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._log_view.setMinimumHeight(300)
        layout.addWidget(self._log_view)
        return box

    def _install_shortcuts(self) -> None:
        QShortcut(QKeySequence("Ctrl+Return"), self, activated=self._send_target)
        QShortcut(QKeySequence(Qt.Key_Left), self, activated=lambda: self._step_axis("x", -1.0))
        QShortcut(QKeySequence(Qt.Key_Right), self, activated=lambda: self._step_axis("x", 1.0))
        QShortcut(QKeySequence(Qt.Key_Down), self, activated=lambda: self._step_axis("y", -1.0))
        QShortcut(QKeySequence(Qt.Key_Up), self, activated=lambda: self._step_axis("y", 1.0))
        QShortcut(QKeySequence(Qt.Key_PageDown), self, activated=lambda: self._step_axis("z", -1.0))
        QShortcut(QKeySequence(Qt.Key_PageUp), self, activated=lambda: self._step_axis("z", 1.0))
        QShortcut(QKeySequence("["), self, activated=lambda: self._step_axis("j4", -1.0))
        QShortcut(QKeySequence("]"), self, activated=lambda: self._step_axis("j4", 1.0))
        QShortcut(QKeySequence("Ctrl+Shift+Return"), self, activated=self._run_action_set)

    def _step_axis(self, axis: str, direction: float) -> None:
        spin = self._field_spins[axis]
        step = self._xyz_step_spin.value() if axis != "j4" else self._j4_step_spin.value()
        spin.setValue(spin.value() + direction * step)

    def _sync_editing_widgets(self) -> None:
        self._syncing_editor = True
        try:
            self._field_spins["x"].setValue(self._editing_target.x)
            self._field_spins["y"].setValue(self._editing_target.y)
            self._field_spins["z"].setValue(self._editing_target.z)
            self._field_spins["j4"].setValue(math.degrees(self._editing_target.j4_rad))
        finally:
            self._syncing_editor = False
        self._update_status_labels()

    def _read_editing_target(self) -> TargetState:
        j4_deg = max(
            self._j4_world_min_deg,
            min(self._j4_world_max_deg, self._field_spins["j4"].value()),
        )
        return TargetState(
            x=self._field_spins["x"].value(),
            y=self._field_spins["y"].value(),
            z=self._field_spins["z"].value(),
            j4_rad=math.radians(j4_deg),
        )

    def _update_status_labels(self) -> None:
        self._editing_label.setText(self._editing_target.to_display())
        self._last_sent_label.setText(self._last_sent.to_display() if self._last_sent else "NA")
        if self._actual_pose is None:
            self._actual_label.setText("waiting for actual pose")
        elif self._actual_pose.world_pitch_rad is None:
            self._actual_label.setText(
                "x={:.4f} y={:.4f} z={:.4f} j4 world=waiting joint states".format(
                    self._actual_pose.x, self._actual_pose.y, self._actual_pose.z
                )
            )
        else:
            self._actual_label.setText(
                "x={:.4f} y={:.4f} z={:.4f} j4 world={:.2f} deg ({:.4f} rad)".format(
                    self._actual_pose.x,
                    self._actual_pose.y,
                    self._actual_pose.z,
                    math.degrees(self._actual_pose.world_pitch_rad),
                    self._actual_pose.world_pitch_rad,
                )
            )
        self._payload_status_label.setText("true" if self._payload_active else "false")
        self._reset_btn.setEnabled(self._actual_pose_ready and self._actual_pose is not None)
        self._home_btn.setEnabled(self._actual_pose_ready and self._home_target is not None)
        self._send_btn.setEnabled(self._actual_pose_ready)
        self._run_action_set_btn.setEnabled(self._actual_pose_ready)
        self._action_set_spin.setEnabled(self._actual_pose_ready)
        if self._wheel_drag_origin:
            active_axes = ",".join(sorted(self._wheel_drag_origin.keys()))
            mode = "continuous" if self._wheel_continuous_send_check.isChecked() else "step"
            self._wheel_status_label.setText(f"dragging {active_axes} ({mode})")
        else:
            self._wheel_status_label.setText("idle")

    def _request_reachability(self) -> None:
        if not self._actual_pose_ready:
            return
        self._backend.queue_reachability(self._editing_target)

    def _on_editing_changed(self) -> None:
        if self._syncing_editor:
            return
        self._editing_target = self._read_editing_target()
        self._editing_dirty = True
        self._update_status_labels()
        self._reachability_timer.start()

    def _wheel_send_period_sec(self) -> float:
        return 1.0 / max(1.0, self._wheel_send_rate_spin.value())

    def _axis_wheel_delta(self, axis: str) -> float:
        return float(self._axis_wheels[axis].value()) * self._xyz_step_spin.value()

    def _set_axis_wheel_delta_label(self, axis: str, delta: float) -> None:
        self._axis_wheel_delta_labels[axis].setText("{:+.4f} m".format(delta))

    def _publish_axis_wheel_target(self) -> None:
        if not self._actual_pose_ready:
            self._wheel_status_label.setText("blocked: waiting for actual pose")
            return
        self._editing_target = self._read_editing_target()
        self._backend.queue_send_target(self._editing_target, False)
        self._backend.queue_reachability(self._editing_target)

    def _on_axis_wheel_pressed(self, axis: str) -> None:
        self._wheel_drag_origin[axis] = self._field_spins[axis].value()
        self._set_axis_wheel_delta_label(axis, 0.0)
        if self._wheel_continuous_send_check.isChecked() and not self._wheel_send_timer.isActive():
            self._wheel_send_timer.setInterval(max(1, int(round(self._wheel_send_period_sec() * 1000.0))))
            self._wheel_send_timer.start()
        self._update_status_labels()

    def _on_axis_wheel_changed(self, axis: str, value: int) -> None:
        if axis not in self._wheel_drag_origin:
            self._set_axis_wheel_delta_label(axis, 0.0)
            return
        del value
        delta = self._axis_wheel_delta(axis)
        self._set_axis_wheel_delta_label(axis, delta)
        if self._wheel_continuous_send_check.isChecked():
            self._apply_axis_wheel_motion(force=True)
        else:
            if abs(delta) > 1.0e-9:
                self._field_spins[axis].setValue(self._field_spins[axis].value() + delta)
                self._publish_axis_wheel_target()

    def _on_axis_wheel_released(self, axis: str) -> None:
        self._wheel_drag_origin.pop(axis, None)
        dial = self._axis_wheels[axis]
        dial.blockSignals(True)
        dial.setValue(0)
        dial.blockSignals(False)
        self._set_axis_wheel_delta_label(axis, 0.0)
        if not self._wheel_drag_origin:
            self._wheel_send_timer.stop()
        self._update_status_labels()

    def _on_wheel_send_rate_changed(self, _value: float) -> None:
        self._wheel_send_timer.setInterval(max(1, int(round(self._wheel_send_period_sec() * 1000.0))))

    def _on_wheel_continuous_toggled(self, enabled: bool) -> None:
        if enabled:
            if self._wheel_drag_origin and not self._wheel_send_timer.isActive():
                self._wheel_send_timer.setInterval(max(1, int(round(self._wheel_send_period_sec() * 1000.0))))
                self._wheel_send_timer.start()
        else:
            self._wheel_send_timer.stop()
        self._update_status_labels()

    def _apply_axis_wheel_motion(self, force: bool = False) -> None:
        if not self._wheel_drag_origin:
            return

        moved_axes = []
        for axis in list(self._wheel_drag_origin.keys()):
            delta = self._axis_wheel_delta(axis)
            self._set_axis_wheel_delta_label(axis, delta)
            if abs(delta) <= 1.0e-9:
                continue
            self._field_spins[axis].setValue(self._field_spins[axis].value() + delta)
            moved_axes.append(axis)

        if moved_axes:
            self._publish_axis_wheel_target()

    def _on_wheel_send_timer(self) -> None:
        self._apply_axis_wheel_motion()

    def _send_target(self) -> None:
        self._editing_target = self._read_editing_target()
        if not self._validate_motion_target("Last send result"):
            return
        self._backend.queue_send_target(self._editing_target, self._send_if_changed.isChecked())

    def _run_action_set(self) -> None:
        duplicates = self._find_duplicate_nodes(
            ["/arm2_middleware", "/rc_arm_target_pose_moveit_executor"]
        )
        if duplicates is None:
            self._middleware_status_label.setText("blocked: unable to inspect ROS nodes")
            return
        if duplicates:
            self._middleware_status_label.setText("blocked: duplicate ROS nodes detected")
            self._show_duplicate_nodes_warning(
                "Duplicate ROS Nodes",
                duplicates,
                auto_cleanup_attempted=False,
            )
            return
        if not self._check_required_single_nodes(
            ["/arm2_middleware", "/rc_arm_target_pose_moveit_executor"],
            "ROS Nodes Not Ready",
            blocked_status=self._middleware_status_label,
        ):
            return
        action_set_id = self._action_set_spin.value()
        self._backend.queue_run_action_set(action_set_id)

    def _send_j5_command(self) -> None:
        target_m = self._j5_target_spin.value()
        self._backend.queue_j5_command(target_m)
        self._j5_command_status_label.setText("queued {:.4f}".format(target_m))

    def _use_actual_j5(self) -> None:
        if self._latest_j5_position is None:
            self._j5_command_status_label.setText("blocked: waiting for J5 actual")
            return
        self._j5_target_spin.setValue(self._latest_j5_position)

    def _validate_motion_target(self, label_name: str) -> bool:
        if not self._actual_pose_ready:
            text = "blocked: waiting for actual pose"
            if label_name == "Last middleware command":
                self._middleware_status_label.setText(text)
            else:
                self._send_status_label.setText(text)
            self._append_log(text)
            return False
        if not (
            math.radians(self._j4_world_min_deg) - 1.0e-9
            <= self._editing_target.j4_rad
            <= math.radians(self._j4_world_max_deg) + 1.0e-9
        ):
            text = "blocked: j4 world must stay within URDF limits [{:.1f}, {:.1f}] deg".format(
                self._j4_world_min_deg,
                self._j4_world_max_deg,
            )
            if label_name == "Last middleware command":
                self._middleware_status_label.setText(text)
            else:
                self._send_status_label.setText(text)
            self._append_log(text)
            QMessageBox.warning(
                self,
                "Invalid j4 World Range",
                "j4 world target must stay within URDF limits {:.1f} to {:.1f} deg.".format(
                    self._j4_world_min_deg,
                    self._j4_world_max_deg,
                ),
            )
            return False
        if self._reachability is not None and not bool(self._reachability.get("reachable", False)):
            text = "blocked: unreachable target"
            if label_name == "Last middleware command":
                self._middleware_status_label.setText(text)
            else:
                self._send_status_label.setText(text)
            self._append_log(
                "{} {}".format(
                    text,
                    self._editing_target.to_display(),
                )
            )
            QMessageBox.warning(
                self,
                "Unreachable Target",
                "Current target is unreachable for the world-pitch solver.\n"
                "Adjust xyz or reduce j4 world before sending.",
            )
            return False
        return True

    def _reset_to_current(self) -> None:
        if self._actual_pose is None or self._actual_pose.world_pitch_rad is None:
            return
        self._editing_target = TargetState(
            self._actual_pose.x,
            self._actual_pose.y,
            self._actual_pose.z,
            self._actual_pose.world_pitch_rad,
        )
        self._sync_editing_widgets()
        self._reachability_timer.start()

    def _reset_to_home(self) -> None:
        home = self._home_target
        self._editing_target = TargetState(home.x, home.y, home.z, home.j4_rad)
        self._sync_editing_widgets()
        self._reachability_timer.start()

    @Slot(object)
    def _on_actual_pose(self, state: object) -> None:
        self._actual_pose = state
        self._actual_pose_ready = True
        self._update_status_labels()

    @Slot(object)
    def _on_last_sent(self, state: object) -> None:
        self._last_sent = state
        self._update_status_labels()

    @Slot(object)
    def _on_reachability(self, report: object) -> None:
        self._reachability = report
        status = report["status"]
        self._reachability_label.setText(status)
        color = QColor("#1f7a1f" if status == "Reachable" else "#b57600" if status == "Near limit" else "#aa2222")
        self._reachability_label.setStyleSheet(f"color: {color.name()}; font-weight: 600;")
        for axis, label in self._range_labels.items():
            lo, hi = report["ranges"][axis]
            label.setText("[{:.3f}, {:.3f}]".format(lo, hi))

        for axis in ("x", "y", "z"):
            spin = self._field_spins[axis]
            lo, hi = report["ranges"][axis]
            value = getattr(self._editing_target, axis)
            style = ""
            if not report["reachable"] or value < lo - 1.0e-6 or value > hi + 1.0e-6:
                style = "background-color: #3a1111; color: #ffd0d0;"
            elif min(value - lo, hi - value) < self._xyz_step_spin.value():
                style = "background-color: #3b2d15; color: #ffe8b3;"
            spin.setStyleSheet(style)

    @Slot(bool)
    def _on_payload_state(self, active: bool) -> None:
        self._payload_active = active
        self._update_status_labels()

    @Slot(object)
    def _on_j5_position(self, position_m: object) -> None:
        self._latest_j5_position = float(position_m)
        self._j5_actual_label.setText("{:.4f}".format(self._latest_j5_position))

    @Slot(str)
    def _set_send_status(self, text: str) -> None:
        self._send_status_label.setText(text)
        self._append_log(text)

    @Slot(str)
    def _set_vacuum_status(self, text: str) -> None:
        self._vacuum_status_label.setText(text)
        self._append_log("vacuum command: " + text)

    @Slot(str)
    def _set_j5_status(self, text: str) -> None:
        self._j5_command_status_label.setText(text)
        self._append_log("J5 command: " + text)

    @Slot(str)
    def _set_middleware_status(self, text: str) -> None:
        self._middleware_status_label.setText(text)
        self._append_log(text)

    def _start_mujoco(self) -> None:
        if self._real_stack.is_running():
            QMessageBox.warning(self, "Mode Busy", "Real stack is running. Stop it first.")
            return
        self._mujoco_bridge.start()
        self._mujoco_stack.start()
        self._refresh_process_buttons()

    def _stop_mujoco(self) -> None:
        self._mujoco_stack.stop()
        self._mujoco_bridge.stop()
        self._refresh_process_buttons()

    def _start_real(self) -> None:
        if self._mujoco_stack.is_running() or self._mujoco_bridge.is_running():
            QMessageBox.warning(self, "Mode Busy", "MuJoCo mode is running. Stop it first.")
            return
        if not self._auto_cleanup_ros_duplicates(
            [
                "/move_group",
                "/rc_arm_target_pose_moveit_executor",
                "/rc_arm_tf_target_pose_bridge",
                "/rc_arm_payload_scene_sync",
                "/robot_state_publisher",
            ],
            "Duplicate ROS Nodes",
            blocked_status=self._process_status_label,
        ):
            return
        self._real_stack.start()
        self._refresh_process_buttons()

    def _stop_real(self) -> None:
        self._real_stack.stop()
        self._refresh_process_buttons()

    def _start_middleware(self) -> None:
        if not self._auto_cleanup_ros_duplicates(
            ["/arm2_middleware"],
            "Duplicate Middleware Nodes",
            blocked_status=self._process_status_label,
        ):
            return
        self._middleware_stack.start()
        self._refresh_process_buttons()

    def _stop_middleware(self) -> None:
        self._middleware_stack.stop()
        self._refresh_process_buttons()

    @Slot(str)
    def _on_process_state_changed(self, _text: str) -> None:
        self._refresh_process_buttons()

    def _refresh_process_buttons(self) -> None:
        mujoco_running = self._mujoco_stack.is_running() or self._mujoco_bridge.is_running()
        real_running = self._real_stack.is_running()
        middleware_running = self._middleware_stack.is_running()
        self._start_mujoco_btn.setEnabled(not mujoco_running and not real_running)
        self._start_real_btn.setEnabled(not real_running and not mujoco_running)
        self._stop_mujoco_btn.setEnabled(mujoco_running)
        self._stop_real_btn.setEnabled(real_running)
        self._start_middleware_btn.setEnabled(not middleware_running)
        self._stop_middleware_btn.setEnabled(middleware_running)
        status_parts = []
        if mujoco_running:
            status_parts.append("MuJoCo running")
        if real_running:
            status_parts.append("Real running")
        if middleware_running:
            status_parts.append("Middleware running")
        text = ", ".join(status_parts) if status_parts else "all stopped"
        self._process_status_label.setText(text)
        self._backend.publish_process_status(
            {
                "mujoco": "running" if self._mujoco_stack.is_running() else "stopped",
                "mujoco_bridge": "running" if self._mujoco_bridge.is_running() else "stopped",
                "real": "running" if real_running else "stopped",
                "middleware": "running" if middleware_running else "stopped",
                "summary": text,
            }
        )

    @Slot(str)
    def _on_remote_control_requested(self, action: str) -> None:
        handlers = {
            "start_mujoco": self._start_mujoco,
            "stop_mujoco": self._stop_mujoco,
            "start_real": self._start_real,
            "stop_real": self._stop_real,
            "start_middleware": self._start_middleware,
            "stop_middleware": self._stop_middleware,
        }
        handler = handlers.get(action)
        if handler is None:
            self._append_log("remote control: unknown action {}".format(action))
            return
        self._append_log("remote control: {}".format(action))
        handler()

    @Slot(str)
    def _append_log(self, text: str) -> None:
        if not text:
            return
        self._log_view.appendPlainText(text)
        self._backend.publish_remote_log("host_gui", "info", text)

    def shutdown(self) -> None:
        if self._shutdown_started:
            return
        self._shutdown_started = True
        self._mujoco_stack.stop()
        self._mujoco_bridge.stop()
        self._real_stack.stop()
        self._middleware_stack.stop()
        self._backend.stop()

    def closeEvent(self, event) -> None:  # noqa: N802
        self.shutdown()
        super().closeEvent(event)


def parse_args():
    parser = argparse.ArgumentParser(description="PySide6 TF target publisher")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--parent-frame", default="world")
    parser.add_argument("--child-frame", default="rc_arm_2_target")
    parser.add_argument("--current-pose-parent-frame", default="world")
    parser.add_argument("--current-pose-child-frame", default="end_effector")
    parser.add_argument("--vacuum-topic", default="/rc_arm_2/vacuum_activate")
    parser.add_argument("--payload-command-topic", default="/rc_arm_2/payload_active_command")
    parser.add_argument("--payload-active-topic", default="/rc_arm_2/payload_active")
    parser.add_argument("--j5-command-topic", default="/rc_arm_2/j5/command_position")
    parser.add_argument("--j5-position-topic", default="/rc_arm_2/j5/actual_position")
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--middleware-target-topic", default="/arm2/middleware/target_point", help=argparse.SUPPRESS)
    parser.add_argument("--middleware-run-action-set-topic", default="/arm2/middleware/run_action_set")
    parser.add_argument("--urdf-path", default="")
    parser.add_argument("--j4-axis", choices=["x", "y", "z"], default="y")
    parser.add_argument("--reachability-step", type=float, default=0.05)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    app = QApplication(sys.argv)
    window = TargetPublisherWindow(args)
    app.aboutToQuit.connect(window.shutdown)
    signal.signal(signal.SIGINT, lambda *_args: app.quit())
    window.show()
    signal_timer = QTimer()
    signal_timer.start(200)
    signal_timer.timeout.connect(lambda: None)
    try:
        exit_code = app.exec()
    finally:
        window.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
