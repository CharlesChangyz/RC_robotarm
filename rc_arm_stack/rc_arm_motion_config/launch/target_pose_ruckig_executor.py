#!/usr/bin/env python3
"""Subscribe visual targets and stream IK + Ruckig joint references."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import threading
from typing import Dict, List, Optional, Sequence, Tuple

from arm_msgs.msg import Arm2MotionExecution, Arm2TargetPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tf2_ros
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml

from rc_arm_world_pitch_kinematics import RcArmWorldPitchKinematics, normalize_angle_rad

try:
    from ruckig import InputParameter, OutputParameter, Result, Ruckig
except ImportError as exc:  # pragma: no cover - explicit startup failure path
    InputParameter = None
    OutputParameter = None
    Result = None
    Ruckig = None
    RUCKIG_IMPORT_ERROR = exc
else:
    RUCKIG_IMPORT_ERROR = None


EXECUTION_ERROR_BUSY = -1
EXECUTION_ERROR_NOT_READY = -2
EXECUTION_ERROR_IK_FAILED = -3
EXECUTION_ERROR_JOINT_STATE_UNAVAILABLE = -4
EXECUTION_ERROR_LIMITS_LOAD_FAILED = -5
EXECUTION_ERROR_TARGET_OUT_OF_LIMITS = -6
EXECUTION_ERROR_TRAJECTORY_GENERATION_FAILED = -7

FEEDBACK_SYNC_MODES = {"init_only", "desync_only", "always"}
FEEDBACK_ACCEL_MODES = {"zero", "filtered"}


def _normalize_frame_id(frame_id: str) -> str:
    return (frame_id or "").strip().lstrip("/")


def _normalize_quat_xyzw(quat: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    norm = math.sqrt(sum(v * v for v in quat))
    if norm < 1.0e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(v / norm for v in quat)


def _quat_angle(q0: Tuple[float, float, float, float], q1: Tuple[float, float, float, float]) -> float:
    a = _normalize_quat_xyzw(q0)
    b = _normalize_quat_xyzw(q1)
    dot = abs(sum(x * y for x, y in zip(a, b)))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def _quaternion_from_world_pitch(axis: str, pitch_deg: float) -> Tuple[float, float, float, float]:
    angle = math.radians(float(pitch_deg))
    half = 0.5 * angle
    s = math.sin(half)
    c = math.cos(half)
    if axis == "x":
        return (s, 0.0, 0.0, c)
    if axis == "y":
        return (0.0, s, 0.0, c)
    return (0.0, 0.0, s, c)


def _copy_pose_stamped(msg: PoseStamped) -> PoseStamped:
    copied = PoseStamped()
    copied.header = msg.header
    copied.pose = msg.pose
    return copied


def _duration_from_seconds(seconds: float) -> Duration:
    clamped = max(0.0, float(seconds))
    sec = int(math.floor(clamped))
    nanosec = int(round((clamped - float(sec)) * 1.0e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    msg = Duration()
    msg.sec = sec
    msg.nanosec = nanosec
    return msg


def _stamp_to_sec(stamp) -> float:
    if stamp is None:
        return 0.0
    return float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) * 1.0e-9


def _parse_bool(text: str) -> bool:
    value = (text or "").strip().lower()
    return value in {"1", "true", "yes", "y", "on"}


@dataclass
class ExecutionRequest:
    source: str
    target: PoseStamped
    execution_id: Optional[int] = None


@dataclass(frozen=True)
class JointLimit:
    min_position: float
    max_position: float
    max_velocity: float
    max_acceleration: float
    max_jerk: float


class TargetPoseRuckigExecutor(Node):
    def __init__(
        self,
        target_topic: str,
        middleware_target_topic: str,
        middleware_result_topic: str,
        joint_names: List[str],
        default_frame: str,
        trajectory_topic: str,
        joint_limits_file: str,
        trajectory_sampling_period: float,
        pos_threshold: float,
        rot_threshold: float,
        check_period: float,
        j4_axis: str,
        joint_state_topic: str,
        urdf_path: str,
        status_log_period: float,
        status_base_frame: str,
        status_eef_frame: str,
        feedback_sync_mode: str,
        feedback_position_reset_threshold: float,
        feedback_position_reset_cycles: int,
        feedback_velocity_reset_enabled: bool,
        feedback_velocity_reset_threshold: float,
        feedback_velocity_filter_alpha: float,
        feedback_accel_mode: str,
    ) -> None:
        super().__init__("rc_arm_target_pose_executor")

        if RUCKIG_IMPORT_ERROR is not None:
            raise RuntimeError(f"ruckig import failed: {RUCKIG_IMPORT_ERROR}")

        self._manual_target_topic = target_topic
        self._middleware_target_topic = middleware_target_topic
        self._middleware_result_topic = middleware_result_topic
        self._joint_names = list(joint_names)
        self._default_frame = _normalize_frame_id(default_frame)
        self._trajectory_topic = trajectory_topic
        self._trajectory_sampling_period = max(0.001, float(trajectory_sampling_period))
        self._stream_period = max(0.001, float(check_period))
        self._pos_threshold = max(0.0, float(pos_threshold))
        self._rot_threshold = max(0.0, float(rot_threshold))
        self._success_velocity_tolerance = 0.25
        self._j4_axis = str(j4_axis).strip().lower() if str(j4_axis).strip() else "x"
        if self._j4_axis not in {"x", "y", "z"}:
            self._j4_axis = "x"
        self._status_log_period = max(0.0, float(status_log_period))
        self._status_base_frame = _normalize_frame_id(status_base_frame)
        self._status_eef_frame = _normalize_frame_id(status_eef_frame)
        self._feedback_sync_mode = str(feedback_sync_mode).strip().lower() or "desync_only"
        if self._feedback_sync_mode not in FEEDBACK_SYNC_MODES:
            self._feedback_sync_mode = "desync_only"
        self._feedback_position_reset_threshold = max(0.0, float(feedback_position_reset_threshold))
        self._feedback_position_reset_cycles = max(1, int(feedback_position_reset_cycles))
        self._feedback_velocity_reset_enabled = bool(feedback_velocity_reset_enabled)
        self._feedback_velocity_reset_threshold = max(0.0, float(feedback_velocity_reset_threshold))
        self._feedback_velocity_filter_alpha = max(0.0, min(1.0, float(feedback_velocity_filter_alpha)))
        self._feedback_accel_mode = str(feedback_accel_mode).strip().lower() or "zero"
        if self._feedback_accel_mode not in FEEDBACK_ACCEL_MODES:
            self._feedback_accel_mode = "zero"

        self._manual_target_lock = threading.Lock()
        self._latest_manual_target: Optional[PoseStamped] = None
        self._latest_manual_target_time_sec = 0.0
        self._active_manual_target: Optional[PoseStamped] = None

        self._ready_solver = False
        self._last_ready = None
        self._joint_state_map: Dict[str, Tuple[float, float]] = {}
        self._joint_state_time_sec = 0.0
        self._joint_state_accelerations: Dict[str, float] = {}
        self._next_execution_id = 0
        self._last_event = "init"
        self._last_event_time_sec = self._now_sec()
        self._active_request: Optional[ExecutionRequest] = None
        self._last_manual_failure_detail = ""
        self._last_manual_failure_time_sec = 0.0
        self._filtered_feedback_velocities: Optional[List[float]] = None
        self._last_feedback_velocities_for_accel: Optional[List[float]] = None
        self._last_feedback_time_sec = 0.0
        self._have_feedback_accel_estimate = False
        self._last_tracking_error_joint: Optional[str] = None
        self._last_tracking_error_value = 0.0
        self._last_tracking_error_log_time_sec = 0.0
        self._position_desync_cycles = 0
        self._velocity_desync_cycles = 0
        self._last_desync_planned_positions: Optional[List[float]] = None
        self._last_desync_planned_velocities: Optional[List[float]] = None

        self._joint_limits = self._load_joint_limits(joint_limits_file, self._joint_names)
        self._kinematics = RcArmWorldPitchKinematics(
            urdf_path=urdf_path or None,
            joint_names=self._joint_names,
            j4_axis=self._j4_axis,
        )
        self._otg = Ruckig(len(self._joint_names), self._trajectory_sampling_period)
        self._otg_input = InputParameter(len(self._joint_names))
        self._otg_output = OutputParameter(len(self._joint_names))
        self._otg_has_state = False
        self._otg_target_positions: Optional[List[float]] = None
        self._otg_last_sync_reason = "init"
        self._last_otg_reset_log_reason = ""
        self._last_otg_reset_log_time_sec = 0.0
        self._otg_plan_feedback_position_reset = self._feedback_position_reset_threshold
        self._otg_plan_feedback_velocity_reset = self._feedback_velocity_reset_threshold
        self._otg_input.max_velocity = [self._joint_limits[name].max_velocity for name in self._joint_names]
        self._otg_input.max_acceleration = [self._joint_limits[name].max_acceleration for name in self._joint_names]
        self._otg_input.max_jerk = [self._joint_limits[name].max_jerk for name in self._joint_names]

        self._trajectory_pub = self.create_publisher(
            JointTrajectory,
            trajectory_topic,
            20,
        )
        self._middleware_result_pub = self.create_publisher(
            Arm2MotionExecution,
            middleware_result_topic,
            20,
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(PoseStamped, target_topic, self._on_manual_target, 20)
        self.create_subscription(Arm2TargetPoint, middleware_target_topic, self._on_middleware_target, 20)
        self.create_subscription(JointState, joint_state_topic, self._on_joint_state, 20)
        self._timer = self.create_timer(self._stream_period, self._on_timer)
        if self._status_log_period > 0.0:
            self._status_timer = self.create_timer(self._status_log_period, self._log_status)

        self.get_logger().info(
            "TargetPose streaming executor started: manual_topic=%s middleware_target=%s "
            "middleware_result=%s trajectory_topic=%s joint_limits=%s status_tf=%s->%s "
            "feedback_sync_mode=%s pos_reset=%.3f/%d vel_reset=%d:%.3f vel_alpha=%.2f accel_mode=%s"
            % (
                self._manual_target_topic,
                self._middleware_target_topic,
                self._middleware_result_topic,
                self._trajectory_topic,
                joint_limits_file,
                self._status_base_frame,
                self._status_eef_frame,
                self._feedback_sync_mode,
                self._feedback_position_reset_threshold,
                self._feedback_position_reset_cycles,
                1 if self._feedback_velocity_reset_enabled else 0,
                self._feedback_velocity_reset_threshold,
                self._feedback_velocity_filter_alpha,
                self._feedback_accel_mode,
            )
        )

    def _load_joint_limits(
        self,
        path: str,
        joint_names: Sequence[str],
    ) -> Dict[str, JointLimit]:
        try:
            with open(path, "r", encoding="utf-8") as handle:
                loaded = yaml.safe_load(handle) or {}
        except Exception as exc:
            raise RuntimeError(f"failed to load joint limits file '{path}': {exc}") from exc

        raw_limits = loaded.get("joint_limits")
        if not isinstance(raw_limits, dict):
            raise RuntimeError(f"joint_limits mapping missing in '{path}'")

        limits: Dict[str, JointLimit] = {}
        for joint_name in joint_names:
            entry = raw_limits.get(joint_name)
            if not isinstance(entry, dict):
                raise RuntimeError(f"joint '{joint_name}' missing in '{path}'")
            try:
                limits[joint_name] = JointLimit(
                    min_position=float(entry["min_position"]),
                    max_position=float(entry["max_position"]),
                    max_velocity=float(entry["max_velocity"]),
                    max_acceleration=float(entry["max_acceleration"]),
                    max_jerk=float(entry["max_jerk"]),
                )
            except Exception as exc:
                raise RuntimeError(f"joint '{joint_name}' has invalid limits in '{path}': {exc}") from exc
        return limits

    def _on_joint_state(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        mapping = self._joint_state_map.copy()
        raw_velocity_map: Dict[str, float] = {}
        for idx, name in enumerate(msg.name):
            if idx >= len(msg.position):
                continue
            raw_velocity = 0.0
            if idx < len(msg.velocity):
                raw_velocity = float(msg.velocity[idx])
            raw_velocity_map[name] = raw_velocity
            mapping[name] = (float(msg.position[idx]), raw_velocity)
        self._joint_state_map = mapping
        sample_time_sec = _stamp_to_sec(msg.header.stamp)
        if sample_time_sec <= 0.0:
            sample_time_sec = self._now_sec()
        self._joint_state_time_sec = sample_time_sec

        ordered_raw_velocities: List[float] = []
        for joint_name in self._joint_names:
            if joint_name not in mapping:
                self._joint_state_accelerations = {}
                return
            ordered_raw_velocities.append(float(raw_velocity_map.get(joint_name, 0.0)))

        if (
            self._filtered_feedback_velocities is None
            or len(self._filtered_feedback_velocities) != len(self._joint_names)
        ):
            filtered_velocities = list(ordered_raw_velocities)
        else:
            alpha = self._feedback_velocity_filter_alpha
            filtered_velocities = [
                alpha * ordered_raw_velocities[index]
                + (1.0 - alpha) * self._filtered_feedback_velocities[index]
                for index in range(len(self._joint_names))
            ]
        self._filtered_feedback_velocities = list(filtered_velocities)

        max_feedback_dt = max(5.0 * self._stream_period, 0.1)
        accelerations: List[float] = [0.0] * len(self._joint_names)
        dt = sample_time_sec - self._last_feedback_time_sec
        if (
            self._last_feedback_velocities_for_accel is not None
            and len(self._last_feedback_velocities_for_accel) == len(self._joint_names)
            and dt > 1.0e-6
            and dt <= max_feedback_dt
        ):
            for index, joint_name in enumerate(self._joint_names):
                raw_acceleration = (
                    filtered_velocities[index] - self._last_feedback_velocities_for_accel[index]
                ) / dt
                limit = self._joint_limits[joint_name].max_acceleration
                accelerations[index] = max(-limit, min(limit, raw_acceleration))
            self._have_feedback_accel_estimate = True
        else:
            self._have_feedback_accel_estimate = False

        self._joint_state_accelerations = {
            joint_name: accelerations[index] for index, joint_name in enumerate(self._joint_names)
        }
        self._last_feedback_velocities_for_accel = list(filtered_velocities)
        self._last_feedback_time_sec = sample_time_sec
        for index, joint_name in enumerate(self._joint_names):
            position, _ = mapping[joint_name]
            mapping[joint_name] = (position, filtered_velocities[index])
        self._joint_state_map = mapping

    def _ordered_joint_state(self) -> Optional[Tuple[List[float], List[float], List[float], float]]:
        if not self._joint_state_map:
            return None
        positions: List[float] = []
        velocities: List[float] = []
        accelerations: List[float] = []
        for joint_name in self._joint_names:
            if joint_name not in self._joint_state_map:
                return None
            position, velocity = self._joint_state_map[joint_name]
            positions.append(position)
            velocities.append(velocity)
            accelerations.append(float(self._joint_state_accelerations.get(joint_name, 0.0)))
        return positions, velocities, accelerations, self._joint_state_time_sec

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _get_current_eef_pose(
        self,
    ) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]]:
        try:
            trans = self._tf_buffer.lookup_transform(
                self._status_base_frame,
                self._status_eef_frame,
                rclpy.time.Time(),
            )
            t = trans.transform.translation
            r = trans.transform.rotation
            return (
                (float(t.x), float(t.y), float(t.z)),
                _normalize_quat_xyzw((float(r.x), float(r.y), float(r.z), float(r.w))),
            )
        except Exception:
            return None

    def _format_eef(self) -> str:
        eef = self._get_current_eef_pose()
        if eef is None:
            return "eef_xyz=(NA, NA, NA)"
        xyz = eef[0]
        return "eef_xyz=(%.3f, %.3f, %.3f)" % xyz

    def _format_target(self, target: PoseStamped) -> str:
        pos = target.pose.position
        return "target_xyz=(%.3f, %.3f, %.3f)" % (pos.x, pos.y, pos.z)

    def _event(self, name: str, target: Optional[PoseStamped] = None, extra: str = "") -> None:
        self._last_event = name
        self._last_event_time_sec = self._now_sec()
        suffix = (" " + extra.strip()) if extra.strip() else ""
        if target is None:
            self.get_logger().info("[EVENT] %s %s%s" % (name, self._format_eef(), suffix))
        else:
            self.get_logger().info(
                "[EVENT] %s %s %s%s"
                % (name, self._format_target(target), self._format_eef(), suffix)
            )

    def _update_ready(self) -> None:
        solver_ready = self._kinematics is not None
        self._ready_solver = solver_ready
        if solver_ready != self._last_ready:
            self.get_logger().info(
                "[STATE] ready solver=%d %s"
                % (1 if solver_ready else 0, self._format_eef())
            )
            self._last_ready = solver_ready

    def _executor_ready(self) -> bool:
        self._update_ready()
        return self._ready_solver

    def _log_status(self) -> None:
        event_age = max(0.0, self._now_sec() - self._last_event_time_sec)
        with self._manual_target_lock:
            has_manual_target = self._latest_manual_target is not None
        active_source = self._active_request.source if self._active_request is not None else "manual"
        tracking_suffix = ""
        if self._last_tracking_error_joint is not None:
            tracking_suffix = " qerr_max=%s:%.3f" % (
                self._last_tracking_error_joint,
                self._last_tracking_error_value,
            )
        otg_suffix = " otg=%s" % ("plan" if self._otg_has_state else "feedback")
        self.get_logger().info(
            "[STATE] ready(solver=%d) manual_target=%d active=%s event=%s(%.2fs) %s%s%s"
            % (
                1 if self._ready_solver else 0,
                1 if has_manual_target else 0,
                active_source,
                self._last_event,
                event_age,
                self._format_eef(),
                tracking_suffix,
                otg_suffix,
            )
        )

    def _resolve_target_pose(self, target: PoseStamped) -> PoseStamped:
        pose_msg = _copy_pose_stamped(target)
        pose_msg.header.frame_id = _normalize_frame_id(pose_msg.header.frame_id) or self._default_frame
        q = pose_msg.pose.orientation
        qx, qy, qz, qw = _normalize_quat_xyzw((float(q.x), float(q.y), float(q.z), float(q.w)))
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        return pose_msg

    def _motion_target_to_pose(self, msg: Arm2TargetPoint) -> PoseStamped:
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self._default_frame
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = float(msg.xyz.x)
        pose_msg.pose.position.y = float(msg.xyz.y)
        pose_msg.pose.position.z = float(msg.xyz.z)
        qx, qy, qz, qw = _quaternion_from_world_pitch(self._j4_axis, float(msg.target_spin_deg))
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        return pose_msg

    def _allocate_execution_id(self) -> int:
        self._next_execution_id += 1
        return self._next_execution_id

    def _publish_motion_execution(self, execution_id: int, status: int, error_code: int, detail: str) -> None:
        msg = Arm2MotionExecution()
        msg.execution_id = int(execution_id)
        msg.status = int(status)
        msg.error_code = int(error_code)
        msg.detail = str(detail)
        self._middleware_result_pub.publish(msg)

    def _on_manual_target(self, msg: PoseStamped) -> None:
        pose_msg = self._resolve_target_pose(msg)
        with self._manual_target_lock:
            self._latest_manual_target = pose_msg
            self._latest_manual_target_time_sec = self._now_sec()
        self._event("manual_target_rx", pose_msg)
        self._last_manual_failure_detail = ""
        self._last_manual_failure_time_sec = 0.0

    def _on_middleware_target(self, msg: Arm2TargetPoint) -> None:
        pose_msg = self._motion_target_to_pose(msg)
        execution_id = self._allocate_execution_id()
        self._event("middleware_target_rx", pose_msg, extra="execution_id=%d" % execution_id)

        if self._active_request is not None:
            self._publish_motion_execution(
                execution_id,
                Arm2MotionExecution.STATUS_FAILED,
                EXECUTION_ERROR_BUSY,
                "middleware target already active",
            )
            return

        if not self._executor_ready():
            self._publish_motion_execution(
                execution_id,
                Arm2MotionExecution.STATUS_FAILED,
                EXECUTION_ERROR_NOT_READY,
                "executor not ready",
            )
            return

        self._active_request = ExecutionRequest(
            source="middleware",
            target=pose_msg,
            execution_id=execution_id,
        )
        self._publish_motion_execution(
            execution_id,
            Arm2MotionExecution.STATUS_ACCEPTED,
            0,
            "accepted",
        )
        self._event("middleware_track_start", pose_msg, extra="execution_id=%d" % execution_id)

    def _target_changed(self, prev: PoseStamped, cur: PoseStamped) -> bool:
        if _normalize_frame_id(prev.header.frame_id) != _normalize_frame_id(cur.header.frame_id):
            return True

        dp = cur.pose.position
        pp = prev.pose.position
        dx = float(dp.x) - float(pp.x)
        dy = float(dp.y) - float(pp.y)
        dz = float(dp.z) - float(pp.z)
        pos_delta = math.sqrt(dx * dx + dy * dy + dz * dz)

        dq = cur.pose.orientation
        pq = prev.pose.orientation
        rot_delta = _quat_angle(
            (float(dq.x), float(dq.y), float(dq.z), float(dq.w)),
            (float(pq.x), float(pq.y), float(pq.z), float(pq.w)),
        )
        return pos_delta > self._pos_threshold or rot_delta > self._rot_threshold

    def _select_manual_target(self) -> Optional[PoseStamped]:
        with self._manual_target_lock:
            if self._latest_manual_target is None:
                self._active_manual_target = None
                return None
            if (
                self._active_manual_target is None
                or self._target_changed(self._active_manual_target, self._latest_manual_target)
            ):
                self._active_manual_target = _copy_pose_stamped(self._latest_manual_target)
            return _copy_pose_stamped(self._active_manual_target)

    def _log_manual_target_failure(self, detail: str) -> None:
        detail = str(detail or "target solve failed")
        now_sec = self._now_sec()
        if (
            detail == self._last_manual_failure_detail
            and (now_sec - self._last_manual_failure_time_sec) < 1.0
        ):
            return
        self._last_manual_failure_detail = detail
        self._last_manual_failure_time_sec = now_sec
        self.get_logger().warn("manual target ignored: %s" % detail)

    def _get_tracking_target(self) -> Optional[ExecutionRequest]:
        if self._active_request is not None:
            return self._active_request

        manual_target = self._select_manual_target()
        if manual_target is None:
            return None
        return ExecutionRequest(source="manual", target=manual_target)

    def _seed_joint_map(self, current_positions: Sequence[float]) -> Dict[str, float]:
        return {
            joint_name: float(current_positions[index])
            for index, joint_name in enumerate(self._joint_names)
        }

    def _first_joint_out_of_limits(self, target_positions: Sequence[float]) -> Optional[str]:
        for index, joint_name in enumerate(self._joint_names):
            limit = self._joint_limits[joint_name]
            position = float(target_positions[index])
            if position < limit.min_position or position > limit.max_position:
                return (
                    f"joint '{joint_name}' target {position:.6f} outside "
                    f"[{limit.min_position:.6f}, {limit.max_position:.6f}]"
                )
        return None

    def _solve_target_positions(
        self,
        request: ExecutionRequest,
        current_positions: Sequence[float],
    ) -> Tuple[Optional[List[float]], Optional[str]]:
        pose = request.target
        q = pose.pose.orientation
        pitch_rad = self._kinematics.world_pitch_from_quaternion(
            (float(q.x), float(q.y), float(q.z), float(q.w))
        )
        q_target = self._kinematics.solve_xyz_pitch(
            float(pose.pose.position.x),
            float(pose.pose.position.y),
            float(pose.pose.position.z),
            pitch_rad,
            seed_joints=self._seed_joint_map(current_positions),
        )
        if q_target is None:
            return None, "ik solve failed"

        target_positions = [float(q_target[joint_name]) for joint_name in self._joint_names]
        out_of_limits = self._first_joint_out_of_limits(target_positions)
        if out_of_limits is not None:
            return None, out_of_limits

        return target_positions, None

    def _publish_stream_reference(
        self,
        current_positions: Sequence[float],
        current_velocities: Sequence[float],
        current_accelerations: Sequence[float],
        target_positions: Sequence[float],
    ) -> Tuple[bool, str]:
        reset_reason = self._otg_reset_reason(
            current_positions,
            current_velocities,
            target_positions,
        )
        if reset_reason is not None:
            self._prime_otg_from_feedback(
                current_positions,
                current_velocities,
                current_accelerations,
                target_positions,
                reset_reason,
            )
        elif self._otg_has_state:
            self._set_otg_target(target_positions)

        result = self._otg.update(self._otg_input, self._otg_output)
        if result not in (Result.Working, Result.Finished):
            return False, f"ruckig update returned {result}"

        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(self._joint_names)

        point = JointTrajectoryPoint()
        point.positions = list(self._otg_output.new_position)
        point.velocities = list(self._otg_output.new_velocity)
        point.accelerations = list(self._otg_output.new_acceleration)
        point.time_from_start = _duration_from_seconds(self._trajectory_sampling_period)
        message.points = [point]
        self._trajectory_pub.publish(message)
        self._advance_otg_state_from_output(target_positions)
        return True, ""

    def _clamped_feedback_velocity(self, current_velocities: Sequence[float]) -> List[float]:
        return [
            max(
                -self._joint_limits[name].max_velocity,
                min(self._joint_limits[name].max_velocity, float(current_velocities[index])),
            )
            for index, name in enumerate(self._joint_names)
        ]

    def _feedback_accelerations_for_otg(self, current_accelerations: Sequence[float]) -> List[float]:
        if self._feedback_accel_mode != "filtered" or not self._have_feedback_accel_estimate:
            return [0.0] * len(self._joint_names)
        return self._clamped_feedback_acceleration(current_accelerations)

    def _clamped_feedback_acceleration(self, current_accelerations: Sequence[float]) -> List[float]:
        return [
            max(
                -self._joint_limits[name].max_acceleration,
                min(self._joint_limits[name].max_acceleration, float(current_accelerations[index])),
            )
            for index, name in enumerate(self._joint_names)
        ]

    def _set_otg_target(self, target_positions: Sequence[float]) -> None:
        self._otg_input.target_position = list(target_positions)
        self._otg_input.target_velocity = [0.0] * len(self._joint_names)
        self._otg_input.target_acceleration = [0.0] * len(self._joint_names)
        self._otg_target_positions = list(target_positions)

    def _otg_reset_reason(
        self,
        current_positions: Sequence[float],
        current_velocities: Sequence[float],
        target_positions: Sequence[float],
    ) -> Optional[str]:
        if not self._otg_has_state:
            return "init_feedback"
        if self._feedback_sync_mode == "always":
            return "always_feedback"
        if self._feedback_sync_mode == "init_only":
            self._position_desync_cycles = 0
            self._velocity_desync_cycles = 0
            return None

        planned_positions = list(self._otg_input.current_position)
        planned_velocities = list(self._otg_input.current_velocity)
        max_position_error = max(
            abs(float(current_positions[index]) - float(planned_positions[index]))
            for index in range(len(self._joint_names))
        )

        if max_position_error > self._otg_plan_feedback_position_reset:
            self._position_desync_cycles += 1
            if self._position_desync_cycles >= self._feedback_position_reset_cycles:
                self._last_desync_planned_positions = list(planned_positions)
                self._last_desync_planned_velocities = list(planned_velocities)
                self._position_desync_cycles = 0
                self._velocity_desync_cycles = 0
                return f"desync_position:{max_position_error:.3f}"
        else:
            self._position_desync_cycles = 0

        if self._feedback_velocity_reset_enabled:
            max_velocity_error = max(
                abs(float(current_velocities[index]) - float(planned_velocities[index]))
                for index in range(len(self._joint_names))
            )
            if max_velocity_error > self._otg_plan_feedback_velocity_reset:
                self._velocity_desync_cycles += 1
                if self._velocity_desync_cycles >= self._feedback_position_reset_cycles:
                    self._last_desync_planned_positions = list(planned_positions)
                    self._last_desync_planned_velocities = list(planned_velocities)
                    self._position_desync_cycles = 0
                    self._velocity_desync_cycles = 0
                    return f"desync_velocity:{max_velocity_error:.3f}"
            else:
                self._velocity_desync_cycles = 0
        else:
            self._velocity_desync_cycles = 0
        return None

    def _max_joint_error_detail(
        self,
        feedback_values: Sequence[float],
        planned_values: Sequence[float],
    ) -> Tuple[Optional[str], float, float, float]:
        max_error_joint: Optional[str] = None
        max_error_value = 0.0
        max_feedback_value = 0.0
        max_planned_value = 0.0
        for index, joint_name in enumerate(self._joint_names):
            feedback_value = float(feedback_values[index])
            planned_value = float(planned_values[index])
            error_value = abs(feedback_value - planned_value)
            if max_error_joint is None or error_value > max_error_value:
                max_error_joint = joint_name
                max_error_value = error_value
                max_feedback_value = feedback_value
                max_planned_value = planned_value
        return max_error_joint, max_error_value, max_feedback_value, max_planned_value

    def _prime_otg_from_feedback(
        self,
        current_positions: Sequence[float],
        current_velocities: Sequence[float],
        current_accelerations: Sequence[float],
        target_positions: Sequence[float],
        reason: str,
    ) -> None:
        self._otg_input.current_position = list(current_positions)
        self._otg_input.current_velocity = self._clamped_feedback_velocity(current_velocities)
        self._otg_input.current_acceleration = self._feedback_accelerations_for_otg(current_accelerations)
        self._set_otg_target(target_positions)
        self._otg_has_state = True
        self._otg_last_sync_reason = reason
        self._position_desync_cycles = 0
        self._velocity_desync_cycles = 0
        self._log_otg_reset(reason, current_positions, current_velocities)

    def _log_otg_reset(
        self,
        reason: str,
        current_positions: Sequence[float],
        current_velocities: Sequence[float],
    ) -> None:
        now_sec = self._now_sec()
        if (
            reason == self._last_otg_reset_log_reason
            and (now_sec - self._last_otg_reset_log_time_sec) < 1.0
        ):
            return

        log_message = f"[OTG] rebuild_from_feedback reason={reason}"
        if reason.startswith("desync_position"):
            max_joint, max_error, feedback_value, planned_value = self._max_joint_error_detail(
                current_positions,
                self._last_desync_planned_positions or self._otg_input.current_position,
            )
            if max_joint is not None:
                log_message += (
                    " joint=%s feedback=%.6f planned=%.6f abs_err=%.6f"
                    % (max_joint, feedback_value, planned_value, max_error)
                )
        elif reason.startswith("desync_velocity"):
            max_joint, max_error, feedback_value, planned_value = self._max_joint_error_detail(
                current_velocities,
                self._last_desync_planned_velocities or self._otg_input.current_velocity,
            )
            if max_joint is not None:
                log_message += (
                    " joint=%s feedback=%.6f planned=%.6f abs_err=%.6f"
                    % (max_joint, feedback_value, planned_value, max_error)
                )

        if reason.startswith("desync_"):
            self.get_logger().warn(log_message)
        else:
            self.get_logger().info(log_message)

        self._last_otg_reset_log_reason = reason
        self._last_otg_reset_log_time_sec = now_sec
        self._last_desync_planned_positions = None
        self._last_desync_planned_velocities = None

    def _advance_otg_state_from_output(self, target_positions: Sequence[float]) -> None:
        try:
            self._otg_output.pass_to_input(self._otg_input)
        except AttributeError:
            self._otg_input.current_position = list(self._otg_output.new_position)
            self._otg_input.current_velocity = list(self._otg_output.new_velocity)
            self._otg_input.current_acceleration = list(self._otg_output.new_acceleration)
        self._set_otg_target(target_positions)
        self._otg_has_state = True

    def _update_tracking_error(
        self,
        current_positions: Sequence[float],
        target_positions: Sequence[float],
    ) -> None:
        max_error_joint: Optional[str] = None
        max_error_value = 0.0
        for index, joint_name in enumerate(self._joint_names):
            error = abs(float(target_positions[index]) - float(current_positions[index]))
            if max_error_joint is None or error > max_error_value:
                max_error_joint = joint_name
                max_error_value = error
        self._last_tracking_error_joint = max_error_joint
        self._last_tracking_error_value = max_error_value

        now_sec = self._now_sec()
        if max_error_joint is not None and (now_sec - self._last_tracking_error_log_time_sec) >= 1.0:
            self.get_logger().info(
                "[TRACK] qerr_max=%s:%.3f"
                % (max_error_joint, max_error_value)
            )
            self._last_tracking_error_log_time_sec = now_sec

    def _request_reached(
        self,
        request: ExecutionRequest,
        current_positions: Sequence[float],
        current_velocities: Sequence[float],
    ) -> bool:
        current_pose = self._get_current_eef_pose()
        if current_pose is None:
            return False

        current_xyz, _current_quat = current_pose
        target = request.target.pose
        dx = current_xyz[0] - float(target.position.x)
        dy = current_xyz[1] - float(target.position.y)
        dz = current_xyz[2] - float(target.position.z)
        pos_error = math.sqrt(dx * dx + dy * dy + dz * dz)

        target_quat = _normalize_quat_xyzw(
            (
                float(target.orientation.x),
                float(target.orientation.y),
                float(target.orientation.z),
                float(target.orientation.w),
            )
        )
        target_pitch = self._kinematics.world_pitch_from_quaternion(target_quat)
        current_pitch = self._kinematics.forward_world_pitch(current_positions)
        rot_error = abs(normalize_angle_rad(current_pitch - target_pitch))
        max_velocity = max((abs(float(v)) for v in current_velocities), default=0.0)
        return (
            pos_error <= self._pos_threshold
            and rot_error <= self._rot_threshold
            and max_velocity <= self._success_velocity_tolerance
        )

    def _complete_active_request(self, success: bool, error_code: int, detail: str) -> None:
        request = self._active_request
        if request is None or request.execution_id is None:
            self._active_request = None
            return

        status = Arm2MotionExecution.STATUS_SUCCEEDED if success else Arm2MotionExecution.STATUS_FAILED
        self._publish_motion_execution(request.execution_id, status, error_code, detail)
        event_name = "middleware_track_done" if success else "middleware_track_fail"
        self._event(event_name, request.target, extra="execution_id=%d" % request.execution_id)
        self._active_request = None

    def _on_timer(self) -> None:
        if not self._executor_ready():
            return

        ordered_joint_state = self._ordered_joint_state()
        if ordered_joint_state is None:
            if self._active_request is not None:
                self._complete_active_request(
                    success=False,
                    error_code=EXECUTION_ERROR_JOINT_STATE_UNAVAILABLE,
                    detail="joint state unavailable",
                )
            return

        current_positions, current_velocities, current_accelerations, _joint_state_time_sec = ordered_joint_state
        request = self._get_tracking_target()
        self._last_tracking_error_joint = None
        self._last_tracking_error_value = 0.0

        if self._active_request is not None and self._request_reached(
            self._active_request,
            current_positions,
            current_velocities,
        ):
            self._complete_active_request(
                success=True,
                error_code=0,
                detail="target reached",
            )
            request = self._get_tracking_target()

        target_positions = list(current_positions)
        if request is not None:
            solved_positions, error = self._solve_target_positions(request, current_positions)
            if solved_positions is None:
                if request.source == "middleware":
                    self._complete_active_request(
                        success=False,
                        error_code=EXECUTION_ERROR_IK_FAILED if error == "ik solve failed" else EXECUTION_ERROR_TARGET_OUT_OF_LIMITS,
                        detail=error or "target solve failed",
                    )
                    request = self._get_tracking_target()
                else:
                    self._log_manual_target_failure(error or "target solve failed")
            else:
                if request.source == "manual":
                    self._last_manual_failure_detail = ""
                    self._last_manual_failure_time_sec = 0.0
                target_positions = solved_positions
                self._update_tracking_error(current_positions, target_positions)

        ok, detail = self._publish_stream_reference(
            current_positions,
            current_velocities,
            current_accelerations,
            target_positions,
        )
        if not ok:
            if self._active_request is not None:
                self._complete_active_request(
                    success=False,
                    error_code=EXECUTION_ERROR_TRAJECTORY_GENERATION_FAILED,
                    detail=detail or "ruckig streaming step failed",
                )
            else:
                self.get_logger().warn("%s; skipping publish" % (detail or "ruckig streaming step failed"))


def parse_args():
    parser = argparse.ArgumentParser(description="Target pose executor using IK + online Ruckig streaming")
    parser.add_argument("--target-topic", default="/rc_arm_2/target_pose")
    parser.add_argument("--middleware-target-topic", default="/arm2/middleware/motion_target")
    parser.add_argument("--middleware-result-topic", default="/arm2/middleware/motion_execution")
    parser.add_argument("--joint-names", default="j1_joint,j2_joint,j3_joint,j4_joint")
    parser.add_argument("--default-frame", default="world")
    parser.add_argument("--trajectory-topic", default="/arm_controller/joint_trajectory")
    parser.add_argument("--joint-limits-file", required=True)
    parser.add_argument("--trajectory-sampling-period", type=float, default=0.01)
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--urdf-path", default="")
    parser.add_argument("--pos-threshold", type=float, default=0.03)
    parser.add_argument("--rot-threshold", type=float, default=0.03)
    parser.add_argument("--check-period", type=float, default=0.01)
    parser.add_argument("--j4-axis", choices=["x", "y", "z"], default="x")
    parser.add_argument("--status-log-period", type=float, default=1.0, help="state log period, <=0 to disable")
    parser.add_argument("--status-base-frame", default="world")
    parser.add_argument("--status-eef-frame", default="end_effector")
    parser.add_argument("--feedback-sync-mode", choices=sorted(FEEDBACK_SYNC_MODES), default="desync_only")
    parser.add_argument("--feedback-position-reset-threshold", type=float, default=0.12)
    parser.add_argument("--feedback-position-reset-cycles", type=int, default=3)
    parser.add_argument("--feedback-velocity-reset-enabled", default="false")
    parser.add_argument("--feedback-velocity-reset-threshold", type=float, default=1.5)
    parser.add_argument("--feedback-velocity-filter-alpha", type=float, default=0.2)
    parser.add_argument("--feedback-accel-mode", choices=sorted(FEEDBACK_ACCEL_MODES), default="zero")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    joints = [j.strip() for j in args.joint_names.split(",") if j.strip()]
    if not joints:
        raise SystemExit("joint-names cannot be empty")

    rclpy.init()
    try:
        node = TargetPoseRuckigExecutor(
            target_topic=args.target_topic,
            middleware_target_topic=args.middleware_target_topic,
            middleware_result_topic=args.middleware_result_topic,
            joint_names=joints,
            default_frame=args.default_frame,
            trajectory_topic=args.trajectory_topic,
            joint_limits_file=args.joint_limits_file,
            trajectory_sampling_period=args.trajectory_sampling_period,
            pos_threshold=args.pos_threshold,
            rot_threshold=args.rot_threshold,
            check_period=args.check_period,
            j4_axis=args.j4_axis,
            joint_state_topic=args.joint_state_topic,
            urdf_path=args.urdf_path,
            status_log_period=args.status_log_period,
            status_base_frame=args.status_base_frame,
            status_eef_frame=args.status_eef_frame,
            feedback_sync_mode=args.feedback_sync_mode,
            feedback_position_reset_threshold=args.feedback_position_reset_threshold,
            feedback_position_reset_cycles=args.feedback_position_reset_cycles,
            feedback_velocity_reset_enabled=_parse_bool(args.feedback_velocity_reset_enabled),
            feedback_velocity_reset_threshold=args.feedback_velocity_reset_threshold,
            feedback_velocity_filter_alpha=args.feedback_velocity_filter_alpha,
            feedback_accel_mode=args.feedback_accel_mode,
        )
    except Exception as exc:
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(str(exc)) from exc

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
