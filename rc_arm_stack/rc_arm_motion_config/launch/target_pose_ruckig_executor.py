#!/usr/bin/env python3
"""Subscribe manual and middleware targets and execute IK + Ruckig trajectories."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import threading
from typing import Dict, List, Optional, Sequence, Tuple

from arm_msgs.msg import Arm2MotionExecution, Arm2TargetPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tf2_ros
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml

from rc_arm_world_pitch_kinematics import RcArmWorldPitchKinematics

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
EXECUTION_ERROR_GOAL_SEND_EXCEPTION = -8
EXECUTION_ERROR_GOAL_REJECTED = -9
EXECUTION_ERROR_RESULT_EXCEPTION = -10
EXECUTION_ERROR_RESULT_FAILED = -11


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
        follow_joint_trajectory_action: str,
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
    ) -> None:
        super().__init__("rc_arm_target_pose_executor")

        if RUCKIG_IMPORT_ERROR is not None:
            raise RuntimeError(f"ruckig import failed: {RUCKIG_IMPORT_ERROR}")

        self._manual_target_topic = target_topic
        self._middleware_target_topic = middleware_target_topic
        self._middleware_result_topic = middleware_result_topic
        self._joint_names = list(joint_names)
        self._default_frame = _normalize_frame_id(default_frame)
        self._follow_joint_trajectory_action = follow_joint_trajectory_action
        self._trajectory_sampling_period = max(0.001, float(trajectory_sampling_period))
        self._pos_threshold = max(0.0, float(pos_threshold))
        self._rot_threshold = max(0.0, float(rot_threshold))
        self._j4_axis = str(j4_axis).strip().lower() if str(j4_axis).strip() else "x"
        if self._j4_axis not in {"x", "y", "z"}:
            self._j4_axis = "x"
        self._status_log_period = max(0.0, float(status_log_period))
        self._status_base_frame = _normalize_frame_id(status_base_frame)
        self._status_eef_frame = _normalize_frame_id(status_eef_frame)

        self._manual_target_lock = threading.Lock()
        self._latest_manual_target: Optional[PoseStamped] = None
        self._last_sent_manual_target: Optional[PoseStamped] = None

        self._busy = False
        self._active_request: Optional[ExecutionRequest] = None
        self._ready_controller_action = False
        self._ready_solver = False
        self._last_ready_tuple = None
        self._joint_state_map: Dict[str, Tuple[float, float]] = {}
        self._next_execution_id = 0
        self._last_event = "init"
        self._last_event_time_sec = self._now_sec()

        self._joint_limits = self._load_joint_limits(joint_limits_file, self._joint_names)

        self._kinematics = RcArmWorldPitchKinematics(
            urdf_path=urdf_path or None,
            joint_names=self._joint_names,
            j4_axis=self._j4_axis,
        )
        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            follow_joint_trajectory_action,
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
        self._timer = self.create_timer(max(0.02, float(check_period)), self._on_timer)
        if self._status_log_period > 0.0:
            self._status_timer = self.create_timer(self._status_log_period, self._log_status)

        self.get_logger().info(
            "TargetPose executor started: manual_topic=%s middleware_target=%s middleware_result=%s "
            "trajectory_action=%s joint_limits=%s status_tf=%s->%s"
            % (
                self._manual_target_topic,
                self._middleware_target_topic,
                self._middleware_result_topic,
                self._follow_joint_trajectory_action,
                joint_limits_file,
                self._status_base_frame,
                self._status_eef_frame,
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
        for idx, name in enumerate(msg.name):
            if idx >= len(msg.position):
                continue
            velocity = 0.0
            if idx < len(msg.velocity):
                velocity = float(msg.velocity[idx])
            mapping[name] = (float(msg.position[idx]), velocity)
        self._joint_state_map = mapping

    def _ordered_joint_state(self) -> Optional[Tuple[List[float], List[float]]]:
        if not self._joint_state_map:
            return None
        positions: List[float] = []
        velocities: List[float] = []
        for joint_name in self._joint_names:
            if joint_name not in self._joint_state_map:
                return None
            position, velocity = self._joint_state_map[joint_name]
            positions.append(position)
            velocities.append(velocity)
        return positions, velocities

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

    def _get_current_eef_xyz(self) -> Optional[Tuple[float, float, float]]:
        pose = self._get_current_eef_pose()
        if pose is None:
            return None
        return pose[0]

    def _format_eef(self) -> str:
        eef = self._get_current_eef_xyz()
        if eef is None:
            return "eef_xyz=(NA, NA, NA)"
        return "eef_xyz=(%.3f, %.3f, %.3f)" % eef

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

    def _set_busy(self, new_state: bool, reason: str) -> None:
        if self._busy != new_state:
            self._busy = new_state
            self.get_logger().info(
                "[STATE] busy=%d reason=%s %s"
                % (1 if new_state else 0, reason, self._format_eef())
            )
        else:
            self._busy = new_state

    def _update_ready(self) -> None:
        controller_ready = self._trajectory_client.server_is_ready()
        solver_ready = self._kinematics is not None

        self._ready_controller_action = controller_ready
        self._ready_solver = solver_ready

        ready_tuple = (controller_ready, solver_ready)
        if ready_tuple != self._last_ready_tuple:
            self.get_logger().info(
                "[STATE] ready trajectory_action=%d solver=%d %s"
                % (1 if controller_ready else 0, 1 if solver_ready else 0, self._format_eef())
            )
            self._last_ready_tuple = ready_tuple

    def _executor_ready(self) -> bool:
        self._update_ready()
        return self._ready_controller_action and self._ready_solver

    def _log_status(self) -> None:
        event_age = max(0.0, self._now_sec() - self._last_event_time_sec)
        with self._manual_target_lock:
            has_manual_target = self._latest_manual_target is not None
        active_source = self._active_request.source if self._active_request is not None else "idle"
        self.get_logger().info(
            "[STATE] busy=%d ready(trajectory_action=%d,solver=%d) manual_target=%d active=%s event=%s(%.2fs) %s"
            % (
                1 if self._busy else 0,
                1 if self._ready_controller_action else 0,
                1 if self._ready_solver else 0,
                1 if has_manual_target else 0,
                active_source,
                self._last_event,
                event_age,
                self._format_eef(),
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
        self._event("manual_target_rx", pose_msg)

    def _on_middleware_target(self, msg: Arm2TargetPoint) -> None:
        pose_msg = self._motion_target_to_pose(msg)
        execution_id = self._allocate_execution_id()
        self._event("middleware_target_rx", pose_msg, extra="execution_id=%d" % execution_id)

        if self._busy:
            self._publish_motion_execution(
                execution_id,
                Arm2MotionExecution.STATUS_FAILED,
                EXECUTION_ERROR_BUSY,
                "executor busy",
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

        self._start_execution(
            ExecutionRequest(
                source="middleware",
                target=pose_msg,
                execution_id=execution_id,
            )
        )

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

    def _on_timer(self) -> None:
        if self._busy or not self._executor_ready():
            return

        with self._manual_target_lock:
            target = _copy_pose_stamped(self._latest_manual_target) if self._latest_manual_target is not None else None

        if target is None:
            return
        if self._last_sent_manual_target is not None and not self._target_changed(self._last_sent_manual_target, target):
            return

        self._start_execution(
            ExecutionRequest(
                source="manual",
                target=target,
            )
        )

    def _start_execution(self, request: ExecutionRequest) -> None:
        if self._busy:
            return
        request.target = self._resolve_target_pose(request.target)
        if request.source == "manual":
            self._last_sent_manual_target = _copy_pose_stamped(request.target)
        self._active_request = request
        self._set_busy(True, "execute_%s" % request.source)
        extra = "source=%s" % request.source
        if request.execution_id is not None:
            extra += " execution_id=%d" % request.execution_id
        self._event("solve_request", request.target, extra=extra)
        self._solve_target(request)

    def _solve_target(self, request: ExecutionRequest) -> None:
        pose = request.target
        q = pose.pose.orientation
        pitch_rad = self._kinematics.world_pitch_from_quaternion(
            (float(q.x), float(q.y), float(q.z), float(q.w))
        )
        seed = self._seed_joint_map()
        q_target = self._kinematics.solve_xyz_pitch(
            float(pose.pose.position.x),
            float(pose.pose.position.y),
            float(pose.pose.position.z),
            pitch_rad,
            seed_joints=seed,
        )
        if q_target is None:
            self._event("solve_fail", pose, extra="source=%s" % request.source)
            self._complete_execution(
                request,
                success=False,
                error_code=EXECUTION_ERROR_IK_FAILED,
                detail="ik solve failed",
                reason="solve_fail",
            )
            return

        ordered_joint_state = self._ordered_joint_state()
        if ordered_joint_state is None:
            self._complete_execution(
                request,
                success=False,
                error_code=EXECUTION_ERROR_JOINT_STATE_UNAVAILABLE,
                detail="joint state unavailable",
                reason="joint_state_unavailable",
            )
            return

        current_positions, current_velocities = ordered_joint_state
        target_positions = [float(q_target[joint_name]) for joint_name in self._joint_names]
        out_of_limits = self._first_joint_out_of_limits(target_positions)
        if out_of_limits is not None:
            self._complete_execution(
                request,
                success=False,
                error_code=EXECUTION_ERROR_TARGET_OUT_OF_LIMITS,
                detail=out_of_limits,
                reason="target_out_of_limits",
            )
            return

        self._event("solve_ok", pose, extra="source=%s pitch=%.3f" % (request.source, pitch_rad))
        trajectory = self._build_trajectory(
            current_positions=current_positions,
            current_velocities=current_velocities,
            target_positions=target_positions,
        )
        if trajectory is None:
            self._complete_execution(
                request,
                success=False,
                error_code=EXECUTION_ERROR_TRAJECTORY_GENERATION_FAILED,
                detail="ruckig trajectory generation failed",
                reason="trajectory_generation_failed",
            )
            return

        self._send_trajectory_goal(request, trajectory)

    def _seed_joint_map(self) -> Optional[Dict[str, float]]:
        ordered_joint_state = self._ordered_joint_state()
        if ordered_joint_state is None:
            return None
        positions, _ = ordered_joint_state
        return {
            joint_name: positions[index]
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

    def _build_trajectory(
        self,
        current_positions: Sequence[float],
        current_velocities: Sequence[float],
        target_positions: Sequence[float],
    ) -> Optional[JointTrajectory]:
        dofs = len(self._joint_names)
        otg = Ruckig(dofs, self._trajectory_sampling_period)
        inp = InputParameter(dofs)
        out = OutputParameter(dofs)

        inp.current_position = list(current_positions)
        inp.current_velocity = list(current_velocities)
        inp.current_acceleration = [0.0] * dofs
        inp.target_position = list(target_positions)
        inp.target_velocity = [0.0] * dofs
        inp.target_acceleration = [0.0] * dofs
        inp.max_velocity = [self._joint_limits[name].max_velocity for name in self._joint_names]
        inp.max_acceleration = [self._joint_limits[name].max_acceleration for name in self._joint_names]
        inp.max_jerk = [self._joint_limits[name].max_jerk for name in self._joint_names]

        message = JointTrajectory()
        message.joint_names = list(self._joint_names)

        initial = JointTrajectoryPoint()
        initial.positions = list(current_positions)
        initial.velocities = list(current_velocities)
        initial.accelerations = [0.0] * dofs
        initial.time_from_start = _duration_from_seconds(0.0)
        message.points.append(initial)

        elapsed = 0.0
        max_steps = 100000
        result = Result.Working
        for _ in range(max_steps):
            result = otg.update(inp, out)
            if result not in (Result.Working, Result.Finished):
                return None

            elapsed += self._trajectory_sampling_period
            point = JointTrajectoryPoint()
            point.positions = list(out.new_position)
            point.velocities = list(out.new_velocity)
            point.accelerations = list(out.new_acceleration)
            point.time_from_start = _duration_from_seconds(elapsed)
            message.points.append(point)

            if result == Result.Finished:
                break
            out.pass_to_input(inp)
        else:
            return None

        if not message.points:
            return None
        return message

    def _send_trajectory_goal(
        self,
        request: ExecutionRequest,
        trajectory: JointTrajectory,
    ) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self._event("goal_send", request.target, extra="source=%s" % request.source)
        try:
            send_future = self._trajectory_client.send_goal_async(goal)
        except Exception as exc:
            self._complete_execution(
                request,
                success=False,
                error_code=EXECUTION_ERROR_GOAL_SEND_EXCEPTION,
                detail=f"trajectory goal send exception: {exc}",
                reason="goal_send_exception",
            )
            return

        send_future.add_done_callback(
            lambda future, request=request: self._on_goal_response(future, request)
        )

    def _on_goal_response(self, future, request: ExecutionRequest) -> None:
        if self._active_request is not request:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._complete_execution(
                request,
                success=False,
                error_code=EXECUTION_ERROR_GOAL_SEND_EXCEPTION,
                detail=f"trajectory goal send exception: {exc}",
                reason="goal_send_exception",
            )
            return

        if goal_handle is None or not goal_handle.accepted:
            self._complete_execution(
                request,
                success=False,
                error_code=EXECUTION_ERROR_GOAL_REJECTED,
                detail="trajectory goal rejected",
                reason="goal_rejected",
            )
            return

        if request.execution_id is not None:
            self._publish_motion_execution(
                request.execution_id,
                Arm2MotionExecution.STATUS_ACCEPTED,
                0,
                "accepted",
            )
        self._event("goal_accepted", request.target, extra="source=%s" % request.source)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, request=request: self._on_goal_result(future, request)
        )

    def _on_goal_result(self, future, request: ExecutionRequest) -> None:
        if self._active_request is not request:
            return

        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as exc:
            self._complete_execution(
                request,
                success=False,
                error_code=EXECUTION_ERROR_RESULT_EXCEPTION,
                detail=f"trajectory result exception: {exc}",
                reason="result_exception",
            )
            return

        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self._event("exec_ok", request.target, extra="source=%s" % request.source)
            self._complete_execution(
                request,
                success=True,
                error_code=result.error_code,
                detail=result.error_string or "execution succeeded",
                reason="goal_done",
            )
            return

        detail = result.error_string or f"trajectory execution failed, error_code={result.error_code}"
        self._event("exec_fail", request.target, extra="source=%s" % request.source)
        self._complete_execution(
            request,
            success=False,
            error_code=EXECUTION_ERROR_RESULT_FAILED,
            detail=detail,
            reason="goal_done",
        )

    def _complete_execution(
        self,
        request: ExecutionRequest,
        success: bool,
        error_code: int,
        detail: str,
        reason: str,
    ) -> None:
        if self._active_request is not request:
            return

        status = (
            Arm2MotionExecution.STATUS_SUCCEEDED
            if success
            else Arm2MotionExecution.STATUS_FAILED
        )

        self._active_request = None
        self._set_busy(False, reason)

        if request.execution_id is not None:
            self._publish_motion_execution(
                request.execution_id,
                status,
                error_code,
                detail,
            )


def parse_args():
    parser = argparse.ArgumentParser(description="Target pose executor using IK + Ruckig + FJT")
    parser.add_argument("--target-topic", default="/rc_arm_2/target_pose")
    parser.add_argument("--middleware-target-topic", default="/arm2/middleware/motion_target")
    parser.add_argument("--middleware-result-topic", default="/arm2/middleware/motion_execution")
    parser.add_argument("--joint-names", default="j1_joint,j2_joint,j3_joint,j4_joint")
    parser.add_argument("--default-frame", default="world")
    parser.add_argument("--follow-joint-trajectory-action", default="/arm_controller/follow_joint_trajectory")
    parser.add_argument("--joint-limits-file", required=True)
    parser.add_argument("--trajectory-sampling-period", type=float, default=0.01)
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--urdf-path", default="")
    parser.add_argument("--pos-threshold", type=float, default=0.003)
    parser.add_argument("--rot-threshold", type=float, default=0.03)
    parser.add_argument("--check-period", type=float, default=0.05)
    parser.add_argument("--j4-axis", choices=["x", "y", "z"], default="x")
    parser.add_argument("--status-log-period", type=float, default=1.0, help="state log period, <=0 to disable")
    parser.add_argument("--status-base-frame", default="world")
    parser.add_argument("--status-eef-frame", default="end_effector")
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
            follow_joint_trajectory_action=args.follow_joint_trajectory_action,
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
