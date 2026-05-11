#!/usr/bin/env python3
"""Subscribe PoseStamped targets and drive Ruckig trajectory execution."""

import argparse
import math
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from arm_msgs.srv import GenerateJointTrajectory
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import tf2_ros
import yaml

from rc_arm_kinematics import RcArmWorldPitchKinematics
from target_trace_utils import (
    build_trace_event,
    publish_trace_event,
    target_fields_from_values,
    trace_id_from_stamp,
)


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


def _copy_pose_stamped(msg: PoseStamped) -> PoseStamped:
    copied = PoseStamped()
    copied.header = msg.header
    copied.pose = msg.pose
    return copied


class TargetPoseRuckigExecutor(Node):
    def __init__(
        self,
        *,
        target_topic: str,
        joint_names: List[str],
        default_frame: str,
        position_threshold: float,
        rotation_threshold: float,
        velocity_scale: float,
        acceleration_scale: float,
        jerk_scale: float,
        goal_tolerance: float,
        control_period: float,
        check_period: float,
        j4_axis: str,
        joint_state_topic: str,
        urdf_path: str,
        status_log_period: float,
        status_base_frame: str,
        status_eef_frame: str,
        trace_event_topic: str,
        trace_goal_context_topic: str,
        trajectory_service_name: str,
        controller_action_name: str,
    ) -> None:
        super().__init__("rc_arm_target_pose_ruckig_executor")
        self._joint_names = list(joint_names)
        self._default_frame = _normalize_frame_id(default_frame)
        self._position_threshold = max(0.0, float(position_threshold))
        self._rotation_threshold = max(0.0, float(rotation_threshold))
        self._velocity_scale = max(0.01, min(1.0, float(velocity_scale)))
        self._acceleration_scale = max(0.01, min(1.0, float(acceleration_scale)))
        self._jerk_scale = max(0.01, min(1.0, float(jerk_scale)))
        self._goal_tolerance = max(1.0e-4, float(goal_tolerance))
        self._control_period = max(0.001, float(control_period))
        self._status_log_period = max(0.0, float(status_log_period))
        self._status_base_frame = _normalize_frame_id(status_base_frame)
        self._status_eef_frame = _normalize_frame_id(status_eef_frame)
        self._target_lock = threading.Lock()
        self._latest_target: Optional[PoseStamped] = None
        self._last_dispatched_target: Optional[PoseStamped] = None
        self._latest_joint_state: Dict[str, Dict[str, float]] = {}
        self._busy = False
        self._cancel_requested = False
        self._current_goal_handle = None
        self._active_trace_target: Optional[PoseStamped] = None
        self._last_event = "init"
        self._last_event_time_ns = time.time_ns()

        self._kinematics = RcArmWorldPitchKinematics(
            urdf_path=urdf_path or None,
            joint_names=self._joint_names,
            j4_axis=j4_axis,
        )
        self._joint_limits = self._load_joint_limits()

        self._trajectory_client = self.create_client(GenerateJointTrajectory, trajectory_service_name)
        self._controller_client = ActionClient(self, FollowJointTrajectory, controller_action_name)
        self._trace_pub = self.create_publisher(String, trace_event_topic, 20)
        self._trace_goal_context_pub = self.create_publisher(String, trace_goal_context_topic, 20)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(PoseStamped, target_topic, self._on_target, 20)
        self.create_subscription(JointState, joint_state_topic, self._on_joint_state, 50)
        self._timer = self.create_timer(max(0.02, float(check_period)), self._on_timer)
        if self._status_log_period > 0.0:
            self._status_timer = self.create_timer(self._status_log_period, self._log_status)

    def _load_joint_limits(self) -> Dict[str, Dict[str, float]]:
        path = get_package_share_directory("rc_arm_ruckig_config") + "/config/rc_arm_2/joint_limits.yaml"
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        limits = data.get("joint_limits", {})
        return {name: limits.get(name, {}) for name in self._joint_names}

    def _trace_id_from_target(self, target: PoseStamped) -> str:
        stamp = target.header.stamp
        if int(stamp.sec) == 0 and int(stamp.nanosec) == 0:
            return f"local-{time.monotonic_ns()}"
        return trace_id_from_stamp(stamp)

    def _trace_event(
        self,
        event_name: str,
        target: PoseStamped,
        *,
        pitch_rad: Optional[float] = None,
        extra: Optional[Dict[str, object]] = None,
    ) -> None:
        publish_trace_event(
            self._trace_pub,
            build_trace_event(
                trace_id=self._trace_id_from_target(target),
                event=event_name,
                source="target_pose_ruckig_executor",
                node=self.get_name(),
                event_ns=time.time_ns(),
                target_fields=target_fields_from_values(
                    target.pose.position.x,
                    target.pose.position.y,
                    target.pose.position.z,
                    j4_rad=pitch_rad,
                ),
                extra=extra,
            ),
        )
        self._last_event = event_name
        self._last_event_time_ns = time.time_ns()

    def _publish_trace_context(self, target: PoseStamped) -> None:
        msg = String()
        msg.data = self._trace_id_from_target(target)
        self._trace_goal_context_pub.publish(msg)

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

    def _on_target(self, msg: PoseStamped) -> None:
        pose_msg = self._resolve_target_pose(msg)
        with self._target_lock:
            self._latest_target = pose_msg
        self._trace_event("executor_target_rx", pose_msg)

    def _on_joint_state(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        mapping = dict(self._latest_joint_state)
        for index, name in enumerate(msg.name):
            if index >= len(msg.position):
                continue
            entry = mapping.setdefault(name, {})
            entry["position"] = float(msg.position[index])
            if index < len(msg.velocity):
                entry["velocity"] = float(msg.velocity[index])
        self._latest_joint_state = mapping

    def _current_joint_vectors(self) -> Optional[Tuple[List[float], List[float], List[float]]]:
        position = []
        velocity = []
        acceleration = []
        for name in self._joint_names:
            entry = self._latest_joint_state.get(name)
            if not entry or "position" not in entry:
                return None
            position.append(float(entry["position"]))
            velocity.append(float(entry.get("velocity", 0.0)))
            acceleration.append(0.0)
        return position, velocity, acceleration

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
        return pos_delta > self._position_threshold or rot_delta > self._rotation_threshold

    def _extract_target_pitch_rad(self, target: PoseStamped) -> float:
        q = target.pose.orientation
        return self._kinematics.world_pitch_from_quaternion((float(q.x), float(q.y), float(q.z), float(q.w)))

    def _goal_within_tolerance(self, current: List[float], target: Dict[str, float]) -> bool:
        return all(abs(cur - target[name]) <= self._goal_tolerance for cur, name in zip(current, self._joint_names))

    def _get_current_eef_xyz(self) -> str:
        try:
            trans = self._tf_buffer.lookup_transform(
                self._status_base_frame,
                self._status_eef_frame,
                rclpy.time.Time(),
            )
            t = trans.transform.translation
            return "(%.3f, %.3f, %.3f)" % (t.x, t.y, t.z)
        except Exception:
            return "(NA, NA, NA)"

    def _log_status(self) -> None:
        age = (time.time_ns() - self._last_event_time_ns) / 1.0e9
        self.get_logger().info(
            "[STATE] busy=%d cancel=%d event=%s(%.2fs) eef=%s"
            % (
                1 if self._busy else 0,
                1 if self._cancel_requested else 0,
                self._last_event,
                age,
                self._get_current_eef_xyz(),
            )
        )

    def _on_timer(self) -> None:
        if not self._trajectory_client.service_is_ready() or not self._controller_client.server_is_ready():
            return

        with self._target_lock:
            target = _copy_pose_stamped(self._latest_target) if self._latest_target is not None else None

        if target is None:
            return

        if self._busy:
            if (
                not self._cancel_requested
                and self._last_dispatched_target is not None
                and self._target_changed(self._last_dispatched_target, target)
                and self._current_goal_handle is not None
            ):
                self._cancel_requested = True
                self._current_goal_handle.cancel_goal_async()
            return

        if self._last_dispatched_target is not None and not self._target_changed(self._last_dispatched_target, target):
            return

        current_state = self._current_joint_vectors()
        if current_state is None:
            return

        current_position, current_velocity, current_acceleration = current_state
        pitch_rad = self._extract_target_pitch_rad(target)
        self._trace_event("executor_solve_request", target, pitch_rad=pitch_rad)
        seed = {name: current_position[idx] for idx, name in enumerate(self._joint_names)}
        q_target = self._kinematics.solve_xyz_pitch(
            float(target.pose.position.x),
            float(target.pose.position.y),
            float(target.pose.position.z),
            pitch_rad,
            seed_joints=seed,
        )
        if q_target is None:
            self._trace_event("executor_solve_fail", target, pitch_rad=pitch_rad)
            return

        if self._goal_within_tolerance(current_position, q_target):
            self._trace_event("skipped_unchanged", target, pitch_rad=pitch_rad)
            self._last_dispatched_target = target
            return

        self._trace_event("executor_solve_ok", target, pitch_rad=pitch_rad)
        self._publish_trace_context(target)

        request = GenerateJointTrajectory.Request()
        request.joint_names = list(self._joint_names)
        request.current_position = list(current_position)
        request.current_velocity = list(current_velocity)
        request.current_acceleration = list(current_acceleration)
        request.target_position = [float(q_target[name]) for name in self._joint_names]
        request.target_velocity = [0.0] * len(self._joint_names)
        request.target_acceleration = [0.0] * len(self._joint_names)
        request.max_velocity = [
            float(self._joint_limits[name]["max_velocity"]) * self._velocity_scale for name in self._joint_names
        ]
        request.max_acceleration = [
            float(self._joint_limits[name]["max_acceleration"]) * self._acceleration_scale for name in self._joint_names
        ]
        request.max_jerk = [
            float(self._joint_limits[name]["max_jerk"]) * self._jerk_scale for name in self._joint_names
        ]
        request.control_period = self._control_period
        request.minimum_duration = self._control_period

        self._busy = True
        self._cancel_requested = False
        self._active_trace_target = target
        self._last_dispatched_target = target
        future = self._trajectory_client.call_async(request)
        future.add_done_callback(self._on_trajectory_ready)

    def _on_trajectory_ready(self, future) -> None:
        target = self._active_trace_target
        if target is None:
            self._busy = False
            return
        try:
            response = future.result()
        except Exception as exc:
            self._trace_event("executor_exec_fail", target, extra={"error": str(exc)})
            self._busy = False
            self._active_trace_target = None
            return

        if not response.success or not response.trajectory.points:
            self._trace_event("executor_exec_fail", target, extra={"error": response.message})
            self._busy = False
            self._active_trace_target = None
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = response.trajectory
        self._trace_event("executor_goal_send", target)
        send_future = self._controller_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        target = self._active_trace_target
        if target is None:
            self._busy = False
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._trace_event("executor_exec_fail", target, extra={"error": str(exc)})
            self._busy = False
            self._active_trace_target = None
            return

        if goal_handle is None or not goal_handle.accepted:
            self._trace_event("executor_goal_rejected", target)
            self._busy = False
            self._active_trace_target = None
            return

        self._current_goal_handle = goal_handle
        self._trace_event("executor_goal_accepted", target)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        target = self._active_trace_target
        self._busy = False
        self._cancel_requested = False
        self._current_goal_handle = None
        self._active_trace_target = None
        if target is None:
            return
        try:
            result = future.result().result
        except Exception as exc:
            self._trace_event("executor_exec_fail", target, extra={"error": str(exc)})
            return

        if int(result.error_code) == 0:
            self._trace_event("executor_exec_ok", target)
        else:
            self._trace_event(
                "executor_exec_fail",
                target,
                extra={"error_code": int(result.error_code), "message": result.error_string},
            )


def parse_args():
    parser = argparse.ArgumentParser(description="target_pose -> Ruckig executor")
    parser.add_argument("--target-topic", default="/rc_arm_2/target_pose")
    parser.add_argument("--joint-names", default="j1_joint,j2_joint,j3_joint,j4_joint")
    parser.add_argument("--default-frame", default="world")
    parser.add_argument("--pos-threshold", default="0.003")
    parser.add_argument("--rot-threshold", default="0.03")
    parser.add_argument("--velocity-scale", default="0.8")
    parser.add_argument("--acceleration-scale", default="0.8")
    parser.add_argument("--jerk-scale", default="0.8")
    parser.add_argument("--goal-tolerance", default="0.02")
    parser.add_argument("--control-period", default="0.02")
    parser.add_argument("--check-period", default="0.05")
    parser.add_argument("--j4-axis", default="x")
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--urdf-path", default="")
    parser.add_argument("--status-log-period", default="1.0")
    parser.add_argument("--status-base-frame", default="world")
    parser.add_argument("--status-eef-frame", default="end_effector")
    parser.add_argument("--trace-event-topic", default="/rc_arm_2/trace_event")
    parser.add_argument("--trace-goal-context-topic", default="/rc_arm_2/trace_goal_context")
    parser.add_argument("--trajectory-service-name", default="/rc_arm_ruckig_trajectory_server/generate_joint_trajectory")
    parser.add_argument("--controller-action-name", default="/arm_controller/follow_joint_trajectory")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = TargetPoseRuckigExecutor(
        target_topic=args.target_topic,
        joint_names=[part.strip() for part in args.joint_names.split(",") if part.strip()],
        default_frame=args.default_frame,
        position_threshold=float(args.pos_threshold),
        rotation_threshold=float(args.rot_threshold),
        velocity_scale=float(args.velocity_scale),
        acceleration_scale=float(args.acceleration_scale),
        jerk_scale=float(args.jerk_scale),
        goal_tolerance=float(args.goal_tolerance),
        control_period=float(args.control_period),
        check_period=float(args.check_period),
        j4_axis=args.j4_axis,
        joint_state_topic=args.joint_state_topic,
        urdf_path=args.urdf_path,
        status_log_period=float(args.status_log_period),
        status_base_frame=args.status_base_frame,
        status_eef_frame=args.status_eef_frame,
        trace_event_topic=args.trace_event_topic,
        trace_goal_context_topic=args.trace_goal_context_topic,
        trajectory_service_name=args.trajectory_service_name,
        controller_action_name=args.controller_action_name,
    )
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
