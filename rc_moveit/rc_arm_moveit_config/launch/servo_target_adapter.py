#!/usr/bin/env python3
"""Adapt target poses into constrained Servo twist commands."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from typing import Dict, Optional, Sequence

from arm_msgs.msg import Arm2MotionExecution, Arm2TargetPoint
from geometry_msgs.msg import PoseStamped, TwistStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from rc_arm_world_pitch_kinematics import RcArmWorldPitchKinematics


EXECUTION_ERROR_PREEMPTED = -1
EXECUTION_ERROR_TIMEOUT = -2
EXECUTION_ERROR_NO_FEEDBACK = -3
EXECUTION_ERROR_UNREACHABLE = -4


def _normalize_frame_id(frame_id: str) -> str:
    return (frame_id or "").strip().lstrip("/")


def _copy_pose_stamped(msg: PoseStamped) -> PoseStamped:
    copied = PoseStamped()
    copied.header = msg.header
    copied.pose = msg.pose
    return copied


def _normalize_quat_xyzw(x: float, y: float, z: float, w: float) -> tuple[float, float, float, float]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _slew_limit(target: float, current: float, max_delta: float) -> float:
    if max_delta <= 0.0:
        return target
    return current + _clamp(target - current, max_delta)


def _format_joint_map_deg(joint_map: Dict[str, float]) -> str:
    return ", ".join(
        "{}={:.1f}deg".format(name, math.degrees(float(value))) for name, value in joint_map.items()
    )


@dataclass
class TargetState:
    pose: PoseStamped
    joint_solution: Optional[Dict[str, float]]
    limit_scale: float


@dataclass
class ActiveExecution:
    execution_id: int
    target: TargetState
    start_time_sec: float
    stable_cycles: int = 0


class ServoTargetAdapter(Node):
    def __init__(
        self,
        target_topic: str,
        middleware_target_topic: str,
        middleware_result_topic: str,
        twist_topic: str,
        joint_state_topic: str,
        planning_frame: str,
        j4_axis: str,
        control_rate_hz: float,
        position_gain: float,
        pitch_gain: float,
        max_linear_velocity: float,
        max_angular_velocity: float,
        max_linear_acceleration: float,
        max_angular_acceleration: float,
        joint_limit_scale_margin: float,
        joint_limit_stop_margin: float,
        minimum_target_limit_scale: float,
        middleware_position_tolerance: float,
        middleware_pitch_tolerance: float,
        middleware_success_cycles: int,
        middleware_timeout_sec: float,
        feedback_timeout_sec: float,
        servo_ns: str,
    ) -> None:
        super().__init__("rc_arm_servo_target_adapter")

        self._target_topic = target_topic
        self._middleware_target_topic = middleware_target_topic
        self._middleware_result_topic = middleware_result_topic
        self._twist_topic = twist_topic
        self._planning_frame = _normalize_frame_id(planning_frame) or "world"
        self._j4_axis = j4_axis
        self._control_period = 1.0 / max(1.0, control_rate_hz)
        self._position_gain = max(0.0, position_gain)
        self._pitch_gain = max(0.0, pitch_gain)
        self._max_linear_velocity = max(0.01, max_linear_velocity)
        self._max_angular_velocity = max(0.01, max_angular_velocity)
        self._max_linear_acceleration = max(0.01, max_linear_acceleration)
        self._max_angular_acceleration = max(0.01, max_angular_acceleration)
        self._joint_limit_scale_margin = max(0.01, joint_limit_scale_margin)
        self._joint_limit_stop_margin = min(
            self._joint_limit_scale_margin - 1.0e-3,
            max(0.0, joint_limit_stop_margin),
        )
        self._minimum_target_limit_scale = max(0.0, min(1.0, minimum_target_limit_scale))
        self._middleware_position_tolerance = max(0.0, middleware_position_tolerance)
        self._middleware_pitch_tolerance = max(0.0, middleware_pitch_tolerance)
        self._middleware_success_cycles = max(1, middleware_success_cycles)
        self._middleware_timeout_sec = max(0.1, middleware_timeout_sec)
        self._feedback_timeout_sec = max(0.1, feedback_timeout_sec)
        self._servo_ns = servo_ns.rstrip("/")

        self._kinematics = RcArmWorldPitchKinematics(j4_axis=j4_axis)
        self._joint_names = tuple(self._kinematics.joint_map(self._kinematics.zero_joints).keys())
        self._latest_joint_map: Dict[str, float] = {}
        self._last_joint_state_time_sec = -1.0e9
        self._latest_manual_target: Optional[TargetState] = None
        self._active_execution: Optional[ActiveExecution] = None
        self._next_execution_id = 0
        self._last_linear_cmd = (0.0, 0.0, 0.0)
        self._last_angular_cmd = 0.0
        self._last_unsafe_target_log_sec = -1.0e9

        self._servo_running = False
        self._start_future = None

        self._twist_pub = self.create_publisher(TwistStamped, twist_topic, 20)
        self._motion_execution_pub = self.create_publisher(
            Arm2MotionExecution, middleware_result_topic, 20
        )

        self.create_subscription(PoseStamped, target_topic, self._on_manual_target, 20)
        self.create_subscription(
            Arm2TargetPoint, middleware_target_topic, self._on_middleware_target, 20
        )
        self.create_subscription(JointState, joint_state_topic, self._on_joint_state, 50)

        self._start_servo_client = self.create_client(Trigger, f"{self._servo_ns}/start_servo")

        self.create_timer(0.5, self._configure_servo)
        self.create_timer(self._control_period, self._control_loop)

        self.get_logger().info(
            "Servo target adapter ready target=%s middleware=%s twist=%s planning_frame=%s"
            % (
                target_topic,
                middleware_target_topic,
                twist_topic,
                self._planning_frame,
            )
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _configure_servo(self) -> None:
        if self._servo_running:
            return
        if self._start_future is not None and not self._start_future.done():
            return
        if not self._start_servo_client.wait_for_service(timeout_sec=0.0):
            return
        self._start_future = self._start_servo_client.call_async(Trigger.Request())
        self._start_future.add_done_callback(self._on_start_servo_done)

    def _on_start_servo_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"start_servo failed: {exc}")
            return
        if response.success:
            self._servo_running = True
            self.get_logger().info("Servo loop started")
        else:
            self.get_logger().warn(f"Servo start rejected: {response.message}")

    def _on_joint_state(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        latest = self._latest_joint_map.copy()
        for idx, name in enumerate(msg.name):
            if idx < len(msg.position):
                latest[name] = float(msg.position[idx])
        self._latest_joint_map = latest
        self._last_joint_state_time_sec = self._now_sec()

    def _on_manual_target(self, msg: PoseStamped) -> None:
        pose_msg = _copy_pose_stamped(msg)
        pose_msg.header.frame_id = _normalize_frame_id(pose_msg.header.frame_id) or self._planning_frame
        q = pose_msg.pose.orientation
        qx, qy, qz, qw = _normalize_quat_xyzw(float(q.x), float(q.y), float(q.z), float(q.w))
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        target_state = self._build_target_state(pose_msg)
        if target_state.joint_solution is not None:
            self._latest_manual_target = target_state
        else:
            current_joint_map = self._current_joint_map()
            hold_target = self._build_hold_current_target_state()
            if hold_target is not None:
                self._latest_manual_target = hold_target
                self._reset_command_state()
                self._log_unsafe_target(
                    "unreachable manual target; holding current pose",
                    current_joint_map=current_joint_map,
                    target_state=target_state,
                )
            else:
                self._log_unsafe_target(
                    "ignoring unreachable manual target",
                    current_joint_map=current_joint_map,
                    target_state=target_state,
                )

    def _on_middleware_target(self, msg: Arm2TargetPoint) -> None:
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self._planning_frame
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = float(msg.xyz.x)
        pose_msg.pose.position.y = float(msg.xyz.y)
        pose_msg.pose.position.z = float(msg.xyz.z)
        qx, qy, qz, qw = self._kinematics.quaternion_from_world_pitch(
            math.radians(float(msg.target_spin_deg))
        )
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        if self._active_execution is not None:
            self._publish_motion_execution(
                self._active_execution.execution_id,
                Arm2MotionExecution.STATUS_FAILED,
                EXECUTION_ERROR_PREEMPTED,
                "superseded by newer middleware target",
            )

        target_state = self._build_target_state(pose_msg)
        self._next_execution_id += 1
        self._active_execution = ActiveExecution(
            execution_id=self._next_execution_id,
            target=target_state,
            start_time_sec=self._now_sec(),
        )
        self._publish_motion_execution(
            self._active_execution.execution_id,
            Arm2MotionExecution.STATUS_ACCEPTED,
            0,
            "accepted",
        )

        if not self._target_is_safe(target_state):
            self._publish_motion_execution(
                self._active_execution.execution_id,
                Arm2MotionExecution.STATUS_FAILED,
                EXECUTION_ERROR_UNREACHABLE,
                "target is unreachable or too close to joint limits",
            )
            self._active_execution = None

    def _publish_motion_execution(
        self, execution_id: int, status: int, error_code: int, detail: str
    ) -> None:
        msg = Arm2MotionExecution()
        msg.execution_id = int(execution_id)
        msg.status = int(status)
        msg.error_code = int(error_code)
        msg.detail = str(detail)
        self._motion_execution_pub.publish(msg)

    def _current_joint_map(self) -> Optional[Dict[str, float]]:
        missing = [name for name in self._joint_names if name not in self._latest_joint_map]
        if missing:
            return None
        return {name: self._latest_joint_map[name] for name in self._joint_names}

    def _active_target(self) -> Optional[TargetState]:
        if self._active_execution is not None:
            return self._active_execution.target
        return self._latest_manual_target

    def _current_seed_joint_map(self) -> Dict[str, float]:
        current = self._current_joint_map()
        if current is not None:
            return current
        return self._kinematics.zero_home_joint_map()

    def _joint_limit_scale(self, joint_solution: Dict[str, float]) -> float:
        min_scale = 1.0
        for index, joint_name in enumerate(self._joint_names):
            value = float(joint_solution[joint_name])
            lower = float(self._kinematics.lower_limits[index])
            upper = float(self._kinematics.upper_limits[index])
            distance = min(value - lower, upper - value)
            if distance <= self._joint_limit_stop_margin:
                return 0.0
            if distance >= self._joint_limit_scale_margin:
                continue
            ratio = (distance - self._joint_limit_stop_margin) / (
                self._joint_limit_scale_margin - self._joint_limit_stop_margin
            )
            min_scale = min(min_scale, max(0.0, min(1.0, ratio)))
        return min_scale

    def _build_target_state(self, pose_msg: PoseStamped) -> TargetState:
        seed_joint_map = self._current_seed_joint_map()
        target_pitch = self._kinematics.world_pitch_from_quaternion(
            (
                float(pose_msg.pose.orientation.x),
                float(pose_msg.pose.orientation.y),
                float(pose_msg.pose.orientation.z),
                float(pose_msg.pose.orientation.w),
            )
        )
        joint_solution = self._kinematics.solve_xyz_pitch(
            float(pose_msg.pose.position.x),
            float(pose_msg.pose.position.y),
            float(pose_msg.pose.position.z),
            target_pitch,
            seed_joints=seed_joint_map,
        )
        if joint_solution is None:
            return TargetState(pose=pose_msg, joint_solution=None, limit_scale=0.0)
        return TargetState(
            pose=pose_msg,
            joint_solution=joint_solution,
            limit_scale=self._joint_limit_scale(joint_solution),
        )

    def _build_hold_current_target_state(self) -> Optional[TargetState]:
        joint_map = self._current_joint_map()
        if joint_map is None:
            return None
        current_xyz = self._kinematics.forward_position(joint_map)
        current_pitch = self._kinematics.forward_world_pitch(joint_map)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self._planning_frame
        pose_msg.pose.position.x = float(current_xyz[0])
        pose_msg.pose.position.y = float(current_xyz[1])
        pose_msg.pose.position.z = float(current_xyz[2])
        qx, qy, qz, qw = self._kinematics.quaternion_from_world_pitch(current_pitch)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        return TargetState(
            pose=pose_msg,
            joint_solution=dict(joint_map),
            limit_scale=1.0,
        )

    def _target_is_safe(self, target_state: TargetState) -> bool:
        return (
            target_state.joint_solution is not None
            and target_state.limit_scale >= self._minimum_target_limit_scale
        )

    def _target_pushes_into_current_limit(
        self,
        current_joint_map: Dict[str, float],
        target_state: TargetState,
    ) -> Optional[str]:
        if target_state.joint_solution is None:
            return None
        for index, joint_name in enumerate(self._joint_names):
            current_value = float(current_joint_map[joint_name])
            target_value = float(target_state.joint_solution[joint_name])
            lower = float(self._kinematics.lower_limits[index])
            upper = float(self._kinematics.upper_limits[index])
            dist_lower = current_value - lower
            dist_upper = upper - current_value
            moving_toward_lower = target_value < current_value - 1.0e-4
            moving_toward_upper = target_value > current_value + 1.0e-4
            if dist_lower <= self._joint_limit_scale_margin and dist_lower <= dist_upper and moving_toward_lower:
                return (
                    "{} -> lower limit; current={:.1f}deg target={:.1f}deg margin={:.1f}deg lower={:.1f}deg"
                ).format(
                    joint_name,
                    math.degrees(current_value),
                    math.degrees(target_value),
                    math.degrees(dist_lower),
                    math.degrees(lower),
                )
            if dist_upper <= self._joint_limit_scale_margin and dist_upper < dist_lower and moving_toward_upper:
                return (
                    "{} -> upper limit; current={:.1f}deg target={:.1f}deg margin={:.1f}deg upper={:.1f}deg"
                ).format(
                    joint_name,
                    math.degrees(current_value),
                    math.degrees(target_value),
                    math.degrees(dist_upper),
                    math.degrees(upper),
                )
        return None

    def _log_unsafe_target(
        self,
        message: str,
        current_joint_map: Optional[Dict[str, float]] = None,
        target_state: Optional[TargetState] = None,
        reason: str = "",
    ) -> None:
        now_sec = self._now_sec()
        if (now_sec - self._last_unsafe_target_log_sec) < 1.0:
            return
        self._last_unsafe_target_log_sec = now_sec
        parts = [message]
        if reason:
            parts.append(reason)
        if current_joint_map is not None:
            current_xyz = self._kinematics.forward_position(current_joint_map)
            current_pitch = math.degrees(self._kinematics.forward_world_pitch(current_joint_map))
            parts.append(
                "current xyz=({:.3f}, {:.3f}, {:.3f}) pitch={:.1f}deg joints=[{}]".format(
                    current_xyz[0],
                    current_xyz[1],
                    current_xyz[2],
                    current_pitch,
                    _format_joint_map_deg(current_joint_map),
                )
            )
        if target_state is not None:
            target_pitch = math.degrees(
                self._kinematics.world_pitch_from_quaternion(
                    (
                        float(target_state.pose.pose.orientation.x),
                        float(target_state.pose.pose.orientation.y),
                        float(target_state.pose.pose.orientation.z),
                        float(target_state.pose.pose.orientation.w),
                    )
                )
            )
            parts.append(
                "target xyz=({:.3f}, {:.3f}, {:.3f}) pitch={:.1f}deg limit_scale={:.2f}".format(
                    float(target_state.pose.pose.position.x),
                    float(target_state.pose.pose.position.y),
                    float(target_state.pose.pose.position.z),
                    target_pitch,
                    target_state.limit_scale,
                )
            )
            if target_state.joint_solution is not None:
                parts.append("target_joints=[{}]".format(_format_joint_map_deg(target_state.joint_solution)))
        self.get_logger().warn(" | ".join(parts))

    def _publish_twist(
        self,
        linear_x: float,
        linear_y: float,
        linear_z: float,
        angular_axis: float,
    ) -> None:
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self._planning_frame
        twist_msg.twist.linear.x = linear_x
        twist_msg.twist.linear.y = linear_y
        twist_msg.twist.linear.z = linear_z
        if self._j4_axis == "x":
            twist_msg.twist.angular.x = angular_axis
        elif self._j4_axis == "y":
            twist_msg.twist.angular.y = angular_axis
        else:
            twist_msg.twist.angular.z = angular_axis
        self._twist_pub.publish(twist_msg)

    def _reset_command_state(self) -> None:
        self._last_linear_cmd = (0.0, 0.0, 0.0)
        self._last_angular_cmd = 0.0

    def _rate_limit_command(
        self,
        linear_x: float,
        linear_y: float,
        linear_z: float,
        angular_axis: float,
    ) -> tuple[float, float, float, float]:
        max_linear_delta = self._max_linear_acceleration * self._control_period
        max_angular_delta = self._max_angular_acceleration * self._control_period
        limited_x = _slew_limit(linear_x, self._last_linear_cmd[0], max_linear_delta)
        limited_y = _slew_limit(linear_y, self._last_linear_cmd[1], max_linear_delta)
        limited_z = _slew_limit(linear_z, self._last_linear_cmd[2], max_linear_delta)
        limited_angular = _slew_limit(angular_axis, self._last_angular_cmd, max_angular_delta)
        self._last_linear_cmd = (limited_x, limited_y, limited_z)
        self._last_angular_cmd = limited_angular
        return limited_x, limited_y, limited_z, limited_angular

    def _control_loop(self) -> None:
        target = self._active_target()
        if target is None or not self._servo_running:
            self._reset_command_state()
            return

        joint_map = self._current_joint_map()
        now_sec = self._now_sec()
        if joint_map is None or (now_sec - self._last_joint_state_time_sec) > self._feedback_timeout_sec:
            if (
                self._active_execution is not None
                and (now_sec - self._active_execution.start_time_sec) > self._feedback_timeout_sec
            ):
                self._publish_motion_execution(
                    self._active_execution.execution_id,
                    Arm2MotionExecution.STATUS_FAILED,
                    EXECUTION_ERROR_NO_FEEDBACK,
                    "joint feedback unavailable",
                )
                self._active_execution = None
            self._reset_command_state()
            return

        if target.joint_solution is None:
            self._reset_command_state()
            return

        limit_push_reason = self._target_pushes_into_current_limit(joint_map, target)
        if limit_push_reason:
            hold_target = self._build_hold_current_target_state()
            if hold_target is not None:
                if self._active_execution is not None:
                    self._publish_motion_execution(
                        self._active_execution.execution_id,
                        Arm2MotionExecution.STATUS_FAILED,
                        EXECUTION_ERROR_UNREACHABLE,
                        "target pushes further into current joint limit",
                    )
                    self._active_execution = None
                else:
                    self._latest_manual_target = hold_target
                self._reset_command_state()
                self._log_unsafe_target(
                    "target pushes into current joint limit; holding current pose",
                    current_joint_map=joint_map,
                    target_state=target,
                    reason=limit_push_reason,
                )
            return

        current_xyz = self._kinematics.forward_position(joint_map)
        current_pitch = self._kinematics.forward_world_pitch(joint_map)
        target_pitch = self._kinematics.world_pitch_from_quaternion(
            (
                float(target.pose.pose.orientation.x),
                float(target.pose.pose.orientation.y),
                float(target.pose.pose.orientation.z),
                float(target.pose.pose.orientation.w),
            )
        )

        err_x = float(target.pose.pose.position.x) - current_xyz[0]
        err_y = float(target.pose.pose.position.y) - current_xyz[1]
        err_z = float(target.pose.pose.position.z) - current_xyz[2]
        err_pitch = math.atan2(
            math.sin(target_pitch - current_pitch), math.cos(target_pitch - current_pitch)
        )

        if self._active_execution is not None:
            pos_error = math.sqrt(err_x * err_x + err_y * err_y + err_z * err_z)
            if (
                pos_error <= self._middleware_position_tolerance
                and abs(err_pitch) <= self._middleware_pitch_tolerance
            ):
                self._active_execution.stable_cycles += 1
                if self._active_execution.stable_cycles >= self._middleware_success_cycles:
                    self._publish_motion_execution(
                        self._active_execution.execution_id,
                        Arm2MotionExecution.STATUS_SUCCEEDED,
                        0,
                        "target reached",
                    )
                    self._active_execution = None
                    return
            else:
                self._active_execution.stable_cycles = 0

            if (now_sec - self._active_execution.start_time_sec) > self._middleware_timeout_sec:
                self._publish_motion_execution(
                    self._active_execution.execution_id,
                    Arm2MotionExecution.STATUS_FAILED,
                    EXECUTION_ERROR_TIMEOUT,
                    "timeout waiting for target convergence",
                )
                self._active_execution = None
                return

        linear_x = _clamp(self._position_gain * err_x, self._max_linear_velocity)
        linear_y = _clamp(self._position_gain * err_y, self._max_linear_velocity)
        linear_z = _clamp(self._position_gain * err_z, self._max_linear_velocity)
        angular_axis = _clamp(self._pitch_gain * err_pitch, self._max_angular_velocity)
        linear_x *= target.limit_scale
        linear_y *= target.limit_scale
        linear_z *= target.limit_scale
        angular_axis *= target.limit_scale
        linear_x, linear_y, linear_z, angular_axis = self._rate_limit_command(
            linear_x, linear_y, linear_z, angular_axis
        )

        if (
            abs(linear_x) < 1.0e-4
            and abs(linear_y) < 1.0e-4
            and abs(linear_z) < 1.0e-4
            and abs(angular_axis) < 1.0e-4
        ):
            return

        self._publish_twist(linear_x, linear_y, linear_z, angular_axis)


def parse_args(args: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Adapt target poses into Servo twist commands")
    parser.add_argument("--target-topic", default="/rc_arm_2/target_pose")
    parser.add_argument("--middleware-target-topic", default="/arm2/middleware/motion_target")
    parser.add_argument("--middleware-result-topic", default="/arm2/middleware/motion_execution")
    parser.add_argument("--twist-topic", default="/servo_node/delta_twist_cmds")
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--planning-frame", default="base_link")
    parser.add_argument("--j4-axis", choices=["x", "y", "z"], default="x")
    parser.add_argument("--control-rate-hz", type=float, default=100.0)
    parser.add_argument("--position-gain", type=float, default=1.8)
    parser.add_argument("--pitch-gain", type=float, default=2.0)
    parser.add_argument("--max-linear-velocity", type=float, default=0.12)
    parser.add_argument("--max-angular-velocity", type=float, default=0.7)
    parser.add_argument("--max-linear-acceleration", type=float, default=0.18)
    parser.add_argument("--max-angular-acceleration", type=float, default=0.7)
    parser.add_argument("--joint-limit-scale-margin", type=float, default=0.28)
    parser.add_argument("--joint-limit-stop-margin", type=float, default=0.18)
    parser.add_argument("--minimum-target-limit-scale", type=float, default=0.45)
    parser.add_argument("--middleware-position-tolerance", type=float, default=0.003)
    parser.add_argument("--middleware-pitch-tolerance", type=float, default=0.03)
    parser.add_argument("--middleware-success-cycles", type=int, default=5)
    parser.add_argument("--middleware-timeout-sec", type=float, default=30.0)
    parser.add_argument("--feedback-timeout-sec", type=float, default=1.0)
    parser.add_argument("--servo-ns", default="/servo_node")
    return parser.parse_args(args=args)


def main(args: Optional[Sequence[str]] = None) -> None:
    parsed = parse_args(args)
    rclpy.init(args=args)
    node = ServoTargetAdapter(
        target_topic=parsed.target_topic,
        middleware_target_topic=parsed.middleware_target_topic,
        middleware_result_topic=parsed.middleware_result_topic,
        twist_topic=parsed.twist_topic,
        joint_state_topic=parsed.joint_state_topic,
        planning_frame=parsed.planning_frame,
        j4_axis=parsed.j4_axis,
        control_rate_hz=parsed.control_rate_hz,
        position_gain=parsed.position_gain,
        pitch_gain=parsed.pitch_gain,
        max_linear_velocity=parsed.max_linear_velocity,
        max_angular_velocity=parsed.max_angular_velocity,
        max_linear_acceleration=parsed.max_linear_acceleration,
        max_angular_acceleration=parsed.max_angular_acceleration,
        joint_limit_scale_margin=parsed.joint_limit_scale_margin,
        joint_limit_stop_margin=parsed.joint_limit_stop_margin,
        minimum_target_limit_scale=parsed.minimum_target_limit_scale,
        middleware_position_tolerance=parsed.middleware_position_tolerance,
        middleware_pitch_tolerance=parsed.middleware_pitch_tolerance,
        middleware_success_cycles=parsed.middleware_success_cycles,
        middleware_timeout_sec=parsed.middleware_timeout_sec,
        feedback_timeout_sec=parsed.feedback_timeout_sec,
        servo_ns=parsed.servo_ns,
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
