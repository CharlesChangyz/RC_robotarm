#!/usr/bin/env python3
"""Sequential middleware for TF-driven rc_arm_2 tasks."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from tf2_msgs.msg import TFMessage
import yaml

from arm_msgs.msg import Arm2TargetPoint


TERMINAL_STATUSES = {
    GoalStatus.STATUS_SUCCEEDED,
    GoalStatus.STATUS_ABORTED,
    GoalStatus.STATUS_CANCELED,
}


class MiddlewareState(str, Enum):
    IDLE = "IDLE"
    STARTING_SET = "STARTING_SET"
    EXECUTING_STEP = "EXECUTING_STEP"
    WAITING_MOTION_TERMINAL = "WAITING_MOTION_TERMINAL"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class TargetPoint:
    x: float
    y: float
    z: float
    target_spin_deg: float


@dataclass(frozen=True)
class ActionStep:
    step_type: str
    label: str
    target_spin_deg: float = 0.0
    offset_xyz: Optional[Tuple[float, float, float]] = None
    xyz: Optional[Tuple[float, float, float]] = None
    enabled: Optional[bool] = None


@dataclass(frozen=True)
class ActionSet:
    action_id: int
    name: str
    steps: Tuple[ActionStep, ...]


@dataclass
class StepResult:
    index: int
    label: str
    success: bool
    detail: str


@dataclass
class ActiveRun:
    action_set: ActionSet
    step_index: int = 0
    current_step: Optional[ActionStep] = None
    waiting_goal_id: Optional[Tuple[int, ...]] = None
    waiting_known_goal_ids: set[Tuple[int, ...]] = field(default_factory=set)
    waiting_started_ns: Optional[int] = None
    had_failures: bool = False
    results: List[StepResult] = field(default_factory=list)


def _share_path(package_name: str, *parts: str) -> Path:
    return Path(get_package_share_directory(package_name)).joinpath(*parts)


def _goal_key(uuid_values: Sequence[int]) -> Tuple[int, ...]:
    return tuple(int(v) for v in uuid_values)


def _stamp_to_ns(sec: int, nanosec: int) -> int:
    return int(sec) * 1_000_000_000 + int(nanosec)


def _normalize_axis(axis: str) -> str:
    axis = (axis or "x").strip().lower()
    return axis if axis in {"x", "y", "z"} else "x"


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


def _status_name(status: int) -> str:
    mapping = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
    }
    return mapping.get(status, f"STATUS_{status}")


class Arm2MiddlewareNode(Node):
    def __init__(self) -> None:
        super().__init__("arm2_middleware")

        default_config = str(_share_path("rc_arm2_middleware", "config", "action_sets.yaml"))
        self.declare_parameter("action_sets_file", default_config)
        self.declare_parameter("target_point_topic", "/arm2/middleware/target_point")
        self.declare_parameter("run_action_set_topic", "/arm2/middleware/run_action_set")
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("parent_frame", "world")
        self.declare_parameter("child_frame", "rc_arm_2_target")
        self.declare_parameter("vacuum_topic", "/rc_arm_2/vacuum_activate")
        self.declare_parameter("payload_active_topic", "/rc_arm_2/payload_active")
        self.declare_parameter(
            "controller_status_topic",
            "/arm_controller/follow_joint_trajectory/_action/status",
        )
        self.declare_parameter("j4_axis", "x")
        self.declare_parameter("motion_wait_timeout_sec", 30.0)

        self._action_sets_file = Path(self.get_parameter("action_sets_file").value)
        self._target_point_topic = str(self.get_parameter("target_point_topic").value)
        self._run_action_set_topic = str(self.get_parameter("run_action_set_topic").value)
        self._tf_topic = str(self.get_parameter("tf_topic").value)
        self._parent_frame = str(self.get_parameter("parent_frame").value)
        self._child_frame = str(self.get_parameter("child_frame").value)
        self._vacuum_topic = str(self.get_parameter("vacuum_topic").value)
        self._payload_active_topic = str(self.get_parameter("payload_active_topic").value)
        self._controller_status_topic = str(self.get_parameter("controller_status_topic").value)
        self._j4_axis = _normalize_axis(str(self.get_parameter("j4_axis").value))
        self._motion_wait_timeout = max(0.0, float(self.get_parameter("motion_wait_timeout_sec").value))

        self._state = MiddlewareState.IDLE
        self._cached_target_point: Optional[TargetPoint] = None
        self._payload_active = False
        self._active_run: Optional[ActiveRun] = None
        self._latest_goal_statuses: Dict[Tuple[int, ...], Tuple[int, int]] = {}

        self._action_sets = self._load_action_sets(self._action_sets_file)

        self._tf_pub = self.create_publisher(TFMessage, self._tf_topic, 10)
        self._vacuum_pub = self.create_publisher(Bool, self._vacuum_topic, 10)
        self.create_subscription(Arm2TargetPoint, self._target_point_topic, self._on_target_point, 20)
        self.create_subscription(Int32, self._run_action_set_topic, self._on_run_action_set, 10)
        self.create_subscription(Bool, self._payload_active_topic, self._on_payload_active, 10)
        self.create_subscription(
            GoalStatusArray,
            self._controller_status_topic,
            self._on_controller_status,
            20,
        )
        self.create_timer(0.1, self._on_timer)

        self.get_logger().info(
            "arm2_middleware ready: action_sets=%s target_point=%s run_action_set=%s tf=%s vacuum=%s status=%s"
            % (
                sorted(self._action_sets.keys()),
                self._target_point_topic,
                self._run_action_set_topic,
                self._tf_topic,
                self._vacuum_topic,
                self._controller_status_topic,
            )
        )

    def _load_action_sets(self, config_path: Path) -> Dict[int, ActionSet]:
        if not config_path.is_file():
            raise FileNotFoundError(f"action set config not found: {config_path}")

        loaded = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        raw_sets = loaded.get("action_sets", [])
        if not isinstance(raw_sets, list):
            raise ValueError("action_sets must be a list")

        action_sets: Dict[int, ActionSet] = {}
        for raw_set in raw_sets:
            action_set = self._parse_action_set(raw_set)
            if action_set.action_id in action_sets:
                raise ValueError(f"duplicate action set id: {action_set.action_id}")
            action_sets[action_set.action_id] = action_set

        if not action_sets:
            raise ValueError("no action_sets defined")
        return action_sets

    def _parse_action_set(self, raw_set: object) -> ActionSet:
        if not isinstance(raw_set, dict):
            raise ValueError(f"invalid action set entry: {raw_set!r}")

        action_id = int(raw_set["id"])
        name = str(raw_set.get("name", f"action_set_{action_id}"))
        raw_steps = raw_set.get("steps", [])
        if not isinstance(raw_steps, list) or not raw_steps:
            raise ValueError(f"action set {action_id} has no steps")

        steps = tuple(self._parse_step(raw_step) for raw_step in raw_steps)
        return ActionSet(action_id=action_id, name=name, steps=steps)

    def _parse_step(self, raw_step: object) -> ActionStep:
        if not isinstance(raw_step, dict):
            raise ValueError(f"invalid step entry: {raw_step!r}")

        step_type = str(raw_step["type"]).strip()
        label = str(raw_step.get("label", step_type))
        target_spin_deg = float(raw_step.get("target_spin", 0.0))

        if step_type == "set_vacuum":
            return ActionStep(
                step_type=step_type,
                label=label,
                enabled=bool(raw_step["enabled"]),
            )

        if step_type == "move_target_offset":
            return ActionStep(
                step_type=step_type,
                label=label,
                target_spin_deg=target_spin_deg,
                offset_xyz=self._parse_xyz(raw_step.get("offset_xyz"), "offset_xyz"),
            )

        if step_type == "move_fixed_pose":
            return ActionStep(
                step_type=step_type,
                label=label,
                target_spin_deg=target_spin_deg,
                xyz=self._parse_xyz(raw_step.get("xyz"), "xyz"),
            )

        raise ValueError(f"unsupported step type: {step_type}")

    def _parse_xyz(self, raw_xyz: object, field_name: str) -> Tuple[float, float, float]:
        if not isinstance(raw_xyz, (list, tuple)) or len(raw_xyz) != 3:
            raise ValueError(f"{field_name} must be a length-3 list")
        return (float(raw_xyz[0]), float(raw_xyz[1]), float(raw_xyz[2]))

    def _set_state(self, new_state: MiddlewareState, detail: str) -> None:
        if self._state == new_state:
            self.get_logger().info(f"state={new_state.value} detail={detail}")
            return
        old_state = self._state
        self._state = new_state
        self.get_logger().info(f"state {old_state.value} -> {new_state.value} detail={detail}")

    def _on_target_point(self, msg: Arm2TargetPoint) -> None:
        self._cached_target_point = TargetPoint(
            x=float(msg.xyz.x),
            y=float(msg.xyz.y),
            z=float(msg.xyz.z),
            target_spin_deg=float(msg.target_spin_deg),
        )
        self.get_logger().info(
            "cached target point x=%.4f y=%.4f z=%.4f spin=%.2f deg"
            % (
                self._cached_target_point.x,
                self._cached_target_point.y,
                self._cached_target_point.z,
                self._cached_target_point.target_spin_deg,
            )
        )

    def _on_run_action_set(self, msg: Int32) -> None:
        requested_id = int(msg.data)
        if self._state != MiddlewareState.IDLE:
            self.get_logger().warn(
                "ignoring action set %d while busy in state=%s"
                % (requested_id, self._state.value)
            )
            return

        action_set = self._action_sets.get(requested_id)
        if action_set is None:
            self.get_logger().warn(f"unknown action set id={requested_id}")
            return

        self._active_run = ActiveRun(action_set=action_set)
        self._set_state(
            MiddlewareState.STARTING_SET,
            f"starting action_set id={action_set.action_id} name={action_set.name}",
        )
        self._start_next_step()

    def _on_payload_active(self, msg: Bool) -> None:
        new_value = bool(msg.data)
        if self._payload_active != new_value:
            self._payload_active = new_value
            self.get_logger().info(f"payload_active={self._payload_active}")

    def _on_controller_status(self, msg: GoalStatusArray) -> None:
        latest: Dict[Tuple[int, ...], Tuple[int, int]] = {}
        for status_entry in msg.status_list:
            goal_id = _goal_key(status_entry.goal_info.goal_id.uuid)
            stamp_ns = _stamp_to_ns(
                status_entry.goal_info.stamp.sec,
                status_entry.goal_info.stamp.nanosec,
            )
            latest[goal_id] = (int(status_entry.status), stamp_ns)
        self._latest_goal_statuses = latest
        self._maybe_finish_motion_from_status()

    def _on_timer(self) -> None:
        if self._state != MiddlewareState.WAITING_MOTION_TERMINAL or self._active_run is None:
            return
        if self._motion_wait_timeout <= 0.0 or self._active_run.waiting_started_ns is None:
            return

        elapsed = self.get_clock().now().nanoseconds - self._active_run.waiting_started_ns
        if elapsed < int(self._motion_wait_timeout * 1_000_000_000):
            return

        self.get_logger().warn(f"motion wait timeout after {self._motion_wait_timeout:.2f} sec")
        self._finish_motion_step(
            success=False,
            detail=f"motion wait timeout after {self._motion_wait_timeout:.2f} sec",
        )

    def _start_next_step(self) -> None:
        run = self._active_run
        if run is None:
            self._set_state(MiddlewareState.IDLE, "no active run")
            return

        if run.step_index >= len(run.action_set.steps):
            summary = (
                f"action_set id={run.action_set.action_id} name={run.action_set.name} "
                f"finished failures={run.had_failures}"
            )
            self._set_state(MiddlewareState.COMPLETED, summary)
            self.get_logger().info(
                "step results: %s"
                % [
                    {
                        "index": result.index,
                        "label": result.label,
                        "success": result.success,
                        "detail": result.detail,
                    }
                    for result in run.results
                ]
            )
            self._active_run = None
            self._set_state(MiddlewareState.IDLE, "ready for next action set")
            return

        step = run.action_set.steps[run.step_index]
        run.current_step = step
        self._set_state(
            MiddlewareState.EXECUTING_STEP,
            f"action_set={run.action_set.action_id} step={run.step_index} label={step.label} type={step.step_type}",
        )

        if step.step_type == "set_vacuum":
            self._publish_vacuum(bool(step.enabled))
            self._record_step_result(True, f"vacuum set to {bool(step.enabled)}")
            run.step_index += 1
            self._start_next_step()
            return

        if step.step_type == "move_target_offset":
            if self._cached_target_point is None:
                self._fail_action_set("move_target_offset requested before target point was received")
                return
            x = self._cached_target_point.x + float(step.offset_xyz[0])
            y = self._cached_target_point.y + float(step.offset_xyz[1])
            z = self._cached_target_point.z + float(step.offset_xyz[2])
            self._publish_target_tf(x, y, z, step.target_spin_deg)
            self._enter_motion_wait(
                f"waiting on target_offset x={x:.4f} y={y:.4f} z={z:.4f} spin={step.target_spin_deg:.2f}",
            )
            return

        if step.step_type == "move_fixed_pose":
            x, y, z = step.xyz
            self._publish_target_tf(x, y, z, step.target_spin_deg)
            self._enter_motion_wait(
                f"waiting on fixed_pose x={x:.4f} y={y:.4f} z={z:.4f} spin={step.target_spin_deg:.2f}",
            )
            return

        self._fail_action_set(f"unsupported step type at runtime: {step.step_type}")

    def _enter_motion_wait(self, detail: str) -> None:
        run = self._active_run
        if run is None:
            self._fail_action_set("internal error: missing active run when entering motion wait")
            return

        run.waiting_goal_id = None
        run.waiting_known_goal_ids = set(self._latest_goal_statuses.keys())
        run.waiting_started_ns = self.get_clock().now().nanoseconds
        self._set_state(MiddlewareState.WAITING_MOTION_TERMINAL, detail)
        self._maybe_finish_motion_from_status()

    def _publish_target_tf(self, x: float, y: float, z: float, target_spin_deg: float) -> None:
        qx, qy, qz, qw = _quaternion_from_world_pitch(self._j4_axis, target_spin_deg)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._parent_frame
        transform.child_frame_id = self._child_frame
        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.translation.z = float(z)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf_pub.publish(TFMessage(transforms=[transform]))
        self.get_logger().info(
            "published TF target %s -> %s x=%.4f y=%.4f z=%.4f spin=%.2f deg"
            % (
                self._parent_frame,
                self._child_frame,
                x,
                y,
                z,
                target_spin_deg,
            )
        )

    def _publish_vacuum(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self._vacuum_pub.publish(msg)
        self.get_logger().info(f"published vacuum={enabled} on {self._vacuum_topic}")

    def _maybe_finish_motion_from_status(self) -> None:
        run = self._active_run
        if run is None or self._state != MiddlewareState.WAITING_MOTION_TERMINAL:
            return

        if run.waiting_goal_id is None:
            candidates = [
                (stamp_ns, goal_id, status)
                for goal_id, (status, stamp_ns) in self._latest_goal_statuses.items()
                if goal_id not in run.waiting_known_goal_ids
            ]
            if not candidates:
                return

            stamp_ns, goal_id, status = max(candidates, key=lambda item: item[0])
            run.waiting_goal_id = goal_id
            self.get_logger().info(
                "tracking controller goal=%s status=%s stamp_ns=%d"
                % (list(goal_id), _status_name(status), stamp_ns)
            )
            if status in TERMINAL_STATUSES:
                self._finish_motion_step(
                    success=(status == GoalStatus.STATUS_SUCCEEDED),
                    detail=f"controller goal terminal status={_status_name(status)}",
                )
            return

        snapshot = self._latest_goal_statuses.get(run.waiting_goal_id)
        if snapshot is None:
            return
        status, _stamp_ns = snapshot
        if status in TERMINAL_STATUSES:
            self._finish_motion_step(
                success=(status == GoalStatus.STATUS_SUCCEEDED),
                detail=f"controller goal terminal status={_status_name(status)}",
            )

    def _finish_motion_step(self, success: bool, detail: str) -> None:
        run = self._active_run
        if run is None:
            return
        run.waiting_goal_id = None
        run.waiting_known_goal_ids.clear()
        run.waiting_started_ns = None
        self._record_step_result(success, detail)
        run.step_index += 1
        self._start_next_step()

    def _record_step_result(self, success: bool, detail: str) -> None:
        run = self._active_run
        if run is None or run.current_step is None:
            return
        run.results.append(
            StepResult(
                index=run.step_index,
                label=run.current_step.label,
                success=bool(success),
                detail=detail,
            )
        )
        if not success:
            run.had_failures = True
        self.get_logger().info(
            "step result action_set=%d step=%d label=%s success=%s detail=%s"
            % (
                run.action_set.action_id,
                run.step_index,
                run.current_step.label,
                success,
                detail,
            )
        )

    def _fail_action_set(self, detail: str) -> None:
        run = self._active_run
        if run is None:
            self._set_state(MiddlewareState.FAILED, detail)
            self._set_state(MiddlewareState.IDLE, "ready for next action set")
            return

        if run.current_step is not None:
            self._record_step_result(False, detail)
        self._set_state(
            MiddlewareState.FAILED,
            f"action_set id={run.action_set.action_id} name={run.action_set.name} detail={detail}",
        )
        self._active_run = None
        self._set_state(MiddlewareState.IDLE, "ready for next action set")


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = Arm2MiddlewareNode()
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
