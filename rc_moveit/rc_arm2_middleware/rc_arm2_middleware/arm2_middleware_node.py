#!/usr/bin/env python3
"""Sequential middleware for rc_arm_2 task execution."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Int32, UInt32
import yaml

from arm_msgs.msg import Arm2MotionExecution, Arm2TargetPoint, CanFrame
from rc_arm2_middleware.dm_serial_action_bridge import (
    DmSerialActionBridge,
    RawCanFrame,
    resolve_allowed_action_set_ids,
)


class MiddlewareState(str, Enum):
    IDLE = "IDLE"
    STARTING_SET = "STARTING_SET"
    EXECUTING_STEP = "EXECUTING_STEP"
    WAITING_MOTION_RESULT = "WAITING_MOTION_RESULT"
    TRACKING_TARGET_OFFSET = "TRACKING_TARGET_OFFSET"
    WAITING_PAYLOAD_ACTIVE = "WAITING_PAYLOAD_ACTIVE"
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
    j5_target_pos: Optional[float] = None
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
    trigger_source: str = "topic"
    trigger_can_id: Optional[int] = None
    step_index: int = 0
    current_step: Optional[ActionStep] = None
    waiting_started_ns: Optional[int] = None
    waiting_execution_baseline_id: int = 0
    waiting_execution_id: Optional[int] = None
    desired_payload_active: Optional[bool] = None
    last_target_offset_command: Optional[TargetPoint] = None
    target_offset_publish_count: int = 0
    waiting_motion_succeeded: bool = False
    waiting_j5_target_pos: Optional[float] = None
    last_j5_command_pos: Optional[float] = None
    had_failures: bool = False
    results: List[StepResult] = field(default_factory=list)


def _share_path(package_name: str, *parts: str) -> Path:
    return Path(get_package_share_directory(package_name)).joinpath(*parts)


def _motion_status_name(status: int) -> str:
    mapping = {
        Arm2MotionExecution.STATUS_ACCEPTED: "ACCEPTED",
        Arm2MotionExecution.STATUS_SUCCEEDED: "SUCCEEDED",
        Arm2MotionExecution.STATUS_FAILED: "FAILED",
    }
    return mapping.get(int(status), f"STATUS_{status}")


class Arm2MiddlewareNode(Node):
    def __init__(self) -> None:
        super().__init__("arm2_middleware")

        default_config = str(_share_path("rc_arm2_middleware", "config", "action_sets.yaml"))
        self.declare_parameter("action_sets_file", default_config)
        self.declare_parameter("target_point_topic", "/arm2/middleware/target_point")
        self.declare_parameter("run_action_set_topic", "/arm2/middleware/run_action_set")
        self.declare_parameter("motion_target_topic", "/arm2/middleware/motion_target")
        self.declare_parameter("motion_execution_topic", "/arm2/middleware/motion_execution")
        self.declare_parameter("vacuum_topic", "/rc_arm_2/vacuum_activate")
        self.declare_parameter("payload_command_topic", "/rc_arm_2/payload_active_command")
        self.declare_parameter("payload_active_topic", "/rc_arm_2/payload_active")
        self.declare_parameter("j5_command_topic", "/rc_arm_2/j5/command_position")
        self.declare_parameter("j5_position_topic", "/rc_arm_2/j5/actual_position")
        self.declare_parameter("j5_position_tolerance", 0.02)
        self.declare_parameter("laser_distance_topic", "/rc_arm_2/laser_distance")
        self.declare_parameter("laser_distance_threshold", 50)
        self.declare_parameter("motion_wait_timeout_sec", 30.0)
        self.declare_parameter("payload_wait_timeout_sec", 5.0)
        self.declare_parameter("laser_wait_timeout_sec", 30.0)
        self.declare_parameter("tracking_publish_rate_hz", 10.0)
        self.declare_parameter("dm_serial_bridge_enabled", True)
        self.declare_parameter("dm_serial_rx_topic", "/rc_arm_2/dm_serial_rx")
        self.declare_parameter("dm_serial_tx_topic", "/rc_arm_2/dm_serial_tx")
        self.declare_parameter("dm_serial_command_base_id", 0x400)
        self.declare_parameter("dm_serial_complete_id", 0x500)
        self.declare_parameter("dm_serial_allowed_action_set_ids", "")

        self._action_sets_file = Path(self.get_parameter("action_sets_file").value)
        self._target_point_topic = str(self.get_parameter("target_point_topic").value)
        self._run_action_set_topic = str(self.get_parameter("run_action_set_topic").value)
        self._motion_target_topic = str(self.get_parameter("motion_target_topic").value)
        self._motion_execution_topic = str(self.get_parameter("motion_execution_topic").value)
        self._vacuum_topic = str(self.get_parameter("vacuum_topic").value)
        self._payload_command_topic = str(self.get_parameter("payload_command_topic").value)
        self._payload_active_topic = str(self.get_parameter("payload_active_topic").value)
        self._j5_command_topic = str(self.get_parameter("j5_command_topic").value)
        self._j5_position_topic = str(self.get_parameter("j5_position_topic").value)
        self._j5_position_tolerance = max(
            0.0, float(self.get_parameter("j5_position_tolerance").value)
        )
        self._laser_distance_topic = str(self.get_parameter("laser_distance_topic").value)
        self._laser_distance_threshold = int(self.get_parameter("laser_distance_threshold").value)
        self._motion_wait_timeout = max(0.0, float(self.get_parameter("motion_wait_timeout_sec").value))
        self._payload_wait_timeout = max(0.0, float(self.get_parameter("payload_wait_timeout_sec").value))
        self._laser_wait_timeout = max(0.0, float(self.get_parameter("laser_wait_timeout_sec").value))
        self._tracking_publish_rate_hz = max(
            0.1, float(self.get_parameter("tracking_publish_rate_hz").value)
        )
        self._dm_serial_bridge_enabled = bool(
            self.get_parameter("dm_serial_bridge_enabled").value
        )
        self._dm_serial_rx_topic = str(self.get_parameter("dm_serial_rx_topic").value)
        self._dm_serial_tx_topic = str(self.get_parameter("dm_serial_tx_topic").value)
        self._dm_serial_command_base_id = int(
            self.get_parameter("dm_serial_command_base_id").value
        )
        self._dm_serial_complete_id = int(self.get_parameter("dm_serial_complete_id").value)
        self._dm_serial_allowed_action_set_ids = self.get_parameter(
            "dm_serial_allowed_action_set_ids"
        ).value

        self._state = MiddlewareState.IDLE
        self._cached_target_point: Optional[TargetPoint] = None
        self._payload_active = False
        self._latest_j5_position: Optional[float] = None
        self._latest_laser_distance: Optional[int] = None
        self._latest_laser_distance_received_ns: Optional[int] = None
        self._active_run: Optional[ActiveRun] = None
        self._latest_motion_execution_id = 0
        self._action_sets_mtime_ns: Optional[int] = None
        self._dm_serial_bridge: Optional[DmSerialActionBridge] = None
        self._dm_serial_tx_pub = None

        self._action_sets = self._load_action_sets(self._action_sets_file)
        self._refresh_dm_serial_bridge()

        self._motion_target_pub = self.create_publisher(Arm2TargetPoint, self._motion_target_topic, 10)
        self._vacuum_pub = self.create_publisher(Bool, self._vacuum_topic, 10)
        self._payload_command_pub = self.create_publisher(Bool, self._payload_command_topic, 10)
        self._j5_command_pub = self.create_publisher(Float64, self._j5_command_topic, 10)
        if self._dm_serial_bridge_enabled:
            self._dm_serial_tx_pub = self.create_publisher(CanFrame, self._dm_serial_tx_topic, 10)
            self.create_subscription(CanFrame, self._dm_serial_rx_topic, self._on_dm_serial_frame, 50)
        self.create_subscription(Arm2TargetPoint, self._target_point_topic, self._on_target_point, 20)
        self.create_subscription(Int32, self._run_action_set_topic, self._on_run_action_set, 10)
        self.create_subscription(Bool, self._payload_active_topic, self._on_payload_active, 10)
        self.create_subscription(Float64, self._j5_position_topic, self._on_j5_position, 20)
        self.create_subscription(UInt32, self._laser_distance_topic, self._on_laser_distance, 20)
        self.create_subscription(
            Arm2MotionExecution,
            self._motion_execution_topic,
            self._on_motion_execution,
            20,
        )
        self.create_timer(1.0 / self._tracking_publish_rate_hz, self._on_timer)

        self.get_logger().info(
            "arm2_middleware ready: action_sets_file=%s action_sets=%s target_point=%s run_action_set=%s motion_target=%s "
            "motion_execution=%s vacuum=%s payload_command=%s payload_active=%s j5_command=%s j5_position=%s "
            "j5_tolerance=%.4f laser_distance=%s laser_threshold=%d laser_wait_timeout=%.2f tracking_publish_rate=%.2f "
            "dm_serial_bridge=%d dm_serial_rx=%s dm_serial_tx=%s dm_serial_command_base_id=0x%X "
            "dm_serial_complete_id=0x%X dm_serial_allowed_action_set_ids=%s"
            % (
                self._action_sets_file,
                sorted(self._action_sets.keys()),
                self._target_point_topic,
                self._run_action_set_topic,
                self._motion_target_topic,
                self._motion_execution_topic,
                self._vacuum_topic,
                self._payload_command_topic,
                self._payload_active_topic,
                self._j5_command_topic,
                self._j5_position_topic,
                self._j5_position_tolerance,
                self._laser_distance_topic,
                self._laser_distance_threshold,
                self._laser_wait_timeout,
                self._tracking_publish_rate_hz,
                1 if self._dm_serial_bridge_enabled else 0,
                self._dm_serial_rx_topic,
                self._dm_serial_tx_topic,
                self._dm_serial_command_base_id,
                self._dm_serial_complete_id,
                str(self._dm_serial_allowed_action_set_ids),
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
        self._action_sets_mtime_ns = config_path.stat().st_mtime_ns
        return action_sets

    def destroy_node(self) -> bool:
        return super().destroy_node()

    def _reload_action_sets_if_changed(self) -> None:
        try:
            current_mtime_ns = self._action_sets_file.stat().st_mtime_ns
        except FileNotFoundError:
            self.get_logger().warn(
                f"action set config disappeared: {self._action_sets_file}"
            )
            return

        if self._action_sets_mtime_ns == current_mtime_ns:
            return

        try:
            reloaded = self._load_action_sets(self._action_sets_file)
        except Exception as exc:
            self.get_logger().warn(
                f"failed to reload action sets from {self._action_sets_file}: {exc}"
            )
            return

        self._action_sets = reloaded
        self._refresh_dm_serial_bridge()
        self.get_logger().info(
            "reloaded action sets from %s ids=%s"
            % (self._action_sets_file, sorted(self._action_sets.keys()))
        )

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

        if step_type in {"set_vacuum", "set_payload_active"}:
            return ActionStep(
                step_type=step_type,
                label=label,
                enabled=bool(raw_step["enabled"]),
            )

        if step_type in {"move_target_offset", "move_target_offset_noj5"}:
            return ActionStep(
                step_type=step_type,
                label=label,
                target_spin_deg=target_spin_deg,
                offset_xyz=self._parse_xyz(raw_step.get("offset_xyz"), "offset_xyz"),
            )

        if step_type in {"move_target_offset_mf", "move_target_offset_mrl"}:
            return ActionStep(
                step_type=step_type,
                label=label,
                target_spin_deg=target_spin_deg,
                offset_xyz=self._parse_xyz(raw_step.get("offset_xyz"), "offset_xyz"),
                j5_target_pos=float(raw_step["j5_target_pos"]),
            )

        if step_type == "move_fixed_pose":
            return ActionStep(
                step_type=step_type,
                label=label,
                target_spin_deg=target_spin_deg,
                xyz=self._parse_xyz(raw_step.get("xyz"), "xyz"),
            )

        if step_type in {"move_fixed_pose_mf", "move_fixed_pose_mrl"}:
            return ActionStep(
                step_type=step_type,
                label=label,
                target_spin_deg=target_spin_deg,
                xyz=self._parse_xyz(raw_step.get("xyz"), "xyz"),
                j5_target_pos=float(raw_step["j5_target_pos"]),
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
        self._try_start_action_set(requested_id, source="topic")

    def _try_start_action_set(
        self,
        requested_id: int,
        source: str,
        can_id: Optional[int] = None,
    ) -> None:
        self._reload_action_sets_if_changed()
        if self._state != MiddlewareState.IDLE:
            self.get_logger().warn(
                "ignoring action set %d from %s while busy in state=%s"
                % (requested_id, source, self._state.value)
            )
            return

        action_set = self._action_sets.get(requested_id)
        if action_set is None:
            self.get_logger().warn(f"unknown action set id={requested_id} source={source}")
            return

        self._active_run = ActiveRun(
            action_set=action_set,
            trigger_source=source,
            trigger_can_id=can_id,
        )
        self._set_state(
            MiddlewareState.STARTING_SET,
            f"starting action_set id={action_set.action_id} name={action_set.name} source={source}",
        )
        self._start_next_step()

    def _on_laser_distance(self, msg: UInt32) -> None:
        self._latest_laser_distance = int(msg.data)
        self._latest_laser_distance_received_ns = self.get_clock().now().nanoseconds

    def _on_payload_active(self, msg: Bool) -> None:
        new_value = bool(msg.data)
        if self._payload_active != new_value:
            self._payload_active = new_value
            self.get_logger().info(f"payload_active={self._payload_active}")

        run = self._active_run
        if (
            run is not None
            and self._state == MiddlewareState.WAITING_PAYLOAD_ACTIVE
            and run.desired_payload_active is not None
            and self._payload_active == run.desired_payload_active
        ):
            self._complete_current_step(
                f"payload_active matched {self._payload_active}",
            )

    def _on_j5_position(self, msg: Float64) -> None:
        self._latest_j5_position = float(msg.data)
        run = self._active_run
        if run is None or self._state != MiddlewareState.WAITING_MOTION_RESULT:
            return
        if not self._is_j5_wait_step(run.current_step):
            return
        if run.waiting_motion_succeeded and self._is_j5_at_target(run.waiting_j5_target_pos):
            self._complete_current_step(self._j5_joint_wait_success_detail(run))

    def _on_motion_execution(self, msg: Arm2MotionExecution) -> None:
        execution_id = int(msg.execution_id)
        if execution_id > self._latest_motion_execution_id:
            self._latest_motion_execution_id = execution_id

        run = self._active_run
        if run is None:
            return

        if self._state == MiddlewareState.TRACKING_TARGET_OFFSET:
            self.get_logger().info(
                "ignoring motion execution during target_offset tracking execution_id=%d status=%s detail=%s"
                % (execution_id, _motion_status_name(msg.status), msg.detail)
            )
            return

        if self._state != MiddlewareState.WAITING_MOTION_RESULT:
            return

        if run.waiting_execution_id is None:
            if execution_id <= run.waiting_execution_baseline_id:
                return
            run.waiting_execution_id = execution_id
            self.get_logger().info(
                "tracking motion execution_id=%d status=%s"
                % (execution_id, _motion_status_name(msg.status))
            )

        if execution_id != run.waiting_execution_id:
            return

        if msg.status == Arm2MotionExecution.STATUS_ACCEPTED:
            return

        if msg.status == Arm2MotionExecution.STATUS_SUCCEEDED:
            if self._is_j5_wait_step(run.current_step):
                run.waiting_motion_succeeded = True
                if self._is_j5_at_target(run.waiting_j5_target_pos):
                    self._complete_current_step(self._j5_joint_wait_success_detail(run, execution_id, msg.detail))
                else:
                    self.get_logger().info(
                        "motion execution_id=%d succeeded, waiting for J5 target=%.4f actual=%s tolerance=%.4f"
                        % (
                            execution_id,
                            float(run.waiting_j5_target_pos),
                            (
                                f"{self._latest_j5_position:.4f}"
                                if self._latest_j5_position is not None
                                else "none"
                            ),
                            self._j5_position_tolerance,
                        )
                    )
            else:
                self._complete_current_step(
                    f"motion execution_id={execution_id} succeeded detail={msg.detail}",
                )
            return

        if msg.status == Arm2MotionExecution.STATUS_FAILED:
            self._fail_action_set(
                f"motion execution_id={execution_id} failed error_code={msg.error_code} detail={msg.detail}",
            )

    def _on_timer(self) -> None:
        run = self._active_run
        if run is None or run.waiting_started_ns is None:
            return

        if self._state == MiddlewareState.WAITING_MOTION_RESULT:
            timeout_sec = self._motion_wait_timeout
            if self._is_j5_wait_step(run.current_step):
                timeout_detail = self._j5_joint_wait_timeout_detail(timeout_sec)
            else:
                timeout_detail = f"motion wait timeout after {timeout_sec:.2f} sec"
        elif self._state == MiddlewareState.TRACKING_TARGET_OFFSET:
            self._refresh_target_offset_target(run)
            laser_distance = self._latest_laser_distance
            if (
                laser_distance is not None
                and laser_distance < self._laser_distance_threshold
            ):
                self._complete_current_step(
                    "target_offset completed with laser_distance=%d threshold=%d"
                    % (laser_distance, self._laser_distance_threshold)
                )
                return
            timeout_sec = self._laser_wait_timeout
            timeout_detail = self._target_offset_timeout_detail(timeout_sec)
        elif self._state == MiddlewareState.WAITING_PAYLOAD_ACTIVE:
            timeout_sec = self._payload_wait_timeout
            desired = run.desired_payload_active
            timeout_detail = (
                f"payload wait timeout after {timeout_sec:.2f} sec waiting for payload_active={desired}"
            )
        else:
            return

        if timeout_sec <= 0.0:
            return

        elapsed = self.get_clock().now().nanoseconds - run.waiting_started_ns
        if elapsed < int(timeout_sec * 1_000_000_000):
            return

        self._fail_action_set(timeout_detail)

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
            if run.trigger_source == "dm_serial":
                self._send_dm_serial_completion(run.action_set.action_id)
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
            self._complete_current_step(f"vacuum set to {bool(step.enabled)}")
            return

        if step.step_type == "set_payload_active":
            desired = bool(step.enabled)
            self._publish_payload_active(desired)
            if self._payload_active == desired:
                self._complete_current_step(f"payload_active already {desired}")
                return
            self._enter_payload_wait(
                detail=f"waiting for payload_active={desired}",
                desired=desired,
            )
            return

        if step.step_type == "move_target_offset":
            if self._cached_target_point is None:
                self._fail_action_set("move_target_offset requested before target point was received")
                return
            target_spin_deg = self._cached_target_point.target_spin_deg
            self._enter_target_offset_tracking(
                "tracking target_offset offset=(%.4f, %.4f, %.4f) spin=%.2f threshold=%d timeout=%.2f"
                % (
                    float(step.offset_xyz[0]),
                    float(step.offset_xyz[1]),
                    float(step.offset_xyz[2]),
                    target_spin_deg,
                    self._laser_distance_threshold,
                    self._laser_wait_timeout,
                )
            )
            self._refresh_target_offset_target(run)
            return

        if step.step_type == "move_target_offset_noj5":
            if self._cached_target_point is None:
                self._fail_action_set(
                    "move_target_offset_noj5 requested before target point was received"
                )
                return
            x = self._cached_target_point.x + float(step.offset_xyz[0])
            y = self._cached_target_point.y + float(step.offset_xyz[1])
            z = self._cached_target_point.z + float(step.offset_xyz[2])
            if(self._cached_target_point.z <0.1):
                target_spin_deg = 0.0
            else:
                target_spin_deg = 90.0
            self._enter_motion_wait(
                "waiting on move_target_offset_noj5 x=%.4f y=%.4f z=%.4f spin=%.2f"
                % (x, y, z, target_spin_deg),
            )
            self._publish_motion_target(x, y, z, target_spin_deg)
            return

        if step.step_type in {"move_target_offset_mf", "move_target_offset_mrl"}:
            if self._cached_target_point is None:
                self._fail_action_set(f"{step.step_type} requested before target point was received")
                return
            x = self._cached_target_point.x + float(step.offset_xyz[0])
            y = self._cached_target_point.y + float(step.offset_xyz[1])
            z = self._cached_target_point.z + float(step.offset_xyz[2])
            j5_target_pos = step.j5_target_pos
            if step.step_type.endswith("_mf"):
                x = 0.0
            else:
                # _mrl
                y = 0.0
                j5_target_pos = float(self._cached_target_point.y) + float(step.j5_target_pos)
            if self._cached_target_point.z <0.1:
                target_spin_deg = 0.0
            else:
                target_spin_deg = 90.0
            self._enter_motion_wait(
                "waiting on %s x=%.4f y=%.4f z=%.4f spin=%.2f j5=%.4f"
                % (step.step_type, x, y, z, target_spin_deg, float(j5_target_pos)),
                j5_target_pos=j5_target_pos,
            )
            self._publish_motion_target(x, y, z, target_spin_deg)
            self._publish_j5_target(j5_target_pos)
            return

        if step.step_type == "move_fixed_pose":
            x, y, z = step.xyz
            self._enter_motion_wait(
                f"waiting on fixed_pose x={x:.4f} y={y:.4f} z={z:.4f} spin={step.target_spin_deg:.2f}",
            )
            self._publish_motion_target(x, y, z, step.target_spin_deg)
            return

        if step.step_type in {"move_fixed_pose_mf", "move_fixed_pose_mrl"}:
            x, y, z = step.xyz
            j5_target_pos = step.j5_target_pos
            if step.step_type.endswith("_mf"):
                x = 0.0
            else:
                y = 0.0
                j5_target_pos = float(step.xyz[1]) + float(step.j5_target_pos)
            self._enter_motion_wait(
                "waiting on %s x=%.4f y=%.4f z=%.4f spin=%.2f j5=%.4f"
                % (step.step_type, x, y, z, step.target_spin_deg, float(j5_target_pos)),
                j5_target_pos=j5_target_pos,
            )
            self._publish_motion_target(x, y, z, step.target_spin_deg)
            self._publish_j5_target(j5_target_pos)
            return

        self._fail_action_set(f"unsupported step type at runtime: {step.step_type}")

    def _enter_motion_wait(self, detail: str, j5_target_pos: Optional[float] = None) -> None:
        run = self._active_run
        if run is None:
            self._fail_action_set("internal error: missing active run when entering motion wait")
            return

        run.waiting_started_ns = self.get_clock().now().nanoseconds
        run.waiting_execution_baseline_id = self._latest_motion_execution_id
        run.waiting_execution_id = None
        run.desired_payload_active = None
        run.last_target_offset_command = None
        run.target_offset_publish_count = 0
        run.waiting_motion_succeeded = False
        run.waiting_j5_target_pos = j5_target_pos
        run.last_j5_command_pos = j5_target_pos
        self._set_state(MiddlewareState.WAITING_MOTION_RESULT, detail)

    def _enter_target_offset_tracking(self, detail: str) -> None:
        run = self._active_run
        if run is None:
            self._fail_action_set("internal error: missing active run when entering target_offset tracking")
            return

        run.waiting_started_ns = self.get_clock().now().nanoseconds
        run.waiting_execution_baseline_id = self._latest_motion_execution_id
        run.waiting_execution_id = None
        run.desired_payload_active = None
        run.last_target_offset_command = None
        run.target_offset_publish_count = 0
        run.waiting_motion_succeeded = False
        run.waiting_j5_target_pos = None
        run.last_j5_command_pos = None
        self._set_state(MiddlewareState.TRACKING_TARGET_OFFSET, detail)

    def _enter_payload_wait(self, detail: str, desired: bool) -> None:
        run = self._active_run
        if run is None:
            self._fail_action_set("internal error: missing active run when entering payload wait")
            return

        run.waiting_started_ns = self.get_clock().now().nanoseconds
        run.waiting_execution_baseline_id = self._latest_motion_execution_id
        run.waiting_execution_id = None
        run.desired_payload_active = bool(desired)
        run.last_target_offset_command = None
        run.target_offset_publish_count = 0
        run.waiting_motion_succeeded = False
        run.waiting_j5_target_pos = None
        run.last_j5_command_pos = None
        self._set_state(MiddlewareState.WAITING_PAYLOAD_ACTIVE, detail)

    def _refresh_target_offset_target(self, run: ActiveRun) -> None:
        step = run.current_step
        target_point = self._cached_target_point
        if step is None or step.step_type != "move_target_offset" or target_point is None:
            return

        x = target_point.x + float(step.offset_xyz[0])
        y = target_point.y + float(step.offset_xyz[1])
        z = target_point.z + float(step.offset_xyz[2])
        target_spin_deg = target_point.target_spin_deg
        self._publish_motion_target(x, y, z, target_spin_deg)
        run.last_target_offset_command = TargetPoint(
            x=float(x),
            y=float(y),
            z=float(z),
            target_spin_deg=float(target_spin_deg),
        )
        run.target_offset_publish_count += 1
        self.get_logger().info(
            "target_offset publish #%d x=%.4f y=%.4f z=%.4f spin=%.2f laser_distance=%s"
            % (
                run.target_offset_publish_count,
                x,
                y,
                z,
                target_spin_deg,
                self._latest_laser_distance if self._latest_laser_distance is not None else "none",
            )
        )

    def _target_offset_timeout_detail(self, timeout_sec: float) -> str:
        run = self._active_run
        last_target = run.last_target_offset_command if run is not None else None
        if last_target is None:
            target_detail = "last_target=none"
        else:
            target_detail = (
                "last_target=(x=%.4f, y=%.4f, z=%.4f, spin=%.2f)"
                % (
                    last_target.x,
                    last_target.y,
                    last_target.z,
                    last_target.target_spin_deg,
                )
            )

        laser_detail = (
            str(self._latest_laser_distance)
            if self._latest_laser_distance is not None
            else "none"
        )
        return (
            "target_offset wait timeout after %.2f sec laser_distance=%s threshold=%d %s"
            % (
                timeout_sec,
                laser_detail,
                self._laser_distance_threshold,
                target_detail,
            )
        )

    def _publish_motion_target(self, x: float, y: float, z: float, target_spin_deg: float) -> None:
        msg = Arm2TargetPoint()
        msg.xyz.x = float(x)
        msg.xyz.y = float(y)
        msg.xyz.z = float(z)
        msg.target_spin_deg = float(target_spin_deg)
        self._motion_target_pub.publish(msg)
        self.get_logger().info(
            "published motion target x=%.4f y=%.4f z=%.4f spin=%.2f deg on %s"
            % (x, y, z, target_spin_deg, self._motion_target_topic)
        )

    def _publish_vacuum(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self._vacuum_pub.publish(msg)
        self.get_logger().info(f"published vacuum={enabled} on {self._vacuum_topic}")

    def _publish_payload_active(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self._payload_command_pub.publish(msg)
        self.get_logger().info(
            f"published payload_active={enabled} on {self._payload_command_topic}"
        )

    def _publish_j5_target(self, target_pos: Optional[float]) -> None:
        if target_pos is None:
            return
        msg = Float64()
        msg.data = float(target_pos)
        self._j5_command_pub.publish(msg)
        self.get_logger().info(
            "published j5 target=%.4f on %s"
            % (float(target_pos), self._j5_command_topic)
        )

    def _is_j5_wait_step(self, step: Optional[ActionStep]) -> bool:
        return step is not None and step.step_type in {
            "move_target_offset_mf",
            "move_target_offset_mrl",
            "move_fixed_pose_mf",
            "move_fixed_pose_mrl",
        }

    def _is_j5_at_target(self, target_pos: Optional[float]) -> bool:
        if target_pos is None or self._latest_j5_position is None:
            return False
        return abs(self._latest_j5_position - float(target_pos)) <= self._j5_position_tolerance

    def _j5_joint_wait_success_detail(
        self,
        run: ActiveRun,
        execution_id: Optional[int] = None,
        motion_detail: str = "",
    ) -> str:
        detail = (
            "motion execution_id=%s succeeded detail=%s j5 target=%.4f actual=%.4f tolerance=%.4f"
            % (
                execution_id if execution_id is not None else run.waiting_execution_id,
                motion_detail,
                float(run.waiting_j5_target_pos),
                float(self._latest_j5_position),
                self._j5_position_tolerance,
            )
        )
        return detail

    def _j5_joint_wait_timeout_detail(self, timeout_sec: float) -> str:
        run = self._active_run
        actual = "none" if self._latest_j5_position is None else f"{self._latest_j5_position:.4f}"
        target = "none"
        motion_succeeded = False
        if run is not None:
            if run.waiting_j5_target_pos is not None:
                target = f"{float(run.waiting_j5_target_pos):.4f}"
            motion_succeeded = bool(run.waiting_motion_succeeded)
        return (
            "motion+j5 wait timeout after %.2f sec motion_succeeded=%s j5_target=%s j5_actual=%s tolerance=%.4f"
            % (
                timeout_sec,
                motion_succeeded,
                target,
                actual,
                self._j5_position_tolerance,
            )
        )

    def _complete_current_step(self, detail: str) -> None:
        run = self._active_run
        if run is None:
            return
        run.waiting_started_ns = None
        run.waiting_execution_baseline_id = self._latest_motion_execution_id
        run.waiting_execution_id = None
        run.desired_payload_active = None
        run.last_target_offset_command = None
        run.target_offset_publish_count = 0
        run.waiting_motion_succeeded = False
        run.waiting_j5_target_pos = None
        run.last_j5_command_pos = None
        self._record_step_result(True, detail)
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

        run.waiting_started_ns = None
        run.waiting_execution_id = None
        run.desired_payload_active = None
        run.last_target_offset_command = None
        run.target_offset_publish_count = 0
        run.waiting_motion_succeeded = False
        run.waiting_j5_target_pos = None
        run.last_j5_command_pos = None
        if run.current_step is not None:
            self._record_step_result(False, detail)
        self._set_state(
            MiddlewareState.FAILED,
            f"action_set id={run.action_set.action_id} name={run.action_set.name} detail={detail}",
        )
        self._active_run = None
        self._set_state(MiddlewareState.IDLE, "ready for next action set")

    def _refresh_dm_serial_bridge(self) -> None:
        allowed_action_set_ids = resolve_allowed_action_set_ids(
            self._action_sets.keys(),
            self._dm_serial_allowed_action_set_ids,
        )
        self._dm_serial_bridge = DmSerialActionBridge(
            action_set_ids=allowed_action_set_ids,
            command_base_id=self._dm_serial_command_base_id,
            complete_id=self._dm_serial_complete_id,
        )

    def _on_dm_serial_frame(self, msg: CanFrame) -> None:
        if not self._dm_serial_bridge_enabled or self._dm_serial_bridge is None:
            return

        dlc = max(0, min(int(msg.dlc), 64))
        frame = RawCanFrame(
            can_id=int(msg.id),
            dlc=dlc,
            data=list(msg.data[:dlc]),
            is_extended=bool(msg.is_extended),
            is_remote=bool(msg.is_remote),
            is_fd=bool(msg.is_fd),
        )
        action_id = self._dm_serial_bridge.action_set_id_from_frame(frame)
        if action_id is None:
            return

        self.get_logger().info(
            "received DM serial command id=0x%X dlc=%d -> action_set=%d"
            % (frame.can_id, frame.dlc, action_id)
        )
        self._try_start_action_set(action_id, source="dm_serial", can_id=frame.can_id)

    def _send_dm_serial_completion(self, action_set_id: int) -> None:
        if self._dm_serial_bridge is None or self._dm_serial_tx_pub is None:
            return

        frame = self._dm_serial_bridge.completion_frame()
        msg = CanFrame()
        msg.id = frame.can_id
        msg.is_extended = frame.is_extended
        msg.is_remote = frame.is_remote
        msg.is_fd = frame.is_fd
        msg.dlc = frame.dlc
        msg.data = [0] * 64
        for index, value in enumerate(frame.data[: frame.dlc]):
            msg.data[index] = value
        self._dm_serial_tx_pub.publish(msg)

        self.get_logger().info(
            "sent DM serial completion id=0x%X for action_set=%d on %s"
            % (frame.can_id, action_set_id, self._dm_serial_tx_topic)
        )


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
