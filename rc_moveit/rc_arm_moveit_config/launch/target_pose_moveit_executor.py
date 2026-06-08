#!/usr/bin/env python3
"""Subscribe manual and middleware targets and drive MoveIt planning/execution."""

import argparse
from dataclasses import dataclass
import json
import math
import threading
from typing import Dict, List, Optional, Tuple

from arm_msgs.msg import Arm2MotionExecution, Arm2TargetPoint
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint, PlanningScene
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
import tf2_ros

from rc_arm_world_pitch_kinematics import RcArmWorldPitchKinematics


EXECUTION_ERROR_BUSY = -1
EXECUTION_ERROR_NOT_READY = -2
EXECUTION_ERROR_IK_FAILED = -3
EXECUTION_ERROR_GOAL_SEND_EXCEPTION = -4
EXECUTION_ERROR_GOAL_REJECTED = -5
EXECUTION_ERROR_RESULT_EXCEPTION = -6
EXECUTION_ERROR_PREEMPTED = -7
EXECUTION_ERROR_PENDING_SUPERSEDED = -8
EXECUTION_ERROR_REDUNDANT_TARGET = -9


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


def _parse_bool(text: str) -> bool:
    v = (text or "").strip().lower()
    return v in {"1", "true", "yes", "y", "on"}


def _as_xyz(values) -> Optional[Tuple[float, float, float]]:
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        return None
    try:
        return float(values[0]), float(values[1]), float(values[2])
    except (TypeError, ValueError):
        return None


def _as_xyzw(values) -> Optional[Tuple[float, float, float, float]]:
    if not isinstance(values, (list, tuple)) or len(values) != 4:
        return None
    try:
        return float(values[0]), float(values[1]), float(values[2]), float(values[3])
    except (TypeError, ValueError):
        return None


def _json_preview(text: str, limit: int = 160) -> str:
    one_line = (text or "").replace("\n", " ").strip()
    if len(one_line) <= limit:
        return one_line
    return one_line[:limit] + "..."


@dataclass
class ExecutionRequest:
    source: str
    target: PoseStamped
    execution_id: Optional[int] = None


class TargetPoseMoveItExecutor(Node):
    def __init__(
        self,
        target_topic: str,
        middleware_target_topic: str,
        middleware_result_topic: str,
        planning_group: str,
        joint_names: List[str],
        default_frame: str,
        move_action_name: str,
        pos_threshold: float,
        rot_threshold: float,
        planning_time: float,
        planning_attempts: int,
        vel_scale: float,
        acc_scale: float,
        joint_tolerance: float,
        check_period: float,
        avoid_collisions: bool,
        j4_axis: str,
        joint_state_topic: str,
        urdf_path: str,
        status_log_period: float,
        status_base_frame: str,
        status_eef_frame: str,
        world_boxes_json: str,
        planning_scene_topic: str,
        scene_publish_retries: int,
        middleware_preempt_interval_sec: float,
        middleware_preempt_pos_threshold: float,
        middleware_preempt_rot_threshold: float,
    ) -> None:
        super().__init__("rc_arm_target_pose_moveit_executor")

        self._manual_target_topic = target_topic
        self._middleware_target_topic = middleware_target_topic
        self._middleware_result_topic = middleware_result_topic
        self._planning_group = planning_group
        self._joint_names = list(joint_names)
        self._default_frame = _normalize_frame_id(default_frame)
        self._pos_threshold = max(0.0, float(pos_threshold))
        self._rot_threshold = max(0.0, float(rot_threshold))
        self._planning_time = max(0.1, float(planning_time))
        self._planning_attempts = max(1, int(planning_attempts))
        self._vel_scale = max(0.01, min(1.0, float(vel_scale)))
        self._acc_scale = max(0.01, min(1.0, float(acc_scale)))
        self._joint_tolerance = max(1.0e-4, float(joint_tolerance))
        self._avoid_collisions = bool(avoid_collisions)
        self._j4_axis = str(j4_axis).strip().lower() if str(j4_axis).strip() else "y"
        if self._j4_axis not in {"x", "y", "z"}:
            self._j4_axis = "y"
        self._status_log_period = max(0.0, float(status_log_period))

        self._status_base_frame = _normalize_frame_id(status_base_frame)
        self._status_eef_frame = _normalize_frame_id(status_eef_frame)
        self._planning_scene_topic = planning_scene_topic
        self._scene_publish_retries_left = max(1, int(scene_publish_retries))
        self._world_box_configs = self._parse_world_boxes_json(world_boxes_json)
        self._middleware_preempt_interval_sec = max(0.0, float(middleware_preempt_interval_sec))
        self._middleware_preempt_pos_threshold = max(
            0.0, float(middleware_preempt_pos_threshold)
        )
        self._middleware_preempt_rot_threshold = max(
            0.0, float(middleware_preempt_rot_threshold)
        )

        self._manual_target_lock = threading.Lock()
        self._latest_manual_target: Optional[PoseStamped] = None
        self._last_sent_manual_target: Optional[PoseStamped] = None

        self._busy = False
        self._active_request: Optional[ExecutionRequest] = None
        self._active_goal_handle = None
        self._pending_request: Optional[ExecutionRequest] = None
        self._canceling_request: Optional[ExecutionRequest] = None
        self._last_preempt_request_sec = -1.0e9
        self._ready_move_action = False
        self._ready_solver = False
        self._last_ready_tuple = None
        self._latest_joint_map: Dict[str, float] = {}
        self._latest_joint_velocity_map: Dict[str, float] = {}
        self._next_execution_id = 0

        self._last_event = "init"
        self._last_event_time_sec = self._now_sec()

        self._kinematics = RcArmWorldPitchKinematics(
            urdf_path=urdf_path or None,
            joint_names=self._joint_names,
            j4_axis=self._j4_axis,
        )
        self._move_group_client = ActionClient(self, MoveGroup, move_action_name)
        scene_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._planning_scene_pub = self.create_publisher(PlanningScene, planning_scene_topic, scene_qos)
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
        self._scene_timer = self.create_timer(1.0, self._publish_static_world_scene)
        if self._status_log_period > 0.0:
            self._status_timer = self.create_timer(self._status_log_period, self._log_status)

        self.get_logger().info(
            "TargetPose->MoveIt executor started: manual_topic=%s middleware_target=%s middleware_result=%s "
            "group=%s avoid_collisions=%d world_boxes=%d planning_scene_topic=%s status_tf=%s->%s"
            % (
                self._manual_target_topic,
                self._middleware_target_topic,
                self._middleware_result_topic,
                self._planning_group,
                1 if self._avoid_collisions else 0,
                len(self._world_box_configs),
                self._planning_scene_topic,
                self._status_base_frame,
                self._status_eef_frame,
            )
        )

    def _on_joint_state(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        mapping = self._latest_joint_map.copy()
        velocity_mapping = self._latest_joint_velocity_map.copy()
        for idx, name in enumerate(msg.name):
            if idx < len(msg.position):
                mapping[name] = float(msg.position[idx])
            if idx < len(msg.velocity):
                velocity_mapping[name] = float(msg.velocity[idx])
        self._latest_joint_map = mapping
        self._latest_joint_velocity_map = velocity_mapping

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

    def _parse_world_boxes_json(self, text: str) -> List[dict]:
        try:
            loaded = json.loads(text or "[]")
        except json.JSONDecodeError as exc:
            self.get_logger().warn(
                "world_boxes_json parse failed at char %d: %s preview='%s'"
                % (exc.pos, exc.msg, _json_preview(text))
            )
            return []

        if not isinstance(loaded, list):
            self.get_logger().warn("world_boxes_json must be a JSON list, got %s" % type(loaded).__name__)
            return []

        boxes = []
        for idx, item in enumerate(loaded, start=1):
            if isinstance(item, dict):
                boxes.append(item)
            else:
                self.get_logger().warn(
                    "world box %d skipped: expected object, got %s" % (idx, type(item).__name__)
                )
        return boxes

    def _build_box_collision_object(
        self,
        box_cfg: dict,
        default_id: str,
        default_frame: str,
        operation: int = CollisionObject.ADD,
    ) -> Optional[CollisionObject]:
        sx_sy_sz = _as_xyz(box_cfg.get("size"))
        if sx_sy_sz is None:
            self.get_logger().warn("box '%s' skipped: size must be [x, y, z]" % default_id)
            return None

        sx, sy, sz = (abs(v) for v in sx_sy_sz)
        if sx <= 1.0e-6 or sy <= 1.0e-6 or sz <= 1.0e-6:
            self.get_logger().warn("box '%s' skipped: size must be non-zero" % default_id)
            return None

        position = _as_xyz(box_cfg.get("position"))
        if position is None:
            if "position" in box_cfg:
                self.get_logger().warn("box '%s': invalid position, using [0, 0, 0]" % default_id)
            position = (0.0, 0.0, 0.0)
        px, py, pz = position

        q = _as_xyzw(box_cfg.get("orientation"))
        if q is None:
            if "orientation" in box_cfg:
                self.get_logger().warn("box '%s': invalid orientation, using identity" % default_id)
            q = (0.0, 0.0, 0.0, 1.0)
        qx, qy, qz, qw = _normalize_quat_xyzw(q)

        obj = CollisionObject()
        obj.id = str(box_cfg.get("id", default_id)).strip() or default_id
        obj.header.frame_id = _normalize_frame_id(str(box_cfg.get("frame_id", ""))) or default_frame
        obj.operation = operation

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [sx, sy, sz]

        pose = Pose()
        pose.position.x = px
        pose.position.y = py
        pose.position.z = pz
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        return obj

    def _publish_scene_diff(
        self,
        world_objects: Optional[List[CollisionObject]] = None,
    ) -> None:
        scene = PlanningScene()
        scene.is_diff = True
        if world_objects:
            scene.world.collision_objects = world_objects
        self._planning_scene_pub.publish(scene)

    def _publish_static_world_scene(self) -> None:
        objects: List[CollisionObject] = []
        for idx, box_cfg in enumerate(self._world_box_configs, start=1):
            obj = self._build_box_collision_object(
                box_cfg,
                default_id="world_box_%d" % idx,
                default_frame=self._default_frame,
            )
            if obj is not None:
                objects.append(obj)

        if objects:
            self._publish_scene_diff(world_objects=objects)
            self.get_logger().info(
                "Published world collision boxes: count=%d retries_left=%d"
                % (len(objects), self._scene_publish_retries_left)
            )

        self._scene_publish_retries_left -= 1
        if self._scene_publish_retries_left <= 0:
            self._scene_timer.cancel()

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
        mg_ready = self._move_group_client.server_is_ready()
        solver_ready = self._kinematics is not None

        self._ready_move_action = mg_ready
        self._ready_solver = solver_ready

        ready_tuple = (mg_ready, solver_ready)
        if ready_tuple != self._last_ready_tuple:
            self.get_logger().info(
                "[STATE] ready move_action=%d solver=%d %s"
                % (1 if mg_ready else 0, 1 if solver_ready else 0, self._format_eef())
            )
            self._last_ready_tuple = ready_tuple

    def _executor_ready(self) -> bool:
        self._update_ready()
        return self._ready_move_action and self._ready_solver

    def _log_status(self) -> None:
        event_age = max(0.0, self._now_sec() - self._last_event_time_sec)
        with self._manual_target_lock:
            has_manual_target = self._latest_manual_target is not None
        active_source = self._active_request.source if self._active_request is not None else "idle"
        self.get_logger().info(
            "[STATE] busy=%d ready(move_action=%d,solver=%d) manual_target=%d active=%s event=%s(%.2fs) %s"
            % (
                1 if self._busy else 0,
                1 if self._ready_move_action else 0,
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

    def _fail_middleware_request(self, request: ExecutionRequest, error_code: int, detail: str) -> None:
        if request.execution_id is None:
            return
        self._publish_motion_execution(
            request.execution_id,
            Arm2MotionExecution.STATUS_FAILED,
            error_code,
            detail,
        )

    def _accept_middleware_request(self, request: ExecutionRequest) -> None:
        if request.execution_id is None:
            return
        self._publish_motion_execution(
            request.execution_id,
            Arm2MotionExecution.STATUS_ACCEPTED,
            0,
            "accepted",
        )

    def _on_manual_target(self, msg: PoseStamped) -> None:
        pose_msg = self._resolve_target_pose(msg)
        with self._manual_target_lock:
            self._latest_manual_target = pose_msg
        self._event("manual_target_rx", pose_msg)

    def _on_middleware_target(self, msg: Arm2TargetPoint) -> None:
        pose_msg = self._motion_target_to_pose(msg)
        request = ExecutionRequest(
            source="middleware",
            target=pose_msg,
            execution_id=self._allocate_execution_id(),
        )
        self._event("middleware_target_rx", pose_msg, extra="execution_id=%d" % request.execution_id)

        if not self._executor_ready():
            self._fail_middleware_request(
                request,
                EXECUTION_ERROR_NOT_READY,
                "executor not ready",
            )
            return

        if self._busy:
            self._queue_pending_middleware_request(request)
            return

        self._accept_middleware_request(request)
        self._start_execution(request)

    def _queue_pending_middleware_request(self, request: ExecutionRequest) -> None:
        reference = self._pending_request if self._pending_request is not None else self._active_request
        if reference is not None and not self._target_changed(
            reference.target,
            request.target,
            pos_threshold=self._middleware_preempt_pos_threshold,
            rot_threshold=self._middleware_preempt_rot_threshold,
        ):
            self._fail_middleware_request(
                request,
                EXECUTION_ERROR_REDUNDANT_TARGET,
                "target delta below preempt threshold",
            )
            return

        if self._pending_request is not None and self._pending_request.execution_id is not None:
            self._fail_middleware_request(
                self._pending_request,
                EXECUTION_ERROR_PENDING_SUPERSEDED,
                "superseded by newer middleware target",
            )

        self._pending_request = request
        self._accept_middleware_request(request)
        self._event(
            "middleware_target_pending",
            request.target,
            extra="execution_id=%d active_source=%s"
            % (
                request.execution_id,
                self._active_request.source if self._active_request is not None else "idle",
            ),
        )
        self._maybe_preempt_active_request()

    def _target_changed(
        self,
        prev: PoseStamped,
        cur: PoseStamped,
        pos_threshold: Optional[float] = None,
        rot_threshold: Optional[float] = None,
    ) -> bool:
        pos_threshold = self._pos_threshold if pos_threshold is None else max(0.0, float(pos_threshold))
        rot_threshold = self._rot_threshold if rot_threshold is None else max(0.0, float(rot_threshold))
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
        return pos_delta > pos_threshold or rot_delta > rot_threshold

    def _on_timer(self) -> None:
        if self._busy:
            self._maybe_preempt_active_request()
            return

        if not self._executor_ready():
            return

        if self._start_pending_request():
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

    def _current_seed_joints(self) -> Optional[Dict[str, float]]:
        if not self._latest_joint_map:
            return None
        missing = [name for name in self._joint_names if name not in self._latest_joint_map]
        if missing:
            return None
        return {name: self._latest_joint_map[name] for name in self._joint_names}

    def _current_joint_velocities(self) -> Optional[Dict[str, float]]:
        if not self._latest_joint_velocity_map:
            return None
        missing = [name for name in self._joint_names if name not in self._latest_joint_velocity_map]
        if missing:
            return None
        return {name: self._latest_joint_velocity_map[name] for name in self._joint_names}

    def _start_pending_request(self) -> bool:
        if self._busy or self._pending_request is None:
            return False
        request = self._pending_request
        self._pending_request = None
        self._start_execution(request)
        return True

    def _maybe_preempt_active_request(self) -> None:
        if self._active_request is None or self._pending_request is None:
            return
        if self._active_request.source != "middleware":
            return
        if self._active_goal_handle is None or self._canceling_request is not None:
            return
        if (
            self._now_sec() - self._last_preempt_request_sec
            < self._middleware_preempt_interval_sec
        ):
            return

        self._canceling_request = self._active_request
        self._last_preempt_request_sec = self._now_sec()
        self._event(
            "goal_cancel_request",
            self._pending_request.target,
            extra="active_execution_id=%s pending_execution_id=%s"
            % (
                self._active_request.execution_id,
                self._pending_request.execution_id,
            ),
        )
        cancel_future = self._active_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            lambda future, request=self._active_request: self._on_cancel_response(future, request)
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
        seed = self._current_seed_joints()
        q_target = self._kinematics.solve_xyz_pitch(
            float(pose.pose.position.x),
            float(pose.pose.position.y),
            float(pose.pose.position.z),
            pitch_rad,
            seed_joints=seed,
        )
        if q_target is None:
            self._event("solve_fail", pose, extra="source=%s" % request.source)
            self.get_logger().warn(
                "4DOF solve failed target_xyz=(%.3f, %.3f, %.3f) target_pitch=%.3f source=%s %s"
                % (
                    float(pose.pose.position.x),
                    float(pose.pose.position.y),
                    float(pose.pose.position.z),
                    pitch_rad,
                    request.source,
                    self._format_eef(),
                )
            )
            self._complete_execution(
                request,
                success=False,
                error_code=EXECUTION_ERROR_IK_FAILED,
                detail="ik solve failed",
                reason="solve_fail",
            )
            return

        self._event("solve_ok", pose, extra="source=%s pitch=%.3f" % (request.source, pitch_rad))
        self._send_goal(request, q_target)

    def _send_goal(self, request: ExecutionRequest, q_target: Dict[str, float]) -> None:
        goal = MoveGroup.Goal()
        goal.request.group_name = self._planning_group
        goal.request.num_planning_attempts = self._planning_attempts
        goal.request.allowed_planning_time = self._planning_time
        goal.request.max_velocity_scaling_factor = self._vel_scale
        goal.request.max_acceleration_scaling_factor = self._acc_scale

        start_joints = self._current_seed_joints()
        if start_joints is not None:
            goal.request.start_state.joint_state.header.stamp = self.get_clock().now().to_msg()
            goal.request.start_state.joint_state.name = list(self._joint_names)
            goal.request.start_state.joint_state.position = [
                start_joints[joint] for joint in self._joint_names
            ]
            start_velocities = self._current_joint_velocities()
            if start_velocities is not None:
                goal.request.start_state.joint_state.velocity = [
                    start_velocities[joint] for joint in self._joint_names
                ]

        constraints = Constraints()
        for joint in self._joint_names:
            jc = JointConstraint()
            jc.joint_name = joint
            jc.position = q_target[joint]
            jc.tolerance_above = self._joint_tolerance
            jc.tolerance_below = self._joint_tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 1

        self._event("goal_send", request.target, extra="source=%s" % request.source)
        send_future = self._move_group_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda future, request=request: self._on_goal_response(future, request)
        )

    def _on_goal_response(self, future, request: ExecutionRequest) -> None:
        if self._active_request is not request:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._event("goal_send_exception", request.target, extra="source=%s" % request.source)
            self.get_logger().warn(f"MoveGroup send exception: {exc}")
            self._complete_execution(
                request,
                success=False,
                error_code=EXECUTION_ERROR_GOAL_SEND_EXCEPTION,
                detail=f"MoveGroup send exception: {exc}",
                reason="goal_send_exception",
            )
            return

        if goal_handle is None or not goal_handle.accepted:
            self._event("goal_rejected", request.target, extra="source=%s" % request.source)
            self.get_logger().warn("MoveGroup goal rejected")
            self._complete_execution(
                request,
                success=False,
                error_code=EXECUTION_ERROR_GOAL_REJECTED,
                detail="MoveGroup goal rejected",
                reason="goal_rejected",
            )
            return

        self._active_goal_handle = goal_handle
        self._event("goal_accepted", request.target, extra="source=%s" % request.source)
        self._maybe_preempt_active_request()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, request=request: self._on_goal_result(future, request)
        )

    def _on_cancel_response(self, future, request: ExecutionRequest) -> None:
        if self._canceling_request is not request:
            return

        try:
            response = future.result()
        except Exception as exc:
            self._canceling_request = None
            self.get_logger().warn(f"MoveGroup cancel exception: {exc}")
            self._event("goal_cancel_exception", request.target, extra="source=%s" % request.source)
            return

        goals_canceling = getattr(response, "goals_canceling", [])
        if not goals_canceling:
            self._canceling_request = None
            self.get_logger().warn("MoveGroup cancel rejected or no goals_canceling returned")
            self._event("goal_cancel_rejected", request.target, extra="source=%s" % request.source)
            return

        self._event("goal_cancel_accepted", request.target, extra="source=%s" % request.source)

    def _on_goal_result(self, future, request: ExecutionRequest) -> None:
        if self._active_request is not request:
            return

        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as exc:
            self._event("exec_result_exception", request.target, extra="source=%s" % request.source)
            self.get_logger().warn(f"MoveGroup result exception: {exc}")
            self._complete_execution(
                request,
                success=False,
                error_code=EXECUTION_ERROR_RESULT_EXCEPTION,
                detail=f"MoveGroup result exception: {exc}",
                reason="exec_result_exception",
            )
            return

        if (
            self._canceling_request is request
            and self._pending_request is not None
            and result.error_code.val != result.error_code.SUCCESS
        ):
            self._event("goal_preempted", request.target, extra="source=%s" % request.source)
            self._complete_execution(
                request,
                success=False,
                error_code=EXECUTION_ERROR_PREEMPTED,
                detail="execution preempted by newer middleware target",
                reason="goal_preempted",
            )
            return

        if result.error_code.val == result.error_code.SUCCESS:
            self._event("exec_ok", request.target, extra="source=%s" % request.source)
            self._complete_execution(
                request,
                success=True,
                error_code=result.error_code.val,
                detail="execution succeeded",
                reason="goal_done",
            )
            return

        detail = "MoveGroup execute failed, error_code=%d" % result.error_code.val
        self._event("exec_fail", request.target, extra="source=%s" % request.source)
        self.get_logger().warn(detail)
        self._complete_execution(
            request,
            success=False,
            error_code=result.error_code.val,
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

        self._active_goal_handle = None
        if self._canceling_request is request:
            self._canceling_request = None
        self._active_request = None
        self._set_busy(False, reason)

        if request.execution_id is not None:
            self._publish_motion_execution(
                request.execution_id,
                status,
                error_code,
                detail,
            )

        self._start_pending_request()


def parse_args():
    parser = argparse.ArgumentParser(description="Target Pose to MoveIt executor")
    parser.add_argument("--target-topic", default="/rc_arm_2/target_pose")
    parser.add_argument("--middleware-target-topic", default="/arm2/middleware/motion_target")
    parser.add_argument("--middleware-result-topic", default="/arm2/middleware/motion_execution")
    parser.add_argument("--planning-group", default="arm")
    parser.add_argument("--joint-names", default="j1_joint,j2_joint,j3_joint,j4_joint")
    parser.add_argument("--default-frame", default="world")
    parser.add_argument("--move-action-name", default="/move_action")
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--urdf-path", default="")
    parser.add_argument("--pos-threshold", type=float, default=0.003)
    parser.add_argument("--rot-threshold", type=float, default=0.03)
    parser.add_argument("--planning-time", type=float, default=2.0)
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--vel-scale", type=float, default=0.5)
    parser.add_argument("--acc-scale", type=float, default=0.5)
    parser.add_argument("--joint-tolerance", type=float, default=0.02)
    parser.add_argument("--check-period", type=float, default=0.05)
    parser.add_argument("--avoid-collisions", action="store_true")
    parser.add_argument("--avoid-collisions-enabled", default="true")
    parser.add_argument("--j4-axis", choices=["x", "y", "z"], default="y")
    parser.add_argument("--status-log-period", type=float, default=1.0, help="state log period, <=0 to disable")
    parser.add_argument("--status-base-frame", default="world")
    parser.add_argument("--status-eef-frame", default="end_effector")
    parser.add_argument(
        "--world-boxes-json",
        default="[]",
        help='world collision boxes JSON list, e.g. [{"id":"keep_out","frame_id":"world","size":[0.2,0.2,0.2],"position":[0.3,0,0.3]}]',
    )
    parser.add_argument("--planning-scene-topic", default="/planning_scene")
    parser.add_argument("--scene-publish-retries", type=int, default=5)
    parser.add_argument("--middleware-preempt-interval-sec", type=float, default=0.25)
    parser.add_argument("--middleware-preempt-pos-threshold", type=float, default=0.01)
    parser.add_argument("--middleware-preempt-rot-threshold", type=float, default=0.08)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    joints = [j.strip() for j in args.joint_names.split(",") if j.strip()]
    if not joints:
        raise SystemExit("joint-names cannot be empty")

    rclpy.init()
    node = TargetPoseMoveItExecutor(
        target_topic=args.target_topic,
        middleware_target_topic=args.middleware_target_topic,
        middleware_result_topic=args.middleware_result_topic,
        planning_group=args.planning_group,
        joint_names=joints,
        default_frame=args.default_frame,
        move_action_name=args.move_action_name,
        pos_threshold=args.pos_threshold,
        rot_threshold=args.rot_threshold,
        planning_time=args.planning_time,
        planning_attempts=args.planning_attempts,
        vel_scale=args.vel_scale,
        acc_scale=args.acc_scale,
        joint_tolerance=args.joint_tolerance,
        check_period=args.check_period,
        avoid_collisions=args.avoid_collisions or _parse_bool(args.avoid_collisions_enabled),
        j4_axis=args.j4_axis,
        joint_state_topic=args.joint_state_topic,
        urdf_path=args.urdf_path,
        status_log_period=args.status_log_period,
        status_base_frame=args.status_base_frame,
        status_eef_frame=args.status_eef_frame,
        world_boxes_json=args.world_boxes_json,
        planning_scene_topic=args.planning_scene_topic,
        scene_publish_retries=args.scene_publish_retries,
        middleware_preempt_interval_sec=args.middleware_preempt_interval_sec,
        middleware_preempt_pos_threshold=args.middleware_preempt_pos_threshold,
        middleware_preempt_rot_threshold=args.middleware_preempt_rot_threshold,
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
