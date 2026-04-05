#!/usr/bin/env python3
"""Subscribe PoseStamped targets and drive MoveIt planning/execution."""

import argparse
import json
import math
import threading
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    CollisionObject,
    Constraints,
    JointConstraint,
    PlanningScene,
)
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
import tf2_ros


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


def _parse_bool(text: str) -> bool:
    v = (text or "").strip().lower()
    return v in {"1", "true", "yes", "y", "on"}


def _parse_forbidden_link_pairs(text: str) -> List[Tuple[str, str]]:
    pairs: List[Tuple[str, str]] = []
    for raw_item in (text or "").split(","):
        item = raw_item.strip()
        if not item or ":" not in item:
            continue
        a, b = [s.strip() for s in item.split(":", 1)]
        if not a or not b or a == b:
            continue
        pairs.append((a, b))
    return pairs


def _parse_env_forbidden_boxes_json(text: str) -> List[dict]:
    """Parse JSON array for environment forbidden boxes, fallback to [] on invalid input."""
    try:
        loaded = json.loads(text or "[]")
    except json.JSONDecodeError:
        return []
    return loaded if isinstance(loaded, list) else []


def _as_xyz(values, default: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Convert [x, y, z] like values to float tuple, otherwise return default."""
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        return default
    try:
        return float(values[0]), float(values[1]), float(values[2])
    except (TypeError, ValueError):
        return default


def _as_xyzw(values, default: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    """Convert [x, y, z, w] like values to float tuple, otherwise return default."""
    if not isinstance(values, (list, tuple)) or len(values) != 4:
        return default
    try:
        return float(values[0]), float(values[1]), float(values[2]), float(values[3])
    except (TypeError, ValueError):
        return default


class TargetPoseMoveItExecutor(Node):
    def __init__(
        self,
        target_topic: str,
        planning_group: str,
        joint_names: List[str],
        default_frame: str,
        move_action_name: str,
        compute_ik_service: str,
        pos_threshold: float,
        rot_threshold: float,
        planning_time: float,
        planning_attempts: int,
        vel_scale: float,
        acc_scale: float,
        joint_tolerance: float,
        check_period: float,
        avoid_collisions: bool,
        enforce_j4_from_target: bool,
        j4_joint_name: str,
        j4_axis: str,
        status_log_period: float,
        status_base_frame: str,
        status_eef_frame: str,
        forbidden_link_pairs: List[Tuple[str, str]],
        env_forbidden_boxes: List[dict],
        planning_scene_topic: str,
    ) -> None:
        super().__init__("rc_arm_target_pose_moveit_executor")

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
        self._enforce_j4_from_target = bool(enforce_j4_from_target)
        self._j4_joint_name = str(j4_joint_name).strip() or "j4_joint"
        self._j4_axis = str(j4_axis).strip().lower() if str(j4_axis).strip() else "x"
        if self._j4_axis not in {"x", "y", "z"}:
            self._j4_axis = "x"
        self._status_log_period = max(0.0, float(status_log_period))

        self._status_base_frame = _normalize_frame_id(status_base_frame)
        self._status_eef_frame = _normalize_frame_id(status_eef_frame)
        self._forbidden_link_pairs = list(forbidden_link_pairs)
        self._env_forbidden_boxes = list(env_forbidden_boxes)
        self._planning_scene_topic = planning_scene_topic
        self._planning_scene_initialized = False

        self._target_lock = threading.Lock()
        self._latest_target: Optional[PoseStamped] = None
        self._last_sent_target: Optional[PoseStamped] = None

        self._busy = False
        self._ready_move_action = False
        self._ready_compute_ik = False
        self._last_ready_tuple = None

        self._last_event = "init"
        self._last_event_time_sec = self._now_sec()

        self._ik_client = self.create_client(GetPositionIK, compute_ik_service)
        self._move_group_client = ActionClient(self, MoveGroup, move_action_name)
        self._planning_scene_pub = self.create_publisher(PlanningScene, planning_scene_topic, 10)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(PoseStamped, target_topic, self._on_target, 20)
        self._timer = self.create_timer(max(0.02, float(check_period)), self._on_timer)
        if self._status_log_period > 0.0:
            self._status_timer = self.create_timer(self._status_log_period, self._log_status)
        self._scene_timer = self.create_timer(1.0, self._publish_planning_scene_diff_once)

        self.get_logger().info(
            "TargetPose->MoveIt executor started: topic=%s group=%s status_tf=%s->%s"
            % (target_topic, self._planning_group, self._status_base_frame, self._status_eef_frame)
        )

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
        mg_ready = self._move_group_client.server_is_ready()
        ik_ready = self._ik_client.service_is_ready()

        self._ready_move_action = mg_ready
        self._ready_compute_ik = ik_ready

        ready_tuple = (mg_ready, ik_ready)
        if ready_tuple != self._last_ready_tuple:
            self.get_logger().info(
                "[STATE] ready move_action=%d compute_ik=%d %s"
                % (1 if mg_ready else 0, 1 if ik_ready else 0, self._format_eef())
            )
            self._last_ready_tuple = ready_tuple

    def _log_status(self) -> None:
        event_age = max(0.0, self._now_sec() - self._last_event_time_sec)
        with self._target_lock:
            has_target = self._latest_target is not None
        self.get_logger().info(
            "[STATE] busy=%d ready(move_action=%d,ik=%d) target=%d event=%s(%.2fs) %s"
            % (
                1 if self._busy else 0,
                1 if self._ready_move_action else 0,
                1 if self._ready_compute_ik else 0,
                1 if has_target else 0,
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

    def _on_target(self, msg: PoseStamped) -> None:
        pose_msg = self._resolve_target_pose(msg)
        with self._target_lock:
            self._latest_target = pose_msg
        self._event("target_rx", pose_msg)

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
        self._update_ready()

        if self._busy:
            return
        if not (self._ready_move_action and self._ready_compute_ik):
            return

        with self._target_lock:
            target = _copy_pose_stamped(self._latest_target) if self._latest_target is not None else None

        if target is None:
            return
        if self._last_sent_target is not None and not self._target_changed(self._last_sent_target, target):
            return

        self._set_busy(True, "request_ik")
        self._event("ik_request", target)
        self._request_ik(target)

    def _build_acm_forbidden_links(self) -> Optional[AllowedCollisionMatrix]:
        if not self._forbidden_link_pairs:
            return None

        names: List[str] = []
        for a, b in self._forbidden_link_pairs:
            if a not in names:
                names.append(a)
            if b not in names:
                names.append(b)
        if not names:
            return None

        idx = {name: i for i, name in enumerate(names)}
        matrix = [[True for _ in names] for _ in names]
        for a, b in self._forbidden_link_pairs:
            i, j = idx[a], idx[b]
            matrix[i][j] = False
            matrix[j][i] = False

        acm = AllowedCollisionMatrix()
        acm.entry_names = names
        for i in range(len(names)):
            row = AllowedCollisionEntry()
            row.enabled = [matrix[i][j] for j in range(len(names))]
            acm.entry_values.append(row)
        return acm

    def _build_env_collision_objects(self) -> List[CollisionObject]:
        objects: List[CollisionObject] = []
        for idx, box_cfg in enumerate(self._env_forbidden_boxes, start=1):
            if not isinstance(box_cfg, dict):
                continue
            sx, sy, sz = _as_xyz(box_cfg.get("size"), (0.2, 0.2, 0.2))
            px, py, pz = _as_xyz(box_cfg.get("position"), (0.0, 0.0, 0.0))
            qx, qy, qz, qw = _as_xyzw(box_cfg.get("orientation"), (0.0, 0.0, 0.0, 1.0))
            frame_id = _normalize_frame_id(str(box_cfg.get("frame_id", ""))) or self._default_frame
            obj_id = str(box_cfg.get("id", f"forbidden_box_{idx}")).strip() or f"forbidden_box_{idx}"

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [abs(sx), abs(sy), abs(sz)]

            pose = Pose()
            pose.position.x = px
            pose.position.y = py
            pose.position.z = pz
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw

            obj = CollisionObject()
            obj.id = obj_id
            obj.header.frame_id = frame_id
            obj.primitives = [primitive]
            obj.primitive_poses = [pose]
            obj.operation = CollisionObject.ADD
            objects.append(obj)
        return objects

    def _publish_planning_scene_diff_once(self) -> None:
        if self._planning_scene_initialized:
            return
        self._planning_scene_initialized = True
        self._scene_timer.cancel()

        scene = PlanningScene()
        scene.is_diff = True

        acm = self._build_acm_forbidden_links()
        if acm is not None:
            scene.allowed_collision_matrix = acm

        env_objects = self._build_env_collision_objects()
        if env_objects:
            scene.world.collision_objects = env_objects

        if acm is None and not env_objects:
            self.get_logger().info("No extra collision forbidden zones configured, skip PlanningScene diff")
            return

        self._planning_scene_pub.publish(scene)
        self.get_logger().info(
            "Collision forbidden zones published: self_pairs=%d, env_boxes=%d"
            % (len(self._forbidden_link_pairs), len(env_objects))
        )

    def _request_ik(self, target: PoseStamped) -> None:
        req = GetPositionIK.Request()
        req.ik_request.group_name = self._planning_group
        req.ik_request.avoid_collisions = self._avoid_collisions
        req.ik_request.robot_state.is_diff = False
        req.ik_request.pose_stamped = self._resolve_target_pose(target)

        sec = int(self._planning_time)
        nsec = int((self._planning_time - sec) * 1e9)
        req.ik_request.timeout.sec = sec
        req.ik_request.timeout.nanosec = nsec

        future = self._ik_client.call_async(req)
        future.add_done_callback(lambda f, target=target: self._on_ik_done(f, target))

    def _extract_j4_target_rad(self, target: PoseStamped) -> float:
        q = target.pose.orientation
        qx, qy, qz, qw = _normalize_quat_xyzw((float(q.x), float(q.y), float(q.z), float(q.w)))
        axis_comp = qx
        if self._j4_axis == "y":
            axis_comp = qy
        elif self._j4_axis == "z":
            axis_comp = qz

        angle = 2.0 * math.atan2(axis_comp, qw)
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _extract_joint_targets(self, joint_state) -> Optional[Dict[str, float]]:
        mapping = {name: float(pos) for name, pos in zip(joint_state.name, joint_state.position)}
        missing = [j for j in self._joint_names if j not in mapping]
        if missing:
            self.get_logger().warn("IK result missing joints: %s" % ", ".join(missing))
            return None
        return {j: mapping[j] for j in self._joint_names}

    def _on_ik_done(self, future, target: PoseStamped) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._event("ik_exception", target)
            self.get_logger().warn(f"IK request exception: {exc}")
            self._set_busy(False, "ik_exception")
            return

        if response is None:
            self._event("ik_empty", target)
            self.get_logger().warn("IK empty response")
            self._set_busy(False, "ik_empty")
            return

        if response.error_code.val != response.error_code.SUCCESS:
            self._event("ik_fail", target)
            self.get_logger().warn(
                "IK failed, error_code=%d %s %s"
                % (response.error_code.val, self._format_target(target), self._format_eef())
            )
            self._set_busy(False, "ik_fail")
            return

        q_target = self._extract_joint_targets(response.solution.joint_state)
        if q_target is None:
            self._event("ik_missing_joint", target)
            self._set_busy(False, "ik_missing_joint")
            return

        if self._enforce_j4_from_target:
            if self._j4_joint_name in q_target:
                q_target[self._j4_joint_name] = self._extract_j4_target_rad(target)
            else:
                self.get_logger().warn(
                    "j4 override skipped: joint '%s' not found in IK result" % self._j4_joint_name
                )

        self._send_goal(target, q_target)

    def _send_goal(self, target: PoseStamped, q_target: Dict[str, float]) -> None:
        goal = MoveGroup.Goal()
        goal.request.group_name = self._planning_group
        goal.request.num_planning_attempts = self._planning_attempts
        goal.request.allowed_planning_time = self._planning_time
        goal.request.max_velocity_scaling_factor = self._vel_scale
        goal.request.max_acceleration_scaling_factor = self._acc_scale

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

        self._event("goal_send", target)
        send_future = self._move_group_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f, target=target, q_target=dict(q_target): self._on_goal_response(f, target, q_target)
        )

    def _on_goal_response(self, future, target: PoseStamped, q_target: Dict[str, float]) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._event("goal_send_exception", target)
            self.get_logger().warn(f"MoveGroup send exception: {exc}")
            self._set_busy(False, "goal_send_exception")
            return

        if goal_handle is None or not goal_handle.accepted:
            self._event("goal_rejected", target)
            self.get_logger().warn("MoveGroup goal rejected")
            self._set_busy(False, "goal_rejected")
            return

        self._event("goal_accepted", target)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, target=target, q_target=dict(q_target): self._on_goal_result(f, target, q_target)
        )

    def _on_goal_result(self, future, target: PoseStamped, q_target: Dict[str, float]) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as exc:
            self._event("exec_result_exception", target)
            self.get_logger().warn(f"MoveGroup result exception: {exc}")
            self._set_busy(False, "exec_result_exception")
            return

        if result.error_code.val == result.error_code.SUCCESS:
            self._last_sent_target = _copy_pose_stamped(target)
            self._event("exec_ok", target)
        else:
            self._event("exec_fail", target)
            self.get_logger().warn(f"MoveGroup execute failed, error_code={result.error_code.val}")

        self._set_busy(False, "goal_done")


def parse_args():
    parser = argparse.ArgumentParser(description="Target Pose to MoveIt executor")
    parser.add_argument("--target-topic", default="/rc_arm_2/target_pose")
    parser.add_argument("--planning-group", default="arm")
    parser.add_argument("--joint-names", default="j1_joint,j2_joint,j3_joint,j4_joint")
    parser.add_argument("--default-frame", default="world")
    parser.add_argument("--move-action-name", default="/move_action")
    parser.add_argument("--compute-ik-service", default="/compute_ik")
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
    parser.add_argument("--enforce-j4-from-target", default="true")
    parser.add_argument("--j4-joint-name", default="j4_joint")
    parser.add_argument("--j4-axis", choices=["x", "y", "z"], default="x")
    parser.add_argument("--status-log-period", type=float, default=1.0, help="state log period, <=0 to disable")
    parser.add_argument("--status-base-frame", default="world")
    parser.add_argument("--status-eef-frame", default="end_effector")
    parser.add_argument(
        "--forbidden-link-pairs",
        default="",
        help="forbid robot self-collision pairs, format: link_a:link_b,link_c:link_d",
    )
    parser.add_argument(
        "--env-forbidden-boxes-json",
        default="[]",
        help='environment forbidden boxes in JSON list, e.g. [{"id":"box1","frame_id":"world","size":[0.2,0.2,0.2],"position":[0.3,0,0.4]}]',
    )
    parser.add_argument("--planning-scene-topic", default="/planning_scene")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    joints = [j.strip() for j in args.joint_names.split(",") if j.strip()]
    if not joints:
        raise SystemExit("joint-names cannot be empty")

    rclpy.init()
    node = TargetPoseMoveItExecutor(
        target_topic=args.target_topic,
        planning_group=args.planning_group,
        joint_names=joints,
        default_frame=args.default_frame,
        move_action_name=args.move_action_name,
        compute_ik_service=args.compute_ik_service,
        pos_threshold=args.pos_threshold,
        rot_threshold=args.rot_threshold,
        planning_time=args.planning_time,
        planning_attempts=args.planning_attempts,
        vel_scale=args.vel_scale,
        acc_scale=args.acc_scale,
        joint_tolerance=args.joint_tolerance,
        check_period=args.check_period,
        avoid_collisions=(args.avoid_collisions or _parse_bool(args.avoid_collisions_enabled)),
        enforce_j4_from_target=_parse_bool(args.enforce_j4_from_target),
        j4_joint_name=args.j4_joint_name,
        j4_axis=args.j4_axis,
        status_log_period=args.status_log_period,
        status_base_frame=args.status_base_frame,
        status_eef_frame=args.status_eef_frame,
        forbidden_link_pairs=_parse_forbidden_link_pairs(args.forbidden_link_pairs),
        env_forbidden_boxes=_parse_env_forbidden_boxes_json(args.env_forbidden_boxes_json),
        planning_scene_topic=args.planning_scene_topic,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
