#!/usr/bin/env python3
"""Sync /rc_arm_2/payload_active into the MoveIt planning scene."""

import argparse
import ast
from pathlib import Path
from typing import Tuple

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool


def _parse_scalar(raw_value: str):
    value = raw_value.strip()
    if not value:
        return ""

    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    if lowered in {"null", "none", "~"}:
        return None

    try:
        return ast.literal_eval(value)
    except (SyntaxError, ValueError):
        return value


def _load_flat_yaml(path: Path) -> dict:
    data = {}
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if ":" not in line:
            raise ValueError(f"unsupported config line {line_number}: {raw_line}")

        key, raw_value = line.split(":", 1)
        key = key.strip()
        if not key:
            raise ValueError(f"empty config key on line {line_number}")

        value = raw_value.split("#", 1)[0]
        data[key] = _parse_scalar(value)
    return data


def _load_config(config_path: str) -> dict:
    path = Path(config_path)
    if not path.is_file():
        raise FileNotFoundError(f"hardware config not found: {config_path}")
    data = _load_flat_yaml(path)
    if not isinstance(data, dict):
        raise ValueError(f"hardware config must be a mapping: {config_path}")
    return data


def _xyz(cfg: dict, prefix: str, fallback: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (
        float(cfg.get(f"{prefix}_x", fallback[0])),
        float(cfg.get(f"{prefix}_y", fallback[1])),
        float(cfg.get(f"{prefix}_z", fallback[2])),
    )


class PayloadSceneSync(Node):
    def __init__(self, hardware_config_file: str, planning_scene_topic: str, object_id: str) -> None:
        super().__init__("rc_arm_payload_scene_sync")
        cfg = _load_config(hardware_config_file)

        self._payload_active_topic = str(cfg.get("payload_active_topic", "/rc_arm_2/payload_active"))
        self._payload_frame = str(cfg.get("payload_frame", "end_effector")).strip() or "end_effector"
        self._payload_box_size = _xyz(cfg, "payload_box_size", (0.05, 0.05, 0.05))
        self._payload_offset = _xyz(cfg, "payload_com_offset", (0.0, 0.0, 0.0))
        self._object_id = object_id
        self._last_state = None

        scene_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._planning_scene_pub = self.create_publisher(PlanningScene, planning_scene_topic, scene_qos)
        self.create_subscription(Bool, self._payload_active_topic, self._on_payload_active, 10)

        self.get_logger().info(
            "payload scene sync started: topic=%s object_id=%s link=%s size=%s offset=%s"
            % (
                self._payload_active_topic,
                self._object_id,
                self._payload_frame,
                self._payload_box_size,
                self._payload_offset,
            )
        )

    def _build_scene(self, payload_active: bool) -> PlanningScene:
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        attached = AttachedCollisionObject()
        attached.link_name = self._payload_frame
        attached.touch_links = [self._payload_frame, "l4"]
        attached.object.id = self._object_id

        if payload_active:
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = list(self._payload_box_size)
            pose = Pose()
            pose.position.x = float(self._payload_offset[0])
            pose.position.y = float(self._payload_offset[1])
            pose.position.z = float(self._payload_offset[2])
            pose.orientation.w = 1.0

            attached.object.header.frame_id = self._payload_frame
            attached.object.operation = CollisionObject.ADD
            attached.object.primitives = [primitive]
            attached.object.primitive_poses = [pose]
        else:
            attached.object.operation = CollisionObject.REMOVE

        scene.robot_state.attached_collision_objects = [attached]
        return scene

    def _on_payload_active(self, msg: Bool) -> None:
        if self._last_state is not None and self._last_state == msg.data:
            return
        self._last_state = msg.data
        self._planning_scene_pub.publish(self._build_scene(msg.data))
        self.get_logger().info(
            "payload scene updated: payload_active=%s" % ("true" if msg.data else "false")
        )


def parse_args():
    parser = argparse.ArgumentParser(description="Sync payload_active to the MoveIt planning scene")
    parser.add_argument("--hardware-config-file", required=True)
    parser.add_argument("--planning-scene-topic", default="/planning_scene")
    parser.add_argument("--object-id", default="payload_block")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = PayloadSceneSync(
        hardware_config_file=args.hardware_config_file,
        planning_scene_topic=args.planning_scene_topic,
        object_id=args.object_id,
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
