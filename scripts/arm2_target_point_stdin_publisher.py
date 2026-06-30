#!/usr/bin/env python3
"""Publish Arm2TargetPoint messages from JSON lines read on stdin."""

from __future__ import annotations

import argparse
import json
import sys

import rclpy
from rclpy.node import Node

from arm_msgs.msg import Arm2TargetPoint


class StdinTargetPointPublisher(Node):
    def __init__(self, topic: str, node_name: str) -> None:
        super().__init__(node_name)
        self._topic = topic
        self._publisher = self.create_publisher(Arm2TargetPoint, topic, 10)
        self.get_logger().info(f"stdin target point publisher ready on {topic}")

    def publish_payload(self, payload: dict) -> None:
        msg = Arm2TargetPoint()
        msg.xyz.x = float(payload["x"])
        msg.xyz.y = float(payload["y"])
        msg.xyz.z = float(payload["z"])
        msg.target_spin_deg = float(payload.get("target_spin_deg", 0.0))
        self._publisher.publish(msg)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish Arm2TargetPoint from stdin JSON lines.")
    parser.add_argument("--topic", default="/arm2/middleware/target_point")
    parser.add_argument("--node-name", default="arm2_target_point_stdin_publisher")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init(args=None)
    node = StdinTargetPointPublisher(topic=args.topic, node_name=args.node_name)
    try:
        for raw_line in sys.stdin:
            line = raw_line.strip()
            if not line:
                continue
            if line == "__quit__":
                break
            try:
                payload = json.loads(line)
                node.publish_payload(payload)
            except Exception as exc:
                node.get_logger().warning(f"failed to publish payload: {exc}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
