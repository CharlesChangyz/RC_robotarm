#!/usr/bin/env python3
"""Subscribe JointState and print per-joint position values periodically."""

import argparse
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _format_joint_name(name: str) -> str:
    return name.lstrip("/")


class JointPositionPrinter(Node):
    def __init__(self, topic: str, rate: float) -> None:
        super().__init__("rc_arm_joint_position_printer")

        self._lock = threading.Lock()
        self._latest_names = []
        self._latest_positions = []

        self.create_subscription(JointState, topic, self._on_joint_state, 20)

        period = 1.0 / max(rate, 0.1)
        self.create_timer(period, self._on_timer)

        self.get_logger().info(f"关节角度打印已启动: topic={topic}, rate={rate:.2f}Hz")

    def _on_joint_state(self, msg: JointState) -> None:
        if not msg.position:
            return

        names = list(msg.name)
        positions = list(msg.position)

        if len(names) < len(positions):
            names.extend([f"joint_{i+1}" for i in range(len(names), len(positions))])

        with self._lock:
            self._latest_names = names
            self._latest_positions = positions

    def _on_timer(self) -> None:
        with self._lock:
            names = self._latest_names[:]
            positions = self._latest_positions[:]

        if not positions:
            return

        pairs = []
        for i, q in enumerate(positions):
            name = _format_joint_name(names[i]) if i < len(names) else f"joint_{i+1}"
            pairs.append(f"{name}={q:.4f}rad")

        self.get_logger().info(" | ".join(pairs))


def parse_args():
    parser = argparse.ArgumentParser(description="Print per-joint position from JointState topic")
    parser.add_argument("--topic", default="/rc_arm_2/mujoco_joint_positions")
    parser.add_argument("--rate", type=float, default=10.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    rclpy.init()
    node = JointPositionPrinter(topic=args.topic, rate=args.rate)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

