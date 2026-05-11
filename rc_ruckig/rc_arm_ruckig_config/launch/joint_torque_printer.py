#!/usr/bin/env python3
"""Subscribe JointState and print per-joint torque values periodically."""

import argparse
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _format_joint_name(name: str) -> str:
    return name.lstrip('/')


class JointTorquePrinter(Node):
    def __init__(self, topic: str, rate: float) -> None:
        super().__init__('rc_arm_joint_torque_printer')

        self._lock = threading.Lock()
        self._latest_names = []
        self._latest_efforts = []

        self.create_subscription(JointState, topic, self._on_joint_state, 20)

        period = 1.0 / max(rate, 0.1)
        self.create_timer(period, self._on_timer)

        self.get_logger().info(f'关节力矩打印已启动: topic={topic}, rate={rate:.2f}Hz')

    def _on_joint_state(self, msg: JointState) -> None:
        if not msg.effort:
            return

        names = list(msg.name)
        efforts = list(msg.effort)

        # 兼容 name 长度不完整的情况
        if len(names) < len(efforts):
            names.extend([f'joint_{i+1}' for i in range(len(names), len(efforts))])

        with self._lock:
            self._latest_names = names
            self._latest_efforts = efforts

    def _on_timer(self) -> None:
        with self._lock:
            names = self._latest_names[:]
            efforts = self._latest_efforts[:]

        if not efforts:
            return

        pairs = []
        for i, tau in enumerate(efforts):
            name = _format_joint_name(names[i]) if i < len(names) else f'joint_{i+1}'
            pairs.append(f'{name}={tau:.3f}Nm')

        self.get_logger().info(' | '.join(pairs))


def parse_args():
    parser = argparse.ArgumentParser(description='Print per-joint torque from JointState topic')
    parser.add_argument('--topic', default='/debug/final_joint_command')
    parser.add_argument('--rate', type=float, default=10.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    rclpy.init()
    node = JointTorquePrinter(topic=args.topic, rate=args.rate)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
