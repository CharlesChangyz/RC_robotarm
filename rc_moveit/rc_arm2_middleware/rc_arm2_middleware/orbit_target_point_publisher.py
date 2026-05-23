#!/usr/bin/env python3
"""Continuously publish Arm2TargetPoint messages around a small XY orbit."""

from __future__ import annotations

import math
import random
import time
from typing import Optional, Sequence

import rclpy
from rclpy.node import Node

from arm_msgs.msg import Arm2TargetPoint


class OrbitTargetPointPublisher(Node):
    def __init__(self) -> None:
        super().__init__("arm2_orbit_target_point_publisher")

        self.declare_parameter("target_point_topic", "/arm2/middleware/target_point")
        self.declare_parameter("center_x", 0.3)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("center_z", 0.3)
        self.declare_parameter("radius_xy", 0.02)
        self.declare_parameter("orbit_period_sec", 12.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("spin_center_deg", 0.0)
        self.declare_parameter("spin_random_range_deg", 8.0)
        self.declare_parameter("random_seed", -1)

        self._target_point_topic = str(self.get_parameter("target_point_topic").value)
        self._center_x = float(self.get_parameter("center_x").value)
        self._center_y = float(self.get_parameter("center_y").value)
        self._center_z = float(self.get_parameter("center_z").value)
        self._radius_xy = max(0.0, float(self.get_parameter("radius_xy").value))
        self._orbit_period_sec = max(0.1, float(self.get_parameter("orbit_period_sec").value))
        self._publish_rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self._spin_center_deg = float(self.get_parameter("spin_center_deg").value)
        self._spin_random_range_deg = max(
            0.0, float(self.get_parameter("spin_random_range_deg").value)
        )
        self._random_seed = int(self.get_parameter("random_seed").value)

        if self._random_seed >= 0:
            self._rng = random.Random(self._random_seed)
        else:
            self._rng = random.Random()

        self._publisher = self.create_publisher(Arm2TargetPoint, self._target_point_topic, 10)
        self._start_time = time.monotonic()
        self._publish_count = 0

        timer_period = 1.0 / self._publish_rate_hz
        self.create_timer(timer_period, self._on_timer)

        self.get_logger().info(
            "orbit target publisher ready: topic=%s center=(%.4f, %.4f, %.4f) radius=%.4f "
            "orbit_period=%.2f rate=%.2f spin_center=%.2f spin_range=+/-%.2f seed=%d"
            % (
                self._target_point_topic,
                self._center_x,
                self._center_y,
                self._center_z,
                self._radius_xy,
                self._orbit_period_sec,
                self._publish_rate_hz,
                self._spin_center_deg,
                self._spin_random_range_deg,
                self._random_seed,
            )
        )

        self._publish_target_point()

    def _on_timer(self) -> None:
        self._publish_target_point()

    def _publish_target_point(self) -> None:
        elapsed_sec = time.monotonic() - self._start_time
        angle_rad = (elapsed_sec / self._orbit_period_sec) * 2.0 * math.pi

        x = self._center_x + self._radius_xy * math.cos(angle_rad)
        y = self._center_y + self._radius_xy * math.sin(angle_rad)
        z = self._center_z
        target_spin_deg = self._spin_center_deg + self._rng.uniform(
            -self._spin_random_range_deg,
            self._spin_random_range_deg,
        )

        msg = Arm2TargetPoint()
        msg.xyz.x = float(x)
        msg.xyz.y = float(y)
        msg.xyz.z = float(z)
        msg.target_spin_deg = float(target_spin_deg)
        self._publisher.publish(msg)

        self._publish_count += 1
        if self._publish_count == 1 or self._publish_count % max(1, int(self._publish_rate_hz)) == 0:
            self.get_logger().info(
                "published target point x=%.4f y=%.4f z=%.4f spin=%.2f deg"
                % (x, y, z, target_spin_deg)
            )


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = OrbitTargetPointPublisher()
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
