#!/usr/bin/env python3
"""Bridge camera-frame Arm2TargetPoint messages into the middleware frame."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs

from arm_msgs.msg import Arm2TargetPoint


DEFAULT_TARGET_FRAME = "world"


def transform_target_point(msg: Arm2TargetPoint, transform: TransformStamped) -> Arm2TargetPoint:
    point = PointStamped()
    point.header.stamp = transform.header.stamp
    point.header.frame_id = transform.child_frame_id
    point.point.x = float(msg.xyz.x)
    point.point.y = float(msg.xyz.y)
    point.point.z = float(msg.xyz.z)

    transformed = tf2_geometry_msgs.do_transform_point(point, transform)

    out = Arm2TargetPoint()
    out.xyz.x = float(transformed.point.x)
    out.xyz.y = float(transformed.point.y)
    out.xyz.z = float(transformed.point.z)
    out.target_spin_deg = float(msg.target_spin_deg)
    return out


class CameraTargetPointBridge(Node):
    def __init__(self) -> None:
        super().__init__("camera_target_point_bridge")

        self.declare_parameter("input_topic", "/arm2/camera_raw_dat")
        self.declare_parameter("output_topic", "/arm2/middleware/target_point")
        self.declare_parameter("source_frame", "camera_d435_optical_frame")
        self.declare_parameter("target_frame", DEFAULT_TARGET_FRAME)
        self.declare_parameter("tf_timeout_sec", 0.2)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._source_frame = str(self.get_parameter("source_frame").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._tf_timeout_sec = max(0.0, float(self.get_parameter("tf_timeout_sec").value))

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(Arm2TargetPoint, self._output_topic, 10)
        self.create_subscription(Arm2TargetPoint, self._input_topic, self._on_target_point, 20)

        self.get_logger().info(
            "camera_target_point_bridge ready: input=%s output=%s source_frame=%s target_frame=%s tf_timeout=%.3f"
            % (
                self._input_topic,
                self._output_topic,
                self._source_frame,
                self._target_frame,
                self._tf_timeout_sec,
            )
        )

    def _on_target_point(self, msg: Arm2TargetPoint) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                self._source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self._tf_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().warn(
                "dropping camera target point: cannot transform %s -> %s: %s"
                % (self._source_frame, self._target_frame, exc)
            )
            return

        out = transform_target_point(msg, transform)
        self._publisher.publish(out)
        self.get_logger().info(
            "published target point in %s: x=%.4f y=%.4f z=%.4f spin=%.2f deg"
            % (
                self._target_frame,
                out.xyz.x,
                out.xyz.y,
                out.xyz.z,
                out.target_spin_deg,
            )
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraTargetPointBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
