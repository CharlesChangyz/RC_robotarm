#!/usr/bin/env python3
"""Subscribe TF and publish PoseStamped target for rc_arm_2 simulation/control."""

import argparse

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def _normalize_frame_id(frame_id: str) -> str:
    return (frame_id or "").strip().lstrip("/")


class TfTargetPoseBridge(Node):
    def __init__(
        self,
        tf_topic: str,
        tf_static_topic: str,
        parent_frame: str,
        child_frame: str,
        target_pose_topic: str,
    ) -> None:
        super().__init__("rc_arm_tf_target_pose_bridge")

        self._parent_frame = _normalize_frame_id(parent_frame)
        self._child_frame = _normalize_frame_id(child_frame)

        self._pose_pub = self.create_publisher(PoseStamped, target_pose_topic, 20)

        self.create_subscription(TFMessage, tf_topic, self._on_tf_message, 50)
        if tf_static_topic and tf_static_topic != tf_topic:
            self.create_subscription(TFMessage, tf_static_topic, self._on_tf_message, 10)

        self.get_logger().info(
            "TF目标桥接已启动: %s -> %s, 发布到 %s"
            % (self._parent_frame or "*", self._child_frame, target_pose_topic)
        )

    def _on_tf_message(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            parent = _normalize_frame_id(transform.header.frame_id)
            child = _normalize_frame_id(transform.child_frame_id)

            if child != self._child_frame:
                continue
            if self._parent_frame and parent != self._parent_frame:
                continue

            pose_msg = PoseStamped()
            pose_msg.header = transform.header
            pose_msg.header.frame_id = parent
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            pose_msg.pose.orientation.x = transform.transform.rotation.x
            pose_msg.pose.orientation.y = transform.transform.rotation.y
            pose_msg.pose.orientation.z = transform.transform.rotation.z
            pose_msg.pose.orientation.w = transform.transform.rotation.w

            self._pose_pub.publish(pose_msg)


def parse_args():
    parser = argparse.ArgumentParser(description="TF to Pose target bridge")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--tf-static-topic", default="/tf_static")
    parser.add_argument("--parent-frame", default="world")
    parser.add_argument("--child-frame", default="rc_arm_2_target")
    parser.add_argument("--target-pose-topic", default="/rc_arm_2/target_pose")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if not _normalize_frame_id(args.child_frame):
        raise SystemExit("child-frame 不能为空")

    rclpy.init()
    node = TfTargetPoseBridge(
        tf_topic=args.tf_topic,
        tf_static_topic=args.tf_static_topic,
        parent_frame=args.parent_frame,
        child_frame=args.child_frame,
        target_pose_topic=args.target_pose_topic,
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
