#!/usr/bin/env python3
"""Lightweight Xbox teleop node for the RC arm Ruckig pipeline."""

import math
from typing import List, Optional

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rc_arm_kinematics import RcArmWorldPitchKinematics


class XboxTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "xbox_teleop_node_rc_arm_2",
            automatically_declare_parameters_from_overrides=True,
        )
        self.update_rate = float(self.get_parameter("update_rate").value)
        self.max_linear_velocity = float(self.get_parameter("max_linear_velocity").value)
        self.max_angular_velocity = float(self.get_parameter("max_angular_velocity").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.joint_names = ["j1_joint", "j2_joint", "j3_joint", "j4_joint"]

        self.kinematics = RcArmWorldPitchKinematics(joint_names=self.joint_names, j4_axis="x")
        self.pose_pub = self.create_publisher(PoseStamped, "/rc_arm_2/target_pose", 20)
        self.action_client = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.create_subscription(Joy, "/joy", self._on_joy, 20)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)

        self.current_joint_positions: Optional[List[float]] = None
        self.current_joy: Optional[Joy] = None
        self.prev_buttons: List[int] = []
        self.target_xyz: Optional[List[float]] = None
        self.target_pitch: float = 0.0
        self.speed_scale = 1.0
        self.create_timer(max(0.01, 1.0 / max(1.0, self.update_rate)), self._on_timer)

    def _on_joint_state(self, msg: JointState) -> None:
        name_to_index = {name: idx for idx, name in enumerate(msg.name)}
        try:
            self.current_joint_positions = [float(msg.position[name_to_index[name]]) for name in self.joint_names]
        except KeyError:
            return

        if self.target_xyz is None and self.current_joint_positions is not None:
            x, y, z = self.kinematics.forward_position(self.current_joint_positions)
            self.target_xyz = [x, y, z]
            self.target_pitch = self.kinematics.forward_world_pitch(self.current_joint_positions)

    def _on_joy(self, msg: Joy) -> None:
        self.current_joy = msg
        if not self.prev_buttons:
            self.prev_buttons = list(msg.buttons)
            return

        if self._pressed(msg.buttons, 0):
            self.speed_scale = 0.4 if self.speed_scale > 0.7 else 1.0
        if self._pressed(msg.buttons, 1):
            self._send_joint_goal([0.0, 0.785, -0.785, 0.0], 2.5)
        if self._pressed(msg.buttons, 3):
            self._send_joint_goal([0.0, 0.0, 0.0, 0.0], 2.5)
        self.prev_buttons = list(msg.buttons)

    def _pressed(self, buttons: List[int], index: int) -> bool:
        return index < len(buttons) and index < len(self.prev_buttons) and buttons[index] == 1 and self.prev_buttons[index] == 0

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def _trigger_value(self, value: float) -> float:
        return max(0.0, min(1.0, (1.0 - value) / 2.0))

    def _on_timer(self) -> None:
        if self.current_joy is None or self.target_xyz is None:
            return

        axes = self.current_joy.axes
        dt = 1.0 / max(1.0, self.update_rate)
        vx = self._apply_deadzone(axes[1] if len(axes) > 1 else 0.0)
        vy = self._apply_deadzone(axes[0] if len(axes) > 0 else 0.0)
        pitch_axis = self._apply_deadzone(axes[4] if len(axes) > 4 else 0.0)
        z_up = self._trigger_value(axes[5] if len(axes) > 5 else 1.0)
        z_down = self._trigger_value(axes[2] if len(axes) > 2 else 1.0)

        linear_step = self.max_linear_velocity * self.speed_scale * dt
        angular_step = self.max_angular_velocity * self.speed_scale * dt
        self.target_xyz[0] += vx * linear_step
        self.target_xyz[1] += vy * linear_step
        self.target_xyz[2] += (z_up - z_down) * linear_step
        self.target_pitch += pitch_axis * angular_step
        self._publish_target_pose()

    def _publish_target_pose(self) -> None:
        if self.target_xyz is None:
            return
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = float(self.target_xyz[0])
        pose.pose.position.y = float(self.target_xyz[1])
        pose.pose.position.z = float(self.target_xyz[2])
        qx, qy, qz, qw = self.kinematics.quaternion_from_world_pitch(self.target_pitch)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pose_pub.publish(pose)

    def _send_joint_goal(self, positions: List[float], duration: float) -> None:
        if not self.action_client.server_is_ready():
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(self.joint_names)

        start = JointTrajectoryPoint()
        start.positions = self.current_joint_positions if self.current_joint_positions else [0.0] * len(self.joint_names)
        start.time_from_start.sec = 0
        goal.trajectory.points.append(start)

        end = JointTrajectoryPoint()
        end.positions = list(positions)
        end.time_from_start.sec = int(math.floor(duration))
        end.time_from_start.nanosec = int((duration - end.time_from_start.sec) * 1.0e9)
        goal.trajectory.points.append(end)
        self.action_client.send_goal_async(goal)


def main() -> None:
    rclpy.init()
    node = XboxTeleopNode()
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
