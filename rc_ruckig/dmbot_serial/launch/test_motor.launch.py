from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="dmbot_serial",
                executable="test_motor",
                name="test_motor",
                output="screen",
            )
        ]
    )
