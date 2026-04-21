#!/usr/bin/env python3
"""
rc_arm_2 real hardware Xbox teleop launch
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware_config_file_arg = DeclareLaunchArgument(
        'hardware_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('rc_arm_description'),
            'config',
            'rc_arm_2',
            'rc_arm_2_hardware.real.yaml'
        ]),
        description='Hardware plugin configuration YAML'
    )

    controllers_file_arg = DeclareLaunchArgument(
        'controllers_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('rc_arm_description'),
            'config',
            'rc_arm_2',
            'rc_arm_2_controllers.yaml'
        ]),
        description='ros2_control controllers YAML'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2'
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rc_arm_moveit_config'),
                'launch',
                'rc_arm_2',
                'robot.launch.py'
            ])
        ]),
        launch_arguments={
            'hardware_config_file': LaunchConfiguration('hardware_config_file'),
            'controllers_file': LaunchConfiguration('controllers_file'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items()
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('device'),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )

    xbox_teleop_node = Node(
        package='rc_arm_teleop',
        executable='xbox_teleop_node_rc_arm_2',
        name='xbox_teleop_node_rc_arm_2',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('rc_arm_teleop'),
                'config',
                'rc_arm_2',
                'xbox_teleop.yaml'
            ])
        ],
        output='screen'
    )

    return LaunchDescription([
        hardware_config_file_arg,
        controllers_file_arg,
        device_arg,
        use_rviz_arg,
        moveit_launch,
        joy_node,
        xbox_teleop_node,
    ])
