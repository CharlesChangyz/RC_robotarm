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
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name'
    )

    host_can_id_arg = DeclareLaunchArgument(
        'host_can_id',
        default_value='253',
        description='Host CAN ID (0xFD = 253)'
    )

    s_curve_enabled_arg = DeclareLaunchArgument(
        's_curve_enabled',
        default_value='true',
        description='Enable S-curve smoothing in hardware interface'
    )

    smoothing_alpha_arg = DeclareLaunchArgument(
        'smoothing_alpha',
        default_value='0.2',
        description='Smoothing alpha used by hardware interface'
    )

    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='2.0',
        description='S-curve max velocity (rad/s)'
    )

    max_acceleration_arg = DeclareLaunchArgument(
        'max_acceleration',
        default_value='8.0',
        description='S-curve max acceleration (rad/s^2)'
    )

    max_jerk_arg = DeclareLaunchArgument(
        'max_jerk',
        default_value='50.0',
        description='S-curve max jerk (rad/s^3)'
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
            'can_interface': LaunchConfiguration('can_interface'),
            'host_can_id': LaunchConfiguration('host_can_id'),
            's_curve_enabled': LaunchConfiguration('s_curve_enabled'),
            'smoothing_alpha': LaunchConfiguration('smoothing_alpha'),
            'max_velocity': LaunchConfiguration('max_velocity'),
            'max_acceleration': LaunchConfiguration('max_acceleration'),
            'max_jerk': LaunchConfiguration('max_jerk'),
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
        can_interface_arg,
        host_can_id_arg,
        s_curve_enabled_arg,
        smoothing_alpha_arg,
        max_velocity_arg,
        max_acceleration_arg,
        max_jerk_arg,
        device_arg,
        use_rviz_arg,
        moveit_launch,
        joy_node,
        xbox_teleop_node,
    ])
