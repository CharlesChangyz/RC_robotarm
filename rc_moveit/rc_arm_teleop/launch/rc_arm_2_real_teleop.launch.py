from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('can_interface', default_value='can0', description='CAN interface name'),
        DeclareLaunchArgument('host_can_id', default_value='253', description='Host CAN ID (0xFD = 253)'),
        DeclareLaunchArgument('s_curve_enabled', default_value='true', description='Enable S-curve smoothing'),
        DeclareLaunchArgument('smoothing_alpha', default_value='0.2', description='Smoothing alpha'),
        DeclareLaunchArgument('max_velocity', default_value='2.0', description='S-curve max velocity (rad/s)'),
        DeclareLaunchArgument('max_acceleration', default_value='8.0', description='S-curve max acceleration (rad/s^2)'),
        DeclareLaunchArgument('max_jerk', default_value='50.0', description='S-curve max jerk (rad/s^3)'),
        DeclareLaunchArgument('device', default_value='/dev/input/js0', description='Joystick device path'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Start RViz2'),
    ]

    include_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rc_arm_teleop'),
                'launch',
                'rc_arm_2',
                'real_teleop.launch.py'
            ])
        ),
        launch_arguments={
            'can_interface': LaunchConfiguration('can_interface'),
            'host_can_id': LaunchConfiguration('host_can_id'),
            's_curve_enabled': LaunchConfiguration('s_curve_enabled'),
            'smoothing_alpha': LaunchConfiguration('smoothing_alpha'),
            'max_velocity': LaunchConfiguration('max_velocity'),
            'max_acceleration': LaunchConfiguration('max_acceleration'),
            'max_jerk': LaunchConfiguration('max_jerk'),
            'device': LaunchConfiguration('device'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items()
    )

    return LaunchDescription(declared_arguments + [include_real])
