from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'hardware_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('rc_arm_description'),
                'config',
                'rc_arm_2',
                'rc_arm_2_hardware.real.yaml'
            ]),
            description='Hardware plugin configuration YAML'),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('rc_arm_description'),
                'config',
                'rc_arm_2',
                'rc_arm_2_controllers.yaml'
            ]),
            description='ros2_control controllers YAML'),
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
            'hardware_config_file': LaunchConfiguration('hardware_config_file'),
            'controllers_file': LaunchConfiguration('controllers_file'),
            'device': LaunchConfiguration('device'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items()
    )

    return LaunchDescription(declared_arguments + [include_real])
