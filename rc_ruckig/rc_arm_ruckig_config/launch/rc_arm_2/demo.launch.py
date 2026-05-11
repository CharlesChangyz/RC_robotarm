"""rc_arm_2 mock-hardware demo launch without the legacy planner stack."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("rc_arm_ruckig_config"),
                    "launch",
                    "rc_arm_2_robot.launch.py",
                ])
            ),
            launch_arguments={
                "hardware_config_file": PathJoinSubstitution([
                    FindPackageShare("rc_arm_description"),
                    "config",
                    "rc_arm_2",
                    "rc_arm_2_hardware.mujoco.yaml",
                ]),
                "use_rviz": "true",
            }.items(),
        )
    ])
