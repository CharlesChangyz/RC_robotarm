"""rc_arm_2 Servo-based demo launch (mock hardware)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_tf_target_bridge", default_value="true"),
        DeclareLaunchArgument("tf_target_topic", default_value="/tf"),
        DeclareLaunchArgument("tf_target_static_topic", default_value="/tf_static"),
        DeclareLaunchArgument("tf_target_parent_frame", default_value="world"),
        DeclareLaunchArgument("tf_target_child_frame", default_value="rc_arm_2_target"),
        DeclareLaunchArgument("tf_target_pose_topic", default_value="/rc_arm_2/target_pose"),
        DeclareLaunchArgument("middleware_motion_target_topic", default_value="/arm2/middleware/motion_target"),
        DeclareLaunchArgument("middleware_motion_result_topic", default_value="/arm2/middleware/motion_execution"),
        DeclareLaunchArgument("target_tracker_j4_axis", default_value="x"),
    ]

    include_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rc_arm_moveit_config"), "launch", "rc_arm_2", "robot.launch.py"]
            )
        ),
        launch_arguments={
            "use_mock_hardware": "true",
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_tf_target_bridge": LaunchConfiguration("use_tf_target_bridge"),
            "tf_target_topic": LaunchConfiguration("tf_target_topic"),
            "tf_target_static_topic": LaunchConfiguration("tf_target_static_topic"),
            "tf_target_parent_frame": LaunchConfiguration("tf_target_parent_frame"),
            "tf_target_child_frame": LaunchConfiguration("tf_target_child_frame"),
            "tf_target_pose_topic": LaunchConfiguration("tf_target_pose_topic"),
            "middleware_motion_target_topic": LaunchConfiguration("middleware_motion_target_topic"),
            "middleware_motion_result_topic": LaunchConfiguration("middleware_motion_result_topic"),
            "target_tracker_j4_axis": LaunchConfiguration("target_tracker_j4_axis"),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [include_robot])
