"""
rc_arm_2 MoveIt demo launch (mock hardware)
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz2 with MoveIt plugin",
    )
    use_rviz = LaunchConfiguration("use_rviz")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rc_arm_description"), "urdf", "rc_arm_2", "rc_arm_2.urdf.xacro"]
            ),
            " use_mock_hardware:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_content = Command(
        [
            "cat ",
            PathJoinSubstitution(
                [FindPackageShare("rc_arm_moveit_config"), "config", "rc_arm_2", "rc_arm_2.srdf"]
            ),
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    kinematics_yaml = load_yaml("rc_arm_moveit_config", "config/rc_arm_2/kinematics.yaml")
    joint_limits_yaml = load_yaml("rc_arm_moveit_config", "config/rc_arm_2/joint_limits.yaml")
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}

    ompl_planning_yaml = load_yaml("rc_arm_moveit_config", "config/rc_arm_2/ompl_planning.yaml")
    ompl_planning_pipeline_config = {"move_group": ompl_planning_yaml}

    moveit_controllers_yaml = load_yaml("rc_arm_moveit_config", "config/rc_arm_2/moveit_controllers.yaml")

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    ros2_controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("rc_arm_description"), "config", "rc_arm_2", "rc_arm_2_controllers.yaml"]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rc_arm_moveit_config"), "config", "moveit.rviz"]
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers_yaml,
            planning_scene_monitor_parameters,
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_yaml],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            kinematics_yaml,
        ],
        condition=IfCondition(use_rviz),
    )

    delay_rviz = TimerAction(
        period=3.0,
        actions=[rviz_node],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        use_rviz_arg,
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
        delay_rviz,
    ])
