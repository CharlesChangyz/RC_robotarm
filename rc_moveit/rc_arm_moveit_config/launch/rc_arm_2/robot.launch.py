"""
rc_arm_2 MoveIt real-hardware launch
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
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
    default_hardware_config = PathJoinSubstitution(
        [FindPackageShare("rc_arm_description"), "config", "rc_arm_2", "rc_arm_2_hardware.real.yaml"]
    )
    default_controllers_file = PathJoinSubstitution(
        [FindPackageShare("rc_arm_description"), "config", "rc_arm_2", "rc_arm_2_controllers.yaml"]
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Use mock hardware and disable CAN communication",
        ),
        DeclareLaunchArgument(
            "hardware_config_file",
            default_value=default_hardware_config,
            description="Hardware plugin configuration YAML",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=default_controllers_file,
            description="ros2_control controllers YAML",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 with MoveIt plugin",
        ),
    ]

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    hardware_config_file = LaunchConfiguration("hardware_config_file")
    controllers_file = LaunchConfiguration("controllers_file")
    use_rviz = LaunchConfiguration("use_rviz")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rc_arm_description"), "urdf", "rc_arm_2", "rc_arm_2.urdf.xacro"]
            ),
            " use_mock_hardware:=",
            use_mock_hardware,
            " hardware_config_file:=",
            hardware_config_file,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_description_semantic_content = Command(
        [
            "cat ",
            PathJoinSubstitution(
                [FindPackageShare("rc_arm_moveit_config"), "config", "rc_arm_2", "rc_arm_2.srdf"]
            ),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }

    kinematics_yaml = load_yaml("rc_arm_moveit_config", "config/rc_arm_2/kinematics.yaml")
    joint_limits_yaml = load_yaml("rc_arm_moveit_config", "config/rc_arm_2/joint_limits.yaml")
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}

    ompl_planning_yaml = load_yaml("rc_arm_moveit_config", "config/rc_arm_2/ompl_planning.yaml")
    ompl_planning_pipeline_config = {"move_group": ompl_planning_yaml}

    moveit_controllers_yaml = load_yaml("rc_arm_moveit_config", "config/rc_arm_2/moveit_controllers.yaml")

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 4.0,
        "trajectory_execution.allowed_goal_duration_margin": 2.0,
        "trajectory_execution.allowed_start_tolerance": 0.1,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rc_arm_moveit_config"), "config", "moveit.rviz"]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        output="both",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="log",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
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

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delay_move_group = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[move_group_node],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            ros2_control_node,
            robot_state_publisher_node,
            static_tf_node,
            joint_state_broadcaster_spawner,
            delay_arm_controller,
            delay_move_group,
            rviz_node,
        ]
    )
