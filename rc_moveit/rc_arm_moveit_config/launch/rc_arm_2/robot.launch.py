"""rc_arm_2 Servo-based real-hardware launch."""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r", encoding="utf-8") as file:
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
        DeclareLaunchArgument("use_mock_hardware", default_value="false"),
        DeclareLaunchArgument("hardware_config_file", default_value=default_hardware_config),
        DeclareLaunchArgument("controllers_file", default_value=default_controllers_file),
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
    servo_yaml = load_yaml("rc_arm_moveit_config", "config/rc_arm_2/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

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

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            kinematics_yaml,
        ],
    )

    tf_target_bridge = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution(
                [FindPackageShare("rc_arm_moveit_config"), "launch", "tf_target_pose_bridge.py"]
            ),
            "--tf-topic",
            LaunchConfiguration("tf_target_topic"),
            "--tf-static-topic",
            LaunchConfiguration("tf_target_static_topic"),
            "--parent-frame",
            LaunchConfiguration("tf_target_parent_frame"),
            "--child-frame",
            LaunchConfiguration("tf_target_child_frame"),
            "--target-pose-topic",
            LaunchConfiguration("tf_target_pose_topic"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_tf_target_bridge")),
    )

    servo_target_adapter = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution(
                [FindPackageShare("rc_arm_moveit_config"), "launch", "servo_target_adapter.py"]
            ),
            "--target-topic",
            LaunchConfiguration("tf_target_pose_topic"),
            "--middleware-target-topic",
            LaunchConfiguration("middleware_motion_target_topic"),
            "--middleware-result-topic",
            LaunchConfiguration("middleware_motion_result_topic"),
            "--planning-frame",
            "base_link",
            "--j4-axis",
            LaunchConfiguration("target_tracker_j4_axis"),
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description, robot_description_semantic],
        condition=IfCondition(use_rviz),
    )

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delay_servo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[servo_node],
        )
    )

    delay_adapter = TimerAction(period=2.0, actions=[servo_target_adapter])
    delay_rviz = TimerAction(period=2.0, actions=[rviz_node], condition=IfCondition(use_rviz))

    return LaunchDescription(
        declared_arguments
        + [
            ros2_control_node,
            robot_state_publisher_node,
            static_tf_node,
            joint_state_broadcaster_spawner,
            delay_arm_controller,
            delay_servo,
            tf_target_bridge,
            delay_adapter,
            delay_rviz,
        ]
    )
