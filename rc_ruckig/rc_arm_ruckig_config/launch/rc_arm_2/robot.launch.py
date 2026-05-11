"""rc_arm_2 hardware/control launch without the legacy planner stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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
        DeclareLaunchArgument("trace_enabled", default_value="true"),
        DeclareLaunchArgument("trace_event_topic", default_value="/rc_arm_2/trace_event"),
        DeclareLaunchArgument("trace_goal_context_topic", default_value="/rc_arm_2/trace_goal_context"),
    ]

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    hardware_config_file = LaunchConfiguration("hardware_config_file")
    controllers_file = LaunchConfiguration("controllers_file")
    use_rviz = LaunchConfiguration("use_rviz")
    trace_enabled = LaunchConfiguration("trace_enabled")
    trace_event_topic = LaunchConfiguration("trace_event_topic")
    trace_goal_context_topic = LaunchConfiguration("trace_goal_context_topic")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("rc_arm_description"), "urdf", "rc_arm_2", "rc_arm_2.urdf.xacro"]),
            " use_mock_hardware:=",
            use_mock_hardware,
            " hardware_config_file:=",
            hardware_config_file,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_file,
            {
                "arm_controller": {
                    "ros__parameters": {
                        "trace_enabled": ParameterValue(trace_enabled, value_type=bool),
                        "trace_event_topic": ParameterValue(trace_event_topic, value_type=str),
                        "trace_goal_context_topic": ParameterValue(trace_goal_context_topic, value_type=str),
                    }
                }
            },
        ],
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
        arguments=["joint_state_broadcaster", "-c", "/controller_manager", "--controller-manager-timeout", "120"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager", "--controller-manager-timeout", "120"],
    )

    ruckig_server_node = Node(
        package="rc_arm_ruckig_config",
        executable="ruckig_trajectory_server",
        name="rc_arm_ruckig_trajectory_server",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[robot_description],
        condition=IfCondition(use_rviz),
    )

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
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
            ruckig_server_node,
            rviz_node,
        ]
    )
