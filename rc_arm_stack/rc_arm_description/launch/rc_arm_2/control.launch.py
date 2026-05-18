"""
rc_arm_2 control launch

Launches ros2_control + robot_state_publisher for rc_arm_2.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
            description="Use mock hardware",
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
            description="Start RViz2",
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
    robot_description = {"robot_description": robot_description_content}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(use_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    delay_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_state_pub_node,
            rviz_node,
            joint_state_broadcaster_spawner,
            delay_arm_controller_spawner,
        ]
    )
