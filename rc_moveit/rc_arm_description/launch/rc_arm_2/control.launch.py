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
    declared_arguments = [
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Use mock hardware",
        ),
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="CAN interface name",
        ),
        DeclareLaunchArgument(
            "host_can_id",
            default_value="253",
            description="Host CAN ID",
        ),
        DeclareLaunchArgument(
            "s_curve_enabled",
            default_value="true",
            description="Enable S-curve trajectory smoothing in hardware interface",
        ),
        DeclareLaunchArgument(
            "smoothing_alpha",
            default_value="0.2",
            description="Smoothing alpha used by hardware interface",
        ),
        DeclareLaunchArgument(
            "max_velocity",
            default_value="2.0",
            description="S-curve max velocity (rad/s)",
        ),
        DeclareLaunchArgument(
            "max_acceleration",
            default_value="8.0",
            description="S-curve max acceleration (rad/s^2)",
        ),
        DeclareLaunchArgument(
            "max_jerk",
            default_value="50.0",
            description="S-curve max jerk (rad/s^3)",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2",
        ),
    ]

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    can_interface = LaunchConfiguration("can_interface")
    host_can_id = LaunchConfiguration("host_can_id")
    s_curve_enabled = LaunchConfiguration("s_curve_enabled")
    smoothing_alpha = LaunchConfiguration("smoothing_alpha")
    max_velocity = LaunchConfiguration("max_velocity")
    max_acceleration = LaunchConfiguration("max_acceleration")
    max_jerk = LaunchConfiguration("max_jerk")
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
            " can_interface:=",
            can_interface,
            " host_can_id:=",
            host_can_id,
            " s_curve_enabled:=",
            s_curve_enabled,
            " smoothing_alpha:=",
            smoothing_alpha,
            " max_velocity:=",
            max_velocity,
            " max_acceleration:=",
            max_acceleration,
            " max_jerk:=",
            max_jerk,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("rc_arm_description"), "config", "rc_arm_2", "rc_arm_2_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
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
