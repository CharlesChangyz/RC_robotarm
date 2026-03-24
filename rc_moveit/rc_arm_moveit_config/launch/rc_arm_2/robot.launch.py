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
    declared_arguments = [
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
            "can_enabled",
            default_value="true",
            description="Enable CAN communication in hardware interface",
        ),
        DeclareLaunchArgument(
            "external_feedback_enabled",
            default_value="false",
            description="Enable external JointState feedback (e.g. MuJoCo) when CAN is disabled",
        ),
        DeclareLaunchArgument(
            "external_feedback_topic",
            default_value="/rc_arm_2/mujoco_joint_states",
            description="External JointState feedback topic",
        ),
        DeclareLaunchArgument(
            "external_feedback_timeout",
            default_value="0.2",
            description="External feedback timeout in seconds",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Use mock hardware and disable CAN communication",
        ),
        DeclareLaunchArgument(
            "s_curve_enabled",
            default_value="true",
            description="Enable S-curve trajectory smoothing in hardware interface",
        ),
        DeclareLaunchArgument(
            "scalar_path_time_enabled",
            default_value="true",
            description="Enable common scalar path-time parameterization (q(s)+s(t))",
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
            "low_stiffness_mode",
            default_value="false",
            description="Enable low-stiffness position + model feedforward mode",
        ),
        DeclareLaunchArgument(
            "low_stiffness_kp",
            default_value="20.0",
            description="Low-stiffness mode Kp",
        ),
        DeclareLaunchArgument(
            "low_stiffness_kd",
            default_value="2.0",
            description="Low-stiffness mode Kd",
        ),
        DeclareLaunchArgument(
            "low_stiffness_kp_j1",
            default_value="0.0",
            description="Joint j1 low-stiffness Kp override (0 means using global low_stiffness_kp)",
        ),
        DeclareLaunchArgument(
            "low_stiffness_kd_j1",
            default_value="0.0",
            description="Joint j1 low-stiffness Kd override (0 means using global low_stiffness_kd)",
        ),
        DeclareLaunchArgument(
            "low_stiffness_kp_j2",
            default_value="0.0",
            description="Joint j2 low-stiffness Kp override (0 means using global low_stiffness_kp)",
        ),
        DeclareLaunchArgument(
            "low_stiffness_kd_j2",
            default_value="0.0",
            description="Joint j2 low-stiffness Kd override (0 means using global low_stiffness_kd)",
        ),
        DeclareLaunchArgument(
            "low_stiffness_kp_j3",
            default_value="0.0",
            description="Joint j3 low-stiffness Kp override (0 means using global low_stiffness_kp)",
        ),
        DeclareLaunchArgument(
            "low_stiffness_kd_j3",
            default_value="0.0",
            description="Joint j3 low-stiffness Kd override (0 means using global low_stiffness_kd)",
        ),
        DeclareLaunchArgument(
            "low_stiffness_kp_j4",
            default_value="0.0",
            description="Joint j4 low-stiffness Kp override (0 means using global low_stiffness_kp)",
        ),
        DeclareLaunchArgument(
            "low_stiffness_kd_j4",
            default_value="0.0",
            description="Joint j4 low-stiffness Kd override (0 means using global low_stiffness_kd)",
        ),
        DeclareLaunchArgument(
            "low_stiffness_torque_bias",
            default_value="0.0",
            description="Low-stiffness mode torque bias (Nm)",
        ),
        DeclareLaunchArgument(
            "use_pinocchio_gravity",
            default_value="true",
            description="Enable Pinocchio gravity torque",
        ),
        DeclareLaunchArgument(
            "gravity_feedforward_ratio",
            default_value="1.0",
            description="Gravity feedforward ratio (0-1)",
        ),
        DeclareLaunchArgument(
            "use_pinocchio_inverse_dynamics",
            default_value="true",
            description="Enable Pinocchio full inverse dynamics feedforward",
        ),
        DeclareLaunchArgument(
            "urdf_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("rc_arm_description"), "urdf", "rc_arm_2", "rc_arm_2.pinocchio.urdf"]
            ),
            description="URDF path used by Pinocchio model loader",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 with MoveIt plugin",
        ),
    ]

    can_interface = LaunchConfiguration("can_interface")
    host_can_id = LaunchConfiguration("host_can_id")
    can_enabled = LaunchConfiguration("can_enabled")
    external_feedback_enabled = LaunchConfiguration("external_feedback_enabled")
    external_feedback_topic = LaunchConfiguration("external_feedback_topic")
    external_feedback_timeout = LaunchConfiguration("external_feedback_timeout")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    s_curve_enabled = LaunchConfiguration("s_curve_enabled")
    scalar_path_time_enabled = LaunchConfiguration("scalar_path_time_enabled")
    smoothing_alpha = LaunchConfiguration("smoothing_alpha")
    max_velocity = LaunchConfiguration("max_velocity")
    max_acceleration = LaunchConfiguration("max_acceleration")
    max_jerk = LaunchConfiguration("max_jerk")
    low_stiffness_mode = LaunchConfiguration("low_stiffness_mode")
    low_stiffness_kp = LaunchConfiguration("low_stiffness_kp")
    low_stiffness_kd = LaunchConfiguration("low_stiffness_kd")
    low_stiffness_kp_j1 = LaunchConfiguration("low_stiffness_kp_j1")
    low_stiffness_kd_j1 = LaunchConfiguration("low_stiffness_kd_j1")
    low_stiffness_kp_j2 = LaunchConfiguration("low_stiffness_kp_j2")
    low_stiffness_kd_j2 = LaunchConfiguration("low_stiffness_kd_j2")
    low_stiffness_kp_j3 = LaunchConfiguration("low_stiffness_kp_j3")
    low_stiffness_kd_j3 = LaunchConfiguration("low_stiffness_kd_j3")
    low_stiffness_kp_j4 = LaunchConfiguration("low_stiffness_kp_j4")
    low_stiffness_kd_j4 = LaunchConfiguration("low_stiffness_kd_j4")
    low_stiffness_torque_bias = LaunchConfiguration("low_stiffness_torque_bias")
    use_pinocchio_gravity = LaunchConfiguration("use_pinocchio_gravity")
    gravity_feedforward_ratio = LaunchConfiguration("gravity_feedforward_ratio")
    use_pinocchio_inverse_dynamics = LaunchConfiguration("use_pinocchio_inverse_dynamics")
    urdf_path = LaunchConfiguration("urdf_path")
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
            " can_enabled:=",
            can_enabled,
            " external_feedback_enabled:=",
            external_feedback_enabled,
            " external_feedback_topic:=",
            external_feedback_topic,
            " external_feedback_timeout:=",
            external_feedback_timeout,
            " s_curve_enabled:=",
            s_curve_enabled,
            " scalar_path_time_enabled:=",
            scalar_path_time_enabled,
            " smoothing_alpha:=",
            smoothing_alpha,
            " max_velocity:=",
            max_velocity,
            " max_acceleration:=",
            max_acceleration,
            " max_jerk:=",
            max_jerk,
            " low_stiffness_mode:=",
            low_stiffness_mode,
            " low_stiffness_kp:=",
            low_stiffness_kp,
            " low_stiffness_kd:=",
            low_stiffness_kd,
            " low_stiffness_kp_j1:=",
            low_stiffness_kp_j1,
            " low_stiffness_kd_j1:=",
            low_stiffness_kd_j1,
            " low_stiffness_kp_j2:=",
            low_stiffness_kp_j2,
            " low_stiffness_kd_j2:=",
            low_stiffness_kd_j2,
            " low_stiffness_kp_j3:=",
            low_stiffness_kp_j3,
            " low_stiffness_kd_j3:=",
            low_stiffness_kd_j3,
            " low_stiffness_kp_j4:=",
            low_stiffness_kp_j4,
            " low_stiffness_kd_j4:=",
            low_stiffness_kd_j4,
            " low_stiffness_torque_bias:=",
            low_stiffness_torque_bias,
            " use_pinocchio_gravity:=",
            use_pinocchio_gravity,
            " gravity_feedforward_ratio:=",
            gravity_feedforward_ratio,
            " use_pinocchio_inverse_dynamics:=",
            use_pinocchio_inverse_dynamics,
            " urdf_path:=",
            urdf_path,
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

    ros2_controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("rc_arm_description"), "config", "rc_arm_2", "rc_arm_2_controllers.yaml"]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rc_arm_moveit_config"), "config", "moveit.rviz"]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_yaml],
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
