from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_hardware_config = PathJoinSubstitution([
        FindPackageShare("rc_arm_description"),
        "config",
        "rc_arm_2",
        "rc_arm_2_hardware.real.yaml",
    ])
    default_controllers_file = PathJoinSubstitution([
        FindPackageShare("rc_arm_description"),
        "config",
        "rc_arm_2",
        "rc_arm_2_controllers.yaml",
    ])

    declared_arguments = [
        DeclareLaunchArgument("hardware_config_file", default_value=default_hardware_config),
        DeclareLaunchArgument("controllers_file", default_value=default_controllers_file),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_tf_target_bridge", default_value="true"),
        DeclareLaunchArgument("use_target_pose_ruckig_executor", default_value="true"),
        DeclareLaunchArgument("trace_enabled", default_value="true"),
        DeclareLaunchArgument("trace_output_dir", default_value="/tmp/rc_arm_trace"),
        DeclareLaunchArgument("trace_event_topic", default_value="/rc_arm_2/trace_event"),
        DeclareLaunchArgument("trace_goal_context_topic", default_value="/rc_arm_2/trace_goal_context"),
        DeclareLaunchArgument("tf_target_topic", default_value="/tf"),
        DeclareLaunchArgument("tf_target_static_topic", default_value="/tf_static"),
        DeclareLaunchArgument("tf_target_parent_frame", default_value="world"),
        DeclareLaunchArgument("tf_target_child_frame", default_value="rc_arm_2_target"),
        DeclareLaunchArgument("tf_target_pose_topic", default_value="/rc_arm_2/target_pose"),
        DeclareLaunchArgument("target_pose_executor_joint_names", default_value="j1_joint,j2_joint,j3_joint,j4_joint"),
        DeclareLaunchArgument("target_pose_executor_default_frame", default_value="world"),
        DeclareLaunchArgument("target_pose_executor_pos_threshold", default_value="0.003"),
        DeclareLaunchArgument("target_pose_executor_rot_threshold", default_value="0.03"),
        DeclareLaunchArgument("target_pose_executor_velocity_scale", default_value="0.8"),
        DeclareLaunchArgument("target_pose_executor_acceleration_scale", default_value="0.8"),
        DeclareLaunchArgument("target_pose_executor_jerk_scale", default_value="0.8"),
        DeclareLaunchArgument("target_pose_executor_goal_tolerance", default_value="0.02"),
        DeclareLaunchArgument("target_pose_executor_control_period", default_value="0.02"),
        DeclareLaunchArgument("target_pose_executor_check_period", default_value="0.05"),
        DeclareLaunchArgument("target_pose_executor_j4_axis", default_value="x"),
        DeclareLaunchArgument("target_pose_executor_joint_state_topic", default_value="/joint_states"),
        DeclareLaunchArgument("target_pose_executor_urdf_path", default_value=""),
        DeclareLaunchArgument("target_pose_executor_status_log_period", default_value="1.0"),
        DeclareLaunchArgument("target_pose_executor_status_base_frame", default_value="world"),
        DeclareLaunchArgument("target_pose_executor_status_eef_frame", default_value="end_effector"),
        DeclareLaunchArgument("use_position_printer", default_value="false"),
        DeclareLaunchArgument("position_print_topic", default_value="/rc_arm_2/mujoco_joint_positions"),
        DeclareLaunchArgument("position_print_rate", default_value="10.0"),
        DeclareLaunchArgument("use_torque_printer", default_value="false"),
        DeclareLaunchArgument("torque_print_topic", default_value="/rc_arm_2/joint_torque"),
        DeclareLaunchArgument("torque_print_rate", default_value="10.0"),
    ]

    include_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("rc_arm_ruckig_config"), "launch", "rc_arm_2", "robot.launch.py"])
        ),
        launch_arguments={
            "hardware_config_file": LaunchConfiguration("hardware_config_file"),
            "controllers_file": LaunchConfiguration("controllers_file"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "trace_enabled": LaunchConfiguration("trace_enabled"),
            "trace_event_topic": LaunchConfiguration("trace_event_topic"),
            "trace_goal_context_topic": LaunchConfiguration("trace_goal_context_topic"),
        }.items(),
    )

    trace_logger = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([FindPackageShare("rc_arm_ruckig_config"), "launch", "target_trace_logger.py"]),
            "--enabled",
            LaunchConfiguration("trace_enabled"),
            "--trace-event-topic",
            LaunchConfiguration("trace_event_topic"),
            "--output-dir",
            LaunchConfiguration("trace_output_dir"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("trace_enabled")),
    )

    tf_target_bridge = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([FindPackageShare("rc_arm_ruckig_config"), "launch", "tf_target_pose_bridge.py"]),
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
            "--trace-event-topic",
            LaunchConfiguration("trace_event_topic"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_tf_target_bridge")),
    )

    target_pose_executor = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([FindPackageShare("rc_arm_ruckig_config"), "launch", "target_pose_ruckig_executor.py"]),
            "--target-topic",
            LaunchConfiguration("tf_target_pose_topic"),
            "--joint-names",
            LaunchConfiguration("target_pose_executor_joint_names"),
            "--default-frame",
            LaunchConfiguration("target_pose_executor_default_frame"),
            "--pos-threshold",
            LaunchConfiguration("target_pose_executor_pos_threshold"),
            "--rot-threshold",
            LaunchConfiguration("target_pose_executor_rot_threshold"),
            "--velocity-scale",
            LaunchConfiguration("target_pose_executor_velocity_scale"),
            "--acceleration-scale",
            LaunchConfiguration("target_pose_executor_acceleration_scale"),
            "--jerk-scale",
            LaunchConfiguration("target_pose_executor_jerk_scale"),
            "--goal-tolerance",
            LaunchConfiguration("target_pose_executor_goal_tolerance"),
            "--control-period",
            LaunchConfiguration("target_pose_executor_control_period"),
            "--check-period",
            LaunchConfiguration("target_pose_executor_check_period"),
            "--j4-axis",
            LaunchConfiguration("target_pose_executor_j4_axis"),
            "--joint-state-topic",
            LaunchConfiguration("target_pose_executor_joint_state_topic"),
            "--urdf-path",
            LaunchConfiguration("target_pose_executor_urdf_path"),
            "--status-log-period",
            LaunchConfiguration("target_pose_executor_status_log_period"),
            "--status-base-frame",
            LaunchConfiguration("target_pose_executor_status_base_frame"),
            "--status-eef-frame",
            LaunchConfiguration("target_pose_executor_status_eef_frame"),
            "--trace-event-topic",
            LaunchConfiguration("trace_event_topic"),
            "--trace-goal-context-topic",
            LaunchConfiguration("trace_goal_context_topic"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_target_pose_ruckig_executor")),
    )

    torque_printer = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([FindPackageShare("rc_arm_ruckig_config"), "launch", "joint_torque_printer.py"]),
            "--topic",
            LaunchConfiguration("torque_print_topic"),
            "--rate",
            LaunchConfiguration("torque_print_rate"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_torque_printer")),
    )

    position_printer = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([FindPackageShare("rc_arm_ruckig_config"), "launch", "joint_position_printer.py"]),
            "--topic",
            LaunchConfiguration("position_print_topic"),
            "--rate",
            LaunchConfiguration("position_print_rate"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_position_printer")),
    )

    return LaunchDescription(
        declared_arguments + [include_robot, trace_logger, tf_target_bridge, target_pose_executor, position_printer, torque_printer]
    )
