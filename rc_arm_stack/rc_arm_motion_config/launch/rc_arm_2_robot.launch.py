from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_hardware_config = PathJoinSubstitution(
        [FindPackageShare("rc_arm_description"), "config", "rc_arm_2", "rc_arm_2_hardware.real.yaml"]
    )
    default_controllers_file = PathJoinSubstitution(
        [FindPackageShare("rc_arm_description"), "config", "rc_arm_2", "rc_arm_2_controllers.yaml"]
    )
    default_joint_limits_file = PathJoinSubstitution(
        [FindPackageShare("rc_arm_motion_config"), "config", "rc_arm_2", "ruckig_joint_limits.yaml"]
    )

    declared_arguments = [
        DeclareLaunchArgument("hardware_config_file", default_value=default_hardware_config, description="硬件插件配置 YAML"),
        DeclareLaunchArgument("controllers_file", default_value=default_controllers_file, description="ros2_control 控制器配置 YAML"),
        DeclareLaunchArgument("joint_limits_file", default_value=default_joint_limits_file, description="Ruckig 关节限制 YAML"),
        DeclareLaunchArgument("use_rviz", default_value="true", description="是否启动 RViz2"),
        DeclareLaunchArgument("use_world_base_static_tf", default_value="true", description="是否发布 world->base_link 静态 TF"),
        DeclareLaunchArgument("world_frame_id", default_value="world", description="全局坐标系名称"),
        DeclareLaunchArgument("base_frame_id", default_value="base_link", description="机械臂基坐标系名称"),
        DeclareLaunchArgument("use_tf_target_bridge", default_value="true", description="是否启动 TF->Pose 目标桥接"),
        DeclareLaunchArgument("tf_target_topic", default_value="/tf", description="TF 动态变换话题"),
        DeclareLaunchArgument("tf_target_static_topic", default_value="/tf_static", description="TF 静态变换话题"),
        DeclareLaunchArgument("tf_target_parent_frame", default_value=LaunchConfiguration("world_frame_id"), description="目标 TF 的父坐标系"),
        DeclareLaunchArgument("tf_target_child_frame", default_value="rc_arm_2_target", description="目标 TF 的子坐标系"),
        DeclareLaunchArgument("tf_target_pose_topic", default_value="/rc_arm_2/target_pose", description="TF 桥接输出的 Pose 话题"),
        DeclareLaunchArgument("middleware_motion_target_topic", default_value="/arm2/middleware/motion_target", description="middleware 发送笛卡尔目标点的话题"),
        DeclareLaunchArgument("middleware_motion_result_topic", default_value="/arm2/middleware/motion_execution", description="executor 返回 middleware 单次运动结果的话题"),
        DeclareLaunchArgument("use_target_pose_executor", default_value="true", description="是否启用 target_pose 执行链路"),
        DeclareLaunchArgument("target_pose_executor_joint_names", default_value="j1_joint,j2_joint,j3_joint,j4_joint", description="target_pose 执行器使用的关节顺序"),
        DeclareLaunchArgument("target_pose_executor_default_frame", default_value=LaunchConfiguration("world_frame_id"), description="target_pose 没有 frame_id 时使用的默认坐标系"),
        DeclareLaunchArgument("target_pose_executor_pos_threshold", default_value="0.003", description="在线追踪位置容差/去抖阈值（m）"),
        DeclareLaunchArgument("target_pose_executor_rot_threshold", default_value="0.03", description="在线追踪姿态容差/去抖阈值（rad）"),
        DeclareLaunchArgument("target_pose_executor_check_period", default_value="0.002", description="target_pose 在线跟踪周期（s）"),
        DeclareLaunchArgument("target_pose_executor_j4_axis", default_value="x", description="目标姿态中 j4 对应的旋转轴（x/y/z）"),
        DeclareLaunchArgument("target_pose_executor_joint_state_topic", default_value="/joint_states", description="target_pose 执行器读取当前关节角的话题"),
        DeclareLaunchArgument("target_pose_executor_urdf_path", default_value="", description="target_pose 执行器共享 4DOF 几何 helper 的 URDF/Xacro 路径，留空则自动查找"),
        DeclareLaunchArgument("target_pose_executor_status_log_period", default_value="1.0", description="target_pose 执行器状态心跳打印周期（秒，<=0 关闭）"),
        DeclareLaunchArgument("target_pose_executor_status_base_frame", default_value=LaunchConfiguration("world_frame_id"), description="状态打印使用的基坐标系"),
        DeclareLaunchArgument("target_pose_executor_status_eef_frame", default_value="end_effector", description="状态打印使用的末端坐标系"),
        DeclareLaunchArgument("target_pose_executor_trajectory_sampling_period", default_value="0.002", description="Ruckig 轨迹采样周期（秒）"),
<<<<<<< HEAD
        DeclareLaunchArgument("target_pose_executor_action_name", default_value="/arm_controller/follow_joint_trajectory", description="FollowJointTrajectory action 名"),
=======
        DeclareLaunchArgument("target_pose_executor_trajectory_topic", default_value="/arm_controller/joint_trajectory", description="在线跟踪 joint trajectory topic"),
>>>>>>> bf3c100 (修改流式传输)
        DeclareLaunchArgument("use_position_printer", default_value="false", description="是否打印各关节当前角度"),
        DeclareLaunchArgument("position_print_topic", default_value="/rc_arm_2/mujoco_joint_positions", description="角度打印订阅话题（JointState.position）"),
        DeclareLaunchArgument("position_print_rate", default_value="10.0", description="角度打印频率（Hz）"),
        DeclareLaunchArgument("use_torque_printer", default_value="false", description="是否打印各关节力矩"),
        DeclareLaunchArgument("torque_print_topic", default_value="/rc_arm_2/joint_torque", description="力矩打印订阅话题（JointState.effort）"),
        DeclareLaunchArgument("torque_print_rate", default_value="10.0", description="力矩打印频率（Hz）"),
    ]

    include_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rc_arm_description"), "launch", "rc_arm_2", "control.launch.py"]
            )
        ),
        launch_arguments={
            "hardware_config_file": LaunchConfiguration("hardware_config_file"),
            "controllers_file": LaunchConfiguration("controllers_file"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "rviz_fixed_frame": LaunchConfiguration("world_frame_id"),
        }.items(),
    )

    world_base_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_link_static_tf",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            LaunchConfiguration("world_frame_id"),
            "--child-frame-id",
            LaunchConfiguration("base_frame_id"),
        ],
        condition=IfCondition(LaunchConfiguration("use_world_base_static_tf")),
        output="screen",
    )

    tf_target_bridge = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([FindPackageShare("rc_arm_motion_config"), "launch", "tf_target_pose_bridge.py"]),
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

    target_pose_executor = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([FindPackageShare("rc_arm_motion_config"), "launch", "target_pose_ruckig_executor.py"]),
            "--target-topic",
            LaunchConfiguration("tf_target_pose_topic"),
            "--middleware-target-topic",
            LaunchConfiguration("middleware_motion_target_topic"),
            "--middleware-result-topic",
            LaunchConfiguration("middleware_motion_result_topic"),
            "--joint-names",
            LaunchConfiguration("target_pose_executor_joint_names"),
            "--default-frame",
            LaunchConfiguration("target_pose_executor_default_frame"),
            "--trajectory-topic",
            LaunchConfiguration("target_pose_executor_trajectory_topic"),
            "--joint-limits-file",
            LaunchConfiguration("joint_limits_file"),
            "--trajectory-sampling-period",
            LaunchConfiguration("target_pose_executor_trajectory_sampling_period"),
            "--joint-state-topic",
            LaunchConfiguration("target_pose_executor_joint_state_topic"),
            "--urdf-path",
            LaunchConfiguration("target_pose_executor_urdf_path"),
            "--pos-threshold",
            LaunchConfiguration("target_pose_executor_pos_threshold"),
            "--rot-threshold",
            LaunchConfiguration("target_pose_executor_rot_threshold"),
            "--check-period",
            LaunchConfiguration("target_pose_executor_check_period"),
            "--j4-axis",
            LaunchConfiguration("target_pose_executor_j4_axis"),
            "--status-log-period",
            LaunchConfiguration("target_pose_executor_status_log_period"),
            "--status-base-frame",
            LaunchConfiguration("target_pose_executor_status_base_frame"),
            "--status-eef-frame",
            LaunchConfiguration("target_pose_executor_status_eef_frame"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_target_pose_executor")),
    )

    torque_printer = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([FindPackageShare("rc_arm_motion_config"), "launch", "joint_torque_printer.py"]),
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
            PathJoinSubstitution([FindPackageShare("rc_arm_motion_config"), "launch", "joint_position_printer.py"]),
            "--topic",
            LaunchConfiguration("position_print_topic"),
            "--rate",
            LaunchConfiguration("position_print_rate"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_position_printer")),
    )

    return LaunchDescription(
        declared_arguments
        + [
            include_control,
            world_base_static_tf,
            tf_target_bridge,
            target_pose_executor,
            position_printer,
            torque_printer,
        ]
    )
