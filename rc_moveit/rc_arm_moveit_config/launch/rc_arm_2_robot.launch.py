from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('can_interface', default_value='can0', description='CAN 通讯接口名称'),
        DeclareLaunchArgument('host_can_id', default_value='253', description='主机端 CAN ID'),
        DeclareLaunchArgument('can_enabled', default_value='false', description='是否启用 CAN 通信（false 时保留完整控制链但不发 CAN）'),
        DeclareLaunchArgument('external_feedback_enabled', default_value='true', description='CAN 关闭时是否启用外部 JointState 反馈（如 MuJoCo）'),
        DeclareLaunchArgument('external_feedback_topic', default_value='/rc_arm_2/mujoco_joint_positions', description='外部反馈 JointState 话题（建议接位置回传）'),
        DeclareLaunchArgument('external_feedback_timeout', default_value='0.2', description='外部反馈超时时间（秒）'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false', description='是否使用 mock 硬件（禁用 CAN 通信）'),
        DeclareLaunchArgument('s_curve_enabled', default_value='true', description='是否启用 S 曲线平滑'),
        DeclareLaunchArgument('scalar_path_time_enabled', default_value='true', description='是否启用公共标量路径参数化（q(s)+s(t)）'),
        DeclareLaunchArgument('smoothing_alpha', default_value='0.2', description='平滑滤波系数 alpha'),
        DeclareLaunchArgument('max_velocity', default_value='40.0', description='S 曲线最大速度（rad/s）'),
        DeclareLaunchArgument('max_acceleration', default_value='20.0', description='S 曲线最大加速度（rad/s^2）'),
        DeclareLaunchArgument('max_jerk', default_value='120.0', description='S 曲线最大加加速度（rad/s^3）'),
        DeclareLaunchArgument('low_stiffness_mode', default_value='true', description='是否启用低刚度位置+模型前馈模式'),
        DeclareLaunchArgument('low_stiffness_kp', default_value='6.00', description='低刚度模式位置增益 Kp'),
        DeclareLaunchArgument('low_stiffness_kd', default_value='2.0', description='低刚度模式阻尼增益 Kd'),
        DeclareLaunchArgument('low_stiffness_kp_j1', default_value='0.0', description='j1 低刚度模式位置增益覆盖（0 表示使用全局 low_stiffness_kp）'),
        DeclareLaunchArgument('low_stiffness_kd_j1', default_value='0.0', description='j1 低刚度模式阻尼增益覆盖（0 表示使用全局 low_stiffness_kd）'),
        DeclareLaunchArgument('low_stiffness_kp_j2', default_value='0.0', description='j2 低刚度模式位置增益覆盖（0 表示使用全局 low_stiffness_kp）'),
        DeclareLaunchArgument('low_stiffness_kd_j2', default_value='0.0', description='j2 低刚度模式阻尼增益覆盖（0 表示使用全局 low_stiffness_kd）'),
        DeclareLaunchArgument('low_stiffness_kp_j3', default_value='0.0', description='j3 低刚度模式位置增益覆盖（0 表示使用全局 low_stiffness_kp）'),
        DeclareLaunchArgument('low_stiffness_kd_j3', default_value='0.0', description='j3 低刚度模式阻尼增益覆盖（0 表示使用全局 low_stiffness_kd）'),
        DeclareLaunchArgument('low_stiffness_kp_j4', default_value='0.035', description='j4 低刚度模式位置增益覆盖（0 表示使用全局 low_stiffness_kp）'),
        DeclareLaunchArgument('low_stiffness_kd_j4', default_value='0.04', description='j4 低刚度模式阻尼增益覆盖（0 表示使用全局 low_stiffness_kd）'),
        DeclareLaunchArgument('low_stiffness_torque_bias', default_value='0.0', description='低刚度模式力矩偏置（Nm）'),
        DeclareLaunchArgument('use_pinocchio_gravity', default_value='false', description='是否启用 Pinocchio 重力补偿力矩'),
        DeclareLaunchArgument('gravity_feedforward_ratio', default_vlue='0.99', description='重力补偿前馈比例（0~1）'),
        DeclareLaunchArgument('use_pinocchio_inverse_dynamics', default_value='true', description='是否启用 Pinocchio 全逆动力学前馈'),
        DeclareLaunchArgument(
            'urdf_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('rc_arm_description'),
                'urdf',
                'rc_arm_2',
                'rc_arm_2.pinocchio.urdf'
            ]),
            description='Pinocchio 模型加载器使用的 URDF 路径'
        ),
        DeclareLaunchArgument('use_rviz', default_value='true', description='是否启动带 MoveIt 插件的 RViz2'),
        DeclareLaunchArgument('use_tf_target_bridge', default_value='true', description='是否启动 TF->Pose 目标桥接（放在 rc_moveit 中订阅 TF）'),
        DeclareLaunchArgument('tf_target_topic', default_value='/tf', description='TF 动态变换话题'),
        DeclareLaunchArgument('tf_target_static_topic', default_value='/tf_static', description='TF 静态变换话题'),
        DeclareLaunchArgument('tf_target_parent_frame', default_value='world', description='目标 TF 的父坐标系'),
        DeclareLaunchArgument('tf_target_child_frame', default_value='rc_arm_2_target', description='目标 TF 的子坐标系（目标坐标来源）'),
        DeclareLaunchArgument('tf_target_pose_topic', default_value='/rc_arm_2/target_pose', description='TF 桥接输出的 Pose 话题'),
        DeclareLaunchArgument('use_target_pose_moveit_executor', default_value='true', description='是否启用 target_pose -> MoveIt 规划执行链路'),
        DeclareLaunchArgument('target_pose_executor_group', default_value='arm', description='target_pose 执行器的 MoveIt 规划组'),
        DeclareLaunchArgument('target_pose_executor_joint_names', default_value='j1_joint,j2_joint,j3_joint,j4_joint', description='target_pose 执行器使用的关节顺序'),
        DeclareLaunchArgument('target_pose_executor_default_frame', default_value='world', description='target_pose 没有 frame_id 时使用的默认坐标系'),
        DeclareLaunchArgument('target_pose_executor_pos_threshold', default_value='0.003', description='新目标触发阈值：位置变化（m）'),
        DeclareLaunchArgument('target_pose_executor_rot_threshold', default_value='0.03', description='新目标触发阈值：旋转变化（rad）'),
        DeclareLaunchArgument('target_pose_executor_planning_time', default_value='2.0', description='MoveIt 单次规划时间（s）'),
        DeclareLaunchArgument('target_pose_executor_planning_attempts', default_value='5', description='MoveIt 单次规划尝试次数'),
        DeclareLaunchArgument('target_pose_executor_vel_scale', default_value='0.5', description='MoveIt 速度缩放（0~1）'),
        DeclareLaunchArgument('target_pose_executor_acc_scale', default_value='0.5', description='MoveIt 加速度缩放（0~1）'),
        DeclareLaunchArgument('target_pose_executor_joint_tolerance', default_value='0.02', description='MoveIt 关节目标容差（rad）'),
        DeclareLaunchArgument('target_pose_executor_check_period', default_value='0.05', description='target_pose 执行器轮询周期（s）'),
        DeclareLaunchArgument('target_pose_executor_enforce_j4_from_target', default_value='true', description='位置优先 IK 下是否从目标姿态提取并强制写回 j4'),
        DeclareLaunchArgument('target_pose_executor_j4_joint_name', default_value='j4_joint', description='j4 关节名称（用于强制写回）'),
        DeclareLaunchArgument('target_pose_executor_j4_axis', default_value='x', description='目标姿态中 j4 对应的旋转轴（x/y/z）'),
        DeclareLaunchArgument('target_pose_executor_status_log_period', default_value='1.0', description='target_pose 执行器状态心跳打印周期（秒，<=0 关闭）'),
        DeclareLaunchArgument('target_pose_executor_status_base_frame', default_value='world', description='状态打印使用的基坐标系'),
        DeclareLaunchArgument('target_pose_executor_status_eef_frame', default_value='end_effector', description='状态打印使用的末端坐标系'),
        DeclareLaunchArgument('use_torque_printer', default_value='false', description='是否打印各关节力矩'),
        DeclareLaunchArgument('torque_print_topic', default_value='/rc_arm_2/joint_torque', description='力矩打印订阅话题（JointState.effort）'),
        DeclareLaunchArgument('torque_print_rate', default_value='10.0', description='力矩打印频率（Hz）'),
    ]

    include_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rc_arm_moveit_config'),
                'launch',
                'rc_arm_2',
                'robot.launch.py'
            ])
        ),
        launch_arguments={
            'can_interface': LaunchConfiguration('can_interface'),
            'host_can_id': LaunchConfiguration('host_can_id'),
            'can_enabled': LaunchConfiguration('can_enabled'),
            'external_feedback_enabled': LaunchConfiguration('external_feedback_enabled'),
            'external_feedback_topic': LaunchConfiguration('external_feedback_topic'),
            'external_feedback_timeout': LaunchConfiguration('external_feedback_timeout'),
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            's_curve_enabled': LaunchConfiguration('s_curve_enabled'),
            'scalar_path_time_enabled': LaunchConfiguration('scalar_path_time_enabled'),
            'smoothing_alpha': LaunchConfiguration('smoothing_alpha'),
            'max_velocity': LaunchConfiguration('max_velocity'),
            'max_acceleration': LaunchConfiguration('max_acceleration'),
            'max_jerk': LaunchConfiguration('max_jerk'),
            'low_stiffness_mode': LaunchConfiguration('low_stiffness_mode'),
            'low_stiffness_kp': LaunchConfiguration('low_stiffness_kp'),
            'low_stiffness_kd': LaunchConfiguration('low_stiffness_kd'),
            'low_stiffness_kp_j1': LaunchConfiguration('low_stiffness_kp_j1'),
            'low_stiffness_kd_j1': LaunchConfiguration('low_stiffness_kd_j1'),
            'low_stiffness_kp_j2': LaunchConfiguration('low_stiffness_kp_j2'),
            'low_stiffness_kd_j2': LaunchConfiguration('low_stiffness_kd_j2'),
            'low_stiffness_kp_j3': LaunchConfiguration('low_stiffness_kp_j3'),
            'low_stiffness_kd_j3': LaunchConfiguration('low_stiffness_kd_j3'),
            'low_stiffness_kp_j4': LaunchConfiguration('low_stiffness_kp_j4'),
            'low_stiffness_kd_j4': LaunchConfiguration('low_stiffness_kd_j4'),
            'low_stiffness_torque_bias': LaunchConfiguration('low_stiffness_torque_bias'),
            'use_pinocchio_gravity': LaunchConfiguration('use_pinocchio_gravity'),
            'gravity_feedforward_ratio': LaunchConfiguration('gravity_feedforward_ratio'),
            'use_pinocchio_inverse_dynamics': LaunchConfiguration('use_pinocchio_inverse_dynamics'),
            'urdf_path': LaunchConfiguration('urdf_path'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items()
    )


    tf_target_bridge = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([
                FindPackageShare('rc_arm_moveit_config'),
                'launch',
                'tf_target_pose_bridge.py',
            ]),
            '--tf-topic',
            LaunchConfiguration('tf_target_topic'),
            '--tf-static-topic',
            LaunchConfiguration('tf_target_static_topic'),
            '--parent-frame',
            LaunchConfiguration('tf_target_parent_frame'),
            '--child-frame',
            LaunchConfiguration('tf_target_child_frame'),
            '--target-pose-topic',
            LaunchConfiguration('tf_target_pose_topic'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_tf_target_bridge')),
    )

    target_pose_executor = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([
                FindPackageShare('rc_arm_moveit_config'),
                'launch',
                'target_pose_moveit_executor.py',
            ]),
            '--target-topic',
            LaunchConfiguration('tf_target_pose_topic'),
            '--planning-group',
            LaunchConfiguration('target_pose_executor_group'),
            '--joint-names',
            LaunchConfiguration('target_pose_executor_joint_names'),
            '--default-frame',
            LaunchConfiguration('target_pose_executor_default_frame'),
            '--pos-threshold',
            LaunchConfiguration('target_pose_executor_pos_threshold'),
            '--rot-threshold',
            LaunchConfiguration('target_pose_executor_rot_threshold'),
            '--planning-time',
            LaunchConfiguration('target_pose_executor_planning_time'),
            '--planning-attempts',
            LaunchConfiguration('target_pose_executor_planning_attempts'),
            '--vel-scale',
            LaunchConfiguration('target_pose_executor_vel_scale'),
            '--acc-scale',
            LaunchConfiguration('target_pose_executor_acc_scale'),
            '--joint-tolerance',
            LaunchConfiguration('target_pose_executor_joint_tolerance'),
            '--check-period',
            LaunchConfiguration('target_pose_executor_check_period'),
            '--enforce-j4-from-target',
            LaunchConfiguration('target_pose_executor_enforce_j4_from_target'),
            '--j4-joint-name',
            LaunchConfiguration('target_pose_executor_j4_joint_name'),
            '--j4-axis',
            LaunchConfiguration('target_pose_executor_j4_axis'),
            '--status-log-period',
            LaunchConfiguration('target_pose_executor_status_log_period'),
            '--status-base-frame',
            LaunchConfiguration('target_pose_executor_status_base_frame'),
            '--status-eef-frame',
            LaunchConfiguration('target_pose_executor_status_eef_frame'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_target_pose_moveit_executor')),
    )

    torque_printer = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([
                FindPackageShare('rc_arm_moveit_config'),
                'launch',
                'joint_torque_printer.py',
            ]),
            '--topic',
            LaunchConfiguration('torque_print_topic'),
            '--rate',
            LaunchConfiguration('torque_print_rate'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_torque_printer')),
    )

    return LaunchDescription(declared_arguments + [include_robot, tf_target_bridge, target_pose_executor, torque_printer])
