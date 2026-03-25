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
        DeclareLaunchArgument('low_stiffness_kp_j4', default_value='0.04', description='j4 低刚度模式位置增益覆盖（0 表示使用全局 low_stiffness_kp）'),
        DeclareLaunchArgument('low_stiffness_kd_j4', default_value='0.04', description='j4 低刚度模式阻尼增益覆盖（0 表示使用全局 low_stiffness_kd）'),
        DeclareLaunchArgument('low_stiffness_torque_bias', default_value='0.0', description='低刚度模式力矩偏置（Nm）'),
        DeclareLaunchArgument('use_pinocchio_gravity', default_value='false', description='是否启用 Pinocchio 重力补偿力矩'),
        DeclareLaunchArgument('gravity_feedforward_ratio', default_value='0.99', description='重力补偿前馈比例（0~1）'),
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
        DeclareLaunchArgument('use_mujoco_bridge', default_value='false', description='是否启动 MuJoCo 低刚度桥接'),
        DeclareLaunchArgument('use_tf_target_bridge', default_value='true', description='是否启动 TF->Pose 目标桥接（放在 rc_moveit 中订阅 TF）'),
        DeclareLaunchArgument('tf_target_topic', default_value='/tf', description='TF 动态变换话题'),
        DeclareLaunchArgument('tf_target_static_topic', default_value='/tf_static', description='TF 静态变换话题'),
        DeclareLaunchArgument('tf_target_parent_frame', default_value='world', description='目标 TF 的父坐标系'),
        DeclareLaunchArgument('tf_target_child_frame', default_value='rc_arm_2_target', description='目标 TF 的子坐标系（目标坐标来源）'),
        DeclareLaunchArgument('tf_target_pose_topic', default_value='/rc_arm_2/target_pose', description='TF 桥接输出的 Pose 话题'),
        DeclareLaunchArgument('mujoco_joint_command_topic', default_value='/debug/final_joint_command_joint_frame', description='MuJoCo 桥接订阅的最终关节控制包话题（position/velocity）'),
        DeclareLaunchArgument('mujoco_pd_gains_topic', default_value='/debug/final_pd_gains', description='MuJoCo 桥接订阅的最终 PD 参数话题（position=Kp, velocity=Kd）'),
        DeclareLaunchArgument('mujoco_torque_ff_topic', default_value='/debug/final_joint_torque_ff', description='MuJoCo 桥接订阅的最终前馈力矩话题（JointState.effort）'),
        DeclareLaunchArgument('mujoco_torque_input_topic', default_value='', description='MuJoCo 桥接订阅的外部关节力矩输入话题（可留空）'),
        DeclareLaunchArgument('mujoco_torque_output_topic', default_value='/rc_arm_2/joint_torque', description='MuJoCo 桥接发布的关节力矩话题'),
        DeclareLaunchArgument('mujoco_torque_input_scale', default_value='0.9', description='外部力矩输入缩放系数'),
        DeclareLaunchArgument('mujoco_kp_scale', default_value='1.0', description='MuJoCo 桥接中的 Kp 缩放系数'),
        DeclareLaunchArgument('mujoco_kd_scale', default_value='1.0', description='MuJoCo 桥接中的 Kd 缩放系数'),
        DeclareLaunchArgument('mujoco_ff_scale', default_value='1.0', description='MuJoCo 桥接中的前馈力矩缩放系数'),
        DeclareLaunchArgument('mujoco_torque_limit', default_value='14.0', description='MuJoCo 关节力矩限幅（Nm）'),
        DeclareLaunchArgument('mujoco_bridge_rate', default_value='500.0', description='MuJoCo 桥接循环频率（Hz）'),
        DeclareLaunchArgument('use_torque_printer', default_value='true', description='是否打印各关节力矩'),
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

    mujoco_bridge = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([
                FindPackageShare('rc_arm_moveit_config'),
                'launch',
                'mujoco_joint_state_bridge.py',
            ]),
            '--joint-command-topic',
            LaunchConfiguration('mujoco_joint_command_topic'),
            '--pd-gains-topic',
            LaunchConfiguration('mujoco_pd_gains_topic'),
            '--torque-ff-topic',
            LaunchConfiguration('mujoco_torque_ff_topic'),
            '--torque-input-topic',
            LaunchConfiguration('mujoco_torque_input_topic'),
            '--torque-output-topic',
            LaunchConfiguration('mujoco_torque_output_topic'),
            '--torque-input-scale',
            LaunchConfiguration('mujoco_torque_input_scale'),
            '--kp-scale',
            LaunchConfiguration('mujoco_kp_scale'),
            '--kd-scale',
            LaunchConfiguration('mujoco_kd_scale'),
            '--ff-scale',
            LaunchConfiguration('mujoco_ff_scale'),
            '--torque-limit',
            LaunchConfiguration('mujoco_torque_limit'),
            '--rate',
            LaunchConfiguration('mujoco_bridge_rate'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_mujoco_bridge')),
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

    return LaunchDescription(declared_arguments + [include_robot, tf_target_bridge, mujoco_bridge, torque_printer])
