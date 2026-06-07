from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_hardware_config = PathJoinSubstitution([
        FindPackageShare('rc_arm_description'),
        'config',
        'rc_arm_2',
        'rc_arm_2_hardware.real.yaml',
    ])
    default_controllers_file = PathJoinSubstitution([
        FindPackageShare('rc_arm_description'),
        'config',
        'rc_arm_2',
        'rc_arm_2_controllers.yaml',
    ])
    default_action_sets_file = PathJoinSubstitution([
        FindPackageShare('rc_arm2_middleware'),
        'config',
        'action_sets.yaml',
    ])

    declared_arguments = [
        DeclareLaunchArgument(
            'hardware_config_file',
            default_value=default_hardware_config,
            description='硬件插件配置 YAML',
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=default_controllers_file,
            description='ros2_control 控制器配置 YAML',
        ),
        DeclareLaunchArgument('use_rviz', default_value='true', description='是否启动带 MoveIt 插件的 RViz2'),
        DeclareLaunchArgument('use_tf_target_bridge', default_value='true', description='是否启动 TF->Pose 目标桥接（放在 rc_moveit 中订阅 TF）'),
        DeclareLaunchArgument('use_dm_serial_frame_bridge', default_value='false', description='是否启动 dmserial 原始帧 topic 桥'),
        DeclareLaunchArgument('dm_serial_bridge_sn', default_value='9940F4E149D904A69924737E3DE6629F', description='USB2CANFD 设备序列号'),
        DeclareLaunchArgument('dm_serial_bridge_nom_baud', default_value='1000000', description='dmserial 仲裁域波特率'),
        DeclareLaunchArgument('dm_serial_bridge_dat_baud', default_value='2000000', description='dmserial 数据域波特率'),
        DeclareLaunchArgument('dm_serial_bridge_rx_topic', default_value='/rc_arm_2/dm_serial_rx', description='dmserial 原始帧接收 topic'),
        DeclareLaunchArgument('dm_serial_bridge_tx_topic', default_value='/rc_arm_2/dm_serial_tx', description='dmserial 原始帧发送 topic'),
        DeclareLaunchArgument('tf_target_topic', default_value='/tf', description='TF 动态变换话题'),
        DeclareLaunchArgument('tf_target_static_topic', default_value='/tf_static', description='TF 静态变换话题'),
        DeclareLaunchArgument('tf_target_parent_frame', default_value='world', description='目标 TF 的父坐标系'),
        DeclareLaunchArgument('tf_target_child_frame', default_value='rc_arm_2_target', description='目标 TF 的子坐标系（目标坐标来源）'),
        DeclareLaunchArgument('tf_target_pose_topic', default_value='/rc_arm_2/target_pose', description='TF 桥接输出的 Pose 话题'),
        DeclareLaunchArgument('use_arm2_middleware', default_value='true', description='是否启动 arm2 middleware 动作集执行节点'),
        DeclareLaunchArgument('middleware_action_sets_file', default_value=default_action_sets_file, description='arm2 middleware 动作集 YAML'),
        DeclareLaunchArgument('middleware_target_point_topic', default_value='/arm2/middleware/target_point', description='middleware 缓存目标点的话题'),
        DeclareLaunchArgument('middleware_run_action_set_topic', default_value='/arm2/middleware/run_action_set', description='middleware 执行动作集的话题'),
        DeclareLaunchArgument('middleware_motion_target_topic', default_value='/arm2/middleware/motion_target', description='middleware 发送笛卡尔目标点的话题'),
        DeclareLaunchArgument('middleware_motion_result_topic', default_value='/arm2/middleware/motion_execution', description='executor 返回 middleware 单次运动结果的话题'),
        DeclareLaunchArgument('middleware_dm_serial_bridge_enabled', default_value='true', description='是否启用 middleware 的 dmserial 动作触发桥'),
        DeclareLaunchArgument('middleware_dm_serial_rx_topic', default_value='/rc_arm_2/dm_serial_rx', description='hardware 发布的 dmserial 原始接收帧 topic'),
        DeclareLaunchArgument('middleware_dm_serial_tx_topic', default_value='/rc_arm_2/dm_serial_tx', description='middleware 回传 dmserial 原始帧 topic'),
        DeclareLaunchArgument('middleware_dm_serial_command_base_id', default_value='1024', description='dmserial 命令基 ID；默认 0x400，因此 0x4xx -> action set xx'),
        DeclareLaunchArgument('middleware_dm_serial_complete_id', default_value='1280', description='动作成功完成后的 dmserial 回传 ID；默认 0x500'),
        DeclareLaunchArgument('middleware_dm_serial_allowed_action_set_ids', default_value='', description='允许 dmserial 触发的 action set ID，逗号分隔；留空表示允许所有 action set'),
        DeclareLaunchArgument('use_target_pose_moveit_executor', default_value='true', description='是否启用 target_pose -> MoveIt 规划执行链路'),
        DeclareLaunchArgument('target_pose_executor_group', default_value='arm', description='target_pose 执行器的 MoveIt 规划组'),
        DeclareLaunchArgument('target_pose_executor_joint_names', default_value='j1_joint,j2_joint,j3_joint,j4_joint', description='target_pose 执行器使用的关节顺序'),
        DeclareLaunchArgument('target_pose_executor_default_frame', default_value='world', description='target_pose 没有 frame_id 时使用的默认坐标系'),
        DeclareLaunchArgument('target_pose_executor_pos_threshold', default_value='0.003', description='新目标触发阈值：位置变化（m）'),
        DeclareLaunchArgument('target_pose_executor_rot_threshold', default_value='0.03', description='新目标触发阈值：旋转变化（rad）'),
        DeclareLaunchArgument('target_pose_executor_planning_time', default_value='1.0', description='MoveIt 单次规划时间（s）'),
        DeclareLaunchArgument('target_pose_executor_planning_attempts', default_value='3', description='MoveIt 单次规划尝试次数'),
        DeclareLaunchArgument('target_pose_executor_vel_scale', default_value='0.8', description='MoveIt 速度缩放（0~1）'),
        DeclareLaunchArgument('target_pose_executor_acc_scale', default_value='0.8', description='MoveIt 加速度缩放（0~1）'),
        DeclareLaunchArgument('target_pose_executor_joint_tolerance', default_value='0.02', description='MoveIt 关节目标容差（rad）'),
        DeclareLaunchArgument('target_pose_executor_check_period', default_value='0.05', description='target_pose 执行器轮询周期（s）'),
        DeclareLaunchArgument('target_pose_executor_middleware_preempt_interval_sec', default_value='0.25', description='middleware 目标最小抢断间隔（s）'),
        DeclareLaunchArgument('target_pose_executor_middleware_preempt_pos_threshold', default_value='0.01', description='middleware 目标位置变化小于此值时不抢断（m）'),
        DeclareLaunchArgument('target_pose_executor_middleware_preempt_rot_threshold', default_value='0.08', description='middleware 目标姿态变化小于此值时不抢断（rad）'),
        DeclareLaunchArgument('target_pose_executor_j4_axis', default_value='x', description='目标姿态中 j4 对应的旋转轴（x/y/z）'),
        DeclareLaunchArgument('target_pose_executor_joint_state_topic', default_value='/joint_states', description='target_pose 执行器读取当前关节角用于分支选择的话题'),
        DeclareLaunchArgument('target_pose_executor_urdf_path', default_value='', description='target_pose 执行器共享 4DOF 几何 helper 的 URDF/Xacro 路径，留空则自动查找'),
        DeclareLaunchArgument('target_pose_executor_status_log_period', default_value='1.0', description='target_pose 执行器状态心跳打印周期（秒，<=0 关闭）'),
        DeclareLaunchArgument('target_pose_executor_status_base_frame', default_value='world', description='状态打印使用的基坐标系'),
        DeclareLaunchArgument('target_pose_executor_status_eef_frame', default_value='end_effector', description='状态打印使用的末端坐标系'),
        DeclareLaunchArgument('target_pose_executor_avoid_collisions_enabled', default_value='true', description='target_pose 执行器是否启用 MoveIt 避障'),
        DeclareLaunchArgument('target_pose_executor_world_boxes_json', default_value='[]', description='启动时注入的世界障碍物 box JSON 数组'),
        DeclareLaunchArgument('target_pose_executor_planning_scene_topic', default_value='/planning_scene', description='PlanningScene diff 发布话题'),
        DeclareLaunchArgument('target_pose_executor_scene_publish_retries', default_value='5', description='启动后重复发布固定世界障碍物次数'),
        DeclareLaunchArgument('use_payload_scene_sync', default_value='true', description='是否启用 payload_active -> MoveIt 场景同步'),
        DeclareLaunchArgument('use_position_printer', default_value='false', description='是否打印各关节当前角度'),
        DeclareLaunchArgument('position_print_topic', default_value='/rc_arm_2/mujoco_joint_positions', description='角度打印订阅话题（JointState.position）'),
        DeclareLaunchArgument('position_print_rate', default_value='10.0', description='角度打印频率（Hz）'),
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
            'hardware_config_file': LaunchConfiguration('hardware_config_file'),
            'controllers_file': LaunchConfiguration('controllers_file'),
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

    dm_serial_frame_bridge = Node(
        package='dmbot_serial',
        executable='dm_serial_frame_bridge',
        name='dm_serial_frame_bridge',
        output='screen',
        parameters=[{
            'sn': ParameterValue(LaunchConfiguration('dm_serial_bridge_sn'), value_type=str),
            'nom_baud': ParameterValue(LaunchConfiguration('dm_serial_bridge_nom_baud'), value_type=int),
            'dat_baud': ParameterValue(LaunchConfiguration('dm_serial_bridge_dat_baud'), value_type=int),
            'rx_topic': ParameterValue(LaunchConfiguration('dm_serial_bridge_rx_topic'), value_type=str),
            'tx_topic': ParameterValue(LaunchConfiguration('dm_serial_bridge_tx_topic'), value_type=str),
        }],
        condition=IfCondition(LaunchConfiguration('use_dm_serial_frame_bridge')),
    )

    arm2_middleware = Node(
        package='rc_arm2_middleware',
        executable='arm2_middleware',
        name='arm2_middleware',
        output='screen',
        parameters=[{
            'action_sets_file': ParameterValue(LaunchConfiguration('middleware_action_sets_file'), value_type=str),
            'target_point_topic': ParameterValue(LaunchConfiguration('middleware_target_point_topic'), value_type=str),
            'run_action_set_topic': ParameterValue(LaunchConfiguration('middleware_run_action_set_topic'), value_type=str),
            'motion_target_topic': ParameterValue(LaunchConfiguration('middleware_motion_target_topic'), value_type=str),
            'motion_execution_topic': ParameterValue(LaunchConfiguration('middleware_motion_result_topic'), value_type=str),
            'dm_serial_bridge_enabled': ParameterValue(LaunchConfiguration('middleware_dm_serial_bridge_enabled'), value_type=bool),
            'dm_serial_rx_topic': ParameterValue(LaunchConfiguration('middleware_dm_serial_rx_topic'), value_type=str),
            'dm_serial_tx_topic': ParameterValue(LaunchConfiguration('middleware_dm_serial_tx_topic'), value_type=str),
            'dm_serial_command_base_id': ParameterValue(LaunchConfiguration('middleware_dm_serial_command_base_id'), value_type=int),
            'dm_serial_complete_id': ParameterValue(LaunchConfiguration('middleware_dm_serial_complete_id'), value_type=int),
            'dm_serial_allowed_action_set_ids': ParameterValue(LaunchConfiguration('middleware_dm_serial_allowed_action_set_ids'), value_type=str),
        }],
        condition=IfCondition(LaunchConfiguration('use_arm2_middleware')),
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
            '--middleware-target-topic',
            LaunchConfiguration('middleware_motion_target_topic'),
            '--middleware-result-topic',
            LaunchConfiguration('middleware_motion_result_topic'),
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
            '--j4-axis',
            LaunchConfiguration('target_pose_executor_j4_axis'),
            '--joint-state-topic',
            LaunchConfiguration('target_pose_executor_joint_state_topic'),
            '--urdf-path',
            LaunchConfiguration('target_pose_executor_urdf_path'),
            '--status-log-period',
            LaunchConfiguration('target_pose_executor_status_log_period'),
            '--status-base-frame',
            LaunchConfiguration('target_pose_executor_status_base_frame'),
            '--status-eef-frame',
            LaunchConfiguration('target_pose_executor_status_eef_frame'),
            '--avoid-collisions-enabled',
            LaunchConfiguration('target_pose_executor_avoid_collisions_enabled'),
            '--world-boxes-json',
            LaunchConfiguration('target_pose_executor_world_boxes_json'),
            '--planning-scene-topic',
            LaunchConfiguration('target_pose_executor_planning_scene_topic'),
            '--scene-publish-retries',
            LaunchConfiguration('target_pose_executor_scene_publish_retries'),
            '--middleware-preempt-interval-sec',
            LaunchConfiguration('target_pose_executor_middleware_preempt_interval_sec'),
            '--middleware-preempt-pos-threshold',
            LaunchConfiguration('target_pose_executor_middleware_preempt_pos_threshold'),
            '--middleware-preempt-rot-threshold',
            LaunchConfiguration('target_pose_executor_middleware_preempt_rot_threshold'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_target_pose_moveit_executor')),
    )

    payload_scene_sync = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([
                FindPackageShare('rc_arm_moveit_config'),
                'launch',
                'payload_scene_sync.py',
            ]),
            '--hardware-config-file',
            LaunchConfiguration('hardware_config_file'),
            '--planning-scene-topic',
            LaunchConfiguration('target_pose_executor_planning_scene_topic'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_payload_scene_sync')),
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

    position_printer = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([
                FindPackageShare('rc_arm_moveit_config'),
                'launch',
                'joint_position_printer.py',
            ]),
            '--topic',
            LaunchConfiguration('position_print_topic'),
            '--rate',
            LaunchConfiguration('position_print_rate'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_position_printer')),
    )

    return LaunchDescription(
        declared_arguments + [
            include_robot,
            tf_target_bridge,
            dm_serial_frame_bridge,
            arm2_middleware,
            target_pose_executor,
            payload_scene_sync,
            position_printer,
            torque_printer,
        ]
    )
