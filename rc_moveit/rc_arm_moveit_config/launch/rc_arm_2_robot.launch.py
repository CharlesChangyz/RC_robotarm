from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
        DeclareLaunchArgument('use_rviz', default_value='false', description='是否启动带 MoveIt 插件的 RViz2'),
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
        DeclareLaunchArgument('target_pose_executor_vel_scale', default_value='0.8', description='MoveIt 速度缩放（0~1）'),
        DeclareLaunchArgument('target_pose_executor_acc_scale', default_value='0.8', description='MoveIt 加速度缩放（0~1）'),
        DeclareLaunchArgument('target_pose_executor_joint_tolerance', default_value='0.02', description='MoveIt 关节目标容差（rad）'),
        DeclareLaunchArgument('target_pose_executor_check_period', default_value='0.05', description='target_pose 执行器轮询周期（s）'),
        DeclareLaunchArgument('target_pose_executor_enforce_j4_from_target', default_value='true', description='位置优先 IK 下是否从目标姿态提取并强制写回 j4'),
        DeclareLaunchArgument('target_pose_executor_j4_joint_name', default_value='j4_joint', description='j4 关节名称（用于强制写回）'),
        DeclareLaunchArgument('target_pose_executor_j4_axis', default_value='x', description='目标姿态中 j4 对应的旋转轴（x/y/z）'),
        DeclareLaunchArgument('target_pose_executor_status_log_period', default_value='1.0', description='target_pose 执行器状态心跳打印周期（秒，<=0 关闭）'),
        DeclareLaunchArgument('target_pose_executor_status_base_frame', default_value='world', description='状态打印使用的基坐标系'),
        DeclareLaunchArgument('target_pose_executor_status_eef_frame', default_value='end_effector', description='状态打印使用的末端坐标系'),
        DeclareLaunchArgument('target_pose_executor_avoid_collisions_enabled', default_value='true', description='target_pose 执行器是否启用 MoveIt 避障'),
        DeclareLaunchArgument('target_pose_executor_world_boxes_json', default_value='[]', description='启动时注入的世界障碍物 box JSON 数组'),
        DeclareLaunchArgument('target_pose_executor_attached_box_command_topic', default_value='/rc_arm_2/attached_box_command', description='运行时末端附着方块 JSON 命令话题'),
        DeclareLaunchArgument('target_pose_executor_planning_scene_topic', default_value='/planning_scene', description='PlanningScene diff 发布话题'),
        DeclareLaunchArgument('target_pose_executor_scene_publish_retries', default_value='5', description='启动后重复发布固定世界障碍物次数'),
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
            '--avoid-collisions-enabled',
            LaunchConfiguration('target_pose_executor_avoid_collisions_enabled'),
            '--world-boxes-json',
            LaunchConfiguration('target_pose_executor_world_boxes_json'),
            '--attached-box-command-topic',
            LaunchConfiguration('target_pose_executor_attached_box_command_topic'),
            '--planning-scene-topic',
            LaunchConfiguration('target_pose_executor_planning_scene_topic'),
            '--scene-publish-retries',
            LaunchConfiguration('target_pose_executor_scene_publish_retries'),
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
        declared_arguments + [include_robot, tf_target_bridge, target_pose_executor, position_printer, torque_printer]
    )
