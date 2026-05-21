# RC Robot Arm MuJoCo + Ruckig

本项目面向 `rc_arm_2` 四轴机械臂，集成了 MuJoCo 仿真、ROS 2 Control、实机 CAN 电机后端、TF 目标位姿跟随、中间件动作集执行，以及基于 IK + Ruckig + `FollowJointTrajectory` 的轨迹生成与执行链。

## 仓库结构

```text
.
├── demo/                         # GUI、TF 目标发布与 MuJoCo 演示脚本
├── scripts/                      # 一键启动仿真/实机链路脚本
├── rc_arm_stack/
│   ├── arm_msgs/                 # 自定义消息
│   ├── rc_arm_motion_config/     # 目标执行器、launch、Ruckig 限制配置
│   ├── rc_arm2_middleware/       # action set 顺序执行中间件
│   ├── rc_arm_controller/        # FollowJointTrajectory 控制器
│   ├── rc_arm_description/       # URDF、控制器与硬件配置
│   ├── rc_arm_hardware/          # ros2_control 硬件插件
│   └── dmbot_serial/             # 实机串口/CAN 后端
└── rc_robotarm_mujoco/           # MuJoCo 侧模型与工具
```

## 执行链

1. 上游发送 `/rc_arm_2/target_pose` 或 `/arm2/middleware/motion_target`
2. `target_pose_ruckig_executor.py` 先做 IK
3. executor 读取 `ruckig_joint_limits.yaml`，用 IK + Ruckig 在线生成单点流式参考
4. executor 通过 `/arm_controller/joint_trajectory` 下发单点轨迹参考
5. `rc_arm_controller` 以流式方式执行参考并返回结果
6. middleware 从 `/arm2/middleware/motion_execution` 获取成功/失败状态

`payload_active` 链路与轨迹执行解耦，仍由：

`payload_active_command -> payload_active -> hardware payload_active_`

驱动带载增益、重力补偿和逆动力学前馈切换。

## 构建

```bash
cd rc_arm_stack
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果在虚拟环境中构建，请确保当前 Python 解释器能导入 `em` 与 `yaml`，并且运行环境可导入 `ruckig`。

## 启动

仿真链路：

```bash
./scripts/run_rc_arm_mujoco.sh
```

实机链路：

```bash
./scripts/run_rc_arm_real.sh
```

桥接演示：

```bash
./scripts/run_rc_arm_mujoco_bridge.sh
```

## 关键文件

- `rc_arm_stack/rc_arm_motion_config/launch/target_pose_ruckig_executor.py`
- `rc_arm_stack/rc_arm_motion_config/config/rc_arm_2/ruckig_joint_limits.yaml`
- `rc_arm_stack/rc_arm_motion_config/launch/rc_arm_2_robot.launch.py`
- `rc_arm_stack/rc_arm2_middleware/rc_arm2_middleware/arm2_middleware_node.py`
- `rc_arm_stack/rc_arm_controller/src/rc_arm_controller.cpp`

## 说明

- 当前仓库不再包含任何基于外部规划场景或碰撞物体同步的执行链。
- Ruckig 执行器默认采用“计划态连续推进，反馈仅用于启动对齐和严重失配恢复”的策略。
- 目标变化默认只更新 `target_position`，不再因为 `target_change` 直接从实机反馈重建 OTG。
- 与反馈重同步相关的参数通过 `rc_arm_2_robot.launch.py` 暴露，默认值偏向实机稳定：
  - `target_pose_executor_feedback_sync_mode=desync_only`
  - `target_pose_executor_feedback_position_reset_threshold=0.12`
  - `target_pose_executor_feedback_position_reset_cycles=3`
  - `target_pose_executor_feedback_velocity_reset_enabled=false`
  - `target_pose_executor_feedback_velocity_reset_threshold=1.5`
  - `target_pose_executor_feedback_velocity_filter_alpha=0.2`
  - `target_pose_executor_feedback_accel_mode=zero`
- `rc_arm_stack/build`、`rc_arm_stack/install` 和 `rc_arm_stack/log` 为构建产物目录，可按需重新生成。
