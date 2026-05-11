# RC Robot Arm MuJoCo + Ruckig

中文 | [English](#english)

## 项目简介

本项目面向 `rc_arm_2` 四轴机械臂，集成了 MuJoCo 仿真、ROS 2 Control、Ruckig 轨迹生成、实机 CAN 电机后端、TF 目标位姿跟随和 Xbox 手柄遥操作。

主链路语义：

- `tf_target_pose_bridge.py` 将 `world -> rc_arm_2_target` TF 转成 `/rc_arm_2/target_pose`
- `target_pose_ruckig_executor.py` 使用仓库内共享 IK 解出目标关节
- `ruckig_trajectory_server` 生成平滑点到点关节轨迹
- `arm_controller/follow_joint_trajectory` 执行轨迹

项目目录：

```text
.
├── rc_robotarm_mujoco/           # MuJoCo 机器人、场地、资产和工具代码
├── demo/                         # MuJoCo 仿真桥接和 TF 目标发布脚本
├── scripts/                      # 一键启动仿真/实机链路脚本
├── rc_ruckig/
│   ├── rc_arm_description/       # URDF/Xacro、网格、ros2_control 配置
│   ├── rc_arm_ruckig_config/     # Ruckig launch、执行器、RViz 配置
│   ├── rc_arm_kinematics/        # 共享 IK/FK 求解器
│   ├── rc_arm_hardware/          # 实机/MuJoCo ros2_control 硬件接口
│   ├── rc_arm_controller/        # 自定义轨迹控制器
│   ├── rc_arm_teleop/            # Xbox 手柄遥操作
│   ├── dmbot_serial/             # 达妙 USB2CANFD 驱动相关代码
│   └── arm_msgs/                 # 自定义消息与服务
└── requirements.txt              # Python/MuJoCo 侧依赖
```

## 环境要求

- Ubuntu + ROS 2 Humble
- ros2_control、controller_manager、RViz2、xacro、ruckig
- Python 3.8+
- MuJoCo、dm-control、NumPy、SciPy
- 实机模式需要 CAN/USB2CANFD 设备和正确的电机 ID 配置

安装 Python 依赖：

```bash
python3 -m pip install -r requirements.txt
python3 -m pip install -e .
```

## 构建 ROS 2 工作空间

```bash
source /opt/ros/humble/setup.bash
cd rc_ruckig
colcon build --symlink-install
source install/setup.bash
```

如果后续修改了 `rc_ruckig` 下的包，请重新执行 `colcon build --symlink-install` 并重新 source。

## MuJoCo 仿真运行

终端 1：

```bash
./scripts/run_rc_arm_mujoco_bridge.sh
```

终端 2：

```bash
./scripts/run_rc_arm_mujoco.sh
```

常用参数：

```bash
USE_RVIZ=false ./scripts/run_rc_arm_mujoco.sh

./scripts/run_rc_arm_mujoco.sh \
  use_tf_target_bridge:=true \
  use_target_pose_ruckig_executor:=true
```

MuJoCo 后端配置：

```text
rc_ruckig/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.mujoco.yaml
```

## 实机运行

先检查：

```text
rc_ruckig/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.real.yaml
```

启动：

```bash
./scripts/run_rc_arm_real.sh
```

调试时可关闭自动目标执行或 RViz：

```bash
USE_TARGET_POSE_RUCKIG_EXECUTOR=false ./scripts/run_rc_arm_real.sh
USE_RVIZ=false ./scripts/run_rc_arm_real.sh
```

## TF 目标位姿控制

`rc_arm_2_robot.launch.py` 默认会启动两段链路：

1. `tf_target_pose_bridge.py`：从 `/tf` 中读取 `world -> rc_arm_2_target`
2. `target_pose_ruckig_executor.py`：订阅 `/rc_arm_2/target_pose`，先解 IK，再生成并执行关节轨迹

可以使用 GUI 工具发布目标 TF：

```bash
source /opt/ros/humble/setup.bash
source rc_ruckig/install/setup.bash
python3 demo/tf_target_cli_publisher.py
```

默认快捷键：

```text
Left / Right        调 x
Down / Up           调 y
PageDown / PageUp   调 z
[ / ]               调 j4
Ctrl+Enter          Send
```

目标位姿执行器常用参数：

```bash
./scripts/run_rc_arm_mujoco.sh \
  target_pose_executor_velocity_scale:=0.3 \
  target_pose_executor_acceleration_scale:=0.3 \
  target_pose_executor_jerk_scale:=0.3
```

## 吸附 / 负载联动

统一联动语义如下：

- 发布 `true` 到 `/rc_arm_2/vacuum_activate`
  - 真空打开
  - `rc_arm_hardware` 切到 `payload_*` 参数集
  - `/rc_arm_2/payload_active` 变为 `true`
  - MuJoCo 专用 `payload_block` 绑定到 `attachment_site`
- 发布 `false` 到 `/rc_arm_2/vacuum_activate`
  - 真空关闭
  - `rc_arm_hardware` 切回 `unloaded_*`
  - `/rc_arm_2/payload_active` 变为 `false`
  - MuJoCo 释放 `payload_block`

## English

This repository targets the `rc_arm_2` 4-DOF arm and uses a Ruckig-based execution stack instead of MoveIt.

Core pipeline:

- TF target bridge publishes `/rc_arm_2/target_pose`
- Shared IK solves the target joint state
- Ruckig generates a smooth point-to-point joint trajectory
- `arm_controller/follow_joint_trajectory` executes it

Build:

```bash
source /opt/ros/humble/setup.bash
cd rc_ruckig
colcon build --symlink-install
source install/setup.bash
```

Run MuJoCo:

```bash
./scripts/run_rc_arm_mujoco_bridge.sh
./scripts/run_rc_arm_mujoco.sh
```

Run real hardware:

```bash
./scripts/run_rc_arm_real.sh
```
