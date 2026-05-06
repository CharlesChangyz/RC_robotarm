# RC Robot Arm MuJoCo + MoveIt

中文 | [English](#english)

## 项目简介

本项目面向 `rc_arm_2` 四轴机械臂，集成了 MuJoCo 仿真、ROS 2 Control、MoveIt 2 规划执行、实机 CAN 电机后端、TF 目标位姿跟随、碰撞物体管理和 Xbox 手柄遥操作。

主要能力：

- 在 MuJoCo 中加载机械臂和赛场环境，并通过 ROS 2 话题与控制链路联动。
- 使用 MoveIt 2 对 `arm` 规划组进行 IK、路径规划和轨迹执行。
- 通过 `ros2_control` 在 MuJoCo 后端和实机后端之间切换。
- 从 TF 目标帧生成 `PoseStamped`，自动触发 MoveIt 规划执行。
- 通过统一的 `vacuum_activate -> payload_active` 链路同步真空吸附、带载参数、MoveIt 场景附着和 MuJoCo 负载块。
- 支持 Xbox 手柄控制仿真或实机机械臂。

## 目录结构

```text
.
├── rc_robotarm_mujoco/           # MuJoCo 机器人、场地、资产和工具代码
├── demo/                         # MuJoCo 仿真桥接和 TF 目标发布脚本
├── scripts/                      # 一键启动 MoveIt 仿真/实机链路脚本
├── rc_moveit/
│   ├── rc_arm_description/       # URDF/Xacro、网格、ros2_control 配置
│   ├── rc_arm_moveit_config/     # MoveIt 2 配置、launch、目标位姿执行器
│   ├── rc_arm_hardware/          # 实机/MuJoCo ros2_control 硬件接口
│   ├── rc_arm_controller/        # 自定义轨迹控制器
│   ├── rc_arm_teleop/            # Xbox 手柄遥操作
│   ├── dmbot_serial/             # 达妙 USB2CANFD 驱动相关代码
│   └── arm_msgs/                 # 自定义消息
└── requirements.txt              # Python/MuJoCo 侧依赖
```

## 环境要求

- Ubuntu + ROS 2 Humble
- MoveIt 2、ros2_control、controller_manager、RViz2、xacro
- Python 3.8+
- MuJoCo、dm-control、NumPy、SciPy、OMPL、TOPP-RA
- 实机模式需要 CAN/USB2CANFD 设备和正确的电机 ID 配置

安装 Python 依赖：

```bash
python3 -m pip install -r requirements.txt
python3 -m pip install -e .
```

## 构建 ROS 2 工作空间

```bash
source /opt/ros/humble/setup.bash
cd rc_moveit
colcon build --symlink-install
source install/setup.bash
```

如果后续修改了 `rc_moveit` 下的包，请重新执行 `colcon build --symlink-install` 并重新 source。

## MuJoCo 仿真运行

终端 1：启动 MuJoCo 仿真桥接。它会发布 `/rc_arm_2/mujoco_joint_states`、`/rc_arm_2/mujoco_joint_positions`、`/rc_arm_2/joint_torque` 等话题，并接收控制链路输出的关节命令，同时在场景里额外创建专用 `payload_block`。

```bash
./scripts/run_rc_arm_mujoco_bridge.sh
```

终端 2：启动 MoveIt、ros2_control、RViz 和目标位姿执行链路。

```bash
./scripts/run_rc_arm_mujoco.sh
```

常用 launch 参数仍然可以追加：

```bash
USE_RVIZ=false ./scripts/run_rc_arm_mujoco.sh

./scripts/run_rc_arm_mujoco.sh \
  use_tf_target_bridge:=true \
  use_target_pose_moveit_executor:=true
```

MuJoCo 后端和负载参数配置位于：

```text
rc_moveit/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.mujoco.yaml
```

## 实机运行

先检查实机硬件配置：

```text
rc_moveit/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.real.yaml
```

重点确认：

- `can_interface`：例如 `can0`
- `dm_sn`：USB2CANFD 设备序列号
- `motor_id_j1` 到 `motor_id_j4`：各关节电机 CAN ID
- `unloaded_*`、`payload_*` 两套控制默认值
- 重力补偿和逆动力学前馈开关

启动实机链路：

```bash
./scripts/run_rc_arm_real.sh
```

实机调试建议先降低速度缩放、关闭自动目标执行或关闭 RViz：

```bash
USE_TARGET_POSE_MOVEIT_EXECUTOR=false ./scripts/run_rc_arm_real.sh
USE_RVIZ=false ./scripts/run_rc_arm_real.sh
```

## TF 目标位姿控制

`rc_arm_2_robot.launch.py` 默认会启动两段链路：

1. `tf_target_pose_bridge.py`：从 `/tf` 中读取 `world -> rc_arm_2_target`。
2. `target_pose_moveit_executor.py`：订阅 `/rc_arm_2/target_pose`，用仓库内共享 4DOF solver 先解出 `j1..j4`，再把 joint goal 交给 MoveIt 做规划、避障和执行。

可以使用 GUI 工具发布目标 TF：

```bash
source /opt/ros/humble/setup.bash
source rc_moveit/install/setup.bash
python3 demo/tf_target_cli_publisher.py
```

GUI 主要行为：

- `Target Editor`：编辑 `x / y / z / j4 world`，其中 `j4 world = 0 deg` 表示工具水平
- `Send`：只在按下时单次发布一次 `/tf` 目标，不再周期持续发布
- `Reset to current`：用当前 `world -> end_effector` 实际位置和真实关节态经共享 FK 算出的当前 `j4 world` 回填编辑框
- `Home`：回到 URDF/实机零位 FK 对应的固定 `xyz + 0 deg`
- `Editing target / Last sent target / Actual current pose` 分开显示，避免“改了但没发”的歧义
- `Reachability`：显示当前编辑点的 `Reachable / Near limit / Unreachable` 状态和近似 `x/y/z` 范围
- `System Control`：可直接启动 `run_rc_arm_mujoco.sh`、`run_rc_arm_mujoco_bridge.sh`、`run_rc_arm_real.sh`，以及单次发送 `Vacuum ON/OFF`

这条 GUI 目标链不再调用 MoveIt 的 `/compute_ik`。`/compute_ik` 仍保留给 teleop 等其他链路使用；当前 GUI / target executor 链只把 MoveIt 用在 joint-goal 规划、碰撞检查和执行上。

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
  target_pose_executor_vel_scale:=0.3 \
  target_pose_executor_acc_scale:=0.3 \
  target_pose_executor_planning_time:=2.0 \
  target_pose_executor_avoid_collisions_enabled:=true
```

## MoveIt 碰撞物体

### 世界障碍物

启动时可向 MoveIt Planning Scene 注入固定 box：

```bash
ros2 launch rc_arm_moveit_config rc_arm_2_robot.launch.py \
  use_rviz:=true \
  target_pose_executor_world_boxes_json:='[{"id":"keep_out","frame_id":"world","size":[0.2,0.2,0.2],"position":[0.3,0.0,0.3]}]'
```

单个 box 支持字段：

```json
{
  "id": "keep_out",
  "frame_id": "world",
  "size": [0.2, 0.2, 0.2],
  "position": [0.3, 0.0, 0.3],
  "orientation": [0.0, 0.0, 0.0, 1.0]
}
```

### 末端附着物体

`/rc_arm_2/attached_box_command` 已移除，不再手工发布 attach/detach JSON。

当前语义改为：

- 用户侧唯一外部输入：`/rc_arm_2/vacuum_activate` (`std_msgs/Bool`)
- 系统内部权威状态：`/rc_arm_2/payload_active` (`std_msgs/Bool`)
- `payload_scene_sync.py` 会根据 `/rc_arm_2/payload_active` 自动把 box 附着到 `end_effector` 或从场景移除

也就是说，用户只需要切换吸附命令，不再直接编辑 MoveIt attached collision object。

## 吸附 / 负载联动

统一联动语义如下：

- 发布 `true` 到 `/rc_arm_2/vacuum_activate`
  - 真空打开
  - `rc_arm_hardware` 切到 `payload_*` 参数集
  - `/rc_arm_2/payload_active` 变为 `true`
  - MoveIt 末端自动附着 payload box
  - MuJoCo 专用 `payload_block` 绑定到 `attachment_site`
- 发布 `false` 到 `/rc_arm_2/vacuum_activate`
  - 真空关闭
  - `rc_arm_hardware` 切回 `unloaded_*`
  - `/rc_arm_2/payload_active` 变为 `false`
  - MoveIt 移除附着 box
  - MuJoCo 释放 `payload_block`

手工测试命令：

```bash
ros2 topic pub --once /rc_arm_2/vacuum_activate std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /rc_arm_2/vacuum_activate std_msgs/msg/Bool "{data: false}"
```

## 参数配置

payload 和 unloaded 的默认值只放在：

```text
rc_moveit/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.real.yaml
rc_moveit/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.mujoco.yaml
```

主要命名约定：

- 通用：`vacuum_activate_topic`、`payload_active_topic`、`payload_frame`
- 空载默认：`unloaded_position_kp`、`unloaded_position_kd`、`unloaded_low_stiffness_*`
- 带载默认：`payload_mass`、`payload_box_size_*`、`payload_position_kp`、`payload_position_kd`、`payload_low_stiffness_*`
- MuJoCo 专用：`mujoco_payload_body_name`、`mujoco_payload_site_name`、`mujoco_payload_initial_pos_*`

脚本不会覆盖这些参数；修改默认行为请直接改对应 YAML。

## Xbox 手柄遥操作

仿真遥操作：

```bash
source /opt/ros/humble/setup.bash
source rc_moveit/install/setup.bash
ros2 launch rc_arm_teleop rc_arm_2_sim_teleop.launch.py device:=/dev/input/js0
```

实机遥操作：

```bash
source /opt/ros/humble/setup.bash
source rc_moveit/install/setup.bash
ros2 launch rc_arm_teleop rc_arm_2_real_teleop.launch.py device:=/dev/input/js0
```

手柄参数位于：

```text
rc_moveit/rc_arm_teleop/config/rc_arm_2/xbox_teleop.yaml
```

## 常用调试命令

```bash
ros2 topic list
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states
ros2 topic echo /rc_arm_2/mujoco_joint_states
ros2 topic echo /rc_arm_2/joint_torque
```

查看目标位姿执行器状态时，可以开启位置或力矩打印：

```bash
./scripts/run_rc_arm_mujoco.sh \
  use_position_printer:=true \
  use_torque_printer:=true
```

## 注意事项

- `rc_moveit/install/setup.bash` 不存在时，请先构建 `rc_moveit` 工作空间。
- MuJoCo 模式需要先运行仿真桥接，否则硬件插件会收不到外部反馈并回退到内部状态。
- MuJoCo 场景里的 `payload_block` 是额外创建的专用负载块，不复用赛场默认方块。
- 实机模式请在上电前确认 CAN 接口、电机 ID、关节限位、低刚度/重力补偿参数。
- 默认规划组为 `arm`，关节顺序为 `j1_joint,j2_joint,j3_joint,j4_joint`。
- `world` 到 `base_link` 的静态 TF 由 launch 文件发布。

---

## English

[中文](#rc-robot-arm-mujoco--moveit) | English

## Overview

This repository targets the `rc_arm_2` four-axis robot arm. It combines MuJoCo simulation, ROS 2 Control, MoveIt 2 planning/execution, a real CAN motor backend, TF target following, MoveIt collision-object management, and Xbox controller teleoperation.

Key features:

- Load the robot arm and field assets in MuJoCo and bridge them to ROS 2 topics.
- Plan and execute motion for the MoveIt `arm` planning group.
- Switch between MuJoCo and real hardware through `ros2_control` hardware configuration files.
- Convert a TF target frame into `PoseStamped` goals and execute them through MoveIt.
- Use one `vacuum_activate -> payload_active` chain to synchronize the real vacuum command, loaded control gains, MoveIt attached geometry, and the MuJoCo payload block.
- Teleoperate the simulated or real robot arm with an Xbox controller.

## Repository Layout

```text
.
├── rc_robotarm_mujoco/           # MuJoCo robot, arena, assets, and utilities
├── demo/                         # MuJoCo bridge and TF target publisher scripts
├── scripts/                      # Convenience launch scripts for sim/real stacks
├── rc_moveit/
│   ├── rc_arm_description/       # URDF/Xacro, meshes, ros2_control configs
│   ├── rc_arm_moveit_config/     # MoveIt 2 configs, launch files, target executor
│   ├── rc_arm_hardware/          # Real/MuJoCo ros2_control hardware interface
│   ├── rc_arm_controller/        # Custom trajectory controller
│   ├── rc_arm_teleop/            # Xbox teleoperation
│   ├── dmbot_serial/             # Damiao USB2CANFD driver code
│   └── arm_msgs/                 # Custom messages
└── requirements.txt              # Python/MuJoCo dependencies
```

## Requirements

- Ubuntu with ROS 2 Humble
- MoveIt 2, ros2_control, controller_manager, RViz2, xacro
- Python 3.8+
- MuJoCo, dm-control, NumPy, SciPy, OMPL, TOPP-RA
- Real-hardware mode requires a CAN/USB2CANFD device and correct motor ID configuration

Install Python dependencies:

```bash
python3 -m pip install -r requirements.txt
python3 -m pip install -e .
```

## Build the ROS 2 Workspace

```bash
source /opt/ros/humble/setup.bash
cd rc_moveit
colcon build --symlink-install
source install/setup.bash
```

Rebuild and source again after changing packages under `rc_moveit`.

## Run MuJoCo Simulation

Terminal 1: start the MuJoCo bridge. It publishes `/rc_arm_2/mujoco_joint_states`, `/rc_arm_2/mujoco_joint_positions`, `/rc_arm_2/joint_torque`, consumes joint commands from the control stack, and creates a dedicated `payload_block` body in the scene.

```bash
./scripts/run_rc_arm_mujoco_bridge.sh
```

Terminal 2: start MoveIt, ros2_control, RViz, and the target-pose execution chain.

```bash
./scripts/run_rc_arm_mujoco.sh
```

Common options can be overridden with environment variables or launch arguments:

```bash
USE_RVIZ=false ./scripts/run_rc_arm_mujoco.sh

./scripts/run_rc_arm_mujoco.sh \
  use_tf_target_bridge:=true \
  use_target_pose_moveit_executor:=true
```

MuJoCo backend configuration:

```text
rc_moveit/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.mujoco.yaml
```

## Run Real Hardware

Review the real-hardware configuration first:

```text
rc_moveit/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.real.yaml
```

Important fields:

- `can_interface`: for example `can0`
- `dm_sn`: USB2CANFD device serial number
- `motor_id_j1` to `motor_id_j4`: motor CAN IDs
- `unloaded_*` and `payload_*` control defaults
- gravity compensation and inverse-dynamics feedforward switches

Start the real-hardware stack:

```bash
./scripts/run_rc_arm_real.sh
```

For early real-hardware debugging, consider reducing scaling, disabling automatic target execution, or disabling RViz:

```bash
USE_TARGET_POSE_MOVEIT_EXECUTOR=false ./scripts/run_rc_arm_real.sh
USE_RVIZ=false ./scripts/run_rc_arm_real.sh
```

## TF Target Pose Control

`rc_arm_2_robot.launch.py` starts two target-pose components by default:

1. `tf_target_pose_bridge.py`: reads `world -> rc_arm_2_target` from `/tf`.
2. `target_pose_moveit_executor.py`: subscribes to `/rc_arm_2/target_pose`, solves `j1..j4` with the shared 4DOF solver in this repo, then passes the joint goal to MoveIt for planning, collision checking, and execution.

Use the GUI TF publisher to command a target:

```bash
source /opt/ros/humble/setup.bash
source rc_moveit/install/setup.bash
python3 demo/tf_target_cli_publisher.py
```

The GUI uses `j4 world` semantics: `0 deg` means the tool is level in the arm's radial vertical plane. It keeps `Editing target`, `Last sent target`, and `Actual current pose` separate, publishes only on `Send`, and supports `Reset to current`, `Home`, `Send if changed only`, `Vacuum ON/OFF`, MuJoCo / Real start-stop buttons, and approximate reachability feedback. This GUI/executor chain no longer calls MoveIt's `/compute_ik`; `/compute_ik` remains available for teleop and other chains.

Common target executor options:

```bash
./scripts/run_rc_arm_mujoco.sh \
  target_pose_executor_vel_scale:=0.3 \
  target_pose_executor_acc_scale:=0.3 \
  target_pose_executor_planning_time:=2.0 \
  target_pose_executor_avoid_collisions_enabled:=true
```

## MoveIt Collision Objects

### World Boxes

Inject fixed boxes into the MoveIt Planning Scene at startup:

```bash
ros2 launch rc_arm_moveit_config rc_arm_2_robot.launch.py \
  use_rviz:=true \
  target_pose_executor_world_boxes_json:='[{"id":"keep_out","frame_id":"world","size":[0.2,0.2,0.2],"position":[0.3,0.0,0.3]}]'
```

Each box supports:

```json
{
  "id": "keep_out",
  "frame_id": "world",
  "size": [0.2, 0.2, 0.2],
  "position": [0.3, 0.0, 0.3],
  "orientation": [0.0, 0.0, 0.0, 1.0]
}
```

### Attached Payload

The user-facing `/rc_arm_2/attached_box_command` interface has been removed.

Use:

- `/rc_arm_2/vacuum_activate` (`std_msgs/Bool`) as the only external command
- `/rc_arm_2/payload_active` (`std_msgs/Bool`) as the internal authoritative state

`payload_scene_sync.py` attaches or removes the payload box automatically based on `/rc_arm_2/payload_active`.

## Vacuum / Payload Sync

Publishing `true` to `/rc_arm_2/vacuum_activate` now:

- enables the vacuum command path
- switches `rc_arm_hardware` to the `payload_*` gain set
- publishes `/rc_arm_2/payload_active = true`
- attaches the payload box in MoveIt
- binds the MuJoCo `payload_block` to `attachment_site`

Publishing `false` reverses the same chain.

```bash
ros2 topic pub --once /rc_arm_2/vacuum_activate std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /rc_arm_2/vacuum_activate std_msgs/msg/Bool "{data: false}"
```

## Parameter Layout

All unloaded/payload defaults live only in:

```text
rc_moveit/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.real.yaml
rc_moveit/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.mujoco.yaml
```

Scripts do not override payload defaults. Change the YAML files directly.

## Xbox Teleoperation

Simulation teleop:

```bash
source /opt/ros/humble/setup.bash
source rc_moveit/install/setup.bash
ros2 launch rc_arm_teleop rc_arm_2_sim_teleop.launch.py device:=/dev/input/js0
```

Real-hardware teleop:

```bash
source /opt/ros/humble/setup.bash
source rc_moveit/install/setup.bash
ros2 launch rc_arm_teleop rc_arm_2_real_teleop.launch.py device:=/dev/input/js0
```

Gamepad parameters:

```text
rc_moveit/rc_arm_teleop/config/rc_arm_2/xbox_teleop.yaml
```

## Useful Debug Commands

```bash
ros2 topic list
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states
ros2 topic echo /rc_arm_2/mujoco_joint_states
ros2 topic echo /rc_arm_2/joint_torque
```

Enable joint-position or torque printing from the main launch script:

```bash
./scripts/run_rc_arm_mujoco.sh \
  use_position_printer:=true \
  use_torque_printer:=true
```

## Notes

- If `rc_moveit/install/setup.bash` does not exist, build the `rc_moveit` workspace first.
- In MuJoCo mode, run the simulation bridge first so the hardware plugin receives external joint feedback.
- The MuJoCo `payload_block` is an extra dedicated body; it does not reuse existing field cubes.
- In real-hardware mode, verify the CAN interface, motor IDs, joint limits, stiffness settings, and gravity compensation before powering the arm.
- The default MoveIt planning group is `arm`, with joint order `j1_joint,j2_joint,j3_joint,j4_joint`.
- The launch files publish a static TF from `world` to `base_link`.
