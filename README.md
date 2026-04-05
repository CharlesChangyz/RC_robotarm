# RC_robotarm 避障禁区配置说明（MoveIt）

已在 `rc_moveit/rc_arm_moveit_config/launch/rc_arm_2_robot.launch.py` 的 `target_pose` 执行链路中增加可自定义避障配置，支持两类禁区：

1. **机械臂本体/末端 自碰撞禁区**
2. **周围环境碰撞禁区（盒体）**

## 1) 机械臂与末端自碰撞禁区

参数名：`target_pose_executor_forbidden_link_pairs`  
格式：`link_a:link_b,link_c:link_d`

示例：

```bash
target_pose_executor_forbidden_link_pairs:=base_link:end_effector,l3:end_effector
```

## 2) 环境碰撞禁区（盒体）

参数名：`target_pose_executor_env_forbidden_boxes_json`  
格式：JSON 数组，每个元素一个 box：

- `id`：障碍物名字（可选）
- `frame_id`：坐标系（可选，默认 `world`）
- `size`：`[x, y, z]`（米）
- `position`：`[x, y, z]`（米）
- `orientation`：`[x, y, z, w]`（可选，默认单位四元数）

示例：

```bash
target_pose_executor_env_forbidden_boxes_json:='[
  {"id":"forbidden_box_1","frame_id":"world","size":[0.20,0.20,0.30],"position":[0.35,0.00,0.25]},
  {"id":"forbidden_box_2","frame_id":"world","size":[0.15,0.15,0.25],"position":[0.20,0.25,0.20]}
]'
```

## 3) 快速启动示例

```bash
ros2 launch rc_arm_moveit_config rc_arm_2_robot.launch.py \
  use_target_pose_moveit_executor:=true \
  target_pose_executor_avoid_collisions_enabled:=true \
  target_pose_executor_forbidden_link_pairs:=base_link:end_effector \
  target_pose_executor_env_forbidden_boxes_json:='[{"id":"keep_out","frame_id":"world","size":[0.2,0.2,0.2],"position":[0.3,0.0,0.3]}]'
```

上述配置会通过 `planning_scene` 动态下发到 MoveIt，实现可配置的避障禁区。
