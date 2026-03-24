#!/usr/bin/env python3
"""
在 MuJoCo 中可视化规划的轨迹
在原场景的基础上添加轨迹标记球体
"""

import sys
import os
import numpy as np
import gymnasium
import mujoco

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

import rc_robotarm_mujoco
from rc_robotarm_mujoco.planning import MotionPlanner

# 创建环境
orientation_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
env = gymnasium.make(
    "rc_robotarm_mujoco/RC_ARM_2Env-v0",
    render_mode="human",
    orientation_axis=orientation_axis,
)

observation, info = env.reset(seed=42)
physics = env.unwrapped._physics
arm = env.unwrapped._arm

# 创建规划器
motion_planner = MotionPlanner(
    arena_mjcf_model=env.unwrapped._arena.mjcf_model,
    arm=arm,
    ompl_solve_time=5.0,
    ik_position_tol=0.05,
    ik_n_restarts=30,
)

# 获取当前 EEF
current_eef = arm.get_eef_pose(physics)
print(f"[Trajectory Viz] Current EEF: {np.round(current_eef[:3], 3)}")

# 定义目标
target = current_eef.copy()
target[0] += 0.1
target[1] -= 0.05

print(f"[Trajectory Viz] Target: {np.round(target[:3], 3)}")

# 规划轨迹
q_current = physics.bind(arm.joints).qpos.copy()
success = motion_planner.request(target, q_current)

if not success:
    print("[Trajectory Viz] Planning failed!")
    env.close()
    sys.exit(1)

# 提取轨迹
duration = motion_planner.duration
num_waypoints = int(duration * 100)  # 100Hz 采样
time_samples = np.linspace(0, duration, num_waypoints)

trajectory_waypoints = []
for t in time_samples:
    eef_pose = motion_planner.query_eef_pose(t)
    trajectory_waypoints.append(eef_pose[:3])

trajectory_waypoints = np.array(trajectory_waypoints)
print(f"[Trajectory Viz] Extracted {len(trajectory_waypoints)} waypoints")

# 模拟运行（显示轨迹和机械臂运动）
print("[Trajectory Viz] Starting visualization... Press Ctrl+C to exit")

try:
    traj_start_time = None
    step = 0

    # 获取 renderer（如果存在）
    renderer = getattr(env.unwrapped, 'renderer', None)

    while True:
        # 初始化轨迹时间
        if traj_start_time is None:
            traj_start_time = 0
        else:
            traj_start_time += physics.timestep()

        # 查询当前 EEF 目标位置
        if traj_start_time <= duration:
            eef_pose = motion_planner.query_eef_pose(traj_start_time)
            env.unwrapped._target.set_mocap_pose(
                physics,
                position=eef_pose[:3],
                quaternion=eef_pose[3:],
            )
        else:
            # 轨迹完成，保持最终位置
            eef_pose = trajectory_waypoints[-1]

        # 步进仿真
        action = np.zeros(env.action_space.shape, dtype=np.float64)
        observation, reward, terminated, truncated, info = env.step(action)

        # 绘制轨迹（每 2 步绘制一次）
        if step % 2 == 0 and renderer is not None:
            try:
                # 绘制整条轨迹线 - 采样关键点
                sample_indices = np.linspace(0, len(trajectory_waypoints)-1, 30, dtype=int)
                for idx in sample_indices:
                    p = trajectory_waypoints[idx]
                    # 用 MuJoCo 的点来标记轨迹
                    # 色彩梯度：绿 → 黄 → 红
                    t_ratio = idx / (len(trajectory_waypoints) - 1)
                    if t_ratio < 0.5:
                        rgba = np.array([2*t_ratio, 1.0, 0.0, 0.6])
                    else:
                        rgba = np.array([1.0, 2*(1-t_ratio), 0.0, 0.6])

                    # 绘制轨迹点
                    mujoco.mjv_point(renderer.scn, rgba, p, 0.008)

                # 绘制起点（大绿球）
                start_color = np.array([0.0, 1.0, 0.0, 0.8])
                mujoco.mjv_point(renderer.scn, start_color, trajectory_waypoints[0], 0.02)

                # 绘制终点（大红球）
                end_color = np.array([1.0, 0.0, 0.0, 0.8])
                mujoco.mjv_point(renderer.scn, end_color, trajectory_waypoints[-1], 0.02)

                # 绘制当前目标位置（蓝球）
                current_target = motion_planner.query_eef_pose(min(traj_start_time, duration))[:3]
                target_color = np.array([0.0, 0.5, 1.0, 0.8])
                mujoco.mjv_point(renderer.scn, target_color, current_target, 0.015)
            except Exception as e:
                print(f"[Trajectory Viz] Drawing error: {e}")

        step += 1

        if terminated or truncated:
            observation, info = env.reset()
            traj_start_time = None

finally:
    env.close()
