#!/usr/bin/env python3
"""
轨迹可视化脚本 - 绘制 OMPL 规划的轨迹
显示：末端执行器的 XYZ 路径、关节角度变化、速度曲线
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

import gymnasium
import rc_robotarm_mujoco

# 创建环境
orientation_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
env = gymnasium.make(
    "rc_robotarm_mujoco/RC_ARM_2Env-v0",
    render_mode="rgb_array",
    orientation_axis=orientation_axis,
)

observation, info = env.reset(seed=42)
physics = env.unwrapped._physics
arm = env.unwrapped._arm

# 导入规划工具
from rc_robotarm_mujoco.planning import MotionPlanner

# 创建规划器（与 demo_2_ompl.py 相同配置）
motion_planner = MotionPlanner(
    arena_mjcf_model=env.unwrapped._arena.mjcf_model,
    arm=arm,
    ompl_solve_time=5.0,
    ik_position_tol=0.05,
    ik_n_restarts=30,
)

# 获取当前 EEF 和目标
current_eef = arm.get_eef_pose(physics)
print(f"[Viz] Current EEF pose: {np.round(current_eef, 3)}")

# 定义目标
target = current_eef.copy()
target[0] += 0.1   # x 前进 0.1m
target[1] -= 0.05  # y 后移 0.05m

print(f"[Viz] Target XYZ: {np.round(target[:3], 3)}")

# 执行规划
q_current = physics.bind(arm.joints).qpos.copy()
success = motion_planner.request(target, q_current)

if not success:
    print("[Viz] Planning failed!")
    sys.exit(1)

print("[Viz] Planning succeeded! Extracting trajectory...")

# 提取轨迹数据
duration = motion_planner.duration
num_samples = int(duration * 100)  # 100Hz 采样
time_samples = np.linspace(0, duration, num_samples)

eef_positions = []
eef_quats = []
joint_angles = []

for t in time_samples:
    eef_pose = motion_planner.query_eef_pose(t)
    eef_positions.append(eef_pose[:3])
    eef_quats.append(eef_pose[3:])

    # 从轨迹获取关节角（通过 executor）
    q, _ = motion_planner.executor._traj.query(t)
    joint_angles.append(q)

eef_positions = np.array(eef_positions)
eef_quats = np.array(eef_quats)
joint_angles = np.array(joint_angles)

print(f"[Viz] Extracted {len(time_samples)} trajectory points over {duration:.3f}s")

# 创建可视化
fig = plt.figure(figsize=(14, 10))

# 1. 3D 末端执行器路径
ax1 = fig.add_subplot(2, 3, 1, projection='3d')
ax1.plot(eef_positions[:, 0], eef_positions[:, 1], eef_positions[:, 2], 'b-', linewidth=2)
ax1.scatter(*current_eef[:3], color='g', s=100, label='Start')
ax1.scatter(*target[:3], color='r', s=100, label='Goal')
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_zlabel('Z (m)')
ax1.set_title('3D EEF Trajectory')
ax1.legend()
ax1.grid(True)

# 2. X 坐标 vs 时间
ax2 = fig.add_subplot(2, 3, 2)
ax2.plot(time_samples, eef_positions[:, 0], 'r-', linewidth=2, label='X')
ax2.plot(time_samples, eef_positions[:, 1], 'g-', linewidth=2, label='Y')
ax2.plot(time_samples, eef_positions[:, 2], 'b-', linewidth=2, label='Z')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Position (m)')
ax2.set_title('EEF Cartesian Position vs Time')
ax2.legend()
ax2.grid(True)

# 3. 关节角度 vs 时间
ax3 = fig.add_subplot(2, 3, 3)
for i in range(4):
    ax3.plot(time_samples, joint_angles[:, i], linewidth=2, label=f'j{i+1}')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Joint Angle (rad)')
ax3.set_title('Joint Angles vs Time')
ax3.legend()
ax3.grid(True)

# 4. 关节速度（数值微分）
ax4 = fig.add_subplot(2, 3, 4)
dt = time_samples[1] - time_samples[0] if len(time_samples) > 1 else 1.0
joint_velocities = np.gradient(joint_angles, dt, axis=0)
for i in range(4):
    ax4.plot(time_samples, joint_velocities[:, i], linewidth=2, label=f'j{i+1}')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Joint Velocity (rad/s)')
ax4.set_title('Joint Velocities vs Time')
ax4.legend()
ax4.grid(True)

# 5. EEF 速度（线速度）
ax5 = fig.add_subplot(2, 3, 5)
eef_velocities = np.gradient(eef_positions, dt, axis=0)
linear_speed = np.linalg.norm(eef_velocities, axis=1)
ax5.plot(time_samples, linear_speed, 'b-', linewidth=2)
ax5.set_xlabel('Time (s)')
ax5.set_ylabel('Linear Velocity (m/s)')
ax5.set_title('EEF Linear Speed vs Time')
ax5.grid(True)

# 6. 轨迹信息文本
ax6 = fig.add_subplot(2, 3, 6)
ax6.axis('off')
info_text = f"""
Trajectory Information
━━━━━━━━━━━━━━━━━━━━━
Duration: {duration:.3f} s
Num Samples: {len(time_samples)}

Start Position:
  X: {current_eef[0]:.4f} m
  Y: {current_eef[1]:.4f} m
  Z: {current_eef[2]:.4f} m

Goal Position:
  X: {target[0]:.4f} m
  Y: {target[1]:.4f} m
  Z: {target[2]:.4f} m

Distance: {np.linalg.norm(target[:3]-current_eef[:3]):.4f} m

Max Joint Speed:
  j1: {np.max(np.abs(joint_velocities[:, 0])):.3f} rad/s
  j2: {np.max(np.abs(joint_velocities[:, 1])):.3f} rad/s
  j3: {np.max(np.abs(joint_velocities[:, 2])):.3f} rad/s
  j4: {np.max(np.abs(joint_velocities[:, 3])):.3f} rad/s

Max EEF Speed: {np.max(linear_speed):.3f} m/s
"""
ax6.text(0.1, 0.5, info_text, fontfamily='monospace', fontsize=10,
         verticalalignment='center', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

plt.tight_layout()
plt.savefig('/home/dust/rc_robotarm_mujoco/trajectory_visualization.png', dpi=150, bbox_inches='tight')
print(f"[Viz] Trajectory visualization saved to trajectory_visualization.png")
plt.show()

env.close()
