import sys
import os
import time
import threading
import numpy as np
import gymnasium
import mujoco

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

import rc_robotarm_mujoco
from rc_robotarm_mujoco.utils.transform_utils import (
    axisangle2quat,
    quat_multiply,
    quat2mat,
    mat2euler,
)


def _goal_quat_from_j4(tool_zero_quat: np.ndarray, j4_angle: float) -> np.ndarray:
    """计算基于 j4 角度的目标四元数（仅适用于 4DOF 机械臂）"""
    axis_world = quat2mat(tool_zero_quat) @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    axis_world = axis_world / (np.linalg.norm(axis_world) + 1e-8)
    q_axis = axisangle2quat(axis_world * j4_angle)
    goal_quat = quat_multiply(q_axis, tool_zero_quat)
    return goal_quat / (np.linalg.norm(goal_quat) + 1e-8)
from rc_robotarm_mujoco.planning import MotionPlanner

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
except Exception as exc:
    raise SystemExit(
        "未找到 ROS2 依赖。请先确保 rclpy、geometry_msgs、sensor_msgs 已安装并可用。"
    ) from exc


# ---------------------------------------------------------------------------
# Environment setup
# ---------------------------------------------------------------------------
orientation_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
env = gymnasium.make(
    "rc_robotarm_mujoco/RC_ARM_2Env-v0",
    render_mode="human",
    orientation_axis=orientation_axis,
)

observation, info = env.reset(seed=42)
physics = env.unwrapped._physics
arm     = env.unwrapped._arm
joint_dof_ids = physics.bind(arm.joints).dofadr

# ---------------------------------------------------------------------------
# Motion planner (created once after env.reset())
# ---------------------------------------------------------------------------
motion_planner = MotionPlanner(
    arena_mjcf_model=env.unwrapped._arena.mjcf_model,
    arm=arm,
    # Adjust vel/acc limits to match your motors if needed
    # vel_limits=np.array([3.0, 2.0, 2.0, 4.0]),
    # acc_limits=np.array([6.0, 4.0, 4.0, 8.0]),
    ompl_solve_time=5.0,
    ik_position_tol=0.05,   # 放宽到 5cm（很宽松）
    ik_n_restarts=30,        # 增加到 30 次
)

# Keep track of trajectory start time
_traj_start_time: float = None
_planning_lock = threading.Lock()
_tool_zero_quat = None  # 将在初始化时设置

# 轨迹可视化
_trajectory_waypoints = []  # 存储规划的轨迹路点
_trajectory_lock = threading.Lock()


def _trigger_plan(target_pose_7d: np.ndarray) -> None:
    """Run full planning pipeline in a background thread, then reset timer."""
    global _traj_start_time, _trajectory_waypoints
    q_current = physics.bind(arm.joints).qpos.copy()
    print(f"[Demo] Planning to target: {np.round(target_pose_7d, 3)}")
    success = motion_planner.request(target_pose_7d, q_current)
    if success:
        with _planning_lock:
            _traj_start_time = time.time()

        # 提取轨迹路点用于可视化
        duration = motion_planner.duration
        num_waypoints = max(50, int(duration * 100))  # 至少 50 个点
        time_samples = np.linspace(0, duration, num_waypoints)

        waypoints = []
        for t in time_samples:
            eef_pose = motion_planner.query_eef_pose(t)
            waypoints.append(eef_pose[:3].copy())  # 只取 XYZ 位置

        with _trajectory_lock:
            _trajectory_waypoints = waypoints

        print(f"[Demo] Trajectory extracted: {len(waypoints)} waypoints")
    else:
        print("[Demo] Planning failed — keeping current position.")
        with _trajectory_lock:
            _trajectory_waypoints = []


def _pose_to_target_array(pose) -> np.ndarray:
    """Convert ROS2 Pose message to target pose with j4 rotation.
    Uses roll angle from Euler to determine j4 rotation.
    """
    pos = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)
    quat = np.array(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
        dtype=np.float64,
    )
    norm = np.linalg.norm(quat)
    quat = quat / norm if norm > 1e-6 else np.array([0.0, 0.0, 0.0, 1.0])

    # 从四元数提取 roll 角作为 j4 旋转角度
    j4_angle = float(mat2euler(quat2mat(quat))[0])

    # 根据 j4 角度计算正确的目标四元数
    target_quat = _goal_quat_from_j4(_tool_zero_quat, j4_angle)

    return np.concatenate([pos, target_quat])


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------
class TargetPoseNode(Node):
    def __init__(self, topic: str = "/rc_arm_2/target_pose") -> None:
        super().__init__("rc_arm_2_ompl_target_listener")
        self._pending_pose = None   # set by callback, consumed by main loop
        self.create_subscription(PoseStamped, topic, self._on_pose, 10)
        self._torque_pub = self.create_publisher(JointState, "/rc_arm_2/joint_torque", 10)
        self._torque_msg = JointState()
        self._torque_msg.name = ["j1", "j2", "j3", "j4"]

    def _on_pose(self, msg: PoseStamped) -> None:
        self._pending_pose = msg.pose

    def pop_pose(self):
        """Return and clear the latest received pose (or None)."""
        pose = self._pending_pose
        self._pending_pose = None
        return pose

    def publish_torque(self, torques):
        msg = self._torque_msg
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.effort = [float(t) for t in torques]
        self._torque_pub.publish(msg)


# ---------------------------------------------------------------------------
# Initial plan: move to a default target so the arm starts moving
# ---------------------------------------------------------------------------
# 获取当前末端姿态作为"工具零位姿"（参考姿态）
_current_eef = arm.get_eef_pose(physics)
_tool_zero_quat = _current_eef[3:].copy()
print(f"[Demo] Current EEF pose: {np.round(_current_eef, 3)}")

# 定义初始目标：保持当前姿态，在 XYZ 方向上移动
_initial_target = _current_eef.copy()
_initial_target[0] += 0.1   # x 前进 0.1m
_initial_target[1] -= 0.05  # y 后移 0.05m
# z 不改

_target_xyz = _initial_target[:3].copy()

print(f"[Demo] Initial target XYZ: {np.round(_target_xyz, 3)}")
_plan_thread = threading.Thread(target=_trigger_plan, args=(_initial_target,), daemon=True)
_plan_thread.start()

# ---------------------------------------------------------------------------
# 轨迹可视化函数
# ---------------------------------------------------------------------------
def _draw_trajectory(viewer):
    """在 MuJoCo viewer 中绘制规划的轨迹"""
    with _trajectory_lock:
        waypoints = _trajectory_waypoints.copy()

    if len(waypoints) < 2:
        return

    # 绘制轨迹线段
    for i in range(len(waypoints) - 1):
        p1 = waypoints[i]
        p2 = waypoints[i + 1]

        # 创建线段（使用 MuJoCo 的调试接口）
        # 色彩：从绿色到红色的梯度
        t = i / (len(waypoints) - 1)
        color = [t, 1.0 - t, 0.0, 1.0]  # 从绿→红

        try:
            mujoco.mjv_line(
                viewer.scn,
                [p1[0], p1[1], p1[2]],
                [p2[0], p2[1], p2[2]],
                color
            )
        except:
            pass

    # 绘制起点（绿色球体）
    try:
        mujoco.mjv_point(viewer.scn, [0.0, 1.0, 0.0, 1.0], waypoints[0], 0.015)
    except:
        pass

    # 绘制终点（红色球体）
    try:
        mujoco.mjv_point(viewer.scn, [1.0, 0.0, 0.0, 1.0], waypoints[-1], 0.015)
    except:
        pass


# ---------------------------------------------------------------------------
# Main simulation loop
# ---------------------------------------------------------------------------
rclpy.init()
node = TargetPoseNode()
sim_timestep = float(physics.timestep())
last_step_wall = time.time()

try:
    while rclpy.ok():
        # --- ROS2 spin (non-blocking) ---
        rclpy.spin_once(node, timeout_sec=0.0)

        # --- Check for new goal from ROS2 ---
        pose_msg = node.pop_pose()
        if pose_msg is not None:
            target_7d = _pose_to_target_array(pose_msg)
            # Trigger planning in background so sim loop isn't blocked
            t = threading.Thread(target=_trigger_plan, args=(target_7d,), daemon=True)
            t.start()

        # --- Query trajectory and update mocap target ---
        with _planning_lock:
            t_start = _traj_start_time

        if t_start is not None and motion_planner.is_active:
            t_elapsed = time.time() - t_start
            eef_pose = motion_planner.query_eef_pose(t_elapsed)
            env.unwrapped._target.set_mocap_pose(
                physics,
                position=eef_pose[:3],
                quaternion=eef_pose[3:],
            )

        # --- Step physics ---
        action = np.zeros(env.action_space.shape, dtype=np.float64)
        observation, reward, terminated, truncated, info = env.step(action)

        # --- Publish joint torques ---
        torques = physics.data.qfrc_applied[joint_dof_ids]
        node.publish_torque(torques)

        # --- Real-time rate limiting ---
        now = time.time()
        elapsed = now - last_step_wall
        if elapsed < sim_timestep:
            time.sleep(sim_timestep - elapsed)
            last_step_wall += sim_timestep
        else:
            last_step_wall = now

        if terminated or truncated:
            observation, info = env.reset()

finally:
    env.close()
    node.destroy_node()
    rclpy.shutdown()
