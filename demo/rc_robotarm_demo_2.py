import sys
import os
import time
import gymnasium
import mujoco

# 将项目根路径加入 sys.path，便于在未通过 pip 安装包时直接以源码方式导入
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

import rc_robotarm_mujoco
import numpy as np
from rc_robotarm_mujoco.utils.transform_utils import (
    axisangle2quat,
    quat_multiply,
    quat2mat,
    mat2euler,
)

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
except Exception as exc:  # pragma: no cover
    raise SystemExit(
        "未找到 ROS2 依赖。请先确保 rclpy、geometry_msgs、sensor_msgs 已安装并可用。"
    ) from exc



# 以 human 模式创建并渲染环境
# 4 自由度机械臂适合跟踪 4 维任务：这里配置为 XYZ + 末端局部 Z 轴旋转（对应 j4）。
# orientation_axis 使用末端工具坐标系下的轴，不是世界坐标系。
orientation_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
env = gymnasium.make(
    'rc_robotarm_mujoco/RC_ARM_2Env-v0',
    render_mode='human',
    orientation_axis=orientation_axis,
)

# 使用指定种子重置环境以便结果可复现
observation, info = env.reset(seed=42)
joint_dof_ids = env.unwrapped._physics.bind(env.unwrapped._arm.joints).dofadr

# 目标位置（XYZ）和 j4 角度（弧度）
target_xyz = np.array([-0.2, 1.8, 0.6], dtype=np.float64)
target_j4 = 0.0  # radians

# 以当前末端姿态作为“工具零位姿”
eef_pose = env.unwrapped._arm.get_eef_pose(env.unwrapped._physics)
tool_zero_quat = eef_pose[3:].copy()


def _goal_quat_from_j4(tool_zero_quat: np.ndarray, j4_angle: float) -> np.ndarray:
    axis_world = quat2mat(tool_zero_quat) @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    axis_world = axis_world / (np.linalg.norm(axis_world) + 1e-8)
    q_axis = axisangle2quat(axis_world * j4_angle)
    goal_quat = quat_multiply(q_axis, tool_zero_quat)
    return goal_quat / (np.linalg.norm(goal_quat) + 1e-8)


class TargetPoseNode(Node):
    def __init__(self, topic: str = "/rc_arm_2/target_pose") -> None:
        super().__init__("rc_arm_2_target_pose_listener")
        self._latest_pose = None
        self.create_subscription(PoseStamped, topic, self._on_pose, 10)
        self._torque_pub = self.create_publisher(JointState, "/rc_arm_2/joint_torque", 10)
        self._torque_msg = JointState()
        self._torque_msg.name = ["j1", "j2", "j3", "j4"]

    def _on_pose(self, msg: PoseStamped) -> None:
        self._latest_pose = msg.pose

    def get_latest_pose(self):
        return self._latest_pose

    def publish_torque(self, torques):
        msg = self._torque_msg
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.effort = [float(t) for t in torques]
        self._torque_pub.publish(msg)


def _pose_to_arrays(pose):
    pos = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)
    quat = np.array(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
        dtype=np.float64,
    )
    norm = np.linalg.norm(quat)
    if norm < 1e-6:
        quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    else:
        quat /= norm
    return pos, quat


def _find_geom_id_by_suffix(model, suffix: str) -> int:
    for i in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
        if name and name.endswith(suffix):
            return i
    return -1


def _collect_kfs_geom_ids(model):
    ids = []
    for gid in range(model.ngeom):
        body_id = model.geom_bodyid[gid]
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
        short_name = body_name.split("/")[-1]
        if short_name.startswith("kfs_"):
            ids.append((gid, short_name))
    return ids


def _attachment_hits_kfs(physics):
    model = physics.model.ptr
    data = physics.data.ptr
    att_id = _find_geom_id_by_suffix(model, "attachment_geom")
    if att_id < 0:
        return False, []

    if not hasattr(_attachment_hits_kfs, "_kfs_geom_ids"):
        _attachment_hits_kfs._kfs_geom_ids = _collect_kfs_geom_ids(model)

    att_pos = data.geom_xpos[att_id]
    att_r = model.geom_rbound[att_id]

    hits = set()
    for gid, short_name in _attachment_hits_kfs._kfs_geom_ids:
        other_pos = data.geom_xpos[gid]
        other_r = model.geom_rbound[gid]
        if np.linalg.norm(att_pos - other_pos) <= (att_r + other_r-0.23):
            hits.add(short_name)

    return len(hits) > 0, sorted(hits)


rclpy.init()
node = TargetPoseNode()
sim_timestep = float(env.unwrapped._physics.timestep())
last_step_time = time.time()

try:
    # 运行仿真（示例为无限循环），也可以使用固定步数：
    # for _ in range(1000):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)

        pose = node.get_latest_pose()
        if pose is not None:
            target_xyz, quat = _pose_to_arrays(pose)
            # 只使用 roll 作为 j4 角度
            target_j4 = float(mat2euler(quat2mat(quat))[0])

        # 固定动作：不移动
        action = np.zeros(env.action_space.shape, dtype=np.float64)

        # 用用户给定的 XYZ + j4 角度更新目标
        goal_quat = _goal_quat_from_j4(tool_zero_quat, target_j4)
        env.unwrapped._target.set_mocap_pose(
            env.unwrapped._physics,
            position=target_xyz,
            quaternion=goal_quat,
        )

        # 使用选定动作在环境中执行一步
        observation, reward, terminated, truncated, info = env.step(action)

        # 发布四个关节的力矩（与控制频率一致）
        torques = env.unwrapped._physics.data.qfrc_applied[joint_dof_ids]
        node.publish_torque(torques)

        # 限速到现实时间：1 秒现实 = 1 秒仿真
        now = time.time()
        elapsed = now - last_step_time
        if elapsed < sim_timestep:
            time.sleep(sim_timestep - elapsed)
            last_step_time = last_step_time + sim_timestep
        else:
            last_step_time = now

        touching, bodies = _attachment_hits_kfs(env.unwrapped._physics)
        if touching:
            print("attachment_site 触碰到 kfs:", bodies)
        else:
            print("attachment_site 没有触碰到任何 kfs。")   
        # 检查回合是否结束（terminated）或被截断（truncated）
        if terminated or truncated:
            # 回合结束或被截断时重置环境
            observation, info = env.reset()
finally:
    # 仿真结束后关闭环境
    env.close()
    node.destroy_node()
    rclpy.shutdown()
