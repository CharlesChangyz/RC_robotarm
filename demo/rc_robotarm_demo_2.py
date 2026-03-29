#!/usr/bin/env python3
import argparse
import os
import sys
import threading
import time

import gymnasium
import mujoco
import numpy as np

# 将项目根路径加入 sys.path，便于在未通过 pip 安装包时直接以源码方式导入
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

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
except Exception as exc:  # pragma: no cover
    raise SystemExit(
        "未找到 ROS2 依赖。请先确保 rclpy、geometry_msgs、sensor_msgs 已安装并可用。"
    ) from exc


ROS_JOINT_ORDER = ["j1_joint", "j2_joint", "j3_joint", "j4_joint"]
SHORT_JOINT_ORDER = ["j1", "j2", "j3", "j4"]
HARD_TORQUE_LIMITS = np.array([14.0, 14.0, 14.0, 6.0], dtype=np.float64)


# 以 human 模式创建并渲染环境
# 4 自由度机械臂适合跟踪 4 维任务：这里配置为 XYZ + 末端局部 Z 轴旋转（对应 j4）。
# orientation_axis 使用末端工具坐标系下的轴，不是世界坐标系。
orientation_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
env = gymnasium.make(
    "rc_robotarm_mujoco/RC_ARM_2Env-v0",
    render_mode="human",
    orientation_axis=orientation_axis,
)

# 使用指定种子重置环境以便结果可复现
observation, info = env.reset(seed=42)
joint_binding = env.unwrapped._physics.bind(env.unwrapped._arm.joints)
joint_dof_ids = joint_binding.dofadr

# 以当前末端姿态作为“工具零位姿”
eef_pose = env.unwrapped._arm.get_eef_pose(env.unwrapped._physics)
tool_zero_quat = eef_pose[3:].copy()

# 目标位置（XYZ）和 j4 角度（弧度）
# 默认初始化为当前末端位置，避免在未提供目标时自动漂移/抬升。
target_xyz = eef_pose[:3].copy()
target_j4 = 0.0  # radians


def _goal_quat_from_j4(tool_zero_quat: np.ndarray, j4_angle: float) -> np.ndarray:
    axis_world = quat2mat(tool_zero_quat) @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    axis_world = axis_world / (np.linalg.norm(axis_world) + 1e-8)
    q_axis = axisangle2quat(axis_world * j4_angle)
    goal_quat = quat_multiply(q_axis, tool_zero_quat)
    return goal_quat / (np.linalg.norm(goal_quat) + 1e-8)


class SimBridgeNode(Node):
    def __init__(
        self,
        target_pose_topic: str,
        joint_command_topic: str,
        pd_gains_topic: str,
        torque_ff_topic: str,
        torque_input_topic: str,
        torque_output_topic: str,
        joint_state_topic: str,
        joint_position_topic: str,
        eef_pose_topic: str,
        eef_frame_id: str,
    ) -> None:
        super().__init__("rc_arm_2_sim_bridge")

        self._lock = threading.Lock()
        self._latest_pose = None

        self._q_cmd = np.zeros(4, dtype=np.float64)
        self._qd_cmd = np.zeros(4, dtype=np.float64)
        self._tau_ff = np.zeros(4, dtype=np.float64)
        self._kp = np.zeros(4, dtype=np.float64)
        self._kd = np.zeros(4, dtype=np.float64)
        self._tau_input = np.zeros(4, dtype=np.float64)

        self._have_command = False
        self._have_pd = False
        self._have_torque_ff = False
        self._have_torque_input = False

        if target_pose_topic:
            self.create_subscription(PoseStamped, target_pose_topic, self._on_pose, 20)

        if joint_command_topic:
            self.create_subscription(JointState, joint_command_topic, self._on_joint_command, 20)

        if pd_gains_topic:
            self.create_subscription(JointState, pd_gains_topic, self._on_pd_gains, 20)

        if torque_ff_topic:
            self.create_subscription(JointState, torque_ff_topic, self._on_torque_ff, 20)

        if torque_input_topic:
            self.create_subscription(JointState, torque_input_topic, self._on_torque_input, 20)

        self._torque_pub = self.create_publisher(JointState, torque_output_topic, 20)
        self._torque_msg = JointState()
        self._torque_msg.name = SHORT_JOINT_ORDER

        self._joint_state_pub = self.create_publisher(JointState, joint_state_topic, 20)
        self._joint_position_pub = self.create_publisher(JointState, joint_position_topic, 20)
        self._eef_pose_pub = self.create_publisher(PoseStamped, eef_pose_topic, 20)
        self._eef_frame_id = eef_frame_id

        self._joint_state_msg = JointState()
        self._joint_state_msg.name = ROS_JOINT_ORDER

        self._joint_position_msg = JointState()
        self._joint_position_msg.name = ROS_JOINT_ORDER

    def _on_pose(self, msg: PoseStamped) -> None:
        self._latest_pose = msg.pose

    @staticmethod
    def _index_map(names):
        return {name: i for i, name in enumerate(names)}

    def _extract_joint_array(self, msg: JointState, values) -> np.ndarray:
        out = np.zeros(4, dtype=np.float64)
        if not msg.name:
            return out

        name_to_index = self._index_map(msg.name)
        for i, (ros_name, short_name) in enumerate(zip(ROS_JOINT_ORDER, SHORT_JOINT_ORDER)):
            idx = None
            if ros_name in name_to_index:
                idx = name_to_index[ros_name]
            elif short_name in name_to_index:
                idx = name_to_index[short_name]

            if idx is not None and idx < len(values):
                out[i] = float(values[idx])

        return out

    def _on_joint_command(self, msg: JointState) -> None:
        q_cmd = self._extract_joint_array(msg, msg.position)
        qd_cmd = self._extract_joint_array(msg, msg.velocity)

        with self._lock:
            self._q_cmd = q_cmd
            self._qd_cmd = qd_cmd
            self._have_command = True

    def _on_pd_gains(self, msg: JointState) -> None:
        kp = self._extract_joint_array(msg, msg.position)
        kd = self._extract_joint_array(msg, msg.velocity)

        with self._lock:
            self._kp = kp
            self._kd = kd
            self._have_pd = True

    def _on_torque_ff(self, msg: JointState) -> None:
        if not msg.effort:
            return

        tau_ff = self._extract_joint_array(msg, msg.effort)
        with self._lock:
            self._tau_ff = tau_ff
            self._have_torque_ff = True

    def _on_torque_input(self, msg: JointState) -> None:
        if not msg.effort:
            return

        tau = self._extract_joint_array(msg, msg.effort)
        with self._lock:
            self._tau_input = tau
            self._have_torque_input = True

    def get_latest_pose(self):
        return self._latest_pose

    def get_mit_inputs(self):
        with self._lock:
            return (
                self._have_command,
                self._q_cmd.copy(),
                self._qd_cmd.copy(),
                self._have_pd,
                self._kp.copy(),
                self._kd.copy(),
                self._have_torque_ff,
                self._tau_ff.copy(),
                self._have_torque_input,
                self._tau_input.copy(),
            )

    def publish_torque(self, torques):
        msg = self._torque_msg
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.effort = [float(t) for t in torques]
        self._torque_pub.publish(msg)

    def publish_sim_state(self, q, qd, tau, eef_pose):
        stamp = self.get_clock().now().to_msg()

        joint_msg = self._joint_state_msg
        joint_msg.header.stamp = stamp
        joint_msg.position = [float(v) for v in q[:4]]
        joint_msg.velocity = [float(v) for v in qd[:4]]
        joint_msg.effort = [float(v) for v in tau[:4]]
        self._joint_state_pub.publish(joint_msg)

        pos_msg = self._joint_position_msg
        pos_msg.header.stamp = stamp
        pos_msg.position = [float(v) for v in q[:4]]
        pos_msg.velocity = []
        pos_msg.effort = []
        self._joint_position_pub.publish(pos_msg)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self._eef_frame_id
        pose_msg.pose.position.x = float(eef_pose[0])
        pose_msg.pose.position.y = float(eef_pose[1])
        pose_msg.pose.position.z = float(eef_pose[2])
        pose_msg.pose.orientation.x = float(eef_pose[3])
        pose_msg.pose.orientation.y = float(eef_pose[4])
        pose_msg.pose.orientation.z = float(eef_pose[5])
        pose_msg.pose.orientation.w = float(eef_pose[6])
        self._eef_pose_pub.publish(pose_msg)


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
        if np.linalg.norm(att_pos - other_pos) <= (att_r + other_r - 0.23):
            hits.add(short_name)

    return len(hits) > 0, sorted(hits)


def parse_args():
    parser = argparse.ArgumentParser(description="RC Arm MuJoCo bridge demo")
    parser.add_argument(
        "--mode",
        choices=["auto", "target_pose", "mit"],
        default="mit",
        help="控制模式：mit=默认；auto=有 MIT 指令走 MIT，无 MIT 但收到 target_pose 时走 target_pose",
    )
    parser.add_argument("--target-pose-topic", default="/rc_arm_2/target_pose")
    parser.add_argument("--joint-command-topic", default="/debug/final_joint_command_joint_frame")
    parser.add_argument("--pd-gains-topic", default="/debug/final_pd_gains")
    parser.add_argument("--torque-ff-topic", default="/debug/final_joint_torque_ff")
    parser.add_argument("--torque-input-topic", default="")
    parser.add_argument("--torque-output-topic", default="/rc_arm_2/joint_torque")
    parser.add_argument("--torque-input-scale", type=float, default=1.0)
    parser.add_argument("--torque-limit", type=float, default=20.0)
    parser.add_argument("--rate", type=float, default=400.0)
    parser.add_argument("--joint-state-topic", default="/rc_arm_2/mujoco_joint_states")
    parser.add_argument("--joint-position-topic", default="/rc_arm_2/mujoco_joint_positions")
    parser.add_argument("--eef-pose-topic", default="/rc_arm_2/mujoco_eef_pose")
    parser.add_argument("--eef-frame-id", default="world")
    parser.add_argument("--collision-print-interval", type=int, default=20)
    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init()
    node = SimBridgeNode(
        target_pose_topic=args.target_pose_topic,
        joint_command_topic=args.joint_command_topic,
        pd_gains_topic=args.pd_gains_topic,
        torque_ff_topic=args.torque_ff_topic,
        torque_input_topic=args.torque_input_topic,
        torque_output_topic=args.torque_output_topic,
        joint_state_topic=args.joint_state_topic,
        joint_position_topic=args.joint_position_topic,
        eef_pose_topic=args.eef_pose_topic,
        eef_frame_id=args.eef_frame_id,
    )

    sim_timestep = float(env.unwrapped._physics.timestep())
    last_step_time = time.time()

    logged_target_pose_mode = False
    logged_mit_mode = False
    step_counter = 0
    logged_non_finite = False

    effective_limit = HARD_TORQUE_LIMITS.copy()
    if args.torque_limit > 0.0:
        effective_limit = np.minimum(effective_limit, float(args.torque_limit))

    node.get_logger().info(
        f"Torque safety clamp active: global={args.torque_limit:.3f} Nm, per-joint={effective_limit.tolist()}"
    )

    global target_xyz, target_j4

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)

            (
                have_command,
                q_cmd,
                qd_cmd,
                have_pd,
                kp,
                kd,
                have_torque_ff,
                tau_ff,
                have_torque_input,
                tau_input,
            ) = node.get_mit_inputs()

            if args.mode == "mit":
                use_mit_mode = True
            elif args.mode == "target_pose":
                use_mit_mode = False
            else:
                # auto: 优先 MIT；仅在未收到 MIT 指令且已收到 target_pose 时才进入 target_pose 分支
                use_mit_mode = have_command or (node.get_latest_pose() is None)

            if use_mit_mode:
                if not logged_mit_mode:
                    node.get_logger().info(
                        "切换到 MIT 命令模式：tau = kp*(q_cmd-q) + kd*(qd_cmd-qd) + tau_ff"
                    )
                    logged_mit_mode = True

                q = joint_binding.qpos.copy()
                qd = joint_binding.qvel.copy()

                if have_command:
                    tau = tau_ff.copy() if have_torque_ff else np.zeros(4, dtype=np.float64)
                    if have_pd:
                        tau = kp * (q_cmd - q) + kd * (qd_cmd - qd) + tau
                else:
                    tau = np.zeros(4, dtype=np.float64)

                if have_torque_input:
                    tau += args.torque_input_scale * tau_input

                if not np.all(np.isfinite(tau)):
                    if not logged_non_finite:
                        node.get_logger().warn("Non-finite torque detected. Forcing zero torque output.")
                        logged_non_finite = True
                    tau = np.zeros_like(tau)
                else:
                    logged_non_finite = False

                tau = np.clip(tau, -effective_limit, effective_limit)

                joint_binding.qfrc_applied = tau
                env.unwrapped._physics.step()

                if env.unwrapped._render_mode == "human":
                    env.unwrapped._render_frame()

                node.publish_torque(tau)
                q_after = joint_binding.qpos.copy()
                qd_after = joint_binding.qvel.copy()
                eef_pose_after = env.unwrapped._arm.get_eef_pose(env.unwrapped._physics)
                node.publish_sim_state(q_after, qd_after, tau, eef_pose_after)
                current_dt = 1.0 / max(args.rate, 1.0)
            else:
                if not logged_target_pose_mode:
                    node.get_logger().info("使用 target_pose + OSC 模式")
                    logged_target_pose_mode = True

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

                observation, reward, terminated, truncated, info = env.step(action)

                torques = env.unwrapped._physics.data.qfrc_applied[joint_dof_ids].copy()
                node.publish_torque(torques)
                q_after = joint_binding.qpos.copy()
                qd_after = joint_binding.qvel.copy()
                eef_pose_after = env.unwrapped._arm.get_eef_pose(env.unwrapped._physics)
                node.publish_sim_state(q_after, qd_after, torques, eef_pose_after)
                current_dt = sim_timestep

                if terminated or truncated:
                    observation, info = env.reset()

            step_counter += 1
            if args.collision_print_interval > 0 and step_counter % args.collision_print_interval == 0:
                touching, bodies = _attachment_hits_kfs(env.unwrapped._physics)
                if touching:
                    print("attachment_site 触碰到 kfs:", bodies)

            # 限速到现实时间
            now = time.time()
            elapsed = now - last_step_time
            if elapsed < current_dt:
                time.sleep(current_dt - elapsed)
                last_step_time = last_step_time + current_dt
            else:
                last_step_time = now
    finally:
        env.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
