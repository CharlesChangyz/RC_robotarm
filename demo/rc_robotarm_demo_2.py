#!/usr/bin/env python3
import argparse
import os
import sys
import threading
import time
from pathlib import Path

from dm_control import mjcf
import mujoco
import mujoco.viewer
import numpy as np
import yaml

# 将项目根路径加入 sys.path，便于在未通过 pip 安装包时直接以源码方式导入
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from rc_robotarm_mujoco.arenas import StandardArena
from rc_robotarm_mujoco.robots import RCArm_2
from rc_robotarm_mujoco.utils.transform_utils import convert_quat, mat2quat

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
except Exception as exc:  # pragma: no cover
    raise SystemExit(
        "未找到 ROS2 依赖。请先确保 rclpy、geometry_msgs、sensor_msgs 已安装并可用。"
    ) from exc


ROS_JOINT_ORDER = ["j1_joint", "j2_joint", "j3_joint", "j4_joint"]
SHORT_JOINT_ORDER = ["j1", "j2", "j3", "j4"]
HARD_TORQUE_LIMITS = np.array([14.0, 14.0, 14.0, 6.0], dtype=np.float64)
ARM_BASE_POS = [0.0, 1.8, 0.61]
ARM_BASE_QUAT = [0.7071068, 0.0, 0.0, -0.7071068]
HOME_QPOS = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
DEFAULT_HARDWARE_CONFIG = str(
    Path(ROOT_DIR) / "rc_moveit" / "rc_arm_description" / "config" / "rc_arm_2" / "rc_arm_2_hardware.mujoco.yaml"
)


def _load_hardware_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"invalid hardware config: {path}")
    return data


def _cfg_xyz(cfg: dict, prefix: str, fallback) -> np.ndarray:
    return np.array(
        [
            float(cfg.get(f"{prefix}_x", fallback[0])),
            float(cfg.get(f"{prefix}_y", fallback[1])),
            float(cfg.get(f"{prefix}_z", fallback[2])),
        ],
        dtype=np.float64,
    )


class SimBridgeNode(Node):
    def __init__(
        self,
        joint_command_topic: str,
        pd_gains_topic: str,
        torque_ff_topic: str,
        torque_input_topic: str,
        torque_output_topic: str,
        joint_state_topic: str,
        joint_position_topic: str,
        eef_pose_topic: str,
        eef_frame_id: str,
        payload_active_topic: str,
    ) -> None:
        super().__init__("rc_arm_2_sim_bridge")

        self._lock = threading.Lock()

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
        self._payload_active = False

        if joint_command_topic:
            self.create_subscription(JointState, joint_command_topic, self._on_joint_command, 20)

        if pd_gains_topic:
            self.create_subscription(JointState, pd_gains_topic, self._on_pd_gains, 20)

        if torque_ff_topic:
            self.create_subscription(JointState, torque_ff_topic, self._on_torque_ff, 20)

        if torque_input_topic:
            self.create_subscription(JointState, torque_input_topic, self._on_torque_input, 20)

        self.create_subscription(Bool, payload_active_topic, self._on_payload_active, 10)

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

    def _on_payload_active(self, msg: Bool) -> None:
        with self._lock:
            self._payload_active = bool(msg.data)

    def get_payload_active(self) -> bool:
        with self._lock:
            return self._payload_active

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


def _create_sim(config: dict):
    arena = StandardArena()
    arm = RCArm_2()
    arena.attach(arm.mjcf_model, pos=ARM_BASE_POS, quat=ARM_BASE_QUAT)

    payload_body_name = str(config.get("mujoco_payload_body_name", "payload_block")).strip() or "payload_block"
    payload_size = _cfg_xyz(config, "payload_box_size", (0.05, 0.05, 0.05))
    payload_initial_pos = _cfg_xyz(config, "mujoco_payload_initial_pos", (0.30, 0.0, 0.20))
    payload_com = _cfg_xyz(config, "payload_com_offset", (0.0, 0.0, 0.0))
    payload_diaginertia = _cfg_xyz(config, "payload_diaginertia", (0.02, 0.02, 0.02))
    payload_mass = float(config.get("payload_mass", 0.63))

    payload_body = arena.mjcf_model.worldbody.add(
        "body",
        name=payload_body_name,
        pos=payload_initial_pos.tolist(),
    )
    payload_freejoint = payload_body.add("freejoint", name=f"{payload_body_name}_freejoint")
    payload_body.add(
        "inertial",
        pos=payload_com.tolist(),
        mass=payload_mass,
        diaginertia=payload_diaginertia.tolist(),
    )
    payload_body.add(
        "geom",
        name=f"{payload_body_name}_geom",
        type="box",
        size=(payload_size * 0.5).tolist(),
        rgba=[0.0, 0.55, 0.9, 1.0],
    )

    physics = mjcf.Physics.from_mjcf_model(arena.mjcf_model)

    with physics.reset_context():
        physics.bind(arm.joints).qpos = HOME_QPOS

    physics.forward()
    return arena, arm, physics, payload_freejoint


def _render_frame(viewer, physics):
    if viewer is None:
        viewer = mujoco.viewer.launch_passive(physics.model.ptr, physics.data.ptr)
    viewer.sync()
    return viewer


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
    parser = argparse.ArgumentParser(description="RC Arm MuJoCo MIT bridge demo")
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
    parser.add_argument("--hardware-config-file", default=DEFAULT_HARDWARE_CONFIG)
    return parser.parse_args()


def main():
    args = parse_args()
    hardware_config = _load_hardware_config(args.hardware_config_file)
    _, arm, physics, payload_freejoint = _create_sim(hardware_config)
    joint_binding = physics.bind(arm.joints)
    payload_binding = physics.bind(payload_freejoint)
    attachment_site_binding = physics.bind(arm._attachment_site)
    viewer = None
    render_disabled = False

    rclpy.init()
    node = SimBridgeNode(
        joint_command_topic=args.joint_command_topic,
        pd_gains_topic=args.pd_gains_topic,
        torque_ff_topic=args.torque_ff_topic,
        torque_input_topic=args.torque_input_topic,
        torque_output_topic=args.torque_output_topic,
        joint_state_topic=args.joint_state_topic,
        joint_position_topic=args.joint_position_topic,
        eef_pose_topic=args.eef_pose_topic,
        eef_frame_id=args.eef_frame_id,
        payload_active_topic=str(hardware_config.get("payload_active_topic", "/rc_arm_2/payload_active")),
    )

    last_step_time = time.time()
    step_counter = 0
    logged_non_finite = False

    effective_limit = HARD_TORQUE_LIMITS.copy()
    if args.torque_limit > 0.0:
        effective_limit = np.minimum(effective_limit, float(args.torque_limit))

    node.get_logger().info(
        "MIT simulation bridge started. Torque clamp: global=%.3f Nm, per-joint=%s, payload_config=%s"
        % (args.torque_limit, effective_limit.tolist(), args.hardware_config_file)
    )
    node.get_logger().info(
        "viewer env DISPLAY=%s WAYLAND_DISPLAY=%s XDG_SESSION_TYPE=%s"
        % (
            os.environ.get("DISPLAY", ""),
            os.environ.get("WAYLAND_DISPLAY", ""),
            os.environ.get("XDG_SESSION_TYPE", ""),
        )
    )

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
            physics.step()

            if node.get_payload_active():
                payload_qpos = np.concatenate(
                    [
                        attachment_site_binding.xpos.copy(),
                        convert_quat(mat2quat(attachment_site_binding.xmat.reshape(3, 3)), to="wxyz"),
                    ]
                )
                payload_binding.qpos[:] = payload_qpos
                payload_binding.qvel[:] = 0.0
                physics.forward()

            if not render_disabled:
                try:
                    viewer = _render_frame(viewer, physics)
                except Exception as exc:
                    node.get_logger().warn(
                        "MuJoCo viewer disabled after render error: %r DISPLAY=%s WAYLAND_DISPLAY=%s"
                        % (
                            exc,
                            os.environ.get("DISPLAY", ""),
                            os.environ.get("WAYLAND_DISPLAY", ""),
                        )
                    )
                    render_disabled = True

            node.publish_torque(tau)
            q_after = joint_binding.qpos.copy()
            qd_after = joint_binding.qvel.copy()
            eef_pose_after = arm.get_eef_pose(physics)
            node.publish_sim_state(q_after, qd_after, tau, eef_pose_after)

            step_counter += 1
            if args.collision_print_interval > 0 and step_counter % args.collision_print_interval == 0:
                touching, bodies = _attachment_hits_kfs(physics)
                if touching:
                    print("attachment_site 触碰到 kfs:", bodies)

            current_dt = 1.0 / max(args.rate, 1.0)
            now = time.time()
            elapsed = now - last_step_time
            if elapsed < current_dt:
                time.sleep(current_dt - elapsed)
                last_step_time = last_step_time + current_dt
            else:
                last_step_time = now
    finally:
        if viewer is not None:
            viewer.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
