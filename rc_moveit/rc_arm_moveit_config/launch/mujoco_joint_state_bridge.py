#!/usr/bin/env python3
"""Bridge final joint motor-command packets from ROS2 to MuJoCo torque control."""

import argparse
import sys
import threading
import time
from pathlib import Path

import numpy as np


def _ensure_project_importable() -> None:
    try:
        import rc_robotarm_mujoco  # noqa: F401
        return
    except Exception:
        pass

    this_file = Path(__file__).resolve()
    candidates = [Path.cwd()] + list(this_file.parents)
    for base in candidates:
        pkg_dir = base / "rc_robotarm_mujoco"
        if (pkg_dir / "__init__.py").exists():
            if str(base) not in sys.path:
                sys.path.insert(0, str(base))
            return


_ensure_project_importable()

import gymnasium  # noqa: E402
import rc_robotarm_mujoco  # noqa: E402,F401
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

ROS_JOINT_ORDER = ["j1_joint", "j2_joint", "j3_joint", "j4_joint"]
SHORT_JOINT_ORDER = ["j1", "j2", "j3", "j4"]
HARD_TORQUE_LIMITS = np.array([14.0, 14.0, 14.0, 6.0], dtype=np.float64)


class MujocoMotorCommandBridge(Node):
    def __init__(
        self,
        joint_command_topic: str,
        pd_gains_topic: str,
        torque_ff_topic: str,
        torque_input_topic: str,
        torque_output_topic: str,
    ) -> None:
        super().__init__("mujoco_motor_command_bridge")

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

        self.create_subscription(
            JointState,
            joint_command_topic,
            self._on_joint_command,
            20,
        )
        self.create_subscription(
            JointState,
            pd_gains_topic,
            self._on_pd_gains,
            20,
        )

        if torque_ff_topic:
            self.create_subscription(
                JointState,
                torque_ff_topic,
                self._on_torque_ff,
                20,
            )

        if torque_input_topic:
            self.create_subscription(
                JointState,
                torque_input_topic,
                self._on_torque_input,
                20,
            )

        self._torque_pub = self.create_publisher(JointState, torque_output_topic, 20)
        self._torque_msg = JointState()
        self._torque_msg.name = SHORT_JOINT_ORDER

    @staticmethod
    def _joint_index_map(msg: JointState):
        return {name: i for i, name in enumerate(msg.name)}

    def _extract_array(self, msg: JointState, values) -> np.ndarray:
        name_to_index = self._joint_index_map(msg)
        out = np.zeros(4, dtype=np.float64)

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
        if not msg.name:
            return

        q_cmd = self._extract_array(msg, msg.position)
        qd_cmd = self._extract_array(msg, msg.velocity)

        with self._lock:
            self._q_cmd = q_cmd
            self._qd_cmd = qd_cmd
            self._have_command = True

    def _on_pd_gains(self, msg: JointState) -> None:
        if not msg.name:
            return

        kp = self._extract_array(msg, msg.position)
        kd = self._extract_array(msg, msg.velocity)

        with self._lock:
            self._kp = kp
            self._kd = kd
            self._have_pd = True

    def _on_torque_ff(self, msg: JointState) -> None:
        if not msg.name or not msg.effort:
            return

        tau_ff = self._extract_array(msg, msg.effort)
        with self._lock:
            self._tau_ff = tau_ff
            self._have_torque_ff = True

    def _on_torque_input(self, msg: JointState) -> None:
        if not msg.name or not msg.effort:
            return

        tau = self._extract_array(msg, msg.effort)
        with self._lock:
            self._tau_input = tau
            self._have_torque_input = True

    def get_control_inputs(self):
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

    def publish_torque(self, tau: np.ndarray) -> None:
        self._torque_msg.header.stamp = self.get_clock().now().to_msg()
        self._torque_msg.effort = [float(v) for v in tau]
        self._torque_pub.publish(self._torque_msg)


def parse_args():
    parser = argparse.ArgumentParser(description="ROS2 final motor command to MuJoCo torque bridge")
    parser.add_argument("--joint-command-topic", default="/debug/final_joint_command_joint_frame")
    parser.add_argument("--pd-gains-topic", default="/debug/final_pd_gains")
    parser.add_argument("--torque-ff-topic", default="/debug/final_joint_torque_ff")
    parser.add_argument("--torque-input-topic", default="")
    parser.add_argument("--torque-output-topic", default="/rc_arm_2/joint_torque")
    parser.add_argument("--torque-input-scale", type=float, default=1.0)
    parser.add_argument("--kp-scale", type=float, default=1.0)
    parser.add_argument("--kd-scale", type=float, default=1.0)
    parser.add_argument("--ff-scale", type=float, default=1.0)
    parser.add_argument("--torque-limit", type=float, default=20.0)
    parser.add_argument("--rate", type=float, default=200.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    rclpy.init()
    node = MujocoMotorCommandBridge(
        joint_command_topic=args.joint_command_topic,
        pd_gains_topic=args.pd_gains_topic,
        torque_ff_topic=args.torque_ff_topic,
        torque_input_topic=args.torque_input_topic,
        torque_output_topic=args.torque_output_topic,
    )

    orientation_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    env = gymnasium.make(
        "rc_robotarm_mujoco/RC_ARM_2Env-v0",
        render_mode="human",
        orientation_axis=orientation_axis,
    )
    env.reset(seed=42)

    physics = env.unwrapped._physics
    arm = env.unwrapped._arm
    joint_binding = physics.bind(arm.joints)

    dt = 1.0 / max(args.rate, 1.0)
    next_tick = time.time()
    logged_ready = False
    logged_non_finite = False

    effective_limit = HARD_TORQUE_LIMITS.copy()
    if args.torque_limit > 0.0:
        effective_limit = np.minimum(effective_limit, float(args.torque_limit))

    node.get_logger().info(
        f"Torque safety clamp active: global={args.torque_limit:.3f} Nm, per-joint={effective_limit.tolist()}"
    )
    node.get_logger().info(
        f"MuJoCo torque scales: kp={args.kp_scale:.3f}, kd={args.kd_scale:.3f}, ff={args.ff_scale:.3f}, ext={args.torque_input_scale:.3f}"
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
            ) = node.get_control_inputs()

            q = joint_binding.qpos.copy()
            qd = joint_binding.qvel.copy()
            tau = np.zeros_like(q)

            if have_command:
                kp_eff = args.kp_scale * kp
                kd_eff = args.kd_scale * kd
                tau_ff_eff = args.ff_scale * tau_ff if have_torque_ff else np.zeros_like(q)

                if have_pd:
                    tau = kp_eff * (q_cmd - q) + kd_eff * (qd_cmd - qd) + tau_ff_eff
                else:
                    tau = tau_ff_eff.copy()

                if not logged_ready:
                    node.get_logger().info(
                        "Received final command packets. MuJoCo motor-equivalent control enabled."
                    )
                    logged_ready = True

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

            if env.unwrapped._render_mode == "human":
                env.unwrapped._render_frame()

            node.publish_torque(tau)

            next_tick += dt
            sleep_time = next_tick - time.time()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_tick = time.time()

    except KeyboardInterrupt:
        pass
    finally:
        env.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
