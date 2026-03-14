import os
import sys
import time
import numpy as np
import gymnasium

# 将项目根路径加入 sys.path，便于在未通过 pip 安装包时直接以源码方式导入
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

import rc_robotarm_mujoco  # noqa: F401
from rc_robotarm_mujoco.utils.transform_utils import (
    quat2mat,
    mat2euler,
    quat_multiply,
    axisangle2quat,
)

MAX_LINEAR_SPEED = 0.8  # m/s, limit target motion to improve stability
TOOL_AXIS_LOCAL = np.array([0.0, 0.0, 1.0], dtype=np.float64)

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
except Exception as exc:  # pragma: no cover
    raise SystemExit(
        "未找到 ROS2 依赖。请先确保 rclpy 与 geometry_msgs 已安装并可用。"
    ) from exc


class TargetPoseNode(Node):
    def __init__(self, topic: str = "/rc_arm_2/target_pose") -> None:
        super().__init__("rc_arm_2_target_pose_listener")
        self._latest_pose = None
        self.create_subscription(PoseStamped, topic, self._on_pose, 10)

    def _on_pose(self, msg: PoseStamped) -> None:
        # 使用 PoseStamped 的 pose，坐标默认理解为仿真 world 坐标系
        self._latest_pose = msg.pose

    def get_latest_pose(self):
        return self._latest_pose


def _pose_to_arrays(pose):
    pos = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)
    quat = np.array(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
        dtype=np.float64,
    )

    # 处理非法四元数（全 0）并归一化
    norm = np.linalg.norm(quat)
    if norm < 1e-6:
        quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    else:
        quat /= norm

    return pos, quat


def _tool_axis_goal_quat(target_quat: np.ndarray, tool_zero_quat: np.ndarray) -> np.ndarray:
    """
    将目标姿态的 roll 解释为“工具轴旋转角”，并生成对应目标四元数。
    """
    # 使用目标四元数的 roll 作为期望角度
    target_rpy = mat2euler(quat2mat(target_quat))
    target_angle = target_rpy[0]

    # 工具轴在世界坐标系中的方向（由零位姿确定）
    axis_world = quat2mat(tool_zero_quat) @ TOOL_AXIS_LOCAL
    axis_world = axis_world / (np.linalg.norm(axis_world) + 1e-8)

    # 绕工具轴旋转目标角度
    q_axis = axisangle2quat(axis_world * target_angle)
    goal_quat = quat_multiply(q_axis, tool_zero_quat)
    goal_quat = goal_quat / (np.linalg.norm(goal_quat) + 1e-8)
    return goal_quat.astype(np.float64)


def _tool_axis_angle(tool_zero_quat: np.ndarray, quat_xyzw: np.ndarray) -> float:
    """
    计算当前姿态相对工具零位姿的“工具轴旋转角”（绕工具 Z）。
    """
    r_rel = quat2mat(tool_zero_quat).T @ quat2mat(quat_xyzw)
    return float(mat2euler(r_rel)[2])


def _unwrap_to_near(angle: float, reference: float) -> float:
    """
    将 angle 按 2π 展开到最接近 reference 的等价角。
    """
    two_pi = 2.0 * np.pi
    return angle + two_pi * np.round((reference - angle) / two_pi)


def main() -> None:
    rclpy.init()
    node = TargetPoseNode()

    # human 渲染模式会打开可视化窗口
    env = gymnasium.make("rc_robotarm_mujoco/RC_ARM_2Env-v0", render_mode="human")
    observation, info = env.reset(seed=42)

    # 初始化目标为当前末端位置，避免第一次跳变
    eef_pose = env.unwrapped._arm.get_eef_pose(env.unwrapped._physics)
    current_pos = eef_pose[:3].copy()
    current_quat = eef_pose[3:].copy()
    tool_zero_quat = current_quat.copy()
    goal_pos = current_pos.copy()
    goal_quat = current_quat.copy()
    last_print_time = time.time()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)

            pose = node.get_latest_pose()
            if pose is not None:
                pos, quat = _pose_to_arrays(pose)
                goal_pos = pos
                # 只跟踪“工具轴旋转角”（从目标姿态的 roll 得到）
                # 把目标角展开到最接近当前工具角，避免在 +/-pi 跳变
                current_tool_angle = _tool_axis_angle(tool_zero_quat, current_quat)
                target_roll = mat2euler(quat2mat(quat))[0]
                target_roll = _unwrap_to_near(target_roll, current_tool_angle)
                # 用展开后的角度生成目标四元数
                axis_world = quat2mat(tool_zero_quat) @ TOOL_AXIS_LOCAL
                axis_world = axis_world / (np.linalg.norm(axis_world) + 1e-8)
                q_axis = axisangle2quat(axis_world * target_roll)
                goal_quat = quat_multiply(q_axis, tool_zero_quat)
                goal_quat = goal_quat / (np.linalg.norm(goal_quat) + 1e-8)

            # 限速移动目标，避免大跳变导致数值爆炸
            max_step = MAX_LINEAR_SPEED * env.unwrapped._physics.model.opt.timestep
            delta = goal_pos - current_pos
            dist = np.linalg.norm(delta)
            if dist > max_step:
                current_pos = current_pos + delta / dist * max_step
            else:
                current_pos = goal_pos.copy()
            # 只跟踪末端工具轴旋转（控制器内部会投影到工具轴）
            current_quat = goal_quat

            env.unwrapped._target.set_mocap_pose(
                env.unwrapped._physics,
                position=current_pos,
                quaternion=current_quat,
            )

            # 动作当前未用于控制，这里用 0 向量即可
            observation, reward, terminated, truncated, info = env.step(
                np.zeros(env.action_space.shape, dtype=np.float64)
            )

            # 每 0.1s 打印一次末端坐标
            now = time.time()
            if now - last_print_time >= 0.1:
                eef_pose = env.unwrapped._arm.get_eef_pose(env.unwrapped._physics)
                pos = eef_pose[:3]
                jpos = env.unwrapped._physics.bind(env.unwrapped._arm.joints).qpos
                # 末端实际 rpy 与目标 rpy（仅作观察）
                ee_quat = eef_pose[3:]
                ee_rpy = mat2euler(quat2mat(ee_quat))
                target_rpy = mat2euler(quat2mat(current_quat))
                tool_angle = _tool_axis_angle(tool_zero_quat, ee_quat)
                tool_target_angle = _tool_axis_angle(tool_zero_quat, current_quat)
                print(
                    "EEF pos: x={:.3f}, y={:.3f}, z={:.3f} | j4={:.3f} | rpy=({:.3f},{:.3f},{:.3f}) -> ({:.3f},{:.3f},{:.3f}) | tool={:.3f}->{:.3f}".format(
                        pos[0],
                        pos[1],
                        pos[2],
                        jpos[-1],
                        ee_rpy[0],
                        ee_rpy[1],
                        ee_rpy[2],
                        target_rpy[0],
                        target_rpy[1],
                        target_rpy[2],
                        tool_angle,
                        tool_target_angle,
                    )
                )
                last_print_time = now

            if terminated or truncated:
                observation, info = env.reset()

    except KeyboardInterrupt:
        pass
    finally:
        env.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
