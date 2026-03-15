"""
RC Robot Arm MuJoCo - Robot arm control and simulation framework
"""

from gymnasium.envs.registration import register

__version__ = '1.0.0'

# 注册环境
register(
    id="rc_robotarm_mujoco/AuboI5Env-v0",
    entry_point="rc_robotarm_mujoco.envs:AuboI5Env",
    max_episode_steps=1000,
)

register(
    id="rc_robotarm_mujoco/UR5eEnv-v0",
    entry_point="rc_robotarm_mujoco.envs:UR5eEnv",
    max_episode_steps=1000,
)

register(
    id="rc_robotarm_mujoco/RC_ARMEnv-v0",
    entry_point="rc_robotarm_mujoco.envs:RC_ARMEnv",
    max_episode_steps=1000,
)

register(
    id="rc_robotarm_mujoco/RC_ARM_2Env-v0",
    entry_point="rc_robotarm_mujoco.envs:RC_ARM_2Env",
    # 延长每次仿真时长，避免 TimeLimit 提前截断
    max_episode_steps=5000000,
)

# 可以添加您自己的环境
register(
    id="rc_robotarm_mujoco/CustomEnv-v0",
    entry_point="rc_robotarm_mujoco.envs:CustomEnv",
    max_episode_steps=1000,
)
