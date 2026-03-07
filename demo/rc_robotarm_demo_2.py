import sys
import os
import gymnasium

# 将项目根路径加入 sys.path，便于在未通过 pip 安装包时直接以源码方式导入
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

import rc_robotarm_mujoco
import numpy as np



# 以 human 模式创建并渲染环境
env = gymnasium.make('rc_robotarm_mujoco/RC_ARM_2Env-v0', render_mode='human')

# 使用指定种子重置环境以便结果可复现
observation, info = env.reset(seed=42)

# 运行仿真（示例为无限循环），也可以使用固定步数：
# for _ in range(1000):
while True:
    # 固定动作：不移动
    action = env.action_space.sample()

    # 使用选定动作在环境中执行一步
    observation, reward, terminated, truncated, info = env.step(action)

    # 检查回合是否结束（terminated）或被截断（truncated）
    if terminated or truncated:
        # 回合结束或被截断时重置环境
        observation, info = env.reset()

# 仿真结束后关闭环境
env.close()
