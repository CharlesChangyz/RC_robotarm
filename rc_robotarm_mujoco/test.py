import os
import time
from dm_control import mjcf
import mujoco.viewer

# 如果从项目根运行，下面的相对路径有效；否则改为绝对路径
xml_rel = "/media/dust/新加卷1/rc_robotarm_mujoco/rc_robotarm_mujoco/assets/robots/rc_arm/0-臂-anOt.xml"
xml_path = os.path.abspath(xml_rel)

m = mjcf.from_path(xml_path)
physics = mjcf.Physics.from_mjcf_model(m)

# 启动只读 viewer（被动模式），可按 Ctrl+C 退出
viewer = mujoco.viewer.launch_passive(physics.model.ptr, physics.data.ptr)
try:
    while True:
        viewer.sync()
        time.sleep(0.01)
except KeyboardInterrupt:
    viewer.close()