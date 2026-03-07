# import os
# import time
# from dm_control import mjcf
# import mujoco.viewer

# # 如果从项目根运行，下面的相对路径有效；否则改为绝对路径
# xml_rel = "/media/dust/新加卷1/rc_robotarm_mujoco/rc_robotarm_mujoco/assets/robots/rc_arm/0-臂-anOt.xml"
# xml_path = os.path.abspath(xml_rel)

# m = mjcf.from_path(xml_path)
# physics = mjcf.Physics.from_mjcf_model(m)

# # 启动只读 viewer（被动模式），可按 Ctrl+C 退出
# viewer = mujoco.viewer.launch_passive(physics.model.ptr, physics.data.ptr)
# try:
#     while True:
#         viewer.sync()
#         time.sleep(0.01)
# except KeyboardInterrupt:
#     viewer.close()

# python
# python
from dm_control import mjcf
import os

xml_path = os.path.abspath("rc_robotarm_mujoco/assets/robots/rc_arm/0-臂-anOt.xml")
m = mjcf.from_path(xml_path)

for j in m.find_all('joint'):
    body = j.parent
    # 只挑 parent 直接拥有的 geom（不包含子 body 的）
    direct_geoms = [g for g in m.find_all('geom') if g.parent is body]
    meshes = [getattr(g, 'mesh', None) for g in direct_geoms if getattr(g, 'mesh', None)]
    print(f"{j.name} -> {getattr(body,'name',None)} : {meshes}")
    