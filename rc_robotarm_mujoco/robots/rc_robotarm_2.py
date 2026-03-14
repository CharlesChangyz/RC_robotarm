import os
from rc_robotarm_mujoco.robots.arm import Arm

_RCARM_XML = os.path.join(
    os.path.dirname(__file__),
    '../assets/robots/rc_arm_2/02_SUB_RoboticArm.xml',
)

_JOINTS = (
    'j1',
    'j2',
    'j3',
    'j4',
)

_EEF_SITE = 'eef_site'

_ATTACHMENT_SITE = 'attachment_site'

class RCArm_2(Arm):
    def __init__(self, name: str = None):
        super().__init__(_RCARM_XML, _EEF_SITE, _ATTACHMENT_SITE, _JOINTS, name)
