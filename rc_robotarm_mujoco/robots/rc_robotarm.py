import os
from rc_robotarm_mujoco.robots.arm import Arm

_RCARM_XML = os.path.join(
    os.path.dirname(__file__),
    '../assets/robots/rc_arm/0-臂-anOt.xml',
)

_JOINTS = (
    'J1',
    'J2',
    'J3',
    'J4',
    'J5',
    'J6',
    'J7',
    'PJ11'
)

_EEF_SITE = 'eef_site'

_ATTACHMENT_SITE = 'attachment_site'

class RCArm(Arm):
    def __init__(self, name: str = None):
        super().__init__(_RCARM_XML, _EEF_SITE, _ATTACHMENT_SITE, _JOINTS, name)