from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rc_arm_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch', 'rc_arm_2'), glob('launch/rc_arm_2/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'rc_arm_2'), glob('config/rc_arm_2/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wy',
    maintainer_email='wy@todo.todo',
    description='Gamepad teleoperation package for the rc_arm_2 robot arm',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'xbox_teleop_node_rc_arm_2 = rc_arm_teleop.xbox_teleop_node_rc_arm_2:main',
        ],
    },
)
