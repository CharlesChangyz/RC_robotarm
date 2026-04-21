from setuptools import setup, find_packages

setup(
    name='rc_robotarm_mujoco',
    version='1.0.0',
    author='CharlesChangyz',
    description='RC Robot Arm control and simulation with MuJoCo',
    packages=find_packages(),
    install_requires=[
        'mujoco>=2.3.3',
        'dm-control',
        'numpy<2.0.0',
    ],
    python_requires='>=3.8',
)
