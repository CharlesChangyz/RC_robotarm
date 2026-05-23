from glob import glob
import os

from setuptools import find_packages, setup


package_name = "rc_arm2_middleware"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Codex",
    maintainer_email="codex@openai.com",
    description="Sequential task middleware for high-level arm execution",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "arm2_middleware = rc_arm2_middleware.arm2_middleware_node:main",
            "orbit_target_point_publisher = rc_arm2_middleware.orbit_target_point_publisher:main",
        ],
    },
)
