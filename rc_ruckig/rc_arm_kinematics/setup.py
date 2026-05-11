from setuptools import find_packages, setup


package_name = "rc_arm_kinematics"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenAI",
    maintainer_email="support@openai.com",
    description="Shared IK/FK helpers for the RC arm Cartesian control chain.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
