from setuptools import setup, find_packages

package_name = "arm_action_integration"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your.email@example.com",
    description="Integrate arm/gripper actions based on official scripts",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 节点启动命令：ros2 run arm_action_integration arm_grab_place_node
            "arm_grab_place_node = arm_action_integration.arm_grab_place_node:main",
        ],
    },
)
