from setuptools import setup
from setuptools import find_packages

package_name = "tools_demo"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/models/vision", ["models/vision/best.pt"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sunrise",
    maintainer_email="sunrise@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "chat = tools_demo.chat_interactive:main",
            "chat_ros = tools_demo.chat_interactive_ros:main",
            "vision_capture = tools_demo.vision.image_capture:main",
            "vision_detect = tools_demo.vision.color_detector:main",
            "simple_teleop = tools_demo.simple_teleop:main",
        ],
    },
)
