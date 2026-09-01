from setuptools import setup
import os
from glob import glob

package_name = 'yzbot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # 1. 配置包的安装路径（ROS2 标准路径）
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 2. 若有配置文件（如 RViz2 配置），可在此添加路径（当前无，预留）
        # (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',  # 替换为你的名字
    maintainer_email='your_email@example.com',  # 替换为你的邮箱
    description='Vision module for lunar resource detection (ROS2 + OpenCV)',  # 功能描述
    license='Apache-2.0',  # 许可证（比赛常用 Apache 2.0）
    tests_require=['pytest'],
    # 3. 声明节点入口（关键！用于 ros2 run 命令启动节点）
    entry_points={
        'console_scripts': [
            'object_detector_node = yzbot_vision.object_detector_node:main',
        ],
    },
)