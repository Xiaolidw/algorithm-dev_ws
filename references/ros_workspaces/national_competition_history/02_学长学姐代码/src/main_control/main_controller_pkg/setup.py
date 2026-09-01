from setuptools import setup
import os
from glob import glob

package_name = 'main_controller_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='你的用户名',
    maintainer_email='你的邮箱',
    description='主控节点（支持智能路径规划）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 这里定义可执行文件名称和入口函数
            'main_controller = main_controller_pkg.main_controller:main',
        ],
    },
)

