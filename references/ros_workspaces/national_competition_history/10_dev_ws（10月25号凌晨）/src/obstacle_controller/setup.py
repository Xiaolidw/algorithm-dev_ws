from setuptools import setup
import os
from glob import glob

package_name = 'obstacle_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='控制障碍物在两点间往返移动',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_obstacle = obstacle_controller.move_obstacle:main',
            'simple_test = obstacle_controller.simple_test:main',
            'test_service = obstacle_controller.test_service:main',
        ],
    },
)
