from glob import glob
from setuptools import find_packages, setup

package_name = 'moon_warehouse_coordinator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
        ('share/' + package_name + '/config',
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_coordinator_node = '
            'moon_warehouse_coordinator.'
            'mission_coordinator_node:main',
            'mission_flow_executor_node = '
            'moon_warehouse_coordinator.'
            'mission_flow_executor_node:main',
            'navigation_pick_place_test = '
            'moon_warehouse_coordinator.'
            'navigation_pick_place_test:main',
            'costudio_mission_gateway = '
            'moon_warehouse_coordinator.'
            'costudio_mission_gateway:main',
            'motion_arm_recorder = '
            'moon_warehouse_coordinator.motion_arm_recorder:main',
        ],
    },
)
