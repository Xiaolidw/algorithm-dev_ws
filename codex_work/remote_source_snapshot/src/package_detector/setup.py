from setuptools import find_packages, setup

package_name = 'moon_warehouse_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/perception.yaml']),
        ('share/' + package_name + '/launch', [
            'launch/perception.launch.py',
            'launch/mock_acceptance.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moon warehouse team',
    maintainer_email='team@example.com',
    description='Independent 2D colour perception.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'perception_node = moon_warehouse_perception.perception_node:main',
            'mock_camera_node = moon_warehouse_perception.mock_camera_node:main',
        ],
    },
)
