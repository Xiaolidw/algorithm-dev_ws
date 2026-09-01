from setuptools import find_packages, setup


package_name = 'moon_warehouse_moveit'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/fixed_manipulation.yaml']),
        ('share/' + package_name + '/launch', ['launch/fixed_manipulation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moon warehouse team',
    maintainer_email='team@example.com',
    description='Configuration-driven fixed pick/place execution for the competition robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fixed_manipulation_server = moon_warehouse_moveit.fixed_manipulation_server:main',
            'manipulation_health_check = moon_warehouse_moveit.manipulation_health_check:main',
            'send_fixed_task = moon_warehouse_moveit.send_fixed_task:main',
            'vision_grasp_adapter = moon_warehouse_moveit.vision_grasp_adapter:main',
        ],
    },
)
