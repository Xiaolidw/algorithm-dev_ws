from setuptools import setup

package_name = 'nav_control'

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
    maintainer_email='your_email@example.com',
    description='Nav2 导航控制模块（比赛用）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 节点入口：ros2 run nav_control nav2_navigator
            'nav2_navigator = nav_control.navigator_node:main',
        ],
    },
)
