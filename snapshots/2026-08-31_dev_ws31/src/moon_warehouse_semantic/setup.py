from glob import glob
from setuptools import find_packages, setup

package_name = 'moon_warehouse_semantic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
data_files=[
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name],
    ),
    (
        'share/' + package_name,
        ['package.xml'],
    ),
    (
        'share/' + package_name + '/launch',
        glob('launch/*.launch.py'),
    ),
    (
        'share/' + package_name + '/config',
        glob('config/*.yaml'),
    ),
],#作用是把 launch 和 YAML 安装到 ROS 2 的 install 目录。
  #否则源码中虽然有文件，ros2 launch 仍然找不到
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'question_source_node = moon_warehouse_semantic.question_source_node:main',
            'semantic_solver_node = moon_warehouse_semantic.semantic_solver_node:main',
        ],
    },
)
