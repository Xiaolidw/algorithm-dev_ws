from setuptools import setup

package_name = 'llama_command_parser'

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
    description='Llama Command Parser for Moon Warehouse Competition',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 节点入口：ros2 run 功能包名 节点命令
            'command_parser = llama_command_parser.command_parser_node:main',
        ],
    },
)
