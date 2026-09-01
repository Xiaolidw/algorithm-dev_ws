from setuptools import setup, find_packages

package_name = 'bot_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='Navigation mission controller, Foxglove visualization and dynamic obstacle tools for robot competition',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 🚗 主任务控制器（导航与抓取任务逻辑）
            'mission_controller = bot_navigation.mission_controller:main',

            # 📊 系统状态汇总节点（发布 /system_status）
            'system_status_node = bot_navigation.system_status_node:main',

            # 🧩 测试发布器（发布假数据或目标点）
            'mock_publishers = scripts.mock_publishers:main',

            # 🎯 单目标导航节点
            'goto_goal = scripts.goto_goal:main',
            
            'predictive_obstacle_layer = bot_navigation.predictive_obstacle_layer:main',
            
            'dynamic_cmd_filter = bot_navigation.dynamic_cmd_filter:main',
        ],
    },
)

