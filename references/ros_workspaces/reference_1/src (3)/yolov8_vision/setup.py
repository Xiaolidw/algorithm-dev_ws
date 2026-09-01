from setuptools import setup

package_name = 'yolov8_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolov8_vision.launch.py']),
        ('share/' + package_name + '/msg', ['msg/YoloDetection.msg', 'msg/YoloDetections.msg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='YOLOv8-based vision module for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = yolov8_vision.yolo_node:main',
        ],
    },
)

