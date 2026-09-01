from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/ros/dev_ws/src/yolov8_vision/models/best.pt',
        description='Path to YOLOv8 model (.pt)'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Input image topic'
    )

    conf_thres_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.5',
        description='Confidence threshold'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cpu',
        description='Inference device: cpu or cuda'
    )

    yolo_node = Node(
        package='yolov8_vision',
        executable='yolo_node',
        name='yolov8_node',
        output='screen',
        parameters=[
            {
                'model_path': LaunchConfiguration('model_path'),
                'image_topic': LaunchConfiguration('image_topic'),
                'conf_threshold': LaunchConfiguration('conf_threshold'),
                'device': LaunchConfiguration('device'),
            }
        ]
    )

    return LaunchDescription([
        model_path_arg,
        image_topic_arg,
        conf_thres_arg,
        device_arg,
        yolo_node
    ])

