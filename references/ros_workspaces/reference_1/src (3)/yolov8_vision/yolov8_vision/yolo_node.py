import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO
import cv2


class YoloV8Node(Node):
    def __init__(self):
        super().__init__('yolov8_node')

        # ========== 参数 ==========
        # 模型路径就用你现在的这个
        model_path_default = '/home/ros/dev_ws/src/yolov8_vision/models/best.pt'

        self.declare_parameter('model_path', model_path_default)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('device', 'cpu')  # or 'cuda'

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.device = self.get_parameter('device').get_parameter_value().string_value

        # ========== 加载模型 ==========
        self.get_logger().info(f'Loading YOLOv8 model from: {model_path}')
        self.model = YOLO(model_path)

        self.bridge = CvBridge()

        # 订阅原始图像
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # 发布带框图像
        self.anno_pub = self.create_publisher(
            Image,
            '/yolov8/annotated_image',
            10
        )

        self.get_logger().info(
            f'YoloV8Node started. Subscribing: {image_topic}, '
            f'publishing annotated image on /yolov8/annotated_image'
        )

    def image_callback(self, msg: Image):
        # ROS Image -> OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # YOLO 推理
        try:
            results = self.model.predict(
                source=frame,
                conf=self.conf_threshold,
                verbose=False,
                device=self.device
            )
        except Exception as e:
            self.get_logger().error(f'YOLO inference error: {e}')
            return

        if len(results) == 0:
            return

        result = results[0]

        # ultralytics 自带画框
        annotated = result.plot()

        try:
            anno_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            anno_msg.header = msg.header
            self.anno_pub.publish(anno_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish annotated image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloV8Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

