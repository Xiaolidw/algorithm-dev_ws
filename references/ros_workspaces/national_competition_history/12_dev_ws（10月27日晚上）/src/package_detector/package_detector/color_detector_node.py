import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from std_msgs.msg import String
import numpy as np


class StaticObjectAnnotator(Node):
    def __init__(self):
        super().__init__('static_object_annotator')
        
        # 1. QoS配置
        radar_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        image_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # 2. 订阅话题（保留传感器输入）
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            image_qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        self.radar_scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.radar_scan_callback,
            radar_qos
        )

        # 3. 发布话题（仅保留标注图像和状态，移除旋转指令发布器）
        self.annotated_img_pub = self.create_publisher(  # Foxglove标注图像
            Image,
            '/detector/annotated_image',
            image_qos
        )
        self.status_pub = self.create_publisher(String, '/detector/status', 10)  # 节点状态

        # 4. 初始化参数（移除旋转相关变量）
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.camera_info_received = False

        self.radar_points = []  # 用于物块存在性验证
        self.radar_received = False

        # 物块检测参数
        self.object_dimensions = {
            "length": 0.5,
            "width": 0.5,
            "height": 0.5
        }
        self.color_params = {
            "Red": {
                "lower": [np.array([0, 50, 30]), np.array([170, 50, 30])],
                "upper": [np.array([15, 255, 255]), np.array([185, 255, 255])]
            },
            "Blue": {
                "lower": [np.array([75, 20, 20])],
                "upper": [np.array([145, 255, 255])]
            }
        }
        self.min_contour_area = 100
        self.radar_min_distance = 0.05

        self.get_logger().info("节点启动：静止状态，仅进行Foxglove物块标注")
        self.status_timer = self.create_timer(1.0, self.publish_status)

    # 相机内参回调
    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.camera_info_received = True
            self.get_logger().info("相机内参已接收")

    # 雷达数据回调（仅用于物块验证）
    def radar_scan_callback(self, msg):
        try:
            self.radar_points.clear()
            current_angle = msg.angle_min
            for range_val in msg.ranges:
                if not np.isnan(range_val) and not np.isinf(range_val):
                    if msg.range_min < range_val < msg.range_max:
                        x = range_val * np.cos(current_angle)
                        y = range_val * np.sin(current_angle)
                        self.radar_points.append((x, y))
                current_angle += msg.angle_increment
            self.radar_received = True
        except Exception as e:
            self.get_logger().error(f"雷达解析错误：{str(e)}")

    # 图像处理与标注（核心功能）
    def image_callback(self, msg):
        if not self.camera_info_received or not self.radar_received:
            self.publish_status("等待相机内参/雷达数据...")
            return

        try:
            # 转换图像并创建标注副本
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            annotated_img = cv_img.copy()
            hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

            detected_count = 0
            # 遍历颜色检测物块
            for color_name, config in self.color_params.items():
                # 生成颜色掩码
                mask = None
                for lower, upper in zip(config["lower"], config["upper"]):
                    current_mask = cv2.inRange(hsv_img, lower, upper)
                    mask = current_mask if mask is None else cv2.bitwise_or(mask, current_mask)

                # 提取轮廓
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                    if cv2.contourArea(cnt) > self.min_contour_area:
                        # 计算轮廓中心
                        M = cv2.moments(cnt)
                        if M['m00'] < 1e-6:
                            continue
                        u = int(M['m10'] / M['m00'])
                        v = int(M['m01'] / M['m00'])

                        # 验证物块存在性
                        if self.verify_object(u, v):
                            detected_count += 1
                            # 绘制标注（Foxglove显示用）
                            self.draw_annotation(annotated_img, cnt, u, v, color_name)

            # 发布标注图像
            self.publish_annotated_image(annotated_img, msg.header)
            self.get_logger().debug(f"检测到物块数量：{detected_count}")

        except Exception as e:
            self.get_logger().error(f"图像处理错误：{str(e)}")

    # 验证物块存在性（不计算坐标）
    def verify_object(self, u, v):
        fx, cx = self.camera_matrix[0, 0], self.camera_matrix[0, 2]
        fy, cy = self.camera_matrix[1, 1], self.camera_matrix[1, 2]

        dir_x = (u - cx) / fx
        dir_y = (v - cy) / fy
        dir_norm = np.linalg.norm([dir_x, dir_y])
        if dir_norm < 1e-6:
            return False
        dir_x /= dir_norm
        dir_y /= dir_norm

        for (rx, ry) in self.radar_points:
            radar_norm = np.linalg.norm([rx, ry])
            if radar_norm < self.radar_min_distance:
                continue
            radar_dir_x = rx / radar_norm
            radar_dir_y = ry / radar_norm

            if (dir_x * radar_dir_x + dir_y * radar_dir_y) > 0.7:
                return True
        return False

    # 绘制物块标注
    def draw_annotation(self, img, contour, u, v, color_name):
        # 绿色轮廓
        cv2.drawContours(img, [contour], -1, (0, 255, 0), 2)
        # 红色中心点
        cv2.circle(img, (u, v), 5, (0, 0, 255), -1)
        # 颜色标签
        cv2.putText(img, color_name, (u+10, v), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        # 尺寸信息
        cv2.putText(img, f"尺寸: {self.object_dimensions['length']}m x {self.object_dimensions['width']}m", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # 发布标注图像到Foxglove
    def publish_annotated_image(self, img, header):
        try:
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            img_msg.header = header
            self.annotated_img_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"标注图像发布失败：{str(e)}")

    # 发布节点状态
    def publish_status(self, custom_msg=None):
        status_msg = String()
        if custom_msg:
            status_msg.data = custom_msg
        else:
            status_msg.data = (
                f"状态：静止标注中 | "
                f"相机就绪：{self.camera_info_received} | "
                f"雷达就绪：{self.radar_received} | "
                f"物块尺寸：{self.object_dimensions['length']}x{self.object_dimensions['width']}m"
            )
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StaticObjectAnnotator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断，节点停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
