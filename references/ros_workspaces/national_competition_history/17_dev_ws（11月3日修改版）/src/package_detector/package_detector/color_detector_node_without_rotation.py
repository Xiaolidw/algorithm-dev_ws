import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from geometry_msgs.msg import Twist
import numpy as np
import struct
from std_msgs.msg import String, Header

class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')
        
        # 1. 配置QoS
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        
        # 2. 订阅相机图像、深度图和相机内参
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        # 3. 发布话题
        self.cloud_pub = self.create_publisher(PointCloud2, '/package_coordinates', qos_profile)
        self.status_pub = self.create_publisher(String, '/color_detector/status', qos_profile)
        self.position_pub = self.create_publisher(String, '/package_raw_position', qos_profile)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 4. 初始化参数
        self.bridge = CvBridge()
        self.latest_depth = None
        self.depth_encoding = None
        self.depth_received = False
        
        # 相机内参（初始值，会被camera_info话题更新）
        self.camera_matrix = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
        self.camera_info_received = False
        
        # 物块检测状态
        self.object_detected = False
        self.rotation_timer = None
        self.rotation_speed = 0.5
        
        # 默认z值（无量纲）
        self.default_z = 1.0
        
        self.get_logger().info("Color detector node started (with default z=1 when z=0)")
        self.publish_initial_message()
        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_initial_message(self):
        status_msg = String()
        status_msg.data = "Node initialized. Waiting for camera data..."
        self.status_pub.publish(status_msg)

    def publish_status(self):
        status_msg = String()
        status_msg.data = (f"Node active. Object detected: {self.object_detected} | "
                          f"Depth received: {self.depth_received} | "
                          f"Camera info received: {self.camera_info_received}")
        self.status_pub.publish(status_msg)

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.camera_info_received = True
            self.get_logger().info(f"Received camera intrinsics: \n{self.camera_matrix}")

    def depth_callback(self, msg):
        try:
            self.depth_encoding = msg.encoding
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_received = True
            
            if hasattr(self.latest_depth, 'shape'):
                self.get_logger().debug(
                    f"Depth image received: shape={self.latest_depth.shape}, "
                    f"type={self.latest_depth.dtype}, "
                    f"min={np.min(self.latest_depth)}, "
                    f"max={np.max(self.latest_depth)}"
                )
            else:
                self.get_logger().warn("Depth image has no shape attribute")
                
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert depth image: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"Error in depth callback: {str(e)}")

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")
            return
            
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        # 颜色掩码
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        red_mask = cv2.bitwise_or(cv2.inRange(hsv_img, lower_red1, upper_red1),
                                cv2.inRange(hsv_img, lower_red2, upper_red2))
        
        lower_blue = np.array([100, 120, 70])
        upper_blue = np.array([130, 255, 255])
        blue_mask = cv2.inRange(hsv_img, lower_blue, upper_blue)

        # 计算物块位置
        red_positions = self.calculate_raw_position(red_mask, color_name="Red")
        blue_positions = self.calculate_raw_position(blue_mask, color_name="Blue")
        all_positions = red_positions + blue_positions

        self.object_detected = len(all_positions) > 0
        
        self.publish_raw_position(all_positions)
        self.publish_point_cloud_data(all_positions, msg.header)
        self.control_robot()

    def calculate_raw_position(self, mask, color_name):
        positions = []
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            if cv2.contourArea(cnt) > 500:
                M = cv2.moments(cnt)
                u = int(M['m10'] / M['m00'])
                v = int(M['m01'] / M['m00'])
                
                # 初始化坐标
                z = 0.0
                x = 0.0
                y = 0.0
                using_default_z = False  # 标记是否使用默认z值
                
                if self.latest_depth is not None:
                    try:
                        if 0 <= v < self.latest_depth.shape[0] and 0 <= u < self.latest_depth.shape[1]:
                            depth_value = self.latest_depth[v, u]
                            
                            # 处理不同编码格式
                            if self.depth_encoding == '16UC1':
                                z_m = depth_value / 1000.0  # 毫米转米
                            else:  # 32FC1
                                z_m = depth_value  # 已经是米
                            
                            # 检查深度值是否有效
                            if z_m <= 0 or np.isnan(z_m) or np.isinf(z_m):
                                # 使用默认z值（无量纲）
                                z = self.default_z
                                using_default_z = True
                                self.get_logger().warn(
                                    f"[{color_name}] Using default z={self.default_z} (invalid depth {z_m}m at (u={u}, v={v}))"
                                )
                            else:
                                # 转换为厘米
                                z = z_m * 100
                                
                            # 计算x和y
                            fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
                            cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
                            
                            # 如果使用默认z值，直接用无量纲计算
                            if using_default_z:
                                x = (u - cx) * z / fx
                                y = (v - cy) * z / fy
                            else:
                                x_m = (u - cx) * z_m / fx  # 米单位
                                y_m = (v - cy) * z_m / fy  # 米单位
                                x = x_m * 100  # 米 -> 厘米
                                y = y_m * 100  # 米 -> 厘米
                        else:
                            # 坐标越界，使用默认z值
                            z = self.default_z
                            x = (u - cx) * z / fx if self.camera_info_received else 0.0
                            y = (v - cy) * z / fy if self.camera_info_received else 0.0
                            using_default_z = True
                            self.get_logger().warn(
                                f"[{color_name}] Using default z={self.default_z} (pixel out of bounds at (u={u}, v={v}))"
                            )
                    except Exception as e:
                        self.get_logger().error(f"Error calculating position: {str(e)}")
                        # 发生错误时使用默认z值
                        z = self.default_z
                        x = 0.0
                        y = 0.0
                        using_default_z = True
                else:
                    # 无深度数据，使用默认z值
                    z = self.default_z
                    fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
                    cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
                    x = (u - cx) * z / fx if self.camera_info_received else 0.0
                    y = (v - cy) * z / fy if self.camera_info_received else 0.0
                    using_default_z = True
                    self.get_logger().warn(
                        f"[{color_name}] Using default z={self.default_z} (no depth data available)"
                    )
                
                positions.append({
                    "color": color_name,
                    "pixel_u": u, "pixel_v": v,
                    "3d_x": round(x, 1),
                    "3d_y": round(y, 1),
                    "3d_z": round(z, 1),
                    "using_default_z": using_default_z
                })
                
                # 日志输出，明确标记是否使用默认值
                if using_default_z:
                    self.get_logger().info(
                        f"[{color_name}] Position (using default z): Pixel(u={u}, v={v}) | 3D(x={x:.1f}, y={y:.1f}, z={z:.1f})"
                    )
                else:
                    self.get_logger().info(
                        f"[{color_name}] Position: Pixel(u={u}, v={v}) | 3D(x={x:.1f}cm, y={y:.1f}cm, z={z:.1f}cm)"
                    )
        
        return positions

    def publish_raw_position(self, positions):
        if not positions:
            return
        pos_msg = String()
        pos_str = []
        for p in positions:
            if p["using_default_z"]:
                pos_str.append(f"[{p['color']}: u={p['pixel_u']},v={p['pixel_v']}, z={p['3d_z']}(default)]")
            else:
                pos_str.append(f"[{p['color']}: u={p['pixel_u']},v={p['pixel_v']}, z={p['3d_z']}cm]")
        pos_msg.data = " | ".join(pos_str)
        self.position_pub.publish(pos_msg)

    def publish_point_cloud_data(self, positions, header):
        all_points = []
        for pos in positions:
            # 点云使用米单位，将厘米转回米；如果是默认值则直接使用
            if pos["using_default_z"]:
                x_m = pos["3d_x"] / 100.0  # 无量纲值按厘米处理后转米
                y_m = pos["3d_y"] / 100.0
                z_m = pos["3d_z"] / 100.0
            else:
                x_m = pos["3d_x"] / 100.0
                y_m = pos["3d_y"] / 100.0
                z_m = pos["3d_z"] / 100.0
            color_code = 0 if pos["color"] == "Red" else 1
            all_points.append((x_m, y_m, z_m, color_code))
        
        if all_points:
            self.publish_point_cloud(all_points, header)
        else:
            self.publish_empty_point_cloud(header)

    def control_robot(self):
        twist = Twist()
        
        if self.object_detected:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Object detected - stopping robot")
            
            if self.rotation_timer:
                self.rotation_timer.cancel()
                self.rotation_timer = None
                
        else:
            if not self.rotation_timer:
                self.get_logger().info("No object detected - starting robot rotation")
                self.rotation_timer = self.create_timer(0.1, self.publish_rotation_command)

    def publish_rotation_command(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.rotation_speed
        self.cmd_vel_pub.publish(twist)

    def publish_point_cloud(self, points, header):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]
        point_step = 16
        data = b''
        for (x, y, z, color) in points:
            rgb = 0xff0000 if color == 0 else 0x0000ff
            rgb_float = struct.unpack('f', struct.pack('I', rgb))[0]
            data += struct.pack('ffff', x, y, z, rgb_float)
        
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.header.frame_id = 'camera_link'
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = point_step
        cloud_msg.row_step = point_step * cloud_msg.width
        cloud_msg.is_dense = True
        cloud_msg.data = data
        self.cloud_pub.publish(cloud_msg)

    def publish_empty_point_cloud(self, header):
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.header.frame_id = 'camera_link'
        cloud_msg.height = 1
        cloud_msg.width = 0
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16
        cloud_msg.row_step = 0
        cloud_msg.is_dense = True
        cloud_msg.data = b''
        self.cloud_pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
