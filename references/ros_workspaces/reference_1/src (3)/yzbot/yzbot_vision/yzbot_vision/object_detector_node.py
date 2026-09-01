import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__("object_detector_node")
        self.bridge = CvBridge()
        
        self.color_config = {
            "red": {  
                "ranges": [
                    (np.array([0, 50, 50]), np.array([10, 255, 255])),   
                    (np.array([170, 50, 50]), np.array([180, 255, 255])) 
                ],
                "draw_color": (0, 255, 0),  # 绿色边界框
                "text_color": (255, 255, 255)  # 白色文字
            },
            "blue": {  
                "ranges": [
                    (np.array([100, 43, 46]), np.array([124, 255, 255]))  
                ],
                "draw_color": (0, 255, 0),  # 绿色边界框
                "text_color": (255, 255, 255)  # 白色文字
            }
        }
        
        self.image_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )
        
        self.detected_img_pub = self.create_publisher(Image, "/detected_image", 10)
        
        self.get_logger().info("物块识别节点（白色文字版）已启动")

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            detected_img = cv_img.copy()
            hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            
            for color_name, config in self.color_config.items():
                mask = np.zeros(hsv_img.shape[:2], dtype=np.uint8)
                for lower, upper in config["ranges"]:
                    mask = cv2.bitwise_or(mask, cv2.inRange(hsv_img, lower, upper))
                
                kernel = np.ones((3, 3), np.uint8)
                mask = cv2.dilate(mask, kernel, iterations=1)
                mask = cv2.erode(mask, kernel, iterations=1)
                
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                min_contour_area = 50
                
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area < min_contour_area:
                        continue
                    
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x = round(x + w/2, 1)
                    center_y = round(y + h/2, 1)
                    
                    # 绘制绿色边界框
                    cv2.rectangle(detected_img, (x, y), (x+w, y+h), config["draw_color"], 2)
                    # 绘制绿色中心点（实心圆+外圈）
                    cv2.circle(detected_img, (int(center_x), int(center_y)), 6, (0, 255, 0), -1)
                    cv2.circle(detected_img, (int(center_x), int(center_y)), 8, (0, 255, 0), 2)
                    # 绘制白色文字
                    text = f"{color_name}: ({center_x},{center_y})"
                    cv2.putText(
                        detected_img, text,
                        (x, y-10 if y > 15 else y+20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, config["text_color"], 1
                    )
            
            detected_img_msg = self.bridge.cv2_to_imgmsg(detected_img, "bgr8")
            detected_img_msg.header = msg.header
            self.detected_img_pub.publish(detected_img_msg)

        except Exception as e:
            self.get_logger().error(f"处理异常：{str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()