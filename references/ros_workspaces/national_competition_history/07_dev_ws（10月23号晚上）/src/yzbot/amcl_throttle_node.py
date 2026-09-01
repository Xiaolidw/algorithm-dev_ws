import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class AmclThrottleNode(Node):
    def __init__(self):
        super().__init__("amcl_throttle_node")
        # 配置：原话题→过滤后话题，目标频率2Hz（间隔0.5秒）
        self.input_topic = "/amcl_pose"
        self.output_topic = "/amcl_pose_throttled"
        self.target_rate = 2.0  # 目标频率（Hz）
        self.min_interval = 1.0 / self.target_rate  # 最小发送间隔（秒）
        
        # 初始化变量：记录上次发送时间
        self.last_send_time = 0.0
        
        # 订阅原话题（10Hz）
        self.subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            self.input_topic,
            self.topic_callback,
            10  # 队列大小，避免消息堆积
        )
        
        # 发布过滤后的话题（2Hz）
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.output_topic,
            10
        )
        
        self.get_logger().info(f"已启动话题过滤：{self.input_topic} → {self.output_topic}（{self.target_rate}Hz）")

    def topic_callback(self, msg):
        """收到原话题消息时，按频率限制转发"""
        current_time = time.time()
        # 检查是否达到发送间隔
        if current_time - self.last_send_time >= self.min_interval:
            self.publisher.publish(msg)  # 转发消息
            self.last_send_time = current_time  # 更新上次发送时间

def main(args=None):
    rclpy.init(args=args)
    node = AmclThrottleNode()
    try:
        rclpy.spin(node)  # 持续运行节点
    except KeyboardInterrupt:
        node.get_logger().info("话题过滤节点已终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
