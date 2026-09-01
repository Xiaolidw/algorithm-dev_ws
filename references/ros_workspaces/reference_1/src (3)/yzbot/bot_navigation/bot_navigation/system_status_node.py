#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class SystemStatusNode(Node):
    def __init__(self):
        super().__init__('system_status_node')
        self.get_logger().info("SystemStatusNode 已启动")

        # 状态缓存
        self.mission_state = "未启动"
        self.nav_status = "空闲"
        self.arm_status = "空闲"
        self.last_update_time = time.time()

        # 订阅来自 mission_controller 与 auto_grasp_moveit 的状态话题
        self.create_subscription(String, '/mission_state', self.mission_callback, 10)
        self.create_subscription(String, '/nav_status', self.nav_callback, 10)
        self.create_subscription(String, '/arm_status', self.arm_callback, 10)

        # 发布汇总系统状态
        self.system_pub = self.create_publisher(String, '/system_status', 10)

        # 定时器，每秒发布一次系统状态
        self.timer = self.create_timer(1.0, self.publish_system_status)

    # ======================== 回调函数 ========================
    def mission_callback(self, msg):
        self.mission_state = msg.data.strip()
        self.last_update_time = time.time()

    def nav_callback(self, msg):
        self.nav_status = msg.data.strip()
        self.last_update_time = time.time()

    def arm_callback(self, msg):
        self.arm_status = msg.data.strip()
        self.last_update_time = time.time()

    # ======================== 状态汇总 ========================
    def publish_system_status(self):
        # 自动判断系统总体状态
        if "导航" in self.nav_status and "成功" not in self.nav_status:
            overall = "导航中"
        elif "抓取" in self.arm_status and "完成" not in self.arm_status:
            overall = "抓取中"
        elif "放置" in self.arm_status and "完成" not in self.arm_status:
            overall = "放置中"
        elif "完成" in self.mission_state:
            overall = "任务完成"
        else:
            overall = "待命"

        msg = String()
        msg.data = f"{overall} | {self.nav_status} | {self.arm_status}"
        self.system_pub.publish(msg)

        # 输出日志
        self.get_logger().info(f"系统状态：{msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = SystemStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断 SystemStatusNode")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
