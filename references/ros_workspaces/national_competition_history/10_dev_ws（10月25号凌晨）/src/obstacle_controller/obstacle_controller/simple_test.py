#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose
import time
import os

class SimpleObstacleTest(Node):
    def __init__(self):
        super().__init__('simple_obstacle_test')
        self.get_logger().info('=' * 50)
        self.get_logger().info('简单障碍物测试节点启动')
        self.get_logger().info(f'ROS_DOMAIN_ID: {os.environ.get("ROS_DOMAIN_ID", "未设置")}')
        
        # 创建服务客户端
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        # 等待服务
        self.get_logger().info('等待/gazebo/set_entity_state服务...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务不可用，继续等待...')
        
        self.get_logger().info('✓ 成功连接到服务！')
        
        # 立即发送测试请求
        self.send_test_request()
        
        # 定时发送移动请求
        self.timer = self.create_timer(2.0, self.send_move_request)
        self.move_direction = 1  # 1为向右，-1为向左
        self.current_x = -2.0

    def send_test_request(self):
        """发送初始测试请求"""
        req = SetEntityState.Request()
        req.state.name = 'moving_obstacle'
        
        # 设置新位置
        req.state.pose.position.x = -1.0
        req.state.pose.position.y = 0.0
        req.state.pose.position.z = 0.15
        req.state.pose.orientation.w = 1.0
        
        req.state.reference_frame = 'world'
        
        self.get_logger().info('发送初始位置更新请求...')
        future = self.client.call_async(req)
        future.add_done_callback(self.test_response_callback)

    def send_move_request(self):
        """发送移动请求"""
        req = SetEntityState.Request()
        req.state.name = 'moving_obstacle'
        
        # 更新位置
        self.current_x += self.move_direction * 0.2
        if self.current_x > 2.0 or self.current_x < -2.0:
            self.move_direction *= -1
        
        req.state.pose.position.x = self.current_x
        req.state.pose.position.y = 0.0
        req.state.pose.position.z = 0.15
        req.state.pose.orientation.w = 1.0
        
        req.state.reference_frame = 'world'
        
        future = self.client.call_async(req)
        future.add_done_callback(self.move_response_callback)

    def test_response_callback(self, future):
        """处理测试响应"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('✓ 初始位置更新成功！')
            else:
                self.get_logger().error(f'✗ 初始位置更新失败: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'✗ 服务调用异常: {str(e)}')

    def move_response_callback(self, future):
        """处理移动响应"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'位置更新: x={self.current_x:.2f}m')
            else:
                self.get_logger().error(f'✗ 位置更新失败: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'✗ 服务调用异常: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleObstacleTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
