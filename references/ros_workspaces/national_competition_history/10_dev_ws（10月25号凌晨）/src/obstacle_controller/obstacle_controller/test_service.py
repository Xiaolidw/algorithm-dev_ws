#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState

class ServiceTest(Node):
    def __init__(self):
        super().__init__('service_test')
        self.get_logger().info('服务测试节点启动')
        
        # 等待服务可用
        self.get_logger().info('等待/gazebo/set_entity_state服务...')
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        # 每隔1秒检查一次服务
        self.timer = self.create_timer(1.0, self.check_service)
        
    def check_service(self):
        if self.client.service_is_ready():
            self.get_logger().info('✓ 服务可用！')
            self.timer.cancel()
            
            # 发送测试请求
            req = SetEntityState.Request()
            req.state.name = 'moving_obstacle'
            req.state.pose.position.x = -1.0
            req.state.pose.position.y = 0.0
            req.state.pose.position.z = 0.15
            req.state.pose.orientation.w = 1.0
            req.state.reference_frame = 'world'
            
            self.get_logger().info('发送位置更新请求...')
            future = self.client.call_async(req)
            future.add_done_callback(self.response_callback)
        else:
            self.get_logger().info('服务仍不可用...')
    
    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('✓ 位置更新成功！')
            else:
                self.get_logger().error(f'✗ 位置更新失败: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'✗ 服务调用异常: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ServiceTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
