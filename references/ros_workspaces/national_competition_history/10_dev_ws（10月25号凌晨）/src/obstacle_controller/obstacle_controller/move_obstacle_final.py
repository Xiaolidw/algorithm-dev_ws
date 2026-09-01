#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
import time

class ObstacleMover(Node):
    def __init__(self):
        super().__init__('obstacle_mover_final')
        self.get_logger().info('障碍物移动器启动')
        
        # 创建服务客户端
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        # 等待服务可用
        self.get_logger().info('等待Gazebo服务...')
        if not self.client.wait_for_service(timeout_sec=20.0):
            self.get_logger().error('服务不可用，退出')
            return
        
        self.get_logger().info('服务连接成功，开始移动障碍物')
        
        # 移动参数
        self.x_pos = -2.0
        self.speed = 0.2  # m/s
        self.direction = 1  # 1向右，-1向左
        
        # 创建定时器，每0.1秒更新一次位置
        self.timer = self.create_timer(0.1, self.update_position)
        
    def update_position(self):
        # 更新位置
        self.x_pos += self.direction * self.speed * 0.1
        
        # 到达边界时改变方向
        if self.x_pos > 2.0:
            self.x_pos = 2.0
            self.direction = -1
        elif self.x_pos < -2.0:
            self.x_pos = -2.0
            self.direction = 1
        
        # 创建请求
        req = SetEntityState.Request()
        req.state.name = 'simple_obstacle'
        req.state.pose.position.x = self.x_pos
        req.state.pose.position.y = 0.0
        req.state.pose.position.z = 0.15
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = 'world'
        
        # 发送请求（非阻塞）
        self.client.call_async(req)
        
        # 每1秒打印一次状态
        if int(time.time()) % 1 == 0:
            self.get_logger().info(f'位置: x={self.x_pos:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('程序终止')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
