#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
import math
import time

class ObstacleMover(Node):
    def __init__(self):
        super().__init__('obstacle_mover')
        
        # 障碍物名称（必须与世界文件中的名称一致）
        self.obstacle_name = "moving_obstacle"
        
        # 运动参数设置
        self.point1 = [-2.0, 0.0, 0.15]  # 起点坐标 (x, y, z)
        self.point2 = [2.0, 0.0, 0.15]   # 终点坐标 (x, y, z)
        self.speed = 0.2                 # 移动速度 (m/s)
        self.update_rate = 10.0          # 更新频率 (Hz)
        self.distance_threshold = 0.05   # 到达目标点的距离阈值 (m)
        
        # 当前状态
        self.current_pose = None
        self.target_point = self.point2  # 初始目标点
        self.moving_forward = True       # 是否正向移动
        self.service_available = False   # 服务是否可用
        self.service_check_count = 0     # 服务检查计数器
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('障碍物移动控制器启动成功')
        self.get_logger().info(f'ROS_DOMAIN_ID: {os.environ.get("ROS_DOMAIN_ID", "未设置")}')
        self.get_logger().info(f'运动参数: 速度={self.speed}m/s')
        self.get_logger().info(f'起点: {self.point1}')
        self.get_logger().info(f'终点: {self.point2}')
        self.get_logger().info(f'障碍物名称: {self.obstacle_name}')
        self.get_logger().info('=' * 50)
        
        # 创建Gazebo服务客户端
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        # 列出所有可用服务（用于调试）
        self.create_timer(5.0, self.list_available_services)
        
        # 使用定时器定期检查服务是否可用
        self.service_check_timer = self.create_timer(1.0, self.check_service_availability)
        
        # 订阅模型状态话题
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )
        
        # 创建定时器控制移动（初始不启动）
        self.move_timer = None
        
        # 检查模型是否存在
        self.create_timer(2.0, self.check_model_existence)

    def list_available_services(self):
        """列出所有可用服务（调试用）"""
        if not self.service_available:
            services = self.get_service_names_and_types()
            gazebo_services = [s for s in services if 'gazebo' in s[0]]
            if gazebo_services:
                self.get_logger().info('')
                self.get_logger().info('发现的Gazebo服务:')
                for srv_name, srv_type in gazebo_services:
                    self.get_logger().info(f'  {srv_name}: {srv_type}')

    def check_service_availability(self):
        """检查服务是否可用"""
        self.service_check_count += 1
        
        if not self.service_available:
            if self.set_state_client.wait_for_service(timeout_sec=0.5):
                self.service_available = True
                self.get_logger().info('')
                self.get_logger().info('✓ 成功连接到/gazebo/set_entity_state服务')
                
                # 停止服务检查定时器
                self.service_check_timer.cancel()
                
                # 启动移动定时器
                self.move_timer = self.create_timer(1.0/self.update_rate, self.update_position)
                self.get_logger().info(f'✓ 启动移动控制器，更新频率: {self.update_rate}Hz')
            else:
                if self.service_check_count % 5 == 0:
                    self.get_logger().warn(f'等待服务中... ({self.service_check_count}秒)')
                    self.get_logger().warn('检查: 1. Gazebo是否完全启动 2. 服务名称是否正确 3. ROS_DOMAIN_ID是否正确')

    def check_model_existence(self):
        """检查模型是否存在于Gazebo中"""
        if self.current_pose is None:
            self.get_logger().warn(f'未找到障碍物模型: {self.obstacle_name}')
            self.get_logger().warn('请检查: 1. world文件中是否添加了模型 2. 模型名称是否一致')
        else:
            self.get_logger().info(f'✓ 找到障碍物模型: {self.obstacle_name}')
            self.get_logger().info(f'✓ 当前位置: x={self.current_pose.position.x:.2f}, y={self.current_pose.position.y:.2f}')

    def model_states_callback(self, msg):
        """获取障碍物当前位置"""
        try:
            index = msg.name.index(self.obstacle_name)
            self.current_pose = msg.pose[index]
        except ValueError:
            # 只在服务可用时才显示警告，避免过多日志
            pass

    def update_position(self):
        """更新障碍物位置"""
        if not self.service_available or self.current_pose is None:
            return
        
        # 获取当前位置
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # 计算到目标点的距离
        dx = self.target_point[0] - current_x
        dy = self.target_point[1] - current_y
        distance_to_target = math.hypot(dx, dy)
        
        # 检查是否到达目标点
        if distance_to_target < self.distance_threshold:
            self.switch_target()
            return
        
        # 计算移动方向
        direction_x = dx / distance_to_target
        direction_y = dy / distance_to_target
        
        # 计算下一步位置
        step_distance = self.speed / self.update_rate
        new_x = current_x + direction_x * step_distance
        new_y = current_y + direction_y * step_distance
        
        # 设置新位置
        self.set_obstacle_position(new_x, new_y, self.target_point[2])

    def switch_target(self):
        """切换目标点"""
        if self.moving_forward:
            self.target_point = self.point1
            self.moving_forward = False
            self.get_logger().info(f'→ 到达终点，开始返回起点 {self.point1}')
        else:
            self.target_point = self.point2
            self.moving_forward = True
            self.get_logger().info(f'→ 到达起点，开始前往终点 {self.point2}')

    def set_obstacle_position(self, x, y, z):
        """设置障碍物位置"""
        req = SetEntityState.Request()
        req.state.name = self.obstacle_name
        
        # 设置新位置
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = z
        
        # 保持原有姿态
        if self.current_pose:
            req.state.pose.orientation = self.current_pose.orientation
        
        req.state.reference_frame = 'world'
        
        # 发送请求
        future = self.set_state_client.call_async(req)
        
        # 处理响应（非阻塞）
        future.add_done_callback(self.set_state_callback)

    def set_state_callback(self, future):
        """处理服务响应"""
        try:
            response = future.result()
            if response.success:
                # 每50次更新打印一次状态
                if self.move_timer.timer_handle.last_call_time_ns % 50 == 0:
                    self.get_logger().info(f'位置更新: x={self.current_pose.position.x:.2f}, y={self.current_pose.position.y:.2f}')
            else:
                self.get_logger().warn(f'设置位置失败: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'服务调用失败: {str(e)}')

def main(args=None):
    import os
    rclpy.init(args=args)
    node = ObstacleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断，关闭节点')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()