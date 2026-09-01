import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from rclpy.timer import Timer
import json
import math
import weakref
import gc
from itertools import permutations
from datetime import datetime, timedelta

class ObstacleTrajectoryPredictor:
    """障碍物轨迹预测器"""
    def __init__(self, obstacle_name):
        self.name = obstacle_name
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_update_time = 0.0
        self.active = False
        
        # 轨迹参数（从room.world获取）
        self.start_x = -9.0
        self.start_y = -4.2
        self.end_x = -5.0
        self.end_y = -3.3
        self.speed = 0.3  # m/s
        
        # 计算轨迹向量
        self.trajectory_vector = (self.end_x - self.start_x, self.end_y - self.start_y)
        self.trajectory_length = math.hypot(*self.trajectory_vector)
        if self.trajectory_length > 0.001:
            self.direction_unit_vector = (
                self.trajectory_vector[0] / self.trajectory_length,
                self.trajectory_vector[1] / self.trajectory_length
            )
        else:
            self.direction_unit_vector = (0.0, 0.0)
        
        # 当前运动方向（1为正向，-1为反向）
        self.current_direction = 1.0
        
    def update_position(self, x, y, current_time):
        """更新位置并计算速度和方向"""
        if self.active:
            time_diff = current_time - self.last_update_time
            if time_diff > 0.01:
                # 计算速度
                self.vx = (x - self.last_x) / time_diff
                self.vy = (y - self.last_y) / time_diff
                
                # 判断运动方向
                speed_magnitude = math.hypot(self.vx, self.vy)
                if speed_magnitude > 0.01:
                    # 计算当前速度与轨迹方向的夹角
                    dot_product = (self.vx * self.direction_unit_vector[0] + 
                                  self.vy * self.direction_unit_vector[1])
                    if dot_product > 0:
                        self.current_direction = 1.0
                    else:
                        self.current_direction = -1.0
        
        self.last_x = self.x
        self.last_y = self.y
        self.x = x
        self.y = y
        self.last_update_time = current_time
        self.active = True
        
    def predict_position_at_time(self, target_time):
        """预测指定时间的位置"""
        if not self.active:
            return (self.x, self.y)
            
        time_diff = target_time - self.last_update_time
        if time_diff <= 0:
            return (self.x, self.y)
        
        # 预测直线运动
        predicted_x = self.x + self.vx * time_diff
        predicted_y = self.y + self.vy * time_diff
        
        # 检查是否到达边界
        distance_to_start = math.hypot(predicted_x - self.start_x, predicted_y - self.start_y)
        distance_to_end = math.hypot(predicted_x - self.end_x, predicted_y - self.end_y)
        
        # 如果超过边界，计算反弹后的位置
        if distance_to_start < 0.1 or distance_to_end < 0.1:
            # 计算到达边界的时间
            if self.current_direction == 1.0:
                boundary_x, boundary_y = self.end_x, self.end_y
            else:
                boundary_x, boundary_y = self.start_x, self.start_y
                
            time_to_boundary = math.hypot(boundary_x - self.x, boundary_y - self.y) / max(math.hypot(self.vx, self.vy), 0.01)
            remaining_time = time_diff - time_to_boundary
            
            if remaining_time > 0:
                # 反弹，速度反向
                rebound_vx = -self.vx
                rebound_vy = -self.vy
                predicted_x = boundary_x + rebound_vx * remaining_time
                predicted_y = boundary_y + rebound_vy * remaining_time
                
        return (predicted_x, predicted_y)
    
    def get_trajectory_segment(self):
        """获取当前轨迹段"""
        if self.current_direction == 1.0:
            return (self.start_x, self.start_y, self.end_x, self.end_y)
        else:
            return (self.end_x, self.end_y, self.start_x, self.start_y)
    
    def is_position_on_trajectory(self, x, y, tolerance=0.1):
        """判断点是否在轨迹上"""
        px, py = self.project_point_to_line(x, y)
        distance_to_line = math.hypot(x - px, y - py)
        return distance_to_line < tolerance
    
    def project_point_to_line(self, x, y):
        """将点投影到轨迹线上"""
        x1, y1, x2, y2 = self.get_trajectory_segment()
        
        A = y2 - y1
        B = x1 - x2
        C = x2 * y1 - x1 * y2
        
        denominator = A**2 + B**2
        if denominator < 0.001:
            return (x1, y1)
            
        t = (A * x + B * y + C) / denominator
        proj_x = x - A * t
        proj_y = y - B * t
        
        # 确保投影点在线段上
        min_x, max_x = min(x1, x2), max(x1, x2)
        min_y, max_y = min(y1, y2), max(y1, y2)
        
        proj_x = max(min_x, min(proj_x, max_x))
        proj_y = max(min_y, min(proj_y, max_y))
        
        return (proj_x, proj_y)
    
    def calculate_time_to_reach_point(self, target_x, target_y):
        """计算到达目标点的时间"""
        if math.hypot(self.vx, self.vy) < 0.01:
            return float("inf")
            
        distance = math.hypot(target_x - self.x, target_y - self.y)
        return distance / math.hypot(self.vx, self.vy)


class SafePassageCalculator:
    """安全通过计算器"""
    def __init__(self):
        # 机器人参数
        self.robot_radius = 0.22  # m
        self.robot_speed = 0.5    # m/s
        
        # 障碍物参数
        self.obstacle_radius = 0.375  # m (0.75m的一半)
        self.obstacle_speed = 0.3     # m/s
        
        # 安全参数
        self.safety_margin = 0.1      # m
        self.min_safe_distance = self.robot_radius + self.obstacle_radius + self.safety_margin
        
    def is_safe_position(self, robot_x, robot_y, obstacle_x, obstacle_y):
        """判断位置是否安全"""
        distance = math.hypot(robot_x - obstacle_x, robot_y - obstacle_y)
        return distance >= self.min_safe_distance
    
    def calculate_passage_time(self, start_x, start_y, end_x, end_y):
        """计算通过路径的时间"""
        distance = math.hypot(end_x - start_x, end_y - start_y)
        return distance / self.robot_speed
    
    def find_safe_passage_window(self, robot_x, robot_y, target_x, target_y, obstacle, max_lookahead=15.0):
        """寻找安全通过窗口"""
        # 计算机器人通过危险区域的时间
        danger_zone_start = obstacle.project_point_to_line(robot_x, robot_y)
        danger_zone_end = obstacle.project_point_to_line(target_x, target_y)
        
        # 如果起点或终点在危险区域内，需要特别处理
        if not self.is_safe_position(robot_x, robot_y, obstacle.x, obstacle.y):
            return self._handle_start_in_danger(robot_x, robot_y, target_x, target_y, obstacle, max_lookahead)
        
        # 计算机器人到达危险区域的时间
        time_to_danger = self.calculate_passage_time(robot_x, robot_y, danger_zone_start[0], danger_zone_start[1])
        time_through_danger = self.calculate_passage_time(danger_zone_start[0], danger_zone_start[1], 
                                                       danger_zone_end[0], danger_zone_end[1])
        
        # 寻找安全窗口
        current_time = obstacle.last_update_time
        safe_windows = []
        
        # 检查未来max_lookahead秒内的安全窗口
        for t_offset in [i * 0.1 for i in range(int(max_lookahead / 0.1))]:
            # 机器人进入危险区域的时间
            robot_entry_time = current_time + t_offset + time_to_danger
            robot_exit_time = robot_entry_time + time_through_danger
            
            # 障碍物在这段时间内的位置
            obstacle_entry_pos = obstacle.predict_position_at_time(robot_entry_time)
            obstacle_exit_pos = obstacle.predict_position_at_time(robot_exit_time)
            
            # 检查机器人进入时是否安全
            if self.is_safe_position(danger_zone_start[0], danger_zone_start[1], 
                                   obstacle_entry_pos[0], obstacle_entry_pos[1]):
                # 检查机器人通过期间是否一直安全
                safe_throughout = True
                for check_time in [robot_entry_time + i * 0.2 for i in range(int((robot_exit_time - robot_entry_time) / 0.2) + 1)]:
                    check_pos = obstacle.predict_position_at_time(check_time)
                    robot_pos_at_check = self._predict_robot_position(robot_x, robot_y, target_x, target_y, check_time - current_time)
                    
                    if not self.is_safe_position(robot_pos_at_check[0], robot_pos_at_check[1], check_pos[0], check_pos[1]):
                        safe_throughout = False
                        break
                
                if safe_throughout:
                    safe_windows.append({
                        'start_delay': t_offset,
                        'total_time': t_offset + time_to_danger + time_through_danger,
                        'safety_level': self._calculate_safety_level(robot_entry_time, robot_exit_time, obstacle)
                    })
        
        if safe_windows:
            # 选择最佳安全窗口（最早且最安全的）
            best_window = min(safe_windows, key=lambda x: (x['start_delay'], -x['safety_level']))
            return {
                'safe': True,
                'action': 'wait_and_pass',
                'wait_time': best_window['start_delay'],
                'total_time': best_window['total_time'],
                'safety_level': best_window['safety_level']
            }
        
        return {
            'safe': False,
            'action': 'need_detour',
            'reason': 'no safe window found'
        }
    
    def _handle_start_in_danger(self, robot_x, robot_y, target_x, target_y, obstacle, max_lookahead):
        """处理机器人起点在危险区域的情况"""
        current_time = obstacle.last_update_time
        
        # 等待障碍物离开
        for t_offset in [i * 0.1 for i in range(int(max_lookahead / 0.1))]:
            future_time = current_time + t_offset
            future_obstacle_pos = obstacle.predict_position_at_time(future_time)
            
            if self.is_safe_position(robot_x, robot_y, future_obstacle_pos[0], future_obstacle_pos[1]):
                return {
                    'safe': True,
                    'action': 'wait_for_clearance',
                    'wait_time': t_offset,
                    'total_time': t_offset + self.calculate_passage_time(robot_x, robot_y, target_x, target_y)
                }
        
        return {
            'safe': False,
            'action': 'need_detour',
            'reason': 'start position in danger zone and no clearance'
        }
    
    def _predict_robot_position(self, start_x, start_y, end_x, end_y, time_elapsed):
        """预测机器人在指定时间后的位置"""
        total_time = self.calculate_passage_time(start_x, start_y, end_x, end_y)
        if total_time < 0.01:
            return (end_x, end_y)
        
        progress = min(time_elapsed / total_time, 1.0)
        x = start_x + (end_x - start_x) * progress
        y = start_y + (end_y - start_y) * progress
        return (x, y)
    
    def _calculate_safety_level(self, entry_time, exit_time, obstacle):
        """计算安全等级"""
        min_distance = float("inf")
        current_time = obstacle.last_update_time
        
        for check_time in [entry_time + i * 0.2 for i in range(int((exit_time - entry_time) / 0.2) + 1)]:
            obstacle_pos = obstacle.predict_position_at_time(check_time)
            robot_pos = self._predict_robot_position(0, 0, 1, 0, check_time - current_time)  # 相对位置
            
            distance = math.hypot(robot_pos[0] - obstacle_pos[0], robot_pos[1] - obstacle_pos[1])
            safety_distance = distance - self.min_safe_distance
            min_distance = min(min_distance, safety_distance)
        
        return max(min_distance, 0.0)


class MainController(Node):
    def __init__(self):
        super().__init__("main_controller")
        # QoS配置
        self.qos_best_effort = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # 预设位置参数
        self.RED_BLOCKS = [
            (7.632928, 5.523903, False),
            (9.604514, -3.707741, False),
            (-5.735783, 5.507306, False),
            (-8.702837, 1.000008, False),
            (-10.748330, 4.014158, False)
        ]
        self.BLUE_BLOCKS = [
            (8.464876, -7.097603, False),
            (5.040937, -7.441163, False),
            (-1.478995, 6.646223, False),
            (-9.664453, -3.267239, False),
            (-3.703343, 0.829596, False)
        ]
        self.AREA_COORDS = {
            "A": (3.143086, -5.807858),
            "B": (-1.196544, -6.485499),
            "C": (-6.873777, -7.785160)
        }

        # 核心变量
        self.current_robot_pose = (0.0, 0.0)
        self.target_blocks = []
        self.GRASP_OFFSET = 0.4
        self.OFFSET_AXIS = "x"
        self.retry_timer: Timer = None
        self.grasp_timer: Timer = None
        self.place_timer: Timer = None
        self.current_task = None  # 当前执行任务
        self.task_queue = []      # 原始任务队列
        self.optimized_tasks = [] # 优化后的任务执行顺序
        self.completed_num = 0    # 当前任务已完成数量
        self.current_step = "WAIT_TASK"  # 状态：WAIT_TASK/NAV_TO_BLOCK/GRASP/NAV_TO_AREA/PLACE
        self.selected_block = None
        self.grasp_confirmed = False
        self.place_confirmed = False
        self.max_grasp_retry = 3
        self.current_grasp_retry = 0
        self.closest_cache = None
        self.cache_expire = 2.0
        self.last_cache_time = 0.0
        
        # 超时设置
        self.GRASP_TIMEOUT = 8.0
        self.PLACE_TIMEOUT = 8.0

        # 动态避障相关参数
        self.ROBOT_RADIUS = 0.22  # 机器人半径（米）
        self.OBSTACLE_RADIUS = 0.375  # 障碍物半径（0.75m的一半）
        self.SAFETY_MARGIN = 0.1  # 安全余量
        self.MIN_SAFE_DISTANCE = self.ROBOT_RADIUS + self.OBSTACLE_RADIUS + self.SAFETY_MARGIN
        self.ROBOT_SPEED = 0.5  # 机器人速度（m/s）
        self.MAX_WAIT_TIME = 15.0  # 最大等待时间（秒）
        self.OBSTACLE_CHECK_INTERVAL = 0.05  # 障碍物检查间隔（秒）
        
        # 障碍物管理
        self.obstacles = {}
        self._initialize_obstacles()
        self.safe_calculator = SafePassageCalculator()
        
        # 避障状态
        self.emergency_stop = False
        self.navigation_paused = False
        self.waiting_for_safe_window = False
        self.wait_start_time = None
        self.wait_duration = 0.0
        self.current_nav_target = None
        self.obstacle_timer = None

        # 订阅器
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_callback, self.qos_best_effort
        )
        self.chat_sub = self.create_subscription(
            String, "/chat", self._chat_callback, self.qos_best_effort
        )
        self.nav_status_sub = self.create_subscription(
            String, "/nav_status", self._nav_status_callback, 10
        )
        self.arm_status_sub = self.create_subscription(
            String, "/arm_status", self._arm_status_callback, 10
        )
        
        # 障碍物位置订阅器
        self.obstacle2_pose_sub = self.create_subscription(
            Pose, "/obstacle2/current_pose", lambda msg: self._obstacle_pose_callback("obstacle2", msg), 10
        )
        self.obstacle3_pose_sub = self.create_subscription(
            Pose, "/obstacle3/current_pose", lambda msg: self._obstacle_pose_callback("obstacle3", msg), 10
        )

        # 发布器
        self.target_cube_pub = self.create_publisher(String, "/current_target_cube", 10)
        self.nav_target_pub = self.create_publisher(String, "/manual_nav_target", 10)
        self.arm_cargo_pub = self.create_publisher(String, "/nav_done_cargo", 10)
        self.arm_area_pub = self.create_publisher(String, "/nav_done_area", 10)
        self.emergency_stop_pub = self.create_publisher(Bool, "/emergency_stop", 10)
        self.foxglove_pubs = {
            "color": self.create_publisher(Int32, "/color", 10),
            "ask": self.create_publisher(Int32, "/ask", 10),
            "pick": self.create_publisher(Int32, "/pick", 10),
            "cur": self.create_publisher(Int32, "/cur", 10)
        }

        # 启动障碍物检测定时器
        self.start_obstacle_detection()

        self.get_logger().info("主控节点（支持智能轨迹避障）启动成功")

    def _initialize_obstacles(self):
        """初始化障碍物参数"""
        # obstacle2参数（从room.world获取）
        obstacle2 = ObstacleTrajectoryPredictor("obstacle2")
        obstacle2.start_x = -9.0
        obstacle2.start_y = -4.2
        obstacle2.end_x = -5.0
        obstacle2.end_y = -3.3
        obstacle2.speed = 0.3
        
        # 重新计算轨迹向量
        dx = obstacle2.end_x - obstacle2.start_x
        dy = obstacle2.end_y - obstacle2.start_y
        distance = math.hypot(dx, dy)
        if distance > 0.001:
            obstacle2.direction_unit_vector = (dx / distance, dy / distance)
        obstacle2.trajectory_vector = (dx, dy)
        obstacle2.trajectory_length = distance
        
        # obstacle3参数（从room.world获取）
        obstacle3 = ObstacleTrajectoryPredictor("obstacle3")
        obstacle3.start_x = -2.5
        obstacle3.start_y = 3.0
        obstacle3.end_x = -2.5
        obstacle3.end_y = -2.0
        obstacle3.speed = 0.3
        
        # 重新计算轨迹向量
        dx3 = obstacle3.end_x - obstacle3.start_x
        dy3 = obstacle3.end_y - obstacle3.start_y
        distance3 = math.hypot(dx3, dy3)
        if distance3 > 0.001:
            obstacle3.direction_unit_vector = (dx3 / distance3, dy3 / distance3)
        obstacle3.trajectory_vector = (dx3, dy3)
        obstacle3.trajectory_length = distance3
        
        self.obstacles = {
            "obstacle2": obstacle2,
            "obstacle3": obstacle3
        }

    def start_obstacle_detection(self):
        """启动障碍物检测定时器"""
        if self.obstacle_timer:
            self.obstacle_timer.cancel()
            self.obstacle_timer.destroy()
        
        self.obstacle_timer = self.create_timer(
            self.OBSTACLE_CHECK_INTERVAL, self._check_obstacles
        )
        self.get_logger().info(f"障碍物检测定时器启动，间隔：{self.OBSTACLE_CHECK_INTERVAL}秒")

    def _obstacle_pose_callback(self, obstacle_name, msg):
        """障碍物位置回调函数"""
        if obstacle_name not in self.obstacles:
            return
            
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.obstacles[obstacle_name].update_position(
            msg.position.x, msg.position.y, current_time
        )

    def _check_obstacles(self):
        """检查障碍物并执行避障逻辑"""
        if self.current_step not in ["NAV_TO_BLOCK", "NAV_TO_AREA"]:
            return
            
        if not self.current_nav_target:
            return
            
        robot_x, robot_y = self.current_robot_pose
        target_x, target_y = self.current_nav_target
        
        # 检查所有障碍物
        for obstacle_name, obstacle in self.obstacles.items():
            if not obstacle.active:
                continue
                
            # 检查紧急碰撞风险
            distance = math.hypot(robot_x - obstacle.x, robot_y - obstacle.y)
            if distance < self.MIN_SAFE_DISTANCE * 0.8:
                self._emergency_stop(f"距离障碍物{obstacle_name}过近：{distance:.2f}米")
                return
            
            # 检查路径是否与障碍物轨迹相交
            if self._is_path_intersecting_trajectory(robot_x, robot_y, target_x, target_y, obstacle):
                self.get_logger().info(f"检测到路径与障碍物{obstacle_name}轨迹相交")
                
                # 计算安全通过窗口
                safe_result = self.safe_calculator.find_safe_passage_window(
                    robot_x, robot_y, target_x, target_y, obstacle, self.MAX_WAIT_TIME
                )
                
                if safe_result['safe']:
                    if safe_result['action'] == 'wait_and_pass' and safe_result['wait_time'] > 0.1:
                        self._wait_for_safe_window(safe_result, obstacle_name)
                    elif safe_result['action'] == 'wait_for_clearance':
                        self._wait_for_clearance(safe_result, obstacle_name)
                    else:
                        # 可以安全通过，继续导航
                        self._resume_navigation()
                else:
                    # 找不到安全窗口，尝试绕开
                    self._attempt_detour(robot_x, robot_y, target_x, target_y, obstacle)
                return
        
        # 如果之前在等待，现在可以继续
        if self.navigation_paused and not self.waiting_for_safe_window:
            self._resume_navigation()

    def _is_path_intersecting_trajectory(self, robot_x, robot_y, target_x, target_y, obstacle):
        """判断机器人路径是否与障碍物轨迹相交"""
        # 投影机器人起点和终点到障碍物轨迹
        proj_start = obstacle.project_point_to_line(robot_x, robot_y)
        proj_end = obstacle.project_point_to_line(target_x, target_y)
        
        # 计算投影点之间的距离
        proj_distance = math.hypot(proj_start[0] - proj_end[0], proj_start[1] - proj_end[1])
        
        # 如果投影距离很小，说明路径基本平行于轨迹
        if proj_distance < 0.1:
            return False
        
        # 检查机器人路径是否靠近轨迹
        min_distance_to_trajectory = float("inf")
        # 采样检查路径上的多个点
        for t in [0.0, 0.25, 0.5, 0.75, 1.0]:
            x = robot_x + t * (target_x - robot_x)
            y = robot_y + t * (target_y - robot_y)
            proj_x, proj_y = obstacle.project_point_to_line(x, y)
            distance = math.hypot(x - proj_x, y - proj_y)
            min_distance_to_trajectory = min(min_distance_to_trajectory, distance)
        
        return min_distance_to_trajectory < self.MIN_SAFE_DISTANCE * 1.5

    def _emergency_stop(self, reason):
        """紧急停止"""
        if not self.emergency_stop:
            self.get_logger().warn(f"紧急停止！{reason}")
            self.emergency_stop = True
            self.navigation_paused = True
            self.emergency_stop_pub.publish(Bool(data=True))
            # 发布空目标以停止导航
            self.nav_target_pub.publish(String(data=json.dumps({"type": "pause"})))

    def _wait_for_safe_window(self, safe_result, obstacle_name):
        """等待安全通过窗口"""
        if not self.waiting_for_safe_window:
            self.get_logger().info(f"等待安全窗口通过障碍物{obstacle_name}，预计等待{safe_result['wait_time']:.1f}秒")
            self.waiting_for_safe_window = True
            self.wait_start_time = datetime.now()
            self.wait_duration = safe_result['wait_time']
            self.navigation_paused = True
            # 暂停导航
            self.nav_target_pub.publish(String(data=json.dumps({"type": "pause"})))
        
        # 检查等待时间是否已到
        elapsed_time = (datetime.now() - self.wait_start_time).total_seconds()
        if elapsed_time >= self.wait_duration - 0.1:  # 提前0.1秒恢复
            self.get_logger().info(f"安全窗口到达，恢复导航通过障碍物{obstacle_name}")
            self._resume_navigation()

    def _wait_for_clearance(self, safe_result, obstacle_name):
        """等待障碍物离开危险区域"""
        if not self.waiting_for_safe_window:
            self.get_logger().info(f"当前位置在障碍物{obstacle_name}危险区域内，等待离开，预计等待{safe_result['wait_time']:.1f}秒")
            self.waiting_for_safe_window = True
            self.wait_start_time = datetime.now()
            self.wait_duration = safe_result['wait_time']
            self.navigation_paused = True
        
        # 检查等待时间是否已到
        elapsed_time = (datetime.now() - self.wait_start_time).total_seconds()
        if elapsed_time >= self.wait_duration:
            self.get_logger().info(f"障碍物{obstacle_name}已离开，恢复导航")
            self._resume_navigation()

    def _attempt_detour(self, robot_x, robot_y, target_x, target_y, obstacle):
        """尝试绕开障碍物"""
        self.get_logger().info(f"无法找到安全窗口，尝试绕开障碍物{obstacle.name}")
        
        # 计算绕开点（在障碍物轨迹的垂直方向）
        detour_distance = self.MIN_SAFE_DISTANCE + 0.2
        dx_trajectory, dy_trajectory = obstacle.trajectory_vector
        trajectory_length = math.hypot(dx_trajectory, dy_trajectory)
        
        if trajectory_length < 0.001:
            self.get_logger().warn("无法计算绕开路径，障碍物轨迹长度为0")
            return
        
        # 计算垂直于轨迹的单位向量（两个方向）
        perp_unit_vector1 = (-dy_trajectory / trajectory_length, dx_trajectory / trajectory_length)
        perp_unit_vector2 = (dy_trajectory / trajectory_length, -dx_trajectory / trajectory_length)
        
        # 计算两个可能的绕开点
        midpoint_x = (robot_x + target_x) / 2
        midpoint_y = (robot_y + target_y) / 2
        
        detour_point1 = (
            midpoint_x + perp_unit_vector1[0] * detour_distance,
            midpoint_y + perp_unit_vector1[1] * detour_distance
        )
        
        detour_point2 = (
            midpoint_x + perp_unit_vector2[0] * detour_distance,
            midpoint_y + perp_unit_vector2[1] * detour_distance
        )
        
        # 选择更好的绕开点（距离更短）
        distance1 = (self.calculate_distance((robot_x, robot_y), detour_point1) + 
                    self.calculate_distance(detour_point1, (target_x, target_y)))
        distance2 = (self.calculate_distance((robot_x, robot_y), detour_point2) + 
                    self.calculate_distance(detour_point2, (target_x, target_y)))
        
        if distance1 < distance2:
            detour_point = detour_point1
        else:
            detour_point = detour_point2
        
        self.get_logger().info(f"使用绕开点：({detour_point[0]:.2f}, {detour_point[1]:.2f})")
        
        # 导航到绕开点
        self.current_nav_target = detour_point
        nav_msg = String()
        nav_msg.data = json.dumps({"type": "custom", "x": detour_point[0], "y": detour_point[1], "yaw": 0.0})
        self.nav_target_pub.publish(nav_msg)
        
        # 记录原始目标，以便绕开后继续
        self.original_nav_target = (target_x, target_y)
        self.navigation_paused = False
        self.waiting_for_safe_window = False

    def _resume_navigation(self):
        """恢复导航"""
        self.emergency_stop = False
        self.navigation_paused = False
        self.waiting_for_safe_window = False
        self.wait_start_time = None
        self.emergency_stop_pub.publish(Bool(data=False))
        
        if self.current_nav_target:
            nav_msg = String()
            nav_msg.data = json.dumps({"type": "custom", "x": self.current_nav_target[0], "y": self.current_nav_target[1], "yaw": 0.0})
            self.nav_target_pub.publish(nav_msg)

    # 计算两点之间的距离
    def calculate_distance(self, point1, point2):
        return math.hypot(point1[0] - point2[0], point1[1] - point2[1])

    # 生成抓取偏移位置
    def get_grasp_position(self, block_pos):
        if self.OFFSET_AXIS == "x":
            return (block_pos[0] - self.GRASP_OFFSET, block_pos[1])
        else:
            return (block_pos[0], block_pos[1] - self.GRASP_OFFSET)

    # 获取符合条件的物块列表
    def get_available_blocks(self, color):
        blocks = self.RED_BLOCKS if color == "red" else self.BLUE_BLOCKS
        available = []
        for i, (x, y, is_grasped) in enumerate(blocks):
            if not is_grasped:
                available.append((x, y, i))
        return available

    # 为单个任务分配最佳物块（考虑已使用的物块）
    def assign_best_block_to_task(self, task, used_blocks=None):
        if used_blocks is None:
            used_blocks = set()
            
        color = task["color"]
        target_area = task["to"]
        area_pos = self.AREA_COORDS[target_area]
        
        best_block = None
        min_cost = float("inf")
        
        # 获取所有可用物块
        available_blocks = self.get_available_blocks(color)
        
        for block_pos in available_blocks:
            block_x, block_y, block_idx = block_pos
            if block_idx in used_blocks:
                continue
                
            # 计算抓取位置
            grasp_pos = self.get_grasp_position((block_x, block_y))
            
            # 计算成本（距离）
            # 从当前位置到抓取位置的距离
            distance = self.calculate_distance(self.current_robot_pose, grasp_pos)
            # 从抓取位置到目标区域的距离
            distance += self.calculate_distance(grasp_pos, area_pos)
            
            if distance < min_cost:
                min_cost = distance
                best_block = block_pos
        
        return best_block, min_cost

    # 计算任务序列的总成本
    def calculate_task_sequence_cost(self, tasks):
        total_cost = 0.0
        current_pos = self.current_robot_pose
        used_blocks = set()
        
        # 首先为每个任务分配最佳物块
        task_assignments = []
        for task in tasks:
            best_block, cost = self.assign_best_block_to_task(task, used_blocks)
            if not best_block:
                return float("inf")  # 无法完成所有任务
            
            block_x, block_y, block_idx = best_block
            target_area = task["to"]
            area_pos = self.AREA_COORDS[target_area]
            grasp_pos = self.get_grasp_position((block_x, block_y))
            
            task_assignments.append({
                "task": task,
                "block_pos": (block_x, block_y),
                "block_idx": block_idx,
                "grasp_pos": grasp_pos,
                "area_pos": area_pos
            })
            
            used_blocks.add(block_idx)
        
        # 然后计算路径成本
        for assignment in task_assignments:
            # 从当前位置到抓取位置
            total_cost += self.calculate_distance(current_pos, assignment["grasp_pos"])
            # 从抓取位置到目标区域
            total_cost += self.calculate_distance(assignment["grasp_pos"], assignment["area_pos"])
            # 更新当前位置
            current_pos = assignment["area_pos"]
        
        return total_cost, task_assignments

    # 优化多任务执行顺序
    def optimize_task_order(self, tasks):
        if len(tasks) <= 1:
            # 单任务不需要优化顺序，但仍需为每个物块分配最佳选择
            cost, assignments = self.calculate_task_sequence_cost(tasks)
            return assignments
        
        self.get_logger().info(f"正在优化{len(tasks)}个任务的执行顺序...")
        
        min_total_cost = float("inf")
        best_assignments = None
        
        # 生成所有可能的任务排列
        for task_order in permutations(tasks):
            cost, assignments = self.calculate_task_sequence_cost(task_order)
            if cost < min_total_cost:
                min_total_cost = cost
                best_assignments = assignments
        
        if best_assignments:
            self.get_logger().info(f"优化完成，最佳路径总距离：{min_total_cost:.2f}米")
            # 输出优化后的执行顺序
            order_info = []
            for i, assignment in enumerate(best_assignments, 1):
                task = assignment["task"]
                block_pos = assignment["block_pos"]
                order_info.append(f"{i}. {task['num']}个{task['color']}物块到{task['to']}区 (物块位置: {block_pos[0]:.2f},{block_pos[1]:.2f})")
            
            for info in order_info:
                self.get_logger().info(info)
        
        return best_assignments

    # 处理下一个优化后的任务
    def _process_next_optimized_task(self):
        if not self.optimized_tasks:
            self.get_logger().info("所有优化任务执行完成，等待新任务...")
            self.current_task = None
            self.current_step = "WAIT_TASK"
            self._update_foxglove()
            return

        # 取出第一个优化任务
        self.current_assignment = self.optimized_tasks.pop(0)
        self.current_task = self.current_assignment["task"]
        self.completed_num = 0
        self.current_step = "NAV_TO_BLOCK"
        
        # 初始化目标物块信息
        self.selected_block = (
            self.current_assignment["block_pos"][0],
            self.current_assignment["block_pos"][1],
            f"{self.current_task['color']}_cube_{self.current_assignment['block_idx'] + 1}",
            self.current_assignment["block_idx"]
        )

        # 重置状态变量
        self.grasp_confirmed = False
        self.place_confirmed = False
        self.current_grasp_retry = 0
        self.closest_cache = None
        self._clean_timers()
        self.navigation_paused = False
        self.emergency_stop = False
        self.waiting_for_safe_window = False
        self.wait_start_time = None
        self.current_nav_target = None
        self.original_nav_target = None

        color = self.current_task["color"]
        original_num = self.current_task.get("original_num", 1)
        current_index = self.current_task.get("current_index", 1)
        
        self.get_logger().info(
            f"开始执行优化任务：{current_index}/{original_num} 个{color}物块 → {self.current_task['to']}区"
        )
        self.get_logger().info(
            f"目标物块位置：({self.current_assignment['block_pos'][0]:.2f}, {self.current_assignment['block_pos'][1]:.2f})"
        )
        self._update_foxglove()
        self.navigate_to_block()

    # 导航到物块（使用优化路径）
    def navigate_to_block(self):
        self._clean_timers()

        if not self.current_assignment:
            return

        self.grasp_confirmed = False
        self.current_grasp_retry = 0
        self.closest_cache = None

        # 使用优化后的抓取位置
        grasp_pos = self.current_assignment["grasp_pos"]
        cube_name = self.selected_block[2]
        
        # 设置当前导航目标
        self.current_nav_target = grasp_pos
        self.original_nav_target = None

        # 发布导航目标
        nav_msg = String()
        nav_msg.data = json.dumps({"type": "custom", "x": grasp_pos[0], "y": grasp_pos[1], "yaw": 0.0})
        self.nav_target_pub.publish(nav_msg)
        self.target_cube_pub.publish(String(data=cube_name))
        self.current_step = "NAV_TO_BLOCK"
        self._update_foxglove()
        self.get_logger().info(f"导航到物块：{cube_name}（抓取位置：{grasp_pos[0]:.2f}, {grasp_pos[1]:.2f}）")

    # 导航到目标区域（使用优化路径）
    def navigate_to_area(self):
        if not self.current_assignment:
            return

        area_pos = self.current_assignment["area_pos"]
        area = self.current_task["to"]
        
        # 设置当前导航目标
        self.current_nav_target = area_pos
        self.original_nav_target = None
        
        nav_msg = String()
        nav_msg.data = json.dumps({"type": "custom", "x": area_pos[0], "y": area_pos[1], "yaw": 0.0})
        self.nav_target_pub.publish(nav_msg)
        self.current_step = "NAV_TO_AREA"
        self._update_foxglove()
        self.get_logger().info(f"导航到目标区域 {area}：({area_pos[0]:.2f}, {area_pos[1]:.2f})")

    # AMCL定位回调
    @staticmethod
    def _amcl_callback_impl(self_ref, msg):
        self = self_ref()
        if not self:
            return
        new_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if abs(new_pose[0] - self.current_robot_pose[0]) > 0.01 or \
           abs(new_pose[1] - self.current_robot_pose[1]) > 0.01:
            if self.current_robot_pose == (0.0, 0.0):
                self.get_logger().info(f"初始定位：({new_pose[0]:.2f}, {new_pose[1]:.2f})")
            self.current_robot_pose = new_pose
            self.closest_cache = None

    def _amcl_callback(self, msg):
        self_ref = weakref.ref(self)
        self._amcl_callback_impl(self_ref, msg)
        del self_ref

    # 聊天指令回调（接收任务）
    @staticmethod
    def _chat_callback_impl(self_ref, msg):
        self = self_ref()
        if not self:
            return
        try:
            task_json = json.loads(msg.data)
            # 支持单个任务对象或任务列表
            if not isinstance(task_json, list):
                task_json = [task_json]
            
            # 验证任务格式并添加到队列
            valid_tasks = []
            for task in task_json:
                required = ["color", "num", "to"]
                if all(k in task for k in required):
                    # 标准化参数
                    task["color"] = task["color"].lower()
                    task["to"] = task["to"].upper()
                    valid_tasks.append(task)
                else:
                    self.get_logger().error(f"任务格式错误（缺少color/num/to）：{task}")

            if valid_tasks:
                self.task_queue.extend(valid_tasks)
                self.get_logger().info(f"接收{len(valid_tasks)}个任务，队列长度：{len(self.task_queue)}")
                
                # 如果当前没有正在执行的任务，立即优化并开始处理
                if self.current_task is None and self.current_step == "WAIT_TASK":
                    self._optimize_and_process_tasks()

        except json.JSONDecodeError:
            self.get_logger().error("任务解析失败（非JSON格式）")

    def _chat_callback(self, msg):
        self_ref = weakref.ref(self)
        self._chat_callback_impl(self_ref, msg)
        del self_ref

    # 优化并处理任务
    def _optimize_and_process_tasks(self):
        if not self.task_queue:
            return
        
        # 复制当前任务队列并清空
        current_tasks = self.task_queue.copy()
        self.task_queue = []
        
        # 展开多数量任务为单个任务
        expanded_tasks = []
        for task in current_tasks:
            num = task["num"]
            for _ in range(num):
                expanded_tasks.append({
                    "color": task["color"],
                    "num": 1,  # 单个物块
                    "to": task["to"],
                    "original_num": num  # 记录原始数量
                })
        
        # 优化任务执行顺序
        self.optimized_tasks = self.optimize_task_order(expanded_tasks)
        
        if self.optimized_tasks:
            # 开始执行优化后的第一个任务
            self._process_next_optimized_task()
        else:
            self.get_logger().error("无法优化任务顺序，使用原始顺序执行")
            # 回退到原始顺序
            self.optimized_tasks = []
            for task in expanded_tasks:
                color = task["color"]
                blocks = self.RED_BLOCKS if color == "red" else self.BLUE_BLOCKS
                available = self.get_available_blocks(color)
                if available:
                    block_x, block_y, block_idx = available[0]
                    area_pos = self.AREA_COORDS[task["to"]]
                    grasp_pos = self.get_grasp_position((block_x, block_y))
                    self.optimized_tasks.append({
                        "task": task,
                        "block_pos": (block_x, block_y),
                        "block_idx": block_idx,
                        "grasp_pos": grasp_pos,
                        "area_pos": area_pos
                    })
            
            if self.optimized_tasks:
                self._process_next_optimized_task()

    # 导航状态回调
    @staticmethod
    def _nav_status_callback_impl(self_ref, msg):
        self = self_ref()
        if not self:
            return
        status = msg.data.strip()
        
        # 如果导航被暂停，忽略导航状态更新
        if self.navigation_paused or self.emergency_stop:
            return
            
        if status == "succeeded":
            # 检查是否是绕开点
            if self.original_nav_target:
                self.get_logger().info("到达绕开点，继续导航到原始目标")
                self.current_nav_target = self.original_nav_target
                self.original_nav_target = None
                nav_msg = String()
                nav_msg.data = json.dumps({"type": "custom", "x": self.current_nav_target[0], "y": self.current_nav_target[1], "yaw": 0.0})
                self.nav_target_pub.publish(nav_msg)
                return
            
            if self.current_step == "NAV_TO_BLOCK":
                self.get_logger().info("到达物块位置，准备抓取")
                self.current_step = "GRASP"
                self.trigger_grasp()
            elif self.current_step == "NAV_TO_AREA":
                self.get_logger().info("到达目标区域，准备放置")
                self.current_step = "PLACE"
                self.trigger_place()
        elif status == "failed":
            self.get_logger().warn("导航失败，重试当前步骤")
            if self.current_step == "NAV_TO_BLOCK":
                self.navigate_to_block()
            elif self.current_step == "NAV_TO_AREA":
                self.navigate_to_area()

    def _nav_status_callback(self, msg):
        self_ref = weakref.ref(self)
        self._nav_status_callback_impl(self_ref, msg)
        del self_ref

    # 机械臂状态回调
    @staticmethod
    def _arm_status_callback_impl(self_ref, msg):
        self = self_ref()
        if not self:
            return
        status = msg.data.strip()
        if status == "grasp_succeeded":
            self.grasp_confirmed = True
            self.get_logger().info("机械臂抓取成功")
        elif status == "place_succeeded":
            self.place_confirmed = True
            self.get_logger().info("机械臂放置成功")
        elif status in ["grasp_failed", "place_failed"]:
            self.get_logger().warn(f"机械臂{status}")

    def _arm_status_callback(self, msg):
        self_ref = weakref.ref(self)
        self._arm_status_callback_impl(self_ref, msg)
        del self_ref

    # 触发抓取
    def trigger_grasp(self):
        if self.current_grasp_retry >= self.max_grasp_retry:
            self.get_logger().error(f"抓取重试达{self.max_grasp_retry}次，跳过该物块")
            self.current_grasp_retry = 0
            self._process_next_optimized_task()
            return

        self.get_logger().info(f"触发抓取（重试次数：{self.current_grasp_retry}/{self.max_grasp_retry}）")
        self.arm_cargo_pub.publish(String(data="arrived_at_cargo"))
        self_ref = weakref.ref(self)
        self.grasp_timer = self.create_timer(self.GRASP_TIMEOUT, lambda: self._check_grasp(self_ref))

    # 检查抓取结果
    @staticmethod
    def _check_grasp(self_ref):
        self = self_ref()
        if not self:
            return
        # 清理定时器
        if self.grasp_timer:
            self.grasp_timer.cancel()
            self.grasp_timer.destroy()
            self.grasp_timer = None

        if self.grasp_confirmed:
            # 标记物块为已抓取
            block_idx = self.current_assignment["block_idx"]
            color = self.current_task["color"]
            blocks = self.RED_BLOCKS if color == "red" else self.BLUE_BLOCKS
            x, y, _ = blocks[block_idx]
            if color == "red":
                self.RED_BLOCKS[block_idx] = (x, y, True)
            else:
                self.BLUE_BLOCKS[block_idx] = (x, y, True)
            
            self.current_step = "NAV_TO_AREA"
            self.closest_cache = None
            self.navigate_to_area()
        else:
            # 重试抓取
            self.current_grasp_retry += 1
            self.get_logger().warn(f"抓取超时/失败，准备重试（{self.current_grasp_retry}/{self.max_grasp_retry}）")
            self.trigger_grasp()
        
        gc.collect()

    # 触发放置
    def trigger_place(self):
        self.get_logger().info("触发放置")
        self.place_confirmed = False
        self.arm_area_pub.publish(String(data="arrived_at_area"))
        self_ref = weakref.ref(self)
        self.place_timer = self.create_timer(self.PLACE_TIMEOUT, lambda: self._check_place(self_ref))

    # 检查放置结果
    @staticmethod
    def _check_place(self_ref):
        self = self_ref()
        if not self:
            return
        # 清理定时器
        if self.place_timer:
            self.place_timer.cancel()
            self.place_timer.destroy()
            self.place_timer = None

        if self.place_confirmed:
            self.completed_num += 1
            original_num = self.current_task.get("original_num", 1)
            current_index = self.current_task.get("current_index", 1)
            
            self.get_logger().info(f"放置完成（{current_index}/{original_num}）")
            
            # 直接处理下一个优化任务
            self._process_next_optimized_task()
        else:
            # 重试放置
            self.get_logger().warn("放置超时/失败，准备重试")
            self.trigger_place()
        
        gc.collect()

    # 清理定时器
    def _clean_timers(self):
        for timer in [self.retry_timer, self.grasp_timer, self.place_timer]:
            if timer:
                timer.cancel()
                timer.destroy()
        self.retry_timer = None
        self.grasp_timer = None
        self.place_timer = None
        gc.collect()

    # 更新foxglove可视化数据
    def _update_foxglove(self):
        if not self.current_task:
            for pub in self.foxglove_pubs.values():
                pub.publish(Int32(data=-1))
            self.foxglove_pubs["cur"].publish(Int32(data=0))
            return

        color = self.current_task["color"]
        color_data = 0 if color == "blue" else 1
        self.foxglove_pubs["color"].publish(Int32(data=color_data))
        self.foxglove_pubs["ask"].publish(Int32(data=color_data))

        # 更新抓取状态
        pick_data = -1 if self.current_step in ["NAV_TO_BLOCK", "WAIT_TASK"] else \
                    0 if self.current_step in ["GRASP", "NAV_TO_AREA"] else 1
        self.foxglove_pubs["pick"].publish(Int32(data=pick_data))

        # 更新步骤状态
        step_map = {
            "WAIT_TASK": 0,
            "NAV_TO_BLOCK": 1,
            "GRASP": 2,
            "NAV_TO_AREA": 3,
            "PLACE": 3
        }
        self.foxglove_pubs["cur"].publish(Int32(data=step_map[self.current_step]))

    # 节点销毁时清理资源
    def destroy_node(self):
        self._clean_timers()
        if self.obstacle_timer:
            self.obstacle_timer.cancel()
            self.obstacle_timer.destroy()
        self.target_blocks = []
        self.current_task = None
        self.task_queue = []
        self.optimized_tasks = []
        self.selected_block = None
        self.current_assignment = None
        super().destroy_node()
        gc.collect()


def main(args=None):
    rclpy.init(args=args)
    main_controller = MainController()
    try:
        rclpy.spin(main_controller)
    except KeyboardInterrupt:
        main_controller.get_logger().info("用户中断，关闭节点")
    finally:
        main_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()