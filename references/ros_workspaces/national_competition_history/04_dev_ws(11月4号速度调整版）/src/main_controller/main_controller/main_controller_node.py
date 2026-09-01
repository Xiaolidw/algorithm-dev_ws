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

class PhysicsBasedObstaclePredictor:
    """基于物理模拟的障碍物轨迹预测器"""
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
        self.nominal_speed = 0.3  # 标称速度
        
        # 物理特性参数
        self.speed_variation = 0.1  # 速度波动范围
        self.position_tolerance = 0.2  # 位置容差（到达目标的判定）
        
        # 轨迹计算
        self.trajectory_vector = (self.end_x - self.start_x, self.end_y - self.start_y)
        self.trajectory_length = math.hypot(*self.trajectory_vector)
        if self.trajectory_length > 0.001:
            self.direction_unit_vector = (
                self.trajectory_vector[0] / self.trajectory_length,
                self.trajectory_vector[1] / self.trajectory_length
            )
        else:
            self.direction_unit_vector = (0.0, 0.0)
        
        # 运动状态
        self.moving_to_end = True  # 当前是否向终点移动
        self.speed_history = []    # 速度历史记录
        self.max_history_size = 5  # 最大历史记录数量
        
    def update_position(self, x, y, current_time):
        """更新位置并计算速度（适应物理模拟的波动）"""
        if self.active:
            time_diff = current_time - self.last_update_time
            if time_diff > 0.01:
                # 计算当前速度
                current_vx = (x - self.last_x) / time_diff
                current_vy = (y - self.last_y) / time_diff
                
                # 更新速度历史
                current_speed = math.hypot(current_vx, current_vy)
                self.speed_history.append(current_speed)
                if len(self.speed_history) > self.max_history_size:
                    self.speed_history.pop(0)
                
                # 平滑速度计算（去除异常值）
                if len(self.speed_history) > 2:
                    # 使用中位数滤波减少噪声影响
                    sorted_speeds = sorted(self.speed_history)
                    median_speed = sorted_speeds[len(sorted_speeds) // 2]
                    
                    # 如果当前速度与中位数差异太大，使用中位数
                    if abs(current_speed - median_speed) > self.speed_variation * 2:
                        # 保持方向，调整速度大小
                        speed_ratio = median_speed / max(current_speed, 0.01)
                        self.vx = current_vx * speed_ratio
                        self.vy = current_vy * speed_ratio
                    else:
                        self.vx = current_vx
                        self.vy = current_vy
                else:
                    self.vx = current_vx
                    self.vy = current_vy
                
                # 更新运动方向状态（检查是否到达目标）
                self._update_motion_direction(x, y)
        
        self.last_x = self.x
        self.last_y = self.y
        self.x = x
        self.y = y
        self.last_update_time = current_time
        self.active = True
        
    def _update_motion_direction(self, x, y):
        """更新运动方向状态"""
        # 计算到起点和终点的距离
        distance_to_start = math.hypot(x - self.start_x, y - self.start_y)
        distance_to_end = math.hypot(x - self.end_x, y - self.end_y)
        
        # 检查是否到达目标点
        if self.moving_to_end and distance_to_end < self.position_tolerance:
            self.moving_to_end = False
            # 到达终点，速度应该减小
            self.vx *= 0.3
            self.vy *= 0.3
        elif not self.moving_to_end and distance_to_start < self.position_tolerance:
            self.moving_to_end = True
            # 到达起点，速度应该减小
            self.vx *= 0.3
            self.vy *= 0.3
    
    def predict_position_at_time(self, target_time):
        """预测指定时间的位置（考虑物理特性）"""
        if not self.active:
            return (self.x, self.y)
            
        time_diff = target_time - self.last_update_time
        if time_diff <= 0:
            return (self.x, self.y)
        
        # 基于当前速度预测
        predicted_x = self.x + self.vx * time_diff
        predicted_y = self.y + self.vy * time_diff
        
        # 检查是否会到达边界
        distance_to_target = 0.0
        target_x = 0.0
        target_y = 0.0
        
        if self.moving_to_end:
            target_x, target_y = self.end_x, self.end_y
        else:
            target_x, target_y = self.start_x, self.start_y
        
        distance_to_target = math.hypot(predicted_x - target_x, predicted_y - target_y)
        
        # 如果预测位置超过目标，考虑反弹
        if distance_to_target < self.position_tolerance + 0.1:
            # 计算到达目标的时间
            current_distance = math.hypot(self.x - target_x, self.y - target_y)
            if current_distance < 0.01:
                # 已经在目标附近，预测为目标位置
                return (target_x, target_y)
            
            speed_magnitude = math.hypot(self.vx, self.vy)
            if speed_magnitude < 0.01:
                # 速度很小，预测为当前位置
                return (self.x, self.y)
            
            time_to_target = current_distance / speed_magnitude
            remaining_time = time_diff - time_to_target
            
            if remaining_time > 0:
                # 反弹后的运动（速度会减小）
                if self.moving_to_end:
                    new_target_x, new_target_y = self.start_x, self.start_y
                else:
                    new_target_x, new_target_y = self.end_x, self.end_y
                
                # 反弹后的速度（考虑能量损失）
                rebound_dx = new_target_x - target_x
                rebound_dy = new_target_y - target_y
                rebound_length = math.hypot(rebound_dx, rebound_dy)
                
                if rebound_length > 0.001:
                    rebound_speed = speed_magnitude * 0.8  # 反弹后速度减小20%
                    rebound_vx = (rebound_dx / rebound_length) * rebound_speed
                    rebound_vy = (rebound_dy / rebound_length) * rebound_speed
                    
                    # 计算反弹后的位置
                    predicted_x = target_x + rebound_vx * remaining_time
                    predicted_y = target_y + rebound_vy * remaining_time
        
        return (predicted_x, predicted_y)
    
    def get_current_trajectory(self):
        """获取当前轨迹段"""
        if self.moving_to_end:
            return (self.x, self.y, self.end_x, self.end_y)
        else:
            return (self.x, self.y, self.start_x, self.start_y)
    
    def is_position_on_trajectory(self, x, y, tolerance=0.2):
        """判断点是否在轨迹附近"""
        px, py = self.project_point_to_line(x, y)
        distance_to_line = math.hypot(x - px, y - py)
        return distance_to_line < tolerance
    
    def project_point_to_line(self, x, y):
        """将点投影到轨迹线上"""
        if self.moving_to_end:
            x1, y1, x2, y2 = self.x, self.y, self.end_x, self.end_y
        else:
            x1, y1, x2, y2 = self.x, self.y, self.start_x, self.start_y
        
        A = y2 - y1
        B = x1 - x2
        C = x2 * y1 - x1 * y2
        
        denominator = A**2 + B**2
        if denominator < 0.001:
            return (x1, y1)
            
        t = (A * x + B * y + C) / denominator
        proj_x = x - A * t
        proj_y = y - B * t
        
        # 确保投影点在当前运动方向上
        if self.moving_to_end:
            # 向终点移动，投影点不应超过终点
            if (proj_x - x1) * (x2 - x1) + (proj_y - y1) * (y2 - y1) < 0:
                return (x1, y1)
            if (proj_x - x2) * (x1 - x2) + (proj_y - y2) * (y1 - y2) < 0:
                return (x2, y2)
        else:
            # 向起点移动，投影点不应超过起点
            if (proj_x - x1) * (x2 - x1) + (proj_y - y1) * (y2 - y1) < 0:
                return (x1, y1)
            if (proj_x - x2) * (x1 - x2) + (proj_y - y2) * (y1 - y2) < 0:
                return (x2, y2)
        
        return (proj_x, proj_y)
    
    def get_estimated_time_to_point(self, target_x, target_y):
        """估算到达目标点的时间"""
        distance = math.hypot(target_x - self.x, target_y - self.y)
        current_speed = math.hypot(self.vx, self.vy)
        
        if current_speed < 0.01:
            # 如果速度很小，使用标称速度估算
            current_speed = self.nominal_speed * 0.5
        
        return distance / current_speed
    
    def get_speed_confidence(self):
        """获取速度预测的置信度"""
        if len(self.speed_history) < 3:
            return 0.5  # 历史数据不足，置信度中等
        
        # 计算速度标准差
        avg_speed = sum(self.speed_history) / len(self.speed_history)
        variance = sum((s - avg_speed)**2 for s in self.speed_history) / len(self.speed_history)
        std_dev = math.sqrt(variance)
        
        # 标准差越小，置信度越高
        max_std_dev = self.speed_variation * 0.5
        confidence = max(0.1, 1.0 - (std_dev / max_std_dev))
        return min(confidence, 0.95)


class AdaptiveSafePassageCalculator:
    """自适应安全通过计算器（考虑物理特性）"""
    def __init__(self):
        # 机器人参数
        self.robot_radius = 0.22  # m
        self.robot_speed = 0.5    # m/s
        
        # 障碍物参数（基于新的物理特性）
        self.obstacle_radius = 0.375  # m (0.75m的一半)
        self.obstacle_mass_factor = 0.3  # 质量较轻，可以被撞开
        
        # 安全参数（自适应调整）
        self.base_safety_margin = 0.15  # 基础安全余量
        self.dynamic_safety_factor = 1.0  # 动态安全系数
        
    def calculate_safe_distance(self, obstacle_speed, robot_speed):
        """计算动态安全距离"""
        # 基础安全距离
        base_distance = self.robot_radius + self.obstacle_radius + self.base_safety_margin
        
        # 根据相对速度调整安全距离
        relative_speed = math.hypot(obstacle_speed - robot_speed, 0)  # 简化为一维相对速度
        speed_factor = 1.0 + min(relative_speed * 0.5, 0.5)
        
        # 根据障碍物质量调整（质量轻可以减小安全距离）
        mass_factor = 1.0 - self.obstacle_mass_factor * 0.5
        
        # 动态安全距离
        safe_distance = base_distance * speed_factor * mass_factor
        
        # 确保最小安全距离
        return max(safe_distance, 0.5)  # 最小0.5米
    
    def is_safe_position(self, robot_x, robot_y, obstacle_x, obstacle_y, obstacle_speed=0.0):
        """判断位置是否安全"""
        distance = math.hypot(robot_x - obstacle_x, robot_y - obstacle_y)
        safe_distance = self.calculate_safe_distance(obstacle_speed, self.robot_speed)
        return distance >= safe_distance
    
    def is_collision_acceptable(self, robot_x, robot_y, obstacle_x, obstacle_y):
        """判断碰撞是否可接受（考虑障碍物可被撞开）"""
        distance = math.hypot(robot_x - obstacle_x, robot_y - obstacle_y)
        # 如果距离非常近，但障碍物质量轻，可以接受轻微碰撞
        return distance >= (self.robot_radius + self.obstacle_radius) * 0.8
    
    def calculate_passage_time(self, start_x, start_y, end_x, end_y):
        """计算通过路径的时间"""
        distance = math.hypot(end_x - start_x, end_y - start_y)
        return distance / self.robot_speed
    
    def find_safe_passage_window(self, robot_x, robot_y, target_x, target_y, obstacle, max_lookahead=12.0):
        """寻找安全通过窗口（适应物理模拟的障碍物）"""
        # 计算机器人路径与障碍物轨迹的关系
        robot_path_length = math.hypot(target_x - robot_x, target_y - robot_y)
        if robot_path_length < 0.1:
            return {'safe': True, 'action': 'already_at_target'}
        
        # 检查是否需要避障
        obstacle_proj = obstacle.project_point_to_line(robot_x, robot_y)
        distance_to_trajectory = math.hypot(robot_x - obstacle_proj[0], robot_y - obstacle_proj[1])
        
        # 如果距离轨迹足够远，不需要避障
        current_obstacle_speed = math.hypot(obstacle.vx, obstacle.vy)
        safe_distance = self.calculate_safe_distance(current_obstacle_speed, self.robot_speed)
        
        if distance_to_trajectory > safe_distance * 1.2:
            return {'safe': True, 'action': 'direct_pass'}
        
        # 检查当前位置是否安全
        if not self.is_safe_position(robot_x, robot_y, obstacle.x, obstacle.y, current_obstacle_speed):
            return self._handle_start_in_danger(robot_x, robot_y, target_x, target_y, obstacle, max_lookahead)
        
        # 计算机器人通过危险区域的时间
        danger_zone_start = obstacle.project_point_to_line(robot_x, robot_y)
        danger_zone_end = obstacle.project_point_to_line(target_x, target_y)
        
        time_to_danger = self.calculate_passage_time(robot_x, robot_y, danger_zone_start[0], danger_zone_start[1])
        time_through_danger = self.calculate_passage_time(danger_zone_start[0], danger_zone_start[1], 
                                                       danger_zone_end[0], danger_zone_end[1])
        
        # 寻找安全窗口（减少计算量，提高效率）
        current_time = obstacle.last_update_time
        safe_windows = []
        speed_confidence = obstacle.get_speed_confidence()
        
        # 根据速度置信度调整检查频率
        if speed_confidence > 0.7:
            time_steps = 60  # 高置信度，检查60个点（6秒）
        else:
            time_steps = 90  # 低置信度，检查90个点（9秒）
        
        for i in range(time_steps):
            t_offset = i * 0.1  # 每0.1秒检查一次
            if t_offset > max_lookahead:
                break
            
            # 机器人进入和离开危险区域的时间
            robot_entry_time = current_time + t_offset + time_to_danger
            robot_exit_time = robot_entry_time + time_through_danger
            
            # 预测障碍物在这段时间内的位置
            obstacle_entry_pos = obstacle.predict_position_at_time(robot_entry_time)
            obstacle_exit_pos = obstacle.predict_position_at_time(robot_exit_time)
            
            # 检查进入时的安全性
            entry_safe = self.is_safe_position(danger_zone_start[0], danger_zone_start[1],
                                             obstacle_entry_pos[0], obstacle_entry_pos[1],
                                             current_obstacle_speed)
            
            # 检查离开时的安全性
            exit_safe = self.is_safe_position(danger_zone_end[0], danger_zone_end[1],
                                            obstacle_exit_pos[0], obstacle_exit_pos[1],
                                            current_obstacle_speed)
            
            if entry_safe and exit_safe:
                # 简化检查：只检查关键时间点
                mid_time = (robot_entry_time + robot_exit_time) / 2
                mid_obstacle_pos = obstacle.predict_position_at_time(mid_time)
                mid_robot_pos = self._predict_robot_position(robot_x, robot_y, target_x, target_y, mid_time - current_time)
                
                mid_safe = self.is_safe_position(mid_robot_pos[0], mid_robot_pos[1],
                                              mid_obstacle_pos[0], mid_obstacle_pos[1],
                                              current_obstacle_speed)
                
                if mid_safe:
                    # 计算安全等级（考虑速度置信度）
                    safety_level = self._calculate_safety_level(robot_entry_time, robot_exit_time, obstacle, 
                                                               robot_x, robot_y, target_x, target_y)
                    safety_level *= speed_confidence  # 根据置信度调整安全等级
                    
                    safe_windows.append({
                        'start_delay': t_offset,
                        'total_time': t_offset + time_to_danger + time_through_danger,
                        'safety_level': safety_level,
                        'confidence': speed_confidence
                    })
        
        if safe_windows:
            # 选择最佳安全窗口（考虑安全性和等待时间）
            best_window = None
            best_score = -float("inf")
            
            for window in safe_windows:
                # 评分公式：安全等级 * 0.6 + (1 / (等待时间 + 1)) * 0.4
                time_score = 1.0 / (window['start_delay'] + 1.0)
                score = window['safety_level'] * 0.6 + time_score * 0.4
                
                if score > best_score:
                    best_score = score
                    best_window = window
            
            if best_window:
                return {
                    'safe': True,
                    'action': 'wait_and_pass',
                    'wait_time': best_window['start_delay'],
                    'total_time': best_window['total_time'],
                    'safety_level': best_window['safety_level'],
                    'confidence': best_window['confidence']
                }
        
        # 如果找不到完美的安全窗口，考虑可接受的碰撞
        if self._is_collision_acceptable(robot_x, robot_y, target_x, target_y, obstacle):
            return {
                'safe': True,
                'action': 'careful_pass',
                'reason': 'obstacle can be pushed away'
            }
        
        return {
            'safe': False,
            'action': 'need_detour',
            'reason': 'no safe passage window found'
        }
    
    def _handle_start_in_danger(self, robot_x, robot_y, target_x, target_y, obstacle, max_lookahead):
        """处理机器人起点在危险区域内的情况"""
        current_time = obstacle.last_update_time
        current_obstacle_speed = math.hypot(obstacle.vx, obstacle.vy)
        
        for t_offset in [i * 0.2 for i in range(int(max_lookahead / 0.2))]:
            future_time = current_time + t_offset
            future_obstacle_pos = obstacle.predict_position_at_time(future_time)
            
            if self.is_safe_position(robot_x, robot_y, future_obstacle_pos[0], future_obstacle_pos[1], current_obstacle_speed):
                return {
                    'safe': True,
                    'action': 'wait_for_clearance',
                    'wait_time': t_offset,
                    'total_time': t_offset + self.calculate_passage_time(robot_x, robot_y, target_x, target_y)
                }
        
        # 如果障碍物一直不离开，考虑是否可以撞开
        if self.obstacle_mass_factor < 0.5:  # 质量很轻
            return {
                'safe': True,
                'action': 'push_away',
                'reason': 'obstacle is light, can be pushed'
            }
        
        return {
            'safe': False,
            'action': 'need_detour',
            'reason': 'start position in danger zone'
        }
    
    def _is_collision_acceptable(self, robot_x, robot_y, target_x, target_y, obstacle):
        """判断是否可以接受碰撞"""
        # 检查障碍物是否在机器人前进方向上
        robot_dir_x = target_x - robot_x
        robot_dir_y = target_y - robot_y
        robot_dir_length = math.hypot(robot_dir_x, robot_dir_y)
        
        if robot_dir_length < 0.01:
            return False
        
        robot_dir_unit_x = robot_dir_x / robot_dir_length
        robot_dir_unit_y = robot_dir_y / robot_dir_length
        
        # 计算障碍物相对于机器人前进方向的位置
        obstacle_rel_x = obstacle.x - robot_x
        obstacle_rel_y = obstacle.y - robot_y
        
        # 投影到前进方向
        projection = obstacle_rel_x * robot_dir_unit_x + obstacle_rel_y * robot_dir_unit_y
        
        # 如果障碍物在机器人后方，不需要考虑
        if projection < -0.5:
            return False
        
        # 计算垂直距离
        perpendicular_distance = math.hypot(obstacle_rel_x - projection * robot_dir_unit_x,
                                          obstacle_rel_y - projection * robot_dir_unit_y)
        
        # 如果垂直距离很小，说明在同一直线上
        if perpendicular_distance < 0.3:
            # 障碍物质量轻，可以接受碰撞
            return self.obstacle_mass_factor < 0.5
        
        return False
    
    def _predict_robot_position(self, start_x, start_y, end_x, end_y, time_elapsed):
        """预测机器人在指定时间后的位置"""
        total_time = self.calculate_passage_time(start_x, start_y, end_x, end_y)
        if total_time < 0.01:
            return (end_x, end_y)
        
        progress = min(time_elapsed / total_time, 1.0)
        x = start_x + (end_x - start_x) * progress
        y = start_y + (end_y - start_y) * progress
        return (x, y)
    
    def _calculate_safety_level(self, entry_time, exit_time, obstacle, robot_x, robot_y, target_x, target_y):
        """计算安全等级"""
        min_safety_distance = float("inf")
        current_time = obstacle.last_update_time
        current_obstacle_speed = math.hypot(obstacle.vx, obstacle.vy)
        
        # 只检查几个关键时间点
        check_times = [entry_time, (entry_time + exit_time) / 2, exit_time]
        
        for check_time in check_times:
            obstacle_pos = obstacle.predict_position_at_time(check_time)
            robot_pos = self._predict_robot_position(robot_x, robot_y, target_x, target_y, check_time - current_time)
            
            distance = math.hypot(robot_pos[0] - obstacle_pos[0], robot_pos[1] - obstacle_pos[1])
            safe_distance = self.calculate_safe_distance(current_obstacle_speed, self.robot_speed)
            safety_margin = distance - safe_distance
            
            min_safety_distance = min(min_safety_distance, safety_margin)
        
        return max(min_safety_distance, 0.0)


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

        # 动态避障相关参数（适应物理模拟的障碍物）
        self.ROBOT_RADIUS = 0.22  # 机器人半径（米）
        self.OBSTACLE_RADIUS = 0.375  # 障碍物半径（0.75m的一半）
        self.ROBOT_SPEED = 0.5  # 机器人速度（m/s）
        self.MAX_WAIT_TIME = 12.0  # 最大等待时间（秒，减少等待时间）
        self.OBSTACLE_CHECK_INTERVAL = 0.15  # 检测间隔（150ms，平衡性能和实时性）
        self.COLLISION_ACCEPTANCE_THRESHOLD = 0.8  # 碰撞可接受阈值
        
        # 障碍物管理（使用基于物理的预测器）
        self.obstacles = {}
        self.path_intersection_logged = {}  # 日志控制标志
        self._initialize_obstacles()
        self.safe_calculator = AdaptiveSafePassageCalculator()
        
        # 避障状态
        self.emergency_stop = False
        self.navigation_paused = False
        self.waiting_for_safe_window = False
        self.wait_start_time = None
        self.wait_duration = 0.0
        self.current_nav_target = None
        self.obstacle_timer = None
        self.wait_timer = None
        
        # 导航状态跟踪
        self.expected_nav_target = None
        self.nav_in_progress = False
        self.nav_success_check_timer = None
        self.arrived_at_target = False
        
        # 小车停稳检测
        self.robot_velocity_history = []
        self.VELOCITY_HISTORY_SIZE = 5
        self.STOPPED_VELOCITY_THRESHOLD = 0.05  # 0.05 m/s以下认为停稳

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
        
        # 障碍物位置订阅器（注意：需要确保障碍物插件发布这些话题）
        # 如果没有发布，需要修改障碍物插件添加ROS发布器
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

        self.get_logger().info("主控节点（支持物理模拟障碍物避障）启动成功")
        self.get_logger().info("适应特性：速度波动、可撞开障碍物、动态安全距离")

    def _initialize_obstacles(self):
        """初始化基于物理的障碍物参数"""
        # obstacle2参数（从room.world获取）
        obstacle2 = PhysicsBasedObstaclePredictor("obstacle2")
        obstacle2.start_x = -9.0
        obstacle2.start_y = -4.2
        obstacle2.end_x = -5.0
        obstacle2.end_y = -3.3
        obstacle2.nominal_speed = 0.3
        obstacle2.speed_variation = 0.1
        
        # 重新计算轨迹向量
        dx = obstacle2.end_x - obstacle2.start_x
        dy = obstacle2.end_y - obstacle2.start_y
        distance = math.hypot(dx, dy)
        if distance > 0.001:
            obstacle2.direction_unit_vector = (dx / distance, dy / distance)
        obstacle2.trajectory_vector = (dx, dy)
        obstacle2.trajectory_length = distance
        
        # obstacle3参数（从room.world获取）
        obstacle3 = PhysicsBasedObstaclePredictor("obstacle3")
        obstacle3.start_x = -2.5
        obstacle3.start_y = 3.0
        obstacle3.end_x = -2.5
        obstacle3.end_y = -2.0
        obstacle3.nominal_speed = 0.3
        obstacle3.speed_variation = 0.1
        
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
        
        # 初始化日志标志
        for obs_name in self.obstacles.keys():
            self.path_intersection_logged[obs_name] = False

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
        """检查障碍物并执行避障逻辑（适应物理模拟）"""
        if self.current_step not in ["NAV_TO_BLOCK", "NAV_TO_AREA"]:
            # 重置轨迹相交日志标志
            for obs_name in self.obstacles.keys():
                self.path_intersection_logged[obs_name] = False
            return
            
        if not self.current_nav_target or self.emergency_stop:
            return
            
        robot_x, robot_y = self.current_robot_pose
        target_x, target_y = self.current_nav_target
        
        # 检查所有障碍物
        for obstacle_name, obstacle in self.obstacles.items():
            if not obstacle.active:
                continue
                
            # 获取当前障碍物状态
            current_obstacle_speed = math.hypot(obstacle.vx, obstacle.vy)
            distance_to_obstacle = math.hypot(robot_x - obstacle.x, robot_y - obstacle.y)
            
            # 紧急碰撞检查（距离过近）
            min_emergency_distance = (self.ROBOT_RADIUS + self.OBSTACLE_RADIUS) * self.COLLISION_ACCEPTANCE_THRESHOLD
            if distance_to_obstacle < min_emergency_distance:
                # 检查是否可以接受碰撞（障碍物可被撞开）
                if not self.safe_calculator.is_collision_acceptable(robot_x, robot_y, obstacle.x, obstacle.y):
                    self._emergency_stop(f"距离障碍物{obstacle_name}过近：{distance_to_obstacle:.2f}米")
                return
            
            # 检查路径是否与障碍物轨迹相交
            path_intersecting = self._is_path_intersecting_trajectory(robot_x, robot_y, target_x, target_y, obstacle)
            
            # 轨迹相交日志控制：只在状态变化时输出
            if path_intersecting and not self.path_intersection_logged[obstacle_name]:
                self.get_logger().info(f"检测到路径与障碍物{obstacle_name}轨迹相交（速度：{current_obstacle_speed:.2f}m/s）")
                self.path_intersection_logged[obstacle_name] = True
            elif not path_intersecting and self.path_intersection_logged[obstacle_name]:
                # 路径不再相交，重置标志
                self.path_intersection_logged[obstacle_name] = False
            
            if path_intersecting:
                # 如果正在等待安全窗口，跳过重新计算
                if self.waiting_for_safe_window:
                    return
                    
                # 计算安全通过窗口（使用自适应计算器）
                safe_result = self.safe_calculator.find_safe_passage_window(
                    robot_x, robot_y, target_x, target_y, obstacle, self.MAX_WAIT_TIME
                )
                
                self._handle_safe_result(safe_result, obstacle_name)
                return
        
        # 如果之前在等待，现在可以继续
        if self.navigation_paused and not self.waiting_for_safe_window:
            self._resume_navigation()

    def _handle_safe_result(self, safe_result, obstacle_name):
        """处理安全检查结果"""
        if safe_result['safe']:
            action = safe_result['action']
            
            if action == 'wait_and_pass' and safe_result['wait_time'] > 0.2:
                # 等待安全窗口
                confidence = safe_result.get('confidence', 0.5)
                safety_level = safe_result.get('safety_level', 0.0)
                
                # 根据置信度调整等待策略
                if confidence > 0.7 and safety_level > 0.2:
                    self._wait_for_safe_window(safe_result, obstacle_name)
                else:
                    # 置信度低或安全等级不高，考虑其他策略
                    self.get_logger().info(f"安全窗口置信度较低，尝试直接通过障碍物{obstacle_name}")
                    self._resume_navigation()
                    
            elif action == 'careful_pass':
                # 谨慎通过（障碍物可被撞开）
                self.get_logger().info(f"谨慎通过障碍物{obstacle_name}（{safe_result['reason']}）")
                self._resume_navigation()
                
            elif action == 'push_away':
                # 推走障碍物
                self.get_logger().info(f"直接推走障碍物{obstacle_name}（{safe_result['reason']}）")
                self._resume_navigation()
                
            elif action == 'direct_pass':
                # 直接通过
                self.get_logger().info(f"可以直接通过障碍物{obstacle_name}")
                self._resume_navigation()
                
            else:
                # 其他安全情况
                self._resume_navigation()
        else:
            # 不安全，尝试绕开
            if safe_result['action'] == 'need_detour':
                self.get_logger().info(f"无法找到安全窗口，尝试绕开障碍物{obstacle_name}")
                self._attempt_detour()
            else:
                # 其他不安全情况，等待
                self.get_logger().warn(f"无法安全通过障碍物{obstacle_name}，等待情况改善")
                self._wait_for_safe_window({'wait_time': 1.0}, obstacle_name, is_simple_wait=True)

    def _is_path_intersecting_trajectory(self, robot_x, robot_y, target_x, target_y, obstacle):
        """判断机器人路径是否与障碍物轨迹相交（适应物理模拟）"""
        # 简化的相交检测：检查路径是否靠近轨迹
        min_distance_to_trajectory = float("inf")
        
        # 采样检查路径上的多个点
        for t in [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]:
            x = robot_x + t * (target_x - robot_x)
            y = robot_y + t * (target_y - robot_y)
            
            # 检查点是否在轨迹附近
            if obstacle.is_position_on_trajectory(x, y, tolerance=0.3):
                # 计算到障碍物当前位置的距离
                distance_to_obstacle = math.hypot(x - obstacle.x, y - obstacle.y)
                current_obstacle_speed = math.hypot(obstacle.vx, obstacle.vy)
                
                # 计算安全距离
                safe_distance = self.safe_calculator.calculate_safe_distance(current_obstacle_speed, self.ROBOT_SPEED)
                
                if distance_to_obstacle < safe_distance * 1.5:
                    return True
        
        return False

    def _emergency_stop(self, reason):
        """紧急停止"""
        if not self.emergency_stop:
            self.get_logger().warn(f"紧急停止！{reason}")
            self.emergency_stop = True
            self.navigation_paused = True
            self.emergency_stop_pub.publish(Bool(data=True))
            # 发布空目标以停止导航
            self.nav_target_pub.publish(String(data=json.dumps({"type": "pause"})))
            
            # 取消所有定时器
            self._clean_all_timers()

    def _wait_for_safe_window(self, safe_result, obstacle_name, is_simple_wait=False):
        """等待安全通过窗口"""
        if self.waiting_for_safe_window:
            return
            
        wait_time = safe_result['wait_time']
        if is_simple_wait:
            self.get_logger().info(f"等待{wait_time:.1f}秒后重试障碍物{obstacle_name}")
        else:
            confidence = safe_result.get('confidence', 0.5)
            safety_level = safe_result.get('safety_level', 0.0)
            self.get_logger().info(f"等待安全窗口通过障碍物{obstacle_name}，预计等待{wait_time:.1f}秒（置信度：{confidence:.2f}，安全等级：{safety_level:.2f}）")
        
        self.waiting_for_safe_window = True
        self.wait_start_time = datetime.now()
        self.wait_duration = wait_time
        self.navigation_paused = True
        
        # 暂停导航
        self.nav_target_pub.publish(String(data=json.dumps({"type": "pause"})))
        
        # 创建等待定时器
        if self.wait_timer:
            self.wait_timer.cancel()
            self.wait_timer.destroy()
        
        self.wait_timer = self.create_timer(
            0.1,  # 每0.1秒检查一次
            lambda: self._check_wait_completion(obstacle_name)
        )
        
        # 取消导航成功检查
        self._cancel_nav_success_check()

    def _check_wait_completion(self, obstacle_name):
        """检查等待是否完成"""
        if not self.waiting_for_safe_window:
            return
            
        elapsed_time = (datetime.now() - self.wait_start_time).total_seconds()
        
        # 检查是否到达等待时间
        if elapsed_time >= self.wait_duration - 0.1:  # 提前0.1秒恢复
            self.get_logger().info(f"等待完成，恢复导航通过障碍物{obstacle_name}")
            self._resume_navigation()
            return
        
        # 检查是否超时（防止无限等待）
        if elapsed_time > self.wait_duration + 3.0:  # 超时3秒
            self.get_logger().warn(f"等待安全窗口超时，强制恢复导航")
            self._resume_navigation()

    def _attempt_detour(self):
        """尝试绕开障碍物（简化版，因为障碍物可被撞开）"""
        # 对于可被撞开的障碍物，简化绕开逻辑
        robot_x, robot_y = self.current_robot_pose
        target_x, target_y = self.current_nav_target
        
        # 计算一个简单的偏移点
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.hypot(dx, dy)
        
        if distance < 0.1:
            return
            
        # 垂直偏移0.5米
        detour_x = robot_x + (dy / distance) * 0.5
        detour_y = robot_y - (dx / distance) * 0.5
        
        self.get_logger().info(f"使用简单绕开点：({detour_x:.2f}, {detour_y:.2f})")
        
        # 导航到绕开点
        self.current_nav_target = (detour_x, detour_y)
        nav_msg = String()
        nav_msg.data = json.dumps({"type": "custom", "x": detour_x, "y": detour_y, "yaw": 0.0})
        self.nav_target_pub.publish(nav_msg)
        
        # 记录原始目标，以便绕开后继续
        self.original_nav_target = (target_x, target_y)
        self.navigation_paused = False
        self.waiting_for_safe_window = False
        
        # 设置导航成功检查
        self._setup_nav_success_check((detour_x, detour_y))

    def _resume_navigation(self):
        """恢复导航"""
        self.emergency_stop = False
        self.navigation_paused = False
        self.waiting_for_safe_window = False
        self.wait_start_time = None
        
        # 清理等待定时器
        if self.wait_timer:
            self.wait_timer.cancel()
            self.wait_timer.destroy()
            self.wait_timer = None
        
        self.emergency_stop_pub.publish(Bool(data=False))
        
        if self.current_nav_target:
            nav_msg = String()
            nav_msg.data = json.dumps({"type": "custom", "x": self.current_nav_target[0], "y": self.current_nav_target[1], "yaw": 0.0})
            self.nav_target_pub.publish(nav_msg)
            
            # 设置导航成功检查
            self._setup_nav_success_check(self.current_nav_target)

    def _setup_nav_success_check(self, target_pos):
        """设置导航成功检查定时器"""
        # 取消之前的定时器
        self._cancel_nav_success_check()
        
        # 计算预计到达时间
        distance = self.calculate_distance(self.current_robot_pose, target_pos)
        estimated_time = distance / self.ROBOT_SPEED + 2.0  # 减少缓冲时间
        
        self.expected_nav_target = target_pos
        self.nav_in_progress = True
        self.arrived_at_target = False
        
        # 创建检查定时器
        self.nav_success_check_timer = self.create_timer(
            0.5,  # 每0.5秒检查一次
            lambda: self._check_nav_success()
        )
        
        self.get_logger().info(f"设置导航成功检查，目标：({target_pos[0]:.2f}, {target_pos[1]:.2f})，预计时间：{estimated_time:.1f}秒")

    def _cancel_nav_success_check(self):
        """取消导航成功检查定时器"""
        if self.nav_success_check_timer:
            self.nav_success_check_timer.cancel()
            self.nav_success_check_timer.destroy()
            self.nav_success_check_timer = None
        
        self.nav_in_progress = False
        self.arrived_at_target = False
        self.expected_nav_target = None

    def _check_nav_success(self):
        """检查导航是否成功到达目标"""
        if not self.nav_in_progress or not self.expected_nav_target or self.arrived_at_target:
            return
            
        robot_x, robot_y = self.current_robot_pose
        target_x, target_y = self.expected_nav_target
        
        # 计算距离
        distance = self.calculate_distance((robot_x, robot_y), (target_x, target_y))
        
        # 如果距离小于阈值，认为导航成功
        if distance < 0.25:  # 25厘米阈值
            # 检查小车是否停稳
            if self._is_robot_stopped():
                self.get_logger().info(f"导航成功检查：到达目标位置，距离：{distance:.2f}米，小车已停稳")
                
                # 检查是否是绕开点
                if hasattr(self, 'original_nav_target') and self.original_nav_target:
                    self.get_logger().info("到达绕开点，继续导航到原始目标")
                    self.current_nav_target = self.original_nav_target
                    self.original_nav_target = None
                    nav_msg = String()
                    nav_msg.data = json.dumps({"type": "custom", "x": self.current_nav_target[0], "y": self.current_nav_target[1], "yaw": 0.0})
                    self.nav_target_pub.publish(nav_msg)
                    self._setup_nav_success_check(self.current_nav_target)
                    return
                
                # 手动触发导航成功处理
                if self.current_step == "NAV_TO_BLOCK":
                    self.get_logger().info("到达物块位置，准备抓取")
                    self.current_step = "GRASP"
                    self.trigger_grasp()
                elif self.current_step == "NAV_TO_AREA":
                    self.get_logger().info("到达目标区域，准备放置")
                    self.current_step = "PLACE"
                    self.trigger_place()
                
                # 清理导航状态
                self._cancel_nav_success_check()

    def _is_robot_stopped(self):
        """检查小车是否停稳"""
        if len(self.robot_velocity_history) < self.VELOCITY_HISTORY_SIZE:
            return False
            
        # 计算平均速度
        avg_velocity = sum(self.robot_velocity_history) / len(self.robot_velocity_history)
        return avg_velocity < self.STOPPED_VELOCITY_THRESHOLD

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

    # 为单个任务分配最佳物块
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
            distance = self.calculate_distance(self.current_robot_pose, grasp_pos)
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
            total_cost += self.calculate_distance(current_pos, assignment["grasp_pos"])
            total_cost += self.calculate_distance(assignment["grasp_pos"], assignment["area_pos"])
            current_pos = assignment["area_pos"]
        
        return total_cost, task_assignments

    # 优化多任务执行顺序
    def optimize_task_order(self, tasks):
        if len(tasks) <= 1:
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
            for i, assignment in enumerate(best_assignments, 1):
                task = assignment["task"]
                block_pos = assignment["block_pos"]
                self.get_logger().info(f"{i}. {task['color']}物块到{task['to']}区 (位置: {block_pos[0]:.2f},{block_pos[1]:.2f})")
        
        return best_assignments

    # 处理下一个优化后的任务
    def _process_next_optimized_task(self):
        if not self.optimized_tasks:
            self.get_logger().info("所有优化任务执行完成，等待新任务...")
            self.current_task = None
            self.current_step = "WAIT_TASK"
            self._update_foxglove()
            
            # 重置所有状态
            self._clean_all_timers()
            self.nav_in_progress = False
            self.expected_nav_target = None
            self.arrived_at_target = False
            self.robot_velocity_history.clear()
            
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
        self._clean_all_timers()
        self.navigation_paused = False
        self.emergency_stop = False
        self.waiting_for_safe_window = False
        self.wait_start_time = None
        self.current_nav_target = None
        self.original_nav_target = None
        self.nav_in_progress = False
        self.expected_nav_target = None
        self.arrived_at_target = False
        self.robot_velocity_history.clear()
        
        # 重置轨迹相交日志标志
        for obs_name in self.obstacles.keys():
            self.path_intersection_logged[obs_name] = False

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

    # 导航到物块
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
        
        # 设置导航成功检查
        self._setup_nav_success_check(grasp_pos)

    # 导航到目标区域
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
        
        # 设置导航成功检查
        self._setup_nav_success_check(area_pos)

    # AMCL定位回调
    @staticmethod
    def _amcl_callback_impl(self_ref, msg):
        self = self_ref()
        if not self:
            return
        new_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
        # 计算速度（用于停稳检测）
        if self.current_robot_pose != (0.0, 0.0):
            distance = math.hypot(new_pose[0] - self.current_robot_pose[0], 
                                 new_pose[1] - self.current_robot_pose[1])
            # 假设AMCL回调频率约为10Hz
            velocity = distance * 10.0  # 估算速度
            self.robot_velocity_history.append(velocity)
            if len(self.robot_velocity_history) > self.VELOCITY_HISTORY_SIZE:
                self.robot_velocity_history.pop(0)
        
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
            for i in range(num):
                expanded_tasks.append({
                    "color": task["color"],
                    "num": 1,  # 单个物块
                    "to": task["to"],
                    "original_num": num,  # 记录原始数量
                    "current_index": i + 1  # 当前索引
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
        
        # 如果导航被暂停或已经到达目标，忽略导航状态更新
        if self.navigation_paused or self.emergency_stop or self.arrived_at_target:
            return
            
        # 过滤掉状态更新消息
        if status.startswith("state:"):
            return
            
        self.get_logger().info(f"导航状态回调：{status}")
            
        if status == "succeeded":
            # 检查是否是绕开点
            if hasattr(self, 'original_nav_target') and self.original_nav_target:
                self.get_logger().info("到达绕开点，继续导航到原始目标")
                self.current_nav_target = self.original_nav_target
                self.original_nav_target = None
                nav_msg = String()
                nav_msg.data = json.dumps({"type": "custom", "x": self.current_nav_target[0], "y": self.current_nav_target[1], "yaw": 0.0})
                self.nav_target_pub.publish(nav_msg)
                
                # 设置新的导航成功检查
                self._setup_nav_success_check(self.current_nav_target)
                return
            
            if self.current_step == "NAV_TO_BLOCK":
                self.get_logger().info("到达物块位置，准备抓取")
                self.current_step = "GRASP"
                self.trigger_grasp()
            elif self.current_step == "NAV_TO_AREA":
                self.get_logger().info("到达目标区域，准备放置")
                self.current_step = "PLACE"
                self.trigger_place()
                
            # 清理导航成功检查
            self._cancel_nav_success_check()
                
        elif status == "failed":
            self.get_logger().warn("导航失败，重试当前步骤")
            
            # 重试导航
            if self.current_step == "NAV_TO_BLOCK" and self.current_nav_target:
                self.navigate_to_block()
            elif self.current_step == "NAV_TO_AREA" and self.current_nav_target:
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
        self.get_logger().info(f"机械臂状态：{status}")
        
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

    # 清理所有定时器
    def _clean_all_timers(self):
        self._clean_timers()
        
        # 清理其他定时器
        self._cancel_nav_success_check()
            
        if self.wait_timer:
            self.wait_timer.cancel()
            self.wait_timer.destroy()
            self.wait_timer = None
            
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
        self._clean_all_timers()
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