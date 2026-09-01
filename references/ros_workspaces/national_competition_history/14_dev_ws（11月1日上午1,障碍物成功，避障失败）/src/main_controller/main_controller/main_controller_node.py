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
from datetime import datetime

class DynamicObstacle:
    """动态障碍物管理类"""
    def __init__(self, obstacle_name):
        self.name = obstacle_name
        self.x = 0.0
        self.y = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.last_update_time = 0.0
        self.active = False
        self.direction_changed = False
        
    def update_position(self, x, y, current_time):
        """更新障碍物位置并计算速度"""
        if self.active:
            time_diff = current_time - self.last_update_time
            if time_diff > 0.01:  # 避免除以太小的数
                self.speed_x = (x - self.last_x) / time_diff
                self.speed_y = (y - self.last_y) / time_diff
                
                # 检测运动方向变化
                old_direction = math.atan2(self.speed_y, self.speed_x) if (self.speed_x != 0 or self.speed_y != 0) else 0
                new_direction = math.atan2(y - self.last_y, x - self.last_x) if (x != self.last_x or y != self.last_y) else 0
                direction_diff = abs(old_direction - new_direction)
                if direction_diff > math.pi/4:  # 45度以上的方向变化
                    self.direction_changed = True
                else:
                    self.direction_changed = False
        
        self.last_x = self.x
        self.last_y = self.y
        self.x = x
        self.y = y
        self.last_update_time = current_time
        self.active = True
        
    def get_position(self):
        """获取当前位置"""
        return (self.x, self.y)
    
    def get_speed(self):
        """获取速度"""
        return (self.speed_x, self.speed_y)
    
    def is_stationary(self):
        """判断是否静止"""
        speed_magnitude = math.hypot(self.speed_x, self.speed_y)
        return speed_magnitude < 0.01
    
    def predict_position(self, time_ahead):
        """预测未来位置"""
        return (self.x + self.speed_x * time_ahead, self.y + self.speed_y * time_ahead)
    
    def is_obsolete(self, current_time, timeout=1.0):
        """判断位置信息是否过时"""
        return current_time - self.last_update_time > timeout


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
        self.SAFE_DISTANCE = 1.0  # 安全距离（米）
        self.EMERGENCY_DISTANCE = 0.6  # 紧急停止距离（米）
        self.PREDICTION_TIME = 2.0  # 预测时间（秒）
        self.OBSTACLE_CHECK_INTERVAL = 0.1  # 障碍物检查间隔（秒）
        self.MAX_WAIT_TIME = 30.0  # 最大等待时间（秒）
        
        # 障碍物管理
        self.obstacles = {
            "obstacle2": DynamicObstacle("obstacle2"),
            "obstacle3": DynamicObstacle("obstacle3")
        }
        self.obstacle_timer = None
        self.emergency_stop = False
        self.obstacle_wait_start_time = None
        self.navigation_paused = False

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

        self.get_logger().info("主控节点（支持智能动态避障）启动成功")

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
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.obstacles[obstacle_name].update_position(
            msg.position.x, msg.position.y, current_time
        )
        
        # 重置等待时间如果障碍物移动
        if self.obstacle_wait_start_time and self.obstacles[obstacle_name].direction_changed:
            self.obstacle_wait_start_time = datetime.now()

    def _check_obstacles(self):
        """检查障碍物并执行避障逻辑"""
        if self.current_step not in ["NAV_TO_BLOCK", "NAV_TO_AREA"]:
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        robot_x, robot_y = self.current_robot_pose
        min_distance = float("inf")
        closest_obstacle = None
        obstacle_in_path = False

        # 检查所有障碍物
        for obstacle_name, obstacle in self.obstacles.items():
            if not obstacle.active or obstacle.is_obsolete(current_time):
                continue
                
            obstacle_x, obstacle_y = obstacle.get_position()
            
            # 计算机器人与障碍物的距离（考虑半径）
            distance = math.hypot(robot_x - obstacle_x, robot_y - obstacle_y)
            actual_distance = distance - self.ROBOT_RADIUS - self.OBSTACLE_RADIUS
            
            if actual_distance < min_distance:
                min_distance = actual_distance
                closest_obstacle = obstacle_name

            # 预测障碍物未来位置
            predicted_x, predicted_y = obstacle.predict_position(self.PREDICTION_TIME)
            predicted_distance = math.hypot(robot_x - predicted_x, robot_y - predicted_y)
            actual_predicted_distance = predicted_distance - self.ROBOT_RADIUS - self.OBSTACLE_RADIUS

            # 检查是否在路径上
            if self.current_assignment:
                target_x, target_y = self._get_current_target_position()
                if self._is_obstacle_in_path(obstacle_x, obstacle_y, target_x, target_y, robot_x, robot_y):
                    obstacle_in_path = True

        # 紧急停止检查
        if min_distance < self.EMERGENCY_DISTANCE:
            if not self.emergency_stop:
                self.get_logger().warn(f"紧急停止！距离障碍物{closest_obstacle}过近：{min_distance:.2f}米")
                self.emergency_stop = True
                self.emergency_stop_pub.publish(Bool(data=True))
                self.navigation_paused = True
                self.obstacle_wait_start_time = datetime.now()
        elif min_distance < self.SAFE_DISTANCE or obstacle_in_path:
            if not self.navigation_paused:
                self.get_logger().info(f"检测到障碍物{closest_obstacle}在路径上，距离：{min_distance:.2f}米，暂停导航")
                self.navigation_paused = True
                self.obstacle_wait_start_time = datetime.now()
                # 发布空目标以暂停导航
                self.nav_target_pub.publish(String(data=json.dumps({"type": "pause"})))
        else:
            if self.navigation_paused:
                self.get_logger().info(f"障碍物{closest_obstacle}已远离，恢复导航")
                self.navigation_paused = False
                self.emergency_stop = False
                self.emergency_stop_pub.publish(Bool(data=False))
                self.obstacle_wait_start_time = None
                
                # 恢复导航到原目标
                if self.current_assignment:
                    target_x, target_y = self._get_current_target_position()
                    nav_msg = String()
                    nav_msg.data = json.dumps({"type": "custom", "x": target_x, "y": target_y, "yaw": 0.0})
                    self.nav_target_pub.publish(nav_msg)

        # 检查等待超时
        if self.obstacle_wait_start_time:
            wait_duration = (datetime.now() - self.obstacle_wait_start_time).total_seconds()
            if wait_duration > self.MAX_WAIT_TIME:
                self.get_logger().warn(f"等待障碍物超时（{wait_duration:.1f}秒），尝试重新规划路径")
                self.navigation_paused = False
                self.emergency_stop = False
                self.emergency_stop_pub.publish(Bool(data=False))
                self.obstacle_wait_start_time = None
                
                # 重新规划路径
                if self.current_assignment:
                    self._replan_path()

    def _get_current_target_position(self):
        """获取当前导航目标位置"""
        if self.current_step == "NAV_TO_BLOCK":
            return self.current_assignment["grasp_pos"]
        elif self.current_step == "NAV_TO_AREA":
            return self.current_assignment["area_pos"]
        return (0.0, 0.0)

    def _is_obstacle_in_path(self, obstacle_x, obstacle_y, target_x, target_y, robot_x, robot_y):
        """判断障碍物是否在机器人到目标的路径上"""
        # 计算机器人到目标的向量
        dx_target = target_x - robot_x
        dy_target = target_y - robot_y
        target_distance = math.hypot(dx_target, dy_target)
        
        if target_distance < 0.1:  # 接近目标，不考虑路径
            return False
            
        # 计算障碍物到机器人的向量
        dx_obstacle = obstacle_x - robot_x
        dy_obstacle = obstacle_y - robot_y
        obstacle_distance = math.hypot(dx_obstacle, dy_obstacle)
        
        # 计算点积（判断障碍物是否在机器人前方）
        dot_product = dx_target * dx_obstacle + dy_target * dy_obstacle
        if dot_product < 0:  # 障碍物在机器人后方
            return False
            
        # 计算障碍物到路径的垂直距离
        cross_product = abs(dx_target * dy_obstacle - dy_target * dx_obstacle)
        perpendicular_distance = cross_product / target_distance
        
        # 计算障碍物在路径上的投影点
        projection_ratio = dot_product / (target_distance ** 2)
        projection_ratio = max(0.0, min(1.0, projection_ratio))  # 限制在0-1之间
        
        # 判断障碍物是否在路径附近且在机器人前方
        path_margin = self.SAFE_DISTANCE + self.ROBOT_RADIUS + self.OBSTACLE_RADIUS
        return (perpendicular_distance < path_margin and projection_ratio > 0.0 and projection_ratio < 1.0)

    def _replan_path(self):
        """重新规划路径"""
        self.get_logger().info("重新规划路径以避开障碍物")
        
        if self.current_assignment:
            target_x, target_y = self._get_current_target_position()
            robot_x, robot_y = self.current_robot_pose
            
            # 尝试在目标位置周围寻找安全点
            safe_offset = self.SAFE_DISTANCE + 0.2
            offset_angles = [math.pi/2, 3*math.pi/2, math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4]
            
            for angle in offset_angles:
                offset_x = safe_offset * math.cos(angle)
                offset_y = safe_offset * math.sin(angle)
                candidate_x = target_x + offset_x
                candidate_y = target_y + offset_y
                
                # 检查候选点是否安全
                safe = True
                current_time = self.get_clock().now().nanoseconds / 1e9
                
                for obstacle in self.obstacles.values():
                    if not obstacle.active or obstacle.is_obsolete(current_time):
                        continue
                        
                    distance = math.hypot(candidate_x - obstacle.x, candidate_y - obstacle.y)
                    actual_distance = distance - self.ROBOT_RADIUS - self.OBSTACLE_RADIUS
                    
                    if actual_distance < self.SAFE_DISTANCE:
                        safe = False
                        break
                
                if safe:
                    self.get_logger().info(f"找到安全绕行点：({candidate_x:.2f}, {candidate_y:.2f})")
                    nav_msg = String()
                    nav_msg.data = json.dumps({"type": "custom", "x": candidate_x, "y": candidate_y, "yaw": 0.0})
                    self.nav_target_pub.publish(nav_msg)
                    
                    # 记录绕行点，之后需要继续到原目标
                    self.detour_target = (target_x, target_y)
                    return
            
            self.get_logger().warn("无法找到安全绕行点，使用原路径")
            nav_msg = String()
            nav_msg.data = json.dumps({"type": "custom", "x": target_x, "y": target_y, "yaw": 0.0})
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

    # 为同一任务的多个物块生成优化顺序
    def optimize_multi_block_order(self, task, remaining_count):
        """为同一任务中的多个物块生成优化的抓取顺序"""
        if remaining_count <= 0:
            return []
            
        self.get_logger().info(f"正在为任务优化{remaining_count}个物块的抓取顺序...")
        
        # 获取所有可用物块
        color = task["color"]
        available_blocks = self.get_available_blocks(color)
        
        if len(available_blocks) < remaining_count:
            self.get_logger().warn(f"可用物块数量不足，需要{remaining_count}个，实际只有{len(available_blocks)}个")
            remaining_count = len(available_blocks)
        
        # 生成所有可能的物块组合和顺序
        min_total_cost = float("inf")
        best_sequence = None
        
        # 从可用物块中选择remaining_count个
        from itertools import combinations
        
        # 为了避免计算量过大，限制最大组合数
        max_combinations = 1000
        combo_count = 0
        
        for block_combination in combinations(available_blocks, remaining_count):
            combo_count += 1
            if combo_count > max_combinations:
                self.get_logger().warn("组合数量过多，使用贪心算法替代")
                # 使用贪心算法
                best_sequence = self._greedy_multi_block_selection(task, remaining_count)
                break
                
            # 尝试所有排列顺序
            for block_order in permutations(block_combination):
                # 计算这个顺序的总成本
                total_cost = 0.0
                current_pos = self.current_robot_pose
                valid = True
                
                for block_pos in block_order:
                    block_x, block_y, block_idx = block_pos
                    grasp_pos = self.get_grasp_position((block_x, block_y))
                    area_pos = self.AREA_COORDS[task["to"]]
                    
                    # 计算成本
                    cost = self.calculate_distance(current_pos, grasp_pos) + \
                           self.calculate_distance(grasp_pos, area_pos)
                    
                    if cost == float("inf"):
                        valid = False
                        break
                        
                    total_cost += cost
                    current_pos = area_pos  # 下一个任务从目标区域开始
                
                if valid and total_cost < min_total_cost:
                    min_total_cost = total_cost
                    best_sequence = block_order
        
        if best_sequence is None:
            # 如果没有找到最佳序列，使用贪心算法
            best_sequence = self._greedy_multi_block_selection(task, remaining_count)
        
        # 转换为任务分配格式
        assignments = []
        for block_pos in best_sequence:
            block_x, block_y, block_idx = block_pos
            area_pos = self.AREA_COORDS[task["to"]]
            grasp_pos = self.get_grasp_position((block_x, block_y))
            
            assignments.append({
                "task": task,
                "block_pos": (block_x, block_y),
                "block_idx": block_idx,
                "grasp_pos": grasp_pos,
                "area_pos": area_pos,
                "multi_block": True  # 标记为多物块任务的一部分
            })
        
        self.get_logger().info(f"多物块优化完成，最佳路径总距离：{min_total_cost:.2f}米")
        return assignments

    # 贪心算法选择多物块顺序
    def _greedy_multi_block_selection(self, task, remaining_count):
        """贪心算法：每次选择当前最优的物块"""
        selected_blocks = []
        used_blocks = set()
        current_pos = self.current_robot_pose
        
        for _ in range(remaining_count):
            best_block = None
            min_cost = float("inf")
            
            # 获取所有可用物块
            color = task["color"]
            available_blocks = self.get_available_blocks(color)
            
            for block_pos in available_blocks:
                block_x, block_y, block_idx = block_pos
                if block_idx in used_blocks:
                    continue
                    
                # 计算抓取位置
                grasp_pos = self.get_grasp_position((block_x, block_y))
                area_pos = self.AREA_COORDS[task["to"]]
                
                # 计算成本
                cost = self.calculate_distance(current_pos, grasp_pos) + \
                       self.calculate_distance(grasp_pos, area_pos)
                
                if cost < min_cost:
                    min_cost = cost
                    best_block = block_pos
            
            if best_block:
                selected_blocks.append(best_block)
                used_blocks.add(best_block[2])
                current_pos = self.AREA_COORDS[task["to"]]  # 下一个从目标区域开始
            else:
                break
        
        return selected_blocks

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
        self.obstacle_wait_start_time = None

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

        # 发布导航目标
        nav_msg = String()
        nav_msg.data = json.dumps({"type": "custom", "x": grasp_pos[0], "y": grasp_pos[1], "yaw": 0.0})
        self.nav_target_pub.publish(nav_msg)
        self.target_cube_pub.publish(String(data=cube_name))
        self.current_step = "NAV_TO_BLOCK"
        self._update_foxglove()
        self.get_logger().info(f"导航到物块：{cube_name}（抓取位置：{grasp_pos[0]:.2f}, {grasp_pos[1]:.2f}）")

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

    # 导航到目标区域（使用优化路径）
    def navigate_to_area(self):
        if not self.current_assignment:
            return

        area_pos = self.current_assignment["area_pos"]
        area = self.current_task["to"]
        
        nav_msg = String()
        nav_msg.data = json.dumps({"type": "custom", "x": area_pos[0], "y": area_pos[1], "yaw": 0.0})
        self.nav_target_pub.publish(nav_msg)
        self.current_step = "NAV_TO_AREA"
        self._update_foxglove()
        self.get_logger().info(f"导航到目标区域 {area}：({area_pos[0]:.2f}, {area_pos[1]:.2f})")

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