import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from rclpy.timer import Timer
import json
import math
import gc
from itertools import permutations

class DynamicObstacle:
    """动态障碍物类，管理单个障碍物的状态和预测"""
    def __init__(self, obstacle_name):
        self.name = obstacle_name
        self.current_pos = (0.0, 0.0)
        self.last_pos = (0.0, 0.0)
        self.velocity = (0.0, 0.0)
        self.acceleration = (0.0, 0.0)
        self.last_update_time = 0.0
        self.size = 0.75  # 障碍物尺寸
        self.speed = 0.05  # 已知速度
        self.is_moving = True
        
        # 运动状态
        self.direction = 1  # 1: 正向, -1: 反向
        self.start_pos = None
        self.end_pos = None
        
    def update_position(self, x, y, current_time):
        """更新障碍物位置并计算速度"""
        self.last_pos = self.current_pos
        self.current_pos = (x, y)
        
        if self.last_update_time > 0 and current_time > self.last_update_time:
            time_diff = current_time - self.last_update_time
            if time_diff > 0.001:  # 避免除以零
                # 计算速度
                vx = (x - self.last_pos[0]) / time_diff
                vy = (y - self.last_pos[1]) / time_diff
                
                # 平滑速度计算（低通滤波）
                alpha = 0.8
                self.velocity = (
                    alpha * self.velocity[0] + (1 - alpha) * vx,
                    alpha * self.velocity[1] + (1 - alpha) * vy
                )
                
                # 检查是否静止
                speed_magnitude = math.hypot(self.velocity[0], self.velocity[1])
                self.is_moving = speed_magnitude > 0.01
        
        self.last_update_time = current_time
    
    def predict_position(self, time_offset):
        """预测指定时间后的位置"""
        if not self.is_moving:
            return self.current_pos
            
        # 使用当前速度预测
        predicted_x = self.current_pos[0] + self.velocity[0] * time_offset
        predicted_y = self.current_pos[1] + self.velocity[1] * time_offset
        
        return (predicted_x, predicted_y)
    
    def get_speed(self):
        """获取当前速度大小"""
        return math.hypot(self.velocity[0], self.velocity[1])
    
    def get_direction(self):
        """获取运动方向（弧度）"""
        if self.get_speed() < 0.01:
            return 0.0
        return math.atan2(self.velocity[1], self.velocity[0])

class MainController(Node):
    def __init__(self):
        super().__init__("main_controller")
        # QoS配置
        self.qos_best_effort = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        self.qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
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

        # 动态障碍物管理
        self.obstacles = {
            "obstacle2": DynamicObstacle("obstacle2"),
            "obstacle3": DynamicObstacle("obstacle3")
        }
        
        # 初始化障碍物的起始和结束位置（根据用户最新修改）
        self.obstacles["obstacle2"].start_pos = (-9.0, -4.2)
        self.obstacles["obstacle2"].end_pos = (-5.0, -3.3)
        self.obstacles["obstacle3"].start_pos = (-2.5, 3.0)
        self.obstacles["obstacle3"].end_pos = (-2.5, -2.0)

        # 核心变量
        self.current_robot_pose = (0.0, 0.0)
        self.robot_velocity = (0.0, 0.0)
        self.robot_radius = 0.22  # 机器人半径
        self.target_blocks = []
        self.GRASP_OFFSET = 0.4
        self.OFFSET_AXIS = "x"
        self.retry_timer: Timer = None
        self.grasp_timer: Timer = None
        self.place_timer: Timer = None
        self.obstacle_timer: Timer = None
        self.collision_check_timer: Timer = None
        self.current_task = None  # 当前执行任务
        self.task_queue = []      # 原始任务队列
        self.optimized_tasks = [] # 优化后的任务执行顺序
        self.completed_num = 0    # 当前任务已完成数量
        self.current_step = "WAIT_TASK"  # 状态：WAIT_TASK/NAV_TO_BLOCK/GRASP/NAV_TO_AREA/PLACE/WAIT_OBSTACLE/EMERGENCY_STOP
        self.selected_block = None
        self.grasp_confirmed = False
        self.place_confirmed = False
        self.max_grasp_retry = 3
        self.current_grasp_retry = 0
        self.closest_cache = None
        self.cache_expire = 2.0
        self.last_cache_time = 0.0
        
        # 动态避障相关变量
        self.navigating = False
        self.current_nav_goal = None
        self.nav_path = []
        self.obstacle_waiting = False
        self.wait_start_time = 0.0
        self.MAX_WAIT_TIME = 30.0  # 最大等待时间
        self.EMERGENCY_DISTANCE = 0.5  # 紧急避障距离
        self.SAFETY_DISTANCE = 0.8  # 安全距离
        self.PREDICTION_TIME = 2.0  # 预测时间（秒）
        
        # 超时设置
        self.GRASP_TIMEOUT = 8.0
        self.PLACE_TIMEOUT = 8.0

        # 订阅器
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_callback, self.qos_best_effort
        )
        self.chat_sub = self.create_subscription(
            String, "/chat", self._chat_callback, self.qos_best_effort
        )
        self.nav_status_sub = self.create_subscription(
            String, "/nav_status", self._nav_status_callback, self.qos_reliable
        )
        self.arm_status_sub = self.create_subscription(
            String, "/arm_status", self._arm_status_callback, self.qos_reliable
        )
        
        # 订阅障碍物实时位置
        self.obstacle2_pose_sub = self.create_subscription(
            Pose, "/obstacle2/current_pose", lambda msg: self._obstacle_pose_callback("obstacle2", msg), self.qos_reliable
        )
        self.obstacle3_pose_sub = self.create_subscription(
            Pose, "/obstacle3/current_pose", lambda msg: self._obstacle_pose_callback("obstacle3", msg), self.qos_reliable
        )

        # 发布器
        self.target_cube_pub = self.create_publisher(String, "/current_target_cube", self.qos_reliable)
        self.nav_target_pub = self.create_publisher(String, "/manual_nav_target", self.qos_reliable)
        self.arm_cargo_pub = self.create_publisher(String, "/nav_done_cargo", self.qos_reliable)
        self.arm_area_pub = self.create_publisher(String, "/nav_done_area", self.qos_reliable)
        self.emergency_stop_pub = self.create_publisher(String, "/emergency_stop", self.qos_reliable)
        self.foxglove_pubs = {
            "color": self.create_publisher(Int32, "/color", self.qos_reliable),
            "ask": self.create_publisher(Int32, "/ask", self.qos_reliable),
            "pick": self.create_publisher(Int32, "/pick", self.qos_reliable),
            "cur": self.create_publisher(Int32, "/cur", self.qos_reliable)
        }

        # 创建定时器
        self.obstacle_timer = self.create_timer(0.05, self._update_obstacle_predictions)  # 50ms更新一次
        self.collision_check_timer = self.create_timer(0.1, self._continuous_collision_check)  # 100ms检查一次碰撞

        self.get_logger().info("=" * 80)
        self.get_logger().info("智能动态避障主控节点启动成功")
        self.get_logger().info("=" * 80)
        self.get_logger().info("避障参数配置：")
        self.get_logger().info(f"  机器人半径：{self.robot_radius:.2f}m")
        self.get_logger().info(f"  紧急避障距离：{self.EMERGENCY_DISTANCE:.2f}m")
        self.get_logger().info(f"  安全距离：{self.SAFETY_DISTANCE:.2f}m")
        self.get_logger().info(f"  预测时间：{self.PREDICTION_TIME:.1f}s")
        self.get_logger().info(f"  最大等待时间：{self.MAX_WAIT_TIME:.1f}s")
        self.get_logger().info("=" * 80)
        self.get_logger().info("障碍物配置：")
        for name, obstacle in self.obstacles.items():
            if obstacle.start_pos and obstacle.end_pos:
                self.get_logger().info(f"  {name}:")
                self.get_logger().info(f"    起始位置：{obstacle.start_pos}")
                self.get_logger().info(f"    结束位置：{obstacle.end_pos}")
                self.get_logger().info(f"    尺寸：{obstacle.size}m × {obstacle.size}m")
        self.get_logger().info("=" * 80)

    # 计算两点之间的距离
    def calculate_distance(self, point1, point2):
        return math.hypot(point1[0] - point2[0], point1[1] - point2[1])

    # 计算点到线段的最短距离
    def distance_to_line_segment(self, point, line_start, line_end):
        """计算点到线段的最短距离"""
        px, py = point
        x1, y1 = line_start
        x2, y2 = line_end
        
        # 向量
        line_vec = (x2 - x1, y2 - y1)
        point_vec = (px - x1, py - y1)
        line_len_sq = line_vec[0]**2 + line_vec[1]**2
        
        if line_len_sq < 0.0001:  # 线段长度为0
            return self.calculate_distance(point, line_start)
        
        # 计算投影参数
        t = (point_vec[0] * line_vec[0] + point_vec[1] * line_vec[1]) / line_len_sq
        t = max(0.0, min(1.0, t))  # 限制在0-1之间
        
        # 投影点
        projection_x = x1 + t * line_vec[0]
        projection_y = y1 + t * line_vec[1]
        
        return self.calculate_distance(point, (projection_x, projection_y))

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
        
        task_assignments = []
        for task in tasks:
            best_block, cost = self.assign_best_block_to_task(task, used_blocks)
            if not best_block:
                return float("inf"), []
            
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
        
        # 计算路径成本
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
            order_info = []
            for i, assignment in enumerate(best_assignments, 1):
                task = assignment["task"]
                block_pos = assignment["block_pos"]
                order_info.append(f"{i}. {task['num']}个{task['color']}物块到{task['to']}区 (物块位置: {block_pos[0]:.2f},{block_pos[1]:.2f})")
            
            for info in order_info:
                self.get_logger().info(info)
        
        return best_assignments

    # AMCL定位回调
    def _amcl_callback(self, msg):
        new_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        position_change = self.calculate_distance(new_pose, self.current_robot_pose)
        
        if position_change > 0.01:
            if self.current_robot_pose == (0.0, 0.0):
                self.get_logger().info(f"📌 初始定位：({new_pose[0]:.2f}, {new_pose[1]:.2f})")
            else:
                self.get_logger().debug(f"🤖 机器人位置更新：({self.current_robot_pose[0]:.2f}, {self.current_robot_pose[1]:.2f}) → ({new_pose[0]:.2f}, {new_pose[1]:.2f})")
            
            # 计算机器人速度（简化版）
            self.robot_velocity = (
                (new_pose[0] - self.current_robot_pose[0]) / 0.1,  # 假设100ms更新一次
                (new_pose[1] - self.current_robot_pose[1]) / 0.1
            )
            
            self.current_robot_pose = new_pose
            self.closest_cache = None
            
            # 输出当前距离所有障碍物的距离
            self._log_obstacle_distances()

    # 输出当前距离所有障碍物的距离
    def _log_obstacle_distances(self):
        """输出当前机器人距离所有障碍物的距离"""
        for name, obstacle in self.obstacles.items():
            distance = self.calculate_distance(self.current_robot_pose, obstacle.current_pos)
            safety_margin = self.robot_radius + obstacle.size / 2
            self.get_logger().debug(f"📏 距离{name}：{distance:.2f}m (安全余量：{safety_margin:.2f}m)")

    # 障碍物位置回调
    def _obstacle_pose_callback(self, obstacle_name, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        x = msg.position.x
        y = msg.position.y
        
        obstacle = self.obstacles.get(obstacle_name)
        if obstacle:
            old_pos = obstacle.current_pos
            obstacle.update_position(x, y, current_time)
            
            # 记录位置变化
            position_change = self.calculate_distance((x, y), old_pos)
            speed = obstacle.get_speed()
            
            # 计算距离机器人的距离
            distance_to_robot = self.calculate_distance(self.current_robot_pose, (x, y))
            
            if position_change > 0.05:  # 位置变化超过5cm才输出详细日志
                self.get_logger().info(f"🚧 {obstacle_name}位置更新：")
                self.get_logger().info(f"   旧位置：({old_pos[0]:.2f}, {old_pos[1]:.2f})")
                self.get_logger().info(f"   新位置：({x:.2f}, {y:.2f})")
                self.get_logger().info(f"   移动距离：{position_change:.2f}m")
                self.get_logger().info(f"   当前速度：{speed:.2f}m/s")
                self.get_logger().info(f"   距离机器人：{distance_to_robot:.2f}m")
                
                # 检查是否过近
                safety_margin = self.robot_radius + obstacle.size / 2
                if distance_to_robot < safety_margin + 0.1:
                    self.get_logger().warn(f"⚠️ {obstacle_name}距离过近！建议距离：{safety_margin:.2f}m，实际距离：{distance_to_robot:.2f}m")

    # 聊天指令回调（接收任务）
    def _chat_callback(self, msg):
        try:
            task_json = json.loads(msg.data)
            if not isinstance(task_json, list):
                task_json = [task_json]
            
            valid_tasks = []
            for task in task_json:
                required = ["color", "num", "to"]
                if all(k in task for k in required):
                    task["color"] = task["color"].lower()
                    task["to"] = task["to"].upper()
                    valid_tasks.append(task)
                else:
                    self.get_logger().error(f"❌ 任务格式错误（缺少color/num/to）：{task}")

            if valid_tasks:
                self.task_queue.extend(valid_tasks)
                self.get_logger().info(f"📥 接收{len(valid_tasks)}个任务，队列长度：{len(self.task_queue)}")
                
                # 输出当前环境状态
                self.get_logger().info("📊 当前环境状态：")
                self.get_logger().info(f"   机器人位置：({self.current_robot_pose[0]:.2f}, {self.current_robot_pose[1]:.2f})")
                self._log_obstacle_distances()
                
                if self.current_task is None and self.current_step == "WAIT_TASK":
                    self._optimize_and_process_tasks()

        except json.JSONDecodeError:
            self.get_logger().error("❌ 任务解析失败（非JSON格式）")

    # 优化并处理任务
    def _optimize_and_process_tasks(self):
        if not self.task_queue:
            return
        
        current_tasks = self.task_queue.copy()
        self.task_queue = []
        
        expanded_tasks = []
        for task in current_tasks:
            num = task["num"]
            for _ in range(num):
                expanded_tasks.append({
                    "color": task["color"],
                    "num": 1,
                    "to": task["to"],
                    "original_num": num
                })
        
        self.optimized_tasks = self.optimize_task_order(expanded_tasks)
        
        if self.optimized_tasks:
            self._process_next_optimized_task()
        else:
            self.get_logger().error("❌ 无法优化任务顺序，使用原始顺序执行")
            self.optimized_tasks = []
            for task in expanded_tasks:
                color = task["color"]
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
            self.get_logger().info("🎉 所有优化任务执行完成，等待新任务...")
            self.current_task = None
            self.current_step = "WAIT_TASK"
            self._update_foxglove()
            return

        self.current_assignment = self.optimized_tasks.pop(0)
        self.current_task = self.current_assignment["task"]
        self.completed_num = 0
        self.current_step = "NAV_TO_BLOCK"
        
        self.selected_block = (
            self.current_assignment["block_pos"][0],
            self.current_assignment["block_pos"][1],
            f"{self.current_task['color']}_cube_{self.current_assignment['block_idx'] + 1}",
            self.current_assignment["block_idx"]
        )

        self.grasp_confirmed = False
        self.place_confirmed = False
        self.current_grasp_retry = 0
        self.closest_cache = None
        self._clean_timers()

        color = self.current_task["color"]
        original_num = self.current_task.get("original_num", 1)
        current_index = self.current_task.get("current_index", 1)
        
        self.get_logger().info("=" * 80)
        self.get_logger().info(f"📋 开始执行任务 {current_index}/{original_num}")
        self.get_logger().info(f"   任务内容：{color}物块 → {self.current_task['to']}区")
        self.get_logger().info(f"   目标物块：{self.selected_block[2]}")
        self.get_logger().info(f"   物块位置：({self.current_assignment['block_pos'][0]:.2f}, {self.current_assignment['block_pos'][1]:.2f})")
        self.get_logger().info(f"   抓取位置：({self.current_assignment['grasp_pos'][0]:.2f}, {self.current_assignment['grasp_pos'][1]:.2f})")
        self.get_logger().info(f"   当前机器人位置：({self.current_robot_pose[0]:.2f}, {self.current_robot_pose[1]:.2f})")
        self._log_obstacle_distances()
        self.get_logger().info("=" * 80)
        
        self._update_foxglove()
        self.navigate_to_block()

    # 导航状态回调
    def _nav_status_callback(self, msg):
        status = msg.data.strip()
        previous_navigating = self.navigating
        self.navigating = False
        
        if status == "succeeded":
            self.get_logger().info("✅ 导航成功完成")
            if self.current_step == "NAV_TO_BLOCK":
                self.get_logger().info(f"📍 到达物块位置：({self.current_robot_pose[0]:.2f}, {self.current_robot_pose[1]:.2f})")
                self.current_step = "GRASP"
                self.trigger_grasp()
            elif self.current_step == "NAV_TO_AREA":
                self.get_logger().info(f"📍 到达目标区域：({self.current_robot_pose[0]:.2f}, {self.current_robot_pose[1]:.2f})")
                self.current_step = "PLACE"
                self.trigger_place()
        elif status == "failed":
            self.get_logger().error("❌ 导航失败")
            self.get_logger().info(f"   当前机器人位置：({self.current_robot_pose[0]:.2f}, {self.current_robot_pose[1]:.2f})")
            if "aborted" in status:
                self.get_logger().error("   原因：路径规划失败或避障中断")
            elif "rejected" in status:
                self.get_logger().error("   原因：目标被Nav2拒绝（可能不可达）")
            
            if self.current_step in ["NAV_TO_BLOCK", "NAV_TO_AREA"]:
                self.get_logger().warn("🔄 重试当前导航步骤")
                if self.current_step == "NAV_TO_BLOCK":
                    self.navigate_to_block()
                elif self.current_step == "NAV_TO_AREA":
                    self.navigate_to_area()

    # 机械臂状态回调
    def _arm_status_callback(self, msg):
        status = msg.data.strip()
        if status == "grasp_succeeded":
            self.grasp_confirmed = True
            self.get_logger().info("✅ 机械臂抓取成功")
        elif status == "place_succeeded":
            self.place_confirmed = True
            self.get_logger().info("✅ 机械臂放置成功")
        elif status == "grasp_failed":
            self.get_logger().error("❌ 机械臂抓取失败")
        elif status == "place_failed":
            self.get_logger().error("❌ 机械臂放置失败")

    # 连续碰撞检测
    def _continuous_collision_check(self):
        """连续检查碰撞风险"""
        if self.current_step not in ["NAV_TO_BLOCK", "NAV_TO_AREA"] or not self.navigating:
            return
        
        if not self.current_nav_goal:
            return
        
        # 检查紧急碰撞
        emergency_collision, obstacle_name, distance = self._check_emergency_collision()
        if emergency_collision:
            self._emergency_stop(obstacle_name, distance)
            return
        
        # 检查路径碰撞
        path_collision, obstacle_name, details = self._check_path_collision(self.current_robot_pose, self.current_nav_goal)
        
        if path_collision and not self.obstacle_waiting:
            # 输出详细的位置信息
            obstacle = self.obstacles.get(obstacle_name)
            if obstacle:
                self.get_logger().warn(f"🚨 发现障碍物{obstacle_name}在路径上，进入等待状态")
                self.get_logger().warn(f"   机器人位置：({self.current_robot_pose[0]:.2f}, {self.current_robot_pose[1]:.2f})")
                self.get_logger().warn(f"   {obstacle_name}位置：({obstacle.current_pos[0]:.2f}, {obstacle.current_pos[1]:.2f})")
                self.get_logger().warn(f"   两者距离：{self.calculate_distance(self.current_robot_pose, obstacle.current_pos):.2f}m")
                self.get_logger().warn(f"   风险详情：{details}")
            
            self.obstacle_waiting = True
            self.wait_start_time = self.get_clock().now().nanoseconds / 1e9
            
            # 暂停当前导航
            self.current_step = "WAIT_OBSTACLE"
            self._publish_empty_nav_target()  # 发布空目标停止导航
            self._update_foxglove()
            
        elif not path_collision and self.obstacle_waiting:
            waiting_time = self.get_clock().now().nanoseconds / 1e9 - self.wait_start_time
            self.get_logger().info(f"✅ 障碍物已离开路径，等待时间：{waiting_time:.1f}秒，继续导航")
            self.obstacle_waiting = False
            self.resume_navigation()
            
        elif self.obstacle_waiting:
            current_time = self.get_clock().now().nanoseconds / 1e9
            waiting_time = current_time - self.wait_start_time
            
            if int(waiting_time) % 2 == 0 and waiting_time > 0:
                # 定期输出等待状态和距离信息
                for name, obstacle in self.obstacles.items():
                    distance = self.calculate_distance(self.current_robot_pose, obstacle.current_pos)
                    self.get_logger().info(f"⏳ 等待{name}离开，已等待：{waiting_time:.1f}秒，当前距离：{distance:.2f}m")
            
            if waiting_time > self.MAX_WAIT_TIME:
                self.get_logger().error(f"❌ 等待时间过长（{waiting_time:.1f}秒 > {self.MAX_WAIT_TIME}秒），尝试重新规划路径")
                self.obstacle_waiting = False
                self.resume_navigation()

    # 检查紧急碰撞
    def _check_emergency_collision(self):
        """检查是否有紧急碰撞风险"""
        for obstacle_name, obstacle in self.obstacles.items():
            distance = self.calculate_distance(self.current_robot_pose, obstacle.current_pos)
            safety_margin = self.robot_radius + obstacle.size / 2 + 0.1
            
            if distance < self.EMERGENCY_DISTANCE:
                self.get_logger().error(f"🚨 紧急碰撞警告！{obstacle_name}距离机器人仅{distance:.2f}米")
                self.get_logger().error(f"   机器人位置：({self.current_robot_pose[0]:.2f}, {self.current_robot_pose[1]:.2f})")
                self.get_logger().error(f"   {obstacle_name}位置：({obstacle.current_pos[0]:.2f}, {obstacle.current_pos[1]:.2f})")
                self.get_logger().error(f"   安全距离：{self.EMERGENCY_DISTANCE:.2f}m，实际距离：{distance:.2f}m")
                return True, obstacle_name, distance
                
        return False, None, 0.0

    # 紧急停止
    def _emergency_stop(self, obstacle_name=None, distance=0.0):
        """紧急停止机器人"""
        self.get_logger().error("🚨 执行紧急停止！")
        self.get_logger().error(f"   机器人位置：({self.current_robot_pose[0]:.2f}, {self.current_robot_pose[1]:.2f})")
        if obstacle_name:
            obstacle = self.obstacles.get(obstacle_name)
            if obstacle:
                self.get_logger().error(f"   {obstacle_name}位置：({obstacle.current_pos[0]:.2f}, {obstacle.current_pos[1]:.2f})")
                self.get_logger().error(f"   距离：{distance:.2f}m")
        
        self.current_step = "EMERGENCY_STOP"
        self.navigating = False
        self.obstacle_waiting = False
        
        # 发布紧急停止指令
        self.emergency_stop_pub.publish(String(data="emergency_stop"))
        
        # 发布空导航目标
        self._publish_empty_nav_target()
        
        self._update_foxglove()
        
        # 5秒后尝试恢复
        self.create_timer(5.0, self._recover_from_emergency)

    # 从紧急状态恢复
    def _recover_from_emergency(self):
        """从紧急停止状态恢复"""
        self.get_logger().info("🔄 尝试从紧急停止状态恢复")
        
        # 检查是否安全
        safe_to_resume = True
        for obstacle_name, obstacle in self.obstacles.items():
            distance = self.calculate_distance(self.current_robot_pose, obstacle.current_pos)
            safety_margin = self.robot_radius + obstacle.size / 2 + 0.2
            
            self.get_logger().info(f"📏 检查{obstacle_name}：距离{distance:.2f}m，安全距离{safety_margin:.2f}m")
            
            if distance < safety_margin:
                safe_to_resume = False
                self.get_logger().warn(f"⚠️ {obstacle_name}仍然太近（{distance:.2f}米），继续等待")
                break
        
        if safe_to_resume:
            self.get_logger().info("✅ 环境安全，恢复导航")
            self.current_step = "NAV_TO_BLOCK" if self.current_assignment else "WAIT_TASK"
            self.resume_navigation()
        else:
            # 1秒后再次检查
            self.create_timer(1.0, self._recover_from_emergency)

    # 发布空导航目标（停止导航）
    def _publish_empty_nav_target(self):
        """发布空导航目标以停止机器人"""
        nav_msg = String()
        nav_msg.data = json.dumps({"type": "empty"})
        self.nav_target_pub.publish(nav_msg)

    # 检查路径是否会与障碍物碰撞
    def _check_path_collision(self, start_pos, end_pos):
        """检查从起点到终点的路径是否会与障碍物碰撞"""
        # 计算机器人到达终点的预计时间
        distance = self.calculate_distance(start_pos, end_pos)
        robot_speed = math.hypot(self.robot_velocity[0], self.robot_velocity[1])
        if robot_speed < 0.01:
            robot_speed = 0.5  # 默认速度
        estimated_time = distance / robot_speed
        
        self.get_logger().debug(f"📏 路径检查：起点({start_pos[0]:.2f},{start_pos[1]:.2f}) → 终点({end_pos[0]:.2f},{end_pos[1]:.2f})")
        self.get_logger().debug(f"   距离：{distance:.2f}米，预计{estimated_time:.1f}秒到达")
        
        # 检查每个障碍物
        for obstacle_name, obstacle in self.obstacles.items():
            obstacle_pos = obstacle.current_pos
            obstacle_size = obstacle.size
            
            # 计算安全距离
            safety_distance = self.robot_radius + obstacle_size / 2 + 0.2
            
            # 预测障碍物在不同时间点的位置
            prediction_times = [0.0, estimated_time * 0.3, estimated_time * 0.6, estimated_time]
            
            collision_risk = False
            closest_distance = float("inf")
            risk_details = []
            
            for t in prediction_times:
                # 预测机器人位置
                robot_progress = min(t / estimated_time, 1.0) if estimated_time > 0 else 1.0
                robot_predicted_x = start_pos[0] + (end_pos[0] - start_pos[0]) * robot_progress
                robot_predicted_y = start_pos[1] + (end_pos[1] - start_pos[1]) * robot_progress
                robot_predicted_pos = (robot_predicted_x, robot_predicted_y)
                
                # 预测障碍物位置
                obstacle_predicted_pos = obstacle.predict_position(t)
                
                # 计算距离
                distance_between = self.calculate_distance(robot_predicted_pos, obstacle_predicted_pos)
                
                if distance_between < closest_distance:
                    closest_distance = distance_between
                
                if distance_between < safety_distance:
                    collision_risk = True
                    risk_details.append(f"t={t:.1f}s时距离{distance_between:.2f}m < 安全距离{safety_distance:.2f}m")
            
            # 检查当前障碍物是否在路径附近
            distance_to_path = self.distance_to_line_segment(obstacle_pos, start_pos, end_pos)
            current_distance_to_robot = self.calculate_distance(start_pos, obstacle_pos)
            
            self.get_logger().debug(f"🔍 检查{obstacle_name}：")
            self.get_logger().debug(f"   当前位置：({obstacle_pos[0]:.2f}, {obstacle_pos[1]:.2f})")
            self.get_logger().debug(f"   距离机器人：{current_distance_to_robot:.2f}m")
            self.get_logger().debug(f"   距离路径：{distance_to_path:.2f}m")
            self.get_logger().debug(f"   最近预测距离：{closest_distance:.2f}m")
            
            if collision_risk or (distance_to_path < safety_distance * 1.5):
                details = f"最近距离{closest_distance:.2f}m，路径距离{distance_to_path:.2f}m"
                if risk_details:
                    details += "，风险时间点：" + " | ".join(risk_details)
                
                self.get_logger().error(f"❌ 检测到碰撞风险：{obstacle_name}")
                self.get_logger().error(f"   机器人位置：({start_pos[0]:.2f}, {start_pos[1]:.2f})")
                self.get_logger().error(f"   {obstacle_name}位置：({obstacle_pos[0]:.2f}, {obstacle_pos[1]:.2f})")
                self.get_logger().error(f"   两者距离：{current_distance_to_robot:.2f}m")
                self.get_logger().error(f"   风险详情：{details}")
                return True, obstacle_name, details
        
        self.get_logger().debug("✅ 路径安全，无碰撞风险")
        return False, None, ""

    # 更新障碍物预测
    def _update_obstacle_predictions(self):
        """更新障碍物预测信息"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        for obstacle_name, obstacle in self.obstacles.items():
            # 预测未来位置
            future_pos = obstacle.predict_position(self.PREDICTION_TIME)
            
            # 检查是否到达边界
            if obstacle.start_pos and obstacle.end_pos:
                dist_to_end = self.calculate_distance(obstacle.current_pos, obstacle.end_pos)
                dist_to_start = self.calculate_distance(obstacle.current_pos, obstacle.start_pos)
                
                if dist_to_end < 0.15:
                    if obstacle.direction != -1:
                        obstacle.direction = -1
                        self.get_logger().info(f"🔄 {obstacle_name}到达终点，开始返回")
                        self.get_logger().info(f"   当前位置：({obstacle.current_pos[0]:.2f}, {obstacle.current_pos[1]:.2f})")
                        self.get_logger().info(f"   终点位置：({obstacle.end_pos[0]:.2f}, {obstacle.end_pos[1]:.2f})")
                elif dist_to_start < 0.15:
                    if obstacle.direction != 1:
                        obstacle.direction = 1
                        self.get_logger().info(f"🔄 {obstacle_name}到达起点，开始前进")
                        self.get_logger().info(f"   当前位置：({obstacle.current_pos[0]:.2f}, {obstacle.current_pos[1]:.2f})")
                        self.get_logger().info(f"   起点位置：({obstacle.start_pos[0]:.2f}, {obstacle.start_pos[1]:.2f})")

    # 恢复导航
    def resume_navigation(self):
        """恢复被暂停的导航"""
        self.get_logger().info("🔄 恢复导航...")
        self.get_logger().info(f"   当前机器人位置：({self.current_robot_pose[0]:.2f}, {self.current_robot_pose[1]:.2f})")
        if self.current_nav_goal:
            self.get_logger().info(f"   导航目标：({self.current_nav_goal[0]:.2f}, {self.current_nav_goal[1]:.2f})")
        
        self._log_obstacle_distances()
        
        if self.current_step == "NAV_TO_BLOCK":
            self.navigate_to_block()
        elif self.current_step == "NAV_TO_AREA":
            self.navigate_to_area()
        self._update_foxglove()

    # 导航到物块
    def navigate_to_block(self):
        self._clean_timers()

        if not self.current_assignment:
            return

        self.grasp_confirmed = False
        self.current_grasp_retry = 0
        self.closest_cache = None
        self.obstacle_waiting = False

        grasp_pos = self.current_assignment["grasp_pos"]
        cube_name = self.selected_block[2]
        self.current_nav_goal = grasp_pos

        self.get_logger().info(f"\n🚀 准备导航到物块：{cube_name}")
        self.get_logger().info(f"🎯 目标抓取位置：({grasp_pos[0]:.2f}, {grasp_pos[1]:.2f})")
        self.get_logger().info(f"🤖 当前机器人位置：({self.current_robot_pose[0]:.2f}, {self.current_robot_pose[1]:.2f})")
        self.get_logger().info(f"📏 距离目标：{self.calculate_distance(self.current_robot_pose, grasp_pos):.2f}m")
        
        # 检查路径是否有碰撞风险
        collision_detected, obstacle_name, details = self._check_path_collision(self.current_robot_pose, grasp_pos)
        
        if collision_detected:
            self.get_logger().warn(f"⚠️  路径上发现障碍物{obstacle_name}，进入等待状态")
            self.current_step = "WAIT_OBSTACLE"
            self.wait_start_time = self.get_clock().now().nanoseconds / 1e9
            self._update_foxglove()
            return

        # 发布导航目标
        nav_msg = String()
        nav_msg.data = json.dumps({"type": "custom", "x": grasp_pos[0], "y": grasp_pos[1], "yaw": 0.0})
        self.nav_target_pub.publish(nav_msg)
        self.target_cube_pub.publish(String(data=cube_name))
        self.current_step = "NAV_TO_BLOCK"
        self.navigating = True
        self._update_foxglove()
        self.get_logger().info(f"✅ 开始导航到物块：{cube_name}")

    # 触发抓取
    def trigger_grasp(self):
        if self.current_grasp_retry >= self.max_grasp_retry:
            self.get_logger().error(f"❌ 抓取重试达{self.max_grasp_retry}次，跳过该物块")
            self.current_grasp_retry = 0
            self._process_next_optimized_task()
            return

        self.get_logger().info(f"🤏 触发抓取（重试次数：{self.current_grasp_retry}/{self.max_grasp_retry}）")
        self.get_logger().info(f"📍 当前位置：({self.current_robot_pose[0]:.2f}, {self.current_robot_pose[1]:.2f})")
        self.arm_cargo_pub.publish(String(data="arrived_at_cargo"))
        self.grasp_timer = self.create_timer(self.GRASP_TIMEOUT, self._check_grasp)

    # 检查抓取结果
    def _check_grasp(self):
        if self.grasp_timer:
            self.grasp_timer.cancel()
            self.grasp_timer.destroy()
            self.grasp_timer = None

        if self.grasp_confirmed:
            self.get_logger().info("✅ 抓取成功")
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
            self.current_grasp_retry += 1
            self.get_logger().warn(f"❌ 抓取超时/失败，准备重试（{self.current_grasp_retry}/{self.max_grasp_retry}）")
            self.trigger_grasp()
        
        gc.collect()

    # 导航到目标区域
    def navigate_to_area(self):
        if not self.current_assignment:
            return

        area_pos = self.current_assignment["area_pos"]
        area = self.current_task["to"]
        self.current_nav_goal = area_pos

        self.get_logger().info(f"\n🚀 准备导航到目标区域：{area}区")
        self.get_logger().info(f"🎯 目标区域位置：({area_pos[0]:.2f}, {area_pos[1]:.2f})")
        self.get_logger().info(f"🤖 当前机器人位置：({self.current_robot_pose[0]:.2f}, {self.current_robot_pose[1]:.2f})")
        self.get_logger().info(f"📏 距离目标：{self.calculate_distance(self.current_robot_pose, area_pos):.2f}m")
        
        # 检查路径是否有碰撞风险
        collision_detected, obstacle_name, details = self._check_path_collision(self.current_robot_pose, area_pos)
        
        if collision_detected:
            self.get_logger().warn(f"⚠️  路径上发现障碍物{obstacle_name}，进入等待状态")
            self.current_step = "WAIT_OBSTACLE"
            self.wait_start_time = self.get_clock().now().nanoseconds / 1e9
            self._update_foxglove()
            return

        nav_msg = String()
        nav_msg.data = json.dumps({"type": "custom", "x": area_pos[0], "y": area_pos[1], "yaw": 0.0})
        self.nav_target_pub.publish(nav_msg)
        self.current_step = "NAV_TO_AREA"
        self.navigating = True
        self._update_foxglove()
        self.get_logger().info(f"✅ 开始导航到目标区域 {area}")

    # 触发放置
    def trigger_place(self):
        self.get_logger().info("👇 触发放置")
        self.get_logger().info(f"📍 当前位置：({self.current_robot_pose[0]:.2f}, {self.current_robot_pose[1]:.2f})")
        self.place_confirmed = False
        self.arm_area_pub.publish(String(data="arrived_at_area"))
        self.place_timer = self.create_timer(self.PLACE_TIMEOUT, self._check_place)

    # 检查放置结果
    def _check_place(self):
        if self.place_timer:
            self.place_timer.cancel()
            self.place_timer.destroy()
            self.place_timer = None

        if self.place_confirmed:
            self.get_logger().info("✅ 放置成功")
            self.completed_num += 1
            original_num = self.current_task.get("original_num", 1)
            current_index = self.current_task.get("current_index", 1)
            
            self.get_logger().info(f"📊 放置完成（{current_index}/{original_num}）")
            self.get_logger().info(f"📊 当前任务完成率：{self.completed_num}/{original_num}")
            
            self._process_next_optimized_task()
        else:
            self.get_logger().warn("❌ 放置超时/失败，准备重试")
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
        pick_data = -1 if self.current_step in ["NAV_TO_BLOCK", "WAIT_TASK", "WAIT_OBSTACLE", "EMERGENCY_STOP"] else \
                    0 if self.current_step in ["GRASP", "NAV_TO_AREA"] else 1
        self.foxglove_pubs["pick"].publish(Int32(data=pick_data))

        # 更新步骤状态
        step_map = {
            "WAIT_TASK": 0,
            "NAV_TO_BLOCK": 1,
            "GRASP": 2,
            "NAV_TO_AREA": 3,
            "PLACE": 3,
            "WAIT_OBSTACLE": 4,
            "EMERGENCY_STOP": 5
        }
        step_data = step_map.get(self.current_step, 0)
        self.foxglove_pubs["cur"].publish(Int32(data=step_data))
        
        # 输出当前状态信息
        step_names = {
            0: "等待任务",
            1: "导航到物块",
            2: "抓取物块",
            3: "导航到区域/放置",
            4: "等待障碍物",
            5: "紧急停止"
        }
        self.get_logger().debug(f"📋 当前状态：{step_names[step_data]} ({self.current_step})")

    # 节点销毁时清理资源
    def destroy_node(self):
        self.get_logger().info("🛑 正在关闭主控节点...")
        self._clean_timers()
        if self.obstacle_timer:
            self.obstacle_timer.cancel()
            self.obstacle_timer.destroy()
        if self.collision_check_timer:
            self.collision_check_timer.cancel()
            self.collision_check_timer.destroy()
            
        self.target_blocks = []
        self.current_task = None
        self.task_queue = []
        self.optimized_tasks = []
        self.selected_block = None
        self.current_assignment = None
        super().destroy_node()
        gc.collect()
        self.get_logger().info("✅ 主控节点已关闭")


def main(args=None):
    rclpy.init(args=args)
    main_controller = MainController()
    try:
        rclpy.spin(main_controller)
    except KeyboardInterrupt:
        main_controller.get_logger().info("👋 用户中断，关闭节点")
    finally:
        main_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()