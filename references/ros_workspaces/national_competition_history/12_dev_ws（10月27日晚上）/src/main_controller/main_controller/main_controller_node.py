import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from rclpy.timer import Timer
import json
import math
import weakref
import gc
from itertools import permutations
import time

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

        # 移动障碍物配置（根据world文件）
        self.MOVING_OBSTACLES = {
            "obstacle2": {
                "start": (3.0, 0.0),
                "end": (8.0, 0.0),
                "speed": 0.05,  # 单位/秒
                "size": 0.75,    # 立方体边长
                "current_pos": (3.0, 0.0),
                "direction": 1,  # 1: 向终点移动, -1: 向起点移动
                "last_update_time": time.time()
            },
            "obstacle3": {  # 假设的第二个障碍物，根据实际情况调整
                "start": (-2.5, 3.0),
                "end": (-2.5, -2.0),
                "speed": 0.05,
                "size": 0.75,
                "current_pos": (-2.5, 3.0),
                "direction": 1,
                "last_update_time": time.time()
            }
        }

        # 避障参数
        self.SAFE_DISTANCE = 1.5  # 安全距离
        self.MAX_WAIT_TIME = 30.0  # 最大等待时间（秒）
        self.OBSTACLE_CHECK_INTERVAL = 0.5  # 障碍物检查间隔（秒）

        # 核心变量
        self.current_robot_pose = (0.0, 0.0)
        self.target_blocks = []
        self.GRASP_OFFSET = 0.4
        self.OFFSET_AXIS = "x"
        self.retry_timer: Timer = None
        self.grasp_timer: Timer = None
        self.place_timer: Timer = None
        self.obstacle_timer: Timer = None  # 障碍物更新定时器
        self.wait_timer: Timer = None      # 等待安全时机定时器
        self.current_task = None  # 当前执行任务
        self.task_queue = []      # 原始任务队列
        self.optimized_tasks = [] # 优化后的任务执行顺序
        self.completed_num = 0    # 当前任务已完成数量
        self.current_step = "WAIT_TASK"  # 状态：WAIT_TASK/NAV_TO_BLOCK/GRASP/NAV_TO_AREA/PLACE/WAIT_OBSTACLE
        self.selected_block = None
        self.grasp_confirmed = False
        self.place_confirmed = False
        self.max_grasp_retry = 3
        self.current_grasp_retry = 0
        self.closest_cache = None
        self.cache_expire = 2.0
        self.last_cache_time = 0.0
        self.nav_target = None  # 当前导航目标
        self.wait_start_time = 0.0  # 等待开始时间
        
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
            String, "/nav_status", self._nav_status_callback, 10
        )
        self.arm_status_sub = self.create_subscription(
            String, "/arm_status", self._arm_status_callback, 10
        )

        # 发布器
        self.target_cube_pub = self.create_publisher(String, "/current_target_cube", 10)
        self.nav_target_pub = self.create_publisher(String, "/manual_nav_target", 10)
        self.arm_cargo_pub = self.create_publisher(String, "/nav_done_cargo", 10)
        self.arm_area_pub = self.create_publisher(String, "/nav_done_area", 10)
        self.foxglove_pubs = {
            "color": self.create_publisher(Int32, "/color", 10),
            "ask": self.create_publisher(Int32, "/ask", 10),
            "pick": self.create_publisher(Int32, "/pick", 10),
            "cur": self.create_publisher(Int32, "/cur", 10)
        }

        # 启动障碍物更新定时器
        self.obstacle_timer = self.create_timer(self.OBSTACLE_CHECK_INTERVAL, self._update_obstacle_positions)

        self.get_logger().info("主控节点（支持智能路径规划和动态避障）启动成功")

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

    # 更新移动障碍物位置
    def _update_obstacle_positions(self):
        current_time = time.time()
        
        for obs_name, obs_info in self.MOVING_OBSTACLES.items():
            # 计算时间差
            time_diff = current_time - obs_info["last_update_time"]
            
            # 计算移动距离
            distance_to_move = obs_info["speed"] * time_diff
            
            # 获取当前位置和目标位置
            current_x, current_y = obs_info["current_pos"]
            start_x, start_y = obs_info["start"]
            end_x, end_y = obs_info["end"]
            
            # 计算到目标点的距离
            if obs_info["direction"] == 1:
                target_x, target_y = end_x, end_y
            else:
                target_x, target_y = start_x, start_y
            
            distance_to_target = self.calculate_distance((current_x, current_y), (target_x, target_y))
            
            if distance_to_move >= distance_to_target:
                # 到达目标点，改变方向
                obs_info["current_pos"] = (target_x, target_y)
                obs_info["direction"] *= -1
            else:
                # 计算移动方向
                dx = target_x - current_x
                dy = target_y - current_y
                total_distance = math.hypot(dx, dy)
                
                if total_distance > 0:
                    move_x = (dx / total_distance) * distance_to_move
                    move_y = (dy / total_distance) * distance_to_move
                    
                    new_x = current_x + move_x
                    new_y = current_y + move_y
                    
                    obs_info["current_pos"] = (new_x, new_y)
            
            obs_info["last_update_time"] = current_time

    # 检查路径是否安全
    def is_path_safe(self, start_pos, end_pos):
        """检查从起点到终点的路径是否安全"""
        # 简化检查：检查起点、终点以及中间点是否与障碍物距离过近
        check_points = [start_pos, end_pos]
        
        # 添加中间检查点
        mid_x = (start_pos[0] + end_pos[0]) / 2
        mid_y = (start_pos[1] + end_pos[1]) / 2
        check_points.append((mid_x, mid_y))
        
        # 检查每个点与障碍物的距离
        for (x, y) in check_points:
            for obs_name, obs_info in self.MOVING_OBSTACLES.items():
                obs_x, obs_y = obs_info["current_pos"]
                distance = self.calculate_distance((x, y), (obs_x, obs_y))
                
                # 考虑障碍物大小
                safe_distance = self.SAFE_DISTANCE + obs_info["size"] / 2
                
                if distance < safe_distance:
                    self.get_logger().warn(f"路径不安全：与{obs_name}距离过近 ({distance:.2f}米 < {safe_distance:.2f}米)")
                    return False
        
        return True

    # 预测障碍物未来位置
    def predict_obstacle_position(self, obs_name, predict_time):
        """预测障碍物在predict_time秒后的位置"""
        obs_info = self.MOVING_OBSTACLES.get(obs_name)
        if not obs_info:
            return None
            
        current_x, current_y = obs_info["current_pos"]
        start_x, start_y = obs_info["start"]
        end_x, end_y = obs_info["end"]
        direction = obs_info["direction"]
        speed = obs_info["speed"]
        
        distance_to_move = speed * predict_time
        
        if direction == 1:
            target_x, target_y = end_x, end_y
        else:
            target_x, target_y = start_x, start_y
        
        distance_to_target = self.calculate_distance((current_x, current_y), (target_x, target_y))
        
        if distance_to_move >= distance_to_target:
            # 会到达目标点并改变方向
            remaining_time = distance_to_target / speed
            remaining_distance = distance_to_move - distance_to_target
            
            # 改变方向后的新目标
            if direction == 1:
                new_target_x, new_target_y = start_x, start_y
            else:
                new_target_x, new_target_y = end_x, end_y
            
            # 计算改变方向后的移动距离
            dx = new_target_x - target_x
            dy = new_target_y - target_y
            total_distance = math.hypot(dx, dy)
            
            if total_distance > 0:
                move_x = (dx / total_distance) * remaining_distance
                move_y = (dy / total_distance) * remaining_distance
                
                final_x = target_x + move_x
                final_y = target_y + move_y
            else:
                final_x, final_y = target_x, target_y
                
            return (final_x, final_y)
        else:
            # 直接移动
            dx = target_x - current_x
            dy = target_y - current_y
            total_distance = math.hypot(dx, dy)
            
            if total_distance > 0:
                move_x = (dx / total_distance) * distance_to_move
                move_y = (dy / total_distance) * distance_to_move
                
                final_x = current_x + move_x
                final_y = current_y + move_y
                return (final_x, final_y)
            else:
                return (current_x, current_y)

    # 计算安全通过时间
    def calculate_safe_pass_time(self, start_pos, end_pos, robot_speed=0.2):
        """计算安全通过路径所需的时间和最早安全时间"""
        # 计算机器人通过路径的时间
        path_distance = self.calculate_distance(start_pos, end_pos)
        robot_pass_time = path_distance / robot_speed
        
        # 检查现在是否安全
        if self.is_path_safe(start_pos, end_pos):
            self.get_logger().info(f"路径当前安全，预计通过时间：{robot_pass_time:.2f}秒")
            return 0.0, robot_pass_time  # 立即可以通过
        
        self.get_logger().info("路径当前不安全，寻找安全时机...")
        
        # 预测未来5秒内的安全时间窗口
        max_prediction_time = 5.0
        time_step = 0.5
        
        for predict_start_time in range(0, int(max_prediction_time / time_step) + 1):
            predict_start_time *= time_step
            
            # 检查在predict_start_time时开始移动是否安全
            safe = True
            
            # 检查移动过程中的关键时间点
            check_times = [0.0, robot_pass_time / 3, 2 * robot_pass_time / 3, robot_pass_time]
            
            for time_offset in check_times:
                current_time = predict_start_time + time_offset
                
                # 预测机器人在该时间点的位置
                progress = min(time_offset / robot_pass_time, 1.0)
                robot_x = start_pos[0] + (end_pos[0] - start_pos[0]) * progress
                robot_y = start_pos[1] + (end_pos[1] - start_pos[1]) * progress
                
                # 检查与所有障碍物的距离
                for obs_name in self.MOVING_OBSTACLES.keys():
                    obs_pos = self.predict_obstacle_position(obs_name, current_time)
                    if not obs_pos:
                        continue
                        
                    obs_x, obs_y = obs_pos
                    obs_size = self.MOVING_OBSTACLES[obs_name]["size"]
                    
                    distance = self.calculate_distance((robot_x, robot_y), (obs_x, obs_y))
                    safe_distance = self.SAFE_DISTANCE + obs_size / 2
                    
                    if distance < safe_distance:
                        safe = False
                        break
                
                if not safe:
                    break
            
            if safe:
                self.get_logger().info(f"找到安全时机：{predict_start_time:.2f}秒后开始移动，预计通过时间：{robot_pass_time:.2f}秒")
                return predict_start_time, robot_pass_time
        
        self.get_logger().warn("在预测时间窗口内未找到安全时机")
        return None, None

    # 等待安全时机
    def wait_for_safe_moment(self, target_pos):
        """等待安全时机后再导航"""
        if self.current_step != "WAIT_OBSTACLE":
            return
            
        start_pos = self.current_robot_pose
        end_pos = target_pos
        
        # 计算安全通过时间
        wait_time, pass_time = self.calculate_safe_pass_time(start_pos, end_pos)
        
        if wait_time is not None:
            if wait_time <= 0.5:  # 如果等待时间很短，立即执行
                self.execute_navigation(target_pos)
            else:
                self.get_logger().info(f"等待{wait_time:.2f}秒后再导航...")
                self.wait_timer = self.create_timer(wait_time, lambda: self.execute_navigation(target_pos))
        else:
            # 如果没有找到安全时机，尝试直接导航（作为最后的手段）
            self.get_logger().warn("无法找到安全时机，尝试直接导航...")
            self.execute_navigation(target_pos)

    # 执行导航
    def execute_navigation(self, target_pos):
        """执行实际的导航操作"""
        if self.wait_timer:
            self.wait_timer.cancel()
            self.wait_timer.destroy()
            self.wait_timer = None
            
        if self.current_step != "WAIT_OBSTACLE":
            return
            
        # 再次检查路径是否安全
        if not self.is_path_safe(self.current_robot_pose, target_pos):
            self.get_logger().warn("导航前路径仍不安全，重新等待...")
            self.wait_timer = self.create_timer(1.0, lambda: self.wait_for_safe_moment(target_pos))
            return
        
        # 发布导航目标
        nav_msg = String()
        nav_msg.data = json.dumps({"type": "custom", "x": target_pos[0], "y": target_pos[1], "yaw": 0.0})
        self.nav_target_pub.publish(nav_msg)
        
        if self.current_step == "WAIT_OBSTACLE" and self.nav_target == "block":
            self.current_step = "NAV_TO_BLOCK"
            cube_name = self.selected_block[2]
            self.target_cube_pub.publish(String(data=cube_name))
            self.get_logger().info(f"开始导航到物块：{cube_name}（抓取位置：{target_pos[0]:.2f}, {target_pos[1]:.2f}）")
        elif self.current_step == "WAIT_OBSTACLE" and self.nav_target == "area":
            self.current_step = "NAV_TO_AREA"
            area = self.current_task["to"]
            self.get_logger().info(f"开始导航到目标区域 {area}：({target_pos[0]:.2f}, {target_pos[1]:.2f})")
        
        self._update_foxglove()

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

    # 导航到物块（使用优化路径和动态避障）
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

        # 检查路径安全性
        if self.is_path_safe(self.current_robot_pose, grasp_pos):
            # 路径安全，直接导航
            nav_msg = String()
            nav_msg.data = json.dumps({"type": "custom", "x": grasp_pos[0], "y": grasp_pos[1], "yaw": 0.0})
            self.nav_target_pub.publish(nav_msg)
            self.target_cube_pub.publish(String(data=cube_name))
            self.current_step = "NAV_TO_BLOCK"
            self._update_foxglove()
            self.get_logger().info(f"导航到物块：{cube_name}（抓取位置：{grasp_pos[0]:.2f}, {grasp_pos[1]:.2f}）")
        else:
            # 路径不安全，等待安全时机
            self.get_logger().warn("导航到物块的路径不安全，等待安全时机...")
            self.nav_target = "block"
            self.current_step = "WAIT_OBSTACLE"
            self._update_foxglove()
            self.wait_for_safe_moment(grasp_pos)

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

    # 导航到目标区域（使用优化路径和动态避障）
    def navigate_to_area(self):
        if not self.current_assignment:
            return

        area_pos = self.current_assignment["area_pos"]
        area = self.current_task["to"]
        
        # 检查路径安全性
        if self.is_path_safe(self.current_robot_pose, area_pos):
            # 路径安全，直接导航
            nav_msg = String()
            nav_msg.data = json.dumps({"type": "custom", "x": area_pos[0], "y": area_pos[1], "yaw": 0.0})
            self.nav_target_pub.publish(nav_msg)
            self.current_step = "NAV_TO_AREA"
            self._update_foxglove()
            self.get_logger().info(f"导航到目标区域 {area}：({area_pos[0]:.2f}, {area_pos[1]:.2f})")
        else:
            # 路径不安全，等待安全时机
            self.get_logger().warn("导航到目标区域的路径不安全，等待安全时机...")
            self.nav_target = "area"
            self.current_step = "WAIT_OBSTACLE"
            self._update_foxglove()
            self.wait_for_safe_moment(area_pos)

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
        for timer in [self.retry_timer, self.grasp_timer, self.place_timer, self.wait_timer]:
            if timer:
                timer.cancel()
                timer.destroy()
        self.retry_timer = None
        self.grasp_timer = None
        self.place_timer = None
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
        pick_data = -1 if self.current_step in ["NAV_TO_BLOCK", "WAIT_TASK", "WAIT_OBSTACLE"] else \
                    0 if self.current_step in ["GRASP", "NAV_TO_AREA"] else 1
        self.foxglove_pubs["pick"].publish(Int32(data=pick_data))

        # 更新步骤状态
        step_map = {
            "WAIT_TASK": 0,
            "NAV_TO_BLOCK": 1,
            "GRASP": 2,
            "NAV_TO_AREA": 3,
            "PLACE": 3,
            "WAIT_OBSTACLE": 4  # 新增等待障碍物状态
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
