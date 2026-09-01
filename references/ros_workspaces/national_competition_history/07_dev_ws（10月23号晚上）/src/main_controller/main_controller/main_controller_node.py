import rclpy
import weakref
import json
import math
import gc
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.timer import Timer
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseWithCovarianceStamped


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

        # 核心变量（含任务队列）
        self.current_robot_pose = (0.0, 0.0)
        self.target_blocks = []
        self.GRASP_OFFSET = 0.4
        self.OFFSET_AXIS = "x"
        self.retry_timer: Timer = None
        self.grasp_timer: Timer = None
        self.place_timer: Timer = None
        self.current_task = None  # 当前执行任务
        self.task_queue = []      # 任务队列
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
        
        # 降低机械臂相关超时时间
        self.GRASP_TIMEOUT = 8.0  # 抓取超时时间（从15秒减少到8秒）
        self.PLACE_TIMEOUT = 8.0  # 放置超时时间（从15秒减少到8秒）

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

        self.get_logger().info("主控节点（支持多任务队列）启动成功")

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
            self.closest_cache = None  # 位置更新后清除缓存

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
                    # 标准化参数（颜色小写，区域大写）
                    task["color"] = task["color"].lower()
                    task["to"] = task["to"].upper()
                    valid_tasks.append(task)
                else:
                    self.get_logger().error(f"任务格式错误（缺少color/num/to）：{task}")

            if valid_tasks:
                self.task_queue.extend(valid_tasks)
                self.get_logger().info(f"接收{len(valid_tasks)}个任务，队列长度：{len(self.task_queue)}")
                
                # 若当前无任务，立即开始处理
                if self.current_task is None and self.current_step == "WAIT_TASK":
                    self._process_next_task()

        except json.JSONDecodeError:
            self.get_logger().error("任务解析失败（非JSON格式）")

    def _chat_callback(self, msg):
        self_ref = weakref.ref(self)
        self._chat_callback_impl(self_ref, msg)
        del self_ref

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

    # 任务队列处理
    def _process_next_task(self):
        """从队列取出下一个任务并初始化执行"""
        if not self.task_queue:
            self.get_logger().info("任务队列已空，等待新任务...")
            self.current_task = None
            self.current_step = "WAIT_TASK"  # 关键修复：任务队列为空时设置为等待状态
            self._update_foxglove()
            return

        # 取出队列首个任务
        self.current_task = self.task_queue.pop(0)
        self.completed_num = 0
        self.current_step = "NAV_TO_BLOCK"
        
        # 初始化目标物块列表
        color = self.current_task["color"]
        self.target_blocks = self.RED_BLOCKS.copy() if color == "red" else self.BLUE_BLOCKS.copy()
        # 重置物块抓取状态
        for i in range(len(self.target_blocks)):
            x, y, _ = self.target_blocks[i]
            self.target_blocks[i] = (x, y, False)

        # 重置状态变量
        self.grasp_confirmed = False
        self.place_confirmed = False
        self.current_grasp_retry = 0
        self.closest_cache = None
        self._clean_timers()

        self.get_logger().info(
            f"开始执行任务：{self.current_task['num']}个{color}物块 → {self.current_task['to']}区"
        )
        self._update_foxglove()
        self.navigate_to_block()

    # 获取最近的未抓取物块
    def get_closest_block(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        # 缓存未过期则直接使用
        if (current_time - self.last_cache_time) < self.cache_expire and self.closest_cache:
            block_x, block_y, block_idx = self.closest_cache
            color = self.current_task["color"]
            cube_id = self._get_cube_id(block_x, block_y, color)
            return (block_x, block_y, f"{color}_cube_{cube_id}", block_idx)

        if not self.target_blocks:
            return None

        # 计算最近未抓取物块
        min_dist = float("inf")
        closest = None
        for idx, (x, y, is_grasped) in enumerate(self.target_blocks):
            if is_grasped:
                continue
            dist = math.hypot(x - self.current_robot_pose[0], y - self.current_robot_pose[1])
            if dist < min_dist:
                min_dist = dist
                closest = (x, y, idx)

        if not closest:
            return None

        # 更新缓存
        self.closest_cache = closest
        self.last_cache_time = current_time
        x, y, idx = closest
        color = self.current_task["color"]
        cube_id = self._get_cube_id(x, y, color)
        return (x, y, f"{color}_cube_{cube_id}", idx)

    # 获取物块ID
    def _get_cube_id(self, x, y, color):
        blocks = self.RED_BLOCKS if color == "red" else self.BLUE_BLOCKS
        for i, (bx, by, _) in enumerate(blocks):
            if abs(x - bx) < 0.001 and abs(y - by) < 0.001:
                return i + 1
        return 0

    # 导航到物块
    def navigate_to_block(self):
        self._clean_timers()

        if not self.current_task:
            return

        self.grasp_confirmed = False
        self.current_grasp_retry = 0
        self.closest_cache = None

        # 获取目标物块
        closest = self.get_closest_block()
        if not closest:
            self.get_logger().warn("未找到可抓取的物块，1秒后重试")
            self.retry_timer = self.create_timer(1.0, self.navigate_to_block)
            return

        x, y, cube_name, idx = closest
        self.selected_block = (x, y, cube_name, idx)

        # 计算抓取偏移位置
        tx = x - self.GRASP_OFFSET if self.OFFSET_AXIS == "x" else x
        ty = y if self.OFFSET_AXIS == "x" else y - self.GRASP_OFFSET

        # 发布导航目标
        nav_msg = String()
        nav_msg.data = json.dumps({"type": "custom", "x": tx, "y": ty, "yaw": 0.0})
        self.nav_target_pub.publish(nav_msg)
        self.target_cube_pub.publish(String(data=cube_name))
        self.current_step = "NAV_TO_BLOCK"
        self._update_foxglove()
        self.get_logger().info(f"导航到物块：{cube_name}（目标位置：{tx:.2f}, {ty:.2f}）")

    # 触发抓取
    def trigger_grasp(self):
        if self.current_grasp_retry >= self.max_grasp_retry:
            self.get_logger().error(f"抓取重试达{self.max_grasp_retry}次，跳过该物块")
            self.current_grasp_retry = 0
            self.navigate_to_block()
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
            idx = self.selected_block[3]
            x, y, _ = self.target_blocks[idx]
            self.target_blocks[idx] = (x, y, True)
            self.current_step = "NAV_TO_AREA"
            self.closest_cache = None
            self.navigate_to_area()
        else:
            # 重试抓取
            self.current_grasp_retry += 1
            self.get_logger().warn(f"抓取超时/失败，准备重试（{self.current_grasp_retry}/{self.max_grasp_retry}）")
            self.trigger_grasp()
        
        gc.collect()

    # 导航到目标区域
    def navigate_to_area(self):
        if not self.current_task:
            return

        area = self.current_task["to"]
        if area not in self.AREA_COORDS:
            self.get_logger().error(f"无效目标区域：{area}")
            return
        
        x, y = self.AREA_COORDS[area]
        nav_msg = String()
        nav_msg.data = json.dumps({"type": "custom", "x": x, "y": y, "yaw": 0.0})
        self.nav_target_pub.publish(nav_msg)
        self.current_step = "NAV_TO_AREA"
        self._update_foxglove()
        self.get_logger().info(f"导航到目标区域 {area}：({x:.2f}, {y:.2f})")

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
            total = self.current_task["num"]
            self.get_logger().info(f"放置完成（{self.completed_num}/{total}）")
            
            if self.completed_num < total:
                # 当前任务未完成，继续抓取下一个
                self.current_step = "NAV_TO_BLOCK"
                self.navigate_to_block()
            else:
                # 当前任务完成，处理下一个任务
                self.get_logger().info("当前任务全部完成！")
                self._process_next_task()
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
        self.target_blocks = []
        self.current_task = None
        self.task_queue = []
        self.selected_block = None
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
