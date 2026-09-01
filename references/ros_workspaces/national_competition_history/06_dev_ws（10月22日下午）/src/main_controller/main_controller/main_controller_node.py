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
        # ===================== QoS配置（核心优化：防止消息堆积）=====================
        self.qos_best_effort = QoSProfile(
            depth=5,  # 队列大小5（默认10，减少缓存）
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # 不缓存过期消息
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # ===================== 物块与目标区配置 =====================
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

        # ===================== 核心变量 =====================
        self.current_robot_pose = (0.0, 0.0)
        self.target_blocks = []
        self.GRASP_OFFSET = 0.4
        self.OFFSET_AXIS = "x"
        self.retry_timer: Timer = None
        self.grasp_timer: Timer = None
        self.place_timer: Timer = None
        self.current_task = None
        self.completed_num = 0
        self.current_step = "WAIT_TASK"
        self.selected_block = None
        self.grasp_confirmed = False
        self.place_confirmed = False
        self.max_grasp_retry = 3
        self.current_grasp_retry = 0
        self.closest_cache = None
        self.cache_expire = 2.0
        self.last_cache_time = 0.0
        
        # 超时时间调整（增加到15秒，确保机械臂有足够时间完成操作）
        self.GRASP_TIMEOUT = 15.0
        self.PLACE_TIMEOUT = 15.0

        # ===================== 订阅与发布器（应用QoS）=====================
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_callback, self.qos_best_effort
        )
        self.chat_sub = self.create_subscription(
            String, "/chat", self._chat_callback, self.qos_best_effort
        )
        self.nav_status_sub = self.create_subscription(
            String, "/nav_status", self._nav_status_callback, self.qos_best_effort
        )
        self.arm_status_sub = self.create_subscription(
            String, "/arm_status", self._arm_status_callback, self.qos_best_effort
        )

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

        self.get_logger().info("主控节点（内存优化2版）启动")

    # ===================== 回调函数（弱引用避免循环引用）=====================
    @staticmethod
    def _amcl_callback_impl(self_ref, msg):
        self = self_ref()
        if not self:
            return
        new_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if abs(new_pose[0] - self.current_robot_pose[0]) > 0.01 or \
           abs(new_pose[1] - self.current_robot_pose[1]) > 0.01:
            if self.current_robot_pose == (0.0, 0.0):
                self.get_logger().info(f"初始定位更新：({new_pose[0]:.2f}, {new_pose[1]:.2f})")
            self.current_robot_pose = new_pose
            self.closest_cache = None

    def _amcl_callback(self, msg):
        self_ref = weakref.ref(self)
        self._amcl_callback_impl(self_ref, msg)
        # 处理后立即清理引用
        del self_ref

    @staticmethod
    def _chat_callback_impl(self_ref, msg):
        self = self_ref()
        if not self:
            return
        try:
            task_json = json.loads(msg.data)
            if isinstance(task_json, list):
                task_json = task_json[0]
            required = ["color", "num", "to"]
            if not all(k in task_json for k in required):
                self.get_logger().error("任务格式错误（需color/num/to）")
                return

            self.current_task = task_json
            self.completed_num = 0
            self.current_step = "NAV_TO_BLOCK"
            color = task_json["color"].lower()
            self.target_blocks = self.RED_BLOCKS if color == "red" else self.BLUE_BLOCKS
            for i in range(len(self.target_blocks)):
                x, y, _ = self.target_blocks[i]
                self.target_blocks[i] = (x, y, False)

            self.grasp_confirmed = False
            self.place_confirmed = False
            self.current_grasp_retry = 0
            self.closest_cache = None
            self._clean_timers()

            self.get_logger().info(f"接收任务：{task_json['num']}个{color}物块到{task_json['to']}区")
            self._update_foxglove()
            self.navigate_to_block()

        except json.JSONDecodeError:
            self.get_logger().error("任务解析失败（JSON格式错误）")

    def _chat_callback(self, msg):
        self_ref = weakref.ref(self)
        self._chat_callback_impl(self_ref, msg)
        # 处理后立即清理引用
        del self_ref

    @staticmethod
    def _nav_status_callback_impl(self_ref, msg):
        self = self_ref()
        if not self:
            return
        status = msg.data.strip()
        if status == "succeeded":
            if self.current_step == "NAV_TO_BLOCK":
                self.get_logger().info("到达物块位置，触发抓取")
                self.current_step = "GRASP"
                self.trigger_grasp()
            elif self.current_step == "NAV_TO_AREA":
                self.get_logger().info("到达目标区，触发放置")
                self.current_step = "PLACE"
                self.trigger_place()
        elif status == "failed":
            self.get_logger().warn("导航失败，重试")
            if self.current_step == "NAV_TO_BLOCK":
                self.navigate_to_block()
            else:
                self.navigate_to_area()

    def _nav_status_callback(self, msg):
        self_ref = weakref.ref(self)
        self._nav_status_callback_impl(self_ref, msg)
        # 处理后立即清理引用
        del self_ref

    @staticmethod
    def _arm_status_callback_impl(self_ref, msg):
        self = self_ref()
        if not self:
            return
        status = msg.data.strip()
        if status == "grasp_succeeded":
            self.grasp_confirmed = True
        elif status == "place_succeeded":
            self.place_confirmed = True

    def _arm_status_callback(self, msg):
        self_ref = weakref.ref(self)
        self._arm_status_callback_impl(self_ref, msg)
        # 处理后立即清理引用
        del self_ref

    # ===================== 核心逻辑 =====================
    def get_closest_block(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if (current_time - self.last_cache_time) < self.cache_expire and self.closest_cache:
            block_x, block_y, block_idx = self.closest_cache
            color = self.current_task["color"].lower()
            cube_id = self._get_cube_id(block_x, block_y, color)
            return (block_x, block_y, f"{color}_cube_{cube_id}", block_idx)

        if not self.target_blocks:
            return None

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

        self.closest_cache = closest
        self.last_cache_time = current_time
        x, y, idx = closest
        color = self.current_task["color"].lower()
        cube_id = self._get_cube_id(x, y, color)
        result = (x, y, f"{color}_cube_{cube_id}", idx)
        # 清理临时变量
        del x, y, idx, color, cube_id
        return result

    def _get_cube_id(self, x, y, color):
        blocks = self.RED_BLOCKS if color == "red" else self.BLUE_BLOCKS
        for i, (bx, by, _) in enumerate(blocks):
            if abs(x - bx) < 0.001 and abs(y - by) < 0.001:
                return i + 1
        return 0

    def navigate_to_block(self):
        self._clean_timers()

        if not self.current_task:
            return

        self.grasp_confirmed = False
        self.current_grasp_retry = 0
        self.closest_cache = None

        closest = self.get_closest_block()
        if not closest:
            self.retry_timer = self.create_timer(1.0, self.navigate_to_block)
            return

        x, y, cube_name, idx = closest
        self.selected_block = (x, y, cube_name, idx)

        tx = x - self.GRASP_OFFSET if self.OFFSET_AXIS == "x" else x
        ty = y if self.OFFSET_AXIS == "x" else y - self.GRASP_OFFSET

        nav_msg = String()
        nav_msg.data = json.dumps({"type": "custom", "x": tx, "y": ty, "yaw": 0.0})
        self.nav_target_pub.publish(nav_msg)
        self.target_cube_pub.publish(String(data=cube_name))
        self.current_step = "NAV_TO_BLOCK"
        self._update_foxglove()
        
        # 清理临时变量
        del x, y, cube_name, idx, tx, ty, nav_msg

    def trigger_grasp(self):
        if self.current_grasp_retry >= self.max_grasp_retry:
            self.get_logger().error(f"抓取重试达{self.max_grasp_retry}次，跳过")
            self.current_grasp_retry = 0
            self.navigate_to_block()
            return

        self.arm_cargo_pub.publish(String(data="arrived_at_cargo"))
        self_ref = weakref.ref(self)
        # 使用延长的超时时间
        self.grasp_timer = self.create_timer(self.GRASP_TIMEOUT, lambda: self._check_grasp(self_ref))

    @staticmethod
    def _check_grasp(self_ref):
        self = self_ref()
        if not self:
            return
        # 确保定时器被取消和清理
        if self.grasp_timer:
            self.grasp_timer.cancel()
            self.grasp_timer.destroy()
            self.grasp_timer = None

        if self.grasp_confirmed:
            self.get_logger().info(f"抓取成功：{self.selected_block[2]}")
            idx = self.selected_block[3]
            x, y, _ = self.target_blocks[idx]
            self.target_blocks[idx] = (x, y, True)
            self.current_step = "NAV_TO_AREA"
            self.closest_cache = None
            self.navigate_to_area()
        else:
            self.current_grasp_retry += 1
            self.get_logger().warn(f"抓取重试({self.current_grasp_retry}/{self.max_grasp_retry})")
            self.trigger_grasp()
        
        # 强制垃圾回收
        gc.collect()

    def navigate_to_area(self):
        area = self.current_task["to"].upper()
        if area not in self.AREA_COORDS:
            self.get_logger().error("无效目标区")
            return
        x, y = self.AREA_COORDS[area]
        nav_msg = String()
        nav_msg.data = json.dumps({"type": "custom", "x": x, "y": y, "yaw": 0.0})
        self.nav_target_pub.publish(nav_msg)
        self.current_step = "NAV_TO_AREA"
        self._update_foxglove()
        
        # 清理临时变量
        del x, y, nav_msg, area

    def trigger_place(self):
        self.place_confirmed = False
        self.arm_area_pub.publish(String(data="arrived_at_area"))
        self_ref = weakref.ref(self)
        # 使用延长的超时时间
        self.place_timer = self.create_timer(self.PLACE_TIMEOUT, lambda: self._check_place(self_ref))

    @staticmethod
    def _check_place(self_ref):
        self = self_ref()
        if not self:
            return
        # 确保定时器被取消和清理
        if self.place_timer:
            self.place_timer.cancel()
            self.place_timer.destroy()
            self.place_timer = None

        if self.place_confirmed:
            self.completed_num += 1
            total = self.current_task["num"]
            self.get_logger().info(f"放置完成({self.completed_num}/{total})")
            if self.completed_num < total:
                self.current_step = "NAV_TO_BLOCK"
                self.navigate_to_block()
            else:
                self.get_logger().info("任务完成！")
                self.current_task = None
                self.target_blocks = []
                self.selected_block = None
                self.current_step = "WAIT_TASK"
                self._update_foxglove()
                gc.collect()  # 任务完成后强制GC
        else:
            self.get_logger().warn("放置重试")
            self.trigger_place()
        
        # 强制垃圾回收
        gc.collect()

    # ===================== 工具函数（内存管理）=====================
    def _clean_timers(self):
        """清理定时器并触发GC"""
        for timer in [self.retry_timer, self.grasp_timer, self.place_timer]:
            if timer:
                timer.cancel()
                timer.destroy()  # 彻底销毁定时器
        self.retry_timer = None
        self.grasp_timer = None
        self.place_timer = None
        gc.collect()  # 清理后强制GC

    def _update_foxglove(self):
        if not self.current_task:
            for pub in self.foxglove_pubs.values():
                pub.publish(Int32(data=-1))
            self.foxglove_pubs["cur"].publish(Int32(data=0))
            return

        color = self.current_task["color"].lower()
        color_data = 0 if color == "blue" else 1
        self.foxglove_pubs["color"].publish(Int32(data=color_data))
        self.foxglove_pubs["ask"].publish(Int32(data=color_data))

        pick_data = -1 if self.current_step in ["NAV_TO_BLOCK", "WAIT_TASK"] else \
                    0 if self.current_step in ["GRASP", "NAV_TO_AREA"] else 1
        self.foxglove_pubs["pick"].publish(Int32(data=pick_data))

        step_map = {"WAIT_TASK":0, "NAV_TO_BLOCK":1, "GRASP":2, "NAV_TO_AREA":3, "PLACE":3}
        self.foxglove_pubs["cur"].publish(Int32(data=step_map[self.current_step]))
        
        # 清理临时变量
        del color, color_data, pick_data

    def destroy_node(self):
        self._clean_timers()
        self.target_blocks = []
        self.current_task = None
        self.selected_block = None
        # 清理所有发布者和订阅者引用
        self.amcl_pose_sub = None
        self.chat_sub = None
        self.nav_status_sub = None
        self.arm_status_sub = None
        self.target_cube_pub = None
        self.nav_target_pub = None
        self.arm_cargo_pub = None
        self.arm_area_pub = None
        self.foxglove_pubs = None
        super().destroy_node()
        gc.collect()


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=2)
    node = MainController()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("节点手动终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
