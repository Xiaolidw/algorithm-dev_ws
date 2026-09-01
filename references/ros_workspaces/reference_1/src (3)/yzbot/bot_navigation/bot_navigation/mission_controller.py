#!/usr/bin/env python3
import rclpy
import math
import time
import json
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from mybot.msg import ObjectPoseArray


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # ========= 参数 =========
        self.nav_offset = 0.40
        self.nav_timeout = 120.0
        self.nav_spin_dt = 0.30
        self.wait_arm_timeout = 20.0

        self.current_pose = None
        self.blocks_cache = {'red': [], 'blue': []}
        self.used_ids = set()
        self.tasks = []
        self.navigator = None

        self.zone_pose_map = {}
        self.robot_spawn_x = None
        self.robot_spawn_y = None
        self.robot_spawn_z = None
        self.initial_pose_published = False
        self.arm_done_flag = False

        # ========= QoS =========
        q_sync = QoSProfile(depth=10)
        q_sync.reliability = QoSReliabilityPolicy.RELIABLE
        q_sync.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.cb_sync = ReentrantCallbackGroup()

        # ========= 通信 =========
        self.pub_nav2arm = self.create_publisher(String, '/task_sync_nav2arm', q_sync)
        self.sub_arm2nav = self.create_subscription(
            String, '/task_sync_arm2nav', self.arm_sync_callback, q_sync,
            callback_group=self.cb_sync
        )
        self.pub_pick_id = self.create_publisher(String, '/pick_target_id', q_sync)

        self.sub_obj = self.create_subscription(
            ObjectPoseArray, '/object_detection/object_poses', self.obj_callback, 10)
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 50)
        self.sub_task = self.create_subscription(
            String, 'robot_task', self.task_callback, 10)

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # ========= 状态话题（用于 Foxglove 可视化）=========
        self.mission_pub = self.create_publisher(String, '/mission_state', 10)
        self.nav_status_pub = self.create_publisher(String, '/nav_status', 10)
        self.target_name_pub = self.create_publisher(String, '/nav_target_name', 10)
        self.arm_status_pub = self.create_publisher(String, '/arm_status', 10)

        # --- 新增：导航阶段 / 任务区 / 包裹颜色 状态枚举（数值以字符串形式发布） ---
        # nav_state：0-3
        #   0：前往仓库（去方块前方）
        #   1：到达仓库（到达方块前方，等待/抓取）
        #   2：前往包裹目标点（去目标区 A/B/C）
        #   3：到达包裹目标地区（到达目标区，等待/放置）
        self.current_nav_state = 0
        self.nav_state_pub = self.create_publisher(String, '/nav_state', 10)

        # task_zone_state：0-3
        #   0：无任务区
        #   1：当前目标区 A
        #   2：当前目标区 B
        #   3：当前目标区 C
        self.current_zone_state = 0
        self.zone_state_pub = self.create_publisher(String, '/task_zone_state', 10)

        # parcel_color_state：0-2
        #   0：无包裹
        #   1：当前任务包裹为红色
        #   2：当前任务包裹为蓝色
        self.current_color_state = 0
        self.color_state_pub = self.create_publisher(String, '/parcel_color_state', 10)

        # 10Hz 定时发布三种状态（适配 Foxglove）
        self.state_pub_timer = self.create_timer(0.1, self._publish_states)

        # ========= 定时器与线程 =========
        self.timer_nav2 = self.create_timer(1.0, self.try_init_nav2)
        self.timer_initpose = self.create_timer(0.5, self.try_publish_initialpose)
        self._runner_thread = None
        self._runner_lock = threading.Lock()

        self.publish_status(self.mission_pub, "启动中：等待 Nav2 激活与 Gazebo 坐标")
        self.get_logger().info("启动中：等待 Nav2 激活与 Gazebo 坐标")

    # ========== 回调 ==========

    def arm_sync_callback(self, msg: String):
        data = msg.data.strip()
        if data == 'arm_done':
            self.arm_done_flag = True
            self.publish_status(self.arm_status_pub, "机械臂完成动作")

    def obj_callback(self, msg: ObjectPoseArray):
        self.blocks_cache['red'] = [o for o in msg.objects if o.type == 'block_red' and o.id not in self.used_ids]
        self.blocks_cache['blue'] = [o for o in msg.objects if o.type == 'block_blue' and o.id not in self.used_ids]
        for o in msg.objects:
            if o.type == "zone":
                name = o.id.lower()
                if "zone_a" in name:
                    self.zone_pose_map["A"] = (o.pose.position.x, o.pose.position.y)
                elif "zone_b" in name:
                    self.zone_pose_map["B"] = (o.pose.position.x, o.pose.position.y)
                elif "zone_c" in name:
                    self.zone_pose_map["C"] = (o.pose.position.x, o.pose.position.y)
            if o.id == "six_arm":
                self.robot_spawn_x = o.pose.position.x
                self.robot_spawn_y = o.pose.position.y
                self.robot_spawn_z = o.pose.position.z

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg

    # 改进后的 Nav2 初始化逻辑（等待 AMCL 首帧后再建 BasicNavigator）
    def try_init_nav2(self):
        # 已初始化则跳过
        if self.navigator:
            return

        # 必须先发布过初始位姿
        if not self.initial_pose_published:
            return

        # 必须先收到 AMCL 的第一帧
        if self.current_pose is None:
            return

        try:
            # 等一秒保证 Nav2 各模块完全就绪
            time.sleep(1.0)
            self.navigator = BasicNavigator()
            self.navigator.waitUntilNav2Active()
            self.publish_status(self.mission_pub, "Nav2 已激活")
            self.timer_nav2.cancel()
        except Exception as e:
            self.get_logger().warn(f"Nav2 激活失败重试中: {e}")

    def try_publish_initialpose(self):
        if self.initial_pose_published or self.robot_spawn_x is None:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(self.robot_spawn_x)
        msg.pose.pose.position.y = float(self.robot_spawn_y)
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self.initial_pose_pub.publish(msg)
        self.initial_pose_published = True
        self.publish_status(self.mission_pub, f"已发布初始位姿 ({self.robot_spawn_x:.3f}, {self.robot_spawn_y:.3f})")
        self.timer_initpose.cancel()

    # ========== 任务处理 ==========

    def task_callback(self, msg: String):
        try:
            lst = json.loads(msg.data)
            if not isinstance(lst, list):
                return
            self.tasks = lst
            self.publish_status(self.mission_pub, f"接收到任务列表，共 {len(lst)} 个")
            if self._runner_thread is None or not self._runner_thread.is_alive():
                with self._runner_lock:
                    if self._runner_thread is None or not self._runner_thread.is_alive():
                        self._runner_thread = threading.Thread(target=self.execute_all_tasks, daemon=True)
                        self._runner_thread.start()
        except Exception as e:
            self.get_logger().error(f"任务解析失败: {e}")

    def execute_all_tasks(self):
        if not self.navigator:
            self.publish_status(self.mission_pub, "Nav2 未准备好，任务挂起")
            return

        # 任务开始前，状态清零
        self.set_zone_state(None)
        self.set_color_state(None)
        self.current_nav_state = 0

        while self.tasks:
            task = self.tasks.pop(0)
            color = str(task['color']).lower()
            num = int(task['num'])
            zone = str(task['to']).upper()
            self.publish_status(self.mission_pub, f"执行任务：抓取 {num} 个 {color} 方块，放置到 {zone} 区")
            self.execute_single_task(color, num, zone)

        # 所有任务结束，状态归 0
        self.set_zone_state(None)
        self.set_color_state(None)
        self.current_nav_state = 0
        self.publish_status(self.mission_pub, "所有任务执行完毕")

    def execute_single_task(self, color: str, num: int, zone: str):
        # 当前任务的目标区 & 包裹颜色状态
        self.set_zone_state(zone)
        self.set_color_state(color)

        for i in range(num):
            self.publish_status(self.mission_pub, f"开始抓取第 {i+1}/{num} 个 {color} 方块")
            target = self.choose_target_block(color)
            if not target:
                self.publish_status(self.mission_pub, "未找到目标方块，跳过")
                continue

            # 导航到仓库（方块前方）
            self.publish_status(self.nav_status_pub, f"导航到 {target.id}")
            self.target_name_pub.publish(String(data=target.id))
            if not self.nav_to_block_front(target):
                self.publish_status(self.nav_status_pub, "导航失败（前往方块）")
                continue

            # 抓取阶段
            self.arm_done_flag = False
            self.pub_pick_id.publish(String(data=target.id))
            self.pub_nav2arm.publish(String(data='nav_done_grasp'))
            self.publish_status(self.arm_status_pub, "等待机械臂抓取完成")
            self.wait_for_arm_done(self.wait_arm_timeout)

            # 放置阶段：前往 A/B/C 目标区
            if not self.nav_to_zone(zone):
                self.publish_status(self.nav_status_pub, "导航失败（前往放置区）")
                continue
            self.arm_done_flag = False
            self.pub_nav2arm.publish(String(data='nav_done_place'))
            self.publish_status(self.arm_status_pub, "等待机械臂放置完成")
            self.wait_for_arm_done(self.wait_arm_timeout)

            # 当前这个包裹搬完
            self.used_ids.add(target.id)
            for k in self.blocks_cache:
                self.blocks_cache[k] = [o for o in self.blocks_cache[k] if o.id != target.id]
            self.publish_status(self.mission_pub, f"完成搬运 {target.id}")

        self.publish_status(self.mission_pub, f"{color} 方块任务完成")
        # 这一轮任务结束，如果后面还有其它任务，execute_all_tasks 会再次设置 zone/color
        self.set_color_state(None)

    # ========== 导航逻辑 ==========

    def wait_for_arm_done(self, timeout_sec: float):
        t0 = time.time()
        while (time.time() - t0) < timeout_sec and not self.arm_done_flag:
            time.sleep(0.05)
        if not self.arm_done_flag:
            self.publish_status(self.arm_status_pub, "等待机械臂超时")

    def nav_to_block_front(self, target):
        # 0：前往仓库（方块前方）
        self.current_nav_state = 0

        tx, ty = target.pose.position.x, target.pose.position.y
        gx, gy = tx - self.nav_offset, ty
        self.publish_status(self.nav_status_pub, f"导航到方块前方 ({gx:.2f}, {gy:.2f})")
        ok = self.nav_to_pose(gx, gy)

        if ok:
            # 1：到达仓库（到达方块前方）
            self.current_nav_state = 1
        return ok

    def nav_to_zone(self, zone):
        if zone not in self.zone_pose_map:
            self.publish_status(self.mission_pub, f"未定义放置区：{zone}")
            return False

        # 2：前往包裹目标点（A/B/C 区）
        self.current_nav_state = 2

        zx, zy = self.zone_pose_map[zone]
        self.publish_status(self.nav_status_pub, f"导航到放置区 {zone} ({zx:.2f}, {zy:.2f})")
        ok = self.nav_to_pose(zx, zy)

        if ok:
            # 3：到达包裹目标地区
            self.current_nav_state = 3
        return ok

    def nav_to_pose(self, x, y):
        goal = self.make_goal_pose(x, y)
        self.navigator.goToPose(goal)
        t0 = time.time()
        while not self.navigator.isTaskComplete():
            fb = self.navigator.getFeedback()
            if fb and (time.time() - t0) % 2 < 0.3:
                self.publish_status(self.nav_status_pub, f"距离目标 {fb.distance_remaining:.2f} 米")
            if (time.time() - t0) > self.nav_timeout:
                self.navigator.cancelTask()
                self.publish_status(self.nav_status_pub, "导航超时")
                return False
            time.sleep(self.nav_spin_dt)
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.publish_status(self.nav_status_pub, "导航成功")
            return True
        else:
            self.publish_status(self.nav_status_pub, "导航失败")
            return False

    def choose_target_block(self, color):
        arr = [o for o in self.blocks_cache.get(color, []) if o.id not in self.used_ids]
        if not arr:
            return None
        if not self.current_pose:
            return arr[0]
        rx = self.current_pose.pose.pose.position.x
        ry = self.current_pose.pose.pose.position.y
        return min(arr, key=lambda o: math.hypot(o.pose.position.x - rx, o.pose.position.y - ry))

    def make_goal_pose(self, x, y, yaw=0.0):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        return goal

    # ========== 状态发布 & 状态设置 ==========

    def _publish_states(self):
        """10Hz 周期发布：/nav_state, /task_zone_state, /parcel_color_state"""
        # 导航阶段
        nav_msg = String()
        nav_msg.data = str(self.current_nav_state)
        self.nav_state_pub.publish(nav_msg)

        # 任务区状态
        zone_msg = String()
        zone_msg.data = str(self.current_zone_state)
        self.zone_state_pub.publish(zone_msg)

        # 包裹颜色状态
        color_msg = String()
        color_msg.data = str(self.current_color_state)
        self.color_state_pub.publish(color_msg)

    def set_zone_state(self, zone: str):
        """zone: 'A'/'B'/'C' 或 None/其它"""
        if zone == "A":
            self.current_zone_state = 1
        elif zone == "B":
            self.current_zone_state = 2
        elif zone == "C":
            self.current_zone_state = 3
        else:
            self.current_zone_state = 0

    def set_color_state(self, color: str):
        """color: 'red'/'blue' 或 None/其它"""
        if not color:
            self.current_color_state = 0
            return
        c = color.lower()
        if c == "red":
            self.current_color_state = 1
        elif c == "blue":
            self.current_color_state = 2
        else:
            self.current_color_state = 0

    def publish_status(self, publisher, msg):
        publisher.publish(String(data=msg))
        self.get_logger().info(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    node.get_logger().info("等待系统初始化...")
    time.sleep(1.5)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

