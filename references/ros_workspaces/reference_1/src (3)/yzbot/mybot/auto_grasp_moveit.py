#!/usr/bin/env python3
import rclpy
import time
import threading
from collections import deque
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String, Int8
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from linkattacher_msgs.srv import AttachLink, DetachLink


class AutoGraspMoveIt(Node):
    def __init__(self):
        super().__init__("auto_grasp_moveit_node")
        self.get_logger().info("AutoGraspMoveIt 启动中...")

        # 机械臂关节与位姿参数
        self.arm_joints = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.arm_pose_init = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.arm_pose_bend = [0.0, 1.15, 1.15, 0.0, -0.3, 0.0]
        
        # 吸附相关参数
        self.robot_name = "six_arm"
        self.robot_tip_link = "link6"
        self.object_link = "link"
        self.last_attached_id = "red_cube_3"
        self.last_attached_stamp = 0.0

        # 机械臂状态枚举
        self.ARM_STATE_IDLE = 0       # 空闲
        self.ARM_STATE_GRASPING = 1   # 正在抓取
        self.ARM_STATE_GRASP_DONE = 2 # 抓取完成
        self.ARM_STATE_PLACING = 3    # 正在放置

        # 核心：状态管理（线程锁+当前状态）
        self.lock = threading.Lock()
        self.current_arm_state = self.ARM_STATE_IDLE
        self.state_map = {0: "空闲", 1: "正在抓取", 2: "抓取完成", 3: "正在放置"}

        # QoS配置（适配rosbridge/Foxglove）
        qos_foxglove = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        qos_sync = QoSProfile(depth=10)
        qos_sync.reliability = QoSReliabilityPolicy.RELIABLE
        qos_sync.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # 回调组
        self.cb_moveit = ReentrantCallbackGroup()
        self.cb_nav = ReentrantCallbackGroup()

        # MoveIt Action客户端
        self.moveit_client = ActionClient(self, MoveGroup, "/move_action", callback_group=self.cb_moveit)
        self.get_logger().info("等待 MoveIt 服务器 ...")
        self.moveit_client.wait_for_server()
        self.get_logger().info("MoveIt 服务器已连接")

        # LinkAttacher服务客户端
        self.attach_cli = self.create_client(AttachLink, "/ATTACHLINK")
        self.detach_cli = self.create_client(DetachLink, "/DETACHLINK")
        self.attach_cli.wait_for_service()
        self.detach_cli.wait_for_service()
        self.get_logger().info("LinkAttacher 服务已连接")

        # 话题订阅/发布（原有逻辑不变）
        self.sub_nav2arm = self.create_subscription(
            String, "/task_sync_nav2arm", self.sync_callback, qos_sync, callback_group=self.cb_nav)
        self.pub_arm2nav = self.create_publisher(String, "/task_sync_arm2nav", qos_sync)
        self.sub_target_id = self.create_subscription(
            String, "/pick_target_id", self.target_id_callback, qos_sync, callback_group=self.cb_nav)

        # 核心修改：发布std_msgs/String类型（数值转字符串）
        self.arm_state_pub = self.create_publisher(String, "/arm_state", qos_foxglove)
        self.state_pub_timer = self.create_timer(0.1, self._publish_arm_state)  # 10Hz发布

        # 指令队列与工作线程
        self._cmd_queue = deque(maxlen=1)
        self._stop_flag = False
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        # 初始状态发布
        self.get_logger().info(f"机械臂初始状态：{self.state_map[self.current_arm_state]}({self.current_arm_state})")

    def _publish_arm_state(self):
        """发布字符串类型的状态（数值转字符串，如"0"/"1"）"""
        with self.lock:
            msg = String()
            msg.data = str(self.current_arm_state)  # 核心：数值转字符串
            self.arm_state_pub.publish(msg)
            self.get_logger().info(f"[发布状态] {self.state_map[self.current_arm_state]}({msg.data})")

    def target_id_callback(self, msg: String):
        target = msg.data.strip()
        if target:
            with self.lock:
                self.last_attached_id = target
                self.last_attached_stamp = time.time()
            self.get_logger().info(f"已接收目标ID：{self.last_attached_id}")

    def sync_callback(self, msg: String):
        cmd = msg.data.strip()
        if cmd in ("nav_done_grasp", "nav_done_place"):
            self._cmd_queue.clear()
            self._cmd_queue.append(cmd)
            self.get_logger().info(f"收到指令：{cmd}")

    def _worker_loop(self):
        while rclpy.ok() and not self._stop_flag:
            if self._cmd_queue:
                cmd = self._cmd_queue.popleft()
                try:
                    if cmd == "nav_done_grasp":
                        self.execute_grasp()
                    elif cmd == "nav_done_place":
                        self.execute_place()
                except Exception as e:
                    self.get_logger().error(f"执行指令{cmd}出错：{str(e)}")
                    with self.lock:
                        self.current_arm_state = self.ARM_STATE_IDLE
                        self.get_logger().error(f"指令执行失败，状态切回：{self.state_map[self.current_arm_state]}({self.current_arm_state})")
            time.sleep(0.05)

    def execute_grasp(self):
        with self.lock:
            self.current_arm_state = self.ARM_STATE_GRASPING
        self.get_logger().info(f"=== 抓取流程启动 ===")
        self.get_logger().info(f"状态更新：{self.state_map[self.current_arm_state]}({self.current_arm_state})")

        grasp_ok = self._send_moveit_joint_goal(self.arm_pose_bend)
        if not grasp_ok:
            self.get_logger().error("抓取位姿移动失败")
            with self.lock:
                self.current_arm_state = self.ARM_STATE_IDLE
            self.get_logger().error(f"状态切回：{self.state_map[self.current_arm_state]}({self.current_arm_state})")
            return

        with self.lock:
            if time.time() - self.last_attached_stamp > 10.0:
                self.get_logger().warning(f"目标ID超时，使用默认：{self.last_attached_id}")

        time.sleep(0.5)
        self._attach_object(self.last_attached_id)

        back_ok = self._send_moveit_joint_goal(self.arm_pose_init)
        if not back_ok:
            self.get_logger().error("抓取后回初始位姿失败")
            with self.lock:
                self.current_arm_state = self.ARM_STATE_IDLE
            self.get_logger().error(f"状态切回：{self.state_map[self.current_arm_state]}({self.current_arm_state})")
            return

        with self.lock:
            self.current_arm_state = self.ARM_STATE_GRASP_DONE
        self.get_logger().info(f"=== 抓取流程完成 ===")
        self.get_logger().info(f"状态更新：{self.state_map[self.current_arm_state]}({self.current_arm_state})")
        self.pub_arm2nav.publish(String(data="arm_done"))

    def execute_place(self):
        with self.lock:
            current_state = self.current_arm_state
            if current_state != self.ARM_STATE_GRASP_DONE:
                self.get_logger().error(f"放置指令无效！当前状态不是抓取完成，而是：{self.state_map[current_state]}({current_state})")
                return
            self.current_arm_state = self.ARM_STATE_PLACING
        self.get_logger().info(f"=== 放置流程启动 ===")
        self.get_logger().info(f"状态更新：{self.state_map[self.current_arm_state]}({self.current_arm_state})")

        place_ok = self._send_moveit_joint_goal(self.arm_pose_bend)
        if not place_ok:
            self.get_logger().error("放置位姿移动失败")
            with self.lock:
                self.current_arm_state = self.ARM_STATE_GRASP_DONE
            self.get_logger().error(f"状态切回：{self.state_map[self.current_arm_state]}({self.current_arm_state})")
            return

        time.sleep(0.5)
        self._detach_object(self.last_attached_id)

        back_ok = self._send_moveit_joint_goal(self.arm_pose_init)
        if not back_ok:
            self.get_logger().error("放置后回初始位姿失败")
            with self.lock:
                self.current_arm_state = self.ARM_STATE_GRASP_DONE
            self.get_logger().error(f"状态切回：{self.state_map[self.current_arm_state]}({self.current_arm_state})")
            return

        with self.lock:
            self.current_arm_state = self.ARM_STATE_IDLE
        self.get_logger().info(f"=== 放置流程完成 ===")
        self.get_logger().info(f"状态更新：{self.state_map[self.current_arm_state]}({self.current_arm_state})")
        self.pub_arm2nav.publish(String(data="arm_done"))

    def _send_moveit_joint_goal(self, joint_values, timeout_sec=20.0):
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.planner_id = "RRTConnectkConfigDefault"
        goal.request.max_velocity_scaling_factor = 0.6
        goal.request.max_acceleration_scaling_factor = 0.6
        goal.request.allowed_planning_time = 5.0
        goal.request.workspace_parameters.header.frame_id = "base_link"

        constraints = Constraints()
        for name, pos in zip(self.arm_joints, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints = [constraints]

        send_future = self.moveit_client.send_goal_async(goal)
        t0 = time.time()
        while not send_future.done() and (time.time() - t0) < 3.0:
            time.sleep(0.02)

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("MoveIt规划请求未被接受")
            return False

        result_future = goal_handle.get_result_async()
        t1 = time.time()
        while not result_future.done() and (time.time() - t1) < timeout_sec:
            time.sleep(0.02)

        if not result_future.done():
            self.get_logger().error("MoveIt执行超时")
            return False

        result = result_future.result()
        if result and result.result.error_code.val == MoveItErrorCodes.SUCCESS:
            return True
        else:
            self.get_logger().error("MoveIt执行失败")
            return False

    def _attach_object(self, obj_id, timeout=5.0):
        req = AttachLink.Request()
        req.model1_name = self.robot_name
        req.link1_name = self.robot_tip_link
        req.model2_name = obj_id
        req.link2_name = self.object_link
        fut = self.attach_cli.call_async(req)
        t0 = time.time()
        while not fut.done() and (time.time() - t0) < timeout:
            time.sleep(0.02)
        if fut.done():
            self.get_logger().info(f"吸附物体成功：{obj_id}")
        else:
            self.get_logger().error(f"吸附物体超时：{obj_id}")

    def _detach_object(self, obj_id, timeout=5.0):
        req = DetachLink.Request()
        req.model1_name = self.robot_name
        req.link1_name = self.robot_tip_link
        req.model2_name = obj_id
        req.link2_name = self.object_link
        fut = self.detach_cli.call_async(req)
        t0 = time.time()
        while not fut.done() and (time.time() - t0) < timeout:
            time.sleep(0.02)
        if fut.done():
            self.get_logger().info(f"分离物体成功：{obj_id}")
        else:
            self.get_logger().error(f"分离物体超时：{obj_id}")

    def destroy_node(self):
        self._stop_flag = True
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutoGraspMoveIt()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("用户中断，停止节点")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

