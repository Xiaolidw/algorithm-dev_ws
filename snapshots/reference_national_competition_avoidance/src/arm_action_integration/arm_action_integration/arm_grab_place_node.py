import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from linkattacher_msgs.srv import AttachLink, DetachLink
import time

class ArmGrabPlaceNode(Node):
    def __init__(self):
        super().__init__("arm_grab_place_node")
        
        # ===================== 1. 新增：初始化状态发布器（给主控发确认信号）=====================
        self.arm_status_pub = self.create_publisher(String, "/arm_status", 10)
        self.get_logger().info("已初始化/arm_status发布器，用于发送抓取/放置确认")

        # ===================== 2. 原有动作/服务客户端初始化（保留不变）=====================
        self.arm_action_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.arm_action_client.wait_for_server()
        self.get_logger().info("机械臂动作服务就绪！")

        self.gripper_action_client = ActionClient(
            self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory"
        )
        self.gripper_action_client.wait_for_server()
        self.get_logger().info("夹爪动作服务就绪！")

        self.attach_client = self.create_client(AttachLink, "/ATTACHLINK")
        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/ATTACHLINK 服务未就绪，继续等待...")
        self.get_logger().info("吸附服务就绪！")

        self.detach_client = self.create_client(DetachLink, "/DETACHLINK")
        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/DETACHLINK 服务未就绪，继续等待...")
        self.get_logger().info("分离服务就绪！")

        # ===================== 3. 原有订阅（保留不变）=====================
        self.target_cube_sub = self.create_subscription(
            String, "/current_target_cube", self.target_cube_callback, 10
        )
        self.cargo_sub = self.create_subscription(
            String, "/nav_done_cargo", self.cargo_callback, 10
        )
        self.area_sub = self.create_subscription(
            String, "/nav_done_area", self.area_callback, 10
        )

        # ===================== 4. 原有参数（保留不变）=====================
        self.arm_joints = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.gripper_joint = ["finger_joint1"]
        self.arm_trajectory = {
            "init": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "bend": [0.0, 1.2, 1.17, 0.0, -0.3, 0.0],
            "lift": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        }
        self.gripper_trajectory = {
            "close": [0.0, 0.1, 0.2],
            "open": [0.3, 0.2, 0.1, 0.0]
        }
        self.attach_params = {
            "model1_name": "six_arm",
            "link1_name": "link6",
            "model2_name": "red_cube_1",
            "link2_name": "link"
        }

        # ===================== 5. 状态控制（保留不变）=====================
        self.is_executing = False
        self.current_joint_pos = None
        self.current_step = 0
        self.current_task = None  # "grab" / "place"

    # ===================== 6. 原有目标物块更新（保留不变）=====================
    def target_cube_callback(self, msg):
        new_cube_name = msg.data.strip()
        if "_cube_" in new_cube_name and (new_cube_name.startswith("red_") or new_cube_name.startswith("blue_")):
            self.attach_params["model2_name"] = new_cube_name
            self.get_logger().info(f"已更新目标物块：{new_cube_name}")
        else:
            self.get_logger().warn(f"无效物块名称：{new_cube_name}，保持当前物块：{self.attach_params['model2_name']}")

    # ===================== 7. 原有机械臂/夹爪动作（保留不变）=====================
    def send_arm_action(self, target_joint_positions, step_done_callback):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = self.arm_joints
        point_start = JointTrajectoryPoint()
        point_start.positions = self.current_joint_pos if self.current_joint_pos else self.arm_trajectory["init"]
        point_start.time_from_start.sec = 0
        point_start.time_from_start.nanosec = 500
        point_target = JointTrajectoryPoint()
        point_target.positions = target_joint_positions
        point_target.time_from_start.sec = 1  # 延长动作时间，确保完成
        point_target.time_from_start.nanosec = 0
        trajectory.points = [point_start, point_target]
        goal_msg.trajectory = trajectory
        self.current_joint_pos = target_joint_positions
        future = self.arm_action_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f: self._arm_goal_done_cb(f, step_done_callback)
        )

    def _arm_goal_done_cb(self, future, step_done_callback):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("机械臂动作请求被拒绝！")
                self._reset_execution()
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda r: self._arm_result_done_cb(r, step_done_callback)
            )
        except Exception as e:
            self.get_logger().error(f"机械臂请求回调异常：{str(e)}")
            self._reset_execution()

    def _arm_result_done_cb(self, future, step_done_callback):
        try:
            result = future.result().result
            if result.error_code == 0:
                self.get_logger().info("机械臂动作执行完成！")
                time.sleep(0.5)  # 动作稳定延迟
                step_done_callback()
            else:
                self.get_logger().error(f"机械臂动作失败，错误码：{result.error_code}")
                self._reset_execution()
        except Exception as e:
            self.get_logger().error(f"机械臂结果回调异常：{str(e)}")
            self._reset_execution()

    def send_gripper_action(self, action_type, step_done_callback):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = self.gripper_joint
        # 优化后的时间序列（将抓取时间从4秒减少到2秒）
        time_steps = [0, 1, 2] if action_type == "close" else [0, 1, 2, 3]

        target_positions = self.gripper_trajectory[action_type]
        points = []
        for i, pos in enumerate(target_positions):
            point = JointTrajectoryPoint()
            point.positions = [pos]
            point.time_from_start.sec = time_steps[i]
            point.time_from_start.nanosec = 200
            points.append(point)
        trajectory.points = points
        goal_msg.trajectory = trajectory
        future = self.gripper_action_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f: self._gripper_goal_done_cb(f, step_done_callback)
        )

    def _gripper_goal_done_cb(self, future, step_done_callback):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("夹爪动作请求被拒绝！")
                self._reset_execution()
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda r: self._gripper_result_done_cb(r, step_done_callback)
            )
        except Exception as e:
            self.get_logger().error(f"夹爪请求回调异常：{str(e)}")
            self._reset_execution()

    def _gripper_result_done_cb(self, future, step_done_callback):
        try:
            result = future.result().result
            if result.error_code == 0:
                self.get_logger().info("夹爪动作执行完成！")
                #time.sleep(1.0)
                step_done_callback()
            else:
                self.get_logger().error(f"夹爪动作失败，错误码：{result.error_code}")
                self._reset_execution()
        except Exception as e:
            self.get_logger().error(f"夹爪结果回调异常：{str(e)}")
            self._reset_execution()

    # ===================== 8. 修复：吸附/分离服务结果判断（移除success字段判断）=====================
    def send_attach_action(self, step_done_callback):
        self.get_logger().info(f"执行吸附动作：机械臂 → 物块（{self.attach_params['model2_name']}）")
        req = AttachLink.Request()
        req.model1_name = self.attach_params["model1_name"]
        req.link1_name = self.attach_params["link1_name"]
        req.model2_name = self.attach_params["model2_name"]
        req.link2_name = self.attach_params["link2_name"]
        future = self.attach_client.call_async(req)
        future.add_done_callback(
            lambda f: self._attach_done_cb(f, step_done_callback)
        )

    def _attach_done_cb(self, future, step_done_callback):
        try:
            # 修复：AttachLink服务无success字段，调用成功即视为吸附成功
            future.result()  # 仅判断是否调用成功，不访问success
            self.get_logger().info(f"物块{self.attach_params['model2_name']}吸附成功！")
            time.sleep(0.5)
            step_done_callback()
        except Exception as e:
            self.get_logger().error(f"吸附动作失败：{str(e)}")
            self._reset_execution()

    def send_detach_action(self, step_done_callback):
        self.get_logger().info(f"执行分离动作：机械臂 → 物块（{self.attach_params['model2_name']}）")
        req = DetachLink.Request()
        req.model1_name = self.attach_params["model1_name"]
        req.link1_name = self.attach_params["link1_name"]
        req.model2_name = self.attach_params["model2_name"]
        req.link2_name = self.attach_params["link2_name"]
        future = self.detach_client.call_async(req)
        future.add_done_callback(
            lambda f: self._detach_done_cb(f, step_done_callback)
        )

    def _detach_done_cb(self, future, step_done_callback):
        try:
            # 修复：DetachLink服务无success字段，调用成功即视为分离成功
            future.result()  # 仅判断是否调用成功，不访问success
            self.get_logger().info(f"物块{self.attach_params['model2_name']}分离成功！")
            time.sleep(0.5)
            step_done_callback()
        except Exception as e:
            self.get_logger().error(f"分离动作失败：{str(e)}")
            self.send_detach_action(step_done_callback)  # 重试分离

    # ===================== 9. 修复：抓取/放置完成后发布状态给主控=====================
    def _grab_proceed(self):
        if not self.is_executing:
            return
            
        if self.current_step == 1:
            self.get_logger().info("步骤2：夹爪闭合...")
            self.current_step = 2
            self.send_gripper_action("close", self._after_gripper_close)
        elif self.current_step == 2:
            self.get_logger().info("步骤3：执行物块吸附...")
            self.current_step = 3
            self.send_attach_action(self._after_attach)
        elif self.current_step == 3:
            self.get_logger().info("步骤4：机械臂举高（回到初始位置）...")
            self.current_step = 4
            self.send_arm_action(self.arm_trajectory["lift"], self._after_arm_lift)
        elif self.current_step == 4:
            self.get_logger().info(f"抓取流程全部完成！已吸附物块：{self.attach_params['model2_name']}")
            # 关键修复：发布抓取成功信号给主控
            self.arm_status_pub.publish(String(data="grasp_succeeded"))
            self.current_step = 5
            self.is_executing = False

    def _after_gripper_close(self):
        self._grab_proceed()

    def _after_attach(self):
        self._grab_proceed()

    def _after_arm_lift(self):
        self._grab_proceed()

    def _place_proceed(self):
        if not self.is_executing:
            return
            
        if self.current_step == 1:
            self.get_logger().info("步骤2：夹爪张开...")
            self.current_step = 2
            self.send_gripper_action("open", self._after_gripper_open)
        elif self.current_step == 2:
            self.get_logger().info("步骤3：执行物块分离...")
            self.current_step = 3
            self.send_detach_action(self._after_detach)
        elif self.current_step == 3:
            self.get_logger().info("步骤4：机械臂举高（回到初始位置）...")
            self.current_step = 4
            self.send_arm_action(self.arm_trajectory["lift"], self._after_arm_lift_place)
        elif self.current_step == 4:
            self.get_logger().info(f"放置流程全部完成！已分离物块：{self.attach_params['model2_name']}")
            # 关键修复：发布放置成功信号给主控
            self.arm_status_pub.publish(String(data="place_succeeded"))
            self.current_step = 5
            self.is_executing = False

    def _after_gripper_open(self):
        self._place_proceed()

    def _after_detach(self):
        self._place_proceed()

    def _after_arm_lift_place(self):
        self._place_proceed()

    def _reset_execution(self):
        self.get_logger().info("重置执行状态...")
        self.is_executing = False
        self.current_step = 0

    # ===================== 10. 原有回调（保留不变）=====================
    def cargo_callback(self, msg):
        if not self.is_executing:
            self.get_logger().info("="*50)
            self.get_logger().info(f"收到到达货物位置信号，准备抓取物块：{self.attach_params['model2_name']}")
            self.is_executing = True
            self.current_task = "grab"
            self.current_step = 1
            self.current_joint_pos = self.arm_trajectory["init"]
            self.get_logger().info("步骤1：机械臂弯曲到抓取位置...")
            self.send_arm_action(self.arm_trajectory["bend"], self._grab_proceed)
        else:
            self.get_logger().warn("正在执行抓取/放置流程，忽略本次信号！")

    def area_callback(self, msg):
        if not self.is_executing:
            self.get_logger().info("="*50)
            self.get_logger().info(f"收到到达放置位置信号，准备分离物块：{self.attach_params['model2_name']}")
            self.is_executing = True
            self.current_task = "place"
            self.current_step = 1
            self.current_joint_pos = self.arm_trajectory["init"]
            self.get_logger().info("步骤1：机械臂弯曲到放置位置...")
            self.send_arm_action(self.arm_trajectory["bend"], self._place_proceed)
        else:
            self.get_logger().warn("正在执行抓取/放置流程，忽略本次信号！")

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = ArmGrabPlaceNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("节点被用户中断！")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
