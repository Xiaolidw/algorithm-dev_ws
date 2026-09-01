import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler  # 欧拉角转四元数
import json

class Nav2CargoNavigator(Node):
    def __init__(self):
        super().__init__("nav2_cargo_navigator")
        
        # -------------------------- 1. 初始化核心组件 --------------------------
        # 1.1 创建 Nav2 动作客户端（调用 /navigate_to_pose 接口）
        self.nav_action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        # 等待 Nav2 服务就绪（超时10秒）
        if not self.nav_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 服务未启动！请先启动 Nav2 导航栈")
            rclpy.shutdown()
            return

        # 1.2 订阅模块二的任务话题 /llama_command（获取目标区域 A/B/C）
        self.task_subscriber = self.create_subscription(
            String,
            "/llama_command",
            self.task_callback,  # 任务解析回调
            10
        )

        # 1.3 订阅模拟的视觉位置话题 /sim_cargo_pos（替代视觉模块）
        self.sim_cargo_subscriber = self.create_subscription(
            String,
            "/sim_cargo_pos",
            self.sim_cargo_callback,  # 模拟货物位置回调
            10
        )

        # 1.4 发布导航状态话题 /nav_status（给后续抓取模块发信号）
        self.nav_status_publisher = self.create_publisher(
            String,
            "/nav_status",
            10
        )

        # -------------------------- 2. 初始化全局变量 --------------------------
        self.current_task = None  # 存储当前任务：{"color":"red","num":2,"area":"A"}
        self.sim_cargo_position = None  # 存储模拟货物位置：{"x":2.5,"y":1.0}
        # 预设 A/B/C 区坐标（需根据实际 Gazebo 地图调试！）
        self.area_coords = {
            "A": {"x": 3.0, "y": 0.0, "yaw": 0.0},
            "B": {"x": 0.0, "y": 3.0, "yaw": 1.57},
            "C": {"x": -3.0, "y": 0.0, "yaw": 3.14}
        }
        self.get_logger().info("Nav2 导航节点初始化完成（视觉位置模拟版）")

    # -------------------------- 3. 回调函数：解析模块二的任务 --------------------------
    def task_callback(self, msg):
        try:
            # 解析 /llama_command 的 JSON 任务
            self.current_task = json.loads(msg.data)
            task_color = self.current_task.get("color", "red")
            task_area = self.current_task.get("area", "A")
            task_num = self.current_task.get("num", 1)
            self.get_logger().info(f"收到任务：搬运 {task_num} 个 {task_color} 包裹到 {task_area} 区")

            # 若已获取模拟货物位置，直接触发导航到货物
            if self.sim_cargo_position:
                self.navigate_to_target(
                    x=self.sim_cargo_position["x"],
                    y=self.sim_cargo_position["y"],
                    yaw=0.0,  # 朝向货物的角度（可调试）
                    target_type="cargo"  # 标记是“导航到货物”
                )
        except json.JSONDecodeError as e:
            self.get_logger().error(f"解析任务失败：{str(e)}，原始消息：{msg.data}")

    # -------------------------- 4. 回调函数：接收模拟的视觉位置 --------------------------
    def sim_cargo_callback(self, msg):
        try:
            # 解析模拟的货物位置（格式：{"x":2.5,"y":1.0,"color":"red"}）
            self.sim_cargo_position = json.loads(msg.data)
            cargo_color = self.sim_cargo_position.get("color", "red")
            self.get_logger().info(f"收到模拟货物位置：x={self.sim_cargo_position['x']}, y={self.sim_cargo_position['y']}, 颜色={cargo_color}")

            # 若已有当前任务，且货物颜色匹配，触发导航
            if self.current_task and self.current_task["color"] == cargo_color:
                self.navigate_to_target(
                    x=self.sim_cargo_position["x"],
                    y=self.sim_cargo_position["y"],
                    yaw=0.0,
                    target_type="cargo"
                )
        except json.JSONDecodeError as e:
            self.get_logger().error(f"解析模拟货物位置失败：{str(e)}，原始消息：{msg.data}")

    # -------------------------- 5. 核心函数：发送导航目标给 Nav2 --------------------------
    def navigate_to_target(self, x, y, yaw, target_type):
        """
        发送导航目标到 Nav2
        :param x: 目标x坐标（map坐标系）
        :param y: 目标y坐标（map坐标系）
        :param yaw: 目标朝向（欧拉角，单位：弧度）
        :param target_type: 目标类型（"cargo"=货物位置，"area"=放置区域）
        """
        # 构造 Nav2 动作目标（PoseStamped 格式，必须用 map 坐标系）
        goal_msg = NavigateToPose.Goal()
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"  # 与 Nav2 全局坐标系一致
        pose_stamped.header.stamp = self.get_clock().now().to_msg()

        # 1. 设置目标位置
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = 0.0  # 2D导航，z固定为0

        # 2. 设置目标朝向（欧拉角转四元数，Nav2 只接受四元数）
        quat = quaternion_from_euler(0.0, 0.0, yaw)  # roll=0, pitch=0, yaw=目标朝向
        pose_stamped.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )

        # 3. 绑定目标位姿到动作目标
        goal_msg.pose = pose_stamped

        # 4. 发送导航目标，并注册结果回调
        self.get_logger().info(f"开始导航到 {target_type}：x={x}, y={y}, 朝向={yaw:.2f}rad")
        send_goal_future = self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback  # 实时反馈回调（可选）
        )
        # 绑定导航完成后的结果处理
        send_goal_future.add_done_callback(
            lambda future: self.nav_result_callback(future, target_type)
        )

    # -------------------------- 6. 导航实时反馈（可选，用于调试） --------------------------
    def nav_feedback_callback(self, feedback_msg):
        # 实时获取导航进度（如距离目标的距离）
        current_pose = feedback_msg.feedback.current_pose.pose
        self.get_logger().debug(f"导航中：当前位置 x={current_pose.position.x:.2f}, y={current_pose.position.y:.2f}")

    # -------------------------- 7. 导航结果处理（触发后续流程） --------------------------
    def nav_result_callback(self, future, target_type):
        try:
            goal_result = future.result()
            # 检查导航是否被接受
            if not goal_result.accepted:
                self.get_logger().error(f"导航目标被拒绝（{target_type}）")
                self.publish_nav_status(f"failed_{target_type}")
                return

            # 检查导航是否成功
            nav_result = goal_result.get_result_async().result().result
            if nav_result.status == 3:  # Nav2 成功状态码：3=SUCCEEDED
                self.get_logger().info(f"导航成功到达 {target_type}！")
                self.publish_nav_status(f"succeeded_{target_type}")

                # 若导航到货物，后续触发抓取；若到目标区，触发放置（给抓取模块信号）
                if target_type == "cargo" and self.current_task:
                    self.get_logger().info("导航到货物，等待抓取模块触发...")
                elif target_type == "area":
                    self.get_logger().info("导航到目标区域，等待放置模块触发...")
            else:
                self.get_logger().error(f"导航失败（{target_type}），状态码：{nav_result.status}")
                self.publish_nav_status(f"failed_{target_type}")
        except Exception as e:
            self.get_logger().error(f"处理导航结果失败：{str(e)}")

    # -------------------------- 8. 辅助函数：发布导航状态 --------------------------
    def publish_nav_status(self, status):
        """发布导航状态：succeeded_cargo/failed_cargo/succeeded_area/failed_area"""
        status_msg = String()
        status_msg.data = status
        self.nav_status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    # 使用多线程执行器（避免回调阻塞）
    executor = MultiThreadedExecutor()
    navigator_node = Nav2CargoNavigator()
    executor.add_node(navigator_node)

    try:
        executor.spin()  # 循环处理回调
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        navigator_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()