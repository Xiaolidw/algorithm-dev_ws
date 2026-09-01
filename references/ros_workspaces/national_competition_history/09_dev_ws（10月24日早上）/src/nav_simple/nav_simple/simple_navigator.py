import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus  # 正确导入GoalStatus（ROS 2标准位置）
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler  # 欧拉角转四元数
import json

class SimpleNav2Navigator(Node):
    def __init__(self):
        super().__init__("simple_nav2_navigator")
        
        # 初始化Nav2动作客户端（对接/navigate_to_pose标准接口）
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        # 等待Nav2服务启动（超时10秒）
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2服务未启动！请先启动导航栈")
            rclpy.shutdown()
            return
        
        # 订阅手动目标话题（接收外部导航指令）
        self.target_sub = self.create_subscription(
            String, "/manual_nav_target", self.target_callback, 10
        )
        
        # 发布导航状态（供外部模块监听）
        self.status_pub = self.create_publisher(String, "/nav_status", 10)
        
        # 预设区域坐标（可根据实际地图修改）
        self.area_coords = {
            "A": {"x": 2.0, "y": 1.0, "yaw": 0.0},
            "B": {"x": 0.0, "y": 3.0, "yaw": 1.57},
            "C": {"x": -2.0, "y": 1.0, "yaw": 3.14}
        }
        
        self.get_logger().info("简化版Nav2导航节点启动成功！")
        self.get_logger().info("用法：发布目标到/manual_nav_target，格式1（区域）：{\"type\":\"area\",\"target\":\"A\"}；格式2（自定义坐标）：{\"type\":\"custom\",\"x\":1.5,\"y\":0.5,\"yaw\":0.0}")

    def target_callback(self, msg):
        """处理接收到的导航目标（区域或自定义坐标）"""
        try:
            data = json.loads(msg.data)
            if data["type"] == "area":
                # 导航到预设区域
                area = data.get("target", "")
                if area not in self.area_coords:
                    self.get_logger().error(f"无效区域：{area}（仅支持A/B/C）")
                    return
                coord = self.area_coords[area]
                self.get_logger().info(f"收到区域目标：{area}区（x={coord['x']}, y={coord['y']}）")
                self.send_goal(coord["x"], coord["y"], coord["yaw"])
            
            elif data["type"] == "custom":
                # 导航到自定义坐标
                x = data.get("x", 0.0)
                y = data.get("y", 0.0)
                yaw = data.get("yaw", 0.0)
                self.get_logger().info(f"收到自定义目标：x={x:.2f}, y={y:.2f}, 朝向={yaw:.2f}rad")
                self.send_goal(x, y, yaw)
            
            else:
                self.get_logger().error("目标类型错误！仅支持\"area\"或\"custom\"")
        
        except json.JSONDecodeError:
            self.get_logger().error("目标格式错误！请使用JSON格式，例如：{\"type\":\"area\",\"target\":\"A\"}")
        except KeyError as e:
            self.get_logger().error(f"缺少必要字段：{str(e)}")

    def send_goal(self, x, y, yaw):
        """构造导航目标并发送给Nav2"""
        # 构造位姿消息（必须使用map坐标系）
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        
        # 位置信息
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0  # 2D导航，z固定为0
        
        # 朝向信息（欧拉角转四元数）
        quat = quaternion_from_euler(0.0, 0.0, yaw)  # roll/pitch固定为0
        pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        
        goal_msg.pose = pose
        
        # 发送目标并注册回调
        self.get_logger().info("正在导航到目标点...")
        self._send_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback  # 实时反馈（可选）
        )
        self._send_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """实时反馈当前位置（可选，不影响核心功能）"""
        feedback = feedback_msg.feedback
        x = feedback.current_pose.pose.position.x
        y = feedback.current_pose.pose.position.y
        self.get_logger().debug(f"当前位置：x={x:.2f}, y={y:.2f}")  # 调试级别，INFO不显示

    def goal_response_callback(self, future):
        """处理目标接受/拒绝的回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("导航目标被Nav2拒绝！可能原因：目标不可达/地图未加载")
            self.status_pub.publish(String(data="failed: rejected"))
            return
        
        # 目标被接受，等待导航结果
        self.get_logger().info("导航目标已被接受，等待完成...")
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """处理导航完成后的结果"""
        try:
            # 获取导航状态（来自GoalStatus，而非result）
            status = future.result().status
            
            # 状态判断（仅使用ROS 2标准定义的状态码）
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("导航成功！已到达目标点")
                self.status_pub.publish(String(data="succeeded"))
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().error("导航失败：路径规划失败/避障中断")
                self.status_pub.publish(String(data="failed: aborted"))
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().warn("导航已被取消")
                self.status_pub.publish(String(data="canceled"))
            else:
                self.get_logger().error(f"导航结果未知，状态码：{status}")
                self.status_pub.publish(String(data=f"unknown: {status}"))
        
        except Exception as e:
            self.get_logger().error(f"处理结果时出错：{str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNav2Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断，退出节点")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
