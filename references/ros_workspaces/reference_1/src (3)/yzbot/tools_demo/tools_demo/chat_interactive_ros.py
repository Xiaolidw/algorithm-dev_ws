import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json
import threading
import time

# 反馈模板（status: success/error/pending）
FEEDBACK_TEMPLATE = {
    "status": "",
    "result": "",
    "message": ""
}

class DualModeRobotTaskNode(Node):
    def __init__(self):
        super().__init__("dual_mode_robot_task")
        # 1. ROS2话题初始化（Foxglove交互用）
        self.task_pub = self.create_publisher(String, "robot_task", 10)  # 任务发布
        self.foxglove_input_sub = self.create_subscription(
            String, "foxglove_task_input", self.foxglove_input_cb, 10
        )  # Foxglove指令输入话题
        self.foxglove_confirm_sub = self.create_subscription(
            String, "foxglove_task_confirm", self.foxglove_confirm_cb, 10
        )  # Foxglove确认指令话题
        self.feedback_pub = self.create_publisher(String, "task_feedback", 10)  # 双端共享反馈话题

        # 2. 共享状态管理（加线程锁，防止两端冲突）
        self.lock = threading.Lock()  # 线程锁
        self.pending_task = None  # 待确认任务（共享）
        self.llama_server_url = "http://localhost:8081/v1/chat/completions"
        self.system_prompt = """
        仅处理红/蓝色物块搬运指令，输出纯JSON数组（无额外文本）：
        输入示例："把1个红色物块放到A，2个蓝色放到B"
        输出示例：[{"color":"red","num":1,"to":"A"},{"color":"blue","num":2,"to":"B"}]
        规则：color仅red/blue，to仅A/B/C，未识别则输出[]
        """

        # 3. 启动终端交互子线程（独立于主线程，不阻塞Foxglove）
        self.terminal_thread = threading.Thread(target=self.terminal_interactive, daemon=True)
        self.terminal_thread.start()

        # 4. 启动提示（双端）
        self.send_feedback(
            "success",
            message="双端交互模块启动！\n终端：直接输入自然语言指令（输入quit退出）\nFoxglove：用「指令输入面板」输入，「确认面板」操作"
        )

    def send_feedback(self, status, result="", message=""):
        """双端同步反馈：终端打印 + Foxglove话题发布"""
        # 1. 生成反馈JSON（Foxglove用）
        feedback = FEEDBACK_TEMPLATE.copy()
        feedback["status"] = status
        feedback["result"] = result
        feedback["message"] = message
        feedback_str = json.dumps(feedback, ensure_ascii=False)
        self.feedback_pub.publish(String(data=feedback_str))

        # 2. 终端打印（自然语言格式）
        terminal_msg = f"\n[反馈] {message}"
        if result:
            terminal_msg += f"\n[解析结果] {result}"
        if status == "error":
            self.get_logger().error(terminal_msg)
        else:
            self.get_logger().info(terminal_msg)

    def parse_task(self, user_input):
        """共享指令解析函数：调用Llama，返回（有效任务列表，提示信息）"""
        try:
            response = requests.post(
                self.llama_server_url,
                json={
                    "model": "qwen2.5-coder-0.5b-instruct",
                    "messages": [{"role":"system","content":self.system_prompt},{"role":"user","content":user_input}],
                    "max_tokens": 256, "temperature": 0.1, "timeout": 3
                },
                headers={"Content-Type": "application/json"}
            )
            response.raise_for_status()
            task_list = json.loads(response.json()["choices"][0]["message"]["content"].strip())
            valid_tasks = [t for t in task_list if t.get("color") in ["red","blue"] and t.get("to") in ["A","B","C"]]
            return valid_tasks, "指令解析成功"
        except requests.exceptions.ConnectionError:
            return [], "Llama模型未连接（请先启动llama-server）"
        except json.JSONDecodeError:
            return [], "模型输出格式错误（非JSON）"
        except Exception as e:
            return [], f"解析失败：{str(e)}"

    # ------------------------------ 终端交互逻辑 ------------------------------
    def terminal_interactive(self):
        """终端自然语言交互：输入→解析→确认→发布"""
        while rclpy.ok():
            # 1. 终端输入自然语言指令
            user_input = input("\n请输入终端指令（输入quit退出）：").strip()
            if user_input.lower() == "quit":
                self.send_feedback("success", message="终端交互退出")
                break
            if not user_input:
                self.send_feedback("error", message="终端指令为空，请重新输入")
                continue

            # 2. 解析指令（加锁，防止与Foxglove冲突）
            with self.lock:
                self.send_feedback("success", message=f"终端正在解析指令：{user_input}")
                valid_tasks, parse_msg = self.parse_task(user_input)

                if not valid_tasks:
                    self.send_feedback("error", result="[]", message=f"终端{parse_msg}（无有效任务）")
                    continue

                # 3. 终端确认（自然语言选择Y/N）
                task_str = json.dumps(valid_tasks, ensure_ascii=False)
                self.send_feedback("pending", result=task_str, message=f"终端解析完成！待确认发布")
                while True:
                    confirm = input("是否发布到robot_task话题？[Y/N]：").strip().upper()
                    if confirm in ["Y", "N"]:
                        break
                    print("请输入Y（确认）或N（取消）")

                # 4. 执行发布/取消
                if confirm == "Y":
                    self.task_pub.publish(String(data=task_str))
                    self.send_feedback("success", result=task_str, message="终端已确认发布任务")
                    self.pending_task = None
                else:
                    self.send_feedback("success", result=task_str, message="终端已取消发布任务")
                    self.pending_task = None

    # ------------------------------ Foxglove交互逻辑 ------------------------------
    def foxglove_input_cb(self, msg):
        """处理Foxglove输入的自然语言指令"""
        user_input = msg.data.strip()
        if not user_input:
            self.send_feedback("error", message="Foxglove指令为空，请重新输入")
            return

        # 加锁处理，防止与终端冲突
        with self.lock:
            # 1. 解析指令
            self.send_feedback("success", message=f"Foxglove正在解析指令：{user_input}")
            valid_tasks, parse_msg = self.parse_task(user_input)

            if not valid_tasks:
                self.send_feedback("error", result="[]", message=f"Foxglove{parse_msg}（无有效任务）")
                return

            # 2. 进入待确认状态（共享pending_task）
            self.pending_task = valid_tasks
            task_str = json.dumps(valid_tasks, ensure_ascii=False)
            self.send_feedback(
                "pending",
                result=task_str,
                message=f"Foxglove解析完成！请在「确认面板」点击“确认发布”或“取消发布”"
            )

    def foxglove_confirm_cb(self, msg):
        """处理Foxglove的确认指令（中文按钮对应“confirm”/“cancel”）"""
        confirm_cmd = msg.data.strip().lower()

        # 加锁处理，防止与终端冲突
        with self.lock:
            if self.pending_task is None:
                self.send_feedback("error", message="Foxglove无待确认任务，请先输入指令")
                return

            # 执行发布/取消
            task_str = json.dumps(self.pending_task, ensure_ascii=False)
            if confirm_cmd == "confirm":
                self.task_pub.publish(String(data=task_str))
                self.send_feedback("success", result=task_str, message="Foxglove已确认发布任务")
                self.pending_task = None
            elif confirm_cmd == "cancel":
                self.send_feedback("success", result=task_str, message="Foxglove已取消发布任务")
                self.pending_task = None
            else:
                self.send_feedback("error", message=f"Foxglove无效指令：{confirm_cmd}，仅支持confirm/cancel")

def main(args=None):
    rclpy.init(args=args)
    node = DualModeRobotTaskNode()
    try:
        rclpy.spin(node)  # 主线程：监听Foxglove话题
    except KeyboardInterrupt:
        node.send_feedback("success", message="程序整体退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
