import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import requests
import json
import re

class LlamaCommandParser(Node):
    def __init__(self):
        super().__init__("llama_command_parser")
        
        # 大模型服务端配置
        self.llama_server_url = "http://localhost:8081/completion"
        self.model_params = {
            "n_predict": 512,
            "temperature": 0.05,
            "stop": ["\n", "```"],  # 停止标记：代码块结束符
            "stream": False
        }

        # 发布者定义：仅保留/chat话题用于发布解析后的JSON
        self.chat_pub = self.create_publisher(String, "/chat", 10)
        self.color_pub = self.create_publisher(Int32, "/color", 10)
        self.ask_pub = self.create_publisher(Int32, "/ask", 10)
        self.pick_pub = self.create_publisher(Int32, "/pick", 10)
        self.cur_pub = self.create_publisher(Int32, "/cur", 10)

        # 订阅者定义：订阅/chat话题接收对话框的自然语言指令
        self.chat_sub = self.create_subscription(
            String, "/chat", self.chat_callback, 10
        )

        self.get_logger().info("大模型指令解析节点已启动 - 请在Foxglove对话框输入自然语言指令")

    def chat_callback(self, msg):
        try:
            # 直接接收自然语言文本（如“搬运3个红色到B区”）
            natural_language = msg.data
            self.get_logger().info(f"从对话框接收到自然语言指令：{natural_language}")
            
            self.parse_and_sync(natural_language)
            
        except Exception as e:
            self.get_logger().error(f"处理指令时发生异常：{str(e)}")
            self._reset_system_state()

    def parse_and_sync(self, natural_language):
        # Prompt：明确要求大模型将自然语言转为指定格式的JSON数组
        prompt = f"""你是一个机器人抓取放置助手。
严格返回纯JSON数组，不要任何前缀（如"json"）、后缀或解释文字。
输入示例: "搬运1个红色到B区，2个蓝色到C区"
输出示例: [{{"color":"red","num":1,"to":"B"}}, {{"color":"blue","num":2,"to":"C"}}] 
必须使用双引号，颜色值仅允许"red"或"blue"，to为字母区（如"A"、"B"）。
用户输入: "{natural_language}"
"""
        request_data = {"prompt": prompt, **self.model_params}

        try:
            self.get_logger().info("正在调用大模型解析指令...")
            response = requests.post(self.llama_server_url, json=request_data, timeout=60)
            response.raise_for_status()
            
            llm_raw_output = response.json()["content"].strip()
            self.get_logger().info(f"大模型原始输出：{llm_raw_output}")
            
            cleaned_json = self.clean_output(llm_raw_output)
            self.get_logger().info(f"清洗后的JSON：{cleaned_json}")
            
            # 解析为Python对象
            task_data = json.loads(cleaned_json)
            self.get_logger().info(f"成功解析任务数据：{task_data}")

            self.sync_to_foxglove(task_data)

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"大模型API请求失败：{str(e)}")
            self._reset_system_state()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON解析失败：{str(e)}，清洗后的JSON为：{cleaned_json}")
            self._reset_system_state()
        except Exception as e:
            self.get_logger().error(f"指令解析过程中发生未知错误：{str(e)}")
            self._reset_system_state()

    def clean_output(self, raw_output):
        """清洗大模型输出，确保为标准JSON数组"""
        cleaned = raw_output.strip().strip('"').strip("`").strip("json").strip()
        # 提取以[开头、]结尾的JSON数组
        json_match = re.search(r'\[.*?\]', cleaned, re.DOTALL)
        if json_match:
            cleaned = json_match.group()
        # 修复单引号为双引号
        cleaned = cleaned.replace("'", '"')
        # 移除末尾多余逗号
        cleaned = re.sub(r',\s*]', ']', cleaned)
        return cleaned

    def sync_to_foxglove(self, task_data):
        # 处理任务数据（兼容单任务和多任务）
        if isinstance(task_data, list) and len(task_data) > 0:
            first_task = task_data[0]
        else:
            first_task = task_data if isinstance(task_data, dict) else {"color":"blue","num":1,"to":"A"}

        # 提取并标准化参数
        color = first_task.get("color", "blue").lower()
        color = "blue" if color not in ["red", "blue"] else color
        num = max(1, int(first_task.get("num", 1)))
        to_area = first_task.get("to", "A").upper()

        self.get_logger().info(f"任务参数 - 颜色:{color}, 数量:{num}, 目标区域:{to_area}")

        # 发布解析后的JSON到/chat话题
        chat_msg = String()
        chat_msg.data = json.dumps(task_data, ensure_ascii=False)
        self.chat_pub.publish(chat_msg)
        self.get_logger().info("✅ 已发布解析后的JSON到/chat话题")

        # 发布其他状态到Foxglove（颜色、任务类型、抓取状态等）
        color_msg = Int32()
        color_msg.data = 0 if color == "blue" else 1
        self.color_pub.publish(color_msg)
        self.ask_pub.publish(Int32(data=color_msg.data))
        self.pick_pub.publish(Int32(data=-1))  # 未使能
        self.cur_pub.publish(Int32(data=1))    # 执行中

        self.get_logger().info("🎯 所有状态已同步到Foxglove")
        # 发布解析后的JSON到/chat话题，添加type标识
        
    chat_msg = String()
    # 包装成带标识的字典，明确这是解析后的任务数据
    wrapped_data = {
        "type": "task",  # 标识字段
        "data": task_data
    }
    chat_msg.data = json.dumps(wrapped_data, ensure_ascii=False)
    self.chat_pub.publish(chat_msg)
    self.get_logger().info("✅ 已发布解析后的JSON到/chat话题")

    def _reset_system_state(self):
        """系统异常时重置状态"""
        self.get_logger().info("⚠️ 重置系统状态")
        self.pick_pub.publish(Int32(data=-1))
        self.cur_pub.publish(Int32(data=0))

def main(args=None):
    rclpy.init(args=args)
    node = LlamaCommandParser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("接收到中断信号，正在关闭节点...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("大模型指令解析节点已安全退出")

if __name__ == "__main__":
    main()
