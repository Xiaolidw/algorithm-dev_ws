import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import requests
import json
import re

class LlamaCommandParser(Node):
    def __init__(self):
        super().__init__("llama_command_parser")
        
        # 1. 大模型配置（符合《相关接口说明.pdf》LLaMA.cpp服务端参数）
        self.llama_server_url = "http://localhost:8081/completion"  # 固定端口
        self.model_params = {
            "n_predict": 512,    # 参考《竞赛实验指导书》模型输出长度
            "temperature": 0.05, # 低温度确保解析稳定（《总步骤.pdf》模块2要求）
            "stop": ["\n"],       # 终止符避免多余输出
            "stream": False       # 非流式响应（便于单次处理）
        }
        
        # 2. 任务ID与状态管理（解决命令跳变）
        self.task_counter = 0          # 任务计数器（生成唯一ID）
        self.active_task_id = None     # 当前活跃任务ID（标记最新命令）
        self.is_processing = False     # 任务处理锁（避免并发）
        self.latest_chat_msg = String()# 存储最新/chat消息
        
        # 3. 发布者（向Foxglove面板发布解析结果）
        self.chat_pub = self.create_publisher(String, "/chat", 10)  # 解析结果显示面板
        #self.color_pub = self.create_publisher(Int32, "/color", 10) # 目标颜色面板（0=蓝，1=红）
        #self.ask_pub = self.create_publisher(Int32, "/ask", 10)    # 目标任务面板（与颜色对应）
        #self.pick_pub = self.create_publisher(Int32, "/pick", 10)  # 抓取状态面板（-1=未使能）
        #self.cur_pub = self.create_publisher(Int32, "/cur", 10)    # 作业状态面板（1=处理中）
        
        # 4. 订阅者（接收Foxglove对话框的自然语言命令，话题改为 /command）
        self.command_sub = self.create_subscription(
            String,
            "/command",  # 关键修改：订阅 /command 接收命令，避免与 /chat 发布冲突
            self.chat_callback,
            10
        )
        self.get_logger().info("指令解析节点启动（话题分离+ID过滤，符合竞赛要求）")

    def chat_callback(self, msg):
        """新命令触发：生成新ID→终止旧任务→仅处理最新任务"""
        try:
            # 步骤1：提取纯命令文本（兼容JSON封装/纯文本）
            task_text = msg.data
            try:
                task_json = json.loads(task_text)
                if isinstance(task_json, dict):
                    task_text = task_json.get("data", task_text)
            except json.JSONDecodeError:
                pass  # 非JSON格式直接使用
            
            # 步骤2：过滤空命令/无效命令
            task_text_stripped = task_text.strip()
            if not task_text_stripped:
                self.get_logger().debug("忽略空命令")
                return
            if not any(keyword in task_text_stripped for keyword in ["红", "蓝", "A", "B", "C"]):
                self.get_logger().warn(f"命令无有效信息（需含颜色/区域）：{task_text_stripped}")
                return
            
            # 步骤3：生成新任务ID并标记为活跃
            self.task_counter += 1
            new_task_id = self.task_counter
            self.active_task_id = new_task_id
            
            # 步骤4：终止旧任务（双重保障：状态锁+ID过滤）
            if self.is_processing:
                self.get_logger().warn(f"新任务（ID:{new_task_id}）插队，终止旧任务")
                self.is_processing = False
            
            # 步骤5：启动新任务
            self.is_processing = True
            self.get_logger().info(f"处理最新任务（ID:{new_task_id}）：{task_text_stripped}")
            self.parse_and_publish_once(task_text_stripped, new_task_id)
            
        except Exception as e:
            self.is_processing = False
            self.get_logger().error(f"命令接收错误：{str(e)}")

    def parse_and_publish_once(self, task_text, current_task_id):
        """单次解析+ID过滤：仅当前活跃任务能发布结果"""
        # 前置校验：任务已过期则终止
        if not self.is_processing or current_task_id != self.active_task_id:
            self.get_logger().info(f"任务（ID:{current_task_id}）已过期，不发布结果")
            return
        
        try:
            # 1. 构造大模型Prompt（强化格式约束）
            prompt = f"""你是月球仓储机器人专属解析助手，仅返回纯JSON数组！无任何多余文本、标点、换行！
必须严格按以下规则解析：
1. 每个任务包含3个字段：
   - color：仅red（对应“红”）或blue（对应“蓝”）
   - num：正整数（对应“1个”“一个”“3个”等数量描述）
   - to：仅A/B/C（对应“到X”“去X”中的区域）
2. 多任务按命令中出现的顺序解析，前一个任务先输出，后一个任务后输出。

单任务示例1（数字数量词）：
输入"抓取1个蓝色物块去B"→输出[{{"color":"blue","num":1,"to":"B"}}]

单任务示例2（中文数量词）：
输入"抓一个红色到A区"→输出[{{"color":"red","num":1,"to":"A"}}]

多任务示例1（混合数量词）：
输入"抓取1个蓝色去B，一个红色去A"→输出[{{"color":"blue","num":1,"to":"B"}},{{"color":"red","num":1,"to":"A"}}]

多任务示例2（纯数字数量词）：
输入"抓取3个蓝色去B，2个红色去A"→输出[{{"color":"blue","num":3,"to":"B"}},{{"color":"red","num":2,"to":"A"}}]

当前任务："{task_text}"
"""

            request_data = {"prompt": prompt, **self.model_params}
            
            # 2. 调用大模型（超时控制+响应校验）
            response = requests.post(
                self.llama_server_url,
                json=request_data,
                timeout=30
            )
            response.raise_for_status()
            response_data = response.json()
            
            # 3. 提取模型输出（校验非空）
            llm_raw_output = response_data.get("content", "").strip()
            if not llm_raw_output:
                raise ValueError("模型返回空内容")
            
            # 4. 二次ID校验：避免延迟返回导致的过期发布
            if current_task_id != self.active_task_id:
                self.get_logger().warn(f"任务（ID:{current_task_id}）已被替换，终止发布")
                return
            
            # 5. 增强版JSON清洗+解析
            task_json = self.clean_and_parse_json(llm_raw_output, current_task_id)
            
            # 6. 标准化任务数据为列表
            if isinstance(task_json, dict):
                task_json = [task_json]
            if not isinstance(task_json, list) or len(task_json) == 0:
                raise ValueError(f"解析结果非有效任务列表：{task_json}")
            
            # 7. 发布最新/chat结果（覆盖旧结果）
            self.latest_chat_msg.data = json.dumps(task_json, ensure_ascii=False)
            self.chat_pub.publish(self.latest_chat_msg)
            self.get_logger().info(f"任务（ID:{current_task_id}）已发布到/chat：{self.latest_chat_msg.data}")
            
            # 8. 同步Foxglove其他面板
            #self.sync_foxglove_panels(task_json[0], current_task_id)
            
        except Exception as e:
            self.get_logger().error(f"解析发布失败：{str(e)}")
        finally:
            if current_task_id == self.active_task_id:
                self.is_processing = False

    def clean_and_parse_json(self, raw_output, task_id):
        """增强版JSON清洗+解析：3层容错"""
        # 层1：基础清洗
        cleaned = raw_output.strip()
        for char in ['。', '.', '"', "'", ';', ' ']:
            cleaned = cleaned.strip(char)
        cleaned = cleaned.replace("```json", "").replace("```", "").strip()
        
        # 层2：尝试直接解析
        try:
            return json.loads(cleaned)
        except json.JSONDecodeError:
            # 层3：正则提取JSON数组
            json_match = re.search(r'\[.*?\]', cleaned, re.DOTALL)
            if not json_match:
                raise ValueError(f"未提取到JSON数组：{cleaned[:100]}...")
            extracted = json_match.group().strip()
            try:
                return json.loads(extracted)
            except json.JSONDecodeError as e:
                raise ValueError(f"提取内容非有效JSON：{extracted}") from e

    def sync_foxglove_panels(self, task_info, current_task_id):
        """同步Foxglove面板：仅活跃任务执行"""
        if current_task_id != self.active_task_id:
            return
        
        color = task_info.get("color", "blue").lower()
        
        # 颜色面板（0=蓝，1=红）
        color_msg = Int32()
        color_msg.data = 0 if color == "blue" else 1
        self.color_pub.publish(color_msg)
        
        # 任务面板（与颜色对应）
        ask_msg = Int32()
        ask_msg.data = 0 if color == "blue" else 1
        self.ask_pub.publish(ask_msg)
        
        # 抓取状态面板（初始化未使能）
        pick_msg = Int32()
        pick_msg.data = -1
        self.pick_pub.publish(pick_msg)
        
        # 作业状态面板（1=处理中）
        cur_msg = Int32()
        cur_msg.data = 1
        self.cur_pub.publish(cur_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LlamaCommandParser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被手动终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()