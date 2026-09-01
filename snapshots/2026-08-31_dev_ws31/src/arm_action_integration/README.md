# moon_warehouse_moveit

阶段 1 固定位置抓取包。当前实现不依赖导航和视觉坐标，使用已知关节位姿验证机械臂、夹爪及 Gazebo Link Attacher 链路。

## 结构

```text
moon_warehouse_moveit/
├── config/fixed_manipulation.yaml       # 控制器、模型名、关节名及固定位姿
├── launch/fixed_manipulation.launch.py  # dry-run/真实执行入口
├── fixed_manipulation_server.py         # 抓取与放置 Action 服务端
├── manipulation_health_check.py         # 只读自动健康检查
└── send_fixed_task.py                   # 人工验收客户端
```

上层只调用 `/manipulation/execute`，不会直接依赖机械臂控制器或 Gazebo 服务。后续把固定关节位姿替换为 MoveIt 规划时，可以保留同一个 Action 接口。

## 执行顺序

- `pick`：张开夹爪 → 到达抓取位姿 → 闭合夹爪 → 吸附物块 → 抬升。
- `place`：到达放置位姿 → 张开夹爪 → 解除吸附 → 返回 Home。
- 校准操作：`home`、`pick_pose`、`lift`、`place_pose`、`open`、`close`，每次只执行一个动作。

所有平台参数均在 `config/fixed_manipulation.yaml` 中配置。该文件中的抓取位姿来源于参考项目，只是首轮标定起点，真实验收前必须在空载条件下确认不会碰撞。

详细操作见工作区根目录的《固定位置机械臂抓取与放置验收指南》。
