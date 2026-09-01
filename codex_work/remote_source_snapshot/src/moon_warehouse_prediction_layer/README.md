# moon_warehouse_prediction_layer

Nav2 costmap layer：把 Gazebo 移动障碍物的**速度外推到未来**，把预测位置
写入 costmap，让 A*/Hybrid-A* 与 MPPI 在障碍物**还没到**之前就看到它并
提前绕行。针对 `SimpleMovePlugin` 这类固定区间往复障碍物，这是成本最低、
见效最快的动态避障增强。

## 原理

- 订阅 `/gazebo/model_states`（或自定义 topic），缓存每个被跟踪模型的位置与线速度。
- 每个 costmap 更新周期按 `prediction_horizon_sec` 线性外推：`pos + vel * horizon`。
- 把外推位置以 `obstacle_radius_m` 为半径画成 LETHAL 成本写进 master costmap。
- MPPI 的优化窗口（`time_steps * dt`）因此会把"即将到来的障碍物"视为即时威胁，
  平滑地绕开；A* 的全局路径也会避开预测区域。

## 部署（远程虚拟机）

```bash
# 1) 把本包放进工作空间
cp -r moon_warehouse_prediction_layer ~/dev_ws/src/
cd ~/dev_ws

# 2) 编译（依赖 nav2_costmap_2d、gazebo_msgs 已在 Humble 环境）
source /opt/ros/humble/setup.bash
colcon build --packages-select moon_warehouse_prediction_layer
source install/setup.bash

# 3) 验证插件被 pluginlib 识别
ros2 pkg prefix moon_warehouse_prediction_layer
ros2 run nav2_costmap_2d nav2_costmap_2d_markers   # 可选，仅确认依赖可加载

# 4) 修改 Nav2 参数文件，注册预测层（见 config/prediction_layer_example.yaml）
#    把 plugins 列表加入 "prediction_layer"，并复制对应参数段。
```

## 需要按你的世界文件调整的参数

| 参数 | 建议值 | 说明 |
|---|---|---|
| `model_name_prefixes` | 你的移动障碍物在 Gazebo 里的模型名前缀 | 用 `gz model --list` 或 `ros2 topic echo /gazebo/model_states` 查看 |
| `robot_model_name` | `six_arm` | 排除机器人自身（含 `_base`） |
| `prediction_horizon_sec` | 2.0 | 与 MPPI `time_steps*dt` 匹配，一般 1.5~2.5s |
| `obstacle_radius_m` | ≥ 障碍物外接半径 | 太大会把通道堵死 |
| `predicted_cost` | 254 | 254=LETHAL；要留安全缓冲可降到 200~250 |

## 调试

```bash
# 看预测层是否在 costmap 里产生成本（RViz 勾选 local_costmap 显示）
ros2 topic echo /local_costmap/costmap --once

# 看障碍物实际位置与速度（确认前缀匹配）
ros2 topic echo /gazebo/model_states --once

# 看日志
ros2 topic echo /rosout --once | grep -i prediction
```

## 已知限制

1. 只做**匀速线性外推**：障碍物急停/转弯时预测会偏。对往复运动已足够，
   如需更强可用二次外推或卡尔曼滤波。
2. 预测成本是"标记"而非"跟踪"：障碍物离开后成本由该层在下一周期重算时
   自然消失（该层不持久化成本，只覆盖其 bounding box 区域）。
3. 本插件只是 costmap 输入之一，不替代 MPPI 本身的参数调优。
