# CoStudio 本地语义发布链（2026-09-10）

本目录备份已部署到 `/home/ros/dev_ws` 的 CoStudio 赛题发布、本地 Qwen 解析和五物块批执行代码，以及 Windows CoStudio 1.1.0 扩展与布局。

验收请求为蓝色 3 件到 B、红色 2 件到 A。端到端用时 237.391 秒（含本地模型解析），执行器用时 231.710 秒，5/5 成功，Nav2 恢复 0 次。证据日志位于虚拟机 `/home/ros/dev_ws/logs/costudio_mission_20260910_200533.log`。

部署对应关系：

- `moon_warehouse_semantic/` → `src/moon_warehouse_semantic/` 中同名模块、配置和 launch；
- `moon_warehouse_coordinator/` → `src/moon_warehouse_coordinator/` 中同名模块、setup 和 launch；
- `competition-dashboard/` → Windows CoStudio 自定义扩展；
- `2026 比赛可视化.json` → CoStudio 布局。

本变更未包含、未修改任何 world、地图、墙、石块、区域、物块初始坐标或移动障碍运动参数。
