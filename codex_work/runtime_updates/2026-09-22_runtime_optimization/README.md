# 2026-09-22 证据驱动整体提速

## 保留的修改

- 修复 A 区退出后前往西侧蓝块时无条件绕到 B 东侧中转的问题；只有实际夹取停车点 `x > -2.30` 才使用东侧中转。
- 比赛批次按稳定目的地区域顺序重排为 `B -> A -> C`；同一区域保持原相对顺序，物块继续由 `fastest-*` 按完整夹取与携带成本选择。CoStudio 发布的计划顺序同步重排，前端目标与实际执行一致。
- C 区实现真实北、南两条物理路线。根据移动障碍物 2 的实时 `y/vy`、实测有效行驶速度和到达交叉点 ETA 选择较低总耗时路线，选择后锁存至通过交叉点。
- 非末件 C 放置后在退出时重新计算 ETA，避免沿用已经过期的进场方向。

## 回归证据

| 验证 | 结果 | 总耗时 | 恢复次数 |
|---|---:|---:|---:|
| A→西侧 blue2 定向重复 | 通过 | 130.250 s（两件） | 0 |
| 3红A+2蓝B，输入故意 A 在前 | 5/5 | 333.420 s | 0 |
| 单蓝→C，旧北路 | 1/1 | 81.768 s | 0 |
| 单蓝→C，ETA 南路 | 1/1 | 62.391 s | 0 |
| 连续两蓝→C，首次双路线 | 2/2 | 198.326 s | 0 |
| 连续两蓝→C，退出 ETA 重选 | 2/2 | 176.119 s | 0 |
| 3红→B + 2蓝→C 五件混合 | 5/5 | 369.564 s | 0 |
| 连续两蓝→C，计入严格航点开销 | 2/2 | 155.073 s | 0 |
| 3红→B + 2蓝→C，计入严格航点开销 | 5/5 | 345.959 s | 0 |
| 连续两蓝→C，近端锁存＋北路裕量＋障碍2 0.50m/s | 2/2 | 153.098 s | 0 |
| 3红→B + 2蓝→C，2026-09-25 完整回归 | 5/5 | 343.571 s | 0 |

对 2026-09-15 基线：A/B 从 483.599 s 降至 333.420 s，缩短 150.179 s；连续两件 C 在退出重选后再缩短 22.207 s；B/C 从 407.875 s 降至 369.564 s，缩短 38.311 s。B/C 单件耗时依次为 63.511 / 67.077 / 59.547 / 114.500 / 64.905 s，剩余瓶颈是第一件 C 的安全进入和非末件退出，而不是 Nav2 恢复或机械臂动作。

进一步校准发现 ETA 只计算距离和动态等待，没有计算北路额外两个严格航点产生的 Nav2 取消、停稳、重新接管成本。按成功日志的实测开销给北路增加 8.0 s 编排成本后，路线几何、控制速度和动态放行条件均未改变：连续两件 C 降至 155.073 s；B/C 完整五件降至 345.959 s，单件为 63.308 / 66.146 / 58.996 / 97.286 / 60.199 s，恢复次数全部为 0。相对 407.875 s 基线累计缩短 61.916 s，正式进入 350 s 底线。

2026-09-25 继续将 C 路线选择延迟到两条路线共有的 `(0.35,-1.00)` 等待点，避免长途行驶期间障碍相位变化使选择过期；该变化没有新增航点。北路只有在预测总耗时至少比南路短 6 秒时才切换，防止亚秒级预测差触发两个额外严格航点。分层验证后，下半区障碍 2 在保持 `(2,-3)↔(2,-6)` 轨迹不变的前提下改为 `0.50 m/s`；上半区障碍保持不变。连续两件 C 为 153.098 s，完整 B/C 五件为 343.571 s，全部恢复0且三维放置验收通过。该组合比 345.959 s 基线小幅缩短 2.388 s，但尚未达到 300 s；三件红→B共192.337 s、两件C共151.231 s，后续优化必须同时覆盖安全开阔主干和航点交接，不能再依赖单纯提高障碍速度。

## 被证据否决并回退的实验

- 曾单变量尝试开阔携带段 `1.15 m/s`、门洞/轨道 `0.80 m/s`。第一件 B 放置时几何、倾角、区域均合格，但松爪后首个速度样本为 `0.179 m/s`，严格验收 code=5。该速度分层已完整回退，正式版本继续使用已验证的全程携带 `1.05 m/s`，没有放宽验收或安全距离。
- 曾把 B/C 最终预接近交接从动态约 0.2--0.3 m 收紧为严格 0.08 m，希望让 Nav2 完成更快的大角度旋转。定向 B+C 虽 2/2 安全通过、恢复0，但单件为 66.983 / 66.963 s，均慢于正式基线；Nav2 精确收敛增加的等待超过了首次对正收益，而且低速进入后仍须二次对正。该改动已完整回退。
- 曾将全程带载速度由 1.05 小幅提高到 1.10 m/s。定向 B+C 为 2/2、恢复0且放置有效，但单件 63.620 / 65.883 s，相对正式基线 63.308 / 60.199 s 没有可重复收益；总耗时由航点停稳、转向和动态相位主导。该改动已回退，避免用无收益的速度增量扩大高重心风险。

## 文件

- `navigation_pick_place_test.py`：最终代码快照。
- `costudio_mission_gateway.py`：CoStudio 顺序同步快照。
- `ab_global_order_333s.log`：A/B 五物块成功证据。
- `c_dual_route_south_62s.log`：C 南路单件成功证据。
- `c_dual_route_two_items_exit_eta_176s.log`：连续 C 进入、放置、退出、再进入成功证据。
- `bc_global_c_eta_369s.log`：B/C 五物块完整混合回归证据。
- `c_two_handoff_eta_155s.log`：严格航点开销校准后的连续两件 C 证据。
- `bc_handoff_eta_345s.log`：当前 B/C 五物块完整终验证据。
- `c_two_nearest_late_lock_margin6_obstacle2_050_20260925.log`：近端锁存、北路切换裕量和障碍2提速后的连续两件 C 证据。
- `bc_full_late_lock_margin6_obstacle2_050_20260925.log`：2026-09-25 B/C 五物块 343.571 s 完整证据。
- `ab_segmented_speed_rejected.log`、`competition_segmented_speed_rejected.log`：提速实验被回退的证据。
- `bc_strict_final_heading_rejected.log`：严格最终航向交接被回退的证据。
- `bc_carried_110_rejected.log`：1.10 m/s 带载速度无有效收益并回退的证据。

## 约束保持

- 未修改地图、墙壁、石块、移动障碍物轨迹或全局安全距离；仅将下半区障碍2沿既有轨迹的速度改为 `0.50 m/s`，上半区障碍保持不变。
- 未取消最终夹取 `0.60 m/s` 减速、两段式放置、三维库存校验或 Collision Monitor。
- 每轮均通过单 Gazebo、单 RViz、Nav2 和两张地图启动门禁后才发布任务。

## 2026-09-27 A/B 289.082 s accepted regression

- No world, map, obstacle geometry, zone, cube coordinate, or safety-clearance file was changed.
- Merged two redundant stop/replan handoffs around the verified A doorway and west-rail route for red3/red4.
- Raised only the already-aligned final straight dropoff approach cap from 0.13 to 0.16 m/s; sharp approaches still rotate in place and all placement checks remain enabled.
- The fastest-mission cost now prices the executed west-rail carried route and its two handoffs instead of using an impossible Euclidean chord to A. This selected `blue5, blue4, red2, red1, red4` and avoided one full rail round trip.
- Accepted full regression: 5/5, recovery 0, 289.082 s. Per-item times: 44.666 / 48.838 / 60.674 / 67.566 / 67.336 s.
- Primary evidence: `ab_full_routecost_20260927.log`; directed evidence: `a_scheduler_routecost_red2_next_20260927.log`, `a_fineapproach016_red2_20260927.log`, and `b_fineapproach016_blue5_20260927.log`.
- Zero-change repeat: 5/5, recovery 0, 294.170 s with the same selected order. The two accepted full runs are 289.082 s and 294.170 s (mean 291.626 s). Repeat evidence: `ab_full_routecost_repeat_20260927.log`.
