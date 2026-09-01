#!/bin/bash
# ============== 配置参数（适配yzbot路径） ==============
MAX_MEM_USAGE=150          # 内存超过150MB则重启（单位：MB）
CHECK_INTERVAL=15          # 每15秒检查一次内存
FORCE_RESTART_INTERVAL=30  # 每30秒强制重启一次（不管内存）
# 实际launch文件路径（根据yzbot结构调整）
ROSBRIDGE_LAUNCH_PATH="$HOME/dev_ws/src/yzbot/mybot/launch/rosbridge_custom_launch.xml"
ROSBRIDGE_PKG="mybot"     # launch文件所属的功能包（mybot在yzbot下）

# ============== 初始化变量 ==============
last_restart_time=$(date +%s)  # 记录上次重启时间（秒级时间戳）

# ============== 循环检查与重启逻辑 ==============
while true; do
    # 1. 检查rosbridge是否在运行，获取PID
    ROSBRIDGE_PID=$(pgrep -f "rosbridge_websocket")
    current_time=$(date +%s)
    time_since_last_restart=$((current_time - last_restart_time))

    # 2. 情况1：rosbridge未运行 → 立即启动
    if [ -z "$ROSBRIDGE_PID" ]; then
        echo "[$(date +'%Y-%m-%d %H:%M:%S')] rosbridge未运行，启动..."
        ros2 launch $ROSBRIDGE_PKG rosbridge_custom_launch.xml &
        last_restart_time=$current_time  # 更新重启时间
        sleep 5  # 等待启动完成
        continue
    fi

    # 3. 情况2：计算当前内存占用（RSS字段，转换为MB）
    MEM_USAGE=$(ps -p $ROSBRIDGE_PID -o rss --no-headers | awk '{print int($1/1024)}')
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] 当前内存：${MEM_USAGE}MB | 距离上次重启：${time_since_last_restart}秒"

    # 4. 触发重启的两个条件（满足其一即重启）
    need_restart=0
    if [ $MEM_USAGE -gt $MAX_MEM_USAGE ]; then
        echo "→ 内存超过${MAX_MEM_USAGE}MB，触发重启"
        need_restart=1
    elif [ $time_since_last_restart -ge $FORCE_RESTART_INTERVAL ]; then
        echo "→ 已超过${FORCE_RESTART_INTERVAL}秒，强制重启"
        need_restart=1
    fi

    # 5. 执行重启（关闭旧进程+启动新进程，强制清空缓存）
    if [ $need_restart -eq 1 ]; then
        # 先关闭旧rosbridge（两种方式确保关闭）
        ros2 node kill /rosbridge_websocket 2>/dev/null
        kill -9 $ROSBRIDGE_PID 2>/dev/null
        sleep 2  # 等待资源释放

        # 启动新rosbridge（自动加载清理后的配置）
        echo "→ 重启rosbridge..."
        ros2 launch $ROSBRIDGE_PKG rosbridge_custom_launch.xml &
        last_restart_time=$current_time  # 更新重启时间
        sleep 5  # 等待启动完成
    fi

    # 6. 等待下一次检查
    sleep $CHECK_INTERVAL
done
