#!/usr/bin/env python3
import sys
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def main():
    rclpy.init()
    nav = BasicNavigator()

    # 等待Nav2完全启动
    nav.waitUntilNav2Active()

    # 从命令行获取目标点坐标
    if len(sys.argv) >= 3:
        try:
            x = float(sys.argv[1])
            y = float(sys.argv[2])
        except ValueError:
            nav.get_logger().error("坐标参数错误，请输入数字，例如: ros2 run bot_navigation goto_goal.py 2.0 1.5")
            rclpy.shutdown()
            return
    else:
        nav.get_logger().info("未指定坐标，使用默认点 (1.0, 1.0)")
        x, y = 1.0, 1.0

    # 设置目标点
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.orientation.w = 1.0

    # 发送目标点
    nav.goToPose(goal_pose)
    nav.get_logger().info(f"开始导航到目标点: x={x}, y={y}")

    # 等待任务完成
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            nav.get_logger().info('导航中...')
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600):
            nav.cancelTask()
            nav.get_logger().warn('导航超时，任务取消')
            break

    # 结果输出
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        nav.get_logger().info('✅ 导航结果：成功')
    elif result == TaskResult.CANCELED:
        nav.get_logger().warn('⚠️ 导航结果：被取消')
    elif result == TaskResult.FAILED:
        nav.get_logger().error('❌ 导航结果：失败')
    else:
        nav.get_logger().error('导航结果：返回状态无效')

    rclpy.shutdown()


if __name__ == '__main__':
    main()

