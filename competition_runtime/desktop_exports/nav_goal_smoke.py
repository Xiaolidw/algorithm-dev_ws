#!/usr/bin/env python3
import math
import sys

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


def main():
    x, y, yaw = map(float, sys.argv[1:4])
    rclpy.init()
    node = rclpy.create_node('nav_goal_smoke')
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    if not client.wait_for_server(timeout_sec=20.0):
        print('SERVER_UNAVAILABLE', flush=True)
        raise SystemExit(2)
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
    last_bucket = [-1]

    def feedback(message):
        distance = message.feedback.distance_remaining
        bucket = int(distance * 2.0)
        if bucket != last_bucket[0]:
            last_bucket[0] = bucket
            print(f'FEEDBACK distance={distance:.3f} recoveries={message.feedback.number_of_recoveries}', flush=True)

    send_future = client.send_goal_async(goal, feedback_callback=feedback)
    rclpy.spin_until_future_complete(node, send_future)
    handle = send_future.result()
    if handle is None or not handle.accepted:
        print('REJECTED', flush=True)
        raise SystemExit(3)
    print('ACCEPTED', flush=True)
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    wrapped = result_future.result()
    print(f'RESULT status={wrapped.status}', flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
