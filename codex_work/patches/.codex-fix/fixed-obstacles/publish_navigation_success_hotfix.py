#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


rclpy.init()
node = Node('navigation_success_hotfix')
qos = QoSProfile(depth=1)
qos.reliability = ReliabilityPolicy.RELIABLE
qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
publisher = node.create_publisher(String, '/mission/navigation_status', qos)
payload = {
    'action_name': '/navigate_to_pose',
    'active': False,
    'result': 'SUCCEEDED',
    'goal': {
        'purpose': 'MISSION_GOAL',
        'frame_id': 'map',
        'x': -13.2,
        'y': -7.1,
        'yaw': -1.5708,
    },
    'feedback': {'distance_remaining_m': 0.0},
}
message = String(data=json.dumps(payload))
for _ in range(10):
    publisher.publish(message)
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.05)
node.destroy_node()
rclpy.shutdown()
