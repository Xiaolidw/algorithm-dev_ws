#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

rclpy.init()
node = Node('exclude_calibration_cube')
qos = QoSProfile(depth=1)
qos.reliability = ReliabilityPolicy.RELIABLE
qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
publisher = node.create_publisher(String, '/mission/execution_status', qos)
message = String(data=json.dumps({
    'state': 'CALIBRATING_GRASP',
    'current_task': {'object_id': 'red_cube_1', 'destination': 'A'},
}))
for _ in range(20):
    publisher.publish(message)
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.05)
node.destroy_node()
rclpy.shutdown()
