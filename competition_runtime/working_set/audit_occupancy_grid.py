#!/usr/bin/env python3
import collections
import sys

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/cube_obstacle_map'
    rclpy.init()
    node = rclpy.create_node('occupancy_grid_audit')
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    result = []

    def callback(msg):
        result.append(msg)

    node.create_subscription(OccupancyGrid, topic, callback, qos)
    deadline = node.get_clock().now().nanoseconds + 8_000_000_000
    while not result and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not result:
        raise RuntimeError(f'No map received from {topic}')
    msg = result[0]
    width, height = msg.info.width, msg.info.height
    occupied = {index for index, value in enumerate(msg.data) if value >= 65}
    components = []
    while occupied:
        seed = occupied.pop()
        queue = collections.deque([seed])
        cells = [seed]
        while queue:
            index = queue.popleft()
            x, y = index % width, index // width
            for nx, ny in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
                neighbor = ny * width + nx
                if 0 <= nx < width and 0 <= ny < height and neighbor in occupied:
                    occupied.remove(neighbor)
                    queue.append(neighbor)
                    cells.append(neighbor)
        components.append(cells)
    resolution = msg.info.resolution
    ox = msg.info.origin.position.x
    oy = msg.info.origin.position.y
    print(f'topic={topic} components={len(components)}')
    for cells in sorted(components, key=len, reverse=True):
        xs = [index % width for index in cells]
        ys = [index // width for index in cells]
        cx = ox + (sum(xs) / len(xs) + 0.5) * resolution
        cy = oy + (sum(ys) / len(ys) + 0.5) * resolution
        min_x, max_x = ox + min(xs)*resolution, ox + (max(xs)+1)*resolution
        min_y, max_y = oy + min(ys)*resolution, oy + (max(ys)+1)*resolution
        print(f'cells={len(cells):4d} center=({cx:7.3f},{cy:7.3f}) '
              f'bbox=({min_x:7.3f},{min_y:7.3f})..({max_x:7.3f},{max_y:7.3f})')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
