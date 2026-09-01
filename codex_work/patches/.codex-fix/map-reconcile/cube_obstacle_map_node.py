#!/usr/bin/env python3
"""Publish the known cargo-cube positions as a latched obstacle map.

The 2D lidar plane (0.125 m) passes above the 0.105 m cube tops, so the
cargo cubes are invisible to the scan-based costmap layers and the robot
can drive straight through them.  The cube positions are fixed by the
world file, so instead of mounting hardware lower we inject them here as
a virtual obstacle layer that both costmaps consume.

发布 /cube_obstacle_map: 与静态地图同元数据的占据栅格, 仅在货物方块
位置标记致死障碍。全局与局部代价地图各挂一层 StaticLayer 消费该话题,
使全局规划绕开物块、局部控制器实施实时防碾压。
"""

import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import OccupancyGrid


class CubeObstacleMapNode(Node):

    def __init__(self):
        super().__init__('cube_obstacle_map_node')

        # 与 competition_v2_map.pgm 完全一致的元数据
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_origin_x', -15.6)
        self.declare_parameter('map_origin_y', -33.4)
        self.declare_parameter('map_width', 658)
        self.declare_parameter('map_height', 822)
        # 方块世界坐标(与 world 文件一致): red y=3.9 / blue y=1.7
        self.declare_parameter(
            'cube_positions',
            ['red_cube_1:-8.35:3.9', 'red_cube_2:-6.15:3.9',
             'red_cube_3:-3.95:3.9', 'red_cube_4:-1.75:3.9',
             'red_cube_5:0.45:3.9', 'blue_cube_1:-8.35:1.7',
             'blue_cube_2:-6.15:1.7', 'blue_cube_3:-3.95:1.7',
             'blue_cube_4:-1.75:1.7', 'blue_cube_5:0.45:1.7'])
        # 标记半径: 略大于方块半宽0.045, 保证覆盖2~3个栅格
        self.declare_parameter('mark_radius', 0.07)

        res = float(self.get_parameter('map_resolution').value)
        origin_x = float(self.get_parameter('map_origin_x').value)
        origin_y = float(self.get_parameter('map_origin_y').value)
        width = int(self.get_parameter('map_width').value)
        height = int(self.get_parameter('map_height').value)
        radius = float(self.get_parameter('mark_radius').value)

        self.resolution = res
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.width = width
        self.height = height
        self.radius = radius
        self.cube_positions = {}
        for spec in self.get_parameter('cube_positions').value:
            try:
                name, xs, ys = str(spec).split(':')
                self.cube_positions[name] = (float(xs), float(ys))
            except ValueError:
                self.get_logger().warning(f'Bad cube spec: {spec!r}')
        self.carried_cubes = set()
        self.grid = self.build_grid()

        # latched 语义: 瞬态本地, 晚加入的代价地图也能拿到
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        self.publisher = self.create_publisher(
            OccupancyGrid,
            '/cube_obstacle_map',
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        # Track Gazebo truth so a cube remains protected if it is nudged, but
        # disappears from the floor map while it is carried by the arm.
        self.create_subscription(
            ModelStates, '/gazebo/model_states', self.model_states_callback, 10)

        # 周期性重发, 保证晚启动的 costmap 与重连场景都能收到
        self.timer = self.create_timer(2.0, self.publish_map)
        self.publish_map()
        self.get_logger().info(
            f'Cube obstacle map published for {len(self.cube_positions)} cubes'
        )

    def model_states_callback(self, message):
        changed = False
        current_carried = set()
        for name, pose in zip(message.name, message.pose):
            if name not in self.cube_positions:
                continue
            # A picked cube is lifted well above its 1.5 cm floor pose. It is
            # then part of the robot footprint, not a floor obstacle.
            if pose.position.z > 0.12:
                current_carried.add(name)
                continue
            position = (pose.position.x, pose.position.y)
            if any(abs(a - b) > 0.01 for a, b in zip(
                    position, self.cube_positions[name])):
                self.cube_positions[name] = position
                changed = True
        if current_carried != self.carried_cubes:
            self.carried_cubes = current_carried
            changed = True
        if changed:
            self.grid = self.build_grid()
            self.publish_map()

    def build_grid(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (self.width * self.height)
        cells = int(math.ceil(self.radius / self.resolution))
        for name, (cx, cy) in self.cube_positions.items():
            if name in self.carried_cubes:
                continue
            pcx = int((cx - self.origin_x) / self.resolution)
            pcy = self.height - int((cy - self.origin_y) / self.resolution) - 1
            for dy in range(-cells, cells + 1):
                for dx in range(-cells, cells + 1):
                    if dx * dx + dy * dy > (self.radius / self.resolution) ** 2:
                        continue
                    px, py = pcx + dx, pcy + dy
                    if 0 <= px < self.width and 0 <= py < self.height:
                        grid.data[py * self.width + px] = 100
        return grid

    def publish_map(self):
        from builtin_interfaces.msg import Time
        self.grid.header.stamp = Time()
        self.publisher.publish(self.grid)


def main(args=None):
    rclpy.init(args=args)
    node = CubeObstacleMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
