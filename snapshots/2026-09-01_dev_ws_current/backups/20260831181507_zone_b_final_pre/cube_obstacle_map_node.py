#!/usr/bin/env python3
"""Publish the cargo-cube positions as a latched virtual obstacle map.

The 2D lidar plane (0.125 m) passes above the 0.105 m cube tops, so the
cargo cubes are invisible to the scan-based costmap layers and the robot
would drive straight through them.  This node injects them as a virtual
obstacle layer for both costmaps.

Two exclusions keep the layer compatible with the grasp geometry (the
approach pose stands only 0.58 m from the target cube, well inside any
inflation halo):

1. Carried cubes: Gazebo truth lifts a welded cube above 0.12 m; it then
   travels with the robot and must not be marked on the floor.
2. Current mission target: the executor's /mission/execution_status
   reports the object being approached; it stays unmarked from navigation
   start until the leg completes so the approach is never blocked by its
   own virtual halo.
3. Just-placed cube: it remains temporarily unmarked until the chassis is
   outside the complete planning-footprint clearance.  The raw safety scan
   still sees the cube during this short handoff, so it cannot be driven
   through, while Nav2 is not initialized with its footprint already inside
   a newly restored lethal cell.
"""

import json
import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


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
        # 抬升判据: 焊接后方块随臂抬高到 0.12m 以上
        self.declare_parameter('carried_z_threshold', 0.12)
        # spawn_entity.py creates the competition robot as "six_arm".  Using
        # the retired "originbot" name leaves robot_xy unset, so a just-placed
        # cube can never be restored after the 0.70 m handoff clearance.
        self.declare_parameter('robot_model_name', 'six_arm')
        self.declare_parameter('placed_cube_release_clearance', 0.70)
        # world 中真实存在的 1m x 1m 固定石块。原始 PGM 将它们画成灰色
        # (205)，按当前 map_server 阈值会被解释为空闲栅格，导致全局规划
        # 穿过石块、局部规划只能原地堵停。这里用世界真值补成占用栅格。
        self.declare_parameter(
            'fixed_obstacles',
            ['stone_0:-11.7361:10.376:0.55:0.55',
             'stone_1:-9.86873:4.59948:0.55:0.55',
             'stone_2:0.764529:6.87658:0.55:0.55',
             'stone_3:14.5365:7.25535:0.55:0.55',
             'stone_4:-3.6643:2.16941:0.55:0.55',
             'stone_5:5.45574:2.04349:0.55:0.55',
             'stone_6:7.19325:-1.71375:0.55:0.55',
             'stone_7:0.605863:-2.01965:0.55:0.55',
             'stone_8:-14.4058:-2.10305:0.55:0.55',
             'stone_9:-11.6359:-5.34072:0.55:0.55',
             'stone_10:11.7248:-5.10078:0.55:0.55',
             'stone_11:-3.47257:-8.45059:0.55:0.55',
             'stone_12:14.5027:-8.59174:0.55:0.55'])
        # 放置区(A/B/C 标记牌 1.0 x 0.5m): 整区视为障碍, 返回/经过时
        # 全局路径会绕开, 不会碾压区内已放置的物块。当前任务的目的地
        # 区域临时放行, 保证本任务可以靠近放置。
        self.declare_parameter(
            'zone_areas',
            # Gazebo competition_v2_scoring.world truth.  The old A value
            # (-13.2,-7.6) belonged to a retired layout and made destination
            # exclusions operate on empty floor.
            ['zone_a:-7.0:-3.0', 'zone_b:-1.5:-4.5',
             'zone_c:11.0:-2.2'])
        self.declare_parameter('zone_half_width', 0.55)
        self.declare_parameter('zone_half_depth', 0.30)
        # Placement signs are floor markings, not physical obstacles.  Marking
        # an entire zone through a max-combined StaticLayer leaves stale lethal
        # cells after the current destination is "released".  Grounded cubes
        # inside a completed zone are already marked individually above.
        self.declare_parameter('mark_zone_areas', False)

        self.resolution = float(self.get_parameter('map_resolution').value)
        self.origin_x = float(self.get_parameter('map_origin_x').value)
        self.origin_y = float(self.get_parameter('map_origin_y').value)
        self.width = int(self.get_parameter('map_width').value)
        self.height = int(self.get_parameter('map_height').value)
        self.radius = float(self.get_parameter('mark_radius').value)
        self.carried_z = float(self.get_parameter('carried_z_threshold').value)
        self.robot_model_name = str(
            self.get_parameter('robot_model_name').value)
        self.placed_cube_release_clearance = max(
            0.55,
            float(self.get_parameter(
                'placed_cube_release_clearance').value),
        )

        self.fixed_obstacles = {}
        for spec in self.get_parameter('fixed_obstacles').value:
            try:
                name, xs, ys, hws, hds = str(spec).split(':')
                self.fixed_obstacles[name] = (
                    float(xs), float(ys), float(hws), float(hds))
            except ValueError:
                self.get_logger().warning(f'Bad fixed-obstacle spec: {spec!r}')

        self.cube_positions = {}
        for spec in self.get_parameter('cube_positions').value:
            try:
                name, xs, ys = str(spec).split(':')
                self.cube_positions[name] = (float(xs), float(ys))
            except ValueError:
                self.get_logger().warning(f'Bad cube spec: {spec!r}')

        self.zone_positions = {}
        for spec in self.get_parameter('zone_areas').value:
            try:
                name, xs, ys = str(spec).split(':')
                self.zone_positions[name] = (float(xs), float(ys))
            except ValueError:
                self.get_logger().warning(f'Bad zone spec: {spec!r}')
        self.zone_hw = float(self.get_parameter('zone_half_width').value)
        self.zone_hd = float(self.get_parameter('zone_half_depth').value)
        self.mark_zones = bool(self.get_parameter('mark_zone_areas').value)

        self.carried_cubes = set()
        self.active_target_cubes = set()
        self.transient_excluded_cubes = set()
        self.completed_exclusion_keys = set()
        self.robot_xy = None
        self.mission_excluded_cubes = set()
        self.mission_excluded_zones = set()
        self.active_dropoff_zone = None
        self.grid = self.build_grid()

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            OccupancyGrid, '/cube_obstacle_map', status_qos)

        # 被夹起的方块随臂抬升, 不再是地面障碍; 被碰撞挪动的方块用真值
        # 位置保持保护。
        self.create_subscription(
            ModelStates, '/gazebo/model_states',
            self.model_states_callback, 10)

        # 当前任务方块在接近/运送阶段从虚拟层摘除, 放置完成后恢复标记。
        self.create_subscription(
            String, '/mission/execution_status',
            self.execution_status_callback, status_qos)

        # 周期性重发, 保证晚启动的 costmap 也能收到
        self.timer = self.create_timer(2.0, self.publish_map)
        self.publish_map()
        self.get_logger().info(
            f'Obstacle map published for {len(self.cube_positions)} cubes, '
            f'{len(self.fixed_obstacles)} fixed stones and '
            f'{len(self.zone_positions)} placement zones')

    def model_states_callback(self, message):
        changed = False
        current_carried = set()
        for name, pose in zip(message.name, message.pose):
            if name == self.robot_model_name:
                self.robot_xy = (
                    float(pose.position.x), float(pose.position.y))
                continue
            if name not in self.cube_positions:
                continue
            if pose.position.z > self.carried_z:
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
        released = set()
        if self.robot_xy is not None:
            for name in self.transient_excluded_cubes:
                cube_xy = self.cube_positions.get(name)
                if cube_xy is None:
                    released.add(name)
                    continue
                if math.hypot(
                    self.robot_xy[0] - cube_xy[0],
                    self.robot_xy[1] - cube_xy[1],
                ) >= self.placed_cube_release_clearance:
                    released.add(name)
        if released:
            self.transient_excluded_cubes.difference_update(released)
            self.get_logger().info(
                'Restored placed cube obstacle(s) after footprint clearance: '
                f'{sorted(released)}')
            changed = True
        if changed:
            self.refresh_mission_exclusions()
            self.grid = self.build_grid()
            self.publish_map()

    def refresh_mission_exclusions(self):
        """Combine the active target with post-place handoff exclusions."""
        self.mission_excluded_cubes = (
            set(self.active_target_cubes)
            | set(self.transient_excluded_cubes)
        )

    def execution_status_callback(self, message):
        try:
            payload = json.loads(message.data)
            task = payload.get('current_task') or {}
            object_id = str(task.get('object_id', '') or '')
            destination = str(task.get('destination', '') or '').upper()
            phase = str(payload.get('current_phase', '') or '').upper()
            state = str(payload.get('state', '') or '').upper()
            active = bool(payload.get('active', False))
            task_index = int(payload.get('current_task_index', -1))
            completed_candidates = []
            for index, candidate in enumerate(payload.get('tasks') or []):
                if not isinstance(candidate, dict):
                    continue
                candidate_object = str(
                    candidate.get('object_id', '') or '')
                candidate_state = str(
                    candidate.get('execution_status', '') or '').upper()
                if (
                    candidate_state == 'COMPLETED'
                    and candidate_object in self.cube_positions
                ):
                    completed_candidates.append((index, candidate_object))
        except (json.JSONDecodeError, TypeError, ValueError):
            return
        active_target_cubes = (
            {object_id} if object_id in self.cube_positions else set())
        # TASK_COMPLETED is immediately followed by the next navigation state.
        # With depth=1 a subscriber may legitimately receive only the newer
        # frame, so derive handoffs from the persistent tasks[] records instead
        # of relying on that one transient state message.
        if state == 'TASK_COMPLETED' and object_id in self.cube_positions:
            completed_candidates.append((task_index, object_id))
        for completion_key in completed_candidates:
            if completion_key in self.completed_exclusion_keys:
                continue
            self.completed_exclusion_keys.add(completion_key)
            completed_object = completion_key[1]
            cube_xy = self.cube_positions.get(completed_object)
            clearance = (
                math.hypot(
                    self.robot_xy[0] - cube_xy[0],
                    self.robot_xy[1] - cube_xy[1],
                )
                if self.robot_xy is not None and cube_xy is not None
                else 0.0
            )
            if clearance < self.placed_cube_release_clearance:
                self.transient_excluded_cubes.add(completed_object)
                self.get_logger().info(
                    f'Holding {completed_object} out of the virtual layer at '
                    f'{clearance:.2f} m until '
                    f'{self.placed_cube_release_clearance:.2f} m clearance')
        if not active and state in {
            'IDLE', 'STOPPED', 'ERROR', 'COMPLETE', 'COMPLETED'
        }:
            self.transient_excluded_cubes.clear()
            self.completed_exclusion_keys.clear()
        self.active_target_cubes = active_target_cubes
        excluded_cubes = (
            active_target_cubes | self.transient_excluded_cubes)
        excluded_zones = (
            {'zone_' + destination.lower()}
            if 'zone_' + destination.lower() in self.zone_positions
            else set())
        destination_zone = next(iter(excluded_zones), None)
        dropoff_active = (
            phase == 'DROPOFF'
            or state in {
                'NAVIGATING_DROPOFF', 'DROPOFF_REACHED', 'PLACING',
                'DEPARTING_DROPOFF',
            }
        )
        active_dropoff_zone = destination_zone if dropoff_active else None
        if (excluded_cubes == self.mission_excluded_cubes
                and excluded_zones == self.mission_excluded_zones
                and active_dropoff_zone == self.active_dropoff_zone):
            return
        self.mission_excluded_cubes = excluded_cubes
        self.mission_excluded_zones = excluded_zones
        self.active_dropoff_zone = active_dropoff_zone
        self.grid = self.build_grid()
        self.publish_map()
        self.get_logger().info(
            f'Obstacle exclusion -> cubes={sorted(self.mission_excluded_cubes)} '
            f'zones={sorted(self.mission_excluded_zones)} '
            f'active_dropoff_zone={self.active_dropoff_zone}')

    def cube_is_in_zone(self, cx, cy, zone_name):
        zone = self.zone_positions.get(zone_name)
        if zone is None:
            return False
        return (
            abs(cx - zone[0]) <= self.zone_hw
            and abs(cy - zone[1]) <= self.zone_hd
        )

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
            # OccupancyGrid 数据第0行对应 origin_y(南端), 行号随 y 增大
            if name in self.carried_cubes or name in self.mission_excluded_cubes:
                continue
            # Completed cargo remains a protected obstacle on every ordinary
            # route.  During the final approach to that same controlled
            # storage bay only, its virtual inflation halo is suppressed: the
            # arm places from outside the sign into a different verified slot,
            # so the chassis does not drive over the cube, while successive
            # deliveries are no longer blocked half a metre from one shared
            # endpoint by a 3 cm object already stored where it belongs.
            if (self.active_dropoff_zone is not None
                    and self.cube_is_in_zone(
                        cx, cy, self.active_dropoff_zone)):
                continue
            pcx = int((cx - self.origin_x) / self.resolution)
            pcy = int((cy - self.origin_y) / self.resolution)
            for dy in range(-cells, cells + 1):
                for dx in range(-cells, cells + 1):
                    if dx * dx + dy * dy > (self.radius / self.resolution) ** 2:
                        continue
                    px, py = pcx + dx, pcy + dy
                    if 0 <= px < self.width and 0 <= py < self.height:
                        grid.data[py * self.width + px] = 100
        # 固定石块始终占用，不随当前任务目标而放行。0.55m 半宽已包含
        # 一个 5cm 栅格的模型/地图离散误差，costmap 再负责机器人外形膨胀。
        for _name, (cx, cy, half_width, half_depth) in (
                self.fixed_obstacles.items()):
            pcx = int((cx - self.origin_x) / self.resolution)
            pcy = int((cy - self.origin_y) / self.resolution)
            cells_x = int(math.ceil(half_width / self.resolution))
            cells_y = int(math.ceil(half_depth / self.resolution))
            for dy in range(-cells_y, cells_y + 1):
                for dx in range(-cells_x, cells_x + 1):
                    px, py = pcx + dx, pcy + dy
                    if 0 <= px < self.width and 0 <= py < self.height:
                        grid.data[py * self.width + px] = 100
        # Optional only for legacy layouts.  Competition v2 keeps this false:
        # placed cargo itself protects an occupied zone without turning its
        # traversable floor sign into a permanent wall.
        if self.mark_zones:
            zone_cells_x = int(self.zone_hw / self.resolution)
            zone_cells_y = int(self.zone_hd / self.resolution)
            for name, (cx, cy) in self.zone_positions.items():
                if name in self.mission_excluded_zones:
                    continue
                pcx = int((cx - self.origin_x) / self.resolution)
                pcy = int((cy - self.origin_y) / self.resolution)
                for dy in range(-zone_cells_y, zone_cells_y + 1):
                    for dx in range(-zone_cells_x, zone_cells_x + 1):
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
