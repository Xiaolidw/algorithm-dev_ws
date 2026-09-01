#!/usr/bin/env python3
"""Send one named or explicit map-frame goal to Nav2 and wait for its result.

All map-specific positions come from competition_v1_semantics.yaml.  A cargo
or area that has no explicit approach_pose is approached using its configured
reference offset; validate those generated poses before recording a final value.
"""

import argparse
import math
import sys
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


def load_yaml(path: Path) -> dict:
    try:
        with path.open(encoding='utf-8') as handle:
            data = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f'Cannot read YAML {path}: {error}') from error
    if not isinstance(data, dict):
        raise ValueError(f'YAML root must be a mapping: {path}')
    return data


def pose_from_values(values, label: str):
    if not isinstance(values, list) or len(values) != 3:
        raise ValueError(f'{label} must be [x, y, yaw], got {values!r}')
    return tuple(float(value) for value in values)


def generated_approach(center, offset: float, axis: str, label: str):
    x, y, _ = pose_from_values(center, label)
    shifts = {
        'negative_x': (-offset, 0.0), 'positive_x': (offset, 0.0),
        'negative_y': (0.0, -offset), 'positive_y': (0.0, offset),
    }
    if axis not in shifts:
        raise ValueError(f'Unsupported approach_policy.reference_axis: {axis!r}')
    dx, dy = shifts[axis]
    approach_x, approach_y = x + dx, y + dy
    return approach_x, approach_y, math.atan2(y - approach_y, x - approach_x)


def resolve_named_pose(name: str, semantic: dict):
    normalized = name.strip()
    policy = semantic.get('approach_policy', {})
    offset = float(policy.get('reference_offset_m', 0.55))
    axis = str(policy.get('reference_axis', 'negative_x'))

    for color, objects in semantic.get('cargo', {}).items():
        for item in objects or []:
            if item.get('id') == normalized:
                saved = item.get('search_pose')
                if saved is not None:
                    return (*pose_from_values(saved, f'cargo.{color}.{normalized}.search_pose'), 'configured')
                x, y, yaw = generated_approach(item.get('object_pose'), offset, axis,
                                                f'cargo.{color}.{normalized}.object_pose')
                return x, y, yaw, 'generated'

    area_name = normalized.removeprefix('area_').upper()
    area = semantic.get('placement_areas', {}).get(area_name)
    if area is not None:
        saved = area.get('approach_pose')
        if saved is not None:
            return (*pose_from_values(saved, f'placement_areas.{area_name}.approach_pose'), 'configured')
        x, y, yaw = generated_approach(area.get('center'), offset, axis,
                                        f'placement_areas.{area_name}.center')
        return x, y, yaw, 'generated'

    known = []
    for objects in semantic.get('cargo', {}).values():
        known.extend(item.get('id', '') for item in objects or [])
    known.extend(f'area_{key}' for key in semantic.get('placement_areas', {}))
    raise ValueError(f'Unknown destination {name!r}. Available: {", ".join(known)}')


class NamedNavGoal(Node):
    def __init__(self, nav_action: str):
        super().__init__('named_nav_goal')
        self._amcl_pose = None
        # AMCL publishes its latest pose with TRANSIENT_LOCAL durability.  A
        # one-shot CLI client must use the same durability or a stationary robot
        # may never publish another pose after this process starts.
        amcl_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, amcl_qos)
        self._client = ActionClient(self, NavigateToPose, nav_action)

    def _on_amcl(self, message):
        self._amcl_pose = message

    def wait_for_amcl(self, timeout_s: float) -> bool:
        deadline = self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while rclpy.ok() and self._amcl_pose is None:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.get_clock().now().nanoseconds >= deadline:
                return False
        return self._amcl_pose is not None

    def navigate(self, x: float, y: float, yaw: float, server_timeout: float,
                 goal_timeout: float) -> int:
        if not self._client.wait_for_server(timeout_sec=server_timeout):
            self.get_logger().error('Nav2 action server is unavailable.')
            return 2

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.get_logger().info(f'Sending goal: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}')

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=server_timeout)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the goal.')
            return 3

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=goal_timeout)
        if not result_future.done():
            self.get_logger().error(f'Goal timed out after {goal_timeout:.1f} s; cancel it in RViz/Nav2.')
            return 4
        status = result_future.result().status
        status_name = STATUS_NAMES.get(status, f'STATUS_{status}')
        self.get_logger().info(f'Navigation result: {status_name}')
        return 0 if status == GoalStatus.STATUS_SUCCEEDED else 5


def main():
    share = Path(get_package_share_directory('moon_warehouse_bringup'))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', nargs='?', help='Cargo ID (for example red_cube_1) or area_A/B/C')
    parser.add_argument('--pose', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                        help='Explicit map-frame goal; bypasses semantic-name lookup')
    parser.add_argument('--list', action='store_true', help='List configured cargo and areas, then exit')
    parser.add_argument('--dry-run', action='store_true', help='Resolve and validate only; do not send Nav2 goal')
    parser.add_argument('--semantic-config', type=Path,
                        default=share / 'config' / 'competition_v2_semantics.yaml')
    parser.add_argument('--client-config', type=Path,
                        default=share / 'config' / 'named_nav_goal_v2.yaml')
    args = parser.parse_args()

    try:
        semantic = load_yaml(args.semantic_config)
        client_config = load_yaml(args.client_config)
        if args.list:
            for color, objects in semantic.get('cargo', {}).items():
                for item in objects or []:
                    print(f"{item['id']} ({color})")
            for area in semantic.get('placement_areas', {}):
                print(f'area_{area}')
            return 0
        if bool(args.destination) == bool(args.pose):
            raise ValueError('Specify exactly one destination name or --pose X Y YAW.')
        if args.pose:
            x, y, yaw = args.pose
            source = 'explicit'
        else:
            x, y, yaw, source = resolve_named_pose(args.destination, semantic)
        bounds = client_config['map_bounds']
        if not (float(bounds['min_x']) <= x <= float(bounds['max_x']) and
                float(bounds['min_y']) <= y <= float(bounds['max_y'])):
            raise ValueError(f'Goal ({x:.3f}, {y:.3f}) is outside YAML map_bounds.')
        print(f'Resolved {args.destination or "explicit pose"}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} ({source})')
        if args.dry_run:
            return 0
    except (KeyError, ValueError) as error:
        print(f'ERROR: {error}', file=sys.stderr)
        return 1

    rclpy.init()
    node = NamedNavGoal(str(client_config['nav_action']))
    try:
        if bool(client_config.get('require_amcl_pose', True)) and not node.wait_for_amcl(
                float(client_config.get('amcl_wait_timeout_s', 15.0))):
            node.get_logger().error('AMCL pose was not received; set the initial pose first.')
            return 6
        return node.navigate(x, y, yaw, float(client_config.get('server_wait_timeout_s', 30.0)),
                             float(client_config.get('goal_timeout_s', 180.0)))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
