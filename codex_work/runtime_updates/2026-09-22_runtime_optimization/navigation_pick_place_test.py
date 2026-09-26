#!/usr/bin/env python3
"""Navigate to a configured cube, pick it, navigate to a zone, and place it."""

import argparse
import json
import math
import os
import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import PoseStamped, Twist
from moon_warehouse_interfaces.action import ExecuteManipulation
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String


ZONE_MODELS = {'A': 'zone_a', 'B': 'zone_b', 'C': 'zone_c'}


class NavigationNoProgress(RuntimeError):
    """A Nav2 goal is alive but its route feedback has stopped changing."""


class ManipulationFailure(RuntimeError):
    """A manipulation action failed with a machine-readable result code."""

    def __init__(self, operation, error_code, message):
        super().__init__(
            f'{operation} failed: code={error_code}, message={message}')
        self.operation = operation
        self.error_code = int(error_code)
        self.result_message = str(message)


class PickPlaceTest(Node):
    """Blocking acceptance client built only on the project's public APIs."""

    def __init__(self, object_id, destination, nav_timeout, arm_timeout):
        super().__init__('navigation_pick_place_test')
        self.object_id = object_id
        self.destination = destination
        self.nav_timeout = nav_timeout
        self.arm_timeout = arm_timeout
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.arm_client = ActionClient(
            self, ExecuteManipulation, '/manipulation/execute')
        self.controller_parameters = self.create_client(
            SetParameters, '/controller_server/set_parameters')
        self.state_client = self.create_client(
            GetEntityState, '/gazebo/get_entity_state')
        # Fine docking shares the normal safety pipeline with Nav2.  Publishing
        # directly on /cmd_vel_nav competed with Collision Monitor's output;
        # after a TF hiccup its retained nonzero command could defeat our stop.
        self.dock_publisher = self.create_publisher(
            Twist, '/cmd_vel_nav_raw', 10)
        self.navigation_status_publisher = self.create_publisher(
            String, '/pick_place/navigation_status', 10)
        self.navigation_status = {
            'phase': 'INITIALIZING',
            'event': 'node_started',
            'label': '',
        }
        self._last_feedback_log = 0.0
        self._navigation_feedback_distance = None
        self._navigation_feedback_recoveries = 0
        self.base_velocity = None
        self.base_velocity_received = 0.0
        self.model_poses = {}
        self.model_twists = {}
        self.models_received = 0.0
        self.global_costmap = None
        self.selected_dropoff_slot = None
        self._c_crossing_mode = None
        self.slot_claim_path = '/tmp/moon_warehouse_slot_claims.json'
        costmap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.create_subscription(
            ModelStates, '/gazebo/model_states', self._models_callback, 10)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap',
            self._global_costmap_callback, costmap_qos)

    def _odom_callback(self, message):
        self.base_velocity = message.twist.twist
        self.base_velocity_received = time.monotonic()

    def _models_callback(self, message):
        self.model_poses = dict(zip(message.name, message.pose))
        self.model_twists = dict(zip(message.name, message.twist))
        self.models_received = time.monotonic()

    def _global_costmap_callback(self, message):
        self.global_costmap = message

    def _load_slot_claims(self):
        try:
            with open(self.slot_claim_path, 'r', encoding='utf-8') as stream:
                value = json.load(stream)
            return value if isinstance(value, dict) else {}
        except (OSError, ValueError, TypeError):
            return {}

    def _save_slot_claims(self, claims):
        temporary = self.slot_claim_path + '.tmp'
        try:
            with open(temporary, 'w', encoding='utf-8') as stream:
                json.dump(claims, stream, sort_keys=True)
            os.replace(temporary, self.slot_claim_path)
        except OSError as error:
            self.get_logger().warning(
                f'Cannot persist placement slot claims: {error}')

    def _global_costmap_cell(self, x, y):
        """Return the published occupancy value at a world point, if known."""
        grid = self.global_costmap
        if grid is None or not grid.data:
            return None
        resolution = float(grid.info.resolution)
        if resolution <= 0.0:
            return None
        column = math.floor((float(x) - grid.info.origin.position.x) / resolution)
        row = math.floor((float(y) - grid.info.origin.position.y) / resolution)
        if (column < 0 or row < 0 or column >= grid.info.width
                or row >= grid.info.height):
            return None
        return int(grid.data[row * grid.info.width + column])

    def _validate_navigation_goal_cost(self, label, x, y):
        """Refuse a Nav2 target already marked lethal by the live costmap."""
        # Give a transient-local sample a bounded opportunity to arrive in a
        # short-lived per-item process.
        deadline = time.monotonic() + 2.0
        while ((self.global_costmap is None
                or time.monotonic() - self.models_received > 0.35)
               and time.monotonic() < deadline):
            rclpy.spin_once(self, timeout_sec=0.05)
        # The global costmap is a large reliable sample and can briefly win
        # the single-threaded executor over /gazebo/model_states.  Require a
        # fresh pose after that sample, but allow one bounded publish cycle
        # on a loaded simulator instead of failing on the first gap.
        robot = self._await_fresh_robot_pose(timeout_sec=1.2)
        start_cost = self._global_costmap_cell(
            robot.position.x, robot.position.y)
        goal_cost = self._global_costmap_cell(x, y)
        self.get_logger().info(
            f'Global costmap probe for {label}: start_cost={start_cost}, '
            f'goal_cost={goal_cost}, goal=({x:.3f},{y:.3f})')
        if goal_cost is not None and goal_cost >= 99:
            raise RuntimeError(
                f'Refusing navigation to {label}: target lies in lethal '
                f'global costmap cell (cost={goal_cost})')

    def _fresh_robot_pose(self):
        if time.monotonic() - self.models_received > 0.35:
            raise RuntimeError('Gazebo model pose is stale; stopping base')
        if 'six_arm' not in self.model_poses:
            raise RuntimeError('six_arm missing from model states')
        return self.model_poses['six_arm']

    def _await_fresh_robot_pose(self, timeout_sec=0.8):
        """Wait briefly for Gazebo truth without accepting a stale sample."""
        deadline = time.monotonic() + float(timeout_sec)
        while (time.monotonic() - self.models_received > 0.35
               and time.monotonic() < deadline):
            rclpy.spin_once(self, timeout_sec=0.04)
        return self._fresh_robot_pose()

    def _dock_coordinates(self):
        robot = self._await_fresh_robot_pose()
        cube = self.model_poses.get(self.object_id)
        if cube is None:
            raise RuntimeError('Target object missing from model states')
        q = robot.orientation
        yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                         1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        dx = cube.position.x - robot.position.x
        dy = cube.position.y - robot.position.y
        return (math.cos(yaw)*dx + math.sin(yaw)*dy,
                -math.sin(yaw)*dx + math.cos(yaw)*dy)

    def publish_navigation_status(self, phase=None, event=None, **values):
        """Publish persistent structured phase and Nav2 feedback telemetry."""
        if phase is not None:
            self.navigation_status['phase'] = str(phase)
        if event is not None:
            self.navigation_status['event'] = str(event)
        self.navigation_status.update(values)
        self.navigation_status['stamp_s'] = (
            self.get_clock().now().nanoseconds / 1e9)
        self.navigation_status_publisher.publish(String(
            data=json.dumps(self.navigation_status, separators=(',', ':'))))

    def wait_for_interfaces(self):
        self.get_logger().info('Waiting for Nav2 and manipulation interfaces...')
        if not self.nav_client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError('/navigate_to_pose is unavailable')
        if not self.arm_client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError('/manipulation/execute is unavailable')
        if not self.state_client.wait_for_service(timeout_sec=20.0):
            raise RuntimeError('/gazebo/get_entity_state is unavailable')

    def wait_for_state_service(self):
        if not self.state_client.wait_for_service(timeout_sec=20.0):
            raise RuntimeError('/gazebo/get_entity_state is unavailable')

    def select_object(self, selector, approaches):
        """Select a requested cube from its live Gazebo pose."""
        selector = str(selector).lower()
        if selector in approaches:
            return selector
        race_selection = selector.startswith('fastest')
        if selector == 'nearest':
            candidates = list(approaches)
        elif selector in ('nearest-red', 'nearest-blue'):
            colour = selector.split('-', 1)[1] + '_cube_'
            candidates = [name for name in approaches if name.startswith(colour)]
        elif selector == 'fastest':
            candidates = list(approaches)
        elif selector in ('fastest-red', 'fastest-blue'):
            colour = selector.split('-', 1)[1] + '_cube_'
            candidates = [name for name in approaches if name.startswith(colour)]
        else:
            raise RuntimeError(
                f'Unknown object selector {selector!r}; use an object ID, '
                "'nearest[-red|-blue]', or 'fastest[-red|-blue]'.")
        robot = self._relative_pose('six_arm', 'world')
        zone = self._relative_pose(ZONE_MODELS[self.destination], 'world')
        zone_orientation = zone.orientation
        zone_yaw = math.atan2(
            2.0 * (zone_orientation.w * zone_orientation.z
                   + zone_orientation.x * zone_orientation.y),
            1.0 - 2.0 * (zone_orientation.y * zone_orientation.y
                         + zone_orientation.z * zone_orientation.z))
        cos_zone = math.cos(zone_yaw)
        sin_zone = math.sin(zone_yaw)
        # A legal return from A must first leave through its east doorway.
        # Ranking from the parking pose by a straight chord ignores the walls
        # and can prefer a longer route.  In that state, compare grasp docks
        # from the verified outer doorway; elsewhere compare them from the
        # live robot pose.
        ranking_x = float(robot.position.x)
        ranking_y = float(robot.position.y)
        robot_in_a = self._relative_pose('six_arm', 'zone_a')
        ranking_mode = 'live_robot_to_grasp_dock'
        if (self.destination == 'A'
                and math.hypot(robot_in_a.position.x,
                               robot_in_a.position.y) <= 1.80):
            ranking_x = -2.30
            ranking_y = 1.30
            ranking_mode = 'a_outer_doorway_to_grasp_dock'
        ranked = []
        for object_id in candidates:
            pose = self._relative_pose(object_id, 'world')
            zone_dx = pose.position.x - zone.position.x
            zone_dy = pose.position.y - zone.position.y
            object_zone_x = cos_zone * zone_dx + sin_zone * zone_dy
            object_zone_y = -sin_zone * zone_dx + cos_zone * zone_dy
            if (abs(object_zone_x) <= 0.50
                    and abs(object_zone_y) <= 0.25):
                self.get_logger().info(
                    f'Skipping {object_id}: already inside destination '
                    f'{self.destination}')
                continue
            configured_yaw = float(approaches[object_id]['yaw'])
            dock_points = [
                (float(pose.position.x) - 0.38 * math.cos(yaw),
                 float(pose.position.y) - 0.38 * math.sin(yaw))
                for yaw in (
                    configured_yaw,
                    configured_yaw + math.pi / 2.0,
                    configured_yaw + math.pi,
                    configured_yaw + 3.0 * math.pi / 2.0)]
            if object_id in ('red_cube_3', 'red_cube_4', 'red_cube_5'):
                # All cubes north of the rail require the immutable west-rail
                # bypass.  red_cube_3 was previously omitted and could remain
                # stopped indefinitely behind moving_obstacle_1.
                # Rank the route that will actually execute; Euclidean
                # ranking incorrectly chose red5 even though its final north
                # leg is 2.2 m longer and intermittently blocked by the mover.
                west_1 = (-2.75, 2.00)
                west_2 = (-2.75, 3.50)
                pickup_distance = (
                    math.hypot(west_1[0] - ranking_x,
                               west_1[1] - ranking_y)
                    + math.hypot(west_2[0] - west_1[0],
                                 west_2[1] - west_1[1])
                    + min(math.hypot(dx - west_2[0], dy - west_2[1])
                          for dx, dy in dock_points))
                candidate_mode = ranking_mode + '_via_west_rail'
            else:
                pickup_distance = min(
                    math.hypot(dx - ranking_x, dy - ranking_y)
                    for dx, dy in dock_points)
                candidate_mode = ranking_mode
            self.get_logger().info(
                f'Pickup candidate {object_id}: '
                f'route_rank={pickup_distance:.3f}m, mode={candidate_mode}')
            if not race_selection:
                ranked.append((pickup_distance, object_id, 0.0))
                continue
            carry_distance = math.hypot(
                pose.position.x - zone.position.x,
                pose.position.y - zone.position.y)
            if (self.destination == 'A'
                    and object_id in (
                        'red_cube_3', 'red_cube_4', 'red_cube_5')):
                # These cubes cannot use the Euclidean chord to A: the
                # executed carried route must first clear the immutable upper
                # obstacle rail at its west end.  Include both physical legs
                # plus an evidence-derived 1.5 m equivalent cost for the two
                # Nav2 stop/replan handoffs.  Without this term the scheduler
                # repeatedly preferred red3/red4 over red1, although the
                # measured red1 delivery was faster and avoided one dynamic
                # crossing entirely.
                rail_1 = (-2.75, 3.50)
                rail_2 = (-2.75, 2.00)
                carry_distance = (
                    math.hypot(
                        pose.position.x - rail_1[0],
                        pose.position.y - rail_1[1])
                    + math.hypot(
                        rail_2[0] - rail_1[0],
                        rail_2[1] - rail_1[1])
                    + math.hypot(
                        zone.position.x - rail_2[0],
                        zone.position.y - rail_2[1])
                    + 1.50)
            # Carrying is slower and less agile than unloaded travel.  The
            # modest weight chooses the fastest whole mission without sending
            # the robot across the field just to collect a slightly nearer cube.
            score = pickup_distance + 1.15 * carry_distance
            ranked.append((score, object_id, carry_distance))
        if not ranked:
            raise RuntimeError(f'No live candidates match {selector!r}')
        score, object_id, carry_distance = min(ranked)
        if race_selection:
            self.get_logger().info(
                f'Fastest-mission selection: {object_id}, score={score:.3f}, '
                f'estimated_carry={carry_distance:.3f}m')
        else:
            self.get_logger().info(
                f'Nearest-object selection: {object_id}, '
                f'distance={score:.3f}m')
        return object_id

    def nearest_dock_target(self, object_id, configured_target):
        """Choose the nearest axis-aligned arm-reachable pose around a cube.

        Keeping the chassis on one of the cube's four face normals prevents a
        diagonal corner grasp.  A corner grasp changes the payload height in
        the fingers and can make the fingers contact the floor before the cube
        reaches the validated release height.
        """
        cube = self._relative_pose(object_id, 'world')
        robot = self._relative_pose('six_arm', 'world')
        radius = 0.38
        configured_yaw = float(configured_target['yaw'])
        yaws = [configured_yaw + index * math.pi / 2.0 for index in range(4)]
        candidates = []
        for yaw in yaws:
            target = {
                'x': float(cube.position.x) - radius * math.cos(yaw),
                'y': float(cube.position.y) - radius * math.sin(yaw),
                'yaw': math.atan2(math.sin(yaw), math.cos(yaw)),
            }
            travel = math.hypot(
                target['x'] - robot.position.x,
                target['y'] - robot.position.y)
            candidates.append((travel, target))
        travel, target = min(candidates, key=lambda item: item[0])
        self.get_logger().info(
            f'Nearest dock for {object_id}: x={target["x"]:.3f}, '
            f'y={target["y"]:.3f}, yaw={target["yaw"]:.3f}, '
            f'direct_distance={travel:.3f}m')
        return target

    def navigate(
            self, label, target, handoff_distance=None, lock_route=False,
            strict_handoff=False, no_progress_timeout=None,
            _allow_stall_retry=True):
        """
        Navigate normally, or hand the final approach to a local controller.

        ``handoff_distance`` is intentionally based on the live Gazebo chassis
        pose instead of Nav2's ``distance_remaining`` feedback.  The latter can
        briefly report zero before a valid path is available and therefore is
        not a safe arrival signal for the pickup transition.
        """
        x = float(target['x'])
        y = float(target['y'])
        yaw = float(target['yaw'])
        self._validate_navigation_goal_cost(label, x, y)
        def stamped_pose(route_target):
            route_yaw = float(route_target['yaw'])
            result = PoseStamped()
            result.header.frame_id = 'map'
            result.header.stamp = self.get_clock().now().to_msg()
            result.pose.position.x = float(route_target['x'])
            result.pose.position.y = float(route_target['y'])
            result.pose.orientation.z = math.sin(route_yaw / 2.0)
            result.pose.orientation.w = math.cos(route_yaw / 2.0)
            return result

        goal = NavigateToPose.Goal()
        goal.pose = stamped_pose(target)
        if lock_route:
            nav2_share = get_package_share_directory('nav2_bt_navigator')
            goal.behavior_tree = (
                f'{nav2_share}/behavior_trees/'
                'navigate_w_replanning_only_if_path_becomes_invalid.xml')

        self.get_logger().info(
            f'Navigating to {label}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}, '
            f'route_policy={"replan_if_invalid" if lock_route else "default"}')
        self.publish_navigation_status(
            event='sending_goal', label=label,
            target_x=x, target_y=y, target_yaw=yaw,
            handoff_distance=(
                '' if handoff_distance is None else float(handoff_distance)),
            distance_remaining='', recoveries=0,
            feedback_x='', feedback_y='')
        self._navigation_feedback_distance = None
        self._navigation_feedback_recoveries = 0
        handle = None
        for attempt in range(1, 4):
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            sent = self.nav_client.send_goal_async(
                goal, feedback_callback=self._navigation_feedback)
            self._wait_future(
                sent, 10.0, f'send navigation goal for {label}')
            handle = sent.result()
            if handle is not None and handle.accepted:
                self.publish_navigation_status(event='goal_accepted')
                break
            if attempt < 3:
                self.get_logger().warning(
                    f'Nav2 transiently rejected the {label} goal; '
                    f'retrying ({attempt}/3) after lifecycle settling.')
                time.sleep(1.0)
        if handle is None or not handle.accepted:
            self.publish_navigation_status(event='goal_rejected')
            raise RuntimeError(
                f'Nav2 rejected the {label} goal after 3 attempts')
        result_future = handle.get_result_async()
        try:
            handed_off = self._wait_navigation_result(
                handle,
                result_future,
                label,
                x,
                y,
                yaw,
                handoff_distance,
                strict_handoff,
                no_progress_timeout,
            )
        except RuntimeError as error:
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=3.0)
            self._publish_navigation_stop()
            if isinstance(error, NavigationNoProgress) and _allow_stall_retry:
                self.get_logger().warning(
                    f'{label} made no route progress; issuing one fresh '
                    'Nav2 goal without changing maps or parameters')
                self.publish_navigation_status(event='bounded_goal_retry')
                time.sleep(0.5)
                return self.navigate(
                    label, target, handoff_distance, lock_route,
                    strict_handoff, no_progress_timeout,
                    _allow_stall_retry=False)
            raise
        if handed_off:
            return
        wrapped = result_future.result()
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            status = None if wrapped is None else wrapped.status
            if handoff_distance is None and label.startswith('destination '):
                robot = self._fresh_robot_pose()
                position_error = math.hypot(
                    float(robot.position.x) - x,
                    float(robot.position.y) - y)
                if position_error <= 0.18:
                    self._publish_navigation_stop()
                    self.get_logger().warning(
                        f'Nav2 ended with status={status}, but destination '
                        f'position is reached ({position_error:.3f}m); '
                        'delegating final yaw to low-speed alignment')
                    self.publish_navigation_status(
                        event='destination_position_handoff',
                        position_error=position_error,
                        nav_status=status)
                    return
            if (status == GoalStatus.STATUS_ABORTED
                    and lock_route and _allow_stall_retry):
                self._publish_navigation_stop()
                self.get_logger().warning(
                    f'{label} was aborted after its path became temporarily '
                    'empty; issuing one fresh locked-route goal')
                self.publish_navigation_status(
                    event='bounded_aborted_goal_retry', nav_status=status)
                time.sleep(0.5)
                return self.navigate(
                    label, target, handoff_distance, lock_route,
                    strict_handoff, no_progress_timeout,
                    _allow_stall_retry=False)
            raise RuntimeError(
                f'Navigation to {label} failed; status={status}')
        self.get_logger().info(f'Navigation to {label} succeeded')
        self.publish_navigation_status(event='goal_succeeded')

    def _wait_navigation_result(
            self, handle, result_future, label, target_x, target_y,
            target_yaw, handoff_distance, strict_handoff,
            no_progress_timeout=None):
        """Wait for Nav2 while supporting a confirmed position-only handoff."""
        destination_handoff = (
            handoff_distance is None and label.startswith('destination '))
        if handoff_distance is None and not destination_handoff:
            self._wait_future(
                result_future, self.nav_timeout,
                f'navigation to {label}')
            return False

        # Destination yaw is deliberately completed by the existing bounded
        # low-speed alignment controller.  Nav2 previously remained beside a
        # zone with distance_remaining=0 for 18 s, attempted collision-checked
        # in-place corrections, and an attached cube/chassis contact caused an
        # ODE impulse.  Stop Nav2 as soon as the chassis position is genuinely
        # reached; the carried-object zone check still gates every release.
        threshold = (0.50 if destination_handoff
                     else max(0.05, float(handoff_distance)))
        deadline = time.monotonic() + self.nav_timeout
        next_pose_check = 0.0
        stable_samples = 0
        last_pose_warning = 0.0
        last_feedback_distance = None
        last_feedback_change = time.monotonic()
        pickup_slowdown_applied = False
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if now >= deadline:
                raise RuntimeError(
                    f'Timeout while waiting for navigation to {label}')
            feedback_distance = self._navigation_feedback_distance
            if feedback_distance is not None and math.isfinite(feedback_distance):
                if (last_feedback_distance is None
                        or abs(feedback_distance - last_feedback_distance) >= 0.08):
                    last_feedback_distance = feedback_distance
                    last_feedback_change = now
                elif (self._navigation_feedback_recoveries >= 1
                      and now - last_feedback_change >= 14.0):
                    raise NavigationNoProgress(
                        f'Nav2 feedback remained at {feedback_distance:.2f}m '
                        f'for {now - last_feedback_change:.1f}s with '
                        f'{self._navigation_feedback_recoveries} recovery events')
                elif (no_progress_timeout is not None
                      and now - last_feedback_change
                      >= float(no_progress_timeout)):
                    velocity_fresh = (
                        now - self.base_velocity_received <= 0.35)
                    stopped = (
                        velocity_fresh
                        and self.base_velocity is not None
                        and abs(self.base_velocity.linear.x) <= 0.03
                        and abs(self.base_velocity.angular.z) <= 0.05)
                    if stopped:
                        raise NavigationNoProgress(
                            f'Nav2 feedback remained at '
                            f'{feedback_distance:.2f}m for '
                            f'{now - last_feedback_change:.1f}s while the '
                            'base was stopped')
            if now < next_pose_check:
                continue
            next_pose_check = now + 0.10
            try:
                robot = self._fresh_robot_pose()
            except RuntimeError as error:
                stable_samples = 0
                if now - last_pose_warning >= 2.0:
                    self.get_logger().warning(
                        f'Cannot evaluate pickup handoff yet: {error}')
                    last_pose_warning = now
                continue

            distance = math.hypot(
                float(robot.position.x) - target_x,
                float(robot.position.y) - target_y)
            object_pickup_handoff = label.startswith('pickup ')
            # Keep the fast empty-base profile over the open route, then slow
            # while Nav2 still owns collision-aware tracking.  Switching at a
            # 1.60 m dock distance leaves room to decelerate before the cube;
            # fine docking is not allowed to take ownership until odometry
            # confirms the lower-speed envelope below.
            if (object_pickup_handoff
                    and not pickup_slowdown_applied
                    and distance <= 1.60):
                self.configure_final_pickup_tracking()
                pickup_slowdown_applied = True
                self.get_logger().info(
                    f'Pickup proximity speed transition armed at '
                    f'{distance:.3f}m from dock')
                continue
            velocity_fresh = now - self.base_velocity_received <= 0.35
            speed = min(1.5, abs(self.base_velocity.linear.x)) if (
                velocity_fresh and self.base_velocity is not None) else 1.5
            # Include pipeline latency, braking and pose sampling.  With the
            # visual stack active, a measured 1.2 m/s pickup approach travelled
            # about 0.77 m between cancellation request and confirmed zero
            # odometry.  The old 150 ms / 3.2 m/s^2 estimate therefore let the
            # chassis pass the dock and touch the cube.  This measured 550 ms /
            # 2.0 m/s^2 envelope stops before the dock while retaining the
            # verified 1.2 m/s cruise speed.
            object_proximity_handoff = False
            if strict_handoff:
                dynamic_threshold = threshold
            elif destination_handoff:
                dynamic_threshold = threshold
            elif object_pickup_handoff:
                dynamic_threshold = max(
                    threshold,
                    0.20 + 0.55*speed + speed*speed/(2.0*2.0))
            else:
                dynamic_threshold = max(
                    threshold,
                    0.18 + 0.15*speed + speed*speed/(2.0*3.2))
            if object_pickup_handoff:
                # Hand off on dock distance even if Nav2 has not completed
                # its final yaw.  Waiting for the cube to enter the front
                # half-plane let the fast base overshoot the dock during its
                # last turn; on blue_cube_3 this produced a physical impact
                # and an ODE impulse.  Fine docking already rotates with zero
                # linear speed until the cube is in front, so stopping early
                # is both safer and deterministic.
                cube_in_base = self._relative_pose(
                    self.object_id, 'six_arm')
                cube_distance = math.hypot(
                    cube_in_base.position.x, cube_in_base.position.y)
                # The global path may approach the dock from its far side, so
                # dock error alone does not bound chassis-to-cube clearance.
                # The action cancellation took up to 1.3 s in visual mode;
                # trigger directly from live cube range with that measured
                # delay, then finish at 0.16 m/s under fine-dock control.
                cube_brake_threshold = (
                    0.385 + 1.20*speed + speed*speed/(2.0*2.0))
                # A cube can be physically close while a wall still separates
                # the chassis from its selected dock.  Do not abandon the
                # collision-aware global path until it has actually entered
                # the final 1.10 m dock corridor; the earlier unrestricted
                # proximity trigger handed blue_cube_3 to the direct controller
                # with 2.26 m of global path remaining and the robot hit a wall.
                object_proximity_handoff = (
                    cube_distance <= cube_brake_threshold
                    and distance <= 1.10)
                if cube_distance <= 0.28:
                    raise RuntimeError(
                        'Cube entered chassis protection radius during '
                        'pickup handoff; stopping before contact')
            destination_approach_ready = True
            if destination_handoff:
                q = robot.orientation
                robot_yaw = math.atan2(
                    2.0 * (q.w*q.z + q.x*q.y),
                    1.0 - 2.0 * (q.y*q.y + q.z*q.z))
                target_bearing = math.atan2(
                    target_y - float(robot.position.y),
                    target_x - float(robot.position.x))
                approach_error = math.atan2(
                    math.sin(target_bearing - robot_yaw),
                    math.cos(target_bearing - robot_yaw))
                final_yaw_error = math.atan2(
                    math.sin(float(target_yaw) - robot_yaw),
                    math.cos(float(target_yaw) - robot_yaw))
                destination_approach_ready = (
                    abs(approach_error) <= 0.75
                    and abs(final_yaw_error) <= 0.40)
            # The dynamic braking envelope can be larger than the final dock
            # corridor at cruise speed.  Applying it without this geometric
            # gate allowed a 1.407 m dock-distance handoff while a wall still
            # separated blue_cube_1 from the robot.  Keep Nav2 in charge until
            # the chassis has entered the already documented 1.10 m corridor;
            # cancellation braking and fine docking then retain their roles.
            pickup_corridor_ready = (
                not object_pickup_handoff or distance <= 1.10)
            pickup_speed_ready = (
                not object_pickup_handoff
                or (pickup_slowdown_applied and speed <= 0.72))
            if (math.isfinite(distance)
                    and (distance <= dynamic_threshold
                         or object_proximity_handoff)
                    and pickup_corridor_ready
                    and pickup_speed_ready
                    and destination_approach_ready):
                stable_samples += 1
            else:
                stable_samples = 0
            if stable_samples < 1:
                continue

            handoff_kind = 'Destination position' if destination_handoff else 'Pickup'
            self.get_logger().info(
                f'{handoff_kind} handoff reached: distance={distance:.3f}m, '
                f'threshold={dynamic_threshold:.3f}m; canceling Nav2')
            self.publish_navigation_status(
                event='handoff_triggered', handoff_distance=distance)
            if not self._cancel_navigation_goal(
                    handle, result_future, label, timeout_sec=3.0):
                raise RuntimeError(
                    f'Nav2 did not stop cleanly during {label} handoff')
            self._publish_navigation_stop()
            if destination_handoff:
                self.get_logger().info(
                    'Nav2 stopped; transferring only final yaw to low-speed '
                    'dropoff alignment')
                self.publish_navigation_status(
                    event='destination_position_handoff',
                    position_error=distance)
            else:
                self.get_logger().info(
                    'Nav2 stopped; transferring base control to fine dock')
                self.publish_navigation_status(event='handoff_complete')
            return True

        return False

    def _cancel_navigation_goal(
            self, handle, result_future, label, timeout_sec):
        """Cancel Nav2 and wait for its terminal result before handoff."""
        deadline = time.monotonic() + max(0.5, float(timeout_sec))
        cancel_future = handle.cancel_goal_async()
        # Begin braking as soon as cancellation is requested.  Waiting for the
        # action result first left the last cruise command active for almost a
        # second, enough to pass a cube from a 0.6 m handoff at 1.25 m/s.
        while (rclpy.ok() and not cancel_future.done()
               and time.monotonic() < deadline):
            self._publish_dock_command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.04)
        if not cancel_future.done() or cancel_future.exception() is not None:
            self.get_logger().error(
                f'Failed to confirm cancellation of navigation to {label}')
            return False

        while (rclpy.ok() and not result_future.done()
               and time.monotonic() < deadline):
            self._publish_dock_command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.04)
        if not result_future.done() or result_future.exception() is not None:
            self.get_logger().error(
                f'Nav2 cancellation accepted but {label} did not terminate')
            return False

        wrapped = result_future.result()
        status = None if wrapped is None else wrapped.status
        if status not in (
                GoalStatus.STATUS_CANCELED,
                GoalStatus.STATUS_SUCCEEDED):
            self.get_logger().error(
                f'Navigation to {label} ended with status={status} during '
                'handoff')
            return False
        return True

    def _publish_navigation_stop(self, count=6, interval_sec=0.04):
        """Do not transfer ownership until fresh odometry confirms a stop."""
        deadline = time.monotonic() + 4.0
        stopped_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            self._publish_dock_command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.04)
            now = time.monotonic()
            velocity = self.base_velocity
            stopped = (velocity is not None
                       and now - self.base_velocity_received < 0.35
                       and abs(velocity.linear.x) < 0.025
                       and abs(velocity.angular.z) < 0.08)
            stopped_since = (stopped_since or now) if stopped else None
            if stopped_since is not None and now - stopped_since >= 0.15:
                return
        velocity = self.base_velocity
        velocity_age = time.monotonic() - self.base_velocity_received
        linear = float('nan') if velocity is None else velocity.linear.x
        angular = float('nan') if velocity is None else velocity.angular.z
        raise RuntimeError(
            'Base did not stop with fresh odometry; handoff refused '
            f'(vx={linear:.4f}, wz={angular:.4f}, age={velocity_age:.3f}s)')

    def manipulate(self, operation):
        goal = ExecuteManipulation.Goal()
        goal.operation = operation
        goal.object_id = self.object_id
        self.get_logger().info(f'Starting {operation}: {self.object_id}')
        sent = self.arm_client.send_goal_async(
            goal, feedback_callback=self._manipulation_feedback)
        self._wait_future(sent, 10.0, f'send {operation} goal')
        handle = sent.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f'Manipulation server rejected {operation}')
        result_future = handle.get_result_async()
        try:
            self._wait_future(
                result_future, self.arm_timeout,
                f'{operation} {self.object_id}')
        except RuntimeError:
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=3.0)
            raise
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError(f'{operation} returned no result')
        result = wrapped.result
        if not result.success:
            raise ManipulationFailure(
                operation, result.error_code, result.message)
        self.get_logger().info(
            f'{operation} succeeded: {result.message}')

    def configure_dropoff_tracking(self):
        """Use a longer RPP preview only for the long carried-object leg."""
        if not self.controller_parameters.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('controller_server parameter service unavailable')
        requested = {
            # The raised arm and attached cube move the centre of mass upward.
            # A 2026-09-12 five-red trace became physically unstable on the
            # long A approach at the 1.40 m/s empty-base cruise setting; Nav2
            # then received an empty path before Gazebo threw the chassis out
            # of the map. Keep the fast setting for empty travel, but use the
            # measured conservative envelope while carrying.
            'FollowPath.desired_linear_vel': 1.05,
            'FollowPath.lookahead_dist': 1.10,
            'FollowPath.min_lookahead_dist': 0.90,
            'FollowPath.max_lookahead_dist': 1.60,
            'FollowPath.lookahead_time': 1.00,
        }
        request = SetParameters.Request()
        request.parameters = [
            ParameterMsg(
                name=name,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=value))
            for name, value in requested.items()
        ]
        future = self.controller_parameters.call_async(request)
        self._wait_future(future, 5.0, 'configure dropoff path tracking')
        results = future.result().results
        failures = [
            result.reason or 'rejected'
            for result in results if not result.successful
        ]
        if failures:
            raise RuntimeError(
                'Cannot configure dropoff path tracking: ' + '; '.join(failures))
        self.get_logger().info(
            'Dropoff RPP profile configured: carried_speed=1.05m/s, '
            'lookahead=1.10m, '
            'range=0.90..1.60m, time=1.00s')

    def configure_pickup_tracking(self):
        """Restore empty cruise and the destination-safe turn profile."""
        if not self.controller_parameters.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('controller_server parameter service unavailable')
        # A controlled 2026-09-09 A-route comparison showed that 0.90 rad
        # removes about 4.6 s of moderate-corner stop/rotate cycles without an
        # acceptance regression.  Applying it globally is unsafe: the
        # 2026-09-22 B/C regression cut the C wall-end crossing and aborted at
        # 1.09 m remaining.  Keep C on the proven 0.60 rad threshold and use
        # 0.90 only for A/B tasks, where all compulsory wall turns remain true
        # 90-degree corners and therefore still rotate in place.
        turn_threshold = 0.60 if self.destination == 'C' else 0.90
        request = SetParameters.Request()
        request.parameters = [
            ParameterMsg(
                name='FollowPath.desired_linear_vel',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=1.40)),
            ParameterMsg(
                name='FollowPath.rotate_to_heading_min_angle',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=turn_threshold)),
        ]
        future = self.controller_parameters.call_async(request)
        self._wait_future(future, 5.0, 'restore pickup cruise speed')
        failures = [
            result.reason or 'rejected'
            for result in future.result().results if not result.successful
        ]
        if failures:
            raise RuntimeError(
                'Cannot restore pickup cruise speed: '
                + '; '.join(failures))
        self.get_logger().info(
            'Pickup RPP profile restored: '
            f'speed=1.40m/s, turn_threshold={turn_threshold:.2f}rad, '
            f'destination={self.destination}')

    def configure_final_pickup_tracking(self):
        """Bound speed before the last Nav2 approach to an ungrasped cube.

        The empty-base transit may safely use the 1.40 m/s profile, but that
        speed leaves too much cancellation distance when Nav2 hands control
        to fine docking.  A cube is still a movable physics body at this
        point, so use a separate 0.60 m/s final-approach profile instead of
        relying on the fine controller to recover after contact.
        """
        if not self.controller_parameters.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('controller_server parameter service unavailable')
        request = SetParameters.Request()
        request.parameters = [ParameterMsg(
            name='FollowPath.desired_linear_vel',
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=0.60))]
        future = self.controller_parameters.call_async(request)
        self._wait_future(future, 5.0, 'configure final pickup approach speed')
        result = future.result().results[0]
        if not result.successful:
            raise RuntimeError(
                'Cannot configure final pickup approach speed: '
                + (result.reason or 'rejected'))
        self.get_logger().info(
            'Final pickup RPP speed configured: 0.60m/s '
            '(empty transit remains 1.40m/s)')

    def navigate_pickup_transits(self, pickup_target=None):
        """Bypass a dynamic swept track before selected pickup approaches."""
        leaving_a = False
        robot_in_a = self._relative_pose('six_arm', 'zone_a')
        if math.hypot(robot_in_a.position.x, robot_in_a.position.y) <= 1.80:
            leaving_a = True
            # After an intermediate A placement, leave through the same
            # verified doorway instead of letting a fresh global plan select
            # a blocked or much longer homotopy.  This applies regardless of
            # the next cube's final destination.
            exit_transits = [
                # Stay on the proven interior corridor, then traverse the
                # exact inbound doorway points in reverse order.
                {'x': -4.00, 'y': -1.75, 'yaw': 0.0},
                {'x': -2.25, 'y': -0.25, 'yaw': math.pi / 2.0},
            ]
            if self.object_id not in ('red_cube_3', 'red_cube_4'):
                exit_transits.append(
                    {'x': -2.30, 'y': 1.30, 'yaw': math.pi / 2.0})
            else:
                # red3/red4 immediately execute the west-rail bypass below.
                # Its first point (-2.75, 2.00) is already beyond the same
                # central doorway, so stopping once at (-2.30, 1.30) and then
                # reacquiring an almost collinear path is redundant.  Keep the
                # two proven A-exit constraints, then let the rail waypoint
                # perform the doorway handoff in one continuous Nav2 leg.
                self.get_logger().info(
                    'A exit merges outer doorway with west-rail bypass for '
                    f'{self.object_id}')
            for index, transit in enumerate(exit_transits, 1):
                self.publish_navigation_status(
                    phase='PICKUP_TRANSIT', event='phase_start',
                    transit_index=index,
                    transit_count=len(exit_transits),
                    route_mode='a_reverse_doorway_handoff')
                self.navigate(
                    f'A reverse doorway transit {index}', transit,
                    handoff_distance=0.35, lock_route=True)
        # A direct last-leg plan from A's north-east doorway to a blue pickup
        # can wedge beside the centre stone (2.57 m remaining in the
        # 2026-09-10 red4/blue1 CoStudio run).  The B east transit is already
        # proven for blue carried-object traffic; use it in reverse only for
        # this A -> blue return, then let the normal live pickup plan finish.
        # Initial blue->B and all red tasks remain on their existing routes.
        east_side_pickup = (
            pickup_target is not None
            and float(pickup_target['x']) > -2.30)
        if (leaving_a
                and self.destination == 'B'
                and self.object_id.startswith('blue_cube_')
                and east_side_pickup):
            transit = {'x': -1.45, 'y': -2.40, 'yaw': math.pi / 2.0}
            self.publish_navigation_status(
                phase='PICKUP_TRANSIT', event='phase_start',
                transit_index=4, transit_count=4,
                route_mode='a_to_blue_b_east_reverse')
            self.navigate(
                'A-to-blue B east reverse transit', transit,
                handoff_distance=0.35, lock_route=True)
        elif (leaving_a
                and self.destination == 'B'
                and self.object_id.startswith('blue_cube_')):
            target_x = (float(pickup_target['x'])
                        if pickup_target is not None else float('nan'))
            self.get_logger().info(
                'A-to-blue pickup stays on west corridor: '
                f'pickup_x={target_x:.3f}, east_boundary=-2.300; '
                'skipping B east reverse transit')
        robot_in_b = self._relative_pose('six_arm', 'zone_b')
        leaving_b = math.hypot(
            robot_in_b.position.x, robot_in_b.position.y) <= 1.80
        if leaving_b:
            # A direct B -> upper-row pickup plan repeatedly changed homotopy
            # around the centre walls: the 2026-09-22 A/B baseline spent
            # 31.5 s on red_cube_2 while distance_remaining jumped between
            # 4 and 9 m.  Retrace the already-proven B inbound and central
            # doorway points before selecting the live final pickup leg.
            # This changes neither obstacle geometry nor the final grasp pose.
            b_exit_transits = (
                {'x': -1.45, 'y': -2.40, 'yaw': math.pi / 2.0},
                {'x': -2.25, 'y': -0.25, 'yaw': math.pi / 2.0},
                {'x': -2.30, 'y': 1.30, 'yaw': math.pi / 2.0},
            )
            for index, transit in enumerate(b_exit_transits, 1):
                self.publish_navigation_status(
                    phase='PICKUP_TRANSIT', event='phase_start',
                    transit_index=index,
                    transit_count=len(b_exit_transits),
                    route_mode='b_reverse_central_doorway')
                self.navigate(
                    f'B reverse doorway transit {index}', transit,
                    handoff_distance=0.35, lock_route=True)
        if self.object_id == 'red_cube_5':
            # red_cube_5 lies north of moving_obstacle_1's immutable y=2.8
            # rail.  The old west-end route then drove east along the cube row
            # and was correctly stopped by red_cube_4.  The two zero-cost
            # staging cells below form a diagonal crossing between blue cube
            # columns; cross only while obstacle 1 is safely to the west.
            transits = (
                {'x': -0.65, 'y': 1.80, 'yaw': 0.0},
                {'x': 1.20, 'y': 3.20, 'yaw': 0.0},
            )
            for index, transit in enumerate(transits, 1):
                if index == 2:
                    self._wait_for_red5_rail_crossing_clear()
                self.publish_navigation_status(
                    phase='PICKUP_TRANSIT', event='phase_start',
                    transit_index=index, transit_count=len(transits),
                    route_mode='red5_gated_vertical_crossing')
                self.navigate(
                    f'red_cube_5 gated rail crossing {index}', transit,
                    handoff_distance=0.08 if index == 2 else 0.20,
                    strict_handoff=(index == 2),
                    lock_route=True)
            return
        if self.object_id not in ('red_cube_3', 'red_cube_4'):
            return
        # A direct origin -> red_cube_4 chord crosses moving_obstacle_1's
        # prescribed y=2.8 rail inside x=[-2, 1].  A contact with that
        # kinematic obstacle can launch the chassis.  Cross 0.75 m beyond the
        # west endpoint, then approach along the north side of the rail.
        transits = (
            {'x': -2.75, 'y': 2.00, 'yaw': math.pi / 2.0},
            {'x': -2.75, 'y': 3.50, 'yaw': 0.0},
        )
        for index, transit in enumerate(transits, 1):
            self.publish_navigation_status(
                phase='PICKUP_TRANSIT', event='phase_start',
                transit_index=index, transit_count=len(transits))
            self.navigate(
                f'{self.object_id} rail bypass {index}', transit,
                # The second point only selects the north-side homotopy.  Its
                # yaw is not an action requirement, and exact yaw convergence
                # caused an endless 0.00-0.21 m rotate/translate oscillation.
                # Request the smallest position-only handoff floor; the live
                # braking-distance term may enlarge it at cruise speed.  The
                # following strict pickup dock and fine dock still establish
                # the actual grasp geometry.
                handoff_distance=0.08 if index == 2 else None,
                lock_route=True)

    def _wait_for_red5_rail_crossing_clear(self, timeout_sec=30.0):
        """Hold south-west of rail 1 until its diagonal crossing is clear."""
        clear_west_x = -1.10
        deadline = time.monotonic() + float(timeout_sec)
        stable_since = None
        last_log = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            self._publish_dock_command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.10)
            now = time.monotonic()
            obstacle = self.model_poses.get('moving_obstacle_1')
            fresh = now - self.models_received < 0.35
            obstacle_x = (float(obstacle.position.x)
                          if obstacle is not None else float('nan'))
            separated = fresh and obstacle is not None and (
                obstacle_x <= clear_west_x)
            if separated:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= 0.40:
                    self.get_logger().info(
                        'red5 rail crossing released: '
                        f'obstacle_x={obstacle_x:.3f}, crossing_x~=0.67')
                    return
            else:
                stable_since = None
            if now - last_log >= 1.0:
                self.get_logger().info(
                    'red5 rail crossing hold: '
                    f'obstacle_x={obstacle_x:.3f}, '
                    f'release_when_x<={clear_west_x:.2f}')
                last_log = now
        raise RuntimeError(
            'moving_obstacle_1 did not clear the red5 crossing in time')

    def _wait_for_c_north_crossing_clear(self, timeout_sec=30.0):
        """Hold west of obstacle 2 until the north crossing is separated."""
        crossing_y = -3.60
        # Once the obstacle has passed 0.90 m south of the crossing and is
        # still moving south, it is separating from the robot for the whole
        # crossing.  Waiting until y=-5.30 added about 3.2 s at every gate
        # without increasing the minimum separation.
        clear_below_y = -4.50
        deadline = time.monotonic() + float(timeout_sec)
        stable_since = None
        last_log = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            self._publish_dock_command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.10)
            now = time.monotonic()
            obstacle = self.model_poses.get('moving_obstacle_2')
            fresh = now - self.models_received < 0.35
            obstacle_y = (float(obstacle.position.y)
                          if obstacle is not None else float('nan'))
            obstacle_twist = self.model_twists.get('moving_obstacle_2')
            obstacle_vy = (float(obstacle_twist.linear.y)
                           if obstacle_twist is not None else float('nan'))
            separated = fresh and obstacle is not None and (
                obstacle_y <= clear_below_y
                and obstacle_twist is not None
                and obstacle_vy <= -0.05)
            if separated:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= 0.40:
                    self.get_logger().info(
                        'C north crossing released: '
                        f'obstacle_y={obstacle_y:.3f}, '
                        f'obstacle_vy={obstacle_vy:.3f}, '
                        f'crossing_y={crossing_y:.2f}')
                    return
            else:
                stable_since = None
            if now - last_log >= 1.0:
                self.get_logger().info(
                    'C north crossing hold: '
                    f'obstacle_y={obstacle_y:.3f}, '
                    f'obstacle_vy={obstacle_vy:.3f}, '
                    f'release_when_y<={clear_below_y:.2f}_and_vy<0')
                last_log = now
        raise RuntimeError(
            'moving_obstacle_2 did not clear the C north crossing in time')

    def _wait_for_c_south_crossing_clear(self, timeout_sec=30.0):
        """Hold west of the south crossing until obstacle 2 is separating."""
        crossing_y = -6.55
        clear_above_y = -4.50
        deadline = time.monotonic() + float(timeout_sec)
        stable_since = None
        last_log = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            self._publish_dock_command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.10)
            now = time.monotonic()
            obstacle = self.model_poses.get('moving_obstacle_2')
            obstacle_twist = self.model_twists.get('moving_obstacle_2')
            fresh = now - self.models_received < 0.35
            obstacle_y = (float(obstacle.position.y)
                          if obstacle is not None else float('nan'))
            obstacle_vy = (float(obstacle_twist.linear.y)
                           if obstacle_twist is not None else float('nan'))
            separated = fresh and obstacle is not None and (
                obstacle_y >= clear_above_y
                and obstacle_twist is not None
                and obstacle_vy >= 0.05)
            if separated:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= 0.40:
                    self.get_logger().info(
                        'C south crossing released: '
                        f'obstacle_y={obstacle_y:.3f}, '
                        f'obstacle_vy={obstacle_vy:.3f}, '
                        f'crossing_y={crossing_y:.2f}')
                    return
            else:
                stable_since = None
            if now - last_log >= 1.0:
                self.get_logger().info(
                    'C south crossing hold: '
                    f'obstacle_y={obstacle_y:.3f}, '
                    f'obstacle_vy={obstacle_vy:.3f}, '
                    f'release_when_y>={clear_above_y:.2f}_and_vy>0')
                last_log = now
        raise RuntimeError(
            'moving_obstacle_2 did not clear the C south crossing in time')

    @staticmethod
    def _predict_c_obstacle(y, vy, seconds):
        """Predict the immutable -6..-3 m shuttle with endpoint reflection."""
        speed = max(0.05, abs(float(vy)))
        direction = 1.0 if float(vy) >= 0.0 else -1.0
        position = max(-6.0, min(-3.0, float(y)))
        remaining = max(0.0, float(seconds))
        while remaining > 1e-9:
            boundary = -3.0 if direction > 0.0 else -6.0
            to_boundary = abs(boundary - position) / speed
            if remaining <= to_boundary:
                position += direction * speed * remaining
                remaining = 0.0
            else:
                position = boundary
                remaining -= to_boundary
                direction *= -1.0
        return position, direction * speed

    def _choose_c_crossing(self):
        """Choose the lower ETA of the real north and south C crossings."""
        obstacle = self.model_poses.get('moving_obstacle_2')
        twist = self.model_twists.get('moving_obstacle_2')
        if obstacle is None or twist is None:
            raise RuntimeError(
                'moving_obstacle_2 state unavailable for C route choice')
        robot = self._relative_pose('six_arm', 'world')
        obstacle_y = float(obstacle.position.y)
        obstacle_vy = float(twist.linear.y)
        first_leg = math.hypot(
            float(robot.position.x) - 0.35,
            float(robot.position.y) + 1.00)

        def estimate(mode):
            crossing_y = -3.60 if mode == 'north' else -6.55
            # Strict waypoint handoffs make the measured carried-route
            # progress about 0.43 m/s even though RPP's straight-line limit is
            # 1.05 m/s.  ETA must use that end-to-end value or it predicts a
            # gate phase roughly five seconds too early and chooses the wrong
            # side of the shuttle.
            effective_speed = 0.43
            arrival = (first_leg + abs(crossing_y + 1.00)) / effective_speed
            wait = 0.0
            while wait <= 24.0:
                predicted_y, predicted_vy = self._predict_c_obstacle(
                    obstacle_y, obstacle_vy, arrival + wait)
                safe = ((mode == 'north'
                         and predicted_y <= -4.50
                         and predicted_vy < 0.0)
                        or (mode == 'south'
                            and predicted_y >= -4.50
                            and predicted_vy > 0.0))
                if safe:
                    break
                wait += 0.10
            after_distance = 6.50 if mode == 'north' else 3.55
            # Distance alone made equal-length routes default to north even
            # though north has two additional strict Nav2 handoffs (cross,
            # turn south, turn east).  Successful 2026-09-22 regressions show
            # each cancel/settle/reacquire boundary costs about 3--4 s.  Model
            # only that measured orchestration cost; geometry, gate release
            # conditions, and controller speeds remain unchanged.
            handoff_overhead = 8.0 if mode == 'north' else 0.0
            total = (arrival + wait + after_distance / effective_speed
                     + handoff_overhead)
            return total, arrival, wait, handoff_overhead

        north = estimate('north')
        south = estimate('south')
        # The north route has two extra stop/cancel/reacquire boundaries.  A
        # sub-second ETA lead is below the measured prediction noise and can
        # turn into an 8+ s loss once those boundaries execute.  Prefer the
        # simpler south route unless north is materially faster.
        north_switch_margin = 6.0
        mode = (
            'north'
            if north[0] + north_switch_margin <= south[0]
            else 'south')
        self.get_logger().info(
            'C crossing ETA choice: '
            f'obstacle_y={obstacle_y:.3f}, obstacle_vy={obstacle_vy:.3f}, '
            f'north_total={north[0]:.2f}s(wait={north[2]:.2f}s,'
            f'handoff={north[3]:.2f}s), '
            f'south_total={south[0]:.2f}s(wait={south[2]:.2f}s,'
            f'handoff={south[3]:.2f}s), '
            f'north_switch_margin={north_switch_margin:.1f}s, '
            f'selected={mode}')
        return mode, obstacle_y

    def _choose_c_exit_crossing(self):
        """Re-evaluate the shorter guarded crossing from the C side."""
        obstacle = self.model_poses.get('moving_obstacle_2')
        twist = self.model_twists.get('moving_obstacle_2')
        if obstacle is None or twist is None:
            raise RuntimeError(
                'moving_obstacle_2 state unavailable for C exit choice')
        robot = self._relative_pose('six_arm', 'world')
        obstacle_y = float(obstacle.position.y)
        obstacle_vy = float(twist.linear.y)
        to_south_gate = math.hypot(
            float(robot.position.x) - 3.90,
            float(robot.position.y) + 6.55)
        effective_speed = 0.43

        def estimate(mode):
            before_distance = (to_south_gate + 0.90 + 2.95
                               if mode == 'north' else to_south_gate)
            arrival = before_distance / effective_speed
            wait = 0.0
            while wait <= 24.0:
                predicted_y, predicted_vy = self._predict_c_obstacle(
                    obstacle_y, obstacle_vy, arrival + wait)
                safe = ((mode == 'north'
                         and predicted_y <= -4.50
                         and predicted_vy < 0.0)
                        or (mode == 'south'
                            and predicted_y >= -4.50
                            and predicted_vy > 0.0))
                if safe:
                    break
                wait += 0.10
            after_distance = 5.25 if mode == 'north' else 9.10
            handoff_overhead = 8.0 if mode == 'north' else 0.0
            return (arrival + wait + after_distance / effective_speed
                    + handoff_overhead), wait, handoff_overhead

        north = estimate('north')
        south = estimate('south')
        north_switch_margin = 6.0
        mode = (
            'north'
            if north[0] + north_switch_margin <= south[0]
            else 'south')
        self.get_logger().info(
            'C exit ETA choice: '
            f'obstacle_y={obstacle_y:.3f}, obstacle_vy={obstacle_vy:.3f}, '
            f'north_total={north[0]:.2f}s(wait={north[1]:.2f}s,'
            f'handoff={north[2]:.2f}s), '
            f'south_total={south[0]:.2f}s(wait={south[1]:.2f}s,'
            f'handoff={south[2]:.2f}s), '
            f'north_switch_margin={north_switch_margin:.1f}s, '
            f'selected={mode}')
        return mode

    def navigate_dropoff_transits(self):
        """Enter each delivery corridor before the final continuous leg."""
        if self.destination == 'C':
            if self.object_id in ('red_cube_3', 'red_cube_4'):
                rail_exit = (
                    {'x': -2.75, 'y': 3.50, 'yaw': math.pi},
                    {'x': -2.75, 'y': 2.00, 'yaw': -math.pi / 2.0},
                )
                for index, transit in enumerate(rail_exit, 1):
                    self.publish_navigation_status(
                        phase='DROPOFF_TRANSIT', event='phase_start',
                        transit_index=index, transit_count=len(rail_exit),
                        route_mode='red34_to_c_west_rail_exit')
                    self.navigate(
                        f'{self.object_id} to-C rail exit {index}', transit,
                        handoff_distance=0.20, lock_route=True)
            elif self.object_id == 'red_cube_5':
                rail_exit = (
                    {'x': 1.20, 'y': 3.20, 'yaw': math.pi},
                    {'x': -0.65, 'y': 1.80, 'yaw': math.pi},
                )
                for index, transit in enumerate(rail_exit, 1):
                    if index == 2:
                        self._wait_for_red5_rail_crossing_clear()
                    self.publish_navigation_status(
                        phase='DROPOFF_TRANSIT', event='phase_start',
                        transit_index=index, transit_count=len(rail_exit),
                        route_mode='red5_to_c_gated_vertical_exit')
                    self.navigate(
                        f'red_cube_5 to-C gated rail exit {index}', transit,
                        handoff_distance=0.08 if index == 2 else 0.20,
                        strict_handoff=(index == 2),
                        lock_route=True)
            # The authoritative officeroom origin is (0.9981, -4.92976).
            # Wall_118 is therefore the vertical segment at x=3.601 from
            # y=-3.533 to -6.033.  Cross obstacle 2's x=2 rail once at the
            # north staging line, settle on x=3.0 (1.0 m from its rail), then
            # travel south without running alongside the obstacle at 0.35 m.
            # The tight handoffs below also ensure the chassis is fully south
            # of Wall_118 before crossing its endpoint.
            # Both physical C routes share this west-side staging point.  Do
            # not lock north/south while still several metres away: the
            # obstacle can complete a large part of its cycle during that
            # common transit, making an otherwise correct ETA stale before
            # the robot reaches the rail.  Arrive and settle here first, then
            # choose once from the freshest obstacle pose.  This adds no
            # waypoint compared with the routes below; it only delays the
            # already-existing branch decision.
            common_staging = {
                'x': 0.35, 'y': -1.00, 'yaw': -math.pi / 2.0,
            }
            self.publish_navigation_status(
                phase='DROPOFF_TRANSIT', event='phase_start',
                transit_index=1, transit_count=1,
                route_mode='c_common_west_staging')
            self.navigate(
                'C common west staging', common_staging,
                handoff_distance=0.30, lock_route=True)

            crossing_mode, obstacle_y = self._choose_c_crossing()
            self._c_crossing_mode = crossing_mode
            if crossing_mode == 'north':
                route_mode = 'c_eta_north_crossing'
                staging = (
                    {'x': 0.35, 'y': -3.60, 'yaw': 0.0},
                )
                after_crossing = (
                    {'x': 3.00, 'y': -3.60, 'yaw': -math.pi / 2.0,
                     'handoff': 0.10, 'strict_handoff': True},
                    {'x': 3.00, 'y': -6.55, 'yaw': 0.0,
                     'handoff': 0.08, 'strict_handoff': True},
                    {'x': 3.90, 'y': -6.55, 'yaw': 0.0,
                     'handoff': 0.20, 'strict_handoff': True},
                )
            else:
                route_mode = 'c_eta_south_crossing'
                staging = (
                    {'x': 0.35, 'y': -6.55, 'yaw': 0.0,
                     'handoff': 0.10, 'strict_handoff': True},
                )
                after_crossing = (
                    {'x': 3.90, 'y': -6.55, 'yaw': 0.0,
                     'handoff': 0.20, 'strict_handoff': True},
                )
            transits = staging + after_crossing
            self.get_logger().info(
                f'C route selected from moving_obstacle_2 y={obstacle_y:.3f}: '
                f'{route_mode}')
            for index, transit in enumerate(transits, 1):
                if index == len(staging) + 1:
                    if crossing_mode == 'north':
                        self._wait_for_c_north_crossing_clear()
                    else:
                        self._wait_for_c_south_crossing_clear()
                self.publish_navigation_status(
                    phase='DROPOFF_TRANSIT', event='phase_start',
                    transit_index=index, transit_count=len(transits),
                    route_mode=route_mode, obstacle_y=obstacle_y)
                self.navigate(
                    f'C dynamic bypass {index}', transit,
                    handoff_distance=transit.get('handoff', 0.30),
                    strict_handoff=transit.get('strict_handoff', False),
                    lock_route=True)
            return
        if self.destination == 'B':
            if self.object_id.startswith('red_cube_'):
                if self.object_id in ('red_cube_3', 'red_cube_4'):
                    # red_cube_4 is picked north of moving_obstacle_1's
                    # immutable y=2.8 rail.  Retrace the west-end bypass while
                    # carrying it; a direct diagonal to the central doorway
                    # crossed the rail and launched the chassis out of bounds.
                    rail_exit_transits = (
                        {'x': -2.75, 'y': 3.50, 'yaw': math.pi},
                        {'x': -2.75, 'y': 2.00, 'yaw': -math.pi / 2.0},
                    )
                    for index, transit in enumerate(rail_exit_transits, 1):
                        self.publish_navigation_status(
                            phase='DROPOFF_TRANSIT', event='phase_start',
                            transit_index=index,
                            transit_count=len(rail_exit_transits),
                            route_mode='red34_to_b_rail_exit')
                        self.navigate(
                            f'{self.object_id} to-B rail exit {index}', transit,
                            handoff_distance=0.35, lock_route=True)
                elif self.object_id == 'red_cube_5':
                    # The east side is disconnected in the static map. Reuse
                    # the proven west-end crossing for this inverse-colour
                    # carried route as well.
                    rail_exit_transits = (
                        {'x': -2.75, 'y': 3.50, 'yaw': math.pi},
                        {'x': -2.75, 'y': 2.00, 'yaw': -math.pi / 2.0},
                    )
                    for index, transit in enumerate(rail_exit_transits, 1):
                        self.publish_navigation_status(
                            phase='DROPOFF_TRANSIT', event='phase_start',
                            transit_index=index,
                            transit_count=len(rail_exit_transits),
                            route_mode='red5_to_b_west_rail_exit')
                        self.navigate(
                            f'red_cube_5 to-B rail exit {index}', transit,
                            handoff_distance=0.35, lock_route=True)
            if (self.object_id.startswith('red_cube_')
                    or self.object_id in ('blue_cube_1', 'blue_cube_2')):
                # Upper-row red cubes and the two west-side blue cubes cannot
                # reliably reach the fixed B east transit with one direct
                # chord.  blue_cube_2 was observed to lose its path twice at
                # 6.53 m remaining after a valid grasp.  Select the already
                # proven central-doorway homotopy before entering B.  The
                # remaining east-side blue cubes retain the shorter direct
                # route.
                cross_zone_transits = (
                    {'x': -2.30, 'y': 1.30, 'yaw': -math.pi / 2.0},
                    {'x': -2.25, 'y': -0.25, 'yaw': -math.pi / 2.0},
                )
                for index, transit in enumerate(cross_zone_transits, 1):
                    self.publish_navigation_status(
                        phase='DROPOFF_TRANSIT', event='phase_start',
                        transit_index=index,
                        transit_count=len(cross_zone_transits),
                        route_mode='west_to_b_central_doorway')
                    self.navigate(
                        f'{self.object_id} to-B doorway transit {index}', transit,
                        handoff_distance=0.35, lock_route=True)
            # The fixed stone west of the centre corridor makes the direct B
            # chord intermittently stop about 1.2 m from the dock.  Approach
            # its east side first; this point is already proven in the
            # production record and does not alter obstacle geometry.
            self.publish_navigation_status(
                phase='DROPOFF_TRANSIT', event='phase_start',
                transit_index=1, transit_count=1)
            self.navigate(
                'B east transit',
                {'x': -1.45, 'y': -2.40, 'yaw': -math.pi / 2.0},
                handoff_distance=0.35, lock_route=True)
            return
        if self.destination != 'A':
            return
        if self.object_id in ('red_cube_3', 'red_cube_4'):
            # The pickup pose is north-east of moving_obstacle_1's immutable
            # y=2.8 rail.  A direct carried-object chord to A crosses that
            # physical obstacle and the 2026-09-09 trace launched the chassis
            # out of the map.  Retrace the proven west-end bypass before
            # entering A; the obstacle's map pose and motion stay untouched.
            rail_transits = (
                {'x': -2.75, 'y': 3.50, 'yaw': math.pi},
                {'x': -2.75, 'y': 2.00, 'yaw': -math.pi / 2.0},
            )
            for index, transit in enumerate(rail_transits, 1):
                self.publish_navigation_status(
                    phase='DROPOFF_TRANSIT', event='phase_start',
                    transit_index=index,
                    transit_count=len(rail_transits),
                    route_mode='red34_carried_rail_bypass')
                self.navigate(
                    f'{self.object_id} carried rail bypass {index}', transit,
                    handoff_distance=0.35, lock_route=True)
        elif self.object_id == 'red_cube_5':
            # The east side is disconnected in the static map; take the same
            # proven west-end crossing as red_cube_4 while carrying red 5.
            rail_transits = (
                {'x': -2.75, 'y': 3.50, 'yaw': math.pi},
                {'x': -2.75, 'y': 2.00, 'yaw': -math.pi / 2.0},
            )
            for index, transit in enumerate(rail_transits, 1):
                self.publish_navigation_status(
                    phase='DROPOFF_TRANSIT', event='phase_start',
                    transit_index=index,
                    transit_count=len(rail_transits),
                    route_mode='red5_carried_west_rail_bypass')
                self.navigate(
                    f'red_cube_5 west rail bypass {index}', transit,
                    handoff_distance=0.35, lock_route=True)
        # These two position handoffs constrain the chassis to the verified
        # east-side doorway.  NavigateThroughPoses was deliberately removed:
        # the default through-poses BT oscillated at the second waypoint
        # (0.00-0.24 m remaining) for the full 120 s timeout.  Explicit
        # position handoffs cost under a second each and have deterministic
        # braking ownership.
        transits = [
            {'x': -2.30, 'y': 1.30, 'yaw': -math.pi / 2.0},
            {'x': -2.25, 'y': -0.25, 'yaw': -math.pi / 2.0},
        ]
        if self.object_id in ('red_cube_3', 'red_cube_4'):
            # The verified west-rail bypass above already ends at
            # (-2.75, 2.00), on the safe west side of the moving obstacle.
            # Re-stopping only 0.83 m later at the outer A doorway forced an
            # unnecessary Nav2 cancel/replan cycle.  Keep the safety-critical
            # rail exit and the inner doorway constraint, but join them with
            # one continuous leg.  No map or clearance parameter is changed.
            transits = transits[1:]
            self.get_logger().info(
                'Carried west-rail exit merges with outer A doorway for '
                f'{self.object_id}')
        # Reverse the already-proven A egress corridor only once two cubes
        # occupy A.  The wall-side homotopy failure was observed on the third
        # approach with two stored cubes; forcing this detour for the first
        # two deliveries adds distance without providing collision clearance.
        # Count live, grounded inventory so mixed and inverse mappings use the
        # same rule instead of relying on colour or batch position.
        grounded_in_a = 0
        for name, pose in self.model_poses.items():
            if (name == self.object_id
                    or not name.startswith(('red_cube_', 'blue_cube_'))
                    or pose.position.z > 0.08):
                continue
            relative = self._relative_pose(name, 'zone_a')
            if (abs(relative.position.x) <= 0.50
                    and abs(relative.position.y) <= 0.25):
                grounded_in_a += 1
        if grounded_in_a >= 2:
            transits.append(
                {'x': -4.00, 'y': -1.75, 'yaw': math.pi})
        self.get_logger().info(
            f'A corridor inventory={grounded_in_a}; '
            f'transit_count={len(transits)}')
        for index, transit in enumerate(transits, 1):
            self.publish_navigation_status(
                phase='DROPOFF_TRANSIT', event='phase_start',
                transit_index=index, transit_count=len(transits),
                route_mode='doorway_position_handoff')
            self.navigate(
                f'A doorway transit {index}', transit,
                handoff_distance=0.35, lock_route=True)

    def check_pick_alignment(self):
        pose = self._relative_pose(self.object_id, 'six_arm')
        distance = math.hypot(pose.position.x, pose.position.y)
        self.get_logger().info(
            'Pickup alignment: object relative to six_arm '
            f'x={pose.position.x:.3f}, y={pose.position.y:.3f}, '
            f'distance={distance:.3f}m')
        if (not 0.36 <= pose.position.x <= 0.40
                or abs(pose.position.y) > 0.010
                or distance > 0.405):
            raise RuntimeError(
                'Pickup navigation ended outside the graspable window; '
                'do not command the arm')

    def fine_dock(self, timeout_sec=20.0):
        """Center a static cube with coupled range and heading control."""
        target_x = 0.385
        deadline = time.monotonic() + timeout_sec
        stable_samples = 0
        last_log = 0.0
        self.get_logger().info(
            'Starting fine dock: target object=(0.385m, 0.00m) in base frame')
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.01)
                x, y = self._dock_coordinates()
                range_error = math.hypot(x, y) - target_x
                distance_error = x - target_x
                heading_error = math.atan2(y, x)
                object_distance = math.hypot(x, y)
                if object_distance <= 0.28:
                    raise RuntimeError(
                        'Object entered chassis protection radius after '
                        'handoff; stopping instead of driving through it')
                # Centre more tightly than the arm's grasp window.  A 15 mm
                # lateral handoff left no margin for base reaction while the
                # arm descended; 6 mm remains quick but repeatable.
                aligned = (
                    abs(distance_error) <= 0.012
                    and abs(y) <= 0.006
                    and abs(heading_error) <= 0.018)
                if aligned:
                    stable_samples += 1
                    self._publish_dock_command(0.0, 0.0)
                    if stable_samples >= 4:
                        self.get_logger().info(
                            f'Fine dock succeeded: x={x:.3f}, y={y:.3f}')
                        return
                else:
                    stable_samples = 0
                    # Drive and steer together.  The former rotate-only gate
                    # kept linear.x at zero while the high angular command
                    # repeatedly overshot the cube centreline.
                    linear = max(-0.12, min(0.16, 1.4 * range_error))
                    if abs(range_error) > 0.015 and abs(linear) < 0.06:
                        linear = math.copysign(0.06, range_error)
                    # Explicit two-stage docking: first face the cube, then
                    # change range.  Coupled reverse+turn at large heading
                    # error orbited around blue_cube_3 and eventually put it
                    # beneath the chassis protection boundary.
                    if abs(heading_error) > 0.20:
                        linear = 0.0
                    else:
                        linear *= max(0.35, math.cos(heading_error) ** 2)
                    # Close the lateral error promptly once range is already
                    # correct.  The former 1.25 gain spent about three seconds
                    # creeping from |y|=0.036 m into the 0.015 m grasp window.
                    angular = max(-0.60, min(0.60, 2.40 * heading_error))
                    if (abs(heading_error) > 0.018
                            and abs(angular) < 0.065):
                        angular = math.copysign(0.065, heading_error)
                    if abs(heading_error) <= 0.004:
                        angular = 0.0
                    self._publish_dock_command(linear, angular)
                    now = time.monotonic()
                    if now - last_log >= 1.0:
                        self.get_logger().info(
                            'Fine dock control: '
                            f'x={x:.3f}, y={y:.3f}, '
                            f'range_error={range_error:.3f}, '
                            f'heading_error={heading_error:.3f}, '
                            f'cmd=({linear:.3f},{angular:.3f})')
                        last_log = now
                rclpy.spin_once(self, timeout_sec=0.10)
        finally:
            for _ in range(3):
                self._publish_dock_command(0.0, 0.0)
                rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError('Fine docking timed out before entering grasp window')

    def _publish_dock_command(self, linear, angular):
        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        self.dock_publisher.publish(command)

    def fine_dropoff_approach(
            self, target, timeout_sec=26.0, target_tolerance=0.15):
        """Finish the carried-object dock slowly after the Nav2 handoff.

        The controller operates only inside the final roughly 0.6 m.  It
        first faces the already-planned dock point, then advances at no more
        than 0.16 m/s.  This matches the already validated straight pickup
        dock cap; sharp approaches still rotate in place and final yaw remains
        a separate stopped operation, so the
        payload cannot sweep through a zone obstacle during translation.
        """
        target_x = float(target['x'])
        target_y = float(target['y'])
        deadline = time.monotonic() + float(timeout_sec)
        stable_since = None
        last_log = 0.0
        initial = self._await_fresh_robot_pose()
        initial_distance = math.hypot(
            initial.position.x - target_x, initial.position.y - target_y)
        if initial_distance > 1.05:
            raise RuntimeError(
                f'Destination handoff occurred too early '
                f'({initial_distance:.3f}m); refusing open-loop approach')
        self.get_logger().info(
            f'Starting low-speed destination approach: '
            f'distance={initial_distance:.3f}m, '
            f'target_tolerance={target_tolerance:.2f}m')
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.04)
                # Model states can miss a publish interval when Gazebo is
                # below real time.  Wait for one genuinely fresh sample
                # rather than failing on the first 350 ms gap; the helper
                # still refuses data older than the original safety limit.
                robot = self._await_fresh_robot_pose()
                if not -0.05 <= float(robot.position.z) <= 0.08:
                    raise RuntimeError(
                        'Chassis left the floor during destination approach; '
                        'stopping before placement')
                dx = target_x - float(robot.position.x)
                dy = target_y - float(robot.position.y)
                distance = math.hypot(dx, dy)
                q = robot.orientation
                yaw = math.atan2(
                    2.0 * (q.w*q.z + q.x*q.y),
                    1.0 - 2.0 * (q.y*q.y + q.z*q.z))
                path_heading = math.atan2(dy, dx)
                heading_error = math.atan2(
                    math.sin(path_heading - yaw),
                    math.cos(path_heading - yaw))
                now = time.monotonic()
                if distance <= target_tolerance:
                    self._publish_dock_command(0.0, 0.0)
                    stable_since = stable_since or now
                    if now - stable_since >= 0.45:
                        self.get_logger().info(
                            f'Low-speed destination approach complete: '
                            f'distance={distance:.3f}m')
                        return
                else:
                    stable_since = None
                    # Do not combine a sharp turn with payload translation.
                    # Once facing the dock, use a mild coupled correction.
                    # Small destination-heading errors are safe to correct
                    # while advancing.  Stopping translation at 0.18 rad
                    # caused a repeated turn/go/turn hesitation near both
                    # scoring zones; reserve in-place rotation for genuinely
                    # sharp approaches.
                    if abs(heading_error) > 0.28:
                        linear = 0.0
                    else:
                        linear = max(0.045, min(0.16, 0.55 * distance))
                        linear *= max(0.45, math.cos(heading_error) ** 2)
                    angular = max(-0.22, min(0.22, 1.35 * heading_error))
                    if abs(heading_error) <= 0.015:
                        angular = 0.0
                    self._publish_dock_command(linear, angular)
                    if now - last_log >= 1.0:
                        self.get_logger().info(
                            f'Low-speed destination control: '
                            f'distance={distance:.3f}, '
                            f'heading_error={heading_error:.3f}, '
                            f'cmd=({linear:.3f},{angular:.3f})')
                        last_log = now
            raise RuntimeError(
                'Low-speed destination approach timed out before parking')
        finally:
            self._publish_navigation_stop()

    def align_dropoff_heading(self, target_yaw, timeout_sec=10.0):
        """Precisely align the chassis before the arm extends for placement."""
        deadline = time.monotonic() + float(timeout_sec)
        stable_since = None
        last_log = 0.0
        # A 0.050 rad (2.9 deg) heading error displaces the carried cube by
        # at most about 20 mm at 0.40 m reach, still well inside the narrowest
        # validated slot margin.  Avoid spending repeated stop time chasing
        # sub-degree corrections before and after the straight final approach.
        tolerance = 0.050
        self.get_logger().info(
            f'Starting dropoff heading alignment: target={target_yaw:.3f}rad')
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.04)
                robot = self._await_fresh_robot_pose()
                q = robot.orientation
                yaw = math.atan2(
                    2.0 * (q.w*q.z + q.x*q.y),
                    1.0 - 2.0 * (q.y*q.y + q.z*q.z))
                error = math.atan2(
                    math.sin(float(target_yaw) - yaw),
                    math.cos(float(target_yaw) - yaw))
                now = time.monotonic()
                if abs(error) <= tolerance:
                    self._publish_dock_command(0.0, 0.0)
                    stable_since = stable_since or now
                    if now - stable_since >= 0.30:
                        self.get_logger().info(
                            f'Dropoff heading aligned: yaw={yaw:.3f}, '
                            f'error={error:.3f}rad')
                        return
                else:
                    stable_since = None
                    # The base is already stopped for this in-place alignment.
                    # A small cap increase trims repeated pre-place rotation time
                    # without changing carried translation speed or cornering.
                    angular = max(-0.65, min(0.65, 1.50 * error))
                    if abs(angular) < 0.07:
                        angular = math.copysign(0.07, error)
                    self._publish_dock_command(0.0, angular)
                    if now - last_log >= 1.0:
                        self.get_logger().info(
                            f'Dropoff heading control: yaw={yaw:.3f}, '
                            f'error={error:.3f}, cmd_angular={angular:.3f}')
                        last_log = now
            raise RuntimeError(
                'Dropoff heading did not settle; refusing arm placement')
        finally:
            self._publish_navigation_stop()

    def egress_after_place(self, travel_distance=0.45, timeout_sec=9.0):
        """Back straight away from a placed cube before starting another task.

        This is deliberately opt-in: the final task still settles in place.
        The manoeuvre keeps the placement heading, verifies that separation
        from the released cube is increasing, and stops on stale Gazebo data
        or lack of progress instead of forcing the chassis through an object.
        """
        robot = self._await_fresh_robot_pose()
        cube = self.model_poses.get(self.object_id)
        if cube is None:
            raise RuntimeError('Placed object missing before B egress')
        start_x = float(robot.position.x)
        start_y = float(robot.position.y)
        q = robot.orientation
        start_yaw = math.atan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        away_x = robot.position.x - cube.position.x
        away_y = robot.position.y - cube.position.y
        initial_separation = math.hypot(away_x, away_y)
        # Gazebo's base model axis is not guaranteed to match the visible arm
        # side.  Select forward or reverse from live geometry so the first
        # command is mathematically away from the released cube.
        heading_dot_away = (
            math.cos(start_yaw) * away_x + math.sin(start_yaw) * away_y)
        linear_command = 0.25 if heading_dot_away >= 0.0 else -0.25
        deadline = time.monotonic() + float(timeout_sec)
        last_progress = time.monotonic()
        best_travel = 0.0
        self.get_logger().info(
            f'Starting post-place egress: initial cube separation='
            f'{initial_separation:.3f}m, target travel={travel_distance:.3f}m, '
            f'direction={"forward" if linear_command > 0.0 else "reverse"}')
        self.publish_navigation_status(
            phase='POST_PLACE_EGRESS', event='phase_start',
            egress_target_distance=float(travel_distance))
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                sample_wait_start = time.monotonic()
                if time.monotonic() - self.models_received > 0.35:
                    self._publish_dock_command(0.0, 0.0)
                robot = self._await_fresh_robot_pose()
                # A telemetry pause is a stopped safety wait, not failed base
                # motion; exclude it from the 1.5 s progress watchdog.
                last_progress += time.monotonic() - sample_wait_start
                cube = self.model_poses.get(self.object_id)
                if cube is None:
                    raise RuntimeError('Placed object disappeared during egress')
                travelled = math.hypot(
                    robot.position.x - start_x, robot.position.y - start_y)
                separation = math.hypot(
                    robot.position.x - cube.position.x,
                    robot.position.y - cube.position.y)
                if separation < initial_separation - 0.025:
                    raise RuntimeError(
                        'Post-place egress moved toward the released cube')
                if travelled > best_travel + 0.008:
                    best_travel = travelled
                    last_progress = time.monotonic()
                if travelled >= float(travel_distance):
                    self.get_logger().info(
                        f'Post-place egress succeeded: travelled={travelled:.3f}m, '
                        f'cube separation={separation:.3f}m')
                    self.publish_navigation_status(
                        event='egress_complete', egress_distance=travelled,
                        cube_separation=separation)
                    return
                if time.monotonic() - last_progress > 1.5:
                    raise RuntimeError(
                        'Post-place egress made no progress; stopping base')
                q = robot.orientation
                yaw = math.atan2(
                    2.0 * (q.w*q.z + q.x*q.y),
                    1.0 - 2.0 * (q.y*q.y + q.z*q.z))
                yaw_error = math.atan2(
                    math.sin(start_yaw - yaw), math.cos(start_yaw - yaw))
                angular = max(-0.16, min(0.16, 1.2 * yaw_error))
                self._publish_dock_command(linear_command, angular)
            raise RuntimeError('Post-place egress timed out')
        finally:
            self._publish_navigation_stop()

    def navigate_c_post_place_exit(self):
        """Leave C through the same guarded corridor used on entry."""
        if self.destination != 'C':
            return
        # A direct next-pick goal lets Nav2 choose a shorter-looking path
        # through the fixed wall and obstacle-2 rail.  Retrace the proven
        # corridor to its north-west staging point before normal selection of
        # the next cube.  Wait on the east side before crossing the rail.
        crossing_mode = self._choose_c_exit_crossing()
        if crossing_mode == 'south':
            before_crossing = (
                {'x': 3.90, 'y': -6.55, 'yaw': math.pi,
                 'handoff': 0.20, 'strict_handoff': True},
            )
            after_crossing = (
                {'x': 0.35, 'y': -6.55, 'yaw': math.pi / 2.0,
                 'handoff': 0.20, 'strict_handoff': True},
                {'x': 0.35, 'y': -1.00, 'yaw': 0.0},
            )
        else:
            before_crossing = (
                {'x': 3.90, 'y': -6.55, 'yaw': math.pi,
                 'handoff': 0.20, 'strict_handoff': True},
                {'x': 3.00, 'y': -6.55, 'yaw': math.pi / 2.0,
                 'handoff': 0.08, 'strict_handoff': True},
                {'x': 3.00, 'y': -3.60, 'yaw': math.pi,
                 'handoff': 0.10, 'strict_handoff': True},
            )
            after_crossing = (
                {'x': 0.35, 'y': -3.60, 'yaw': math.pi / 2.0},
                {'x': 0.35, 'y': -1.00, 'yaw': 0.0},
            )
        transits = before_crossing + after_crossing
        self.get_logger().info(
            'Starting guarded C post-place exit before next pickup: '
            f'locked_crossing={crossing_mode}')
        for index, transit in enumerate(transits, 1):
            if index == len(before_crossing) + 1:
                if crossing_mode == 'north':
                    self._wait_for_c_north_crossing_clear()
                else:
                    self._wait_for_c_south_crossing_clear()
            self.publish_navigation_status(
                phase='POST_PLACE_TRANSIT', event='phase_start',
                transit_index=index, transit_count=len(transits),
                route_mode=f'c_guarded_{crossing_mode}_exit')
            self.navigate(
                f'C post-place exit {index}', transit,
                handoff_distance=transit.get('handoff', 0.30),
                strict_handoff=transit.get('strict_handoff', False),
                lock_route=True)
        self.get_logger().info('Guarded C post-place exit complete')

    def check_drop_alignment(self):
        zone = ZONE_MODELS[self.destination]
        pose = self._relative_pose(self.object_id, zone)
        self.get_logger().info(
            f'Drop alignment relative to {zone}: '
            f'x={pose.position.x:.3f}, y={pose.position.y:.3f}')
        if abs(pose.position.x) > 0.50 or abs(pose.position.y) > 0.25:
            raise RuntimeError(
                'Carried object is not inside the configured placement zone; '
                'keeping it attached')

    @staticmethod
    def _pose_roll_pitch(pose):
        q = pose.orientation
        roll = math.atan2(
            2.0 * (q.w*q.x + q.y*q.z),
            1.0 - 2.0 * (q.x*q.x + q.y*q.y))
        pitch = math.asin(max(-1.0, min(1.0,
            2.0 * (q.w*q.y - q.z*q.x))))
        return roll, pitch

    def validate_destination_inventory(self, settle_sec=0.60):
        """Recheck all stored cubes after a placement, including older ones."""
        deadline = time.monotonic() + float(settle_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            self._publish_dock_command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)
        if time.monotonic() - self.models_received > 0.35:
            raise RuntimeError('Gazebo model state stale during inventory validation')

        zone = ZONE_MODELS[self.destination]
        checked = []
        checked_positions = []
        failures = []
        for name, world_pose in sorted(self.model_poses.items()):
            if (not name.startswith(('red_cube_', 'blue_cube_'))
                    or world_pose.position.z > 0.08):
                continue
            local = self._relative_pose(name, zone)
            # Expanded envelope catches a cube knocked just outside the zone.
            if abs(local.position.x) > 0.60 or abs(local.position.y) > 0.35:
                continue
            roll, pitch = self._pose_roll_pitch(world_pose)
            tilt = max(abs(roll), abs(pitch))
            twist = self.model_twists.get(name)
            speed = float('inf') if twist is None else math.sqrt(
                twist.linear.x**2 + twist.linear.y**2 + twist.linear.z**2)
            valid = (
                abs(local.position.x) <= 0.475
                and abs(local.position.y) <= 0.225
                and 0.012 <= world_pose.position.z <= 0.020
                and tilt <= 0.12
                and speed <= 0.03)
            checked.append(name)
            checked_positions.append(
                (name, local.position.x, local.position.y))
            self.get_logger().info(
                f'Inventory 3D check {name}: local='
                f'({local.position.x:.3f},{local.position.y:.3f}), '
                f'z={world_pose.position.z:.4f}m, tilt={tilt:.4f}rad, '
                f'speed={speed:.4f}m/s, valid={valid}')
            if not valid:
                failures.append(name)
        for index, (name_a, ax, ay) in enumerate(checked_positions):
            for name_b, bx, by in checked_positions[index + 1:]:
                separation = math.hypot(ax - bx, ay - by)
                self.get_logger().info(
                    f'Inventory pair clearance {name_a}/{name_b}: '
                    f'{separation:.3f}m')
                if separation < 0.05:
                    failures.append(
                        f'{name_a}/{name_b}:clearance={separation:.3f}m')
        if self.object_id not in checked:
            failures.append(f'{self.object_id}:missing_from_zone')
        if failures:
            raise RuntimeError(
                'Destination inventory validation failed: '
                + ', '.join(failures))

    def compute_dropoff_target(self, configured_target):
        """Approach the nearest safe side and place the cube inside the zone."""
        zone = ZONE_MODELS[self.destination]
        carried = self._relative_pose(self.object_id, 'six_arm')
        zone_world = self._relative_pose(zone, 'world')
        robot = self._relative_pose('six_arm', 'world')

        orientation = zone_world.orientation
        zone_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y
                         + orientation.z * orientation.z))
        cos_zone = math.cos(zone_yaw)
        sin_zone = math.sin(zone_yaw)
        robot_dx = robot.position.x - zone_world.position.x
        robot_dy = robot.position.y - zone_world.position.y
        robot_zone_x = cos_zone * robot_dx + sin_zone * robot_dy
        robot_zone_y = -sin_zone * robot_dx + cos_zone * robot_dy

        # The scoring zone is 1.00 x 0.50 m.  Keep generous margins while
        # selecting the point closest to the incoming robot, instead of always
        # forcing it to circle around to a fixed-yaw centre pose.
        # Leave room for the 0.15 m Nav2 position tolerance.  The old
        # (+0.36,+0.13) edge slot produced an observed x=+0.536 m and the
        # safety check correctly refused to release the cube.
        if self.destination in ('A', 'B', 'C'):
            # Reuse the proven Git slot concept without importing its world
            # plugin: choose a centre-line slot from live Gazebo truth and
            # keep one cube width plus 55 mm surface clearance from cargo
            # already stored in B.
            occupied = []
            for name, pose in self.model_poses.items():
                if (name == self.object_id
                        or not (name.startswith('red_cube_')
                                or name.startswith('blue_cube_'))
                        or pose.position.z > 0.08):
                    continue
                dx = pose.position.x - zone_world.position.x
                dy = pose.position.y - zone_world.position.y
                local_x = cos_zone * dx + sin_zone * dy
                local_y = -sin_zone * dx + cos_zone * dy
                if abs(local_x) <= 0.50 and abs(local_y) <= 0.25:
                    occupied.append((local_x, local_y, name))
            # Keep every chassis dock on the zone's open east side
            # (local x=+0.18)
            # and distribute cargo across the short y axis.  Moving the slot
            # along x also moved the chassis into previously released cubes;
            # five lateral slots preserve one repeatable x≈-2.0 parking line.
            if self.destination == 'A':
                if self.object_id.startswith('red_cube_'):
                    # Restore the two-row layout that completed the five-red
                    # cumulative regression on 2026-09-07.  Red grasp offsets
                    # can sweep a nominal x=0.18 landing to x=0.375; in a
                    # single row that one cube makes every middle chassis dock
                    # lethal.  The staggered inner row preserves a second
                    # approach line while retaining the strict inventory gate.
                    candidates = [
                        (0.08, -0.13), (0.08, 0.05), (0.08, 0.18),
                        (0.28, -0.13), (0.28, 0.16), (0.20, 0.02),
                    ]
                else:
                    # Blue cubes have a larger observed post-release sweep;
                    # keep their verified inset one-row geometry.
                    candidates = [
                        (0.18, -0.18), (0.18, -0.09), (0.18, 0.00),
                        (0.18, 0.09),
                    ]
            elif self.destination == 'B':
                # B is not constrained to a single row.  Its floor lettering
                # makes some otherwise regular y positions lethal for the
                # chassis, so retain a two-row interior candidate grid and let
                # the live cargo-clearance and chassis-cost filters choose at
                # most the four physically permitted placements.  These are
                # the previously proven B coordinates; no map geometry or
                # manipulation trajectory is changed here.
                candidates = [
                    (0.08, -0.13), (0.08, 0.05), (0.08, 0.13),
                    (0.18, -0.13), (0.18, 0.10), (0.13, 0.02),
                ]
            else:
                # C is approached only from its open north side.  Keep every
                # slot from the north with the same straight-arm IK.  A single
                # four-wide row left the last chassis dock inside cost 99 after
                # three valid placements, so use two well-separated columns
                # and two depths.  The north row gets a tighter final docking
                # tolerance below; this prevents the previously observed
                # +0.230 landing from exceeding the true +0.225 boundary.
                candidates = [
                    (-0.16, -0.18), (0.16, -0.18),
                    (-0.16, 0.05), (0.16, 0.05),
                ]
            preferred_indices = {
                'A': ({5: 1, 2: 2, 3: 0, 1: 4, 4: 3}
                      if self.object_id.startswith('red_cube_')
                      else {4: 0, 3: 1, 2: 2, 5: 3, 1: 3}),
                'B': {5: 1, 4: 2, 3: 0, 2: 4, 1: 3},
                'C': {5: 0, 4: 1, 3: 2, 2: 3, 1: 0},
            }[self.destination]
            # An actual landing point can differ from its nominal candidate
            # because Nav2 has finite pose tolerance.  Merely measuring the
            # distance back to candidates allowed that nominal slot to be
            # selected again on a later invocation.  Reserve the closest
            # nominal slot for every stored cube, in addition to the live
            # clearance check.
            claims = self._load_slot_claims()
            zone_claims = claims.get(self.destination, {})
            if not isinstance(zone_claims, dict):
                zone_claims = {}
            occupied_names = {name for _ox, _oy, name in occupied}
            zone_claims = {
                name: int(index)
                for name, index in zone_claims.items()
                if name in occupied_names
                and isinstance(index, int)
                and 0 <= index < len(candidates)
            }
            claims[self.destination] = zone_claims
            self._save_slot_claims(claims)
            reserved = set(zone_claims.values())
            unclaimed_occupied = [
                (ox, oy, name) for ox, oy, name in occupied
                if name not in zone_claims
            ]
            reserved.update(
                preferred_indices[int(name.rsplit('_', 1)[1])]
                for _ox, _oy, name in unclaimed_occupied
                if int(name.rsplit('_', 1)[1]) in preferred_indices
            )
            # A lethal chassis cell can force an object away from its
            # number-based preferred slot.  Reserve the closest nominal slot
            # to every live landing as well; otherwise the next short-lived
            # process forgets that fallback choice and may reuse it.
            reserved.update(
                min(
                    range(len(candidates)),
                    key=lambda index: math.hypot(
                        candidates[index][0] - ox,
                        candidates[index][1] - oy),
                )
                for ox, oy, _name in unclaimed_occupied
            )
            minimum_clearance = 0.05
            safe = [
                candidate for index, candidate in enumerate(candidates)
                if index not in reserved
                and all(math.hypot(candidate[0] - ox,
                                   candidate[1] - oy) >= minimum_clearance
                        for ox, oy, _name in occupied)
            ]
            # A cargo slot can be clear while its corresponding chassis dock
            # lies inside the inflated obstacle cell of an earlier delivery.
            # Evaluate the base pose for every candidate now, so a lethal
            # preferred slot falls through to another slot instead of failing
            # only after the B transit has already completed.
            dock_yaw = (
                -math.pi / 2.0 if self.destination == 'C' else math.pi)
            carried_world_dx = (
                math.cos(dock_yaw) * carried.position.x
                - math.sin(dock_yaw) * carried.position.y)
            carried_world_dy = (
                math.sin(dock_yaw) * carried.position.x
                + math.cos(dock_yaw) * carried.position.y)
            cost_safe = []
            rejected_costs = []
            for candidate in safe:
                candidate_world_x = (
                    zone_world.position.x
                    + cos_zone * candidate[0] - sin_zone * candidate[1])
                candidate_world_y = (
                    zone_world.position.y
                    + sin_zone * candidate[0] + cos_zone * candidate[1])
                dock_x = candidate_world_x - carried_world_dx
                dock_y = candidate_world_y - carried_world_dy
                dock_cost = self._global_costmap_cell(dock_x, dock_y)
                if dock_cost is not None and dock_cost >= 99:
                    rejected_costs.append(
                        (candidate, dock_cost, dock_x, dock_y))
                else:
                    cost_safe.append(candidate)
            if rejected_costs:
                self.get_logger().warning(
                    f'{self.destination} slots rejected by chassis cost: '
                    f'{[(slot, cost, round(x, 3), round(y, 3)) for slot, cost, x, y in rejected_costs]}')
            safe = cost_safe
            if not safe:
                raise RuntimeError(
                    f'No cargo-and-chassis-safe {self.destination} placement slot '
                    'remains; holding object')
            preferred = None
            # Stable one-to-one claims survive separate per-cube test
            # processes.  Inferring a claim from the actual landing point is
            # ambiguous because arm sweep can move a cube closer to an
            # adjacent nominal slot.
            suffix = int(self.object_id.rsplit('_', 1)[1])
            preferred_index = preferred_indices.get(suffix)
            if self.destination == 'A' and not occupied:
                # Always seed an empty A zone at an outer lateral slot.  A
                # middle first placement inflates all three remaining docks
                # to lethal cost; starting at the edge leaves the opposite
                # edge and centre reachable regardless of which cube number
                # happens to be nearest first.
                preferred = candidates[0]
            elif preferred_index is not None:
                preferred = candidates[preferred_index]
            if preferred in safe:
                object_zone_x, object_zone_y = preferred
            elif occupied:
                # Maximise the nearest existing-cube clearance.  The robot's
                # incoming side must not pull successive cubes into one corner.
                object_zone_x, object_zone_y = max(
                    safe,
                    key=lambda candidate: min(
                        math.hypot(candidate[0] - ox,
                                   candidate[1] - oy)
                        for ox, oy, _name in occupied))
            else:
                object_zone_x, object_zone_y = {
                    'A': (0.18, 0.0),
                    'B': (0.18, 0.0),
                    'C': (-0.16, -0.10),
                }[self.destination]
            selected_index = min(
                range(len(candidates)),
                key=lambda index: math.hypot(
                    candidates[index][0] - object_zone_x,
                    candidates[index][1] - object_zone_y))
            zone_claims[self.object_id] = selected_index
            claims[self.destination] = zone_claims
            self._save_slot_claims(claims)
            self.selected_dropoff_slot = (
                float(object_zone_x), float(object_zone_y))
            self.get_logger().info(
                f'{self.destination} slot selection: '
                f'selected=({object_zone_x:.2f},'
                f'{object_zone_y:.2f}), claim={selected_index}, '
                f'reserved={sorted(reserved)}, occupied='
                f'{[(name, round(x, 3), round(y, 3)) for x, y, name in occupied]}')
        object_world_x = (
            zone_world.position.x
            + cos_zone * object_zone_x - sin_zone * object_zone_y)
        object_world_y = (
            zone_world.position.y
            + sin_zone * object_zone_x + cos_zone * object_zone_y)

        to_slot_x = object_world_x - robot.position.x
        to_slot_y = object_world_y - robot.position.y
        if self.destination == 'A':
            # The collision-free corridor reaches zone A from east to west.
            # Align with its final tangent instead of rotating toward the
            # start-to-goal chord after already reaching the zone.
            yaw = math.pi
        elif self.destination == 'B':
            # Park on the east side of B and extend the arm straight west,
            # matching the pickup-style forward reach.  A yaw chosen from the
            # incoming chord left the chassis facing the north obstacle after
            # release and every following Nav2 command was collision-rejected.
            # This fixed heading also leaves a straight reverse egress to the
            # open east corridor when another object remains.
            yaw = math.pi
        elif self.destination == 'C':
            # Face south so the arm extends straight into C from its open
            # north side, mirroring the stable pickup/place geometry.
            yaw = -math.pi / 2.0
        elif math.hypot(to_slot_x, to_slot_y) > 0.05:
            yaw = math.atan2(to_slot_y, to_slot_x)
        else:
            yaw = float(configured_target['yaw'])
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        world_dx = cos_yaw * carried.position.x - sin_yaw * carried.position.y
        world_dy = sin_yaw * carried.position.x + cos_yaw * carried.position.y
        world_target_x = object_world_x - world_dx
        world_target_y = object_world_y - world_dy
        target = {
            # Place the carried cube at the selected near-side slot.  Using
            # the zone centre here silently discarded object_world_{x,y} and
            # made the goal position inconsistent with the incoming yaw,
            # producing a long terminal rotation after the cube was already
            # inside the scoring area.
            'x': world_target_x,
            'y': world_target_y,
            'yaw': yaw,
        }
        self.get_logger().info(
            f'Nearest-side drop dock for {zone}: '
            f'base=({target["x"]:.3f},{target["y"]:.3f}), '
            f'object_slot=({object_zone_x:.3f},{object_zone_y:.3f}), '
            f'yaw={yaw:.3f}, '
            f'carried_offset=({carried.position.x:.3f},'
            f'{carried.position.y:.3f})')
        return target

    def _relative_pose(self, entity, reference):
        last_error = None
        for attempt in range(1, 4):
            request = GetEntityState.Request()
            request.name = entity
            request.reference_frame = reference
            future = self.state_client.call_async(request)
            try:
                self._wait_future(
                    future, 5.0,
                    f'query {entity} relative to {reference}')
                response = future.result()
                if response is not None and response.success:
                    return response.state.pose
                last_error = RuntimeError(
                    f'Cannot query {entity} relative to {reference}')
            except RuntimeError as error:
                last_error = error
            if attempt < 3:
                self.get_logger().warning(
                    f'Gazebo entity query delayed for {entity}; '
                    f'retrying ({attempt}/3)')
                time.sleep(0.5)
        raise last_error

    def _wait_future(self, future, timeout, description):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(f'Timeout while waiting for {description}')
        if future.exception() is not None:
            raise RuntimeError(
                f'{description} raised: {future.exception()}')

    def _navigation_feedback(self, message):
        feedback = message.feedback
        self._navigation_feedback_distance = float(feedback.distance_remaining)
        self._navigation_feedback_recoveries = int(feedback.number_of_recoveries)
        pose = feedback.current_pose.pose.position
        self.publish_navigation_status(
            event='feedback',
            distance_remaining=float(feedback.distance_remaining),
            recoveries=int(feedback.number_of_recoveries),
            feedback_x=float(pose.x), feedback_y=float(pose.y))
        now = time.monotonic()
        if now - self._last_feedback_log < 2.0:
            return
        self._last_feedback_log = now
        self.get_logger().info(
            f'Nav2 remaining={feedback.distance_remaining:.2f}m, '
            f'recoveries={feedback.number_of_recoveries}')

    def _manipulation_feedback(self, message):
        feedback = message.feedback
        self.get_logger().info(
            f'Arm stage={feedback.stage}, progress={feedback.progress:.0%}')


def load_targets(destination, requested_object=None):
    share = get_package_share_directory('moon_warehouse_coordinator')
    path = f'{share}/config/task_execution.yaml'
    with open(path, 'r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)['flow_execution']
    all_approaches = config['object_approaches']
    destinations = config['destinations']
    if destination not in destinations or destination not in ZONE_MODELS:
        raise RuntimeError(f'No destination configured for {destination}')

    # The competition mapping assigns at most five cubes to one destination.
    # Restrict generic selectors such as ``nearest`` to that legal set instead
    # of allowing a geometrically close cube of the wrong colour to win.
    mapping_path = f'{share}/config/task_mapping.yaml'
    with open(mapping_path, 'r', encoding='utf-8') as stream:
        mapping = yaml.safe_load(stream)['mission_mapping']
    assigned = []
    for rule in mapping.values():
        if str(rule.get('destination', '')).upper() == destination:
            assigned.extend(str(name) for name in rule.get('objects', []))
    # CoStudio can dynamically map either colour to either destination.  A
    # colour-qualified selector therefore draws from all cubes of that colour;
    # fixed IDs and the normal static mapping retain their old behaviour.
    dynamic_colour = None
    if requested_object in ('nearest-red', 'fastest-red'):
        dynamic_colour = 'red_cube_'
    elif requested_object in ('nearest-blue', 'fastest-blue'):
        dynamic_colour = 'blue_cube_'
    if dynamic_colour is not None:
        assigned = [
            name for name in all_approaches
            if str(name).startswith(dynamic_colour)
        ]

    # An explicit acceptance command may intentionally test a colour/zone
    # pairing outside the official mapping.  Permit that one named cube while
    # keeping generic nearest/fastest selectors restricted to the competition
    # mapping, so the normal red->A and blue->B choice/order is unchanged.
    if (requested_object in all_approaches
            and requested_object not in assigned):
        assigned.append(requested_object)
    if not assigned:
        raise RuntimeError(
            f'No competition objects are assigned to destination {destination}')
    missing = [name for name in assigned if name not in all_approaches]
    if missing:
        raise RuntimeError(
            'Missing approach poses for assigned objects: ' + ', '.join(missing))
    approaches = {name: all_approaches[name] for name in assigned}
    return approaches, destinations[destination]


def execute_one_task(node, requested_object, destination,
                     pickup_handoff_distance, egress_after_place=False,
                     select_only=False):
    """Execute one item while allowing the ROS node to be reused in a batch."""
    node.object_id = requested_object
    node.destination = destination
    node.navigation_status = {
        'phase': 'INITIALIZING',
        'event': 'task_started',
        'label': '',
    }
    approaches, dropoff = load_targets(destination, requested_object)
    node.wait_for_state_service()
    selected = node.select_object(requested_object, approaches)
    node.object_id = selected
    node.publish_navigation_status(
        phase='SELECTED', event='object_selected', object_id=selected,
        destination=destination)
    if select_only:
        print(f'SELECTED_OBJECT={selected}', flush=True)
        return selected
    node.wait_for_interfaces()
    node.configure_pickup_tracking()
    pickup = node.nearest_dock_target(selected, approaches[selected])
    node.navigate_pickup_transits(pickup)
    node.publish_navigation_status(phase='NAV_PICKUP', event='phase_start')
    node.navigate(
        f'pickup {selected}', pickup,
        handoff_distance=pickup_handoff_distance)
    node.publish_navigation_status(phase='FINE_DOCK', event='phase_start')
    node.fine_dock()
    node.check_pick_alignment()
    node.publish_navigation_status(phase='PICK', event='phase_start')
    try:
        node.manipulate('pick')
    except ManipulationFailure as error:
        if error.error_code not in (5, 7):
            raise
        node.get_logger().warning(
            f'Pick geometry changed after a failed grasp '
            f'(code={error.error_code}); re-running one live fine dock '
            'before the final pick attempt')
        node.publish_navigation_status(
            phase='FINE_DOCK', event='post_grasp_reacquire',
            manipulation_error=error.error_code)
        node.fine_dock()
        node.check_pick_alignment()
        node.manipulate('pick')
    node.configure_dropoff_tracking()
    dropoff = node.compute_dropoff_target(dropoff)
    node.navigate_dropoff_transits()
    node.publish_navigation_status(
        phase='NAV_DROPOFF', event='phase_start')
    if destination in ('A', 'B', 'C'):
        # The permissive Nav2 goal checker can report success at a scoring
        # slot with the chassis still facing away from it or with enough
        # lateral position error to put the carried cube outside the narrow
        # half-depth.  Establish the final west-facing heading at a clear
        # east-side pre-approach, then let the bounded controller enter the
        # slot almost straight.  This is needed for both A and B; each uses
        # its own computed slot and base target.
        pre_approach_offset = {
            'A': 0.25,
            'B': 0.55,
            'C': 0.45,
        }[destination]
        pre_approach = {
            'x': (float(dropoff['x'])
                  - pre_approach_offset * math.cos(float(dropoff['yaw']))),
            'y': (float(dropoff['y'])
                  - pre_approach_offset * math.sin(float(dropoff['yaw']))),
            'yaw': float(dropoff['yaw']),
        }
        node.navigate(
            f'{destination} final pre-approach', pre_approach,
            handoff_distance=0.20, lock_route=True,
            no_progress_timeout=(
                6.0 if destination in ('A', 'B') else None))
        node.align_dropoff_heading(float(dropoff['yaw']))
    else:
        node.navigate(f'destination {destination}', dropoff, lock_route=True)
    node.publish_navigation_status(
        phase='DROPOFF_FINE_APPROACH', event='phase_start')
    dropoff_tolerance = 0.15
    if (destination == 'C'
            and node.selected_dropoff_slot is not None
            and node.selected_dropoff_slot[1] > 0.0):
        # The C north row has only 0.175 m from its nominal centre to the
        # manipulation server's positive-y release boundary.  Finish closer
        # to the computed base target so residual chassis error cannot push a
        # correctly oriented cube beyond that boundary.
        dropoff_tolerance = 0.09
    node.fine_dropoff_approach(
        dropoff, target_tolerance=dropoff_tolerance)
    node.publish_navigation_status(
        phase='DROPOFF_ALIGN', event='phase_start')
    node.align_dropoff_heading(float(dropoff['yaw']))
    node.check_drop_alignment()
    node.publish_navigation_status(phase='PLACE', event='phase_start')
    node.manipulate('place')
    node.validate_destination_inventory()
    if egress_after_place:
        node.egress_after_place()
        node.navigate_c_post_place_exit()
    node.publish_navigation_status(phase='COMPLETE', event='acceptance_passed')
    node.get_logger().info(
        'ACCEPTANCE PASSED: navigation + pick + carry + place')
    return selected


def optimize_batch_order(tasks):
    """Return the evidence-backed zone order while preserving task identity.

    Repeated full-chain logs show that leaving A to serve B creates the most
    expensive cross-map transition, while leaving C between items pays the
    guarded corridor egress a second time.  A stable B -> A -> C ordering
    therefore removes both penalties without changing any map, safety, or
    manipulation setting.  Relative order inside one destination is retained;
    live ``fastest-*`` selection still chooses the lowest whole-mission-cost
    cube when that item begins.
    """
    zone_rank = {'B': 0, 'A': 1, 'C': 2}
    return sorted(
        tasks,
        key=lambda task: zone_rank.get(str(task[1]).upper(), 99))


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Navigate, pick one cube, navigate, and place it.')
    parser.add_argument('--object', default='nearest')
    parser.add_argument('--destination', default='A', choices=('A', 'B', 'C'))
    parser.add_argument('--navigation-timeout', type=float, default=180.0)
    parser.add_argument('--manipulation-timeout', type=float, default=60.0)
    parser.add_argument(
        '--pickup-handoff-distance', type=float, default=0.70,
        help='Distance from the coarse pickup dock pose before fine dock takes '
             'control (metres).')
    parser.add_argument('--select-only', action='store_true')
    parser.add_argument(
        '--egress-after-place', action='store_true',
        help='After a successful non-final placement, reverse straight away '
             'from the released cube before returning success.')
    parser.add_argument(
        '--batch', default='',
        help='Comma-separated object:destination items. Reuses one ROS node '
             'and automatically omits egress after the final item.')
    parser.add_argument(
        '--preserve-batch-order', action='store_true',
        help='Run the supplied batch literally (directed regression only). '
             'Normal competition batches use the validated B -> A -> C order.')
    parsed, ros_args = parser.parse_known_args(args)
    destination = parsed.destination.upper()

    tasks = []
    if parsed.batch:
        for item in parsed.batch.split(','):
            fields = item.strip().rsplit(':', 1)
            if len(fields) != 2 or fields[1].upper() not in ZONE_MODELS:
                parser.error(
                    f'Invalid --batch item {item!r}; expected object:A|B|C')
            tasks.append((fields[0], fields[1].upper()))
    else:
        tasks.append((parsed.object, destination))

    original_tasks = list(tasks)
    if len(tasks) > 1 and not parsed.preserve_batch_order:
        tasks = optimize_batch_order(tasks)

    rclpy.init(args=ros_args)
    node = PickPlaceTest(
        tasks[0][0], tasks[0][1],
        parsed.navigation_timeout, parsed.manipulation_timeout)
    exit_code = 1
    try:
        batch_start = time.monotonic()
        if tasks != original_tasks:
            node.get_logger().info(
                'BATCH ROUTE OPTIMIZED: '
                f'original={original_tasks}, planned={tasks}; '
                'policy=B-before-A,C-last')
        for index, (requested_object, task_destination) in enumerate(tasks, 1):
            item_start = time.monotonic()
            execute_one_task(
                node, requested_object, task_destination,
                parsed.pickup_handoff_distance,
                egress_after_place=(
                    parsed.egress_after_place
                    if len(tasks) == 1 else index < len(tasks)),
                select_only=parsed.select_only)
            node.get_logger().info(
                f'BATCH ITEM {index}/{len(tasks)} complete: '
                f'{requested_object}->{task_destination}, '
                f'elapsed={time.monotonic() - item_start:.3f}s')
        node.get_logger().info(
            f'BATCH COMPLETE: items={len(tasks)}, '
            f'elapsed={time.monotonic() - batch_start:.3f}s')
        exit_code = 0
    except (KeyboardInterrupt, RuntimeError, KeyError, ValueError) as error:
        node.publish_navigation_status(
            phase='FAILED', event='acceptance_failed', message=str(error))
        node.get_logger().error(f'ACCEPTANCE FAILED: {error}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main(sys.argv[1:])
