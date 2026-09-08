#!/usr/bin/env python3
"""Navigate to a configured cube, pick it, navigate to a zone, and place it."""

import argparse
import json
import math
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
        self.dock_publisher = self.create_publisher(
            Twist, '/cmd_vel_nav', 10)
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
        robot = self._fresh_robot_pose()
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

    def _dock_coordinates(self):
        robot = self._fresh_robot_pose()
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
            pickup_distance = math.hypot(
                pose.position.x - robot.position.x,
                pose.position.y - robot.position.y)
            if not race_selection:
                ranked.append((pickup_distance, object_id, 0.0))
                continue
            carry_distance = math.hypot(
                pose.position.x - zone.position.x,
                pose.position.y - zone.position.y)
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
                handoff_distance,
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
            raise RuntimeError(
                f'Navigation to {label} failed; status={status}')
        self.get_logger().info(f'Navigation to {label} succeeded')
        self.publish_navigation_status(event='goal_succeeded')

    def _wait_navigation_result(
            self, handle, result_future, label, target_x, target_y,
            handoff_distance):
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
            velocity_fresh = now - self.base_velocity_received <= 0.35
            speed = min(1.5, abs(self.base_velocity.linear.x)) if (
                velocity_fresh and self.base_velocity is not None) else 1.5
            # Include pipeline latency, conservative braking and pose sampling.
            # Braking allowance: 150 ms command/actuation latency plus a
            # conservative 3.2 m/s^2 measured deceleration.  The previous
            # 1.0 m/s^2 assumption handed control to fine dock more than a
            # metre early at cruise speed and added ~8 s of crawling.
            dynamic_threshold = max(
                threshold, 0.18 + 0.15*speed + speed*speed/(2.0*3.2))
            if math.isfinite(distance) and distance <= dynamic_threshold:
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
                       and abs(velocity.angular.z) < 0.05)
            stopped_since = (stopped_since or now) if stopped else None
            if stopped_since is not None and now - stopped_since >= 0.25:
                return
        raise RuntimeError('Base did not stop with fresh odometry; handoff refused')

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
            raise RuntimeError(
                f'{operation} failed: code={result.error_code}, '
                f'message={result.message}')
        self.get_logger().info(
            f'{operation} succeeded: {result.message}')

    def configure_dropoff_tracking(self):
        """Use a longer RPP preview only for the long carried-object leg."""
        if not self.controller_parameters.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('controller_server parameter service unavailable')
        requested = {
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
            'Dropoff RPP preview configured: lookahead=1.10m, '
            'range=0.90..1.60m, time=1.00s')

    def navigate_pickup_transits(self):
        """Bypass a dynamic swept track before selected pickup approaches."""
        if self.object_id != 'red_cube_4':
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
                f'red_cube_4 rail bypass {index}', transit, lock_route=True)

    def navigate_dropoff_transits(self):
        """Use the real A-room doorway instead of a fragile direct chord."""
        if self.destination == 'B':
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
                lock_route=True)
            return
        if self.destination != 'A':
            return
        # World geometry evidence: Wall_38 closes the upper-left room at
        # y=0.896 until x=-2.857, while Wall_57/75 form the x=-3.2 vertical
        # divider.  The valid entrance is the gap at the east end of Wall_46.
        # These poses do not edit the map; they constrain the carried route to
        # that existing doorway and keep it away from the blue-cube row.
        transits = (
            {'x': -2.30, 'y': 1.30, 'yaw': -math.pi / 2.0},
            {'x': -2.25, 'y': -0.25, 'yaw': math.pi},
            {'x': -4.00, 'y': -0.25, 'yaw': -math.pi / 2.0},
            {'x': -4.00, 'y': -1.75, 'yaw': -2.40},
        )
        for index, transit in enumerate(transits, 1):
            self.publish_navigation_status(
                phase='DROPOFF_TRANSIT', event='phase_start',
                transit_index=index, transit_count=len(transits))
            self.navigate(
                f'A doorway transit {index}', transit, lock_route=True)

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
        target_x = 0.38
        deadline = time.monotonic() + timeout_sec
        stable_samples = 0
        last_log = 0.0
        self.get_logger().info(
            'Starting fine dock: target object=(0.38m, 0.00m) in base frame')
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.01)
                x, y = self._dock_coordinates()
                range_error = math.hypot(x, y) - target_x
                distance_error = x - target_x
                heading_error = math.atan2(y, x)
                if x <= 0.05:
                    raise RuntimeError(
                        'Object behind or underneath base after handoff; '
                        'stopping instead of rotating around it')
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
                    if abs(heading_error) > 0.12:
                        linear = 0.0
                    else:
                        linear *= max(0.35, math.cos(heading_error) ** 2)
                    # Close the lateral error promptly once range is already
                    # correct.  The former 1.25 gain spent about three seconds
                    # creeping from |y|=0.036 m into the 0.015 m grasp window.
                    angular = max(-0.45, min(0.45, 2.40 * heading_error))
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

    def fine_dropoff_approach(self, target, timeout_sec=16.0):
        """Finish the carried-object dock slowly after the Nav2 handoff.

        The controller operates only inside the final roughly 0.6 m.  It
        first faces the already-planned dock point, then advances at no more
        than 0.10 m/s.  Final yaw remains a separate stopped operation, so the
        payload cannot sweep through a zone obstacle during translation.
        """
        target_x = float(target['x'])
        target_y = float(target['y'])
        deadline = time.monotonic() + float(timeout_sec)
        stable_since = None
        last_log = 0.0
        initial = self._fresh_robot_pose()
        initial_distance = math.hypot(
            initial.position.x - target_x, initial.position.y - target_y)
        if initial_distance > 0.90:
            raise RuntimeError(
                f'Destination handoff occurred too early '
                f'({initial_distance:.3f}m); refusing open-loop approach')
        self.get_logger().info(
            f'Starting low-speed destination approach: '
            f'distance={initial_distance:.3f}m, target_tolerance=0.14m')
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.04)
                robot = self._fresh_robot_pose()
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
                if distance <= 0.14:
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
                    if abs(heading_error) > 0.18:
                        linear = 0.0
                    else:
                        linear = max(0.045, min(0.10, 0.55 * distance))
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
        tolerance = 0.035
        self.get_logger().info(
            f'Starting dropoff heading alignment: target={target_yaw:.3f}rad')
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.04)
                robot = self._fresh_robot_pose()
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
                    angular = max(-0.28, min(0.28, 1.20 * error))
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

    def egress_after_place(self, travel_distance=0.70, timeout_sec=11.0):
        """Back straight away from a placed cube before starting another task.

        This is deliberately opt-in: the final task still settles in place.
        The manoeuvre keeps the placement heading, verifies that separation
        from the released cube is increasing, and stops on stale Gazebo data
        or lack of progress instead of forcing the chassis through an object.
        """
        robot = self._fresh_robot_pose()
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
        linear_command = 0.10 if heading_dot_away >= 0.0 else -0.10
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
                robot = self._fresh_robot_pose()
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
        prefix = 'red_cube_' if self.destination == 'A' else 'blue_cube_'
        checked = []
        checked_positions = []
        failures = []
        for name, world_pose in sorted(self.model_poses.items()):
            if not name.startswith(prefix) or world_pose.position.z > 0.08:
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
        if self.destination in ('A', 'B'):
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
                # A's arm trajectory has an observed lateral sweep of up to
                # 0.09 m.  A 2x3 interior grid keeps that sweep inside the
                # 25 mm boundary margin and preserves >=0.10 m nominal
                # centre spacing for five 30 mm cubes.
                candidates = [
                    (0.08, -0.13), (0.08, 0.05), (0.08, 0.18),
                    (0.28, -0.13), (0.28, 0.16), (0.20, 0.02),
                ]
            else:
                # A single five-wide row left only 70 mm nominal spacing.
                # With measured Nav2/arm landing error that allowed a later
                # attached cube to contact stored cargo and lift the chassis.
                # This interior 2x3 grid keeps every base target near the
                # proven (-2,-5) parking line.  Extending the second column to
                # local x=0.28 put the chassis on the raised zone edge and the
                # pre-release tilt guard correctly refused to open.
                candidates = [
                    (0.08, -0.13), (0.08, 0.05), (0.08, 0.13),
                    (0.18, -0.13), (0.18, 0.10), (0.13, 0.02),
                ]
            preferred_indices = (
                {5: 1, 2: 2, 3: 0, 1: 4, 4: 3}
                if self.destination == 'A'
                else {5: 1, 4: 2, 3: 0, 2: 4, 1: 3})
            # An actual landing point can differ from its nominal candidate
            # because Nav2 has finite pose tolerance.  Merely measuring the
            # distance back to candidates allowed that nominal slot to be
            # selected again on a later invocation.  Reserve the closest
            # nominal slot for every stored cube, in addition to the live
            # clearance check.
            reserved = {
                preferred_indices[int(name.rsplit('_', 1)[1])]
                for _ox, _oy, name in occupied
                if int(name.rsplit('_', 1)[1]) in preferred_indices
            }
            minimum_clearance = 0.05
            safe = [
                candidate for index, candidate in enumerate(candidates)
                if index not in reserved
                and all(math.hypot(candidate[0] - ox,
                                   candidate[1] - oy) >= minimum_clearance
                        for ox, oy, _name in occupied)
            ]
            if not safe:
                raise RuntimeError(
                    f'No collision-free {self.destination} placement slot '
                    'remains; holding object')
            preferred = None
            # Stable one-to-one claims survive separate per-cube test
            # processes.  Inferring a claim from the actual landing point is
            # ambiguous because arm sweep can move a cube closer to an
            # adjacent nominal slot.
            suffix = int(self.object_id.rsplit('_', 1)[1])
            preferred_index = preferred_indices.get(suffix)
            if preferred_index is not None:
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
                object_zone_x, object_zone_y = (
                    (0.22, 0.0) if self.destination == 'A'
                    else (0.18, 0.0))
            self.get_logger().info(
                f'{self.destination} slot selection: '
                f'selected=({object_zone_x:.2f},'
                f'{object_zone_y:.2f}), reserved={sorted(reserved)}, occupied='
                f'{[(name, round(x, 3), round(y, 3)) for x, y, name in occupied]}')
        else:
            object_zone_x = max(-0.18, min(0.18, robot_zone_x))
            object_zone_y = max(-0.05, min(0.05, robot_zone_y))
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


def load_targets(destination):
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
    if not assigned:
        raise RuntimeError(
            f'No competition objects are assigned to destination {destination}')
    missing = [name for name in assigned if name not in all_approaches]
    if missing:
        raise RuntimeError(
            'Missing approach poses for assigned objects: ' + ', '.join(missing))
    approaches = {name: all_approaches[name] for name in assigned}
    return approaches, destinations[destination]


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
    parsed, ros_args = parser.parse_known_args(args)
    destination = parsed.destination.upper()

    rclpy.init(args=ros_args)
    node = PickPlaceTest(
        parsed.object, destination,
        parsed.navigation_timeout, parsed.manipulation_timeout)
    exit_code = 1
    try:
        approaches, dropoff = load_targets(destination)
        node.wait_for_state_service()
        selected = node.select_object(parsed.object, approaches)
        node.object_id = selected
        node.publish_navigation_status(
            phase='SELECTED', event='object_selected', object_id=selected,
            destination=destination)
        if parsed.select_only:
            print(f'SELECTED_OBJECT={selected}', flush=True)
            exit_code = 0
            return
        node.wait_for_interfaces()
        pickup = node.nearest_dock_target(selected, approaches[selected])
        node.navigate_pickup_transits()
        node.publish_navigation_status(phase='NAV_PICKUP', event='phase_start')
        node.navigate(
            f'pickup {selected}', pickup,
            handoff_distance=parsed.pickup_handoff_distance)
        node.publish_navigation_status(phase='FINE_DOCK', event='phase_start')
        node.fine_dock()
        node.check_pick_alignment()
        node.publish_navigation_status(phase='PICK', event='phase_start')
        node.manipulate('pick')
        node.configure_dropoff_tracking()
        dropoff = node.compute_dropoff_target(dropoff)
        node.navigate_dropoff_transits()
        node.publish_navigation_status(
            phase='NAV_DROPOFF', event='phase_start')
        # The carried payload and extended arm are not represented by the
        # chassis-only recovery footprint.  Never run the default Spin/BackUp
        # recovery subtree while carrying: a recovery rotation beside a wall
        # or obstacle can create a rigid-body collision and launch the robot.
        # This tree still replans when the path becomes invalid, but fails
        # safely instead of executing those chassis recovery motions.
        node.navigate(
            f'destination {destination}', dropoff, lock_route=True)
        node.publish_navigation_status(
            phase='DROPOFF_FINE_APPROACH', event='phase_start')
        node.fine_dropoff_approach(dropoff)
        node.publish_navigation_status(
            phase='DROPOFF_ALIGN', event='phase_start')
        node.align_dropoff_heading(float(dropoff['yaw']))
        node.check_drop_alignment()
        node.publish_navigation_status(phase='PLACE', event='phase_start')
        node.manipulate('place')
        node.validate_destination_inventory()
        if parsed.egress_after_place:
            node.egress_after_place()
        node.publish_navigation_status(phase='COMPLETE', event='acceptance_passed')
        node.get_logger().info(
            'ACCEPTANCE PASSED: navigation + pick + carry + place')
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
