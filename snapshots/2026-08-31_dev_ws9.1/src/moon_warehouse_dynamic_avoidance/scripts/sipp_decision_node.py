#!/usr/bin/env python3
"""Deterministic lightweight SIPP decision layer for dynamic obstacles."""

import json
import math
import time

from geometry_msgs.msg import Pose, PoseStamped, Twist
from moon_warehouse_interfaces.msg import ObstacleTrajectoryArray
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, Int32, String
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    siny = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y
    )
    cosy = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )
    return math.atan2(siny, cosy)


class LightweightSippDecision(Node):
    def __init__(self):
        super().__init__('sipp_decision_node')
        defaults = {
            'shadow_mode': False,
            'decision_frequency': 10.0,
            'conflict_radius': 0.78,
            'hard_release_radius': 0.65,
            'conflict_longitudinal_margin': 0.30,
            'maximum_occupancy_gap': 0.25,
            'time_margin': 0.35,
            'release_time_margin': 0.15,
            'half_vehicle_length': 0.20,
            'static_stop_margin': 0.70,
            'comfortable_deceleration': 2.50,
            'launch_acceleration': 2.00,
            'stop_reached_tolerance': 0.05,
            'minimum_prediction_speed': 0.30,
            'maximum_prediction_speed': 1.40,
            'endpoint_distance': 0.40,
            'data_timeout': 0.60,
            'plan_timeout': 2.50,
            'global_frame': 'map',
            'robot_base_frame': 'base_footprint',
            'tf_timeout': 0.05,
            'maximum_path_deviation': 0.75,
            'maximum_index_backtrack': 2,
            'same_goal_tolerance': 0.25,
            'route_change_threshold': 0.50,
            'route_change_confirm_cycles': 3,
            'conflict_confirm_cycles': 2,
            'conflict_release_time': 0.60,
            'conflict_match_radius': 0.45,
            'conflict_clear_margin': 0.35,
            'prepare_lead_time': 0.70,
            'departure_heading_lookahead': 0.40,
            'departure_yaw_tolerance': 0.08,
            'prepare_yaw_kp': 1.50,
            'prepare_max_angular_speed': 1.30,
            'prepare_min_angular_speed': 0.25,
            'prepare_settle_time': 0.20,
            'fast_cruise_speed': 1.40,
            'normal_cruise_speed': 1.25,
            'cautious_speed': 0.65,
            'fast_cruise_horizon': 3.0,
            'fast_cruise_min_goal_distance': 1.0,
            'fast_cruise_max_heading_error': 0.25,
            'fast_cruise_max_curvature': 0.60,
            'mppi_feedback_timeout': 0.40,
            'mppi_pass_min_safe_ratio': 0.80,
            'pass_commit_timeout_margin': 2.00,
            'pass_entry_hysteresis': 0.05,
            'pass_release_min_nav_speed': 0.12,
            'pass_recheck_confirm_cycles': 3,
            'pass_stall_timeout': 2.00,
            'pass_stall_speed': 0.05,
            'pass_stall_angular_speed': 0.15,
            'blocked_redecision_hold': 0.80,
            'departure_turn_heading_error': 0.55,
            'departure_turn_path_distance': 0.30,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in defaults:
            setattr(self, name, self.get_parameter(name).value)

        self.decision_frequency = max(1.0, float(self.decision_frequency))
        self.conflict_radius = max(0.10, float(self.conflict_radius))
        self.hard_release_radius = min(
            self.conflict_radius,
            max(0.05, float(self.hard_release_radius)),
        )
        self.maximum_occupancy_gap = max(0.05, float(self.maximum_occupancy_gap))
        self.comfortable_deceleration = max(
            0.05, float(self.comfortable_deceleration))
        self.launch_acceleration = max(0.05, float(self.launch_acceleration))
        self.minimum_prediction_speed = max(
            0.05, float(self.minimum_prediction_speed))
        self.maximum_prediction_speed = max(
            self.minimum_prediction_speed, float(self.maximum_prediction_speed))
        self.route_change_confirm_cycles = max(
            1, int(self.route_change_confirm_cycles))
        self.conflict_confirm_cycles = max(1, int(self.conflict_confirm_cycles))
        self.pass_recheck_confirm_cycles = max(
            1, int(self.pass_recheck_confirm_cycles))

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.plan = None
        self.plan_received = 0.0
        self.plan_version = 0
        self.goal_position = None
        # This is armed once per navigation goal, never by same-goal replans.
        self.departure_turn_pending = False
        self.pending_route = None
        self.pending_route_count = 0
        self.predictions = None
        self.predictions_received = 0.0
        self.prediction_sequence = 0
        self.robot_pose = None
        self.odom_pose = None
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.last_nearest_index = None
        self.pending_conflict = None
        self.active_conflict = None
        self.state = 'IDLE'
        self.current_diagnostics = {}
        self.mppi_feedback = None
        self.mppi_feedback_received = 0.0
        self.mppi_feedback_sequence = 0
        self.nav_cmd_linear = 0.0
        self.nav_cmd_angular = 0.0
        self.nav_cmd_received = 0.0
        self.pass_stall_since = None
        self.mppi_block_until = 0.0
        self.decision_sequence = 0

        self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 20)
        self.create_subscription(Twist, '/cmd_vel_nav', self.nav_cmd_callback, 20)
        self.create_subscription(
            String, '/mppi/sipp_feedback', self.mppi_feedback_callback, 20)
        self.create_subscription(
            ObstacleTrajectoryArray,
            '/moon_warehouse/dynamic_obstacle_trajectories',
            self.prediction_callback,
            10,
        )
        self.state_publisher = self.create_publisher(String, '/sipp/state', 10)
        self.speed_mode_publisher = self.create_publisher(
            String, '/sipp/speed_mode', 10)
        self.decision_publisher = self.create_publisher(
            String, '/sipp/decision', 10)
        self.hold_publisher = self.create_publisher(Bool, '/sipp/hold', 10)
        self.stop_line_publisher = self.create_publisher(
            PoseStamped, '/sipp/stop_line', 10)
        self.conflict_publisher = self.create_publisher(
            PoseStamped, '/sipp/conflict_point', 10)
        self.remaining_path_publisher = self.create_publisher(
            Float32, '/sipp/remaining_path', 10)
        self.nearest_index_publisher = self.create_publisher(
            Int32, '/sipp/nearest_path_index', 10)
        self.create_timer(1.0 / self.decision_frequency, self.evaluate)
        self.get_logger().info(
            f'Deterministic SIPP ready: shadow={self.shadow_mode} '
            f'soft_radius={self.conflict_radius:.2f} '
            f'hard_radius={self.hard_release_radius:.2f} '
            f'confirm={self.conflict_confirm_cycles}'
        )

    @staticmethod
    def normalize_frame(frame_id):
        return str(frame_id).strip().lstrip('/')

    @staticmethod
    def distance(first, second):
        return math.hypot(
            first.position.x - second.position.x,
            first.position.y - second.position.y,
        )

    def travel_time_for_distance(self, distance, initial_speed=0.0):
        """Return acceleration-aware travel time along the current path."""
        distance = max(0.0, float(distance))
        if distance <= 1e-6:
            return 0.0
        maximum_speed = max(0.05, float(self.maximum_prediction_speed))
        acceleration = max(0.05, float(self.launch_acceleration))
        initial_speed = min(max(0.0, float(initial_speed)), maximum_speed)
        acceleration_distance = max(
            0.0,
            (maximum_speed * maximum_speed - initial_speed * initial_speed)
            / (2.0 * acceleration),
        )
        if distance <= acceleration_distance:
            return (
                math.sqrt(initial_speed * initial_speed + 2.0 * acceleration * distance)
                - initial_speed
            ) / acceleration
        acceleration_time = (maximum_speed - initial_speed) / acceleration
        return acceleration_time + (distance - acceleration_distance) / maximum_speed

    @staticmethod
    def cumulative_distances(poses):
        cumulative = [0.0]
        for index in range(1, len(poses)):
            cumulative.append(
                cumulative[-1]
                + LightweightSippDecision.distance(
                    poses[index - 1].pose, poses[index].pose))
        return cumulative

    @staticmethod
    def project_point_to_segment(px, py, first, second):
        ax = first.position.x
        ay = first.position.y
        dx = second.position.x - ax
        dy = second.position.y - ay
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-12:
            return 0.0, math.hypot(px - ax, py - ay)
        fraction = ((px - ax) * dx + (py - ay) * dy) / length_squared
        fraction = min(1.0, max(0.0, fraction))
        projected_x = ax + fraction * dx
        projected_y = ay + fraction * dy
        return fraction, math.hypot(px - projected_x, py - projected_y)

    def project_point_to_path(self, point, poses, cumulative, start_index=0):
        best = None
        start = max(0, min(start_index, len(poses) - 2))
        for index in range(start, len(poses) - 1):
            fraction, lateral = self.project_point_to_segment(
                point.position.x,
                point.position.y,
                poses[index].pose,
                poses[index + 1].pose,
            )
            segment = cumulative[index + 1] - cumulative[index]
            projected_s = cumulative[index] + fraction * segment
            if best is None or lateral < best['lateral']:
                best = {
                    's': projected_s,
                    'lateral': lateral,
                    'segment_index': index,
                    'fraction': fraction,
                }
        return best

    def route_difference(self, candidate, reference):
        if reference is None or len(reference.poses) < 2:
            return math.inf
        reference_cumulative = self.cumulative_distances(reference.poses)
        step = max(1, len(candidate.poses) // 24)
        differences = []
        for pose in candidate.poses[::step]:
            projection = self.project_point_to_path(
                pose.pose,
                reference.poses,
                reference_cumulative,
                0,
            )
            if projection is not None:
                differences.append(projection['lateral'])
        return max(differences) if differences else math.inf

    def accept_plan(self, message, increment_version, preserve_commit=False):
        self.plan = message
        if increment_version:
            self.plan_version += 1
            self.last_nearest_index = None
            self.pending_conflict = None
            if self.active_conflict is not None and preserve_commit:
                # A same-goal global replan may prune or slightly reshape the
                # path.  It must not revoke an already committed temporal
                # decision.  Track clearance in world coordinates and carry
                # the commit into the new plan version.
                self.active_conflict['plan_version'] = self.plan_version
            else:
                self.active_conflict = None
        elif self.robot_pose is not None:
            self.last_nearest_index = min(
                range(len(message.poses)),
                key=lambda index: self.distance(
                    message.poses[index].pose, self.robot_pose),
            )
        elif self.last_nearest_index is not None:
            self.last_nearest_index = min(
                self.last_nearest_index, len(message.poses) - 1)

    def plan_callback(self, message):
        self.plan_received = time.monotonic()
        if len(message.poses) < 2:
            return
        endpoint = message.poses[-1].pose.position
        endpoint_xy = (float(endpoint.x), float(endpoint.y))
        if self.plan is None:
            self.goal_position = endpoint_xy
            self.departure_turn_pending = True
            self.accept_plan(message, True)
            return
        goal_changed = self.goal_position is None or math.hypot(
            endpoint_xy[0] - self.goal_position[0],
            endpoint_xy[1] - self.goal_position[1],
        ) > float(self.same_goal_tolerance)
        if goal_changed:
            self.goal_position = endpoint_xy
            self.departure_turn_pending = True
            self.pending_route = None
            self.pending_route_count = 0
            self.accept_plan(message, True)
            return
        difference = self.route_difference(message, self.plan)
        if difference <= float(self.route_change_threshold):
            self.pending_route = None
            self.pending_route_count = 0
            self.accept_plan(message, False)
            return
        pending_difference = self.route_difference(
            message, self.pending_route) if self.pending_route is not None else math.inf
        if pending_difference <= float(self.route_change_threshold):
            self.pending_route_count += 1
        else:
            self.pending_route = message
            self.pending_route_count = 1
        if self.pending_route_count >= self.route_change_confirm_cycles:
            self.accept_plan(message, True, preserve_commit=True)
            self.pending_route = None
            self.pending_route_count = 0

    def odom_callback(self, message):
        self.odom_pose = message.pose.pose
        self.linear_speed = float(message.twist.twist.linear.x)
        self.angular_speed = float(message.twist.twist.angular.z)

    def prediction_callback(self, message):
        self.predictions = message
        self.predictions_received = time.monotonic()
        self.prediction_sequence += 1

    def nav_cmd_callback(self, message):
        self.nav_cmd_linear = float(message.linear.x)
        self.nav_cmd_angular = float(message.angular.z)
        self.nav_cmd_received = time.monotonic()

    def mppi_feedback_callback(self, message):
        try:
            parsed = json.loads(message.data)
            if not isinstance(parsed, dict):
                return
            self.mppi_feedback = parsed
            self.mppi_feedback_received = time.monotonic()
            self.mppi_feedback_sequence += 1
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    def valid_mppi_feedback(self, now=None, expected_state=None):
        if self.mppi_feedback is None:
            return None
        if now is None:
            now = time.monotonic()
        if now - self.mppi_feedback_received > float(self.mppi_feedback_timeout):
            return None
        if (
            expected_state is not None
            and str(self.mppi_feedback.get('sipp_state', '')) != expected_state
        ):
            return None
        return self.mppi_feedback

    def lookup_robot_pose(self):
        transform = self.tf_buffer.lookup_transform(
            str(self.global_frame),
            str(self.robot_base_frame),
            Time(),
            timeout=Duration(seconds=float(self.tf_timeout)),
        )
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def plan_frame_id(self):
        if self.plan is None:
            return ''
        frame = self.normalize_frame(self.plan.header.frame_id)
        if not frame and self.plan.poses:
            frame = self.normalize_frame(self.plan.poses[0].header.frame_id)
        return frame

    def prediction_frame_id(self):
        if self.predictions is None:
            return ''
        return self.normalize_frame(self.predictions.header.frame_id)

    def nearest_path_index(self, poses, robot_pose):
        if self.last_nearest_index is None:
            search_start = 0
        else:
            search_start = max(
                0,
                min(
                    self.last_nearest_index - int(self.maximum_index_backtrack),
                    len(poses) - 1,
                ),
            )
        nearest = min(
            range(search_start, len(poses)),
            key=lambda index: self.distance(poses[index].pose, robot_pose),
        )
        self.last_nearest_index = nearest
        return nearest, self.distance(poses[nearest].pose, robot_pose)

    def pose_at_s(self, poses, cumulative, target_s):
        target_s = min(max(0.0, target_s), cumulative[-1])
        upper = 1
        while upper < len(cumulative) and cumulative[upper] < target_s:
            upper += 1
        upper = min(upper, len(cumulative) - 1)
        lower = max(0, upper - 1)
        span = max(1e-9, cumulative[upper] - cumulative[lower])
        ratio = (target_s - cumulative[lower]) / span
        first = poses[lower]
        second = poses[upper]
        result = PoseStamped()
        result.header = first.header
        result.pose.position.x = (
            first.pose.position.x
            + ratio * (second.pose.position.x - first.pose.position.x))
        result.pose.position.y = (
            first.pose.position.y
            + ratio * (second.pose.position.y - first.pose.position.y))
        result.pose.position.z = first.pose.position.z
        yaw = math.atan2(
            second.pose.position.y - first.pose.position.y,
            second.pose.position.x - first.pose.position.x,
        )
        result.pose.orientation.z = math.sin(0.5 * yaw)
        result.pose.orientation.w = math.cos(0.5 * yaw)
        return result

    def tangent_yaw_at_s(self, poses, cumulative, target_s, lookahead=0.15):
        before = self.pose_at_s(poses, cumulative, max(0.0, target_s - lookahead))
        after = self.pose_at_s(
            poses, cumulative, min(cumulative[-1], target_s + lookahead))
        return math.atan2(
            after.pose.position.y - before.pose.position.y,
            after.pose.position.x - before.pose.position.x,
        )

    def make_conflict_candidate(
        self, group, obstacle_id, poses, cumulative, base_s, current_speed
    ):
        enter_s = min(item['s_enter'] for item in group)
        exit_s = max(item['s_exit'] for item in group)
        center_s = 0.5 * (enter_s + exit_s)
        center_pose = self.pose_at_s(poses, cumulative, center_s)
        hard_group = [
            item for item in group
            if item['lateral'] <= self.hard_release_radius
        ]
        hard_occupied = bool(hard_group)
        if hard_occupied:
            hard_enter_s = min(item['s_enter'] for item in hard_group)
            hard_exit_s = max(item['s_exit'] for item in hard_group)
            obstacle_hard_t_enter = min(item['time'] for item in hard_group)
            obstacle_hard_t_exit = max(item['time'] for item in hard_group)
        else:
            hard_enter_s = center_s
            hard_exit_s = center_s
            obstacle_hard_t_enter = -1.0
            obstacle_hard_t_exit = -1.0
        return {
            'obstacle_id': str(obstacle_id),
            'plan_version': self.plan_version,
            'conflict_world_x': float(center_pose.pose.position.x),
            'conflict_world_y': float(center_pose.pose.position.y),
            'conflict_yaw': self.tangent_yaw_at_s(poses, cumulative, center_s),
            'conflict_s_enter': enter_s,
            'conflict_s_center': center_s,
            'conflict_s_exit': exit_s,
            'obstacle_t_enter': min(item['time'] for item in group),
            'obstacle_t_exit': max(item['time'] for item in group),
            'robot_t_enter': self.travel_time_for_distance(
                enter_s - base_s, current_speed),
            'robot_t_exit': self.travel_time_for_distance(
                exit_s - base_s, current_speed),
            'hard_occupied': hard_occupied,
            'hard_conflict_s_enter': hard_enter_s,
            'hard_conflict_s_exit': hard_exit_s,
            'obstacle_hard_t_enter': obstacle_hard_t_enter,
            'obstacle_hard_t_exit': obstacle_hard_t_exit,
            'robot_hard_t_enter': self.travel_time_for_distance(
                hard_enter_s - base_s, current_speed),
            'robot_hard_t_exit': self.travel_time_for_distance(
                hard_exit_s - base_s, current_speed),
            'minimum_space': min(item['lateral'] for item in group),
        }

    def find_conflicts(self, poses, cumulative, nearest_index, current_speed):
        candidates = []
        base_s = cumulative[nearest_index]
        search_start = max(0, nearest_index - int(self.maximum_index_backtrack))
        for trajectory in self.predictions.trajectories:
            count = min(len(trajectory.future_poses), len(trajectory.future_times))
            occupied = []
            for index in range(count):
                projection = self.project_point_to_path(
                    trajectory.future_poses[index],
                    poses,
                    cumulative,
                    search_start,
                )
                if projection is None or projection['lateral'] > self.conflict_radius:
                    continue
                occupied.append({
                    'time': float(trajectory.future_times[index]),
                    's_enter': projection['s'] - float(self.conflict_longitudinal_margin),
                    's_exit': projection['s'] + float(self.conflict_longitudinal_margin),
                    'lateral': projection['lateral'],
                })
            group = []
            for item in occupied:
                if group and item['time'] - group[-1]['time'] > self.maximum_occupancy_gap:
                    candidate = self.make_conflict_candidate(
                        group, trajectory.id, poses, cumulative, base_s, current_speed)
                    if candidate['conflict_s_exit'] >= base_s:
                        candidates.append(candidate)
                    group = []
                group.append(item)
            if group:
                candidate = self.make_conflict_candidate(
                    group, trajectory.id, poses, cumulative, base_s, current_speed)
                if candidate['conflict_s_exit'] >= base_s:
                    candidates.append(candidate)
        candidates.sort(key=lambda item: (
            max(base_s, item['conflict_s_enter']),
            item['obstacle_t_enter'],
        ))
        return candidates

    def candidate_matches(self, candidate, tracked):
        return (
            candidate['obstacle_id'] == tracked['obstacle_id']
            and candidate['plan_version'] == tracked['plan_version']
            and math.hypot(
                candidate['conflict_world_x'] - tracked['conflict_world_x'],
                candidate['conflict_world_y'] - tracked['conflict_world_y'],
            ) <= float(self.conflict_match_radius)
        )

    def find_match(self, candidates, tracked):
        matches = [item for item in candidates if self.candidate_matches(item, tracked)]
        if not matches:
            return None
        return min(matches, key=lambda item: math.hypot(
            item['conflict_world_x'] - tracked['conflict_world_x'],
            item['conflict_world_y'] - tracked['conflict_world_y'],
        ))

    def relevant_candidate(self, candidates):
        for candidate in candidates:
            if (
                candidate['obstacle_t_exit'] + float(self.time_margin)
                < candidate['robot_t_enter']
            ):
                continue
            return candidate
        return None

    def conflict_pose(self, conflict):
        result = PoseStamped()
        result.header.frame_id = str(self.global_frame)
        result.header.stamp = self.get_clock().now().to_msg()
        result.pose.position.x = float(conflict['conflict_world_x'])
        result.pose.position.y = float(conflict['conflict_world_y'])
        yaw = float(conflict['conflict_yaw'])
        result.pose.orientation.z = math.sin(0.5 * yaw)
        result.pose.orientation.w = math.cos(0.5 * yaw)
        return result

    def stop_pose(self, conflict):
        if conflict is None or conflict.get('stop_pose') is None:
            return None
        result = PoseStamped()
        result.header.frame_id = str(self.global_frame)
        result.header.stamp = self.get_clock().now().to_msg()
        result.pose.position.x = float(conflict['stop_pose']['x'])
        result.pose.position.y = float(conflict['stop_pose']['y'])
        yaw = float(conflict['stop_pose']['yaw'])
        result.pose.orientation.z = math.sin(0.5 * yaw)
        result.pose.orientation.w = math.cos(0.5 * yaw)
        return result

    def track_details(self, track):
        if track is None:
            return {}
        keys = (
            'conflict_id', 'obstacle_id', 'plan_version',
            'conflict_world_x', 'conflict_world_y',
            'conflict_s_enter', 'conflict_s_center', 'conflict_s_exit',
            'obstacle_t_enter', 'obstacle_t_exit',
            'robot_t_enter', 'robot_t_exit', 'minimum_space',
            'hard_occupied', 'hard_conflict_s_enter', 'hard_conflict_s_exit',
            'obstacle_hard_t_enter', 'obstacle_hard_t_exit',
            'robot_hard_t_enter', 'robot_hard_t_exit',
            'stop_s', 'departure_yaw', 'confirm_count',
            'first_seen_time', 'last_seen_time', 'safe_release_delay',
            'robot_entry_from_stop', 'robot_clears_first_now',
            'decision_reason',
            'distance_to_stop', 'prepare_yaw_error', 'prepare_angular_z',
            'pass_execution_ready', 'pass_deadline',
        )
        return {key: track[key] for key in keys if key in track}

    def path_metrics(self, poses, cumulative, base_s, robot_yaw):
        lookahead_s = min(cumulative[-1], base_s + 0.40)
        path_yaw = self.tangent_yaw_at_s(poses, cumulative, lookahead_s)
        heading_error = abs(normalize_angle(path_yaw - robot_yaw))
        curvature = 0.0
        previous_yaw = path_yaw
        previous_s = lookahead_s
        sample_s = lookahead_s + 0.20
        while sample_s <= min(cumulative[-1], base_s + 2.0):
            yaw = self.tangent_yaw_at_s(poses, cumulative, sample_s)
            curvature = max(
                curvature,
                abs(normalize_angle(yaw - previous_yaw))
                / max(0.05, sample_s - previous_s),
            )
            previous_yaw = yaw
            previous_s = sample_s
            sample_s += 0.20
        return heading_error, curvature

    def select_speed_mode(
        self, state, remaining, heading_error, curvature, candidates
    ):
        future_conflict = any(
            item['obstacle_t_enter'] <= float(self.fast_cruise_horizon)
            and not (
                item['robot_t_exit'] + float(self.time_margin)
                < item['obstacle_t_enter']
                or item['obstacle_t_exit'] + float(self.time_margin)
                < item['robot_t_enter']
            )
            for item in candidates
        )
        fast = (
            (
                state == 'PASS_COMMITTED'
                or (state == 'CRUISE' and not future_conflict)
            )
            and remaining > float(self.fast_cruise_min_goal_distance)
            and heading_error < float(self.fast_cruise_max_heading_error)
            and curvature < float(self.fast_cruise_max_curvature)
        )
        if fast:
            return 'FAST_CRUISE', float(self.fast_cruise_speed), future_conflict
        if state in {
            'CRUISE', 'PASS_COMMITTED', 'APPROACH_STOP_LINE',
            'STOP_COMMITTED', 'ENDPOINT_TURN', 'TOO_LATE_BYPASS'
        }:
            return 'NORMAL_CRUISE', float(self.normal_cruise_speed), future_conflict
        return 'CAUTIOUS', float(self.cautious_speed), future_conflict

    def publish(
        self,
        state,
        details,
        hold=False,
        stop_line=None,
        conflict=None,
    ):
        self.state = state
        payload = {
            'state': state,
            'shadow_mode': bool(self.shadow_mode),
            **self.current_diagnostics,
            **details,
        }
        self.state_publisher.publish(String(data=state))
        self.speed_mode_publisher.publish(String(
            data=str(payload.get('speed_mode', 'CAUTIOUS'))))
        self.decision_publisher.publish(String(data=json.dumps(
            payload, ensure_ascii=False, sort_keys=True)))
        self.hold_publisher.publish(Bool(
            data=bool(hold and not self.shadow_mode)))
        self.remaining_path_publisher.publish(Float32(
            data=float(payload.get('remaining_path', math.nan))))
        self.nearest_index_publisher.publish(Int32(
            data=int(payload.get('nearest_path_index', -1))))
        if stop_line is not None:
            self.stop_line_publisher.publish(stop_line)
        if conflict is not None:
            self.conflict_publisher.publish(conflict)

    def publish_degraded(self, reason, **details):
        self.current_diagnostics = {}
        self.publish('DEGRADED', {'reason': reason, **details})

    def input_snapshot(self):
        now = time.monotonic()
        if self.plan is None or len(self.plan.poses) < 2:
            self.publish('IDLE', {'reason': 'waiting_for_plan'})
            return None
        plan_age = now - self.plan_received
        if plan_age > float(self.plan_timeout):
            self.publish_degraded('plan_timeout', plan_age=plan_age)
            return None
        if self.predictions is None:
            self.publish_degraded('waiting_for_prediction')
            return None
        prediction_age = now - self.predictions_received
        if prediction_age > float(self.data_timeout):
            self.publish_degraded('prediction_timeout', prediction_age=prediction_age)
            return None
        expected = self.normalize_frame(self.global_frame)
        plan_frame = self.plan_frame_id()
        prediction_frame = self.prediction_frame_id()
        if plan_frame != expected:
            self.publish_degraded(
                'plan_frame_mismatch', expected_frame=expected, plan_frame=plan_frame)
            return None
        if prediction_frame != expected:
            self.publish_degraded(
                'prediction_frame_mismatch',
                expected_frame=expected,
                prediction_frame=prediction_frame,
            )
            return None
        try:
            robot_pose = self.lookup_robot_pose()
        except TransformException as error:
            self.publish_degraded('tf_unavailable', tf_error=str(error))
            return None
        self.robot_pose = robot_pose
        poses = self.plan.poses
        cumulative = self.cumulative_distances(poses)
        nearest, deviation = self.nearest_path_index(poses, robot_pose)
        if deviation > float(self.maximum_path_deviation):
            self.publish_degraded(
                'robot_too_far_from_plan',
                path_deviation=deviation,
                maximum_path_deviation=float(self.maximum_path_deviation),
                nearest_path_index=nearest,
                plan_version=self.plan_version,
            )
            return None
        remaining = max(0.0, cumulative[-1] - cumulative[nearest])
        tf_odom_error = math.nan
        if self.odom_pose is not None:
            tf_odom_error = self.distance(robot_pose, self.odom_pose)
        robot_yaw = yaw_from_quaternion(robot_pose.orientation)
        heading_error, curvature = self.path_metrics(
            poses, cumulative, cumulative[nearest], robot_yaw)
        self.current_diagnostics = {
            'plan_age': plan_age,
            'prediction_age': prediction_age,
            'plan_frame': plan_frame,
            'prediction_frame': prediction_frame,
            'plan_version': self.plan_version,
            'route_change_pending_count': self.pending_route_count,
            'nearest_path_index': nearest,
            'path_deviation': deviation,
            'remaining_path': remaining,
            'robot_x': robot_pose.position.x,
            'robot_y': robot_pose.position.y,
            'robot_yaw': robot_yaw,
            'tf_odom_error': tf_odom_error,
            'path_heading_error': heading_error,
            'path_curvature': curvature,
        }
        # Decision timing must use the speed that will be commanded after a
        # release. Using the current velocity makes WAIT's zero speed predict a
        # very late robot arrival and opens unsafe/too-early PASS windows.
        reference_speed = min(
            self.maximum_prediction_speed,
            max(
                self.minimum_prediction_speed,
                self.normal_cruise_speed,
                abs(self.linear_speed),
            ),
        )
        candidates = self.find_conflicts(
            poses, cumulative, nearest, abs(self.linear_speed))
        return {
            'now': now,
            'poses': poses,
            'cumulative': cumulative,
            'nearest': nearest,
            'base_s': cumulative[nearest],
            'remaining': remaining,
            'robot_pose': robot_pose,
            'robot_yaw': robot_yaw,
            'reference_speed': reference_speed,
            'heading_error': heading_error,
            'curvature': curvature,
            'candidates': candidates,
        }

    def decorate_speed(self, details, state, snapshot):
        mode, limit, future_conflict = self.select_speed_mode(
            state,
            snapshot['remaining'],
            snapshot['heading_error'],
            snapshot['curvature'],
            snapshot['candidates'],
        )
        details['speed_mode'] = mode
        details['speed_limit'] = limit
        details['future_conflict_3s'] = future_conflict
        feedback = self.valid_mppi_feedback(snapshot['now'])
        if feedback is None:
            details['mppi_feedback_status'] = 'UNAVAILABLE'
        else:
            details['mppi_feedback_status'] = str(
                feedback.get('status', 'UNKNOWN'))
            details['mppi_safe_trajectory_ratio'] = float(
                feedback.get('safe_trajectory_ratio', -1.0))
            details['mppi_min_clearance'] = float(
                feedback.get('min_clearance', -1.0))
            details['mppi_collision_probability'] = float(
                feedback.get('collision_probability', -1.0))
        return details

    def robot_cleared_track(self, track, snapshot):
        forward_progress = self.track_forward_progress(track, snapshot)
        required = (
            track['conflict_s_exit'] - track['conflict_s_center']
            + float(self.conflict_clear_margin))
        # Active conflict geometry is fixed in the world. A same-goal replan
        # changes path arc-length coordinates, so using the new base_s with an
        # old conflict_s can clear or revoke a commitment at the wrong place.
        return forward_progress > required

    @staticmethod
    def track_forward_progress(track, snapshot):
        robot_pose = snapshot['robot_pose']
        dx = robot_pose.position.x - float(track['conflict_world_x'])
        dy = robot_pose.position.y - float(track['conflict_world_y'])
        return (
            dx * math.cos(float(track['conflict_yaw']))
            + dy * math.sin(float(track['conflict_yaw'])))

    def arm_pass_commit(self, track, snapshot):
        """Record the time/progress budget of one physical PASS attempt."""
        track['pass_committed_time'] = snapshot['now']
        track['pass_committed_base_s'] = snapshot['base_s']
        track['pass_last_progress_s'] = self.track_forward_progress(
            track, snapshot)
        track['pass_last_progress_time'] = snapshot['now']
        track['pass_block_count'] = 0
        track['pass_last_feedback_sequence'] = self.mppi_feedback_sequence
        track['pass_last_prediction_sequence'] = self.prediction_sequence
        expected_exit = max(
            0.0,
            float(track.get('robot_hard_t_exit', track.get('robot_t_exit', 0.0))),
        )
        track['pass_deadline'] = (
            snapshot['now'] + expected_exit
            + float(self.pass_commit_timeout_margin))

    def pass_before_entry(self, track, snapshot):
        entry_s = float(track.get(
            'hard_conflict_s_enter', track['conflict_s_enter']))
        entry_progress = entry_s - float(track['conflict_s_center'])
        return (
            self.track_forward_progress(track, snapshot)
            < entry_progress - float(self.pass_entry_hysteresis))

    def fallback_from_pass(self, track, snapshot, reason):
        """Revoke PASS only while the original fixed stop line is reachable."""
        distance_to_stop = self.distance_to_fixed_stop(track, snapshot)
        track['distance_to_stop'] = distance_to_stop
        self.pass_stall_since = None
        if distance_to_stop >= -float(self.stop_reached_tolerance):
            if distance_to_stop <= float(self.stop_reached_tolerance):
                track['status'] = 'WAIT_AT_STOP_LINE'
                self.publish_track(
                    'WAIT_AT_STOP_LINE', track, snapshot, reason, True)
            else:
                track['status'] = 'APPROACH_STOP_LINE'
                self.publish_track(
                    'APPROACH_STOP_LINE', track, snapshot, reason)
        else:
            # The fixed stop line belongs to the pre-commit decision. Once a
            # committed pass has crossed it, creating a WAIT-like bypass track
            # strands the robot in the crossing. Keep the physical commitment;
            # MPPI and downstream Collision Monitor retain safety authority.
            track['status'] = 'PASS_COMMITTED'
            self.publish_track(
                'PASS_COMMITTED', track, snapshot,
                'pass_recheck_after_stop_line_keep_commit')

    def pass_execution_ready(self, snapshot):
        """Require fresh MPPI safety evidence before releasing a held robot.

        MPPI runs upstream of this gate, so its safety ensemble remains
        observable while PREPARE_TO_PASS holds the executed translation at
        zero.  Do not require an already-positive raw linear command here:
        while the robot is aligning at the stop line MPPI may legitimately
        emit rotation-only commands, and making that command a prerequisite
        for PASS creates a circular WAIT/PREPARE deadlock.  The caller has
        already verified the fixed departure heading and temporal window;
        fresh MPPI feedback, its 80% safe-trajectory threshold, the committed
        pass watchdog, and Collision Monitor retain independent vetoes.
        """
        command_fresh = (
            snapshot['now'] - self.nav_cmd_received
            <= float(self.mppi_feedback_timeout))
        if not command_fresh:
            return False
        feedback = self.valid_mppi_feedback(
            snapshot['now'], expected_state='PREPARE_TO_PASS')
        if feedback is None:
            return False
        return (
            str(feedback.get('status', 'UNKNOWN')) == 'ACCEPTED'
            and float(feedback.get('safe_trajectory_ratio', 0.0))
            >= float(self.mppi_pass_min_safe_ratio))

    def distance_to_fixed_stop(self, track, snapshot):
        stop_dx = (
            float(track['stop_pose']['x']) - float(track['conflict_world_x']))
        stop_dy = (
            float(track['stop_pose']['y']) - float(track['conflict_world_y']))
        stop_progress = (
            stop_dx * math.cos(float(track['conflict_yaw']))
            + stop_dy * math.sin(float(track['conflict_yaw'])))
        return stop_progress - self.track_forward_progress(track, snapshot)

    def update_track_prediction(self, track, matched, now):
        if matched is None:
            return
        for key in (
            'obstacle_t_enter', 'obstacle_t_exit',
            'robot_t_enter', 'robot_t_exit', 'minimum_space',
            'hard_occupied', 'hard_conflict_s_enter', 'hard_conflict_s_exit',
            'obstacle_hard_t_enter', 'obstacle_hard_t_exit',
            'robot_hard_t_enter', 'robot_hard_t_exit',
        ):
            track[key] = matched[key]
        track['last_seen_time'] = now

    def prepare_command(self, track, snapshot):
        error = normalize_angle(track['departure_yaw'] - snapshot['robot_yaw'])
        track['prepare_yaw_error'] = error
        if abs(error) <= float(self.departure_yaw_tolerance):
            if track.get('prepare_aligned_since') is None:
                track['prepare_aligned_since'] = snapshot['now']
            angular = 0.0
        else:
            track['prepare_aligned_since'] = None
            angular = min(
                float(self.prepare_max_angular_speed),
                float(self.prepare_yaw_kp) * abs(error),
            )
            angular = max(float(self.prepare_min_angular_speed), angular)
            angular = math.copysign(angular, error)
        track['prepare_angular_z'] = angular
        return error, angular

    def publish_track(self, state, track, snapshot, reason=None, hold=False):
        details = self.track_details(track)
        if reason is not None:
            details['reason'] = reason
        self.decorate_speed(details, state, snapshot)
        self.publish(
            state,
            details,
            hold=hold,
            stop_line=self.stop_pose(track),
            conflict=self.conflict_pose(track),
        )

    def evaluate_active_track(self, snapshot):
        track = self.active_conflict
        if track is None:
            return False
        if track['plan_version'] != self.plan_version:
            self.active_conflict = None
            return False
        matched = self.find_match(snapshot['candidates'], track)
        self.update_track_prediction(track, matched, snapshot['now'])
        cleared = self.robot_cleared_track(track, snapshot)
        status = track['status']

        if status == 'PASS_COMMITTED':
            feedback = self.valid_mppi_feedback(
                snapshot['now'], expected_state='PASS_COMMITTED')
            emergency_feedback = self.valid_mppi_feedback(snapshot['now'])
            before_entry = self.pass_before_entry(track, snapshot)
            world_progress = self.track_forward_progress(track, snapshot)
            if world_progress > track.get('pass_last_progress_s', -math.inf) + 0.05:
                track['pass_last_progress_s'] = world_progress
                track['pass_last_progress_time'] = snapshot['now']
            feedback_blocks = False
            if feedback is not None:
                feedback_blocks = (
                    str(feedback.get('status', 'UNKNOWN')) in {
                        'INFEASIBLE', 'TEMPORARILY_BLOCKED', 'SAFETY_OVERRIDE'}
                    or float(feedback.get('safe_trajectory_ratio', 1.0))
                    < float(self.mppi_pass_min_safe_ratio))
            timing_blocks = (
                matched is not None
                and matched.get('hard_occupied', True)
                and not (
                    matched['robot_hard_t_exit'] + float(self.time_margin)
                    < matched['obstacle_hard_t_enter']
                    or matched['obstacle_hard_t_exit'] + float(self.time_margin)
                    < matched['robot_hard_t_enter']))
            # A hard imminent-collision recommendation remains an immediate
            # veto. Ordinary sampled-feasibility and timing estimates are
            # noisy at the edge of a window, so require several consecutive
            # blocked frames before revoking an already committed pass.
            if emergency_feedback is not None and bool(
                emergency_feedback.get('override_recommended', False)
            ):
                track['pass_block_count'] = 0
                self.pass_stall_since = None
                self.publish_track(
                    'SAFETY_OVERRIDE', track, snapshot,
                    'mppi_imminent_collision_override')
                return True
            new_feedback = (
                feedback is not None
                and self.mppi_feedback_sequence
                > int(track.get('pass_last_feedback_sequence', -1)))
            new_prediction = (
                self.prediction_sequence
                > int(track.get('pass_last_prediction_sequence', -1)))
            new_evidence = new_feedback or new_prediction
            if new_feedback:
                track['pass_last_feedback_sequence'] = self.mppi_feedback_sequence
            if new_prediction:
                track['pass_last_prediction_sequence'] = self.prediction_sequence
            if new_evidence:
                if before_entry and (feedback_blocks or timing_blocks):
                    track['pass_block_count'] = int(
                        track.get('pass_block_count', 0)) + 1
                else:
                    track['pass_block_count'] = 0
            # A nominal travel-time deadline is diagnostic only.  Curvature,
            # acceleration limits and an initial in-place turn can all make a
            # safe committed pass take longer than distance/reference_speed.
            # Revoke only on fresh timing or MPPI safety evidence; the explicit
            # stalled-command check below handles genuine no-progress cases.
            if (
                before_entry
                and track['pass_block_count']
                >= self.pass_recheck_confirm_cycles
            ):
                if feedback_blocks:
                    reason = 'pass_recheck_mppi_blocked'
                else:
                    reason = 'pass_recheck_time_window_closed'
                self.fallback_from_pass(track, snapshot, reason)
                return True
            if cleared:
                self.active_conflict = None
                self.pass_stall_since = None
                return False
            # Readiness is checked before release, but the controller can
            # still lose translational intent after the state changes. Path
            # progress is the authoritative execution signal: before entering
            # the hard zone, revoke a PASS that fails to advance 5 cm within
            # the configured timeout, regardless of angular chatter.
            if (
                before_entry
                and snapshot['now'] - float(track['pass_last_progress_time'])
                >= float(self.pass_stall_timeout)
            ):
                self.mppi_block_until = (
                    snapshot['now'] + float(self.blocked_redecision_hold))
                self.fallback_from_pass(
                    track, snapshot, 'pass_recheck_no_path_progress')
                return True
            # Once physically inside the conflict, do not switch to a planned
            # WAIT: its stop line is already behind the robot. MPPI keeps
            # steering and Collision Monitor retains the final veto.
            command_fresh = (
                snapshot['now'] - self.nav_cmd_received
                <= float(self.mppi_feedback_timeout))
            stalled = (
                command_fresh
                and abs(self.nav_cmd_linear) <= float(self.pass_stall_speed)
                and abs(self.linear_speed) <= float(self.pass_stall_speed)
                and abs(self.nav_cmd_angular)
                <= float(self.pass_stall_angular_speed)
                and abs(self.angular_speed)
                <= float(self.pass_stall_angular_speed)
                and snapshot['heading_error']
                <= float(self.fast_cruise_max_heading_error))
            if stalled:
                if self.pass_stall_since is None:
                    self.pass_stall_since = snapshot['now']
                elif (
                    snapshot['now'] - self.pass_stall_since
                    >= float(self.pass_stall_timeout)
                ):
                    if before_entry:
                        self.mppi_block_until = (
                            snapshot['now'] + float(self.blocked_redecision_hold))
                        self.fallback_from_pass(
                            track, snapshot, 'pass_recheck_no_progress')
                        return True
            else:
                self.pass_stall_since = None
            self.publish_track(
                'PASS_COMMITTED', track, snapshot, 'conflict_locked')
            return True

        if status == 'TOO_LATE_BYPASS':
            if cleared:
                self.active_conflict = None
                return False
            if matched is None:
                if track.get('safe_since') is None:
                    track['safe_since'] = snapshot['now']
                if snapshot['now'] - track['safe_since'] >= float(self.conflict_release_time):
                    self.active_conflict = None
                    return False
            else:
                track['safe_since'] = None
            self.publish_track(
                'TOO_LATE_BYPASS', track, snapshot, 'too_late_conflict_locked')
            return True

        distance_to_stop = self.distance_to_fixed_stop(track, snapshot)
        track['distance_to_stop'] = distance_to_stop
        hard_entry_s = track.get(
            'hard_conflict_s_enter', track['conflict_s_enter'])
        robot_entry_from_stop = self.travel_time_for_distance(
            hard_entry_s - track['stop_s'], 0.0)
        track['robot_entry_from_stop'] = robot_entry_from_stop
        if matched is None:
            # One missing prediction/match is not proof that the obstacle has
            # left. Require the same release hysteresis used elsewhere before
            # opening a window; this prevents a 0.1-0.2 s inference gap from
            # releasing a held robot into the crossing.
            missing_duration = max(
                0.0, snapshot['now'] - float(track['last_seen_time']))
            missing_is_safe = (
                missing_duration >= float(self.conflict_release_time))
            safe_delay = -1.0 if missing_is_safe else math.inf
            robot_clears_first = False
        elif not matched.get('hard_occupied', True):
            safe_delay = -1.0
            robot_clears_first = False
        else:
            safe_delay = (
                matched['obstacle_hard_t_exit']
                + float(self.release_time_margin)
                - robot_entry_from_stop)
            robot_clears_first = (
                matched['robot_hard_t_exit'] + float(self.release_time_margin)
                < matched['obstacle_hard_t_enter'])
        track['safe_release_delay'] = safe_delay
        track['robot_clears_first_now'] = robot_clears_first
        safe_now = safe_delay <= 0.0 or robot_clears_first

        if safe_now:
            if track.get('safe_since') is None:
                track['safe_since'] = snapshot['now']
        else:
            track['safe_since'] = None

        if status == 'STOP_COMMITTED':
            track['status'] = 'APPROACH_STOP_LINE'
            self.publish_track(
                'STOP_COMMITTED', track, snapshot, 'stop_line_committed')
            return True

        if status == 'APPROACH_STOP_LINE':
            if distance_to_stop <= float(self.stop_reached_tolerance):
                track['status'] = 'WAIT_AT_STOP_LINE'
                self.publish_track(
                    'WAIT_AT_STOP_LINE', track, snapshot, 'stop_line_reached', True)
                return True
            if (
                track.get('safe_since') is not None
                and snapshot['now'] - track['safe_since']
                >= float(self.conflict_release_time)
                and self.pass_execution_ready(snapshot)
            ):
                track['status'] = 'PASS_COMMITTED'
                track['safe_since'] = None
                self.arm_pass_commit(track, snapshot)
                self.publish_track(
                    'PASS_COMMITTED', track, snapshot, 'safe_before_stop')
                return True
            self.publish_track(
                'APPROACH_STOP_LINE', track, snapshot, 'approaching_fixed_stop')
            return True

        if status == 'WAIT_AT_STOP_LINE':
            if safe_now:
                track['status'] = 'PREPARE_TO_PASS'
                track['prepare_started'] = snapshot['now']
                self.prepare_command(track, snapshot)
                self.publish_track(
                    'PREPARE_TO_PASS', track, snapshot,
                    'safe_window_available', True)
                return True
            if safe_delay <= float(self.prepare_lead_time):
                track['status'] = 'PREPARE_TO_PASS'
                track['prepare_started'] = snapshot['now']
                self.prepare_command(track, snapshot)
                self.publish_track(
                    'PREPARE_TO_PASS', track, snapshot, 'release_window_approaching', True)
                return True
            self.publish_track(
                'WAIT_AT_STOP_LINE', track, snapshot, 'waiting_for_safe_window', True)
            return True

        if status == 'PREPARE_TO_PASS':
            self.prepare_command(track, snapshot)
            if not safe_now and safe_delay > float(self.prepare_lead_time):
                track['status'] = 'WAIT_AT_STOP_LINE'
                track['prepare_angular_z'] = 0.0
                track['prepare_aligned_since'] = None
                self.publish_track(
                    'WAIT_AT_STOP_LINE', track, snapshot,
                    'safe_window_closed', True)
                return True
            aligned_since = track.get('prepare_aligned_since')
            aligned = (
                aligned_since is not None
                and snapshot['now'] - aligned_since >= float(self.prepare_settle_time))
            safe_stable = (
                track.get('safe_since') is not None
                and snapshot['now'] - track['safe_since'] >= float(self.prepare_settle_time))
            track['pass_execution_ready'] = self.pass_execution_ready(snapshot)
            if aligned and safe_stable and track['pass_execution_ready']:
                track['status'] = 'PASS_COMMITTED'
                track['safe_since'] = None
                track['prepare_angular_z'] = 0.0
                self.arm_pass_commit(track, snapshot)
                self.publish_track(
                    'PASS_COMMITTED', track, snapshot, 'prepared_safe_release')
                return True
            self.publish_track(
                'PREPARE_TO_PASS', track, snapshot, 'fixed_departure_alignment', True)
            return True

        self.active_conflict = None
        return False

    def pending_matches(self, candidate):
        return self.pending_conflict is not None and self.candidate_matches(
            candidate, self.pending_conflict)

    def create_track(self, candidate, snapshot, relation, confirm_count):
        now = snapshot['now']
        track = dict(candidate)
        self.decision_sequence += 1
        track['decision_id'] = self.decision_sequence
        track['conflict_id'] = (
            f"{candidate['plan_version']}:{candidate['obstacle_id']}:"
            f"{candidate['conflict_world_x']:.2f}:"
            f"{candidate['conflict_world_y']:.2f}")
        track['relation'] = relation
        track['confirm_count'] = confirm_count
        track['first_seen_time'] = (
            self.pending_conflict.get('first_seen_time', now)
            if self.pending_conflict else now)
        track['last_seen_time'] = now
        track['safe_since'] = None
        braking_distance = (
            abs(self.linear_speed) * abs(self.linear_speed)
            / (2.0 * self.comfortable_deceleration))
        stop_offset = (
            braking_distance
            + float(self.half_vehicle_length)
            + float(self.static_stop_margin))
        stop_s = candidate['conflict_s_enter'] - stop_offset
        track['braking_distance'] = braking_distance
        track['stop_offset'] = stop_offset
        track['stop_s'] = stop_s
        if stop_s < snapshot['base_s'] - float(self.stop_reached_tolerance):
            track['status'] = 'TOO_LATE_BYPASS'
            return track
        stop_pose = self.pose_at_s(
            snapshot['poses'], snapshot['cumulative'], stop_s)
        track['stop_pose'] = {
            'x': float(stop_pose.pose.position.x),
            'y': float(stop_pose.pose.position.y),
            'yaw': yaw_from_quaternion(stop_pose.pose.orientation),
        }
        departure_s = min(
            snapshot['cumulative'][-1],
            candidate['conflict_s_exit']
            + float(self.departure_heading_lookahead))
        track['departure_yaw'] = self.tangent_yaw_at_s(
            snapshot['poses'], snapshot['cumulative'], departure_s)
        track['distance_to_stop'] = stop_s - snapshot['base_s']
        if relation == 'PASS':
            track['status'] = 'PASS_COMMITTED'
            self.arm_pass_commit(track, snapshot)
            return track
        track['status'] = 'STOP_COMMITTED'
        return track

    def evaluate(self):
        snapshot = self.input_snapshot()
        if snapshot is None:
            return
        if self.evaluate_active_track(snapshot):
            return

        # SIPP timing is invalid during the initial turn of a new competition
        # leg.  This bypass is one-shot; ordinary replans for the same goal
        # must not repeatedly restart it in the middle of the route.
        if self.departure_turn_pending:
            at_departure = (
                snapshot['base_s'] <= float(self.departure_turn_path_distance))
            needs_turn = (
                snapshot['heading_error'] >= float(self.departure_turn_heading_error))
            if at_departure and needs_turn:
                self.pending_conflict = None
                details = self.decorate_speed(
                    {'reason': 'new_goal_departure_turn_sipp_bypass'},
                    'ENDPOINT_TURN', snapshot)
                self.publish('ENDPOINT_TURN', details)
                return
            self.departure_turn_pending = False

        candidate = self.relevant_candidate(snapshot['candidates'])
        if candidate is None:
            self.pending_conflict = None
            if snapshot['remaining'] <= float(self.endpoint_distance):
                details = self.decorate_speed(
                    {'reason': 'endpoint_sipp_bypass'}, 'ENDPOINT_TURN', snapshot)
                self.publish('ENDPOINT_TURN', details)
            else:
                details = self.decorate_speed(
                    {'reason': 'no_spatiotemporal_conflict'}, 'CRUISE', snapshot)
                self.publish('CRUISE', details)
            return

        if self.pending_matches(candidate):
            self.pending_conflict['confirm_count'] += 1
            self.pending_conflict.update(candidate)
            self.pending_conflict['last_seen_time'] = snapshot['now']
        else:
            self.pending_conflict = dict(candidate)
            self.pending_conflict['confirm_count'] = 1
            self.pending_conflict['first_seen_time'] = snapshot['now']
            self.pending_conflict['last_seen_time'] = snapshot['now']
        confirmation = self.pending_conflict['confirm_count']
        if confirmation < self.conflict_confirm_cycles:
            details = self.track_details(self.pending_conflict)
            details['reason'] = 'conflict_detected'
            self.decorate_speed(details, 'DETECTED', snapshot)
            self.publish(
                'DETECTED', details, conflict=self.conflict_pose(candidate))
            return

        if not candidate.get('hard_occupied', True):
            relation = 'PASS'
            relation_reason = 'no_hard_zone_occupancy'
        elif (
            candidate['robot_hard_t_exit'] + float(self.release_time_margin)
            < candidate['obstacle_hard_t_enter']
        ):
            relation = 'PASS'
            relation_reason = 'robot_clears_hard_zone_first'
        else:
            relation = 'STOP'
            relation_reason = 'overlapping_hard_time_windows'
        feedback = self.valid_mppi_feedback(snapshot['now'])
        if relation == 'PASS' and feedback is not None:
            feedback_status = str(feedback.get('status', 'UNKNOWN'))
            safe_ratio = float(feedback.get('safe_trajectory_ratio', 1.0))
            if (
                feedback_status in {
                    'INFEASIBLE', 'TEMPORARILY_BLOCKED', 'SAFETY_OVERRIDE'}
                or safe_ratio < float(self.mppi_pass_min_safe_ratio)
            ):
                relation = 'STOP'
                relation_reason = 'mppi_feedback_blocks_pass'
        if relation == 'PASS' and snapshot['now'] < self.mppi_block_until:
            relation = 'STOP'
            relation_reason = 'mppi_redecision_hold'
        track = self.create_track(candidate, snapshot, relation, confirmation)
        track['decision_reason'] = relation_reason
        self.pending_conflict = None
        self.active_conflict = track
        if track['status'] == 'PASS_COMMITTED':
            self.publish_track(
                'PASS_COMMITTED', track, snapshot, relation_reason)
        elif track['status'] == 'TOO_LATE_BYPASS':
            self.publish_track(
                'TOO_LATE_BYPASS', track, snapshot, 'stop_line_behind_robot')
        else:
            self.publish_track(
                'STOP_COMMITTED', track, snapshot, relation_reason)


def main(args=None):
    rclpy.init(args=args)
    node = LightweightSippDecision()
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
