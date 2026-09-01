#!/usr/bin/env python3
"""Lightweight time-aware conflict decision layer for Nav2 paths.
The first release runs in shadow mode by default.  It observes every Nav2
global path, finds the first path corridor occupied by a predicted obstacle,
compares the robot and obstacle occupancy intervals, and publishes a decision
without changing velocity commands.
"""
import json
import math
import time
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from moon_warehouse_interfaces.msg import ObstacleTrajectoryArray
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String


class LightweightSippDecision(Node):
    def __init__(self):
        super().__init__('sipp_decision_node')
        self.declare_parameter('shadow_mode', True)
        self.declare_parameter('decision_frequency', 10.0)
        self.declare_parameter('conflict_radius', 0.95)
        self.declare_parameter('conflict_longitudinal_margin', 0.30)
        self.declare_parameter('time_margin', 0.8)
        self.declare_parameter('stop_line_distance', 0.65)
        self.declare_parameter('half_vehicle_length', 0.20)
        self.declare_parameter('static_stop_margin', 0.15)
        self.declare_parameter('comfortable_deceleration', 0.60)
        self.declare_parameter('stop_reached_tolerance', 0.12)
        self.declare_parameter('release_hold_time', 1.0)
        self.declare_parameter('minimum_prediction_speed', 0.20)
        self.declare_parameter('maximum_prediction_speed', 0.50)
        self.declare_parameter('endpoint_distance', 0.40)
        self.declare_parameter('turning_angular_speed', 0.15)
        self.declare_parameter('data_timeout', 0.6)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('plan_timeout', 2.5)
        self.declare_parameter('tf_timeout', 0.05)
        self.declare_parameter('maximum_path_deviation', 0.75)
        self.declare_parameter('maximum_index_backtrack', 2)
        self.declare_parameter('maximum_occupancy_gap', 0.25)
        self.declare_parameter('conflict_confirm_cycles', 3)
        self.declare_parameter('pass_commit_min_time', 0.8)
        # 2.新增冲突区匹配半径参数声明
        self.declare_parameter('commit_match_radius', 0.45)
        self.declare_parameter('too_late_min_hold_time', 0.5)

        self.shadow_mode = bool(self.get_parameter('shadow_mode').value)
        self.conflict_radius = float(self.get_parameter('conflict_radius').value)
        self.conflict_longitudinal_margin = max(
            0.05,
            float(self.get_parameter('conflict_longitudinal_margin').value),
        )
        self.time_margin = float(self.get_parameter('time_margin').value)
        self.stop_line_distance = float(self.get_parameter('stop_line_distance').value)
        self.half_vehicle_length = max(
            0.0, float(self.get_parameter('half_vehicle_length').value))
        self.static_stop_margin = max(
            0.0, float(self.get_parameter('static_stop_margin').value))
        self.comfortable_deceleration = max(
            0.05, float(self.get_parameter('comfortable_deceleration').value))
        self.stop_tolerance = float(self.get_parameter('stop_reached_tolerance').value)
        self.release_hold_time = float(self.get_parameter('release_hold_time').value)
        self.minimum_speed = float(self.get_parameter('minimum_prediction_speed').value)
        self.maximum_speed = float(self.get_parameter('maximum_prediction_speed').value)
        self.endpoint_distance = float(self.get_parameter('endpoint_distance').value)
        self.turning_speed = float(self.get_parameter('turning_angular_speed').value)
        self.data_timeout = float(self.get_parameter('data_timeout').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.robot_base_frame = str(self.get_parameter('robot_base_frame').value)
        self.plan_timeout = max(
            0.5,
            float(self.get_parameter('plan_timeout').value),
        )
        self.tf_timeout = max(
            0.01,
            float(self.get_parameter('tf_timeout').value),
        )
        self.maximum_path_deviation = max(
            0.1,
            float(self.get_parameter('maximum_path_deviation').value),
        )
        self.maximum_index_backtrack = max(
            0,
            int(self.get_parameter('maximum_index_backtrack').value),
        )
        self.maximum_occupancy_gap = max(
            0.05,
            float(self.get_parameter('maximum_occupancy_gap').value),
        )
        self.conflict_confirm_cycles = max(
            1,
            int(self.get_parameter('conflict_confirm_cycles').value),
        )
        self.pass_commit_min_time = max(
            0.0,
            float(self.get_parameter('pass_commit_min_time').value),
        )
        # 读取新增commit_match_radius，下限0.10
        self.commit_match_radius = max(
            0.10,
            float(
                self.get_parameter(
                    'commit_match_radius'
                ).value
            ),
        )
        self.too_late_min_hold_time = max(
            0.0,
            float(
                self.get_parameter(
                    'too_late_min_hold_time'
                ).value
            ),
        )

        # TF init
        self.tf_buffer = Buffer(
            cache_time=Duration(seconds=5.0)
        )
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )
        self.plan = None
        self.plan_received = 0.0
        self.predictions = None
        self.predictions_received = 0.0
        self.robot_pose = None
        self.odom_pose = None
        self.odom_received = 0.0
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.state = 'IDLE'
        self.safe_since = None
        # 3.增加已提交冲突区世界坐标成员
        self.committed_obstacle_id = None
        self.committed_conflict_x = None
        self.committed_conflict_y = None
        self.pass_committed_since = None
        self.too_late_obstacle_id = None
        self.too_late_conflict_x = None
        self.too_late_conflict_y = None
        self.too_late_since = None
        self.pending_conflict_id = None
        self.pending_conflict_count = 0
        self.committed_stop_line = None
        self.wait_committed_conflict_id = None
        self.last_nearest_index = None
        self.plan_sequence = 0
        self.current_diagnostics = {}

        self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 20)
        self.create_subscription(
            ObstacleTrajectoryArray,
            '/moon_warehouse/dynamic_obstacle_trajectories',
            self.prediction_callback,
            10,
        )
        self.state_publisher = self.create_publisher(String, '/sipp/state', 10)
        self.decision_publisher = self.create_publisher(String, '/sipp/decision', 10)
        self.hold_publisher = self.create_publisher(Bool, '/sipp/hold', 10)
        self.stop_line_publisher = self.create_publisher(
            PoseStamped, '/sipp/stop_line', 10)
        self.conflict_publisher = self.create_publisher(
            PoseStamped, '/sipp/conflict_point', 10)
        self.remaining_path_publisher = self.create_publisher(
            Float32,
            '/sipp/remaining_path',
            10,
        )
        self.nearest_index_publisher = self.create_publisher(
            Int32,
            '/sipp/nearest_path_index',
            10,
        )
        frequency = max(1.0, float(self.get_parameter('decision_frequency').value))
        self.create_timer(1.0 / frequency, self.evaluate)
        self.get_logger().info(
            'Lightweight SIPP ready: '
            f'shadow={self.shadow_mode} '
            f'radius={self.conflict_radius:.2f} '
            f'time_margin={self.time_margin:.2f}'
        )

    def plan_callback(self, message):
        self.plan = message
        self.plan_received = time.monotonic()
        # Nav2每次重新发布路径，都视为一版新计划。
        # 原始路径索引可以重新从0附近开始。
        self.plan_sequence += 1
        self.last_nearest_index = None

    def odom_callback(self, message):
        self.linear_speed = float(
            message.twist.twist.linear.x
        )
        self.angular_speed = float(
            message.twist.twist.angular.z
        )
        self.odom_pose = message.pose.pose
        self.odom_received = time.monotonic()

    def prediction_callback(self, message):
        self.predictions = message
        self.predictions_received = time.monotonic()

    @staticmethod
    def normalize_frame(frame_id):
        return str(frame_id).strip().lstrip('/')

    def plan_frame_id(self):
        if self.plan is None:
            return ''
        frame_id = self.normalize_frame(
            self.plan.header.frame_id
        )
        if not frame_id and self.plan.poses:
            frame_id = self.normalize_frame(
                self.plan.poses[0].header.frame_id
            )
        return frame_id

    def prediction_frame_id(self):
        if self.predictions is None:
            return ''
        return self.normalize_frame(
            self.predictions.header.frame_id
        )

    def lookup_robot_pose(self):
        transform = self.tf_buffer.lookup_transform(
            self.global_frame,
            self.robot_base_frame,
            Time(),
            timeout=Duration(seconds=self.tf_timeout),
        )
        pose = Pose()
        pose.position.x = (
            transform.transform.translation.x
        )
        pose.position.y = (
            transform.transform.translation.y
        )
        pose.position.z = (
            transform.transform.translation.z
        )
        pose.orientation = transform.transform.rotation
        return pose

    @staticmethod
    def distance(first, second):
        return math.hypot(first.position.x - second.position.x,
                          first.position.y - second.position.y)

    @staticmethod
    def cumulative_distances(poses):
        result = [0.0]
        for index in range(1, len(poses)):
            result.append(
                result[-1] + LightweightSippDecision.distance(
                    poses[index - 1].pose, poses[index].pose))
        return result

    def nearest_path_index(self, poses, robot_pose):
        if self.last_nearest_index is None:
            search_start = 0
        else:
            search_start = max(
                0,
                self.last_nearest_index
                - self.maximum_index_backtrack,
            )
        nearest_index = min(
            range(search_start, len(poses)),
            key=lambda index: self.distance(
                poses[index].pose,
                robot_pose,
            ),
        )
        path_deviation = self.distance(
            poses[nearest_index].pose,
            robot_pose,
        )
        self.last_nearest_index = nearest_index
        return nearest_index, path_deviation

    @staticmethod
    def pose_at_distance(poses, cumulative, target):
        index = min(
            range(len(cumulative)),
            key=lambda item: abs(cumulative[item] - target),
        )
        result = PoseStamped()
        result.header = poses[index].header
        result.pose = poses[index].pose
        return result

    @staticmethod
    def project_point_to_segment(px, py, first, second):
        ax = first.position.x
        ay = first.position.y
        bx = second.position.x
        by = second.position.y
        dx = bx - ax
        dy = by - ay
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-12:
            return 0.0, math.hypot(px - ax, py - ay)
        fraction = ((px - ax) * dx + (py - ay) * dy) / length_squared
        fraction = min(1.0, max(0.0, fraction))
        projected_x = ax + fraction * dx
        projected_y = ay + fraction * dy
        return fraction, math.hypot(px - projected_x, py - projected_y)

    def project_point_to_path(self, point, poses, cumulative, nearest_index):
        best = None
        start = max(0, nearest_index - self.maximum_index_backtrack)
        for index in range(start, len(poses) - 1):
            fraction, lateral = self.project_point_to_segment(
                point.position.x,
                point.position.y,
                poses[index].pose,
                poses[index + 1].pose,
            )
            segment_length = cumulative[index + 1] - cumulative[index]
            projected_s = cumulative[index] + fraction * segment_length
            if best is None or lateral < best['lateral']:
                best = {'s': projected_s, 'lateral': lateral}
        return best

    def make_conflict_candidate(
        self, group, trajectory_id, cumulative, base_s, speed
    ):
        zone_s_enter = min(item['s_enter'] for item in group)
        zone_s_exit = max(item['s_exit'] for item in group)
        obstacle_entry = min(item['time'] for item in group)
        obstacle_exit = max(item['time'] for item in group)
        minimum_space = min(item['lateral'] for item in group)
        conflict_s = 0.5 * (zone_s_enter + zone_s_exit)
        path_index = min(
            range(len(cumulative)),
            key=lambda index: abs(cumulative[index] - conflict_s),
        )
        return {
            'path_index': path_index,
            'conflict_s': conflict_s,
            'zone_s_enter': zone_s_enter,
            'zone_s_exit': zone_s_exit,
            'distance_ahead': max(0.0, zone_s_enter - base_s),
            'robot_entry': max(0.0, (zone_s_enter - base_s) / speed),
            'robot_exit': max(0.0, (zone_s_exit - base_s) / speed),
            'obstacle_entry': obstacle_entry,
            'obstacle_exit': obstacle_exit,
            'obstacle_id': trajectory_id,
            'minimum_space': minimum_space,
        }

    def find_conflicts(self, poses, cumulative, nearest_index, speed):
        candidates = []
        base_s = cumulative[nearest_index]
        for trajectory in self.predictions.trajectories:
            count = min(
                len(trajectory.future_poses),
                len(trajectory.future_times),
            )
            occupied = []
            for index in range(count):
                projection = self.project_point_to_path(
                    trajectory.future_poses[index],
                    poses,
                    cumulative,
                    nearest_index,
                )
                if projection is None or projection['lateral'] > self.conflict_radius:
                    continue
                occupied.append({
                    'time': float(trajectory.future_times[index]),
                    's_enter': (
                        projection['s']
                        - self.conflict_longitudinal_margin
                    ),
                    's_exit': (
                        projection['s']
                        + self.conflict_longitudinal_margin
                    ),
                    'lateral': projection['lateral'],
                })
            group = []
            for item in occupied:
                if group and item['time'] - group[-1]['time'] > self.maximum_occupancy_gap:
                    candidate = self.make_conflict_candidate(
                        group, trajectory.id, cumulative, base_s, speed)
                    if candidate['zone_s_exit'] >= base_s:
                        candidates.append(candidate)
                    group = []
                group.append(item)
            if group:
                candidate = self.make_conflict_candidate(
                    group, trajectory.id, cumulative, base_s, speed)
                if candidate['zone_s_exit'] >= base_s:
                    candidates.append(candidate)
        candidates.sort(key=lambda item: (
            max(base_s, item['zone_s_enter']),
            item['obstacle_entry'],
        ))
        return candidates

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
            'shadow_mode': self.shadow_mode,
            **self.current_diagnostics,
            **details,
        }
        self.state_publisher.publish(
            String(data=state)
        )
        self.decision_publisher.publish(
            String(
                data=json.dumps(
                    payload,
                    ensure_ascii=False,
                    sort_keys=True,
                )
            )
        )
        self.hold_publisher.publish(
            Bool(data=bool(hold and not self.shadow_mode))
        )
        remaining = payload.get(
            'remaining_path',
            math.nan,
        )
        nearest_index = payload.get(
            'nearest_path_index',
            -1,
        )
        self.remaining_path_publisher.publish(
            Float32(data=float(remaining))
        )
        self.nearest_index_publisher.publish(
            Int32(data=int(nearest_index))
        )
        if stop_line is not None:
            self.stop_line_publisher.publish(stop_line)
        if conflict is not None:
            self.conflict_publisher.publish(conflict)

    def evaluate(self):
        now = time.monotonic()
        self.current_diagnostics = {}
        # 1. 检查路径
        if self.plan is None or len(self.plan.poses) < 2:
            self.publish(
                'IDLE',
                {'reason': 'waiting_for_plan'},
            )
            return
        plan_age = now - self.plan_received
        if plan_age > self.plan_timeout:
            self.publish(
                'DEGRADED',
                {
                    'reason': 'plan_timeout',
                    'plan_age': plan_age,
                },
            )
            return
        # 2. 检查动态障碍预测
        if (
            self.predictions is None
            or not self.predictions.trajectories
        ):
            self.publish(
                'DEGRADED',
                {'reason': 'waiting_for_prediction'},
            )
            return
        prediction_age = (
            now - self.predictions_received
        )
        if prediction_age > self.data_timeout:
            self.publish(
                'DEGRADED',
                {
                    'reason': 'prediction_timeout',
                    'prediction_age': prediction_age,
                },
            )
            return
        # 3. 检查frame_id
        expected_frame = self.normalize_frame(
            self.global_frame
        )
        plan_frame = self.plan_frame_id()
        prediction_frame = self.prediction_frame_id()
        if not plan_frame:
            self.publish(
                'DEGRADED',
                {'reason': 'plan_frame_empty'},
            )
            return
        if not prediction_frame:
            self.publish(
                'DEGRADED',
                {'reason': 'prediction_frame_empty'},
            )
            return
        if plan_frame != expected_frame:
            self.publish(
                'DEGRADED',
                {
                    'reason': 'plan_frame_mismatch',
                    'expected_frame': expected_frame,
                    'plan_frame': plan_frame,
                },
            )
            return
        if prediction_frame != expected_frame:
            self.publish(
                'DEGRADED',
                {
                    'reason': 'prediction_frame_mismatch',
                    'expected_frame': expected_frame,
                    'prediction_frame': prediction_frame,
                },
            )
            return
        # 4. 查询Nav2所使用的真实TF位姿
        try:
            robot_pose = self.lookup_robot_pose()
        except TransformException as error:
            self.publish(
                'DEGRADED',
                {
                    'reason': 'tf_unavailable',
                    'tf_error': str(error),
                    'target_frame': expected_frame,
                    'source_frame': self.robot_base_frame,
                },
            )
            return
        self.robot_pose = robot_pose
        # 5. 查找机器人在当前路径上的位置
        poses = self.plan.poses
        cumulative = self.cumulative_distances(poses)
        nearest_index, path_deviation = (
            self.nearest_path_index(
                poses,
                robot_pose,
            )
        )
        if path_deviation > self.maximum_path_deviation:
            self.publish(
                'DEGRADED',
                {
                    'reason': 'robot_too_far_from_plan',
                    'path_deviation': path_deviation,
                    'maximum_path_deviation':
                        self.maximum_path_deviation,
                    'nearest_path_index': nearest_index,
                    'plan_sequence': self.plan_sequence,
                },
            )
            return
        remaining = max(
            0.0,
            cumulative[-1]
            - cumulative[nearest_index],
        )
        tf_odom_error = math.nan
        if self.odom_pose is not None:
            tf_odom_error = self.distance(
                robot_pose,
                self.odom_pose,
            )
        self.current_diagnostics = {
            'plan_age': plan_age,
            'prediction_age': prediction_age,
            'plan_frame': plan_frame,
            'prediction_frame': prediction_frame,
            'plan_sequence': self.plan_sequence,
            'nearest_path_index': nearest_index,
            'path_deviation': path_deviation,
            'remaining_path': remaining,
            'robot_x': robot_pose.position.x,
            'robot_y': robot_pose.position.y,
            'tf_odom_error': tf_odom_error,
        }
        speed = min(
            self.maximum_speed,
            max(self.minimum_speed, abs(self.linear_speed)),
        )
        candidates = self.find_conflicts(
            poses, cumulative, nearest_index, speed)

        # TOO_LATE只锁定同一障碍物、同一世界坐标冲突区。
        # 避免每帧重新进入冲突确认，形成
        # TOO_LATE -> CRUISE -> TOO_LATE 抖动。
        if (
            self.state == 'TOO_LATE_FOR_PLANNED_STOP'
            and self.too_late_obstacle_id is not None
        ):
            matched_too_late = None
            if (
                self.too_late_conflict_x is not None
                and self.too_late_conflict_y is not None
            ):
                for item in candidates:
                    if (
                        item['obstacle_id']
                        != self.too_late_obstacle_id
                    ):
                        continue
                    point = poses[
                        item['path_index']
                    ].pose.position
                    distance_to_locked = math.hypot(
                        point.x - self.too_late_conflict_x,
                        point.y - self.too_late_conflict_y,
                    )
                    if (
                        distance_to_locked
                        <= self.commit_match_radius
                    ):
                        matched_too_late = item
                        break
            too_late_age = (
                now - self.too_late_since
                if self.too_late_since is not None
                else 0.0
            )
            if (
                matched_too_late is not None
                or too_late_age < self.too_late_min_hold_time
            ):
                details = dict(matched_too_late or {})
                details.pop('path_index', None)
                details['reason'] = (
                    'too_late_conflict_locked'
                    if matched_too_late is not None
                    else 'too_late_minimum_hold'
                )
                details['too_late_hold_age'] = too_late_age
                details['too_late_conflict_x'] = (
                    self.too_late_conflict_x
                )
                details['too_late_conflict_y'] = (
                    self.too_late_conflict_y
                )
                locked_conflict_pose = None
                if matched_too_late is not None:
                    locked_conflict_pose = PoseStamped()
                    locked_conflict_pose.header = poses[
                        matched_too_late['path_index']
                    ].header
                    locked_conflict_pose.pose = poses[
                        matched_too_late['path_index']
                    ].pose
                self.publish(
                    'TOO_LATE_FOR_PLANNED_STOP',
                    details,
                    conflict=locked_conflict_pose,
                )
                return
            # 原来的冲突区已经离开剩余路径，解除锁定。
            self.too_late_obstacle_id = None
            self.too_late_conflict_x = None
            self.too_late_conflict_y = None
            self.too_late_since = None

        # 4.替换后的 PASS_COMMITTED 锁定逻辑：按世界坐标冲突区匹配
        # PASS只锁定原来的世界坐标冲突区，不能仅凭障碍物ID锁定。
        if (
            self.state == 'PASS_COMMITTED'
            and self.committed_obstacle_id is not None
        ):
            committed = None
            if (
                self.committed_conflict_x is not None
                and self.committed_conflict_y is not None
            ):
                for item in candidates:
                    if (
                        item['obstacle_id']
                        != self.committed_obstacle_id
                    ):
                        continue
                    point = poses[
                        item['path_index']
                    ].pose.position
                    distance_to_committed = math.hypot(
                        point.x - self.committed_conflict_x,
                        point.y - self.committed_conflict_y,
                    )
                    if (
                        distance_to_committed
                        <= self.commit_match_radius
                    ):
                        committed = item
                        break
            commit_age = (
                now - self.pass_committed_since
                if self.pass_committed_since is not None
                else 0.0
            )
            if (
                committed is not None
                or commit_age < self.pass_commit_min_time
            ):
                details = dict(committed or {})
                details.pop('path_index', None)
                details['reason'] = (
                    'pass_conflict_locked'
                    if committed is not None
                    else 'pass_minimum_commit_time'
                )
                details['commit_age'] = commit_age
                details['committed_conflict_x'] = (
                    self.committed_conflict_x
                )
                details['committed_conflict_y'] = (
                    self.committed_conflict_y
                )
                committed_pose = None
                if committed is not None:
                    committed_pose = PoseStamped()
                    committed_pose.header = poses[
                        committed['path_index']
                    ].header
                    committed_pose.pose = poses[
                        committed['path_index']
                    ].pose
                self.publish(
                    'PASS_COMMITTED',
                    details,
                    conflict=committed_pose,
                )
                return
            # 原冲突区已经离开剩余路径，解除PASS并重新判断。
            self.committed_obstacle_id = None
            self.committed_conflict_x = None
            self.committed_conflict_y = None
            self.pass_committed_since = None
        # 按路径前进方向选择第一个真正需要决策的时空冲突。
        conflict = None
        relation = None
        for candidate in candidates:
            if candidate['obstacle_exit'] + self.time_margin < candidate['robot_entry']:
                continue
            conflict = candidate
            if candidate['robot_exit'] + self.time_margin < candidate['obstacle_entry']:
                relation = 'PASS'
            else:
                relation = 'WAIT'
            break
        if conflict is None:
            if remaining <= self.endpoint_distance:
                self.safe_since = None
                self.pending_conflict_id = None
                self.pending_conflict_count = 0
                self.publish(
                    'ENDPOINT_TURN',
                    {'remaining_path': remaining, 'reason': 'endpoint_sipp_bypass'},
                )
                return
            if self.safe_since is None:
                self.safe_since = now
            if now - self.safe_since < self.release_hold_time and self.state in {
                'APPROACH_STOP_LINE', 'WAIT_AT_STOP_LINE'}:
                self.publish(
                    self.state,
                    {'reason': 'release_hysteresis'},
                    hold=self.state == 'WAIT_AT_STOP_LINE',
                    stop_line=self.committed_stop_line,
                )
                return
            # 6.无冲突时清理世界坐标
            self.committed_obstacle_id = None
            self.committed_conflict_x = None
            self.committed_conflict_y = None
            self.pass_committed_since = None
            self.too_late_obstacle_id = None
            self.too_late_conflict_x = None
            self.too_late_conflict_y = None
            self.too_late_since = None
            self.pending_conflict_id = None
            self.pending_conflict_count = 0
            self.committed_stop_line = None
            self.wait_committed_conflict_id = None
            self.publish('CRUISE', {'remaining_path': remaining})
            return
        conflict_pose = PoseStamped()
        conflict_pose.header = poses[conflict['path_index']].header
        conflict_pose.pose = poses[conflict['path_index']].pose
        details = {
            key: value for key, value in conflict.items() if key != 'path_index'
        }
        details['relation'] = relation
        # 已经处于计划等待时，瞬时安全帧先经过释放迟滞，避免WAIT/PASS抖动。
        if relation != 'WAIT' and self.state in {
            'APPROACH_STOP_LINE', 'WAIT_AT_STOP_LINE'}:
            if self.safe_since is None:
                self.safe_since = now
            if now - self.safe_since < self.release_hold_time:
                details['reason'] = 'release_hysteresis'
                self.publish(
                    self.state,
                    details,
                    hold=self.state == 'WAIT_AT_STOP_LINE',
                    stop_line=self.committed_stop_line,
                    conflict=conflict_pose,
                )
                return
            self.committed_stop_line = None
            self.wait_committed_conflict_id = None
            self.publish('CRUISE', details, conflict=conflict_pose)
            return
        self.safe_since = None
        if relation == 'PASS':
            # 5.提交 PASS 时保存世界坐标
            self.committed_obstacle_id = conflict['obstacle_id']
            self.committed_conflict_x = float(
                conflict_pose.pose.position.x
            )
            self.committed_conflict_y = float(
                conflict_pose.pose.position.y
            )
            self.pass_committed_since = now
            self.pending_conflict_id = None
            self.pending_conflict_count = 0
            self.publish('PASS_COMMITTED', details, conflict=conflict_pose)
            return
        conflict_id = (
            str(conflict['obstacle_id']),
            round(conflict['conflict_s'], 1),
        )
        already_waiting = (
            self.state in {'APPROACH_STOP_LINE', 'WAIT_AT_STOP_LINE'}
            and self.wait_committed_conflict_id is not None
        )
        if not already_waiting:
            if self.pending_conflict_id == conflict_id:
                self.pending_conflict_count += 1
            else:
                self.pending_conflict_id = conflict_id
                self.pending_conflict_count = 1
            details['confirmation_count'] = self.pending_conflict_count
            if self.pending_conflict_count < self.conflict_confirm_cycles:
                details['reason'] = 'conflict_confirmation'
                self.publish('CRUISE', details, conflict=conflict_pose)
                return
        # 第一次提交WAIT时固定世界坐标停止线，后续帧不再跟随预测抖动。
        if not already_waiting or self.committed_stop_line is None:
            braking_distance = (
                abs(self.linear_speed) * abs(self.linear_speed)
                / (2.0 * self.comfortable_deceleration)
            )
            stop_offset = (
                braking_distance
                + self.half_vehicle_length
                + self.static_stop_margin
            )
            stop_s = conflict['zone_s_enter'] - stop_offset
            details['braking_distance'] = braking_distance
            details['stop_offset'] = stop_offset
            if stop_s < cumulative[nearest_index] - self.stop_tolerance:
                details['stop_s'] = stop_s
                details['distance_to_stop'] = (
                    stop_s - cumulative[nearest_index]
                )
                details['reason'] = 'stop_line_behind_robot'

                # 锁定当前世界坐标冲突区，下一帧不再重新确认。
                self.too_late_obstacle_id = conflict['obstacle_id']
                self.too_late_conflict_x = float(
                    conflict_pose.pose.position.x
                )
                self.too_late_conflict_y = float(
                    conflict_pose.pose.position.y
                )
                self.too_late_since = now

                self.pending_conflict_id = None
                self.pending_conflict_count = 0

                self.publish(
                    'TOO_LATE_FOR_PLANNED_STOP',
                    details,
                    conflict=conflict_pose,
                )
                return
            self.committed_stop_line = self.pose_at_distance(
                poses, cumulative, stop_s)
            self.wait_committed_conflict_id = conflict_id
        stop_line = self.committed_stop_line
        stop_projection = self.project_point_to_path(
            stop_line.pose, poses, cumulative, nearest_index)
        stop_s = (
            stop_projection['s'] if stop_projection is not None
            else conflict['zone_s_enter'] - self.stop_line_distance
        )
        distance_to_stop = stop_s - cumulative[nearest_index]
        details['stop_s'] = stop_s
        details['distance_to_stop'] = distance_to_stop
        details['stop_line_locked'] = True
        if distance_to_stop <= self.stop_tolerance:
            self.publish(
                'WAIT_AT_STOP_LINE', details, hold=True,
                stop_line=stop_line, conflict=conflict_pose)
        else:
            self.publish(
                'APPROACH_STOP_LINE', details,
                stop_line=stop_line, conflict=conflict_pose)


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
