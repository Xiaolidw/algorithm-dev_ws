#!/usr/bin/env python3

"""Dynamic-obstacle trajectory predictor.

The ROS interface remains unchanged:

Inputs:
    /moving_obstacle_1/current_pose
    /moving_obstacle_2/current_pose

Output:
    /moon_warehouse/dynamic_obstacle_trajectories

Prediction order:
    LSTM -> route-reflection fallback
"""

from collections import defaultdict, deque
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import (
    get_package_share_directory,
)
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
try:
    import torch
except ImportError:
    # The competition image does not ship PyTorch.  The node already has a
    # deterministic route-reflection predictor; keep the safety pipeline
    # available instead of aborting the whole dynamic-avoidance launch.
    torch = None

from moon_warehouse_interfaces.msg import (
    ObstacleTrajectory,
    ObstacleTrajectoryArray,
)


class LstmObstaclePredictor(Node):

    def __init__(self):
        super().__init__(
            'dynamic_obstacle_predictor_node'
        )

        self.declare_node_parameters()

        self.prediction_horizon = max(
            0.5,
            float(
                self.get_parameter(
                    'prediction_horizon'
                ).value
            ),
        )

        self.prediction_dt = max(
            0.02,
            float(
                self.get_parameter(
                    'prediction_dt'
                ).value
            ),
        )

        self.history_size = max(
            3,
            int(
                self.get_parameter(
                    'history_size'
                ).value
            ),
        )

        self.frame_id = str(
            self.get_parameter('frame_id').value
        )

        self.predictor_mode = str(
            self.get_parameter(
                'predictor_mode'
            ).value
        ).lower()

        if self.predictor_mode == 'lstm' and torch is None:
            self.get_logger().warn(
                'PyTorch is unavailable; using route-reflection fallback '
                'for dynamic-obstacle prediction.'
            )
            self.predictor_mode = 'fallback'

        self.model_filename = str(
            self.get_parameter(
                'model_filename'
            ).value
        )

        self.normalization_filename = str(
            self.get_parameter(
                'normalization_filename'
            ).value
        )

        self.observation_timeout = max(
            0.1,
            float(
                self.get_parameter(
                    'observation_timeout'
                ).value
            ),
        )

        self.maximum_speed = max(
            0.1,
            float(
                self.get_parameter(
                    'maximum_speed'
                ).value
            ),
        )

        self.teleport_reset_distance = max(
            0.05,
            float(self.get_parameter('teleport_reset_distance').value),
        )
        self.teleport_reset_speed = max(
            self.maximum_speed,
            float(self.get_parameter('teleport_reset_speed').value),
        )
        self.history_gap_reset = max(
            self.prediction_dt,
            float(self.get_parameter('history_gap_reset').value),
        )
        self.route_attachment_tolerance = max(
            0.05,
            float(self.get_parameter('route_attachment_tolerance').value),
        )
        self.recent_velocity_window = max(
            2, int(self.get_parameter('recent_velocity_window').value)
        )
        self.minimum_route_speed = min(
            self.maximum_speed,
            max(0.02, float(self.get_parameter('minimum_route_speed').value)),
        )

        self.inference_timeout_ms = max(
            1.0,
            float(
                self.get_parameter(
                    'inference_timeout_ms'
                ).value
            ),
        )

        self.enable_fallback = bool(
            self.get_parameter(
                'enable_fallback'
            ).value
        )

        self.routes = self.load_routes()

        self.model = None
        self.model_ready = False

        self.input_mean = None
        self.input_std = None
        self.target_mean = None
        self.target_std = None

        self.last_inference_warning_time = 0.0
        self.last_inference_error_time = 0.0

        if self.predictor_mode == 'lstm':
            self.load_lstm_model()

        self.histories = defaultdict(
            lambda: deque(
                maxlen=self.history_size
            )
        )

        self.pose_subscriptions = []

        for obstacle_id in self.routes:
            topic_name = (
                f'/{obstacle_id}/current_pose'
            )

            subscription = (
                self.create_subscription(
                    Pose,
                    topic_name,
                    lambda message, name=obstacle_id:
                        self.pose_callback(
                            name,
                            message,
                        ),
                    20,
                )
            )

            self.pose_subscriptions.append(
                subscription
            )

        self.trajectory_publisher = (
            self.create_publisher(
                ObstacleTrajectoryArray,
                (
                    '/moon_warehouse/'
                    'dynamic_obstacle_trajectories'
                ),
                10,
            )
        )

        self.prediction_timer = (
            self.create_timer(
                self.prediction_dt,
                self.timer_callback,
            )
        )

        if self.model_ready:
            active_mode = 'lstm'
        else:
            active_mode = 'route_reflection_fallback'

        self.get_logger().info(
            'Dynamic obstacle predictor ready: '
            f'mode={active_mode}, '
            f'history={self.history_size}, '
            f'horizon={self.prediction_horizon:.1f}s, '
            f'dt={self.prediction_dt:.2f}s'
        )

        self.get_logger().info(
            'Publishing predictions to: '
            '/moon_warehouse/'
            'dynamic_obstacle_trajectories'
        )

    def declare_node_parameters(self):
        self.declare_parameter(
            'prediction_horizon',
            3.0,
        )

        self.declare_parameter(
            'prediction_dt',
            0.1,
        )

        self.declare_parameter(
            'history_size',
            20,
        )

        self.declare_parameter(
            'frame_id',
            'map',
        )

        self.declare_parameter(
            'predictor_mode',
            'lstm',
        )

        self.declare_parameter(
            'model_filename',
            'dynamic_obstacle_lstm.ts',
        )

        self.declare_parameter(
            'normalization_filename',
            'normalization.json',
        )

        self.declare_parameter(
            'observation_timeout',
            0.5,
        )

        self.declare_parameter(
            'maximum_speed',
            0.8,
        )

        # Reset recurrent history when a simulator object is teleported or
        # observations resume after a gap.  Mixing the old route with the new
        # pose creates a fresh message containing a stale "ghost" trajectory.
        self.declare_parameter('teleport_reset_distance', 0.60)
        self.declare_parameter('teleport_reset_speed', 2.00)
        self.declare_parameter('history_gap_reset', 0.40)
        self.declare_parameter('route_attachment_tolerance', 0.35)
        self.declare_parameter('recent_velocity_window', 6)
        self.declare_parameter('minimum_route_speed', 0.12)

        self.declare_parameter(
            'inference_timeout_ms',
            20.0,
        )

        self.declare_parameter(
            'enable_fallback',
            True,
        )

        self.declare_parameter(
            'route_ids',
            [
                'moving_obstacle_1',
                'moving_obstacle_2',
            ],
        )

        self.declare_parameter(
            'route_axes',
            ['x', 'x'],
        )

        self.declare_parameter(
            'route_mins',
            [-2.0, 5.5],
        )

        self.declare_parameter(
            'route_maxs',
            [1.0, 8.0],
        )

        self.declare_parameter(
            'route_fixed_coordinates',
            [2.8, -6.0],
        )

    def load_routes(self):
        route_ids = list(
            self.get_parameter(
                'route_ids'
            ).value
        )

        route_axes = list(
            self.get_parameter(
                'route_axes'
            ).value
        )

        route_mins = list(
            self.get_parameter(
                'route_mins'
            ).value
        )

        route_maxs = list(
            self.get_parameter(
                'route_maxs'
            ).value
        )

        fixed_coordinates = list(
            self.get_parameter(
                'route_fixed_coordinates'
            ).value
        )

        lengths = {
            len(route_ids),
            len(route_axes),
            len(route_mins),
            len(route_maxs),
            len(fixed_coordinates),
        }

        if len(lengths) != 1:
            raise ValueError(
                'Dynamic-obstacle route parameter '
                'arrays must have equal length.'
            )

        routes = {}

        for (
            obstacle_id,
            axis,
            minimum,
            maximum,
            fixed_coordinate,
        ) in zip(
            route_ids,
            route_axes,
            route_mins,
            route_maxs,
            fixed_coordinates,
        ):
            routes[str(obstacle_id)] = {
                'axis': str(axis),
                'minimum': float(minimum),
                'maximum': float(maximum),
                'fixed_coordinate': float(
                    fixed_coordinate
                ),
            }

        return routes

    def load_lstm_model(self):
        try:
            package_share = Path(
                get_package_share_directory(
                    'moon_warehouse_dynamic_avoidance'
                )
            )

            model_directory = (
                package_share / 'models'
            )

            model_path = (
                model_directory
                / self.model_filename
            )

            normalization_path = (
                model_directory
                / self.normalization_filename
            )

            if not model_path.is_file():
                raise FileNotFoundError(
                    f'Model was not found: {model_path}'
                )

            if not normalization_path.is_file():
                raise FileNotFoundError(
                    'Normalization file was not found: '
                    f'{normalization_path}'
                )

            with normalization_path.open(
                encoding='utf-8',
            ) as json_file:
                normalization = json.load(
                    json_file
                )

            torch.set_num_threads(1)

            self.model = torch.jit.load(
                str(model_path),
                map_location='cpu',
            )

            self.model.eval()

            self.input_mean = torch.tensor(
                normalization['input_mean'],
                dtype=torch.float32,
            ).view(1, 1, 4)

            self.input_std = torch.tensor(
                normalization['input_std'],
                dtype=torch.float32,
            ).view(1, 1, 4)

            self.target_mean = torch.tensor(
                normalization['target_mean'],
                dtype=torch.float32,
            ).view(1, 1, 2)

            self.target_std = torch.tensor(
                normalization['target_std'],
                dtype=torch.float32,
            ).view(1, 1, 2)

            self.model_ready = True

            self.get_logger().info(
                f'Loaded LSTM model: {model_path}'
            )

            self.get_logger().info(
                'Loaded normalization parameters: '
                f'{normalization_path}'
            )

        except Exception as error:
            self.model_ready = False

            message = (
                'Failed to load LSTM model: '
                f'{error}'
            )

            if self.enable_fallback:
                self.get_logger().error(
                    message
                    + '; route-reflection fallback '
                    'will be used.'
                )
            else:
                raise RuntimeError(
                    message
                ) from error

    def pose_callback(
        self,
        obstacle_id,
        message,
    ):
        timestamp = (
            self.get_clock().now().nanoseconds
            * 1e-9
        )

        history = self.histories[obstacle_id]
        current_x = float(message.position.x)
        current_y = float(message.position.y)
        reset_reason = None
        if history:
            previous = history[-1]
            dt = timestamp - previous['timestamp']
            displacement = math.hypot(
                current_x - previous['x'],
                current_y - previous['y'],
            )
            # Gazebo can publish more than once at the same simulation stamp.
            # Replace that sample instead of dividing by an almost-zero dt and
            # falsely classifying ordinary motion as a teleport.
            if dt <= 1e-6:
                if displacement > self.teleport_reset_distance:
                    history.clear()
                else:
                    previous.update({
                        'x': current_x,
                        'y': current_y,
                        'z': float(message.position.z),
                    })
                    return
                reset_reason = f'same_stamp_jump={displacement:.3f}m'
            elif dt > self.history_gap_reset:
                reset_reason = f'observation_gap={dt:.3f}s'
            elif displacement > self.teleport_reset_distance:
                reset_reason = f'position_jump={displacement:.3f}m'
            else:
                observed_speed = displacement / dt
                if observed_speed > self.teleport_reset_speed:
                    reset_reason = f'implausible_speed={observed_speed:.3f}m/s'

        if reset_reason is not None:
            history.clear()
            self.get_logger().warning(
                f'Reset prediction history for {obstacle_id}: {reset_reason}'
            )

        history.append({
            'timestamp': timestamp,
            'x': current_x,
            'y': current_y,
            'z': float(message.position.z),
        })

    def limit_velocity(
        self,
        velocity_x,
        velocity_y,
    ):
        speed = math.hypot(
            velocity_x,
            velocity_y,
        )

        if speed <= self.maximum_speed:
            return velocity_x, velocity_y

        scale = self.maximum_speed / speed

        return (
            velocity_x * scale,
            velocity_y * scale,
        )

    def calculate_sample_velocity(
        self,
        points,
        index,
    ):
        if index == 0:
            previous_point = points[0]
            next_point = points[1]

        elif index == len(points) - 1:
            previous_point = points[-2]
            next_point = points[-1]

        else:
            previous_point = points[index - 1]
            next_point = points[index + 1]

        time_delta = (
            next_point['timestamp']
            - previous_point['timestamp']
        )

        if time_delta <= 1e-6:
            return 0.0, 0.0

        velocity_x = (
            next_point['x']
            - previous_point['x']
        ) / time_delta

        velocity_y = (
            next_point['y']
            - previous_point['y']
        ) / time_delta

        return self.limit_velocity(
            velocity_x,
            velocity_y,
        )

    def build_model_input(self, history):
        points = list(history)

        current_x = points[-1]['x']
        current_y = points[-1]['y']

        features = []

        for index, point in enumerate(points):
            velocity_x, velocity_y = (
                self.calculate_sample_velocity(
                    points,
                    index,
                )
            )

            features.append([
                point['x'] - current_x,
                point['y'] - current_y,
                velocity_x,
                velocity_y,
            ])

        input_tensor = torch.tensor(
            features,
            dtype=torch.float32,
        ).view(
            1,
            self.history_size,
            4,
        )

        normalized_input = (
            input_tensor - self.input_mean
        ) / self.input_std

        return normalized_input

    def run_lstm_inference(self, history):
        model_input = self.build_model_input(
            history
        )

        start_time = time.perf_counter()

        with torch.no_grad():
            normalized_output = self.model(
                model_input
            )

            relative_output = (
                normalized_output
                * self.target_std
                + self.target_mean
            )

        elapsed_ms = (
            time.perf_counter() - start_time
        ) * 1000.0

        current_monotonic_time = time.monotonic()

        if (
            elapsed_ms > self.inference_timeout_ms
            and current_monotonic_time
            - self.last_inference_warning_time
            > 5.0
        ):
            self.get_logger().warning(
                'LSTM inference exceeded time limit: '
                f'{elapsed_ms:.2f} ms > '
                f'{self.inference_timeout_ms:.2f} ms'
            )

            self.last_inference_warning_time = (
                current_monotonic_time
            )

        if not torch.isfinite(
            relative_output
        ).all():
            raise RuntimeError(
                'LSTM output contains NaN or Inf.'
            )

        if relative_output.shape != (
            1,
            30,
            2,
        ):
            raise RuntimeError(
                'Unexpected LSTM output shape: '
                f'{tuple(relative_output.shape)}'
            )

        return (
            relative_output
            .squeeze(0)
            .cpu()
            .tolist()
        )

    def estimate_average_velocity(
        self,
        history,
    ):
        points = list(history)

        if len(points) < 3:
            return 0.0, 0.0

        first_point = points[0]
        last_point = points[-1]

        duration = (
            last_point['timestamp']
            - first_point['timestamp']
        )

        if duration <= 1e-6:
            return 0.0, 0.0

        velocity_x = (
            last_point['x']
            - first_point['x']
        ) / duration

        velocity_y = (
            last_point['y']
            - first_point['y']
        ) / duration

        return self.limit_velocity(
            velocity_x,
            velocity_y,
        )

    def estimate_route_velocity(self, history, route):
        """Estimate signed route speed without cancelling at a reversal.

        A full-history average tends to zero when the history straddles an
        endpoint.  Use recent valid increments, preserve the newest reliable
        direction, and infer the departure direction at a stationary endpoint.
        """
        points = list(history)[-(self.recent_velocity_window + 1):]
        axis = route['axis']
        values = []
        newest_direction = 0.0
        for first, second in zip(points[:-1], points[1:]):
            duration = second['timestamp'] - first['timestamp']
            if duration <= 1e-4 or duration > self.history_gap_reset:
                continue
            speed = (second[axis] - first[axis]) / duration
            if not math.isfinite(speed) or abs(speed) > self.teleport_reset_speed:
                continue
            values.append(abs(speed))
            if abs(speed) >= 0.02:
                newest_direction = math.copysign(1.0, speed)

        current = history[-1][axis]
        if newest_direction == 0.0:
            distance_to_min = abs(current - route['minimum'])
            distance_to_max = abs(route['maximum'] - current)
            newest_direction = 1.0 if distance_to_min <= distance_to_max else -1.0

        meaningful = sorted(value for value in values if value >= 0.02)
        if meaningful:
            speed = meaningful[len(meaningful) // 2]
        else:
            speed = self.minimum_route_speed
        speed = min(self.maximum_speed, max(self.minimum_route_speed, speed))
        return newest_direction * speed

    @staticmethod
    def reflect_position(
        position,
        velocity,
        minimum,
        maximum,
        future_time,
    ):
        route_length = maximum - minimum

        if route_length <= 1e-6:
            return position

        clamped_position = min(
            max(position, minimum),
            maximum,
        )

        phase = (
            clamped_position
            - minimum
            + velocity * future_time
        ) % (2.0 * route_length)

        if phase <= route_length:
            return minimum + phase

        return (
            minimum
            + 2.0 * route_length
            - phase
        )

    @staticmethod
    def constrain_to_route(
        position,
        minimum,
        maximum,
    ):
        route_length = maximum - minimum

        if route_length <= 1e-6:
            return minimum

        phase = (
            position - minimum
        ) % (2.0 * route_length)

        if phase <= route_length:
            return minimum + phase

        return (
            maximum
            - (phase - route_length)
        )

    def observation_matches_route(self, point, route):
        tolerance = self.route_attachment_tolerance
        if route['axis'] == 'x':
            cross_track = abs(point['y'] - route['fixed_coordinate'])
            along_track = point['x']
        else:
            cross_track = abs(point['x'] - route['fixed_coordinate'])
            along_track = point['y']
        return (
            cross_track <= tolerance
            and route['minimum'] - tolerance
            <= along_track
            <= route['maximum'] + tolerance
        )

    def run_fallback_prediction(
        self,
        obstacle_id,
        history,
    ):
        velocity_x, velocity_y = (
            self.estimate_average_velocity(
                history
            )
        )

        current_point = history[-1]
        current_x = current_point['x']
        current_y = current_point['y']

        route = self.routes.get(
            obstacle_id
        )
        if route is not None and not self.observation_matches_route(
            current_point, route
        ):
            route = None

        if route is not None:
            route_velocity = self.estimate_route_velocity(history, route)
            if route['axis'] == 'x':
                velocity_x, velocity_y = route_velocity, 0.0
            else:
                velocity_x, velocity_y = 0.0, route_velocity

        number_of_steps = max(
            1,
            int(
                self.prediction_horizon
                / self.prediction_dt
            ),
        )

        relative_predictions = []

        for step in range(
            1,
            number_of_steps + 1,
        ):
            future_time = (
                step * self.prediction_dt
            )

            if route is None:
                future_x = (
                    current_x
                    + velocity_x * future_time
                )

                future_y = (
                    current_y
                    + velocity_y * future_time
                )

            elif route['axis'] == 'x':
                future_x = self.reflect_position(
                    current_x,
                    velocity_x,
                    route['minimum'],
                    route['maximum'],
                    future_time,
                )

                future_y = (
                    route['fixed_coordinate']
                )

            else:
                future_x = (
                    route['fixed_coordinate']
                )

                future_y = self.reflect_position(
                    current_y,
                    velocity_y,
                    route['minimum'],
                    route['maximum'],
                    future_time,
                )

            relative_predictions.append([
                future_x - current_x,
                future_y - current_y,
            ])

        return relative_predictions

    def predict_obstacle(
        self,
        obstacle_id,
        history,
    ):
        if (
            self.model_ready
            and len(history) == self.history_size
        ):
            try:
                prediction = (
                    self.run_lstm_inference(
                        history
                    )
                )

                return prediction, 'lstm'

            except Exception as error:
                current_time = time.monotonic()

                if (
                    current_time
                    - self.last_inference_error_time
                    > 5.0
                ):
                    self.get_logger().error(
                        'LSTM inference failed: '
                        f'{error}; using fallback.'
                    )

                    self.last_inference_error_time = (
                        current_time
                    )

        if not self.enable_fallback:
            return None, 'unavailable'

        return (
            self.run_fallback_prediction(
                obstacle_id,
                history,
            ),
            'fallback',
        )

    def create_trajectory_message(
        self,
        obstacle_id,
        history,
        relative_predictions,
        prediction_mode,
    ):
        current_point = history[-1]
        route = self.routes.get(
            obstacle_id
        )
        if route is not None and not self.observation_matches_route(
            current_point, route
        ):
            route = None

        trajectory = ObstacleTrajectory()
        trajectory.id = obstacle_id

        if prediction_mode == 'lstm':
            trajectory.type = (
                'obstacle_dynamic_lstm'
            )
        else:
            trajectory.type = (
                'obstacle_dynamic_fallback'
            )

        for index, relative_position in enumerate(
            relative_predictions
        ):
            future_pose = Pose()

            predicted_x = (
                current_point['x']
                + float(relative_position[0])
            )

            predicted_y = (
                current_point['y']
                + float(relative_position[1])
            )

            if route is not None:
                if route['axis'] == 'x':
                    predicted_x = (
                        self.constrain_to_route(
                            predicted_x,
                            route['minimum'],
                            route['maximum'],
                        )
                    )

                    predicted_y = (
                        route['fixed_coordinate']
                    )

                else:
                    predicted_x = (
                        route['fixed_coordinate']
                    )

                    predicted_y = (
                        self.constrain_to_route(
                            predicted_y,
                            route['minimum'],
                            route['maximum'],
                        )
                    )

            future_pose.position.x = predicted_x
            future_pose.position.y = predicted_y

            future_pose.position.z = (
                current_point['z']
            )

            future_pose.orientation.w = 1.0

            future_time = (
                (index + 1)
                * self.prediction_dt
            )

            trajectory.future_poses.append(
                future_pose
            )

            trajectory.future_times.append(
                float(future_time)
            )

        return trajectory

    def timer_callback(self):
        current_ros_time = (
            self.get_clock().now()
        )

        current_seconds = (
            current_ros_time.nanoseconds
            * 1e-9
        )

        output_message = (
            ObstacleTrajectoryArray()
        )

        output_message.header.stamp = (
            current_ros_time.to_msg()
        )

        output_message.header.frame_id = (
            self.frame_id
        )

        for obstacle_id, history in list(
            self.histories.items()
        ):
            if len(history) < 3:
                continue

            observation_age = (
                current_seconds
                - history[-1]['timestamp']
            )

            if (
                observation_age
                > self.observation_timeout
            ):
                continue

            prediction, prediction_mode = (
                self.predict_obstacle(
                    obstacle_id,
                    history,
                )
            )

            if prediction is None:
                continue

            trajectory = (
                self.create_trajectory_message(
                    obstacle_id,
                    history,
                    prediction,
                    prediction_mode,
                )
            )

            output_message.trajectories.append(
                trajectory
            )

        # An empty array is meaningful: it actively clears stale trajectories
        # in SIPP and MPPI when every observation has disappeared or reset.
        self.trajectory_publisher.publish(
            output_message
        )


def main(arguments=None):
    rclpy.init(args=arguments)

    node = LstmObstaclePredictor()

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
