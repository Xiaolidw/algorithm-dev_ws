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
import math
from pathlib import Path
import time

from ament_index_python.packages import (
    get_package_share_directory,
)
from geometry_msgs.msg import Pose
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

try:
    import torch
    from torch import nn
except ImportError:
    torch = None
    nn = None

from moon_warehouse_interfaces.msg import (
    ObstacleTrajectory,
    ObstacleTrajectoryArray,
)


if nn is not None:
    class LSTMThetaModel(nn.Module):
        """V3 model: historical x/y positions to seven analytic parameters."""

        def __init__(self, input_dim=2, hidden_dim=64, num_layers=2, param_dim=7):
            super().__init__()
            self.lstm = nn.LSTM(
                input_size=input_dim,
                hidden_size=hidden_dim,
                num_layers=num_layers,
                batch_first=True,
            )
            self.fc = nn.Linear(hidden_dim, param_dim)

        def forward(self, observations):
            output, _ = self.lstm(observations)
            return self.fc(output[:, -1, :])
else:
    class LSTMThetaModel:
        def __init__(self, *args, **kwargs):
            raise RuntimeError('PyTorch is required only for predictor_mode=lstm')


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

        self.model_filename = str(
            self.get_parameter(
                'model_filename'
            ).value
        )

        self.model_path_parameter = str(
            self.get_parameter('model_path').value
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

        self.debug_theta = bool(
            self.get_parameter('debug_theta').value
        )

        self.routes = self.load_routes()

        if torch is not None:
            self.device = torch.device(
                'cuda' if torch.cuda.is_available() else 'cpu'
            )
            if self.device.type == 'cpu':
                torch.set_num_threads(1)
        else:
            self.device = 'cpu-no-torch'

        self.model = None
        self.model_ready = False
        self.model_observation_count = self.history_size
        self.model_dt = self.prediction_dt

        self.last_inference_warning_time = 0.0
        self.last_inference_error_time = 0.0

        if self.predictor_mode == 'lstm':
            self.load_lstm_model()

        # Checkpoint metadata is authoritative for the recurrent input length.
        self.history_size = self.model_observation_count

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

        self.theta_publisher = self.create_publisher(
            Float32MultiArray,
            '/moon_warehouse/dynamic_obstacle_theta',
            10,
        )

        self.prediction_timer = (
            self.create_timer(
                self.prediction_dt,
                self.timer_callback,
            )
        )

        self.add_on_set_parameters_callback(
            self.parameter_callback
        )

        if self.model_ready:
            active_mode = 'theta_lstm'
        else:
            active_mode = 'route_reflection_fallback'

        self.get_logger().info(
            'Dynamic obstacle predictor ready: '
            f'mode={active_mode}, '
            f'history={self.history_size}, '
            f'horizon={self.prediction_horizon:.1f}s, '
            f'dt={self.prediction_dt:.2f}s'
            f', device={self.device}'
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
            16,
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
            'theta_lstm.pth',
        )

        self.declare_parameter(
            'model_path',
            '',
        )

        self.declare_parameter('debug_theta', False)

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
            ['x', 'y'],
        )

        self.declare_parameter(
            'route_mins',
            [-2.0, -6.0],
        )

        self.declare_parameter(
            'route_maxs',
            [1.0, -3.0],
        )

        self.declare_parameter(
            'route_fixed_coordinates',
            [2.8, 2.0],
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
            if torch is None:
                raise RuntimeError('PyTorch is not installed')
            model_path = self.resolve_model_path()
            (
                self.model,
                self.model_observation_count,
                self.model_dt,
            ) = self.read_checkpoint(model_path)
            self.model_ready = True
            self.get_logger().info(
                f'Loaded theta LSTM: {model_path}, '
                f'obs_len={self.model_observation_count}, '
                f'model_dt={self.model_dt:.3f}s'
            )
        except Exception as error:
            self.model = None
            self.model_ready = False
            message = f'Failed to load theta LSTM: {error}'
            if self.enable_fallback:
                self.get_logger().error(
                    message + '; route-reflection fallback will be used.'
                )
            else:
                raise RuntimeError(message) from error

    def resolve_model_path(self, explicit_path=None, filename=None):
        configured = (
            self.model_path_parameter
            if explicit_path is None else str(explicit_path)
        )
        if configured:
            return Path(configured).expanduser()
        package_share = Path(get_package_share_directory(
            'moon_warehouse_dynamic_avoidance'))
        model_name = self.model_filename if filename is None else str(filename)
        return package_share / 'models' / model_name

    def read_checkpoint(self, model_path):
        if not model_path.is_file():
            raise FileNotFoundError(f'Model was not found: {model_path}')
        try:
            checkpoint = torch.load(
                str(model_path), map_location=self.device, weights_only=True)
        except TypeError:
            checkpoint = torch.load(str(model_path), map_location=self.device)
        required = {
            'model_state_dict', 'obs_len', 'hidden_dim',
            'num_layers', 'param_dim',
        }
        missing = required.difference(checkpoint)
        if missing:
            raise ValueError(f'Checkpoint is missing keys: {sorted(missing)}')
        if int(checkpoint['param_dim']) != 7:
            raise ValueError(
                f"Expected seven theta parameters, got {checkpoint['param_dim']}"
            )
        model = LSTMThetaModel(
            input_dim=2,
            hidden_dim=int(checkpoint['hidden_dim']),
            num_layers=int(checkpoint['num_layers']),
            param_dim=7,
        ).to(self.device)
        model.load_state_dict(checkpoint['model_state_dict'], strict=True)
        model.eval()
        return (
            model,
            max(3, int(checkpoint['obs_len'])),
            max(0.02, float(checkpoint.get('dt', self.prediction_dt))),
        )

    def parameter_callback(self, parameters):
        proposed_path = self.model_path_parameter
        proposed_filename = self.model_filename
        reload_requested = False
        for parameter in parameters:
            if parameter.name == 'model_path':
                proposed_path = str(parameter.value)
                reload_requested = True
            elif parameter.name == 'model_filename':
                proposed_filename = str(parameter.value)
                reload_requested = True
        if not reload_requested:
            return SetParametersResult(successful=True)
        try:
            path = self.resolve_model_path(proposed_path, proposed_filename)
            model, observation_count, model_dt = self.read_checkpoint(path)
        except Exception as error:
            return SetParametersResult(successful=False, reason=str(error))
        self.model = model
        self.model_ready = True
        self.model_path_parameter = proposed_path
        self.model_filename = proposed_filename
        self.model_observation_count = observation_count
        self.model_dt = model_dt
        if observation_count != self.history_size:
            self.history_size = observation_count
            self.histories = defaultdict(
                lambda: deque(maxlen=self.history_size)
            )
        self.get_logger().info(f'Reloaded theta LSTM from {path}')
        return SetParametersResult(successful=True)

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
        coordinates = [
            [point['x'], point['y']]
            for point in history
        ]
        input_tensor = torch.tensor(
            [coordinates],
            dtype=torch.float32,
            device=self.device,
        )
        expected = (1, self.model_observation_count, 2)
        if tuple(input_tensor.shape) != expected:
            raise RuntimeError(
                f'Unexpected theta input shape {tuple(input_tensor.shape)}, '
                f'expected {expected}'
            )
        return input_tensor

    def run_lstm_inference(self, obstacle_id, history):
        model_input = self.build_model_input(
            history
        )

        start_time = time.perf_counter()

        with torch.no_grad():
            output = self.model(
                model_input
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

        if not torch.isfinite(output).all():
            raise RuntimeError(
                'Theta output contains NaN or Inf.'
            )

        if tuple(output.shape) != (1, 7):
            raise RuntimeError(
                'Unexpected theta output shape: '
                f'{tuple(output.shape)}'
            )
        values = output[0].detach().cpu().tolist()
        raw_min = float(values[1])
        raw_max = float(values[2])
        theta = {
            'axis': 'x' if float(values[0]) < 0.5 else 'y',
            'minimum': max(-20.0, min(20.0, min(raw_min, raw_max))),
            'maximum': max(-20.0, min(20.0, max(raw_min, raw_max))),
            'fixed': max(-10.0, min(10.0, float(values[3]))),
            'speed': max(
                0.1,
                min(self.maximum_speed, abs(float(values[4]))),
            ),
            's_amplitude': max(0.0, min(2.0, float(values[5]))),
            's_waves': max(0.0, min(3.0, float(values[6]))),
        }
        if theta['maximum'] - theta['minimum'] <= 0.05:
            raise RuntimeError('Theta route span is too small.')
        current_axis = history[-1][theta['axis']]
        tolerance = self.route_attachment_tolerance
        if not (
            theta['minimum'] - tolerance
            <= current_axis
            <= theta['maximum'] + tolerance
        ):
            raise RuntimeError(
                f'Theta bounds do not contain current {theta["axis"]} position.'
            )

        # A checkpoint trained in another map can produce a numerically valid
        # route for the wrong motion axis.  Never feed that domain mismatch to
        # SIPP: use the configured competition route only as a validity prior,
        # then let the existing deterministic fallback produce the prediction.
        route = self.routes.get(obstacle_id)
        if route is not None:
            if theta['axis'] != route['axis']:
                raise RuntimeError(
                    f'Theta axis {theta["axis"]} disagrees with configured '
                    f'route axis {route["axis"]} for {obstacle_id}.'
                )
            cross_track_allowance = (
                tolerance + theta['s_amplitude']
            )
            if abs(
                theta['fixed'] - route['fixed_coordinate']
            ) > cross_track_allowance:
                raise RuntimeError(
                    f'Theta fixed coordinate is detached from the configured '
                    f'route for {obstacle_id}.'
                )
        if self.debug_theta:
            self.get_logger().info(
                f'[theta] id={obstacle_id} axis={theta["axis"]} '
                f'min={theta["minimum"]:.3f} max={theta["maximum"]:.3f} '
                f'fixed={theta["fixed"]:.3f} speed={theta["speed"]:.3f} '
                f'amp={theta["s_amplitude"]:.3f} '
                f'waves={theta["s_waves"]:.3f}'
            )
        debug_message = Float32MultiArray()
        debug_message.data = [
            0.0 if theta['axis'] == 'x' else 1.0,
            theta['minimum'],
            theta['maximum'],
            theta['fixed'],
            theta['speed'],
            theta['s_amplitude'],
            theta['s_waves'],
        ]
        self.theta_publisher.publish(debug_message)
        return theta

    @staticmethod
    def direction_sign(history, axis, minimum, maximum):
        """Estimate direction from several frames instead of one noisy delta."""
        first_index = max(0, len(history) // 2 - 1)
        first = history[first_index][axis]
        last = history[-1][axis]
        displacement = last - first
        if abs(displacement) > 1e-4:
            return 1.0 if displacement > 0.0 else -1.0
        if abs(maximum - last) < abs(last - minimum):
            return -1.0
        return 1.0

    def generate_theta_prediction(self, obstacle_id, history):
        theta = self.run_lstm_inference(obstacle_id, history)
        current = history[-1]
        axis = theta['axis']
        direction = self.direction_sign(
            history,
            axis,
            theta['minimum'],
            theta['maximum'],
        )
        velocity = direction * theta['speed']
        route_length = theta['maximum'] - theta['minimum']
        number_of_steps = max(
            1,
            int(self.prediction_horizon / self.prediction_dt),
        )
        relative_predictions = []
        for step in range(1, number_of_steps + 1):
            future_time = step * self.prediction_dt
            coordinate = self.reflect_position(
                current[axis],
                velocity,
                theta['minimum'],
                theta['maximum'],
                future_time,
            )
            phase = max(
                0.0,
                min(1.0, (coordinate - theta['minimum']) / route_length),
            )
            offset = theta['s_amplitude'] * math.sin(
                math.pi * (2.0 * phase - 1.0) * theta['s_waves']
            )
            if axis == 'x':
                predicted_x = coordinate
                predicted_y = theta['fixed'] + offset
            else:
                predicted_x = theta['fixed'] + offset
                predicted_y = coordinate
            relative_predictions.append([
                predicted_x - current['x'],
                predicted_y - current['y'],
            ])
        return relative_predictions

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
                    self.generate_theta_prediction(
                        obstacle_id,
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
                        f'Theta prediction failed for {obstacle_id}: '
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
        # V3 analytic output is already bounded by learned theta.  The known
        # route constraint remains exclusive to the deterministic fallback.
        route = None
        if prediction_mode != 'lstm':
            route = self.routes.get(obstacle_id)
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
