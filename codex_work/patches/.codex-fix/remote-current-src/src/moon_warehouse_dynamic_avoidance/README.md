# Dynamic avoidance interface contract

This package does not modify the project's established navigation interfaces.
`dynamic_obstacle_predictor_node` subscribes to the existing Gazebo-plugin
topics `/moving_obstacle_1/current_pose` and
`/moving_obstacle_2/current_pose`, then publishes
`/moon_warehouse/dynamic_obstacle_trajectories`. Nav2 MPPI consumes that
prediction through `LSTMDynamicObstacleCritic`.

The original velocity chain remains unchanged:
`controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel -> base`.

The current predictor uses configured route reflection because the supplied
LSTM was trained against different obstacle routes. A calibrated model can
later replace only the predictor core.
