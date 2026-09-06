# 2026-09-06 Git-referenced pick/place runtime

These are the exact u22 source/configuration files used by the successful
targeted regressions on 2026-09-06. No Gazebo world or map file is included.

- `fixed_manipulation_server.py`: dynamic grasp plus symmetric two-stage place
- `fixed_manipulation.yaml`: matching arm poses, timing, and validation limits
- `navigation_pick_place_test.py`: targeted nearest-cube acceptance runner

Evidence logs on u22:

- `/home/ros/dev_ws/logs/git_pick_place_red_to_a_validation_3.log`
- `/home/ros/dev_ws/logs/git_pick_place_blue_to_b_validation_3.log`
- `/home/ros/dev_ws/logs/git_pick_place_manipulation_3.log`
- `/home/ros/dev_ws/logs/git_pick_place_manipulation_4.log`

World hash (source and install):
`d8078a1a2e9eb1ad371f3a11626cb1b74c56eb7faee4bd19a38f7b4de37e5ae2`.
