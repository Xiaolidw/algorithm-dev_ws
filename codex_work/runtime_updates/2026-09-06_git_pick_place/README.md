# 2026-09-07 final differential-IK pick/place runtime

These are the exact u22 source/configuration files used by the successful
five-red cumulative and ten-object production regressions on 2026-09-07.
No Gazebo world or map file is included.

- `fixed_manipulation_server.py`: dynamic grasp plus symmetric two-stage place
- `fixed_manipulation.yaml`: matching arm poses, timing, and validation limits
- `navigation_pick_place_test.py`: targeted nearest-cube acceptance runner

Evidence logs on u22:

- `/home/ros/dev_ws/logs/git_pick_place_red_to_a_validation_3.log`
- `/home/ros/dev_ws/logs/git_pick_place_blue_to_b_validation_3.log`
- `/home/ros/dev_ws/logs/git_pick_place_manipulation_3.log`
- `/home/ros/dev_ws/logs/git_pick_place_manipulation_4.log`
- `/home/ros/dev_ws/logs/final_cardinal_red_summary_20260907.log`
- `/home/ros/dev_ws/logs/final_ten_v5_summary_20260907.log`
- `/home/ros/dev_ws/logs/final_ten_v5_item_1_20260907.log` through `item_10`
- `/home/ros/dev_ws/logs/final_ten_v5_stack_20260907.log`

Final results:

- five-red cumulative: 5/5, 322.258 seconds, 0 Nav2 recoveries
- ten-object production chain: 10/10, 613.570 seconds, 0 Nav2 recoveries
- final item remains stopped in place after placement

World hash (source and install):
`d8078a1a2e9eb1ad371f3a11626cb1b74c56eb7faee4bd19a38f7b4de37e5ae2`.
