# map2 scattered-cargo optimization

This branch replaces the original two-row cargo layout with ten user-specified
poses and removes object-number-specific route assumptions from the production
runner.

## Authoritative coordinates

| Object | x | y |
|---|---:|---:|
| red_cube_1 | 1.0 | 4.0 |
| red_cube_2 | 3.5 | 0.5 |
| red_cube_3 | -1.7 | 1.6 |
| red_cube_4 | -6.0 | 0.0 |
| red_cube_5 | 1.0 | -9.0 |
| blue_cube_1 | -2.0 | 4.0 |
| blue_cube_2 | 0.5 | 1.6 |
| blue_cube_3 | -8.0 | 0.0 |
| blue_cube_4 | -6.0 | -7.0 |
| blue_cube_5 | -1.0 | -9.0 |

The corrected layout contains no duplicate initial cube coordinates. Runtime
scheduling still reads Gazebo's settled poses before each decision.

## Changed production artifacts

- `../2026-09-22_runtime_optimization/competition_v2_scoring.world`: both
  saved-state and model-spawn poses use the map2 coordinates.
- `../2026-09-13_c_zone_regression/competition_v2_semantics.yaml`: semantic
  object poses match the world.
- `../2026-09-13_c_zone_regression/task_execution.yaml`: fallback approach
  poses face each new object location.
- `../2026-09-06_git_pick_place/cube_obstacle_map_node.py`: startup virtual
  obstacle defaults match map2, then yield to live ModelStates.
- `../2026-09-22_runtime_optimization/navigation_pick_place_test.py`: upper
  rail routing is selected from live geometry; after every placement the next
  colour-matching pickup is preplanned, and the nearest two candidates are
  compared by pickup plus carried-route cost.

Collision Monitor, dynamic crossing gates, 3-D inventory validation, placement
clearance checks, and the proven B-before-A/C-last batch order remain enabled.

Timing evidence and accepted/rejected follow-up changes are recorded outside
the repository in `C:\Users\奶龙\Desktop\103.md` as requested.
