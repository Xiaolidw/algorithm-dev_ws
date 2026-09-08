# 2026-09-08 evidence-driven regression snapshot

This directory mirrors the tested u22 runtime changes and the corresponding
Windows-side evidence record.

- `competition_v2_scoring.world`: restores the stone aggregate to its original
  dynamic `static=0` setting; no model/link coordinates were changed.
- `SimpleMovePlugin.cc`: prescribed moving-obstacle rail motion with the
  existing endpoints and 0.25 m/s speed.
- `navigation_pick_place_test.py`: 0.70 m pickup handoff, live global-costmap
  goal probe, A/B transit routes, inboard B slots, and the red-cube-4 west rail
  bypass.
- `navigation.launch.py`, `initial_pose_publisher.py`: deterministic initial
  pose burst and one RViz launch.
- `ros_runtime_cleanup.sh`, `fastdds_udp_only.xml`, and helper scripts: bounded
  startup/readiness checks and repeatable Fast DDS cleanup/transport.
- `9.1.md`: test logs, timings, root causes, accepted fixes, and remaining work.

The Gazebo map geometry, wall/stone poses, zone coordinates, cube initial
coordinates, and moving-obstacle endpoints/speeds are not retuned in this
snapshot.
