# obca_navigation

- C++17 runtime: planner_node (local mapping/reference/OBCA) and tracker_node (path following).
- Core in include/obca_navigation and src/core.cpp, src/solver.cpp is ROS-independent and tested
  in test/core_test.cpp. Ipopt solves actual dual separating-distance constraints for rectangles.
- Inputs: LaserScan, Odometry, PoseWithCovarianceStamped, diagnostic_msgs/DiagnosticArray.
  Outputs: f110_msgs/WpntArray, nav_msgs/Path, nav_msgs/OccupancyGrid, std_msgs/String,
  ackermann_msgs/AckermannDriveStamped. No custom messages.
- All operating parameters are in config/navigation.yaml, loaded by launch/navigation.launch.py.
  CMake installs executables, config, launch, and docs. Korean instructions: docs/obca_navigation.md.
- Unknown/expired cells block motion. Validate swept vehicle footprint and the NLP residuals
  independently before publishing. Obstacle-budget overflow means stop, never dropping obstacles.
- Manual initial pose resets map, prior path, and tracker. Reject data predating the reset.
- The tracker is a separate process with its own wall timer and data watchdog; a blocked solver
  cannot keep stale drive commands alive. Default launch emits commands on /obca/drive only.
