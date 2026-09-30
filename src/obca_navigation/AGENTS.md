# obca_navigation

- C++17 runtime: planner_node (local mapping/reference/OBCA) and tracker_node (path following).
- Core in include/obca_navigation and src/core.cpp, src/solver.cpp is ROS-independent and tested
  in test/core_test.cpp. Ipopt solves actual dual separating-distance constraints for rectangles.
- Inputs: LaserScan, Odometry, PoseWithCovarianceStamped, diagnostic_msgs/DiagnosticArray.
  Outputs: f110_msgs/WpntArray, nav_msgs/Path, nav_msgs/OccupancyGrid, std_msgs/String,
  ackermann_msgs/AckermannDriveStamped. No custom messages.
- All operating parameters are in config/navigation.yaml, loaded by launch/navigation.launch.py.
  navigation_real.launch.py and navigation_sim.launch.py include this common launch and load
  config/real.yaml and config/sim.yaml respectively. Explicit CLI overrides win over YAML profiles.
  Hardware commands go to /drive_autonomous (external mux), gym commands to /drive. Common launch
  alone retains /obca/drive. These launches do not start drivers, mux, or the external simulator.
  CMake installs executables, config, launch, and docs. Korean instructions: docs/obca_navigation.md.
- Unknown/expired cells block motion. Validate swept vehicle footprint and the NLP residuals
  independently before publishing. Obstacle-budget overflow means stop, never dropping obstacles.
- Manual initial pose resets map, prior path, and tracker. Reject data predating the reset.
- The tracker is a separate process with its own wall timer and data watchdog; a blocked solver
  cannot keep stale drive commands alive. The common launch emits commands on /obca/drive;
  environment-specific launches use the hardware mux or simulator topic described above.
- test/test_launch_profiles.py checks launch wiring and parameter precedence with lightweight
  launch API doubles. Passing it does not establish ROS runtime compatibility.

- Standard gym uses wall time (use_sim_time=false). Enable simulated time only when the
  bridge publishes /clock and stamps sensors/TF in that same clock domain.
- Planner stop statuses distinguish clock mismatch, stale pose/diagnostics, and ICP quality
  with measured values. Preserve both arrival-time watchdogs and diagnostic quality checks.
- If circular reference clearance blocks bootstrap at a LiDAR blind spot, a forward connector
  may seed the search only after swept-footprint validation. Never mark unknown cells free.
- After initial pose, startup_straight_distance in navigation.yaml selects a zero-steering
  startup phase. Validate its entire reference and optimized swept body against observed space.
- Knot/obstacle constraints may be omitted only using conservative reachable-body bounds
  derived from the enforced dynamics, speed, acceleration and terminal-stop constraints.
  Account for validation tolerance. Keep full-grid final validation and report pair counts.
- Keep the sparse objective/Jacobian/Lagrangian Hessian consistent; verify analytical
  derivatives against finite differences whenever solver equations change.
