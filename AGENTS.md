# Repository rules

- User-facing communication and node documentation are Korean. Runtime nodes are C++17, ROS 2 Jazzy.
- This is an independent repository initialized from the parent source snapshot with the user's
  authorization. Before edits inspect git status; fetch/update only when a remote exists. Never
  overwrite uncommitted work. The initial repository has no remote.
- Read the closest package AGENTS.md. Keep changes focused. Prefer existing f110_msgs and standard
  ROS messages. Every configurable node has installed YAML, launch.py, and Korean documentation.
- This stack uses manual initial pose, wheel-odometry-aided ICP, observed local free space, and
  rectangular-body OBCA. It has no global raceline, Frenet state machine, or loop closure.
- Never treat unknown space as free, truncate obstacle constraints silently, or publish a failed
  optimization as a valid path. Stop on stale inputs, localization failure, or invalid paths.
- Check ~/.zshrc aliases before Linux build/run work. Build and run appropriate tests, and clearly
  distinguish standalone core checks from ROS integration and closed-loop driving validation.
- Keep upstream copyright/license notices. OBCA is independently implemented from the paper;
  no Julia source is copied. Preserve provenance in THIRD_PARTY.md.
