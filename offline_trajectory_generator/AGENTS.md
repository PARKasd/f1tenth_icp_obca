# AGENTS.md

This directory contains a standalone, offline trajectory generation tool.

Port context: this independent ICP/OBCA repository consumes generated CSV directly in
obca_navigation. References below to the parent's global_trajectory_publisher_node describe
output compatibility; that node is not included or required in this repository.

Rules:
- Do not add ROS 2 runtime dependencies here.
- Keep the CLI runnable with plain `python3` after the workspace Python dependencies are available.
- Inputs are ROS map-server compatible SLAM map YAML/image files.
- Outputs should stay compatible with the existing `f110_msgs/Wpnt` field names when practical.
- Put user-facing operation docs in Korean.
- Prefer configurable CLI arguments over hard-coded map, speed, width, or smoothing values.
- When adding or changing generator parameters, update the GUI defaults, saved parameter YAML, and Korean README in the same change.

## Layout

- `generate_global_trajectory.py` — map -> centerline -> raceline -> speed profile pipeline (CLI).
  `write_outputs()` also bakes the RViz MarkerArrays (`trajectory_markers` /
  `trackbound_markers` / `centerline_markers`) into global_waypoints.json.
  `global_trajectory_publisher_node` publishes them verbatim and builds nothing at runtime,
  so marker geometry/style changes must be made here and the map regenerated. Styles are
  module constants (`MARKER_FRAME_ID`, `TRAJ_MARKER_WIDTH`, `TRACKBOUND_MARKER_WIDTH`) —
  do not turn them into CLI flags, that would create a second source of truth next to the
  node YAML.
  Optimizers: `centerline` (keep the skeleton line), `mincurv` (scipy L-BFGS-B minimum
  curvature), and `d_ratio` (fixed-ratio lateral offset; see
  `docs/proposal_d_ratio_raceline.md`). The standard speed profile reads `config/velocity_limits.csv` with columns
  `[speed_mps, max_accel_mps2, max_decel_mps2, max_lateral_accel_mps2]`; keep the CLI, GUI,
  and saved YAML on the same table semantics.
  Extraction robustness invariants (keep when refactoring): centerline candidates are ranked by
  ENCLOSED contour area (not arc length — noise scribbles are long but enclose nothing); skeleton
  pixels in corridors narrower than `min_track_width` are dropped; the Zhang-Suen fallback keeps
  the skeleton loop connected without opencv-contrib; map cleanup must NEVER erase measured walls
  (`cleanup_free_mask(occupied_mask=...)` stamps raw occupied pixels back after morphology so a
  big morph kernel cannot swallow a thin interior wall); `count_off_map_waypoints` checks densely
  (between-waypoint segments too) and must keep warning when the raceline leaves free space.
  Steering-limit (`--max-curvature`) invariants: the speed model alone never rejects an
  undrivable kink (v_min floors it), so the kappa-excess penalty must stay in the mincurv
  objective, and it must stay MEAN-based (a stiff sum-based term destabilizes L-BFGS-B's
  numerical gradients — measured); `--raceline-smooth-sigma` (final-resample smoothing) removes
  the phantom per-vertex curvature spikes of the piecewise-linear optimizer output;
  `limit_curvature_spikes` may only touch ISOLATED runs (<= 3 wpts) — blending a sustained
  corner just moves the kink to the run boundary and amplifies it (6 -> 17 rad/m measured);
  the final validation warning (kappa_violations/max_abs_kappa) must keep firing when the
  delivered raceline exceeds the limit.
  `d_ratio` invariants: positive ratio moves toward `d_right` (the OPPOSITE sign of the
  mincurv alpha / Frenet d convention — user spec, do not "fix"); the ratio scales the usable
  half-width `max(d_side - clearance, 0)`, never the raw wall distance; optional smoothing
  applies to the alpha array and MUST be followed by a clip back to the RAW per-point corridor
  (smoothing the width arrays instead inflates narrow sections and overshoots the wall);
  `offset_by_d_ratio` must stay per-point-array compatible (np.where, no scalar-only branches)
  for future ratio scheduling on adaptive_global; the fold guard (cap |alpha| below the local
  curvature radius when moving toward the curvature center) must stay LAST and must only ever
  shrink |alpha| — without it a hairpin inside-offset folds into a self-loop (observed at
  d_ratio=-0.5 on map.yaml; widening boundary_margin does NOT help, it is a curvature problem). `report_min_clearance` (dense
  distance-transform audit of the final raceline) must keep running for every optimizer —
  off-map counting alone cannot see car-half-width wall clipping.
- `trajectory_gui.py` — tkinter GUI over the same args (`NUMERIC_SPECS` must cover new params).
  `write_outputs` JSON-serializes `vars(args)` into metadata.json; never inject non-JSON-safe
  values (callables etc.) into the args namespace — that broke the GUI Save button once.
  The saved `gui_params.yaml` may hold values from removed features (e.g. an old optimizer
  name); loading must sanitize such values instead of crashing.
- `config/velocity_limits.csv` — speed-dependent net acceleration limits for the GUI/CLI.
