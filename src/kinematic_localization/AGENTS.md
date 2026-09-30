# Package-level AGENTS.md for kinematic_localization

This document defines package-specific rules for `kinematic_localization`. This repository's
root AGENTS.md applies in full; entries below only add package detail.

## Independent mapless stack

- `require_initial_pose` gates SLAM auto-initialization; navigation.launch.py enables it.
- On a manual reset clear accumulated SLAM points as well as the ICP local map and timing state.
- Preserve the original scan-end synchronization, deskew, and odometry prior.
- The navigation launch disables pose smoothing so local mapping and raw ICP share a frame.
- No frozen map is loaded in the mapless launch. Existing frozen-map tools remain optional.
- Document changes in docs/mapless_initialization.md (Korean).

## Launch and parameter loading

- Both launch files must resolve their YAML to an absolute installed-share path and fail before
  node creation when the file is absent or its symlink is broken. Do not pass an unchecked
  `PathJoinSubstitution` as the parameter-file entry: `launch_ros` can silently omit a missing
  file and start the node with C++ defaults.
- `config_schema_version` is mandatory in both YAML files. Both C++ nodes must compare it with
  their compiled expectation and fail fast on missing, stale, or wrong YAML. Keep the two C++
  constants and both YAML values equal; `test/test_config_guard.py` locks this contract.
- Keep the startup parameter-summary logs. They are the rosbag-visible evidence of the effective
  gate, smoothing, source-downsample, voxel, and mapping settings used in a field session.

## Package Purpose

Kinematic-ICP based map localization. Scans are optionally chassis-tilt compensated, deskewed,
and aligned (ICP with a wheel-odometry prior) against a **frozen** voxel map (`.kissmap`), so the
estimate cannot diverge the way pure odometry does (e.g. in reverse driving). Interface is drop-in compatible with
`particle_filter_cpp` (MCL): it publishes `/pf/pose/odom` and the `map -> odom` TF.

- `localization_node` (C++): online localization, waits for `/initialpose` like MCL.
  With `slam_mode: true` the same node runs as online SLAM (freeze_local_map = false,
  no frozen map, auto-init at identity on the first scan, accumulated map saved via the
  `~/save_map` Trigger service or on shutdown, published on `~/map_points`).
  Output smoothing (complementary filter: wheel-odom prediction + ICP correction with a
  velocity-adaptive alpha, `smoothing_*` params) is applied to the published pose and
  `map -> odom` TF; SLAM map accumulation always uses the raw ICP pose. Rotation is
  pulled with its own fixed gain (`smoothing_alpha_rot`, YAML 0.12; < 0 = follow the
  translation alpha): the ICP yaw measurement is ~10x noisier than the wheel-odom yaw
  prediction while cornering (docs §8), so a uniform alpha turns corners into yaw jitter. The node also
  serves `/map` (`nav_msgs/OccupancyGrid`, transient_local) rasterized from the map
  points, so RViz 2D Pose Estimate works without a separate map_server. A non-empty
  `map_name` that fails to load is FATAL (fail fast — no silent pure-odometry); the
  cached grid is re-published every `map_publish_period_sec` so late/restarted
  subscribers always converge.
  Robustness features (kicp_robustness_plan, see `docs/kinematic_localization.md` §7):
  always-on NaN guard (core + node rollback), scan-gap watchdog (odom dead reckoning,
  `watchdog_*`), registration diagnostics on `~/diagnostics` (`diagnostics_*`),
  map-based pose validity check (`pose_check_*`), Mahalanobis gate (`gate_*`,
  default OFF until bag-measured), soft lateral DoF (`lateral_*`, default OFF).
  Watchdog/pose-check/gate/lateral are forced off in `slam_mode`. The node runs on a
  single-threaded executor by design — the shared state is lock-free; do not move
  callbacks into other callback groups or a MultiThreadedExecutor.
- `mapping_node` (C++): offline `.kissmap` builder from a rosbag (message-count based; bag
  send/receive timestamps are never used — sqlite3 bags report `send_timestamp == 0`).
- `scripts/pgm_to_kissmap.py`: converts a map_server occupancy map (pgm/png + yaml) to
  `.kissmap`. Point density is set by `--downsample` (2D grid step, default 0.1 m);
  `--voxel-size` is header metadata only (a mismatch with the node's `voxel_size`
  just WARNs at load). Keep `--downsample` <= ~0.14 m so a wall line stays under
  `max_points_per_voxel` (20) per 1 m voxel.
- `scripts/wall_rate.py`: wall-alignment metric used to validate localization quality.
- `scripts/compare_trajectory.py`: TUM trajectory comparison helper (from the KICP evaluation).
- `scripts/param_sweep.py`: replays an mcap bag against the node with several
  `smoothing_alpha_rot` values and reports the lag/jitter trade-off. Note: `residual_rms`
  is blind to this parameter — the smoothing filter is output-only and never feeds back
  into the ICP core (`last_pose_ = new_pose`), so the script measures published yaw lag
  against an `alpha_rot = 1.0` reference run instead. ROS 2 Humble has no mcap storage
  plugin, so the script replays the bag itself rather than calling `ros2 bag play`.

## Vendored Core

The ICP core lives in `third_party/kinematic-icp` (MIT, PRBonn kinematic-icp + kiss-icp).
kiss-icp v1.2.0, Sophus 1.22.10 (with `sophus.patch` pre-applied), and tsl-robin-map v1.2.1
are vendored in `third_party/kiss-icp|sophus|robin-map` — `kiss-icp.cmake` points
`FETCHCONTENT_SOURCE_DIR_*` at them, so builds need **no internet** (Jetson-safe). The
vendored dirs carry `COLCON_IGNORE` so colcon does not treat them as packages. If a vendored
dir is deleted, the build falls back to downloading the upstream tarballs. Local patches are
marked with `Localization patch (2026_IFAC)` comments:

- `Config::freeze_local_map` (KinematicICP.hpp/.cpp) — frozen-map localization mode.
- `USE_SYSTEM_SOPHUS/OFF`, `USE_SYSTEM_TSL-ROBIN-MAP/OFF` defaults (CMake).
- **NaN guards** (Registration.cpp `ComputeRobotMotion`): `correspondences.empty()` →
  return the odom prediction before the regularization lambda, and `break` after the
  in-loop re-association. Without them a 0-correspondence frame produces 0/0 = NaN and
  `Sophus::SE3d::exp(NaN)` **aborts the process** (SOPHUS_ENSURE) — this was the
  "/initialpose kills the node" bug. Never remove; they are deliberately toggle-free.
- **Registration diagnostics** (Registration.hpp `Diagnostics`, `diagnostics()`,
  `KinematicICP::registrationDiagnostics()`): per-frame inlier/residual/JTJ snapshot,
  reset at the top of every `ComputeRobotMotion` call. `residual_rms`/`JTJ` are one
  iteration stale by design (recorded at final-iteration entry) — consumers must widen
  trust via `converged`/`final_dx_norm` instead of re-associating.
- **Soft lateral DoF** (Registration.cpp `lateral_dof_enable_` branch, Config
  `lateral_dof_enable|lateral_regularization_scale|lateral_regularization_floor_tau2`):
  3-DoF perturbation with `beta_lat = max(floor/tau², scale·beta)`. The 2-DoF arc path
  is preserved verbatim and must stay bit-identical when the toggle is off.
- **Source-only downsample** (KinematicICP.hpp `Config::source_voxel_size`,
  KinematicICP.cpp `RegisterFrame` Voxelize call): > 0 replaces `voxel_size` in the
  source 0.5x/1.5x cascade only; map hash grid, correspondence search reach and
  adaptive-threshold floor keep using `voxel_size` so the convergence basin is
  unchanged. <= 0 (default) = upstream behavior.

Do not upgrade the vendored core without re-applying these patches. Sophus and
tsl-robin-map are **not** available system-wide (no sudo/apt) — never flip those options ON.

## §9 Scan-end synchronisation (2026-08-22)

- The registration prior and the published pose/TF stamp must both sit at the
  **deskew reference**, which is the end of the LiDAR sweep. Before §9 they
  disagreed by one sweep: the pose went out at KISS's `end_stamp`
  (`header + sweep`) while the odometry prior ended at the raw header.
- **Which one is right was settled by a registration A/B, not by reasoning.**
  Replay of `run_20260822_035120`, localizer seeded from the bag's own first
  pose, everything else identical:

  | arm | prior/publish reference | yaw corr p95 | >1° | >2° | residual p95 | wait |
  |---|---|---|---|---|---|---|
  | A legacy | prior@header, publish@header+sweep | 0.77° | 182 | 29 | 0.567 | 0 ms |
  | B `end` | both @header | 0.69° | 165 | 28 | 0.567 | 15 ms |
  | **C `begin`** | both @header+sweep | **0.40°** | **80** | **12** | **0.417** | 35 ms |

  Clean lap (`run_20260822_034812`) moves the same way: yaw corr p95 0.19° → 0.10°.
  So `scan_stamp_convention: "begin"` — the header is the first ray's time,
  the plain ROS `LaserScan` convention.
- ⚠️ **Never settle this from the arrival lag.** On this car `receive - header`
  is −2.3 ms for `/scan` (IQR 0.1 ms) against +1.1 ms for `/odom`, which looks
  impossible for a begin-stamped scan. It only looks that way because the
  urg_node stamp is not on the host clock. The node logs the lag heuristic and
  explicitly does not act on it (`LagSuggestedConvention`).
- `OdomAtStrict` never clamps and never extrapolates; `OdomAt` still clamps and
  is watchdog-only. Mixing them up is what made the 2026-08-21 attempt a 2×
  regression — a truncated prior is worse than an offset one.
- A scan whose end stamp is not bracketed by odometry **waits, then is dropped**
  (`odom_end_sync_max_wait_sec`, `odom_end_sync_max_queue`). Never register
  against a guessed prior: a hole in the output is recoverable, a silently
  wrong pose is not. The 034812 sensor blackout exercises this (5 drops).
- `map->odom` TF, `/pf/pose/odom` pose and its twist are all emitted at the
  same `scan_end`; `T_odom_base` is interpolated rather than taken from
  `odom_history_.back()`, which is newer once the dispatcher waits.
- Cost: the prior points one sweep into the future, so the dispatcher waits
  ~35 ms. The legacy path already carried a structural one-scan lag (~25 ms),
  so the real increase is about +10 ms — **measured +8.7 ms on the car**.
- ✅ **Real-car A/B/A, 2026-08-22** — same session, no obstacles, steering
  authority 7.93 on all three, nothing but §9 changed:

  | arm | latency | lat_err corner p95 | attitude yaw (ω>0.8) | lap times |
  |---|---|---|---|---|
  | A1 `065117` | 6.2 ms | 0.307 | 0.83° | 8.57 / 8.51 / 8.47 |
  | **B** `065236` | **15.6 ms** | **0.216** | **0.43°** | 8.58 / 8.64 / 8.54 |
  | A2 `065444` | 7.5 ms | 0.202 | 0.87° | 8.60 / 8.61 / 8.56 |

  The attitude correction returns on BOTH sides (0.83 → 0.43 → 0.87), so it is
  the change and not drift: −49 %, matching the −48 % the replay predicted.
  The latency cost does not surface as tracking error — 8.7 ms × 4.6 m/s is
  4 cm, but the corner p95 differs from A2 by only 1.4 cm, and lap time by
  +0.04 s (measurement noise). `odom_window_at_scan_end` is therefore **true**
  by default since 2026-08-22; rolling back is that one line.
  ⚠️ Three laps per arm only resolves the attitude metric (thousands of
  frames). Lap time and tracking error need ~10 laps per arm to separate.

## Chassis-roll compensation (prototype port, 2026-08-22)

- Only the transition_global tilt component is ported. Keep prototype's scan-end FIFO,
  `OdomAtStrict`, timestamp convention, deskew path, auto-initialization, free-mode, and KICP core
  unchanged.
- Runtime roll is `roll_gradient_rad_per_mps2 * (v * yaw_rate)`, computed from the same strictly
  bracketed scan-end-to-scan-end wheel-odometry delta as the ICP prior. Do not add a second IMU or
  timestamp path.
- `utils::LevelScan` applies the base-frame roll through the LiDAR extrinsic, projects the corrected
  source back to z=0, and falls back to the original source if point rejection leaves fewer than 10
  points.
- `tilt_compensation_enable`, `roll_gradient_rad_per_mps2`, `pitch_gradient_rad_per_mps2`,
  `tilt_max_angle_rad`, and `tilt_max_point_height_m` must stay synchronized between localization
  and mapping YAML when producing a frozen map from a fast-driving bag. Low-speed occupancy/SLAM
  maps do not require regeneration. Pitch and point rejection remain 0.0 until measured.
- Every diagnostics message must include `tilt_roll_deg`, `tilt_pitch_deg`, and
  `tilt_dropped_points`; watchdog-only samples report zeros.

## .kissmap Format (defined by this package)

Binary: magic `KISSMAP1` (8 bytes), `double voxel_size`, `double max_range`, `uint64 count`,
then `double x,y,z` per point (map frame). Maps live in `maps/<map_name>.kissmap` and are
selected with the `map_name` parameter (an absolute path is used as-is).

## Message / Topic Policy

- Subscribed: `/scan` (`sensor_msgs/LaserScan`, sensor-data QoS), `/tf`, `/tf_static`,
  `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`, map frame — same UX as MCL).
- Published: `/pf/pose/odom` (`nav_msgs/Odometry`, reliable depth 10, frame `map`, child
  `base_link`), TF `map -> odom` computed as `T_map_base * T_odom_base^-1` (same convention
  as MCL in `particle_filter.cpp::publish_tf`), `/map` (`nav_msgs/OccupancyGrid`,
  transient_local depth 1, rasterized from the map points).
- SLAM mode only: publishes `~/map_points` (`sensor_msgs/PointCloud2`, transient_local) and
  serves `~/save_map` (`std_srvs/Trigger`, saves the accumulated map to `map_output_file`;
  the map is also auto-saved on clean shutdown).
- Diagnostics: `~/diagnostics` (`diagnostic_msgs/DiagnosticArray`, `diagnostics_enable`) —
  registration quality (inlier_ratio, residual_rms, tau, beta, convergence, gate state,
  dead_reckoning_sec) plus the applied tilt values. Standard type on purpose (root policy: no
  custom msg needed).
- No new custom message types; standard `sensor_msgs`/`nav_msgs`/`geometry_msgs`/
  `diagnostic_msgs` only.

## Parameter Policy

- `config/kinematic_localization.yaml`: all topics, frames, KICP tuning, covariances,
  mandatory `config_schema_version`,
  the output-smoothing block (`smoothing_enable`, `smoothing_alpha`,
  `smoothing_alpha_gain`, `smoothing_velocity_full_mps`, `smoothing_alpha_max` —
  starting values for real-car tuning), the built-in `/map` server block (`map_topic`,
  `map_grid_resolution`, `map_point_dilation_m`), the SLAM-mode block (`slam_mode`,
  `map_output_file`, `map_publish_period_sec`), and the robustness blocks
  (`watchdog_*`, `diagnostics_*`, `pose_check_*`/`permissible_radius_m`, `gate_*`,
  `lateral_*`) and chassis-tilt compensation (`tilt_*`, `roll_gradient_*`, `pitch_gradient_*`).
  Defaults policy: watchdog/diagnostics/pose-check ON (safe direction
  only). Gate and lateral DoF are ON since 2026-08-18 (docs §9): the cornering
  slip-hiding symptom was field-measured, §3 nominal distributions were measured
  (residual_rms p99 0.21, inlier p1 1.00), and the gate at chi2 11.34 rejected
  0/8401 nominal frames. Rollback: lateral_dof_enable:=false gate_enable:=false
  (then gate_chi2 9.21 for 2-DoF).
  Verified tuning: `voxel_size 1.0` — do NOT shrink it: it also sets the
  correspondence search reach and the adaptive-threshold floor, and 0.25
  collapsed the convergence basin (run_0818 replay diverged meters, inlier
  ratio 0.26). The indoor-scan source collapse (9-25 points at voxel 1.0) is
  fixed by the source-only `source_voxel_size 0.25` core patch instead
  (docs §8). `max_num_iterations 30`, `max_range 30.0`, `min_range 0.1`,
  `deskew true`, adaptive threshold/regularization on.
- `config/mapping.yaml`: mandatory `config_schema_version`, the same KICP and tilt blocks, and
  bag/output paths for `mapping_node`.
- Launch files must not hard-code tuning values; they only select the YAML and pass
  runtime selections (`map_name`, `bag_path`, `output_path`, `use_sim_time`).

## Launch / Run

- Localization: `ros2 launch kinematic_localization kinematic_localization.launch.py map_name:=<name>`
  then publish `/initialpose` once (RViz 2D Pose Estimate or `ros2 topic pub --once`).
- Mapping: `ros2 launch kinematic_localization mapping.launch.py bag_path:=<bag> output_path:=<file>`

## Documentation

Korean node documentation: `docs/kinematic_localization.md`. Keep it in sync when topics,
parameters, or the run procedure change.
