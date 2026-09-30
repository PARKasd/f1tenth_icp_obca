# kinematic_localization 노드 문서

이 독립 저장소의 지도 없는 주행은 [지도 없는 초기화](mapless_initialization.md)를 먼저
참고하십시오. `obca_navigation` launch는 `require_initial_pose=true`로 수동 초기화를
기다리고 frozen map 없이 실행합니다. 아래 내용은 복사한 기존 노드의 일반 기능 설명입니다.

## 1. 목적

Kinematic-ICP(kiss-icp 파생) 기반의 **동결 맵(frozen map) localization** 노드다.
기존 MCL(`particle_filter_cpp`)과 동일한 출력 인터페이스(`/pf/pose/odom` + `map -> odom` TF)를
제공하므로, 스택의 다른 노드(글로벌 플래너, 상태 머신, 제어기)를 수정하지 않고 MCL을 대체할 수 있다.

순수 KICP odometry는 스캔으로 로컬 맵을 계속 갱신하는 SLAM 방식이라 후진 주행처럼 정합이
묄때 그대로 발산하고 복구되지 않는다. 이 패키지는 **맵을 미리 만들어 동결**해 두고
(코어 패치 `Config::freeze_local_map = true`), 스캔은 맵을 절대 수정하지 않고 정합만
수행하므로 정합 실패 후에도 다음 스캔에서 다시 맵에 맞춰 복구된다.

## 2. 동작 원리

1. **초기화**: `/initialpose`(map 프레임)를 받으면 `SetPose(T_map_base)`로 포즈를 리셋한다.
   코어의 `SetPose`는 로컬 맵을 비우므로, 직후에 동결 맵 점들을 `VoxelMap().AddPoints()`로
   재주입한다. 초기 포즈 수신 전에는 스캔을 버리고 대기한다.
2. **매 스캔**:
   - `laser_geometry::LaserProjection::projectLaser(..., channel_option::Timestamp)`로
     포인트별 타임스탬프가 포함된 PointCloud2를 만든다.
   - `/odom` 토픽 히스토리를 스캔 창 양끝 시각으로 **보간**해 휠 odometry 델타
     (모션 프라이어)를 구한다(TF가 아닌 토픽 기준 — bag/실차의 odom TF에는
     수백 ms 공백이 있어 보간 프라이어가 깨지기 쉽다).
   - 롤 보상이 켜져 있으면 같은 odometry 델타에서 `a_lat = v·yaw_rate`를 구하고
     `roll = roll_gradient·a_lat`로 스캔면을 수평화한다. 별도 IMU나 별도 timestamp
     경로는 사용하지 않는다.
   - `RegisterFrame(points, timestamps, extrinsic, delta)` 호출 — 낶적으로
     deskew → voxelize → ICP(휠 odom 프라이어 + adaptive regularization)를 수행한다.
     정차 중에도 항상 등록한다(동결 맵은 오염되지 않으므로 정차 중 ICP가 포즈를
     맵 위로 당겨 복구할 수 있다).
3. **출력**: 추정 포즈 `T_map_base`를 `/pf/pose/odom`으로 발행하고,
   `T_map_odom = T_map_base * T_odom_base^-1`을 `map -> odom` TF로 발행한다
   (MCL의 `publish_tf`와 동일한 의미).

### 출력 스묨 (상보필터)

ICP 생(raw) 포즈는 실차 기준 프레임당 위치 스텝 p50 ≈ 8–10 mm, yaw 스텝
p50 ≈ 0.22–0.27°, 5 cm 이상 점프가 프레임의 19–26%로 휠 odometry 대비 10배
이상 거칠다(측정: 38.7 Hz, 점프의 95%가 2 m/s 초과 구간). 그래서 MCL의
스묨과 같은 아이디어의 **상보필터**를 출력에 적용한다 (`smoothing_enable`,
기본 on):

```
T_pred    = T_last_out * delta_odom         # 휠 odom 델타로 예측
err       = T_pred^-1 * T_icp               # ICP 보정량
alpha     = clamp(base + gain * min(v / v_full, 1), 0, alpha_max)   # 병진 gain
alpha_rot = smoothing_alpha_rot (>= 0 이면 고정값, < 0 이면 alpha 따름)
T_out     = T_pred * exp(diag(alpha, alpha_rot) * log(err))         # 성분별 적용
```

- `v`는 해당 스캔 창의 odom 델타/Δt로 추정한 속도. 정지·저속에서는 ICP 지터가
  순수 노이즈이므로 작은 alpha로 강하게 스묨하고, 고속에서는 실제 포즈가 빠르게
  변하므로 alpha를 키워 필터가 ICP 추정에 뒤처지지 않게 한다(지연 누적 방지).
- **회전(yaw)은 별도 고정 gain(`smoothing_alpha_rot`)으로 당긴다.** 실차 코너링
  bag(run_0818, ~4 m/s) 실측에서 ICP yaw 측정 노이즈는 0.7~2.7°/프레임인 반면
  휠 odom yaw 예측 오차는 0.06°/프레임으로 10배 이상 깨끗했다. 속도 부스트가
  붙은 병진 alpha(≈0.5)로 yaw까지 당기면 코너에서 요레이트 지터(±14°/s 이상)가
  그대로 출력으로 새므로, yaw는 속도와 무관하게 약하게(기본 0.12, 시상수 ≈0.2 s
  @40 Hz) 당긴다. odom yaw 바이어스가 0.1°/s 수준이라 추종 지연은 무시 가능.
  `< 0`이면 기존처럼 병진 alpha를 그대로 따른다(§8 참고).
- 초기화(`/initialpose` 또는 SLAM 자동 초기화) 직후 **첫 출력은 ICP 생 포즈
  그대로** 발행하고, 이후부터 필터를 적용한다.
- `/pf/pose/odom`의 포즈와 `map -> odom` TF 모두 필터링된 포즈 기준이다.
  twist는 **휠 오도메트리 값을 그대로 전달**한다(MCL과 동일 관례, `linear.x`/
  `angular.z`만, `linear.y`=0). 포즈 델타 기반 속도는 노이즈가 커서(실측 std
  0.285 m/s, run_0818_173938) 제어기의 L1 전방거리·속도 PI·사전감속을 디더링했다.
- SLAM 모드에서 **맵 누적은 ICP 생 포즈 기준**을 유지한다. 누적 맵은 ICP
  추정과 일관되어야 하고, 필터 지연(lag)이 맵에 번지는 것을 막기 위해서다.
  출력만 필터링한다.

### 차체 롤 보상

`transition_global`의 차체 기울기 보정만 prototype 파이프라인에 이식했다. prototype의
scan-end FIFO, strict odometry bracket, deskew 기준, 자동 초기화와 KICP 튜닝은 변경하지 않는다.

```text
yaw_rate = log(delta_odom.rotation).z / dt
speed    = |delta_odom.translation| / dt
a_lat    = speed * yaw_rate
roll     = clamp(roll_gradient_rad_per_mps2 * a_lat, +/-tilt_max_angle_rad)
```

롤은 `base_link` 축에서 정의되므로 LiDAR 외향 파라미터를 이용해 LiDAR 프레임 회전으로
켤레변환한 뒤 각 점에 적용한다. 보정된 점은 2D 동결 맵과 맞도록 z=0에 투영한다.
`tilt_max_point_height_m > 0`이면 기울였을 때 맵 평면에서 벗어나는 점을 버리지만, 10점
미만이 남으면 ICP 입력 고갈을 막기 위해 원본 스캔으로 자동 복귀한다.

- 현재 YAML은 롤 보상을 켜고(`true`) 실측 기울기 `0.0270 rad/(m/s²)`를 사용한다.
- 피치 기울기와 높이 필터는 측정 근거가 없어 0.0으로 유지한다.
- 저속 occupancy/SLAM 기반 맵은 런타임 보상만 켜도 된다. 고속 주행 bag으로
  `mapping_node` 맵을 다시 만들 때는 mapping/localization YAML의 tilt 값을 동일하게 둔다.
- 즉시 되돌리려면 `tilt_compensation_enable: false`로 바꾼다.

### 내장 `/map` 맵 서버

RViz의 2D Pose Estimate 등 맵 기반 도구를 위해, 노드가 가진 맵 점들을
occupancy grid로 래스터화해 `/map`(`nav_msgs/OccupancyGrid`,
transient_local depth 1)으로 발행한다.

- 동결 모드: `.kissmap` 로드 직후 1회 발행(latched)하고, 이후
  `map_publish_period_sec` 주기로 **캐시된 그리드를 재발행**한다. RViz를
  나중에 켜거나 재시작해도(volatile 구독이어도) 주기 재발행으로 맵이
  반드시 도착한다.
- SLAM 모드: `map_publish_period_sec` 주기로 누적 맵을 갱신 발행한다.
- 각 점을 반경 `map_point_dilation_m`의 디스크로 칠하고(점밖에 모르는
  영역은 -1 unknown), 그리드 범위는 점 bbox + 1 m 마진이다.
  `map_grid_resolution`은 원본 occupancy 맵 yaml의 resolution과 같은 의미다.
- **맵 로드 실패는 FATAL 종료**: `map_name`이 비어 있지 않은데 파일이
  없거나 형식이 틀리면 노드가 명확한 에러와 함께 즉시 종료한다(exit 1).
  이전에는 조용히 "pure odometry"로 강등되어 `/map`이 발행되지 않아
  "맵 서버가 안 뜬다"로 보였다. 순수 odometry로 돌리려면 `map_name:=''`을
  명시한다.

### mapping_node (오프라인)

rosbag을 메시지 **개수** 기준으로 순회한다(시간 창 미사용 — sqlite3 bag의
`send_timestamp == 0` 문제 원천 회피). 1패스에서 `/tf`·`/tf_static`을 TF 버퍼에 적재하고,
2패스에서 동결 해제(freeze=false) 상태로 모든 스캔을 등록한다. 매 프레임 등록에 사용된
downsampled 포인트를 최종 포즈로 map 프레임(odom 프레임 기준)에 누적한 뒤, 마지막에
`voxel_size * 0.5`로 voxel downsample하여 `.kissmap`으로 저장한다.

### SLAM 모드 (온라인, localization_node)

`slam_mode: true`이면 동일한 파이프라인이 온라인 SLAM으로 동작한다.

- `freeze_local_map = false`: KISS 로컬 맵이 스캔으로 계속 갱신된다(일반 KISS-ICP
  odometry+mapping 동작). 동결 맵은 로드하지 않으며 `map_name`은 무시된다.
- **초기화**: `/initialpose`가 오면 그 값으로 초기화(기존 경로 그대로). 없으면
  **첫 스캔 시점에 원점 identity로 자동 초기화** — SLAM이므로 map 프레임 = 시작 자세.
- **누적**: 매 스캔 등록 후 `RegisterFrame` 반환의 downsampled 프레임을 `icp_->pose()`로
  map 프레임에 변환해 누적한다(정차 프레임은 제외 — mapping_node와 동일 기준).
- **저장**: `~/save_map` 서비스(`std_srvs/srv/Trigger`) 호출 시, 또는 노드 정상 종료
  (Ctrl-C) 시 누적 맵을 `voxel_size * 0.5`로 voxel downsample하여 `map_output_file`에
  `.kissmap`으로 저장한다.
- **시각화**: `~/map_points`(`sensor_msgs/PointCloud2`, transient_local)로 누적 맵을
  `map_publish_period_sec` 주기로 발행한다(0이면 발행 안 함).

드리프트가 있는 SLAM 결과이므로, 루프가 닫히지 않는 한 장거리 주행에서는 맵이 점점
틀어진다. 만든 맵은 `maps/`에 넣고 `slam_mode: false`의 동결 맵 localization에 사용한다.

### .kissmap 포맷 (이 패키지가 정의)

| 오프셋 | 타입 | 내용 |
|---|---|---|
| 0 | char[8] | 매직 `KISSMAP1` |
| 8 | double | voxel_size |
| 16 | double | max_range |
| 24 | uint64 | 점 개수 N |
| 32 | double × 3N | x, y, z (map 프레임) |

## 3. 구독 / 발행 토픽

### localization_node

| 방향 | 토픽 | 메시지 타입 | QoS |
|---|---|---|---|
| 구독 | `/scan` | `sensor_msgs/msg/LaserScan` | sensor data (best effort) |
| 구독 | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | TF 리스너 |
| 구독 | `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | reliable 10 |
| 발행 | `/pf/pose/odom` | `nav_msgs/msg/Odometry` (frame `map`, child `base_link`) | reliable 10 |
| 발행 | `/tf` (`map -> odom`) | `tf2_msgs/msg/TFMessage` | TF broadcaster |
| 발행 | `/map` | `nav_msgs/msg/OccupancyGrid` (frame `map`) | transient_local 1 |
| 발행 | `~/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | reliable 10 |
| 발행 | `~/map_points` (SLAM 모드만) | `sensor_msgs/msg/PointCloud2` (frame `map`) | transient_local 1 |
| 서비스 | `~/save_map` (SLAM 모드만) | `std_srvs/srv/Trigger` | - |

## 4. 주요 파라미터

파일: `config/kinematic_localization.yaml` (검증된 튜닝값 반영)

### 4.1 E2 파라미터 적용 가드

두 launch 파일은 설치된 YAML의 절대경로가 실제 파일인지 먼저 확인합니다. 파일이 없거나
symlink가 깨졌으면 노드를 시작하지 않습니다. 이어서 각 노드는 YAML의
`config_schema_version`을 컴파일된 기대 버전과 대조합니다. 따라서 다른 세대의 YAML이나
파라미터 파일 없이 `ros2 run`으로 직접 시작하는 경우도 즉시 실패합니다.

정상 기동 시 `/rosout`에 `E2 parameter summary` 한 줄이 기록됩니다. 실차 주행 전에는 다음
순서로 확인합니다.

1. launch가 오류 없이 시작되는지 확인합니다.
2. localization 요약의 `schema`, `gate`, `smoothing_alpha_rot`, `source_voxel_size`,
   `voxel_size`, `tilt`, `roll_gradient`를 확인합니다.
3. 매핑 세션에서는 mapping 요약의 schema와 KICP 설정도 확인합니다.

YAML을 의도적으로 삭제하거나 깨진 symlink로 바꿨을 때 launch가 즉시 실패하고, YAML의
schema 값을 바꿨을 때 노드가 FATAL로 종료해야 가드가 정상입니다.

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `lidar_topic` | `/scan` | 입력 스캔 토픽 |
| `pose_topic` | `/pf/pose/odom` | 포즈 출력 토픽 (MCL 호환) |
| `map_frame` / `odom_frame` / `base_frame` | `map` / `odom` / `base_link` | 프레임 이름 |
| `publish_map_odom_tf` | `true` | `map -> odom` TF 발행 여부 |
| `map_name` | `""` | `maps/<map_name>.kissmap` (절대경로면 그대로 사용, 비면 순수 odometry). `slam_mode`에서는 무시 |
| `slam_mode` | `false` | 온라인 SLAM 모드 (동결 맵 미사용, 첫 스캔 자동 초기화, 맵 누적/저장) |
| `map_output_file` | `slam_map.kissmap` | SLAM 맵 저장 경로 (상대경로면 실행 cwd 기준) |
| `map_publish_period_sec` | `2.0` | `~/map_points`·SLAM 모드 `/map` 발행 주기 [s] (0이면 발행 안 함) |
| `smoothing_enable` | `true` | 출력 상보필터(휠 odom 예측 + ICP 보정) on/off |
| `smoothing_alpha` | `0.2` | 상보필터 기본 gain (정지~저속). **실차 튜닝 대상 초기값** |
| `smoothing_alpha_gain` | `0.3` | 속도 적응 gain (v = `smoothing_velocity_full_mps`에서 alpha = base+gain). **실차 튜닝 대상 초기값** |
| `smoothing_velocity_full_mps` | `3.0` | 속도 적응 포화 기준 [m/s]. **실차 튜닝 대상 초기값** |
| `smoothing_alpha_max` | `0.8` | alpha 상한 (재수렴 속도와 노이즈 억제의 트레이드오프) |
| `smoothing_alpha_rot` | `-1.0` (YAML `0.12`) | 회전 전용 gain (속도 부스트 없음). `< 0`이면 병진 alpha를 따름(구버전 동작). §8 코너링 지터 억제 |
| `map_topic` | `/map` | 내장 맵 서버 출력 토픽 |
| `map_grid_resolution` | `0.05` | `/map` occupancy 그리드 해상도 [m/cell] |
| `map_point_dilation_m` | `0.15` | 각 맵 점을 칠하는 디스크 반경 [m] |
| `voxel_size` | `1.0` | voxel 맵 해상도 [m]. 맵 해시 그리드 + 대응 탐색 반경 + 적응 임계값 하한을 함께 결정 — **줄이지 말 것** (0.25에서 수렴 베이슨 붕괴로 재생 수 m 발산, §8) |
| `source_voxel_size` | `-1.0` (YAML `0.25`) | 등록 소스 전용 다운샘플 [m] (코어 패치). 소스 점수만 늘리고 탐색 반경·임계값은 유지. `<= 0`이면 `voxel_size`를 따름(업스트림 동작). §8 |
| `max_range` / `min_range` | `30.0` / `0.1` | 스캔 유효 거리 [m] |
| `max_num_iterations` | `30` | ICP 반복 횟수 |
| `use_adaptive_threshold` | `true` | 적응 correspondence threshold |
| `use_adaptive_odometry_regularization` | `true` | 휠 odom 프라이어 적응 정규화 |
| `deskew` | `true` | 스캔 왜곡 보정 |
| `tilt_compensation_enable` | `false` (YAML `true`) | odometry 기반 차체 롤/피치 스캔면 보상. `false`면 prototype 원래 입력 유지 |
| `roll_gradient_rad_per_mps2` | `0.0270` | `roll = gradient·v·yaw_rate`의 차체별 롤 기울기 [rad/(m/s²)] |
| `pitch_gradient_rad_per_mps2` | `0.0` | 종가속도 기반 피치 기울기. 측정 전에는 0 유지 |
| `tilt_max_angle_rad` | `0.26` | 롤/피치 보상 절대 상한 [rad] |
| `tilt_max_point_height_m` | `0.0` | 평면 이탈점 제거 임계 [m]. 0이면 제거하지 않음 |
| `position_covariance` / `orientation_covariance` | `0.1` | 출력 odometry 기본 공분산 (dead reckoning 중에는 §7.2 규칙으로 증가) |

강건화(§7) 파라미터:

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `watchdog_enable` | `true` | §7.2 스캔 갭 워치독 on/off (slam_mode에서는 강제 off) |
| `watchdog_rate_hz` | `40.0` | 워치독 tick 주기 [Hz] |
| `scan_timeout_sec` | `0.15` | 이 시간 이상 스캔이 없으면 odom 외삽 발행 시작 |
| `max_dead_reckoning_sec` | `2.0` | 외삽 발행 최대 지속 시간 — 초과 시 발행 중단 + ERROR |
| `watchdog_trans_error_rate` | `0.03` | 외삽 중 위치 std 증가율 [m/s] (공분산 반영) |
| `watchdog_rot_error_rate` | `0.10` | 외삽 중 yaw std 증가율 [rad/s] |
| `diagnostics_enable` | `true` | §7.3 정합 품질 진단 발행 on/off |
| `diagnostics_topic` | `~/diagnostics` | 진단 토픽 이름 |
| `min_inlier_ratio_warn` | `0.3` | inlier_ratio가 이 값 미만으로 10프레임 연속이면 WARN 로그 |
| `pose_check_enable` | `true` | §7.4 맵 기반 포즈 유효성 검사 (slam_mode에서는 강제 off) |
| `permissible_radius_m` | `0.15` | 포즈 중심 이 반경 내에 비점유 셀이 없으면 impermissible |
| `gate_enable` | `false` | §7.5 마할라노비스 게이트 — **실측 검증 전 켜지 말 것** |
| `gate_chi2` | `9.21` | 게이트 임계값 (2-DoF 99%; `lateral_dof_enable` 시 11.34 권장) |
| `gate_trans_error_rate` | `0.03` | 예측 공분산 P: 주행거리 비례 위치 std [m/m] |
| `gate_rot_error_rate` | `0.10` | 예측 공분산 P: 회전 비례 yaw std [rad/rad] |
| `gate_meas_std_floor` | `0.02` | 측정 공분산 R 대각 하한 [m] (과신 방지) |
| `gate_force_accept` | `20` | 연속 기각 이 횟수째에 ICP 값 강제 수용 (§7.4 통과 시에만) |
| `lateral_dof_enable` | `false` | §7.6 횡방향 소프트 제약 — **§7.5 게이트 없이 켜지 말 것** |
| `lateral_regularization_scale` | `1.0` | `beta_lat = max(floor/tau², scale·beta)`의 scale |
| `lateral_regularization_floor_tau2` | `1.0` | floor = 이 값 / tau² (beta의 대수적 하한 배수) |

매핑 노드 파라미터는 `config/mapping.yaml` (`bag_path`, `output_path`, `lidar_topic` 등 + 동일 KICP 블록).

## 5. 실행 방법

### 5.1 동결 맵 준비 (둘 중 하나)

방법 A — 기존 occupancy 맵 변환 (매핑 런 불필요):

```bash
ros2 run kinematic_localization pgm_to_kissmap.py \
    <map.yaml> src/kinematic_localization/maps/<map_name>.kissmap \
    --voxel-size 1.0 --max-range 30 --downsample 0.1
# 이후 재빌드(또는 symlink-install 사용 시 불필요)
```

`--downsample`은 점 다운샘플 2D 그리드 간격 [m]이다(기본 0.1). `--voxel-size`는
kissmap 헤더 메타데이터일 뿐 밀도에 영향을 주지 않는다. 1 m voxel 맵은 voxel당
최대 20점(`max_points_per_voxel`)을 유지하므로, 벽 라인을 0.1 m 간격으로 샘플링하면
voxel당 ~10점이 되어 버려지는 점이 없다. 너무 성긴 맵(예: 0.25 m 간격)은 벽 점
사이가 벌어져 고속에서 ICP 대응점이 잘못 스냅된다.

방법 B — 주행 bag으로 오프라인 매핑:

```bash
ros2 launch kinematic_localization mapping.launch.py \
    bag_path:=/path/to/bag output_path:=$(pwd)/src/kinematic_localization/maps/<map_name>.kissmap
```

### 5.2 localization 실행

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch kinematic_localization kinematic_localization.launch.py map_name:=<map_name>
```

기동 후 **초기 포즈 1회 지정** (MCL과 동일 UX):

```bash
# RViz "2D Pose Estimate" 또는 직접 발행
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: -0.427, y: 0.456}, orientation: {z: 0.3651, w: 0.9310}}}}'
```

### 5.3 온라인 SLAM 모드 (주행하면서 맵 만들기)

1. **기동** — 동결 맵 없이 SLAM 모드로 실행:

   ```bash
   ros2 launch kinematic_localization kinematic_localization.launch.py \
       slam_mode:=true map_output_file:=$(pwd)/slam_map.kissmap
   ```

   초기 포즈 지정 불필요 — 첫 스캔에서 원점 identity로 자동 초기화된다
   (map 프레임 = 시작 자세). `/initialpose`를 별도로 주면 그 값으로 초기화된다.

2. **주행** — 평소처럼 주행한다. RViz에서 `~/map_points`(transient_local)를
   Add 하면 누적 맵이 `map_publish_period_sec` 주기로 갱신되며 보인다.

3. **저장** — 주행 종료 후 둘 중 하나:

   ```bash
   ros2 service call /kinematic_localization/save_map std_srvs/srv/Trigger
   # 또는 그냥 Ctrl-C — 정상 종료 시 자동 저장
   ```

   `map_output_file`에 `.kissmap`이 생성된다(저장 전 `voxel_size * 0.5` downsample).

4. **맵 사용** — 생성된 파일을 `src/kinematic_localization/maps/<map_name>.kissmap`으로
   옮기고 재빌드(symlink-install이면 불필요)한 뒤, 일반 localization으로 기동:

   ```bash
   ros2 launch kinematic_localization kinematic_localization.launch.py map_name:=<map_name>
   ```

   SLAM 맵의 map 프레임은 SLAM 시작 자세이므로, 초기 포즈는 **맵 기준 차량 위치**를
   지정해야 한다(SLAM 시작 위치에서 시작하면 identity 근처).

### 5.4 검증 (벽 정합률)

```bash
ros2 run kinematic_localization wall_rate.py \
    --scan-bag <원본 bag> --pose-bag <출력 bag> --map <map.yaml> \
    --intervals 0,11,20,30,45,52
```

## 6. 주의 사항

- 휠 odometry는 **`/odom` 토픽**에서 가져온다(히스토리 보간). 재생 bag에는
  `/odom`, `/scan`, `/tf_static`(base → laser 외향 파라미터)이 반드시 있어야 한다.
- localization 노드는 `slam_mode: false`(기본)에서는 스캔을 map에 누적하지 않는다.
  `slam_mode: true`이면 누적 맵이 만들어지며(§5.3), 그 외에 맵 갱신이 필요하면
  mapping_node로 `.kissmap`을 다시 만들어야 한다.
- 맵 파일의 `voxel_size`/`max_range`가 노드 파라미터와 다륩면 WARN 로그만 뜨고 계속 동작한다
  (정합 품질이 떨어질 수 있으니 맞추는 것을 권장).

## 7. 강건화 기능 (kicp_robustness_plan)

MCL 대비 취약점을 메우는 기능들이다. 기본값 정책: §7.1~§7.4는 **on**(안전
방향으로만 동작), §7.5·§7.6은 **off**(실측 검증 후 활성화). 모든 항목은
개별 파라미터로 재빌드 없이 끌 수 있다(§7.1 NaN 가드만 예외 — 토글 없이 항상 on).

### 7.1 NaN 가드 (항상 on)

**문제**: 초기 포즈가 어긋나거나 맵과 스캔이 안 맞아 ICP 대응점이 0개가 되면
코어에서 0/0 = NaN이 생기고, `Sophus::SE3d::exp(NaN)`의 내부 검증
(SOPHUS_ENSURE)이 **프로세스를 abort**시킨다. `/initialpose`를 찍는 순간
노드가 꺼지던 버그의 원인이 정확히 이 경로다.

**동작**:

1. 코어(Registration.cpp): 대응점 0개면 보정 없이 odometry 예측을 반환
   (진입 전 `return` + 루프 내 재-association 후 `break`).
2. 노드: `RegisterFrame` 직후와 발행 직전에 포즈 유한성 검사. 비정상이면
   마지막 정상 ICP 포즈로 롤백(비-const `pose()` 접근자 — 동결 맵 보존)하고
   그 프레임은 발행하지 않는다.

**검증**: 맵 밖 좌표(예: x=100)에 `/initialpose`를 찍어도 노드가 살아 있고,
odom 예측을 따라 발행이 계속되는지 확인한다.

### 7.2 스캔 갭 워치독 (`watchdog_*`)

**문제**: 발행이 스캔 콜백에만 걸려 있어 라이다가 끊기면 `/pf/pose/odom`과
`map->odom` TF가 통째로 멈춘다(MCL은 40 Hz 타이머 + odom-only 케이스로 계속
발행 — 실차에서 실재했던 조건).

**동작** (`watchdog_rate_hz` 주기 tick):

1. 마지막 스캔 처리 후 `scan_timeout_sec`가 지나면 **odom-only 외삽 발행**:
   `T_out = last_out · (T_odom(앵커)⁻¹ · T_odom(최신))`. ICP는 호출하지 않는다.
2. 발행 공분산을 갭 길이에 비례해 증가시킨다
   (`std += watchdog_*_error_rate × 갭[s]`).
3. 외삽할 때마다 출력 포즈·odom 앵커(`last_scan_stamp_`)·ICP seed
   (`icp_->pose()`)를 **함께 전진**시킨다 — 스캔 복귀 프레임의 이동량 이중
   계상과 odom 링버퍼 이탈, seed 오염을 동시에 막는 3종 세트다.
4. 갭이 `max_dead_reckoning_sec`를 넘으면 발행을 중단하고 ERROR 로그를 남긴다
   (무한 dead reckoning으로 다운스트림을 속이는 것이 더 위험).

slam_mode에서는 강제 off (외삽 재시드가 누적 맵을 오염시키므로).

### 7.3 정합 품질 진단 (`diagnostics_*`)

매 프레임 `~/diagnostics`(`diagnostic_msgs/DiagnosticArray`)로 발행:
`inlier_ratio`(대응점/소스점), `residual_rms`, `tau`, `beta`, `iterations`,
`converged`, `final_dx_norm`, `speed`, `alpha`(상보필터), `dead_reckoning_sec`,
게이트 상태(`gate_rejected`·`gate_d2`·`gate_reject_streak`),
`pose_impermissible`, 롤 보상 상태(`tilt_roll_deg`·`tilt_pitch_deg`·
`tilt_dropped_points`). rqt의 diagnostics 뷰어로 바로 볼 수 있다. 워치독의
odom-only 발행은 새 스캔 보정이 아니므로 tilt 세 값이 0으로 나온다.

- `residual_rms`는 마지막 반복 **진입 시점** 값(1반복 stale)이다. 미수렴
  프레임은 `converged=false` + `final_dx_norm`으로 구분한다.
- `inlier_ratio < min_inlier_ratio_warn`이 10프레임 연속이면 WARN 로그.
- **§7.5·§7.6의 임계값은 이 발행값의 실측 분포로만 정한다.** 무장애물 정상
  주행 bag과 상대차 포함 bag에서 `(residual_rms, inlier_ratio)` **결합 분포**를
  기록할 것 (상대차 근접의 주 지표는 MSR 상승이 아니라 inlier_ratio 하락).

```bash
ros2 topic echo /kinematic_localization/diagnostics
```

### 7.4 맵 기반 포즈 유효성 검사 (`pose_check_*`)

MCL `is_pose_permissible()` 대응. 내장 맵 서버가 캐시한 occupancy grid에서
포즈 중심 반경 `permissible_radius_m` 내에 비점유(미점유·unknown) 셀이
하나도 없으면 — 즉 벽 블롭 속 깊이 들어가 있으면 — impermissible로 판정한다.

- **즉시 기각은 하지 않는다**: WARN 로그 + 진단 플래그만. (넓은 비토는 벽
  밀착 주행이 많은 타이트 맵에서 회복을 막아 오히려 악화 — MCL 재생 검증에서
  확인된 사실.)
- 유일한 강제 용도: §7.5 게이트의 force-accept 차단 조건 (벽 속 재고정 방지).
- 이 패키지의 그리드는 점 주변만 점유로 칠하고 나머지는 unknown이므로,
  "unknown = 벽 아님"으로 해석한다 (MCL의 pgm free 셀과 의미가 다름).
- slam_mode에서는 강제 off (맵이 자라는 중이라 unknown이 정상).

### 7.5 마할라노비스 게이트 (`gate_*`, 기본 off)

ICP 추정이 odom 예측에서 통계적으로 너무 멀면 그 프레임을 기각하고 예측을
발행한다. MCL의 EKF 게이트 대응.

- 혁신량 ν = `(T_pred⁻¹·T_icp).log()`의 (종방향, yaw) 성분
  (§7.6 on이면 횡방향 포함 3-DoF — JTJ 차원을 따라 자동).
- R = `residual_rms² · JTJ⁻¹` (§7.3 값), 대각 하한 `gate_meas_std_floor`,
  미수렴 프레임은 `final_dx_norm` 비례로 부풀림.
- P = 주행거리·회전 비례 (`gate_trans_error_rate`, `gate_rot_error_rate`).
- `d² = νᵀ(P+R)⁻¹ν > gate_chi2`면 기각: 발행은 T_pred로, 코어도
  `icp_->pose() = T_pred`로 롤백(SetPose 아님 — 맵 보존).
- `gate_force_accept` 프레임 연속 기각 시 ICP 값을 강제 수용하되,
  §7.4가 벽 속이라고 판정하면 차단.

**⚠️ 켜기 전 필수 절차**: §7.3으로 정상 주행 bag의 분포를 측정하고, 정상
구간 기각률이 1% 미만임을 재생으로 확인한 뒤에만 켠다. 게이트를 잘못 잠그면
dead reckoning으로 발산한다 (MCL에서 후방 스냅 → 벽 충돌 사고 이력 있음).

### 7.6 횡방향 소프트 제약 (`lateral_*`, 기본 off)

기본 2-DoF(종방향+yaw) 보정은 "옆으로 밀기"를 표현할 수 없어 횡방향 오차를
yaw를 흔들며 호로만 씻어낸다. 이 옵션은 보정을 3-DoF로 풀되, 횡방향에
`beta_lat = max(floor/tau², scale·beta)` 정규화를 걸어 **비홀로노믹 제약을
연속(soft) 버전**으로 바꾼다 (scale→∞면 기존 하드 제약과 사실상 동일).

- off일 때 upstream 2-DoF 경로가 비트 단위로 보존된다.
- **§7.5 게이트 없이 켜지 말 것**: 상대차 점이 벽 점과 오대응되는 순간
  횡방향 순간이동이 가능해지는 것이 최악 실패 모드고, 게이트가 그 안전망이다.
- slam_mode에서는 강제 off (누적 드리프트 억제가 필요한 모드).
- 착수 조건: `/initialpose`에 횡방향 10 cm 오프셋을 주는 재생에서 yaw
  오버슈트가 실측될 때만 (plan §6의 검증 절차).

### 7.7 회귀 검증 하네스

```bash
# 1) 기준선 저장 (전 토글 off 상태와 비교하려면 EXTRA_PARAMS로 끄기)
EXTRA_PARAMS="-p watchdog_enable:=false -p diagnostics_enable:=false -p pose_check_enable:=false" \
  ros2 run kinematic_localization replay_check.sh <bag> <map_name> baseline.tum
# 2) 동일 커맨드 재실행 → 비트 단위 비교, 다르면 non-zero exit
```

**비트 단위 비교의 한계 (2026-08-16 실측)**: 이 하네스는 rate 0.5로도 재생
간 완전 결정론이 **되지 않는다** — odom 히스토리 가장자리 nearest 폴백이
벽시계 타이밍에 따라 달라지는 잔여 레이스가 남아 있다. 147 s 실주행 bag
(rosbag2_1970_01_01-10_04_00, 신맵)으로 3회 재생한 실측:

| 비교 | 바이너리 | max Δpos |
|---|---|---|
| 순정 vs 수정 run1 | 다름 | 2.299 m |
| 수정 run1 vs 수정 run2 | **동일** | 2.299 m |
| 순정 vs 수정 run2 | 다름 | 1.904 m |

**같은 바이너리끼리의 재생 간 편차가 서로 다른 바이너리 간 편차와 같은
수준**이므로 이 편차는 코드가 아니라 재생 환경 기인이다. 세 실행 모두
(1) 5772 poses 스탬프 완전 일치(프레임 드랍 0), (2) 편차는 특정 취약
구간(t≈85–95 s, 120–140 s)에만 국한, (3) 그 밖 구간과 종단에서는 **비트
단위로 재수렴**(마지막 포즈 delta 0.0000 cm)했다. 따라서 회귀 판정 기준은
"cmp 통과"가 아니라: 스탬프 완전 일치 + 취약 구간 밖 mm 미만 + 종단 재수렴.
(취약 구간의 재생 간 발산 자체가 §7.3 진단으로 조사할 가치가 있는 신호다 —
그 구간에서 정합이 이력에 민감할 만큼 약하다는 뜻.)

## 8. 코너링 요레이트 지터 억제 (2026-08-18)

### 8.1 증상과 원인 (실차 bag 실측)

run_0818 실차 bag 3개(~4 m/s, 40 Hz 스캔)에서 **코너에서만** `/pf/pose/odom`
yaw가 프레임 단위로 톱니 진동(부호 교대율 80%)했다. IMU 자이로를 물리 기준으로
프레임(25 ms)당 yaw 오차를 분해한 결과:

| yaw 오차 (std/프레임) | 직진 | 코너 |
|---|---|---|
| 휠 odom 예측 | 0.03° | 0.06° |
| raw ICP 측정 (필터 역산 복원) | 0.33° | **0.71~2.65°** |
| 발행 출력 | 0.17° | **0.36~1.01°** (≈14~40°/s) |

- 물리 진동이 아니라 **ICP yaw 측정 노이즈가 코너에서 2~5배 증폭**된 것.
  횡방향 위치 지터도 직진 1.7 mm → 코너 17~24 mm.
- ICP 혁신의 횡방향 성분은 0.9 mm(2-DoF 아크 구속 정상). 노이즈는 자유도가
  열린 yaw·종방향에만 있다 — §7.6 lateral DoF나 §7.5 게이트로 풀 문제가 아니다.

**근본 원인 1 — 등록 소스 붕괴**: `voxel_size 1.0`(kiss-icp 실외 3D 기본값)의
0.5x/1.5x 이중 다운샘플이 1081점 스캔을 **중앙값 14점(9~25점)**으로 붕괴시켰다.
이 트랙은 중앙 레인지 1.3~1.5 m라 1.5 m 복셀 하나가 복도 폭 전체다. 회전 중에는
복셀 생존점이 프레임마다 ±25% 교체(churn)되어 yaw 추정이 흔들린다.

**근본 원인 2 — 필터가 yaw를 과하게 당김**: 속도 부스트가 붙은 alpha(≈0.5)가
병진·회전에 동일 적용되어, odom 예측보다 10배 노이즈가 큰 ICP yaw를 절반씩
출력에 전달했다.

### 8.2 왜 voxel_size를 줄이면 안 되는가 (실측)

소스 붕괴의 1차 시도였던 `voxel_size 1.0 → 0.25`는 **오히려 localization을
발산시켰다**. kiss-icp에서 voxel_size는 소스 다운샘플만이 아니라 (1) 맵 해시
그리드, (2) **대응 탐색 반경**(인접 복셀만 탐색 → 도달 거리 ≈ 복셀 크기),
(3) **적응 임계값 하한**(`map_resolution() = voxel/√20`)을 함께 결정한다.
0.25에서는 수렴 베이슨이 ~1.5 m → ~0.3 m로 좁아져, 일시적 오차를 넘어서는
순간 대응을 잃고(inlier 비율 1.00 → 0.26) odometry 드리프트로 미끄러졌다
(run_0818 재생에서 라이브 궤적 대비 수 m 발산).

해법은 **소스 전용 다운샘플 분리** — 코어 패치 `Config::source_voxel_size`
(`> 0`이면 소스 0.5x/1.5x 캐스케이드에만 적용, 맵 그리드·탐색 반경·임계값은
`voxel_size` 유지, `<= 0` = 업스트림 동작).

### 8.3 수정과 검증 (run_0818_140819, 212 s, 라이브 프레임 맵 재생 A/B)

수정 2건 (모두 YAML, 코어 패치 1건):

1. `source_voxel_size: 0.25` — 등록 소스 14점 → 51점 (inlier 1.00 유지).
2. `smoothing_alpha_rot: 0.12` — 상보필터 회전 성분 전용 고정 gain
   (§2 "출력 스묨" 참고). §7.3 진단에 `alpha_rot` KV 추가.

검증: 라이브 bag의 스캔을 라이브 pf 포즈로 누적(지속성 필터 ≥10 프레임 관측,
0.1 m 셀)한 라이브-프레임 맵으로 4개 변형을 동조건 재생, IMU 자이로 기준
코너(|wz|>0.6 rad/s) 프레임당 yaw 오차:

| 변형 | 소스 | 코너 yaw 지터 | 직진 | 벽 정합 p99 |
|---|---|---|---|---|
| 기존 (voxel 1.0, 균일 α) | 14점 | 0.397°/fr = 15.4°/s | 0.152° | 76 mm |
| + source 0.25 만 | 51점 | 0.288°/fr = 11.1°/s | 0.136° | 75 mm |
| + α_rot 0.12 만 | 14점 | 0.184°/fr = 7.1°/s | 0.080° | 78 mm |
| **+ 둘 다 (채택)** | 51점 | **0.165°/fr = 6.4°/s** | **0.078°** | **66 mm** |
| (참고) voxel 0.25 | 51점 | 1.1~1.6°/fr | — | **발산** |

- 코너 요레이트 노이즈 **15.4 → 6.4°/s (2.4배 감쇠)** — 수정 전 직진 수준
  (5.9°/s)까지 내려옴. 궤적은 라이브 레이싱 라인과 동일하게 추종.
- 절대 벽 정합(스캔→맵 최근접 중앙값)은 4개 변형 모두 37~38 mm로 동등,
  채택안이 p99 꼬리 최선.
- 교란 초기화(+0.3 m, +10°) 수렴 스모크 통과 — voxel 1.0 유지로 베이슨 보존.

### 8.4 되돌리기 / 재튜닝

- 구버전 동작 복원: `-p source_voxel_size:=-1.0 -p smoothing_alpha_rot:=-1.0`.
- yaw 수렴이 너무 느리면(초기화 직후 헤딩 오차가 0.5 s 넘게 남으면)
  `smoothing_alpha_rot`을 0.2까지 올린다. 코너 지터가 남으면 0.08까지 내리되,
  odom yaw 바이어스(§7.3 진단 `alpha_rot`·`speed`와 함께 확인)가 0.5°/s를
  넘는 차에서는 내리지 말 것.
- `source_voxel_size`는 0.15까지 내려도 안전 방향(소스 증가·탐색 반경 불변)
  이나, 젯슨 40 Hz 처리 시간을 §7.3 진단으로 확인하며 조정할 것.
- `voxel_size` 자체는 줄이지 말 것 (§8.2).

## 9. 코너 와이드 주행 — 슬립 은폐 진단과 §5·§6 활성화 (2026-08-18)

### 9.1 증상

같은 제어 파라미터에서 MCL은 라인 안쪽으로 도는데, kinematic_localization은
min-curv 경로임에도 직선을 우선시하며 크게 돌았다 (실차 관찰, run_0818).

### 9.2 진단 (bag 실측 체인)

1. **pose는 라인 위**: run_0818_140819(212 s, 4.1 m/s)에서 기록된 pose의
   레이스라인 대비 코너 횡오차는 평균 +3.6 cm(±10 cm) — 컨트롤러는 "라인 위"
   라고 믿고 만족한 상태였다. 16:20 재주행 bag도 동일(+3.0 cm).
2. **차는 물리적으로 슬립 영역**: 코너 횡가속 p50 5.1 / p90 6.7 m/s²
   (IMU 교차검증 일치), 지시 조향의 무슬립 운동학 예측 대비 실측 요레이트
   비율 p50 **0.66** — 타이어가 크게 미끄러지는 언더스티어 영역.
3. **결론**: 2-DoF 아크 구속(전진+요)은 횡슬립을 표현할 수 없고, 휠 odom
   프라이어도 슬립을 모른다 → 코너에서 pose가 실제보다 **안쪽**에 남는다.
   컨트롤러는 오차를 못 보므로 조향을 더하지 않고, 실차만 바깥으로 밀린다.
   MCL은 파티클이 전 자유도로 퍼져 슬립을 추종하므로 컨트롤러가 보정했다
   (같은 파라미터로 "안쪽 주행"). §8의 지터와는 별개인 **저주파 바이어스**
   문제이며, 정확히 §7.6 lateral DoF가 설계된 증상이다.

### 9.3 검증과 활성화

live-frame 맵 재생 A/B (`lateral_dof_enable:=true`, §8.3과 같은 조건):

| 항목 | lateral off | lateral on |
|---|---|---|
| 코너 yaw 지터 | 6.4°/s | 6.1°/s (악화 없음) |
| 코너 inside-offset | +6.2 cm (안쪽 바이어스) | +0.2 cm (소멸) |
| 코너 횡 이동 | — | 평균 6 cm 바깥쪽 |
| 라이브 대비 안정성 p50/max | 15.1/70 cm | 12.5/59 cm (개선) |
| §5 게이트(χ²=11.34) 개입 | — | 0회 / 8401프레임 |

- 검증 맵(live-frame 맵)은 라이브 pose로 만들어 **안쪽 바이어스를 상속**하므로
  위 6 cm는 하한이다. 젯슨의 원본 맵(저속 매핑, 무편향)에서는 더 크게 표현된다.
- 플랜의 활성화 전제 충족: 증상 실측 ✓, §3 분포 실측(residual_rms p99 0.21,
  inlier p1 1.00, 비수렴 0) ✓, §5 게이트 동반(χ² 11.34, 오탐 0) ✓ →
  **YAML 기본값 `lateral_dof_enable: true` + `gate_enable: true`로 전환.**
- 되돌리기: `-p lateral_dof_enable:=false -p gate_enable:=false`.

### 9.4 남는 항목 (localization 밖)

- **조향 전달률 0.66은 제어/LUT 쪽 이슈로 별도 확인 필요**: 슬립을 pose가
  보여줘도, 지시 조향 대비 실제 요레이트가 66%면 컨트롤러가 그만큼 더 큰
  각을 명령해야 한다. 현재 LUT(`LUT_calibrated.csv`)가 고횡가속 영역을
  과소 보정하는지 점검할 것.
- 첫 실차 적용은 중속 셰이크다운으로: 코너에서 pose가 바깥으로 더 나오는
  만큼 컨트롤러가 조향을 더 쓰게 되므로 거동이 눈에 띄게 달라진다.
