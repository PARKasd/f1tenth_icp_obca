# ICP + 최소 곡률 레이스라인 + OBCA 주행

## 1. 목적과 구성

초기 위치와 실제 차체 방향을 수동 지정하고, 최소 곡률 레이스라인과 최근 LiDAR 관측으로 경로를 생성하는
ROS 2 Humble / Ubuntu 22.04 저속 프로토타입입니다. 모든 런타임 노드는 C++17입니다.
Jazzy / Ubuntu 24.04는 `main` 브랜치를 사용합니다. 배포판 전환 시 빌드 폴더를 공유하지 않습니다.

| 노드 | 역할 |
|---|---|
| `kinematic_localization/localization_node` | 휠 odometry 예측 + 최근 스캔 맵에 대한 Kinematic ICP |
| `obca_navigation/planner_node` | 관측 지도, 연결된 통로의 기준 경로, OBCA 최적화와 검증 |
| `obca_navigation/tracker_node` | Pure Pursuit 추종, 속도 제한, 독립적인 입력 신선도 감시 |

기존 `global_planning`, `local_planning`, `state_machine`, `f1tenth_control`은 실행하지 않습니다.
이 패키지의 추종기는 기존 L1/LUT 제어기를 복사한 것이 아닙니다.

## 2. 동작 순서

1. launch 직후에는 `/initialpose`를 기다리고 속도 0 명령을 발행합니다.
2. 초기 위치가 들어오면 ICP·계획 지도·기존 경로를 초기화합니다. yaw는 차체가 실제로 향한
   방향의 좌표 표현입니다. 이후 코너에서는 현재 차체 방향과 관측 통로를 따라 목표점이 변합니다.
3. wheel odometry prior와 ICP로 `/pf/pose/odom`을 발행합니다. 기존 스캔 끝 시각 동기화와
   deskew 로직을 유지합니다. frozen map은 사용하지 않습니다.
4. 계획기는 스캔 시작·끝을 둘러싸는 위치 샘플이 확보될 때까지 기다립니다. 두 위치 사이를
   보간하고 `base_frame → scan.frame_id` 외부변환으로 각 레이의 원점·방향을 계산합니다.
   저속 평면 운동을 가정하며 이 보간이 실제 운동을 완벽히 복원하는 것은 아닙니다.
   높은 센서 발행률에서 큐 전체를 처리하다 입력이 만료되지 않도록, 매 계획 주기에는
   위치 기록과 동기화된 가장 최신 스캔 한 개만 반영합니다. 처리하지 않은 관측을 free로 추정하지 않습니다.
5. 레이가 지나간 셀을 free, 유효한 반사 끝점을 occupied로 기록합니다. 모든 free ray를
   처리한 뒤 끝점을 기록해 같은 스캔에서 이웃 레이가 장애물을 지우지 않도록 합니다.
   NaN은 무시합니다. +inf/range_max는 기본적으로 free 증거가 아닙니다.
6. `map_radius` 바깥과 `map_ttl`보다 오래된 관측은 제거합니다. 계획용 로컬 지도는 제한된
   범위만 유지하고, ICP의 누적 지도는 별도로 저장할 수 있습니다.
7. 시뮬 기본 `raceline` 모드는 원본 저장소의 offline generator CSV를 읽습니다. 초기 방향에 맞춰
   순방향 또는 역방향을 선택하고, 가까운 arc-length 구간에서 현재 진행 위치를 갱신합니다.
   누적 진행 거리는 랩 경계를 넘어 이어지며 가까운 반대편 헤어핀으로 재탐색하지 않습니다.
   `local` 모드에서는 연결된 free 셀 안에서 Dijkstra 탐색을 수행합니다. 대각선으로 막힌 코너를 통과하지
   않으며 전진성·이동 거리·이전 경로와의 연속성을 점수화해 전방 목표점을 선택합니다.
   초기화 직후에는 `startup_straight_distance`만큼 전진할 때까지 별도의 직진 reference를
   사용합니다. 관측된 공간에서 차체 전체가 통과하는 구간만 생성하고, 최적화의 조향각과
   yaw를 고정합니다. 앞이 막혔으면 정지하며 강제로 직진하지 않습니다.
8. 기준 경로와 이전 성공 해를 초기값으로 사용해 OBCA 최적화를 수행합니다. 레이스라인의
   속도 프로파일은 차량 상한·감속 한계와 함께 적용합니다. 레이스라인이 새 장애물과 겹쳐도
   목표 자체를 삭제하지 않고 OBCA가 우회하도록 합니다. 지도에서 생성한 레이스라인은
   현재 공간이 안전하다는 증거가 아니며, 최종 경로는 관측된 공간 안에 있어야 합니다.
   이전 해가 없으면 기준선 방향의 제한된 조향과 감속으로 bicycle model 초기 궤적을 만듭니다.
   격자 코너 위에 상태를 놓고 조향만 0으로 시작하던 초기값의 비일관성을 줄입니다.
   이 초기 궤적은 직접 발행하지 않으며 동일한 최적화·충돌 검사 후에만 경로를 발행합니다.
9. 해 상태, 시간 제한, 모든 변수·제약 잔차, 연속 구간의 차량 swept footprint를 검사합니다.
   검사에 실패하면 빈 `WpntArray`를 발행하며 추종기는 정지합니다.
10. 성공한 경로를 별도 추종 프로세스가 따라갑니다. 최적화가 오래 걸려도 추종기의
    wall timer는 독립적으로 동작해 오래된 경로·스캔·위치·ICP 진단을 정지 처리합니다.

차량이 이미 차지한 실제 직사각형은 후방 센서 사각지대에서도 알려진 영역입니다.
미관측 셀 전체를 free로 바꾸지는 않고, 미관측 셀과 검사 차체의 교집합이 현재 차체 안에
완전히 포함될 때만 허용합니다. 이 처리는 차체 바깥의 미관측 공간으로 확장되지 않습니다.

## 3. OBCA 최적화

상태는 `(x, y, yaw, v)`, 입력은 `(steering, acceleration)`입니다.
시간 간격 `dt`의 Euler bicycle model을 사용하며 `wheelbase`는 상태 기준점과 맞아야 합니다.
기본 horizon은 16, dt는 0.2초입니다. 매 계획마다 종단 속도는 0으로 고정합니다.

장애물 직사각형은 `A p ≤ b`, 차체 직사각형은 차체 좌표에서 `G q ≤ g`로 표현합니다.
각 시점·장애물마다 4차원 `lambda`, `mu`를 두고 다음 제약을 풉니다.

```text
lambda >= 0, mu >= 0
||Aᵀ lambda||² <= 1
Gᵀ mu + R(yaw)ᵀ Aᵀ lambda = 0
(A t - b)ᵀ lambda - gᵀ mu >= d_min
```

이는 차량 중심점만 피하는 비용함수가 아니라 회전하는 직사각형 차체의 거리 제약입니다.
`d_min = sqrt(2) * (margin + validation_step) + validation_tolerance`로 설정해
미래 상태의 독립 검사에서 쓰는 보수적 여유와 이산화 오차를 수용합니다. 고정된 현재 상태는
그 앞에 보간 구간이 없으므로 `sqrt(2) * margin + validation_tolerance`를 적용합니다.
현재 상태도 차체 및 margin 검사를 통과해야 합니다. 따라서 실제 최소 계획 여유는
`margin` 한 값보다 큽니다. 장애물 셀 자체도 반사점보다 보수적인 사각 영역입니다.

목적함수는 기준 위치·방향·속도 추종과 조향·가속도·조향 변화의 합입니다.
속도, 조향각, 조향 변화율, 가감속, 횡가속도를 제한합니다.
Ipopt C API와 희소 analytic Jacobian·Hessian을 사용합니다. Hessian은 목적함수와
제약 Lagrangian의 해석적 2차 미분이며, 수치 미분 회귀 검사로 확인합니다.
최적화가 끝나면 별도로 모든 제약 잔차를 계산합니다.

충돌 검사는 인접 상태 사이를 세분하고, 양 끝 직사각형의 convex hull에 회전 호의
sagitta를 더해 보간 구간 전체를 검사합니다. occupied 셀에는 margin도 적용합니다.
이 검사는 계획 모델의 궤적에 대한 검사이며 실제 차량의 추종 오차 보장은 아닙니다.

서로 인접한 occupied 셀만 사각형으로 합칩니다. 도달 가능 범위를 감싼 사각 ROI 안의
장애물을 모두 포함하며, `max_obstacles`를 초과하면 정지합니다. 일부를 임의로 버리지 않습니다.
최적화의 시점별 장애물 제약은 별도로 줄입니다. 각 시점까지의 최대 이동 거리를 속도·가감속
한계와 종단 정지 조건으로 계산하고, 차체 외접 반경과 안전 여유·검증 오차까지 더해도
도달할 수 없는 장애물만 해당 시점의 제약에서 제외합니다. 출발 직진 모드에서는 고정 yaw와
직선 위치 한계로 더 좁은 도달 영역을 사용합니다. 최종 검증은 생략한 장애물을 포함한 전체
관측 지도를 그대로 사용합니다. `/obca/status`의 `pairs`는 실제 OBCA 제약 쌍 개수입니다.
unknown 공간은 현재 구현에서 최적화 후의 전체 footprint 검사로 막습니다. 따라서 경로가
존재하더라도 최적화가 unknown 쪽으로 휘면 정지할 수 있으며, 완전한 탐색 알고리즘은 아닙니다.

## 4. 토픽과 메시지

| 토픽 기본값 | 형식 | 방향·용도 |
|---|---|---|
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 전체 초기화, `map` frame |
| `/scan` | `sensor_msgs/LaserScan` | ICP와 계획 입력, sensor-data QoS |
| `/odom` | `nav_msgs/Odometry` | ICP 휠 prior, 시뮬에서는 launch로 변경 |
| `/pf/pose/odom` | `nav_msgs/Odometry` | ICP 출력, 계획·추종 입력 |
| `/kinematic_localization/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 정합률·잔차·수렴·거부 여부 |
| `/obca/local_grid` | `nav_msgs/OccupancyGrid` | 계획용 지도: -1 unknown / 0 free / 100 occupied |
| `/obca/waypoints` | `f110_msgs/WpntArray` | 검사 완료 경로, 빈 배열은 정지 |
| `/obca/path` | `nav_msgs/Path` | RViz 경로 |
| `/obca/raceline` | `nav_msgs/Path` | 전체 기준 레이스라인, transient-local QoS |
| `/obca/status` | `std_msgs/String` | 성공·정지 이유 및 계산시간 |
| `/obca/drive` | `ackermann_msgs/AckermannDriveStamped` | 추종 출력, 계획기의 최근 조향 추정 입력 |

`Wpnt.s_m`은 이번 로컬 경로의 누적 길이이며 글로벌 Frenet 좌표가 아닙니다.
`x_m/y_m/psi_rad/vx_mps/ax_mps2/kappa_radpm`을 채우고 미사용 Frenet·트랙 경계 필드는
기본값으로 둡니다. 경로의 header는 최적화가 시작된 위치 샘플의 시각입니다.
`WpntArray`에는 시점별 시간이 없으므로 추종기는 시간 궤적을 정확히 실행하는 MPC가 아니라
기하 경로와 속도를 추종합니다.

## 5. 주요 설정

파일: `config/navigation.yaml`. launch의 `params_file`로 다른 공통 YAML을 선택할 수 있습니다.
실차용 `navigation_real.launch.py`는 `config/real.yaml`, 시뮬용 `navigation_sim.launch.py`는
`config/sim.yaml`을 공통 YAML 다음에 적용합니다. 마지막으로 명시한 CLI 인자가 덮어씁니다.
CLI 인자를 생략하면 YAML 값을 유지합니다. 공통 launch만 실행하면 환경별 파일 없이
`/obca/drive`로 명령을 발행합니다.
ICP 전체 기본값은 `kinematic_localization/config/kinematic_localization.yaml`을 먼저 읽고
navigation YAML과 launch 선택값으로 덮어씁니다.
Jazzy의 wildcard·노드별 YAML 우선순위 차이를 피하기 위해 launch가 노드별 최종 파라미터를
하나의 사전으로 합친 뒤 전달합니다. 저속 평면 gym에서는 `sim.yaml`의 ICP voxel 0.25 m,
source voxel 0.1 m, 수렴 임계값 0.0001을 사용하고 차체 roll 보정을 끕니다. 실차는 기존
1.0 m voxel을 유지합니다. 시뮬 튜닝값을 고속 실차에 그대로 적용하지 마십시오.

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `reference_mode` | local, 시뮬 raceline | 로컬 탐색 또는 글로벌 레이스라인 기준 |
| `raceline_file` | 시뮬 launch에서 설치된 map.csv | x_m/y_m/psi_rad/vx_mps 열이 있는 CSV |
| `raceline_max_error/raceline_max_heading_error` | 1.0 m / 1.2 rad | 초기화 및 추종 투영 허용 오차 |
| `raceline_forward_window/raceline_backward_window` | 2.0 / 0.3 m | 이전 진행 거리 주변의 탐색 범위, 각각 반 랩 미만 |
| `front/rear/half_width` | 0.38 / 0.14 / 0.16 m | base_frame 기준 차체 형상, 실제 장착 기준점 확인 필요 |
| `margin` | 0.12 m | 운영 YAML의 차체 추가 여유; 미래 OBCA 거리 제약은 약 0.227 m |
| `wheelbase` | 0.3302 m | bicycle model 축간거리 |
| `max_speed` | 0.8 m/s | 저속 프로토타입 상한 |
| `max_accel/max_decel` | 1.0 / 1.5 m/s² | 계획 한계, 실제 제동 성능 확인 필요 |
| `horizon/dt` | 16 / 0.2 s | 계획 예측 구간 |
| `solve_seconds` | 0.15 s | solver 시간 예산, 초과 결과는 폐기 |
| `planning_period/control_period` | 0.1 / 0.02 s | 계획·추종 타이머 |
| `input_timeout/path_timeout` | 0.3 / 0.3 s | ROS stamp와 실제 수신 경과시간 모두 검사 |
| `grid_resolution/map_radius/map_ttl` | 0.1 m / 6 m / 2 s | 계획용 지도 |
| `reference_distance/reference_clearance` | 2.5 / 0.35 m | 로컬 기준 경로 탐색 범위·중심 여유 |
| `startup_straight_distance` | 0.3 m | 초기화 후 직진 제약을 유지하는 전방 이동 거리, 0이면 사용하지 않음 |
| `seed_lookahead` | 0.45 m | 이전 해가 없는 최적화 초기 궤적의 조향 목표 거리 |
| `goal_*_weight` | YAML 참조 | 전진성·측방향·경로 길이·기존 목표점 연속성 |
| `lookahead/max_tracking_error` | 0.45 / 0.25 m | Pure Pursuit 전방거리·오차 정지 한계 |
| `min_inlier_ratio/max_icp_residual` | 0.4 / 0.35 m | ICP 진단 통과 기준, 수렴도 요구 |
| `no_return_is_free` | false | 센서가 명시적으로 clear ray를 표현할 때만 활성화 |

Ipopt 중간 콜백은 반복 사이에만 시간을 확인하므로 hard real-time 선점은 아닙니다.
별도 추종 프로세스의 watchdog이 계산 중단 지연을 보완하지만 실제 제동거리 보장은
센서 지연·통신·제어기·차량 성능까지 포함한 폐루프 검증이 필요합니다.

## 6. 실행 순서

1. 저장소 루트에서 README의 의존성 설치·빌드를 수행합니다. 이전 저장소 상위 폴더에서
   colcon을 실행하면 중복 패키지를 찾을 수 있으므로 새 저장소 안에서 빌드하십시오.
2. 센서 또는 gym을 실행합니다. `base_frame → LiDAR frame` TF가 있어야 합니다.
3. 실차는 `ros2 launch obca_navigation navigation_real.launch.py`, 시뮬은
   `ros2 launch obca_navigation navigation_sim.launch.py`를 실행합니다.
4. RViz의 2D Pose Estimate 또는 `/initialpose` 1회 발행으로 시작합니다. frame은 `map`입니다.
5. `/obca/status`, `/obca/local_grid`, `/obca/path`와 해당 환경의 제어 토픽을 확인합니다.
   실차는 `/drive_autonomous`, 시뮬은 `/drive`입니다. 관찰용 출력으로 바꾸려면
   `drive_topic:=/obca/drive`를 추가합니다.
6. 시뮬에서 명령을 연결하려면 아래 예시를 사용합니다.

```zsh
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch obca_navigation navigation_sim.launch.py
```

시뮬 프레임 이름은 실제 bridge 설정과 일치시켜야 합니다. gym이 이미 map 기반 TF를
발행하는 경우 중복 부모를 만들지 않도록 시뮬 프로필은 ICP의 map→odom 발행을 끕니다.
시뮬 기본값은 일반 gym bridge에 맞춘 `use_sim_time=false`입니다. `/clock`을 발행하고
센서·TF도 같은 시뮬 시각을 사용하는 bridge에서만 `use_sim_time:=true`로 실행합니다. 실차는 map→odom TF를 발행하며 기존 mux의
`/drive_autonomous`를 사용합니다. 다른 mux 구성에서는 `drive_topic`을 변경하십시오.
센서 드라이버·mux·gym은 이 저장소의 launch에 포함하지 않습니다.

## 7. 정지 이유와 한계

- `waiting for initial pose and ICP`: 초기 위치 또는 그 이후의 위치 샘플 대기.
- `waiting for fresh synchronized scan/TF`: `reason`으로 pose bracket 대기, 실제 센서 TF 누락,
  오래된 스캔, 유효 ray 부재를 구분합니다. `map_age`와 `queued`도 확인하십시오.
  이 메시지가 항상 TF 누락을 뜻하지는 않습니다. 유효 상태의 `mapping_ms`는 스캔 반영 시간입니다.
  모든 상태의 `reference_mode`로 현재 `local`/`raceline` 선택도 확인할 수 있습니다.
- `ROS clock not started` / `localization stamp is in the future`: `/clock` 및 `use_sim_time` 불일치.
- `waiting for ICP diagnostics`: 초기화 이후 정합 진단이 아직 수신되지 않음.
- `stale ICP diagnostics` / `stale ICP pose`: 수신 경과시간과 stamp 나이를 초 단위로 확인.
- `ICP quality`: 수렴 여부, 정합률과 최소값, 잔차와 최대값, dead reckoning 및 거부 여부를 확인.
- `reference missing`: 관측된 공간에서 연결된 reference를 찾지 못함. 출발점의 원형 여유 공간 검사가 후방 사각지대에 걸리면, 차량의 실제 swept footprint로 검증한 짧은 직진 연결 구간을 통해 탐색을 다시 시도합니다. 미관측 셀을 자유 공간으로 바꾸지는 않습니다.
- `raceline tracking lost`: 위치·방향 또는 연속 진행 거리 범위를 벗어남. 좌표계, 초기 위치, ICP 누적 오차를 확인합니다.
- `obstacle budget exceeded`: 장애물 사각형 수가 설정 한도를 초과함. 현재 개수와 한도를 함께 표시합니다.
- `Ipopt status=...`: 비수렴·불가능한 문제·계산시간 초과.
- `swept footprint reaches ...`: 장애물 또는 미관측 영역에 차체가 닿음.

초기 버전에는 루프 클로저, 전역 지도 최적화, 상대차 운동 예측, 교차로 경로 선택,
막다른 길 후진 복구, 레이싱 속도 최적화가 없습니다. ICP 누적 지도의 장시간 메모리와
드리프트도 별도 검증 대상입니다. 관측 지도에 동적 물체가 들어오면 일시적인 장애물로
취급하며 움직임을 예측하지 않습니다. 첫 검증 대상은 정적 트랙의 저속 주행입니다.

### 시뮬 초기화 후 정지 원인 확인

1. 기존 navigation 프로세스를 종료하고 `ros2 launch obca_navigation navigation_sim.launch.py use_sim_time:=false`로 실행합니다.
2. RViz의 Fixed Frame을 `map`으로 맞추고 **2D Pose Estimate**로 위치와 방향을 지정합니다.
3. 계속 정지하면 `ros2 topic echo /obca/status`로 구체적인 정지 이유를 확인합니다.
4. `ICP quality`이면 `ros2 topic echo /kinematic_localization/diagnostics --once`를 확인합니다. 첫 스캔은 지도 초기화 때문에 미수렴일 수 있으나 이후에도 계속되면 정합 원인을 조사해야 합니다.
5. 원인 확인 전에 품질 검사나 입력 timeout을 해제하지 않습니다.

## 8. 레이스라인 준비와 시각화

1. [racelines/README.md](../racelines/README.md)의 명령으로 실제 트랙 지도에서 CSV를 생성합니다.
   원본의 최소 곡률 알고리즘을 사용하며, 지도는 오프라인 생성에만 사용합니다.
2. 곡률·벽 여유 경고를 확인하고 차량 조향 한계에 맞는 결과인지 검사합니다.
3. `ros2 launch obca_navigation navigation_sim.launch.py raceline_file:=/absolute/path/global_waypoints.csv`로 실행합니다.
   실차는 `navigation_real.launch.py reference_mode:=raceline raceline_file:=...`를 사용합니다.
4. 같은 지도 좌표계에서 실제 차체 위치와 방향을 `/initialpose`로 지정합니다.
5. RViz Path에 `/obca/raceline`을 추가하고 Durability를 Transient Local로 설정합니다.
   `/obca/path`는 회피·종단 정지 제약을 적용한 현재 OBCA 경로입니다.
6. `/obca/status`의 `raceline_s`와 `lap_length`로 누적 진행 거리를 확인합니다.

레이스라인 파일은 ICP의 절대 위치를 보정하지 않습니다. frozen map이나 loop closure 없이
장시간 주행하면 누적 드리프트로 기준선이 실제 벽에 가까워질 수 있고, 이 경우 OBCA와
관측 지도 검사에 의해 우회하거나 정지합니다. 전역 최단 시간 및 무한 연속주행 보장은 없습니다.
