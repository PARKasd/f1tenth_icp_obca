# 지도 없는 ICP + OBCA 주행

## 1. 목적과 구성

초기 위치와 실제 차체 방향을 수동 지정하고, 최근 LiDAR 관측으로 경로를 생성하는
ROS 2 Jazzy 저속 프로토타입입니다. 모든 런타임 노드는 C++17입니다.

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
5. 레이가 지나간 셀을 free, 유효한 반사 끝점을 occupied로 기록합니다. 모든 free ray를
   처리한 뒤 끝점을 기록해 같은 스캔에서 이웃 레이가 장애물을 지우지 않도록 합니다.
   NaN은 무시합니다. +inf/range_max는 기본적으로 free 증거가 아닙니다.
6. `map_radius` 바깥과 `map_ttl`보다 오래된 관측은 제거합니다. 계획용 로컬 지도는 제한된
   범위만 유지하고, ICP의 누적 지도는 별도로 저장할 수 있습니다.
7. 연결된 free 셀 안에서 Dijkstra 탐색을 수행합니다. 대각선으로 막힌 코너를 통과하지
   않으며 전진성·이동 거리·이전 경로와의 연속성을 점수화해 전방 목표점을 선택합니다.
8. 기준 경로와 이전 성공 해를 초기값으로 사용해 OBCA 최적화를 수행합니다.
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
독립 검사에서 쓰는 보수적 여유와 이산화 오차를 수용합니다. 따라서 실제 최소 계획 여유는
`margin` 한 값보다 큽니다. 장애물 셀 자체도 반사점보다 보수적인 사각 영역입니다.

목적함수는 기준 위치·방향·속도 추종과 조향·가속도·조향 변화의 합입니다.
속도, 조향각, 조향 변화율, 가감속, 횡가속도를 제한합니다.
Ipopt C API와 희소 analytic Jacobian을 사용하고 Hessian은 limited-memory로 근사합니다.
최적화가 끝나면 별도로 모든 제약 잔차를 계산합니다.

충돌 검사는 인접 상태 사이를 세분하고, 양 끝 직사각형의 convex hull에 회전 호의
sagitta를 더해 보간 구간 전체를 검사합니다. occupied 셀에는 margin도 적용합니다.
이 검사는 계획 모델의 궤적에 대한 검사이며 실제 차량의 추종 오차 보장은 아닙니다.

서로 인접한 occupied 셀만 사각형으로 합칩니다. 도달 가능 범위를 감싼 사각 ROI 안의
장애물을 모두 포함하며, `max_obstacles`를 초과하면 정지합니다. 일부를 임의로 버리지 않습니다.
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

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `front/rear/half_width` | 0.38 / 0.14 / 0.16 m | base_frame 기준 차체 형상, 실제 장착 기준점 확인 필요 |
| `wheelbase` | 0.3302 m | bicycle model 축간거리 |
| `max_speed` | 0.8 m/s | 저속 프로토타입 상한 |
| `max_accel/max_decel` | 1.0 / 1.5 m/s² | 계획 한계, 실제 제동 성능 확인 필요 |
| `horizon/dt` | 16 / 0.2 s | 계획 예측 구간 |
| `solve_seconds` | 0.15 s | solver 시간 예산, 초과 결과는 폐기 |
| `planning_period/control_period` | 0.1 / 0.02 s | 계획·추종 타이머 |
| `input_timeout/path_timeout` | 0.3 / 0.3 s | ROS stamp와 실제 수신 경과시간 모두 검사 |
| `grid_resolution/map_radius/map_ttl` | 0.1 m / 6 m / 2 s | 계획용 지도 |
| `reference_distance/reference_clearance` | 2.5 / 0.23 m | 기준 경로 탐색 범위·중심 여유 |
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
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch obca_navigation navigation_sim.launch.py
```

시뮬 프레임 이름은 실제 bridge 설정과 일치시켜야 합니다. gym이 이미 map 기반 TF를
발행하는 경우 중복 부모를 만들지 않도록 시뮬 프로필은 ICP의 map→odom 발행을 끕니다.
시뮬 기본값은 `use_sim_time=true`이므로 `/clock`이 필요합니다. `/clock`이 없는 gym은
`use_sim_time:=false`로 실행합니다. 실차는 map→odom TF를 발행하며 기존 mux의
`/drive_autonomous`를 사용합니다. 다른 mux 구성에서는 `drive_topic`을 변경하십시오.
센서 드라이버·mux·gym은 이 저장소의 launch에 포함하지 않습니다.

## 7. 정지 이유와 한계

- `waiting for initial pose and ICP`: 초기 위치 또는 그 이후의 위치 샘플 대기.
- `waiting for fresh synchronized scan/TF`: 스캔 시각을 둘러싼 위치 또는 센서 TF 부족.
- `stale or degraded localization`: 오래된 입력, 낮은 정합률, 큰 잔차, 미수렴 등.
- `obstacle budget exceeded`: 장애물 사각형 수가 설정 한도를 초과함.
- `Ipopt status=...`: 비수렴·불가능한 문제·계산시간 초과.
- `swept footprint reaches ...`: 장애물 또는 미관측 영역에 차체가 닿음.

초기 버전에는 루프 클로저, 전역 지도 최적화, 상대차 운동 예측, 교차로 경로 선택,
막다른 길 후진 복구, 레이싱 속도 최적화가 없습니다. ICP 누적 지도의 장시간 메모리와
드리프트도 별도 검증 대상입니다. 관측 지도에 동적 물체가 들어오면 일시적인 장애물로
취급하며 움직임을 예측하지 않습니다. 첫 검증 대상은 정적 트랙의 저속 주행입니다.
