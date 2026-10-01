# F1TENTH ICP + OBCA

ROS 2 Jazzy / C++17 기반 ICP + 레이스라인 + OBCA 저속 주행 프로토타입입니다.
`/initialpose`로 초기 좌표와 실제 차체 방향을 지정한 뒤, wheel odometry + Kinematic ICP로
스캔을 정합하고, 오프라인 최소 곡률 레이스라인을 기준으로 OBCA 경로를 반복 생성합니다.
기존 지도 없는 로컬 reference 모드도 선택할 수 있습니다.

```text
/scan + /odom + 센서 TF → Kinematic ICP → /pf/pose/odom
       트랙 지도 → 기존 offline_trajectory_generator → 최소 곡률 레이스라인
       /scan + 위치 기록 → 관측 로컬 지도 + 레이스라인 → OBCA
       /obca/waypoints → 별도 추종 노드 → /obca/drive
```

- ICP는 frozen map을 사용하지 않습니다. 시뮬 기본 모드의 레이스라인은 트랙 지도로 미리 생성합니다.
- 레이스라인과 초기 pose는 같은 map 좌표계를 사용해야 합니다. 기존 Frenet 상태 머신은 필요 없습니다.
- OBCA는 직사각형 차체와 장애물의 쌍대 거리 제약을 Ipopt로 풉니다.
- 초기 위치 지정 전, 최적화 실패, 미관측 영역 침범, 입력 지연 시 정지 명령을 발행합니다.
- 초기 설정 최고 속도는 0.8 m/s입니다. 실차 제동·추종 성능이 검증된 값은 아닙니다.
- 공통 launch는 `/obca/drive`, 실차용은 `/drive_autonomous`, 시뮬용은 `/drive`로 명령을 발행합니다.
- 기존 주행 스택과 동시에 `/drive`를 발행하지 마십시오.

## 1. 설치·빌드

Ubuntu 24.04 / ROS 2 Jazzy 환경에서 다음 순서로 실행합니다.

```zsh
cd ~/f1tenth_icp_obca
# 먼저 ~/.zshrc에 기존 빌드/ROS 환경 alias가 있는지 확인하고 활용합니다.
rg 'alias|colcon|jazzy' ~/.zshrc
source /opt/ros/jazzy/setup.zsh
sudo apt install build-essential cmake pkg-config coinor-libipopt-dev libeigen3-dev libtbb-dev
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --packages-up-to obca_navigation
source install/setup.zsh
colcon test --packages-select obca_navigation kinematic_localization
colcon test-result --verbose
```

## 2. 실행

실차에서는 센서·휠 odometry·TF·차량 mux를 먼저 실행합니다. 시뮬에서는 gym bridge를 먼저
실행합니다. 아래 launch들은 ICP·OBCA·추종기 3개를 시작하며 드라이버·mux·gym은 포함하지 않습니다.
`f1sim_C`의 기존 Jazzy bridge를 사용한다면 [LiDAR 각도 간격 수정](patches/README.md)도 확인하십시오.

| 실행 파일 | 설정 파일 | 시간 | 휠 odometry | 제어 출력 |
|---|---|---|---|---|
| `navigation_real.launch.py` | `config/real.yaml` | 실제 시간 | `/odom` | `/drive_autonomous` |
| `navigation_sim.launch.py` | `config/sim.yaml` | 실제 시간 (gym 기본) | `/ego_racecar/odom` | `/drive` |
| `navigation.launch.py` | 공통 설정만 | 실제 시간 | `/odom` | `/obca/drive` |

```zsh
# 실차: 기존 f1tenth_stack 자율 mux 입력으로 전달
ros2 launch obca_navigation navigation_real.launch.py

# 시뮬: 포함된 map.csv 레이스라인 + ego_racecar 프레임 + /drive
ros2 launch obca_navigation navigation_sim.launch.py

# 지도 없는 로컬 reference 모드
ros2 launch obca_navigation navigation_sim.launch.py reference_mode:=local

# 다른 트랙 또는 실차의 레이스라인 (동일 map 좌표계의 CSV 필요)
ros2 launch obca_navigation navigation_real.launch.py \
  reference_mode:=raceline raceline_file:=/absolute/path/global_waypoints.csv

# 명령 출력만 확인하려면 출력 토픽을 덮어쓸 수 있음
ros2 launch obca_navigation navigation_real.launch.py drive_topic:=/obca/drive
```

환경에 맞는 명령 하나만 실행합니다. 시뮬용 기본값은 `base_frame=ego_racecar/base_link`,
`odom_frame=ego_racecar/odom`, `publish_map_odom_tf=false`, `use_sim_time=false`입니다.
일반 gym bridge는 실제 시간으로 센서를 발행하며 `/clock`을 제공하지 않습니다.
`/clock`과 센서·TF 시각을 함께 사용하는 시뮬만 `use_sim_time:=true`로 실행하십시오. 실제 프레임·토픽 이름이 다르면
`base_frame`, `odom_frame`, `wheel_odom_topic`, `scan_topic`, `drive_topic` 인자로 변경합니다.
명령행 인자는 공통 YAML과 환경별 YAML보다 우선합니다.

다른 터미널에서 초기 위치를 지정합니다. 세 launch 모두 초기 위치 지정 전에는 주행하지 않습니다.

```zsh
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: 6.505364, y: -0.146211}, orientation: {z: 0.0270372, w: 0.9996344}}}}'
```

위 좌표는 포함된 레이스라인의 한 시작점입니다. 다른 지도에서는 실제 시작 위치와 방향으로
바꾸십시오. 실차의 초기 pose는 차체를 회전시키는 명령이 아니며 gym에서는 텔레포트로도 사용됩니다.
레이스라인 모드는 CSV와 같은 map 좌표계가 필요하고, 초기 yaw로 순방향 또는 역방향을 선택합니다.
RViz에서는 Fixed Frame을 `map`으로 두고 `/obca/local_grid`, `/obca/path`, `/obca/raceline`을 추가합니다.
`/obca/raceline`의 Durability Policy는 **Transient Local**로 설정합니다.

포함된 CSV는 특정 시뮬 `map.png`에서 생성했습니다. 다른 지도에 그대로 사용하지 마십시오.
[레이스라인 생성 방법과 지도 해시](src/obca_navigation/racelines/README.md)를 확인하십시오.
차용한 원본 알고리즘은 **최소 곡률 최적화**이며 전역 최소 랩타임을 보장하지 않습니다.
ICP에는 frozen map이나 loop closure가 없어 장시간 누적 오차가 글로벌 레이스라인과의 차이로
남을 수 있습니다. 현재는 저속 검증용이며 고속 레이싱 완성판이 아닙니다.

상세 설계·수식·파라미터·실패 원인·검증 범위는
[노드 문서](src/obca_navigation/docs/obca_navigation.md),
[검증 기록](docs/validation.md), [의존성 출처](THIRD_PARTY.md)를 참고하십시오.
