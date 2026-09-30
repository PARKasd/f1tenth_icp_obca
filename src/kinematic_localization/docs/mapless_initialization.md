# 지도 없는 ICP 초기화

1. `obca_navigation/navigation.launch.py`는 `slam_mode=true`,
   `require_initial_pose=true`, `auto_init_from_waypoints=false`로 이 노드를 시작합니다.
2. `/initialpose`를 받기 전에는 스캔 정합과 위치 발행을 시작하지 않습니다.
3. 초기 위치의 yaw는 실제 차체 방향입니다. 원하는 방향으로 차를 돌리지 않고 yaw만
   바꾸면 센서 지도 전체의 좌표가 회전할 뿐, 차의 진행 방향이 물리적으로 바뀌지 않습니다.
4. 휠 odometry 예측과 최근 스캔의 ICP 정합으로 위치를 추정합니다. frozen map은 읽지 않습니다.
5. `/initialpose`를 다시 지정하면 ICP 로컬 맵, 누적 지도, 시간 동기화 상태를 비웁니다.
   차량을 정지한 상태에서 지정하십시오.
6. 지도 없는 launch는 `smoothing_enable=false`를 사용해 계획용 스캔 변환과 ICP 지도의
   좌표 일관성을 유지합니다. 기존 단독 localization launch 기본값은 유지합니다.

입력은 `/scan` (`sensor_msgs/LaserScan`), `/odom` (`nav_msgs/Odometry`),
`/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`)입니다.
출력은 `/pf/pose/odom` (`nav_msgs/Odometry`), `map → odom` TF,
`/kinematic_localization/diagnostics`와 SLAM 지도입니다.
전체 설정은 `config/kinematic_localization.yaml`, 실행 예시는
`ros2 launch obca_navigation navigation.launch.py`입니다.

이 모드에는 루프 클로저가 없습니다. ICP가 누적 오차를 완전히 제거하지는 않습니다.
