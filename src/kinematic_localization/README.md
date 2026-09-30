# kinematic_localization

Kinematic-ICP 기반 맵 localization 패키지. MCL(`particle_filter_cpp`)과 동일한 인터페이스
(`/pf/pose/odom` + `map -> odom` TF)를 제공하는 drop-in 대체 localization입니다.

강건화 기능 내장: NaN 가드(항상 on — `/initialpose` 직후 abort 버그 수정),
스캔 갭 워치독(odom dead reckoning), `~/diagnostics` 정합 품질 발행,
맵 기반 포즈 유효성 검사, 마할라노비스 게이트(기본 off), 횡방향 소프트
제약(기본 off). 맵 로드 실패는 FATAL 즉시 종료, `/map`은 주기 재발행됩니다.

자세한 내용은 [docs/kinematic_localization.md](docs/kinematic_localization.md) 참고.

## 빠른 시작

```bash
# 빌드
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to kinematic_localization

# occupancy 맵 -> 동결 맵 변환 (최초 1회)
ros2 run kinematic_localization pgm_to_kissmap.py <map.yaml> \
    src/kinematic_localization/maps/<map_name>.kissmap \
    --voxel-size 1.0 --max-range 30 --downsample 0.1

# localization 실행
ros2 launch kinematic_localization kinematic_localization.launch.py map_name:=<map_name>
# RViz 2D Pose Estimate 또는 /initialpose 1회 발행으로 초기화

# 오프라인 매핑 (bag -> .kissmap)
ros2 launch kinematic_localization mapping.launch.py \
    bag_path:=<bag_dir> output_path:=map.kissmap

# 온라인 SLAM (주행하면서 맵 생성, 초기 포즈 불필요)
ros2 launch kinematic_localization kinematic_localization.launch.py \
    slam_mode:=true map_output_file:=slam_map.kissmap
# 저장: ros2 service call /kinematic_localization/save_map std_srvs/srv/Trigger
#       또는 Ctrl-C (자동 저장). 생성된 .kissmap은 maps/에 넣고 localization에 사용.
```
