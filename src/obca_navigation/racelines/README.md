# 시뮬 레이스라인 생성

1. `map.csv`는 원본 F1TENTH 저장소에서 가져온 `offline_trajectory_generator`의
   `mincurv` 알고리즘으로 생성했습니다. 최소 곡률 기준이며 전역 최소 랩타임 해는 아닙니다.
2. 사용한 `map.png`의 SHA-256은 아래와 같습니다. 지도가 다르면 새로 생성하십시오.

   `3cc986b7b9b0901609e744a5476b103585c6e13087dd94f962b8b63ce0260bdc`

3. ROS 런타임과 별도로 Python 오프라인 의존성을 설치하고 저장소 루트에서 실행합니다.

```bash
sudo apt install python3-numpy python3-scipy python3-opencv python3-yaml
python3 offline_trajectory_generator/generate_global_trajectory.py \
  --map-yaml ~/Desktop/f1sim_C/f1tenth_gym_ros/maps/map.yaml \
  --output-dir /tmp/obca-raceline-map \
  --optimizer mincurv --safety-width 0.7 --boundary-margin 0.08 \
  --max-speed 0.8 --min-speed 0.15 --max-curvature 1.15 \
  --smooth-sigma 4 --raceline-smooth-sigma 12 \
  --no-straighten-straights --debug-image
```

4. 출력의 곡률·벽 여유 경고와 `debug_overlay.png`를 검사합니다. 포함된 결과는
   472개 waypoint, 길이 약 43.36 m, 생성기 기준 최소 벽 거리 0.584 m입니다.
   이는 기준점의 오프라인 검사이며 실제 차체의 주행 안전성 검증과 구분해야 합니다.
5. `global_waypoints.csv`를 절대 경로로 지정해 실행합니다.

```bash
ros2 launch obca_navigation navigation_sim.launch.py \
  raceline_file:=/tmp/obca-raceline-map/global_waypoints.csv
```

6. CSV를 저장소 기본값으로 교체할 때는 생성 옵션과 지도 해시도
   `map.metadata.json`에 함께 갱신하고 다시 빌드합니다. 실제 시작 pose는 CSV와 같은
   `map` 좌표계로 지정합니다. ICP는 이 지도 파일을 읽지 않습니다.
