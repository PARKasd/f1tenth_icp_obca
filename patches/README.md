# f1sim_C 센서 각도 수정

1. `PARKasd/f1sim_C`의 `ros2_jazzy` 브랜치 `047f550`에서 C++ ray caster는
   `FOV/(num_beams-1)`을 쓰지만 gym bridge는 `FOV/num_beams`를 발행했습니다.
   같은 scan 데이터의 각도가 달라 ICP 입력 형상이 왜곡됩니다.
2. 해당 버전이면 시뮬을 종료하고 아래 패치를 적용합니다. 이미 수정된 버전이면 건너뜁니다.
   센서/지도 설정의 다른 로컬 변경은 유지하십시오.

```bash
cd ~/Desktop/f1sim_C
git apply --check ~/Desktop/f1tenth_icp_obca/patches/f1sim_C-laserscan-angle.patch
git apply ~/Desktop/f1tenth_icp_obca/patches/f1sim_C-laserscan-angle.patch
colcon build --symlink-install --packages-select f1tenth_gym_ros
source install/setup.zsh
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

3. `angle_min + (len(ranges)-1)*angle_increment`와 `angle_max`가 부동소수점 오차 내에서
   일치하는지 확인합니다. 1080빔/FOV 4.7 rad이면 수정 전 차이는 약 -0.00435 rad입니다.
4. 이 패치는 외부 시뮬레이터용입니다. 실제 LiDAR 드라이버에 적용하지 않습니다.
   원격 검증 PC에는 적용했으며 외부 시뮬 저장소의 변경은 별도 커밋/푸시하지 않았습니다.
