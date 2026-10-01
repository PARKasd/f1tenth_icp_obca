# Ubuntu 22.04 / ROS 2 Humble

## 1. 브랜치 선택

- `humble`: Ubuntu 22.04, ROS 2 Humble, C++17.
- `main`: Ubuntu 24.04, ROS 2 Jazzy.

레이스라인·ICP·OBCA·Pure Pursuit 및 실차/시뮬 launch 구성은 같습니다.
Humble에서 새로 clone하고 빌드하십시오. Jazzy의 바이너리와 install overlay는 재사용하지 않습니다.

## 2. 설치 및 실행

1. [ROS 공식 설치 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)에
   따라 Humble을 설치합니다. `.zshrc`의 ROS source와 `cb`, `sc` 같은 alias도 확인합니다.
2. README의 `git clone --branch humble`, 의존성 설치, `rosdep`, `colcon build/test`를 실행합니다.
   Bash는 `setup.bash`, Zsh는 `setup.zsh`를 사용합니다.
3. gym 또는 실제 센서·odom·TF·차량 mux를 먼저 실행합니다. 외부 gym도 Humble 환경에서
   별도로 빌드해야 하며, 이 저장소가 시뮬레이터를 설치하지는 않습니다.
4. 시뮬은 `ros2 launch obca_navigation navigation_sim.launch.py`,
   실차는 `ros2 launch obca_navigation navigation_real.launch.py`를 실행합니다.
5. 같은 지도 좌표계의 `/initialpose`를 지정합니다. 시뮬 기본은 설치된 최소 곡률 레이스라인이며,
   지도 교체 시 CSV도 재생성해야 합니다. 지도 없는 모드는 `reference_mode:=local`입니다.

## 3. 검증

`.github/workflows/humble.yml`은 Ubuntu 22.04의 공식 `ros:humble-ros-base-jammy` 컨테이너에서
선언된 의존성을 설치하고 세 패키지를 모두 빌드·테스트합니다. 실제 실차/시뮬 launch를 실행해
세 C++ 노드의 생존, 파라미터 적용, 레이스라인 발행, 초기화 전 속도 0 출력을 검사합니다.
검사는 별도 DDS domain 117과 `/obca/test_drive`를 사용합니다.

이 검사는 외부 gym 완주나 실차 제동 성능을 검증하지 않습니다. `docs/validation.md`의
약 108 m/두 바퀴 주행 기록은 Jazzy 원격 환경의 결과이며 Humble 결과로 바꾸어 해석하지 않습니다.
누적 ICP 드리프트 및 세 번째 바퀴 정지 한계도 동일하게 남아 있습니다.
