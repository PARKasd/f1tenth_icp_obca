# 검증 기록

## 1. 실제 수행한 검증

Windows x64에서 임시 도구를 사용했습니다. 도구는 저장소에 포함하지 않았습니다.

- Git for Windows MinGit 2.56.0: 독립 저장소 `main` 초기화.
- w64devkit GCC 16.2.0 / CMake 4.4.3: ROS 비의존 C++17 core 컴파일.
- CasADi Windows wheel에 포함된 Ipopt 3.14.19 C API 라이브러리: 최적화를 실제 실행.
  다운로드 wheel은 PyPI 메타데이터의 SHA-256과 대조했습니다.
- CMake `OBCA_STANDALONE=ON` 구성, 빌드, CTest 실행 성공.

최종 핵심 테스트 실행 결과:

```text
straight: solved 11.053 ms
corridor: solved 39.961 ms
detour: solved 43.726 ms
24-step ideal-model corner: x=0.835588 y=2.93258 yaw=1.5708 peak_solve_ms=97.795
100% tests passed out of 1
Total Test time (real) = 3.46 sec
```

CTest 항목 1개 안에서 다음 시나리오를 검사합니다.

1. 목적함수 gradient와 전체 analytic Jacobian의 수치 미분 대조.
2. 직선 경로 생성과 종단 정지 속도.
3. 양쪽 벽 사이 OBCA 경로와 차체 비충돌.
4. 중앙 정적 장애물 우회 및 별도 swept footprint 검사.
5. 초기 차체 충돌 시 실패, 실패 해 미발행.
6. 장애물 수 초과 시 실패, 계산시간 초과 시 실패.
7. 미관측 영역 진입 차단, 지도 TTL 만료, 초기화 후 기존 지도 제거.
8. 얇은 벽을 가로지르는 두 상태 사이의 swept collision 검출.
9. 차체 사각지대 예외가 차체 바깥으로 확장되지 않는지 검사.
10. 270도 LiDAR와 앞쪽 센서 장착을 가정한 스캔으로 출발 경로 생성.
11. L자 통로에서 24회 연속 최적화·warm start·이상적인 차량 모델 진행.

기능 테스트의 solver 시간 예산은 느린 테스트 환경을 위해 5초입니다. 별도의 deadline
실패 테스트도 포함합니다. YAML 운영 예산은 0.15초입니다. 위 시간은 이 PC의 특정
합성 장면에서 측정한 값이며 Jetson 성능이나 최악 계산시간을 보장하지 않습니다.
24단계 테스트는 optimizer의 첫 구간을 모델대로 실행한 것이며 ROS Pure Pursuit 추종기의
폐루프 성능을 검증한 것은 아닙니다.

## 2. 재현

Linux에서는 ROS 없이 core만 빌드할 수 있습니다.

```bash
sudo apt install build-essential cmake pkg-config coinor-libipopt-dev
cmake -S src/obca_navigation -B build/core -DOBCA_STANDALONE=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/core -j2
ctest --test-dir build/core --output-on-failure -V
```

이번 Windows 환경의 임시 도구를 그대로 사용한다면:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/test_core_windows.ps1 `
  -ToolRoot "$env:TEMP/f1tenth-obca-tools"
```

스크립트는 이미 내려받은 도구를 사용하며 시스템 설치나 추가 다운로드를 하지 않습니다.

## 3. 아직 수행하지 못한 검증

현재 PC에는 ROS 2 Jazzy, ament, WSL Linux 배포판이 없습니다. 따라서 다음 항목은
완료됐다고 볼 수 없습니다.

실제 ROS 구성도 시도했으며 `find_package(ament_cmake)`에서
`ament_cmakeConfig.cmake`를 찾지 못해 중단됐습니다. standalone core 구성은 복구했습니다.

- ROS 전체 패키지 빌드, 메시지 생성, launch 실행.
- 복사·수정한 ICP 노드의 새 초기화 및 지도 리셋 동작.
- 실제 DDS 순서, TF 외부변환, 스캔 타임스탬프와 위치 보간.
- `/initialpose` 전 정지, stale 입력 정지, solver 정지 중 추종 watchdog의 ROS 동작.
- gym 폐루프 주행과 실차 추종·제동거리·실시간 성능.

Ubuntu 24.04 / ROS 2 Jazzy에서 README의 colcon 빌드·테스트 후, 센서와 gym을 먼저
띄우고 `/obca/drive`를 관찰해야 합니다. 초기화 전 속도 0, 초기화 후 경로 생성,
scan/ICP/planner 각각 중단 시 속도 0, 초기 위치 재지정 시 이전 경로 제거를 확인한 뒤
시뮬의 `/drive`로 연결하십시오.

프로토타입 구현과 core 실행 검증 상태이며, 차량 주행 검증이 완료된 릴리스는 아닙니다.
