# 검증 기록

최신 검증을 위에 기록합니다. 아래 초기 Windows 검증의 미수행 항목은 당시의 상태이며,
이후 원격 ROS/gym 검증 결과와 구분해서 읽으십시오.

## 레이스라인 기준 연속주행 (2026-10-01)

1. 원본 `offline_trajectory_generator`를 차용해 현재 시뮬 지도에서 최소 곡률 레이스라인을
   생성했습니다. 472점, 폐곡선 길이 43.358 m, 최대 절대 곡률 1.149128 rad/m,
   생성기 기준 최소 벽 거리 0.584 m입니다. 생성 옵션과 지도 해시는 `racelines`에 기록했습니다.
2. C++ CSV 입력·초기 방향 선택·랩 경계의 누적 진행 거리·속도 프로파일 제한을 추가했습니다.
   Windows 코어 검사에서 CSV, 두 랩 투영, 역방향 선택, 진행 창 밖 점프 거부,
   0.2 m/s 속도 프로파일, 현재 상태의 안전 여유 회귀 검사와 기존 검사가 통과했습니다.
   Python launch 검사 8개도 통과했습니다.
3. 실제 Jazzy에서 노드별 YAML 값이 뒤의 wildcard launch override보다 우선하는 문제를
   확인하고, launch에서 노드별 최종 사전을 합쳐 전달하도록 수정했습니다.
4. 외부 `f1sim_C`의 C++ ray caster와 ROS LaserScan의 angle_increment 불일치를 수정했습니다.
   1080빔에서 각도 끝점 오차는 수정 전 약 -0.00435 rad, 수정 후 2.01e-7 rad였습니다.
   [외부 시뮬 패치](../patches/README.md)를 함께 제공합니다. 원격 시뮬 저장소의 사용자
   지도·config 변경은 유지했고 외부 저장소에는 커밋/푸시하지 않았습니다.
5. 원격 Jazzy/gym에서 `(6.505364, -0.146211, 0.054081 rad)`로 초기화하고 `/drive`를
   연결했습니다. 최대 속도 0.8 m/s, ICP voxel 0.25 m/source voxel 0.1 m,
   frozen map·loop closure 없이 **약 108 m, 완전한 두 바퀴 이후 세 번째 바퀴 일부**를 주행했습니다.
   1초 상태 샘플의 누적 raceline 진행 값은 19.546 → 128.470 m였습니다.
6. 이 실행에서 출발 경로 11회, 일반 OBCA 경로 1379회가 유효했습니다. 상태에 기록된
   전체 solve 시간은 5.46~99.66 ms였습니다. 1초마다 채집한 유효 상태의 solve 시간은
   중앙값 36.09 ms, p95 56.56 ms입니다. 최악 실행시간 보장은 아닙니다.
7. **무정지 반복 완주는 아직 보장하지 않습니다.** 중간의 미관측 공간 검증 실패로 짧은
   정지가 있었고, 약 140초/108 m 이후 세 번째 코너에서 `swept footprint ... step 2`가
   지속돼 정지했습니다. 1초 위치 샘플 기준 ICP와 gym 실제 위치 차이는 최대 약 0.273 m였습니다.
   절대 좌표 보정 없는 누적 드리프트와 미관측 영역을 최적화 후 검사하는 구조가 남은 한계입니다.
   ICP voxel을 더 작은 0.1 m로 바꾼 비교 실행은 약 64 m에서 정지해 기본값으로 채택하지 않았습니다.
8. 검증 종료 시 속도 0을 발행하고 테스트용 navigation을 종료했습니다. 안전 검사,
   150 ms solver 예산, 300 ms 입력 watchdog, 미관측 공간 차단을 해제하지 않았습니다.

주행 로그는 원격 `/tmp/obca-raceline-angle-fixed025-result.json`과
`/tmp/obca-raceline-angle-fixed025-trace.jsonl`에 남겼습니다. 같은 지도와 위 초기 pose로
README의 시뮬 launch를 실행하고 `/obca/status`, `/pf/pose/odom`, `/ego_racecar/odom`을
기록하면 진행 거리·계산시간·위치 차이를 재확인할 수 있습니다. 실차 고속 검증은 수행하지 않았습니다.

## 출발 직진 제약 및 계산량 개선 (2026-10-01)

1. 초기 pose 후 기본 0.3 m 전진 구간에 조향 0·고정 yaw 제약을 추가했습니다. 직진 reference와 최종 차량 swept footprint 모두 관측 지도로 검증하며, 미관측 영역을 free로 간주하지 않습니다.
2. 속도·가감속·종단 정지로 시점별 도달 범위를 계산해 도달 불가능한 장애물–시점 제약만 제외했습니다. 직진 모드는 고정 방향과 관측된 직선 길이를 추가로 활용합니다. 전체 지도 충돌 검증과 기존 장애물 개수 한도는 유지했습니다.
3. Ipopt의 limited-memory Hessian을 해석적 희소 Lagrangian Hessian으로 교체했습니다. 이전 성공 궤적의 조향도 초기값으로 사용합니다. 목적함수 gradient, Jacobian, Hessian 수치 미분 오차가 1e-6 미만임을 검사했습니다.
4. 원격 PC의 저장 지도 재생에서 출발 모드는 33개 제약 쌍·364개 변수로 약 14 ms, 일반 모드는 같은 전환 지도에서 약 175 ms → 36 ms였습니다. 원래 초기 문제의 408쌍·약 450 ms와 출발 모드는 제약 내용도 달라 직접적인 동일 문제 성능 비교는 아닙니다.
5. Windows 코어 검사에서 복도 약 5 ms, 장애물 우회 약 8 ms, 24단계 코너 최대 약 15 ms였습니다. 24개 방향의 270도 스캔 출발, 미관측 공간·전방 장애물 차단, 장애물 한도·도달 가능성 검사도 통과했습니다.
6. 원격 gym에서 실제 `/drive`를 연결해 20초씩 검사했습니다. 약 40도 초기 방향에서는 출발 경로 12회·일반 OBCA 경로 9회가 유효했습니다. 같은 위치에서 yaw=0으로 시작한 검사에서는 출발 11회·일반 72회가 유효했고, 초기 `(6.491, -0.213)`에서 `(7.929, 3.953)`까지 이동했습니다. 최대 명령 속도는 0.8 m/s였습니다.
7. **연속 완주 검증은 아닙니다.** 두 검사 모두 이후 시간 초과·불가능한 최적화 등의 정지가 남았습니다. yaw=0 검사에서 실패 시도를 포함한 계산시간은 약 6~186 ms였습니다. 150 ms 예산 초과 결과는 계속 폐기하며, 성공 재생의 14/36 ms 수치를 최악 실행시간 보장으로 해석하면 안 됩니다.

진단 파일은 원격 `/tmp/obca-driving-result.json`, `/tmp/obca-driving-result-heading40.json`, `/tmp/obca-startup-navigation.log`에 보관했습니다. 테스트 종료 시 속도 0을 발행하고 테스트용 navigation을 종료했습니다.

## 원격 ROS 2 Jazzy / gym 확인 (2026-10-01)

- 실행 중인 gym에서 기존 `reference missing or obstacle budget exceeded`를 재현했습니다. ICP 정합률 1.0, 수렴 true, 잔차 약 0.060으로 위치추정은 정상이었습니다.
- 격자 원점에서 벗어난 270도 LiDAR 스캔으로 reference 초기화 실패를 재현했습니다. 검증된 짧은 직진 연결 구간으로 탐색을 재시도하도록 수정한 후 24개 방향의 회귀 검사가 통과했습니다.
- 원격 Ubuntu에서 `cb --packages-select obca_navigation --cmake-args -DCMAKE_BUILD_TYPE=Release` 빌드 및 `colcon test --packages-select obca_navigation`의 C++ 코어·launch 검사가 통과했습니다.
- 같은 시뮬 위치에서 수정 후 reference 생성은 통과했습니다. 캡처 지도에서 reference 21점, 장애물 24개로 장애물 한도 64개 이내였습니다.
- **주행 성공은 확인하지 못했습니다.** 실제 지도 재생에서 최적화는 약 450 ms로 기본 150 ms 제한을 초과했습니다. 진단 목적으로만 제한을 늘려 얻은 해도 step 2에서 미관측 영역에 차량 swept footprint가 걸려 거부됐습니다. 출발 직후 조향에 따른 후방 사각지대 제약과 계산시간 개선이 남아 있습니다.
- 원격 진단에서는 출력을 `/obca/debug_drive`로 분리했고 종료 후 진단용 navigation을 정리했습니다. 운영 timeout·미관측 영역 충돌 검사는 완화하지 않았습니다.

## 2026-10-01 시뮬 시간 설정 수정

- 일반 gym bridge에 맞춰 시뮬 기본 `use_sim_time=false`로 수정했습니다.
- launch 검사 7개 통과: 모든 노드의 기본 시간 설정과 명시적 `use_sim_time=true` 재정의를 확인했습니다.
- Windows 독립 C++ 코어 빌드 및 CTest 통과. 24단계 코너 검증 최대 solve 시간은 약 88 ms였습니다.
- planner 정지 상태에 시간 오류, 입력 지연, ICP 품질 수치를 구분했습니다. 품질·입력 만료 차단 조건은 유지합니다.
- 이 환경에는 ROS 2가 없어 변경한 ROS 노드의 빌드·gym 실행은 검증하지 못했습니다. 코어 검사는 ROS 노드 검증을 대신하지 않습니다.

## 2026-10-01 환경별 launch 추가 검증

`navigation_real.launch.py`, `navigation_sim.launch.py`와 각 YAML 프로필을 추가했습니다.
Python 3.11.9와 PyYAML 6.0.3으로 launch 배선·파라미터 적용 순서를 검사한 7개 테스트가
모두 통과했습니다. 검사 대상은 Python/YAML 문법, 환경별 파일 선택, 실차·시뮬의
시간·프레임·출력 토픽, 공통 launch의 관찰용 출력 유지, CLI 덮어쓰기, 누락 파일과
잘못된 boolean의 실패 처리입니다.

이 테스트는 ROS launch 클래스의 가벼운 대역을 사용해 실제 launch 함수와 YAML을
평가합니다. 실제 ROS 노드 기동·DDS·TF·차량 주행 검증을 대신하지 않습니다.
ROS 테스트 환경에서는 `colcon test --packages-select obca_navigation`에 함께 등록됩니다.
ROS 없이 실행하려면 PyYAML 설치 후 다음 명령을 사용합니다.

```bash
python3 src/obca_navigation/test/test_launch_profiles.py -v
```

이번 변경은 launch·설정·문서에 한정되며 아래 C++ core 결과는 이전 구현 검증 기록입니다.

## 1. 실제 수행한 검증

Windows x64에서 임시 도구를 사용했습니다. 도구는 저장소에 포함하지 않았습니다.

- Git for Windows MinGit 2.56.0: 독립 저장소 `main` 초기화.
- w64devkit GCC 16.2.0 / CMake 4.4.3: ROS 비의존 C++17 core 컴파일.
- CasADi Windows wheel에 포함된 Ipopt 3.14.19 C API 라이브러리: 최적화를 실제 실행.
  다운로드 wheel은 PyPI 메타데이터의 SHA-256과 대조했습니다.
- CMake `OBCA_STANDALONE=ON` 구성, 빌드, CTest 실행 성공.

초기 핵심 테스트 실행 결과:

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

## 3. 초기 Windows 환경에서 수행하지 못한 검증

초기 로컬 PC에는 ROS 2 Jazzy, ament, WSL Linux 배포판이 없었습니다. 아래 항목은 당시
완료되지 않았으며, 이후 원격 환경에서 수행한 항목은 문서 위쪽에 별도로 기록했습니다.

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
