# nhw_ifac 이식 노트 (2026-08-19)

`edge_test`(`e12c24c`)의 `kinematic_localization` + `third_party/{kinematic-icp,kiss-icp,sophus,robin-map}`를
이 워크스페이스(`nhw_ifac`)로 **외과적으로만** 가져왔다. `edge_test` 전체는 머지하지 않았다
(두 브랜치가 577파일 규모로 갈라져 있고, 그쪽은 `monte_carlo_localization`을 삭제한 상태다).

## 이 워크스페이스에서 달라진 점

| | edge_test | 여기 |
|---|---|---|
| `monte_carlo_localization` | 삭제됨 | **그대로 유지** — KICP는 drop-in 대안이고, 둘 중 하나만 띄운다 |
| `local_planning` 맵 출처 | `kinematic_localization/maps` | `particle_filter_cpp/maps` **그대로** (지금 동작하므로 안 건드림) |
| `maps/map.{png,yaml}` | origin `[-17.094,-1.460]` (다른 트랙) | 이 워크스페이스의 현 맵(`particle_filter_cpp/maps/map.*`, 08-18 16:06 = 현 레이스라인과 같은 프레임)으로 교체 |
| `maps/map.kissmap`, `ifac_track.kissmap` | 옛 트랙 | 위 occupancy 맵에서 `pgm_to_kissmap.py`로 **재생성** |
| `auto_init_from_waypoints` | 없음 (항상 `/initialpose` 필요) | **신설, 기본 on** — 아래 참고 |

⚠️ `maps/map_0818_track.kissmap`은 edge_test 것 그대로 두었지만 **이 워크스페이스 맵과 정합하지
않는다**(최근접 거리 중앙 0.215 m, 최적 시프트 dx −0.30/dy +0.15에서 0.105 m, 점의 23%가 맵
바깥). 다른 트랙이다 — 쓰지 말 것. 기본은 `map_name:=map`.

## 🔴 신설: `/global_waypoints` 자동 초기화 + 검증 게이트

**왜**: 이 스택의 MCL은 `auto_init_from_waypoints: true`로 스타트라인 자세에서 자동
초기화한다. edge_test의 KICP는 `/initialpose`를 사람이 매번 찍어야 해서, 그대로 가져오면
운영 절차가 바뀐다. MCL과 같은 규약(`/global_waypoints` 첫 웨이포인트, `psi_rad` = yaw)으로 맞췄다.

**그런데 자동 초기화만 넣으면 위험하다 — 실측으로 확인했다.** `run_0818_182531` 재생에서
스타트라인이 실제 차 위치와 4.7 m 떨어진 자세로 초기화하니 ICP가 **60초 내내 회복하지
못하고**(9~16 m 이탈) 그 틀린 포즈를 40 Hz로 **조용히 계속 발행**했다. 수렴 베이슨이
`voxel_size`(1.0 m) 수준이라 구조적으로 못 돌아온다.

그래서 초기화 직후 40프레임(≈1 s) **발행을 보류**하고 `residual_rms` 중앙값으로 판정한다:

| | 정상 초기화 | 4.7 m 오초기화 |
|---|---|---|
| `residual_rms` 중앙값(40프레임) | 0.19 (관측 최악 0.275) | **0.45 ~ 0.56** |

분리비 2.3배가 warmup 0~30프레임 어디서나 일정해 임계 **0.35**를 그 중간에 뒀다.
거부하면 미초기화로 되돌아가 `/initialpose`를 기다린다 = **포즈를 아예 발행하지 않는다**
→ 컨트롤러 odom 워치독(0.5 s)이 차를 세운다(fail-safe).
🔑 오탐의 대가는 "사람이 2D Pose Estimate를 한 번 찍는 것"(= 이식 전과 같은 절차)이고
미탐의 대가는 벽이다. **애매하면 거부하는 쪽**으로 잡았다.

- 수동 `/initialpose`에는 게이트를 걸지 않는다(사람이 보고 찍은 값이므로). 수동이 오면
  이후 `/global_waypoints` 재발행이 덮어쓰지 못한다.
- edge_test 거동으로 되돌리기: `-p auto_init_from_waypoints:=false`
  (게이트만 끄려면 `-p auto_init_validate_frames:=0`)

## 실측 검증 — `run_0818_182531`(229 s, MCL로 주행한 bag) 동일 조건 재생 A/B

같은 `/scan`·`/odom`으로 KICP를 돌려, 기록된 MCL 출력과 나란히 비교했다.
절대 기준은 스캔→맵 최근접 거리(잔차 1 m 초과는 상대차/이상치로 절단).

| | MCL (기록) | KICP (재생) |
|---|---|---|
| 스캔→맵 잔차 p50 | 22.7 mm | **21.2 mm** |
| p90 | 46.9 mm | **25.1 mm** |
| **p99** | 300.6 mm | **91.5 mm** |
| 사이클 포즈 점프 > 0.5 m | **6회** | **0회** |
| 발행 최대 공백 | **340 ms** (>100 ms 5회) | **50 ms** (>100 ms 0회) |
| 포즈 NaN | 0 | 0 |
| 발행률 | 49.8 Hz | 39.2 Hz (스캔률) |

🔑 **꼬리가 3배 좁고 파국적 점프가 0이다.** 팀원 보고("안정적")가 이 워크스페이스의
맵·트랙에서도 재현된다.

🔴 **단 평상시 지터는 KICP가 더 크다** (같은 bag, 같은 시계, 미디언 창 0.15~0.50 s에서 일관):

| Frenet s 사이클 지터 σ | MCL **13 mm** | KICP **19~22 mm** |
|---|---|---|

즉 "큰 점프를 막으려고 넣은 가드"는 근거가 약해졌지만, **"지터를 흡수하려고 넣은 필터·마진은
그대로 두어야 한다"**. ⚠️ `docs/worklog_2026-08-18_handoff.md` §5가 `sector_scale_blend`
제거 근거로 든 "KICP s 지터 σ 12.6 mm"는 이 재생에서 **MCL 쪽 값**과 일치한다 —
지우기 전에 재확인할 것.

## 빌드 / 실행

```bash
cd ~/2026_IFAC && source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-select kinematic_localization
source install/setup.zsh

# MCL 대신 이것을 띄운다 (둘 다 띄우면 /pf/pose/odom·map->odom TF가 충돌한다)
ros2 launch kinematic_localization kinematic_localization.launch.py map_name:=map
```

- `/map`은 이 노드가 직접 발행한다(별도 map_server 불필요). `local_planning`·
  `obstacle_detector`의 map_server는 `/map`이 아닌 자체 토픽으로 remap돼 있어 충돌 없음.
- `/pf/pose/odom`의 twist는 **휠 오도메트리 패스스루**(MCL 관례) → 컨트롤러
  `current_speed_` 의미가 안 바뀐다. 컨트롤러 쪽 수정 불필요.
- 맵을 새로 만들면 `.kissmap`도 같이 재생성할 것:
  `ros2 run kinematic_localization pgm_to_kissmap.py <map.yaml> src/kinematic_localization/maps/<name>.kissmap --voxel-size 1.0 --max-range 30 --downsample 0.1`
- 나중에 MCL을 지울 때 고칠 곳: `local_planning/launch/local_planning.launch.py:40`,
  `local_planning/package.xml`의 `particle_filter_cpp` → `kinematic_localization`
  (맵 파일은 이미 같은 것을 넣어 두었다).
