# 제안·구현 기록: d_ratio 기반 레이스라인 추출기

- 작성일: 2026-08-13 (외부 리뷰 반영 갱신: 2026-08-13)
- 대상 브랜치: `jazzy_main` (이후 `adaptive_global`에서 확장 예정)
- 상태: **구현 완료** — §7 검증 결과 참고
- 관련 문서: `src/global_planning/docs/proposal_offline_generator_bridge_cleanup.md` (선행 정리 기록)

## 0. 요구사항 (사용자 정의)

- 공통 전처리(지도 → free-space → skeleton → centerline → 평활·재샘플링)와
  공통 후처리(재샘플링 → 평활화 → 직선화 → 곡률 스파이크 완화 → 곡률 계산 →
  속도 프로파일 → 파일 출력)는 **그대로 유지**한다.
- `d_ratio ∈ [-1.0, +1.0]` 파라미터를 새로 만든다.
- 각 센터라인 포인트의 `d_left`/`d_right` 값으로 횡방향 critical 지점을 정한다:
  - `d_ratio = +0.3` → 센터라인에서 **d_right 방향으로** 가용 폭의 30% 지점
  - `d_ratio = -0.8` → 센터라인에서 **d_left 방향으로** 가용 폭의 80% 지점
  - `d_ratio = 0.0` → 센터라인 그 자체
- 각 센터라인 점에서 위 규칙으로 포인트를 추출한 뒤, 기존 공통 후처리에 그대로 태운다.

### 원 정의에서 의도적으로 벗어난 부분 (외부 리뷰 §2 수용)

사용자 원 정의는 "d_right의 30%"(원시 벽 거리 비율)였으나, 구현은
**가용 폭 `max(d_side − clearance, 0)`의 30%** 를 사용한다
(`clearance = safety_width/2 + boundary_margin`, mincurv 바운드와 동일한 식).
이유: 원시 비율로는 `±1.0`이 "벽 위의 점"이 되어 물리적으로 무의미해진다.
가용 폭 기준에서는 `±1.0` = "벽에서 안전 여유만 남긴 최대 오프셋"이고,
`d_side ≥ clearance`인 점은 이동 후에도 `d_new = (1−r)·d_side + r·clearance ≥ clearance`가
대수적으로 보장된다 (r ∈ [0,1], 스무딩 off·후처리 전 기준).

### ⚠️ 부호 규약 주의 (미래의 나에게)

이 정의는 **양수 = 오른쪽(d_right 방향)** 이다. 기존 mincurv의 α(양수 = 왼쪽,
`normals_from_heading()`이 왼쪽 법선을 반환)와 **반대**이고, Frenet `d`(양수 = 왼쪽)와도
반대다. 사용자 정의 스펙이므로 그대로 따르며, 구현부에 부호 반전 주석이 있다.
나중에 "부호가 이상하다"고 뒤집지 말 것. (`reverse` 시에는 `track_widths`가 뒤집힌
센터라인 기준으로 재계산되므로 "진행방향 기준 오른쪽" 규약이 자동 유지된다.)

## 1. 파이프라인 삽입 지점

`generate_trajectory()`의 흐름은 전처리 → **`optimize_raceline()` 디스패치 1곳** → 후처리로
분리되어 있으므로, 디스패치에 세 번째 분기(`d_ratio`)만 추가하면 전처리·후처리가 그대로
유지된다. `build_trajectory()`가 최종 라인 기준으로 `d_right`/`d_left`/κ/속도를 전부
재계산하므로 추출 시점의 폭 값이 출력에 잘못 남지 않는다.

```
전처리   load_map → cleanup_free_mask → skeletonize → extract_centerline_pixels
         → smooth_closed → filter_and_resample_closed(optimizer_step)
폭 계산  track_widths(center_xy) → (d_right, d_left)
디스패치 optimize_raceline(): centerline | d_ratio | mincurv     ← 분기 추가 지점
후처리   resample(waypoint_step) → smooth(raceline_smooth_sigma)
         → straighten → limit_curvature_spikes → build_trajectory
         → off-map / dense clearance / kappa 검증 → write_outputs
```

## 2. 오프셋 설계 (외부 리뷰 §4·§5 반영 최종안)

### 2.1 smooth-then-clip 구조

```python
def offset_by_d_ratio(center_xy, d_right, d_left, args):
    _, psi, _ = headings_and_curvature(center_xy)
    normals = normals_from_heading(psi)          # left normals
    clearance = args.safety_width * 0.5 + args.boundary_margin
    usable_right = np.maximum(d_right - clearance, 0.0)
    usable_left = np.maximum(d_left - clearance, 0.0)
    ratio = np.asarray(args.d_ratio)             # scalar today, per-point ready
    alpha = -ratio * np.where(ratio >= 0.0, usable_right, usable_left)
    if args.d_ratio_alpha_smooth_sigma > 0.0:
        alpha = gaussian_filter1d(alpha, sigma=..., mode="wrap")
        alpha = np.clip(alpha, -usable_right, usable_left)   # RAW corridor
    return center_xy + normals * alpha[:, None]
```

설계 결정과 근거:

1. **폭 배열을 스무딩하지 않는다 (초안 폐기, 리뷰 §4 수용)**: 가우시안은 양방향 평균이라
   좁은 구간의 폭을 부풀린다(예: 0.4 m 병목이 주변 1.0 m에 이끌려 0.7 m로 계산 →
   r=1.0에서 안전 한계를 0.3 m 초과 이동). 좁은 구간이 바로 안전이 걸린 지점이므로
   구조적 결함이었다.
2. **스무딩 대상은 결정 변수 α, 사영은 원시 폭 corridor**: clip은 |α|를 줄이기만 하므로
   **스무딩 오차가 항상 센터라인 쪽(안전한 방향)으로만 발생**한다. 이것이 폭 스무딩과의
   결정적 비대칭이고 이 구조를 채택한 진짜 근거다.
3. **스무딩 기본값 off (`d_ratio_alpha_smooth_sigma = 0.0`)**: hybrid 폭 모드가 이미
   2대 노이즈원(단일 픽셀 가짜 벽 → `min_wall_pixels=2`, 틈새 관통 → min-clearance
   floor)을 억제하고, 후처리(`raceline_smooth_sigma`·직선화·스파이크 완화)가 경로를
   다시 다듬는다. 스무딩+clip도 병목(바운드가 조여진 곳)에서는 킹크를 못 없애므로
   (clip이 도로 눌러버림), 이득은 바운드가 놀고 있는 구간의 잔떨림 완화로 제한적이다.
   실제 출력에서 잔떨림이 관측될 때만 켠다.
4. **스칼라 CLI, 배열 호환 내부**: `np.where` 브로드캐스팅으로 포인트별 ratio 배열을
   그대로 받는다. adaptive_global의 out-out-out 스케줄링(§6) 확장 시 이 함수는 수정이
   거의 없다. `if ratio >= 0` 같은 스칼라 전용 분기는 쓰지 않는다.
4b. **Fold guard (구현 중 실측으로 추가)**: 오프셋 곡선 미분 `q′ = (1−κα)t + α′n`에서
   `κ·α → 1`(국소 곡률 중심 방향으로 회전반경 이상 이동)이면 경로가 cusp/자가 루프로
   접힌다 — 리뷰 §13의 이론적 우려였는데, 실제 map.yaml 헤어핀에서 `d_ratio=-0.5`만으로
   루프가 발생했고 `boundary_margin` 증가로는 전혀 완화되지 않았다(벽 거리가 아니라
   곡률 문제이므로). 대응: 곡률 중심 방향 이동(`κ_smooth·α > 0`)에 한해 |α|를 국소
   회전반경의 90%로 캡. 스무딩된 κ(내부 σ=2, 직선화의 내부 σ=1 하드코딩과 같은 관례)를
   사용하고, 캡은 |α|를 줄이기만 하므로 corridor 바운드는 그대로 유지된다. 마지막
   단계(스무딩·clip 이후)에 적용한다.
5. **스무딩을 켜면 d_ratio의 의미가 바뀐다 (리뷰 §6)**: 각 점의 정확한 최종 비율이
   아니라 "스무딩 전 목표 비율"이 된다. 정확한 비율이 필요하면 스무딩을 끌 것
   (후처리도 경로를 움직이므로 최종 출력에서 완전히 정확한 비율은 어차피 불가).

### 2.2 안전성의 한계 — 무엇을 보장하고 무엇을 보장하지 않는가

**두 안전성을 구분해야 한다 (리뷰 §5 과장 정정 + §10):**

- **오프셋 시점 안전성 (보장)**: α는 항상 원시 폭 corridor `[-usable_right, +usable_left]`
  안에 있다. 스무딩 여부와 무관.
- **최종 출력 안전성 (보장 안 됨 → §2.3 검사로 확인)**: 공통 후처리(경로 스무딩·직선화·
  스파이크 완화·재샘플링)가 오프셋 이후 경로를 다시 움직인다. 특히 경로 스무딩은
  병목에서 clip으로 눌러놓은 지점을 다시 벽 쪽으로 되돌릴 수 있고, 직선화의 clearance
  기준(`safety_width/2 + straight_clearance_margin`)은 d_ratio의 기준
  (`safety_width/2 + boundary_margin`)과 **의도적으로 다르다** — 직선화 코드 주석 참고:
  boundary_margin(corridor 조형 노브)을 포함시키면 좁은 트랙에서 직선화가 통째로
  죽는다. margin 통일은 해법이 아니며, 최종 dense clearance 검사가 해법이다.

추가 non-goal (리뷰 §8): `d_side < clearance`인 점은 가용 폭 0이라 **이동하지 않는다**.
d_ratio는 "이미 안전한 센터라인을 벽 쪽으로 안전하게 이동"시킬 뿐, 안전하지 않은
센터라인을 복구하지 않는다. 참고: 현재 `gui_params.yaml` 저장값은
`safety_width 0.4 + boundary_margin 0.4` → clearance 0.6 m로 커서, 좁은 구간에서는
가용 폭이 0이 되어 센터라인에 머무는 점이 있을 수 있다(정상 동작).

### 2.3 최종 dense clearance 검사 (리뷰 §11 수용, 신규)

`count_off_map_waypoints()`는 densified 경로가 free 픽셀 위인지만 검사한다 —
"차 반폭이 들어가는가"는 별개 문제다. 최종 레이스라인을 ~2 px 간격으로 densify한 뒤
distance transform으로 벽까지 최소 거리를 측정해:

- `min wall clearance` 값을 항상 출력하고,
- `safety_width/2`(물리적 통과 하한) 미달 샘플 수를 WARN으로 출력한다.

위반 기준을 `+boundary_margin`까지로 잡지 않은 이유: boundary_margin은 corridor 조형
노브(현 저장값 0.4 m)라 그 기준으로는 정상 라인도 대량 WARN이 된다. corridor 목표치는
INFO 줄에 같이 출력해 사용자가 직접 비교한다. 이 검사는 optimizer 종류와 무관하게
모든 실행에서 돈다.

### 2.4 알려진 한계 (리뷰 §9·§13)

- `d_left`/`d_right`는 센터라인 법선 방향 raycast 값이다: 유클리드 최근접 벽 거리,
  waypoint 사이 선분 clearance, 차량 길이 footprint와는 다르다. §2.3 dense 검사가
  유클리드 clearance를 보완한다.
- 오프셋 곡선의 cusp/자가 교차는 v1 검증 중 실제로 관측되어 **fold guard로 원천
  차단했다**(§2.1 결정 4b). 잔여 안전망: 기존 `--max-curvature` 검증 WARN + off-map
  검사 + §2.3 dense 검사 + `debug_overlay.png` 육안 확인. 큰 |ratio|에서 벽에 바싹
  붙는 라인은 후처리(선분·스무딩)가 half-width 미만으로 파고들 수 있고 §2.3 WARN이
  이를 알려준다 — 완화는 `boundary_margin` 증가 + `--d-ratio-alpha-smooth-sigma` 활성
  (실측: -0.5 기준 0.000 m → 두 개 조합 시 0.140 m).
- width_mode는 `hybrid` 권장. `distance` 모드는 `d_left == d_right`(최근접 벽 거리
  복사)라 "방향별 가용 폭"의 의미가 사라지고 보수적 오프셋이 된다.

## 3. 구현된 수정 목록

| # | 파일 | 내용 |
|---|---|---|
| 1 | `generate_global_trajectory.py` | `--optimizer` choices에 `d_ratio` 추가, help 갱신 |
| 2 | 〃 | `--d-ratio`(기본 0.0), `--d-ratio-alpha-smooth-sigma`(기본 0.0=off) 인자 추가 |
| 3 | 〃 `validate_args` | `-1 ≤ d_ratio ≤ 1`, `sigma ≥ 0` 검증 |
| 4 | 〃 | `offset_by_d_ratio()` 신규 (§2.1) |
| 5 | 〃 `optimize_raceline` | `d_ratio` 분기 추가 |
| 5b | 〃 `offset_by_d_ratio` | fold guard (§2.1 결정 4b) |
| 6 | 〃 | `report_min_clearance()` 신규 + `generate_trajectory()`에서 호출·출력 (§2.3) |
| 6b | 〃 `count_off_map_waypoints` | 잠복 버그 수정: `cumulative_s`가 n+1 길이(선두 0.0 포함)를 반환하는데 boolean 마스크(n)로 인덱싱해 off-map 히트가 있을 때만 IndexError — `s_dense[:-1][off]`로 수정 (d_ratio 음수 검증 중 발견) |
| 7 | `trajectory_gui.py` | NumericSpec `d_ratio`(−1.0~1.0, step 0.05)·`d_ratio_alpha_smooth_sigma`(0~10, step 0.5), group "D-ratio" |
| 8 | 〃 | Optimizer 탭 그룹 `("Min-curvature", "D-ratio", "Straightening")` |
| 9 | 〃 | 옵티마이저 콤보·sanitize 목록에 `d_ratio` 추가 |
| 10 | 〃 `make_namespace` | 두 값 float 변환 추가 |
| 11 | `gui_params.yaml` | `d_ratio: 0.0`, `d_ratio_alpha_smooth_sigma: 0.0` 추가 |
| 12 | `AGENTS.md` | 옵티마이저 3종·부호 규약·smooth-then-clip 불변식 기록 |
| 13 | `README.md` / `README_en.md` | d_ratio 옵티마이저 설명·CLI 예시 추가 |

## 4. 외부(Codex) 리뷰 검증 기록

| 리뷰 절 | 판정 | 처리 |
|---|---|---|
| §1 수식 재구성 | 정확 | — |
| §2 "d_right의 30%" 문구 | 수용 | §0에 의도적 스펙 변경으로 명시 |
| §3 clearance 증명 | 정확 | §0에 요지 반영 |
| §4 폭 스무딩 결함 | 수용 (초안 결함) | §2.1 결정 1로 폐기 |
| §5 smooth-then-clip | 구조 수용, 주장 정정 | "안전경계를 넘지 못한다"는 오프셋 시점 한정(§2.2), `if ratio>=0` 스칼라 전용 분기는 np.where로 대체, 기본값은 off(리뷰는 on 전제) |
| §6 스무딩 시 의미 변화 | 수용 | §2.1 결정 5 |
| §7 클립 킹크 | 수용 | §2.1 결정 3 근거의 일부. 제약 QP는 v1 제외(리뷰도 동의) |
| §8 비안전 centerline 비복구 | 구조 수용 | §2.2 non-goal. 단 "사전 측정 0.55 m" 수치는 출처 불명으로 채택 안 함(gui 저장값 0.4/0.4→clearance 0.6은 검증됨) |
| §9 폭 측정의 한계 | 수용 | §2.4 |
| §10 직선화 margin 불일치 | 수용+뉘앙스 | 의도된 설계(코드 주석 존재) — margin 통일 대신 §2.3 검사로 대응 |
| §11 off-map 불충분 | 수용 | §2.3 신규 검사 |
| §12 width mode 표 | 정확 | §2.4 |
| §13 cusp | 수용 | §2.4 (전용 검사는 보류) |
| §14 검증 계획 보완 | 수용 | §7 갱신 (셸 표기·출력 디렉터리 분리·부호 내적 검사 등) |

## 5. (참고) 왜 mincurv 파라미터로는 out-out-out이 안 되는가

mincurv 비용의 세 항 중 곡률 항은 out-in-out(apex 클리핑), 길이 항은 in-in-in,
스무딩 항은 진폭 축소(센터라인 방향)를 선호한다 — "바깥쪽" 힘을 만드는 항이 없어
파라미터 조합으로는 "레이싱 라인 ↔ 센터라인" 사이만 오갈 수 있다. d_ratio는 이 한계를
우회하는 별도 로직이다.

## 6. adaptive_global 확장 방향 (이번 구현 범위 아님)

스칼라 d_ratio는 "항상 한쪽" 라인만 만든다. out-out-out(코너 진입·apex·탈출 모두
바깥쪽)은 센터라인 κ 부호로 포인트별 ratio를 스케줄링하면 된다 — 좌코너(κ>0)는
바깥=오른쪽이므로 `+|r|`, 우코너는 `-|r|`, 직선은 0으로 블렌딩. §2.1 함수가 배열
호환이므로 스케줄러(κ 스무딩 + 부호 → ratio 배열) 한 함수만 추가하면 된다. 이 확장은
사용자가 adaptive_global 브랜치에서 직접 진행한다.

## 7. 검증 계획·결과

각 ratio는 **별도 명령으로** 실행하고 출력 디렉터리를 분리한다(기본 출력 디렉터리는
맵 이름에서 파생되므로 같은 곳을 덮어쓴다):

```bash
for R in 0.0 0.5 -0.5 1.0 -1.0; do
  python3 offline_trajectory_generator/generate_global_trajectory.py \
    --map-yaml src/monte_carlo_localization/maps/map.yaml \
    --optimizer d_ratio --d-ratio $R --width-mode hybrid --debug-image \
    --output-dir /tmp/dr_test/ratio_$R
done
```

체크리스트 및 결과 (2026-08-13, map.yaml + hybrid, CLI 기본 파라미터):

1. ✅ `d_ratio=0.0` 결과가 `--optimizer centerline`과 waypoint 좌표 일치 (max |Δxy| < 1e-9)
2. ✅ 부호 방향: ±0.5·±1.0 전부, 이동 벡터와 왼쪽 법선 내적 부호가 스펙과 일치
3. ✅ 모든 |α| ≤ 해당 방향 `usable` (스무딩 on(σ=3)에서도 raw corridor 유지)
4. ✅ `usable = 0`인 점은 이동량 0
5. ⚠️ off-map WARN은 전 ratio에서 0건. dense min clearance는 0.0→0.55 m,
   +0.5→0.35 m, +1.0→0.15 m, ±0.5·−1.0→0.07 m — 큰 |ratio|에서는 후처리가
   half-width(0.17 m) 미만으로 파고드는 구간이 남고 §2.3 WARN이 출력됨(§2.4 완화
   레시피 참고). fold guard 이전에는 −0.5에서 자가 루프 + clearance 0.000 m였음.
6. ✅ `mincurv`·`centerline` 회귀 완주
7. ✅ GUI 헤드리스 검증: `default_gui_values`/`make_namespace`/`SPEC_BY_KEY` 슬라이더
   그리드(−1.0~1.0, 0.05 스냅)/`gui_params.yaml` 라운드트립/metadata.json 직렬화 통과.
   실제 tk 화면 조작(콤보 선택→Rebuild)은 사용자 확인 필요
8. ⬜ 산출 JSON을 터미널 3(`global_planning.launch.py`)으로 로딩, RViz 마커 확인 (젯슨)

검증 스크립트: 세션 스크래치 `dr_test/verify_d_ratio.py` (일회성, 저장소 미포함)
