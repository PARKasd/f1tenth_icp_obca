#!/usr/bin/env bash
# replay_check.sh — bag 재생 기준선 회귀 검사 (기능 토글 전부 off 상태에서
# 기존 궤적과 비트 단위 동일함을 확인하는 하네스).
#
# 사용: replay_check.sh <bag> <map_name> [baseline.tum]
#   baseline 미지정(또는 파일 없음): <bag>_kicp_baseline.tum 으로 기준선 저장
#   baseline 지정: 바이트 비교, 다르면 non-zero exit
#
# 초기 포즈: bag의 첫 /pf/pose/odom 메시지에서 추출. bag에 그 토픽이 없으면
#   INIT_POSE="x y z qx qy qz qw" 환경변수로 직접 지정.
#
# 알려진 한계: bag 재생 직후 첫 스캔은 /tf_static 도착 경합으로 드롭될 수
# 있어, 선두 1~2행 차이는 회귀가 아닐 수 있다 (ponytail: 레이스 창구,
# 정밀 비교가 필요하면 diff -u 로 내용 확인).
set -euo pipefail

BAG=${1:?usage: replay_check.sh <bag> <map_name> [baseline.tum]}
MAP_NAME=${2:?usage: replay_check.sh <bag> <map_name> [baseline.tum]}
BASE=${3:-"${BAG%/}_kicp_baseline.tum"}
OUT=$(mktemp /tmp/replay_check_XXXXXX.tum)
NODE_LOG=$(mktemp /tmp/replay_check_node_XXXXXX.log)
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

NODE_PID=""
REC_PID=""
cleanup() {
    # 노드는 프로세스 그룹째 종료 — ros2 run 래퍼만 죽이면 자식 노드가
    # 고아로 살아남아 다음 실행의 기록을 오염시킨다
    [[ -n $NODE_PID ]] && kill -- -"$NODE_PID" 2>/dev/null || true
    if [[ -n $REC_PID ]]; then
        kill "$REC_PID" 2>/dev/null || true
        wait "$REC_PID" 2>/dev/null || true   # SIGTERM → flush 보장
    fi
    [[ -n $NODE_PID ]] && wait "$NODE_PID" 2>/dev/null || true
}
trap cleanup EXIT

# 이전에 비정상 종료된 실행의 잔여 노드 정리 (있으면 이번 기록이 오염된다)
pkill -f "localization_node --ros-args" 2>/dev/null && sleep 1 || true

# 1) 초기 포즈 확보
INIT_POSE=${INIT_POSE:-}
if [[ -z $INIT_POSE ]]; then
    INIT_POSE=$(python3 - "$BAG" <<'EOF'
import glob, os, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

bag = sys.argv[1]
storage = "mcap" if glob.glob(os.path.join(bag, "*.mcap")) else "sqlite3"
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage),
            rosbag2_py.ConverterOptions("", ""))
topics = {t.name for t in reader.get_all_topics_and_types()}
if "/pf/pose/odom" not in topics:
    sys.exit(1)
reader.set_filter(rosbag2_py.StorageFilter(topics=["/pf/pose/odom"]))
_, data, _ = reader.read_next()
m = deserialize_message(data, Odometry)
p, q = m.pose.pose.position, m.pose.pose.orientation
print(f"{p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}")
EOF
    ) || { echo "ERROR: bag에 /pf/pose/odom이 없습니다. INIT_POSE=\"x y z qx qy qz qw\"로 지정하세요." >&2; exit 1; }
fi
read -r PX PY PZ QX QY QZ QW <<<"$INIT_POSE"

# 2) 노드 기동 (기능 토글 기본값 = 전부 off)
# max_num_threads:=1 강제: TBB 병렬 환산 순서가 실행마다 달라 멀티스레드 출력은
# 실행 간 수 m까지 비결정적으로 발산한다(실측). 회귀 검사는 단일 스레드 결정론
# 모드에서만 비트 단위 비교가 성립한다. launch를 거치지 않고 직접 띄우는 이유다.
# EXTRA_PARAMS 환경변수로 추가 -p 오버라이드 전달 가능 (예: 강건화 토글 off
# 상태의 기준선 비교 — EXTRA_PARAMS="-p watchdog_enable:=false ...")
YAML=$(ros2 pkg prefix kinematic_localization)/share/kinematic_localization/config/kinematic_localization.yaml
setsid ros2 run kinematic_localization localization_node --ros-args \
    --params-file "$YAML" \
    -p map_name:="$MAP_NAME" -p use_sim_time:=true -p max_num_threads:=1 \
    ${EXTRA_PARAMS:-} \
    >"$NODE_LOG" 2>&1 &
NODE_PID=$!

for _ in $(seq 1 40); do
    ros2 node list 2>/dev/null | grep -q /kinematic_localization && break
    sleep 0.5
done
ros2 node list 2>/dev/null | grep -q /kinematic_localization || {
    echo "ERROR: 노드 기동 실패. 로그: $NODE_LOG" >&2; exit 1; }

# 3) 초기 포즈 발행 (구독 매칭 대기)
ros2 topic pub --once -w 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: $PX, y: $PY, z: $PZ}, orientation: {x: $QX, y: $QY, z: $QZ, w: $QW}}}}" \
    >/dev/null

# 4) 포즈 기록 시작
python3 "$SCRIPT_DIR/record_pose_tum.py" /pf/pose/odom "$OUT" &
REC_PID=$!
sleep 1

# 5) bag 재생 (완료까지 블로킹)
# bag 안의 기록된 /pf/pose/odom(MCL)이 우리 노드 출력과 섞이지 않도록 리맵.
# --rate 0.5: 단일 스레드 executor에서 스캔 처리 중 뒤처진 odom이 OdomAt의
# 100 ms 가장자리 허용을 넘으면 그 프레임은 등록을 건너뛰는데(로그: "No wheel
# odometry available"), 이 레이스가 실행마다 다른 지점에서 터져 실측상 수 m
# 발산까지 갔다. 반속 재생으로 콜백 여유를 확보해 이 경합을 제거한다.
ros2 bag play "$BAG" --clock --rate 0.5 --remap /pf/pose/odom:=/mcl_pose/odom

# 6) 기록 마무리
kill "$REC_PID" 2>/dev/null || true
wait "$REC_PID" 2>/dev/null || true
REC_PID=""
N_POSES=$(wc -l <"$OUT")
echo "recorded $N_POSES poses -> $OUT"

# 7) 비교 또는 기준선 저장
if [[ -f $BASE ]]; then
    if cmp -s "$OUT" "$BASE"; then
        echo "OK: 기준선과 비트 단위로 동일 ($BASE)"
    else
        echo "FAIL: 기준선과 다릅니다. diff: diff -u $BASE $OUT" >&2
        exit 1
    fi
else
    cp "$OUT" "$BASE"
    echo "기준선 저장: $BASE ($N_POSES poses)"
fi
