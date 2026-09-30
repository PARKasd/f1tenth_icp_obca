#!/usr/bin/env python3
"""localization_node의 파라미터 하나를 바꿔가며 같은 로그를 재생하고 결과를 비교한다.

왜 필요한가
-----------
실차 로그를 그대로 다시 흘려 넣어야 파라미터 변경의 효과를 같은 조건에서 잴 수 있다.
ROS 2 Humble에는 mcap 스토리지 플러그인이 없어 `ros2 bag play`를 쓸 수 없으므로,
이 스크립트가 직접 mcap을 읽어 `/clock`·`/tf_static`·`/scan`·`/odom`·
`/global_waypoints`·`/initialpose`를 재생하고 노드를 값별로 띄운다.
`/tf`(map->odom)는 노드가 스스로 발행하므로 재생하지 않는다.

지표를 파라미터 종류에 맞춰 골라야 한다
---------------------------------------
- **ICP 코어 안에서 작동하는 파라미터** (`lateral_regularization_floor_tau2`,
  `lateral_regularization_scale`, `max_range`, `source_voxel_size`,
  `max_num_iterations`, `voxel_size` 등): `residual_rms`·`iterations`·수렴률이
  그대로 유효한 지표다.
- **출력 스무딩 파라미터** (`smoothing_alpha_rot`, `smoothing_alpha`):
  `residual_rms`로는 판정할 수 없다. 스무딩은 출력 전용이고 필터 결과가 ICP
  코어로 되먹여지지 않아(`last_pose_ = new_pose`, KinematicICP.cpp:86),
  값을 바꿔도 residual이 전혀 움직이지 않는다(실측 확인). 그래서 이 경우에만
  `smoothing_alpha_rot = 1.0`(= 스무딩 없음, T_out = T_icp) 기준 실행을 추가로
  돌려 발행 yaw가 raw ICP에서 얼마나 뒤처지는지(`yaw lag`)를 잰다.
`yaw jerk p95`(발행 yaw의 2차 차분)는 두 경우 모두 지터 지표로 함께 본다.

사용 예
-------
    # 횡 정규화 하한 스윕 (ICP 코어 파라미터 → residual이 유효)
    python3 src/kinematic_localization/scripts/param_sweep.py \
        --bag ~/Downloads/run_20260821_021/run_20260821_021805 \
        --param lateral_regularization_floor_tau2 --values 1 10 15 20 \
        --map-name map --start 334 --end 375

    # 출력 스무딩 yaw 게인 스윕 (자동으로 기준 실행이 추가된다)
    python3 src/kinematic_localization/scripts/param_sweep.py \
        --bag ... --param smoothing_alpha_rot --values 0.12 0.2 0.3 0.5
"""
from __future__ import annotations

import argparse
import glob
import math
import os
import subprocess
import sys
import time
from pathlib import Path

REPLAY_TOPICS = ("/scan", "/odom", "/global_waypoints", "/initialpose", "/tf_static")


def find_mcap(bag: Path) -> Path:
    if bag.is_file():
        return bag
    hits = sorted(glob.glob(str(bag / "*.mcap")))
    if not hits:
        sys.exit(f"[error] mcap 파일을 찾지 못했다: {bag}")
    return Path(hits[0])


def msg_type_for(schema_name: str):
    """'sensor_msgs/msg/LaserScan' -> 실제 파이썬 메시지 클래스."""
    pkg, kind, name = schema_name.split("/")
    mod = __import__(f"{pkg}.{kind}", fromlist=[name])
    return getattr(mod, name)


def load_messages(mcap_path: Path, start: float, end: float):
    """(rel_time, topic, ros_msg) 목록을 돌려준다.

    mcap_ros2의 동적 디코더는 publish에 쓸 수 없는 임시 클래스를 만든다. 백에 담긴
    것은 CDR 그대로이므로 rclpy의 deserialize_message로 실제 타입에 직접 넣는다.
    손상된 백도 읽은 데까지 쓰도록 스트리밍한다.
    """
    from mcap.records import Channel, Message, Schema
    from mcap.stream_reader import StreamReader
    from rclpy.serialization import deserialize_message

    chans, schemas, types = {}, {}, {}
    raw: list[tuple[float, str, object]] = []
    t0 = None

    with open(mcap_path, "rb") as f:
        try:
            for rec in StreamReader(f).records:
                if isinstance(rec, Schema):
                    schemas[rec.id] = rec
                elif isinstance(rec, Channel):
                    chans[rec.id] = rec
                elif isinstance(rec, Message):
                    ch = chans.get(rec.channel_id)
                    if ch is None or ch.topic not in REPLAY_TOPICS:
                        continue
                    t = rec.log_time / 1e9
                    if t0 is None:
                        t0 = t
                    rel = t - t0
                    # tf_static은 창 밖이어도 반드시 필요하다(라이다 외부파라미터).
                    if ch.topic != "/tf_static" and not (start <= rel <= end):
                        continue
                    if ch.id not in types:
                        types[ch.id] = msg_type_for(schemas[ch.schema_id].name)
                    raw.append((rel, ch.topic, deserialize_message(rec.data, types[ch.id])))
        except Exception as exc:  # 손상 백은 읽은 데까지 쓴다
            print(f"[warn] mcap 조기 종료({type(exc).__name__}) — {len(raw)}건까지 사용", file=sys.stderr)
    if not raw:
        sys.exit("[error] 재생할 메시지가 없다. --start/--end 범위를 확인하라.")
    raw.sort(key=lambda r: r[0])
    return raw


def load_kissmap(map_name: str):
    """map_name(.kissmap)의 맵 점을 (N,2)로 읽는다. 정렬 지표 계산에 쓴다."""
    import struct
    import numpy as np
    path = Path(map_name) if map_name.startswith("/") else None
    if path is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory("kinematic_localization")) / "maps" / f"{map_name}.kissmap"
        except Exception:
            path = Path("src/kinematic_localization/maps") / f"{map_name}.kissmap"
    if not path.is_file():
        print(f"[warn] kissmap을 찾지 못했다: {path} — 정렬 지표는 생략된다", file=sys.stderr)
        return None
    with open(path, "rb") as f:
        f.read(8)
        struct.unpack("<dd", f.read(16))
        (n,) = struct.unpack("<Q", f.read(8))
        pts = np.frombuffer(f.read(n * 24), dtype="<f8").reshape(n, 3)
    return pts[:, :2]


def alignment_stats(map_xy, scans, poses, laser_x: float, laser_y: float):
    """발행 포즈로 스캔을 맵에 투영해 최근접 맵점까지의 거리 분포를 낸다.

    `residual_rms`는 τ 안에 든 대응점만 평균하므로 τ를 줄이면 기계적으로 작아진다
    (순환 논리). 이 지표는 **모든** 스캔점을 세므로 τ에 무관하다 — τ 스윕에서
    "정말 잘 맞게 됐는가"를 판정할 수 있는 유일한 값이다.
    """
    import numpy as np
    if map_xy is None or not scans or not poses:
        return (float("nan"),) * 5
    try:
        from scipy.spatial import cKDTree
    except ImportError:
        print("[warn] scipy 없음 — 정렬 지표 생략", file=sys.stderr)
        return (float("nan"),) * 5
    tree = cKDTree(map_xy)
    st = [s[0] for s in scans]
    all_d = []
    corr_hi = []          # 요레이트 > 0.8 rad/s 프레임의 (위치보정 m, yaw보정 deg)
    wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi
    yaw_rate = {}
    for i in range(1, len(poses)):
        dt = poses[i][0] - poses[i - 1][0]
        if dt > 0:
            yaw_rate[poses[i][0]] = abs(wrap(poses[i][3] - poses[i - 1][3]) / dt)
    for t, x, y, yaw in poses:
        i = min(range(len(st)), key=lambda k: abs(st[k] - t)) if st else None
        if i is None or abs(st[i] - t) > 0.05:
            continue
        sc = scans[i][1]
        ang = np.arange(len(sc.ranges)) * sc.angle_increment + sc.angle_min
        rr = np.asarray(sc.ranges, dtype=float)
        ok = np.isfinite(rr) & (rr > 0.1) & (rr < 12.0)
        if not ok.any():
            continue
        lx = rr[ok] * np.cos(ang[ok]) + laser_x
        ly = rr[ok] * np.sin(ang[ok]) + laser_y
        c, s_ = math.cos(yaw), math.sin(yaw)
        pts = np.stack([x + c * lx - s_ * ly, y + s_ * lx + c * ly], 1)
        all_d.append(tree.query(pts)[0])
        # 코너 프레임만: 발행 포즈에서 '맵 최적 정합'까지 남은 강체 보정을 잰다.
        # trimmed(80%) 점-대-점 ICP — 이상치가 회전을 끌지 않게.
        if yaw_rate.get(t, 0.0) <= 0.8:
            continue
        P = pts.copy()
        C = np.array([x, y])
        dth = 0.0
        for _ in range(25):
            dist, idx = tree.query(P)
            keep = dist <= np.quantile(dist, 0.8)
            A, B = P[keep], map_xy[idx[keep]]
            ca, cb = A.mean(0), B.mean(0)
            U, _S, Vt = np.linalg.svd((A - ca).T @ (B - cb))
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[-1] *= -1
                R = Vt.T @ U.T
            tr = cb - R @ ca
            P = (R @ P.T).T + tr
            C = R @ C + tr
            step = math.atan2(R[1, 0], R[0, 0])
            dth += step
            if abs(step) < 1e-5 and np.linalg.norm(tr) < 1e-4:
                break
        corr_hi.append((float(np.hypot(*(C - np.array([x, y])))), abs(math.degrees(dth))))
    if not all_d:
        return (float("nan"),) * 5
    a = np.concatenate(all_d)
    hi = [c for c in corr_hi if c is not None]
    yaw_hi = float(np.quantile([c[1] for c in hi], 0.5)) if hi else float("nan")
    pos_hi = float(np.quantile([c[0] for c in hi], 0.5)) if hi else float("nan")
    return (float(np.quantile(a, 0.5)), float(np.quantile(a, 0.9)),
            float(100.0 * (a > 0.3).mean()), pos_hi, yaw_hi)


def stop_process_group(proc: subprocess.Popen) -> None:
    """`ros2 run` 래퍼와 그 자식 노드를 한꺼번에 정리한다."""
    import signal
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        return
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass


def kill_stray_nodes() -> None:
    """남아 있는 localization_node를 모두 정리한다(이중 진단 수집 방지)."""
    subprocess.run(["pkill", "-9", "-f", "kinematic_localization/localization_node"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    time.sleep(0.5)


def run_one(param: str, value: float, msgs, args) -> dict:
    """localization_node를 한 번 띄우고 재생한 뒤 진단 통계를 돌려준다."""
    import rclpy
    from builtin_interfaces.msg import Time as TimeMsg
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from rosgraph_msgs.msg import Clock

    # 이전 실행의 잔재가 남아 있으면 진단이 이중으로 수집되어 비교가 무의미해진다.
    kill_stray_nodes()
    # `ros2 run`은 래퍼 프로세스라 terminate해도 자식 노드가 살아남는다.
    # 새 세션으로 띄우고 프로세스 그룹째 죽인다.
    proc = subprocess.Popen(
        ["ros2", "run", "kinematic_localization", "localization_node", "--ros-args",
         "--params-file", args.params,
         "-p", "use_sim_time:=true",
         "-p", f"map_name:={args.map_name}",
         "-p", f"{param}:={value}",
         "-p", "watchdog_enable:=false",    # 재생 중 스캔 공백에 개입하지 않게
         *[a for kv in (args.set or []) for a in ("-p", kv)]],
        stdout=subprocess.DEVNULL if args.quiet else None,
        stderr=subprocess.DEVNULL if args.quiet else None,
        start_new_session=True,
    )
    if not rclpy.ok():
        rclpy.init()
    node = Node("param_sweep_player")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])

    latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    pubs = {}
    from f110_msgs.msg import WpntArray
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    pubs["/scan"] = node.create_publisher(LaserScan, "/scan", 10)
    pubs["/odom"] = node.create_publisher(Odometry, "/odom", 20)
    pubs["/global_waypoints"] = node.create_publisher(WpntArray, "/global_waypoints", latched)
    pubs["/initialpose"] = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    pubs["/tf_static"] = node.create_publisher(TFMessage, "/tf_static", latched)
    clock_pub = node.create_publisher(Clock, "/clock", 10)

    diags: list[dict] = []
    poses: list = []
    played_scans: list = []   # (stamp, LaserScan) — τ 무관 정렬 지표용

    def on_diag(msg):
        for st in msg.status:
            d = {kv.key: kv.value for kv in st.values}
            d["_msg"] = st.message
            diags.append(d)

    node.create_subscription(DiagnosticArray, "/kinematic_localization/diagnostics", on_diag, 50)

    def on_pose(msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        poses.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                      msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))

    node.create_subscription(Odometry, "/pf/pose/odom", on_pose, 50)

    def publish_clock(rel: float):
        c = Clock()
        c.clock = TimeMsg(sec=int(rel), nanosec=int((rel % 1.0) * 1e9))
        clock_pub.publish(c)

    # 노드 기동 전에 재생을 시작하면 앞부분 스캔을 통째로 놓쳐 비교가 무의미해진다.
    # 노드가 /scan을 구독할 때까지(= 프로세스가 살아 그래프에 들어올 때까지) 기다린다.
    t_first = min(rel for rel, topic, _ in msgs if topic != "/tf_static")
    static_msgs = [m for _, topic, m in msgs if topic == "/tf_static"]
    deadline = time.time() + args.startup_timeout
    ready = False
    while time.time() < deadline:
        publish_clock(t_first)
        for m in static_msgs:
            pubs["/tf_static"].publish(m)
        rclpy.spin_once(node, timeout_sec=0.05)
        if pubs["/scan"].get_subscription_count() > 0:
            ready = True
            break
    if not ready:
        print(f"[warn] {param}={value}: 노드가 {args.startup_timeout:.0f}s 안에 /scan을 "
              f"구독하지 않았다 — 이 행은 신뢰할 수 없다", file=sys.stderr)
    # 구독 성립 후 잠깐 더 안정화(map 로드/latched 토픽 전달).
    for _ in range(30):
        publish_clock(t_first)
        rclpy.spin_once(node, timeout_sec=0.02)

    wall0 = time.time()
    for rel, topic, m in msgs:
        if topic == "/tf_static":
            continue
        target = wall0 + (rel - t_first) / args.rate
        while True:
            dt = target - time.time()
            if dt <= 0:
                break
            publish_clock(rel)
            rclpy.spin_once(node, timeout_sec=min(dt, 0.002))
        publish_clock(rel)
        pubs[topic].publish(m)
        if topic == "/scan":
            played_scans.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m))
        rclpy.spin_once(node, timeout_sec=0.0)
    for _ in range(50):  # 마지막 프레임 처리 대기
        rclpy.spin_once(node, timeout_sec=0.02)

    node.destroy_node()
    stop_process_group(proc)
    kill_stray_nodes()

    if not poses:
        print("[warn] /pf/pose/odom을 한 건도 못 받았다 — yaw jerk는 산출 불가", file=sys.stderr)
    align = alignment_stats(args._map_xy, played_scans, poses, args._laser_x, args._laser_y)
    out = summarize(value, diags, poses)
    (out["align_p50"], out["align_p90"], out["align_over30"],
     out["pos_hi"], out["yaw_hi"]) = align
    return out


def fmt_value(v) -> str:
    """표 출력용. 숫자면 3자리, 아니면 문자열 그대로."""
    try:
        return f"{float(v):.3f}"
    except (TypeError, ValueError):
        return str(v)


def summarize(value, diags, poses) -> dict:
    def col(key):
        out = []
        for d in diags:
            v = d.get(key)
            if v in (None, "", "nan"):
                continue
            try:
                out.append(float(v))
            except ValueError:
                pass
        return out

    def med(v):
        return sorted(v)[len(v) // 2] if v else float("nan")

    def pct(v, p):
        return sorted(v)[int(p * (len(v) - 1))] if v else float("nan")

    moving = [d for d in diags if float(d.get("speed", 0) or 0) > 0.3]
    res = col("residual_rms")
    inl = col("inlier_ratio")
    res_mv = [float(d["residual_rms"]) for d in moving if d.get("residual_rms")]
    it = col("iterations")

    # yaw jerk: 출력 yaw의 2차 차분 RMS. 지터가 늘면 커진다.
    wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi
    jerk = []
    for i in range(2, len(poses)):
        dt1 = poses[i - 1][0] - poses[i - 2][0]
        dt2 = poses[i][0] - poses[i - 1][0]
        if dt1 <= 0 or dt2 <= 0:
            continue
        w1 = wrap(poses[i - 1][3] - poses[i - 2][3]) / dt1
        w2 = wrap(poses[i][3] - poses[i - 1][3]) / dt2
        jerk.append(abs(w2 - w1) / dt2)
    return {
        "poses": poses,
        "value": value,
        "frames": len(diags),
        "moving": len(moving),
        "res_p50": med(res),
        "res_p95": pct(res, 0.95),
        "res_mv_p50": med(res_mv),
        "inlier_p50": med(inl),
        "it_p50": med(it),
        "it30_pct": 100.0 * sum(1 for x in it if x >= 30) / len(it) if it else float("nan"),
        "unconv_pct": 100.0 * sum(1 for d in diags if d.get("_msg") != "ok") / len(diags) if diags else float("nan"),
        "yaw_jerk_p95": pct(jerk, 0.95),
    }


def yaw_lag_vs(ref_poses, poses):
    """기준(raw ICP) 궤적 대비 발행 yaw가 뒤처진 각도의 (p50, p95)를 도 단위로 돌려준다."""
    import bisect
    wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi
    if not ref_poses or not poses:
        return (float("nan"), float("nan"))
    rt = [p[0] for p in ref_poses]
    diffs = []
    for t, _x, _y, y in poses:
        i = bisect.bisect_left(rt, t)
        if i <= 0 or i >= len(rt):
            continue
        t0_, _, _, y0 = ref_poses[i - 1]
        t1_, _, _, y1 = ref_poses[i]
        span = t1_ - t0_
        yr = y0 if span <= 0 else y0 + (t - t0_) / span * wrap(y1 - y0)
        diffs.append(abs(wrap(y - yr)))
    if not diffs:
        return (float("nan"), float("nan"))
    diffs.sort()
    return (math.degrees(diffs[len(diffs) // 2]),
            math.degrees(diffs[int(0.95 * (len(diffs) - 1))]))


def main() -> int:
    ap = argparse.ArgumentParser(description="localization_node 파라미터 스윕 재생 비교")
    ap.add_argument("--bag", required=True, type=Path, help="mcap 파일 또는 백 디렉토리")
    ap.add_argument("--param", default="smoothing_alpha_rot", help="스윕할 노드 파라미터 이름")
    ap.add_argument("--values", nargs="+", required=True,
                    help="시험할 값들. bool 파라미터에는 true/false를 쓴다 "
                         "(0/1은 rcl이 bool로 받지 않는다)")
    ap.add_argument("--map-name", default="map", help="localization_node의 map_name (.kissmap)")
    ap.add_argument("--params", default="src/kinematic_localization/config/kinematic_localization.yaml")
    ap.add_argument("--start", type=float, default=0.0, help="백 시작 기준 초")
    ap.add_argument("--end", type=float, default=1e9, help="백 시작 기준 초")
    ap.add_argument("--rate", type=float, default=1.0, help="재생 배속")
    ap.add_argument("--startup-timeout", type=float, default=30.0,
                    help="노드가 /scan 구독을 시작할 때까지 기다릴 최대 초")
    ap.add_argument("--set", action="append", metavar="NAME:=VALUE",
                    help="모든 실행에 동일하게 적용할 고정 파라미터 (반복 가능). "
                         "예: --set use_adaptive_threshold:=false")
    ap.add_argument("--quiet", action="store_true", help="노드 로그 숨김")
    args = ap.parse_args()

    smoothing = args.param.startswith("smoothing_")
    mcap_path = find_mcap(args.bag)
    print(f"백: {mcap_path}")
    print(f"파라미터: {args.param} = {args.values}")
    if args.set: print(f"고정 파라미터: {args.set}")
    print(f"구간: {args.start}~{args.end}s  배속 {args.rate}x  맵 {args.map_name}")
    msgs = load_messages(mcap_path, args.start, args.end)
    body = [m for m in msgs if m[1] != "/tf_static"]
    span = body[-1][0] - body[0][0] if body else 0.0
    n_scan = sum(1 for _, t, _ in msgs if t == "/scan")
    print(f"재생 대상 {len(msgs)}건 (scan {n_scan}건, {span:.1f}s)\n")

    # 정렬 지표 준비: 맵 점과 라이다 외부파라미터(base_link -> laser)
    args._map_xy = load_kissmap(args.map_name)
    args._laser_x, args._laser_y = 0.0, 0.0
    for _, topic, m in msgs:
        if topic == "/tf_static":
            for tr in m.transforms:
                if "laser" in tr.child_frame_id or "lidar" in tr.child_frame_id:
                    args._laser_x = tr.transform.translation.x
                    args._laser_y = tr.transform.translation.y
    print(f"정렬 지표: 맵점 {0 if args._map_xy is None else len(args._map_xy)}개, "
          f"laser 외부파라미터 ({args._laser_x:.3f}, {args._laser_y:.3f})\n")

    ref = None
    if smoothing:
        # 스무딩 파라미터는 residual로 판정할 수 없다 → raw ICP 기준 실행을 만든다.
        print("── 기준 실행 (smoothing_alpha_rot = 1.0, 스무딩 없음) …", flush=True)
        ref = run_one("smoothing_alpha_rot", 1.0, msgs, args)

    rows = []
    for v in args.values:
        print(f"── {args.param} = {v} 재생 중 …", flush=True)
        r = run_one(args.param, v, msgs, args)
        r["yaw_lag_deg"] = yaw_lag_vs(ref["poses"], r["poses"]) if ref else (float("nan"),) * 2
        rows.append(r)

    print("\n" + "=" * 104)
    if smoothing:
        print(f"{args.param:>20s}{'주행중':>8s}{'res p50':>9s}"
              f"{'yaw lag p50':>13s}{'yaw lag p95':>13s}"
              f"{'정렬 p50':>10s}{'정렬 p90':>10s}{'정렬>0.3m':>11s}{'yaw jerk':>10s}")
        for r in rows:
            print(f"{fmt_value(r['value']):>20s}{r['moving']:8d}{r['res_p50']:9.3f}"
                  f"{r['yaw_lag_deg'][0]:12.3f}°{r['yaw_lag_deg'][1]:12.3f}°"
                  f"{r['align_p50']:10.3f}{r['align_p90']:10.3f}{r['align_over30']:10.1f}%"
                  f"{r['yaw_jerk_p95']:10.2f}")
        print("=" * 104)
        print("yaw lag : raw ICP 대비 발행 yaw가 뒤처진 각도. 작을수록 스캔이 빨리 따라온다.")
        print("res     : 스무딩은 출력 전용이라 이 파라미터에 반응하지 않는다(참고용).")
    else:
        print(f"{args.param:>18s}{'주행중':>7s}{'inlier':>8s}{'res p50':>9s}{'it p50':>8s}{'it=30':>7s}"
              f"{'비정상':>7s}{'정렬 p50':>10s}{'정렬>0.3m':>11s}"
              f"{'코너 위치보정':>14s}{'코너 yaw보정':>14s}{'yaw jerk':>10s}")
        for r in rows:
            print(f"{fmt_value(r['value']):>18s}{r['moving']:7d}{r['inlier_p50']:8.3f}{r['res_mv_p50']:9.3f}"
                  f"{r['it_p50']:8.1f}{r['it30_pct']:6.1f}%{r['unconv_pct']:6.1f}%"
                  f"{r['align_p50']:10.3f}{r['align_over30']:10.1f}%"
                  f"{r['pos_hi']*100:13.1f}cm{r['yaw_hi']:13.2f}°{r['yaw_jerk_p95']:10.2f}")
        print("=" * 104)
        print("코너 보정 = 요레이트>0.8 rad/s 프레임에서 발행 포즈→맵 최적정합까지 남은 강체 보정(중앙값).")
        print("⚠️ res/inlier는 τ 스윕에서 공정한 지표가 아니다 — τ를 줄이면 가까운 점만 대응점이 되어")
        print("   기계적으로 좋아진다(순환). **정렬 p50/p90/>0.3m** 이 τ에 무관한 판정 지표다:")
        print("   발행 포즈로 스캔 전체를 맵에 투영해 최근접 맵점까지의 거리를 잰 값이다.")
        print("   맵점 간격이 0.10 m이므로 완벽하면 0.05 m 수준이 된다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
