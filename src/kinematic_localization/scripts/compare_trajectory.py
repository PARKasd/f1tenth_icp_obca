#!/usr/bin/env python3
"""Kinematic-ICP TUM 궤적 vs bag의 /pf/pose/odom(MCL) 비교.

사용:
  python3 compare_trajectory.py extract   # bag에서 MCL TUM 생성
  python3 compare_trajectory.py compare <kicp_tum> <mcl_tum>
"""
import sys

import numpy as np

BAG = "/home/parkm/Downloads/rosbag2_2026_08_13-00_14_31-20260812T155821Z-1-001/rosbag2_2026_08_13-00_14_31"
MCL_TUM = "/home/parkm/2026_IFAC/third_party/kicp_ws/mcl_pose_tum.txt"


def extract():
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=BAG, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/pf/pose/odom"]))
    n = 0
    with open(MCL_TUM, "w") as f:
        while reader.has_next():
            _, data, t = reader.read_next()
            m = deserialize_message(data, Odometry)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            f.write(f"{stamp:.6f} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n")
            n += 1
    print(f"wrote {n} poses -> {MCL_TUM}")


def load_tum(path):
    rows = [l.split() for l in open(path) if l.strip()]
    a = np.array(rows, dtype=float)
    t = a[:, 0]
    pos = a[:, 1:4]
    yaw = np.arctan2(
        2 * (a[:, 7] * a[:, 6] + a[:, 4] * a[:, 5]),
        1 - 2 * (a[:, 5] ** 2 + a[:, 6] ** 2),
    )
    return t, pos, yaw


def align_se2(src_xy, dst_xy):
    """src를 dst에 맞추는 SE(2) (Umeyama 2D). R, t 반환."""
    mu_s, mu_d = src_xy.mean(0), dst_xy.mean(0)
    s, d = src_xy - mu_s, dst_xy - mu_d
    h = (d.T @ s) / len(s)
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1] *= -1
        r = vt.T @ u.T
    return r, mu_d - r @ mu_s


def compare(kicp_path, mcl_path):
    tk, pk, yk = load_tum(kicp_path)
    tm, pm, ym = load_tum(mcl_path)
    # MCL 포즈를 kicp 타임스탬프에 보간
    mx = np.interp(tk, tm, pm[:, 0])
    my = np.interp(tk, tm, pm[:, 1])
    myaw = np.interp(tk, tm, np.unwrap(ym))
    # SE(2) 정렬 후 절대 궤적 오차
    r, tr = align_se2(pk[:, :2], np.c_[mx, my])
    aligned = (r @ pk[:, :2].T).T + tr
    err = np.linalg.norm(aligned - np.c_[mx, my], axis=1)
    yaw_err = np.abs(
        (np.arctan2(*np.c_[np.sin(yk - myaw), np.cos(yk - myaw)].T)) * 180 / np.pi
    )
    dist = np.sum(np.linalg.norm(np.diff(np.c_[mx, my], axis=0), axis=1))
    dur = tk[-1] - tk[0]
    print(f"poses={len(tk)}  주행거리={dist:.1f} m  시간={dur:.1f} s")
    print(
        f"위치오차  RMSE={np.sqrt(np.mean(err**2)):.3f} m  "
        f"mean={err.mean():.3f}  p95={np.percentile(err, 95):.3f}  max={err.max():.3f}"
    )
    print(
        f"헤딩오차  mean={yaw_err.mean():.2f} deg  "
        f"p95={np.percentile(yaw_err, 95):.2f}  max={yaw_err.max():.2f}"
    )
    np.savetxt(
        kicp_path.replace(".txt", "_aligned_xy.txt"),
        np.c_[tk, aligned, err],
        fmt="%.6f",
        header="t x y err(m)",
    )


if __name__ == "__main__":
    if sys.argv[1] == "extract":
        extract()
    else:
        compare(sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else MCL_TUM)
