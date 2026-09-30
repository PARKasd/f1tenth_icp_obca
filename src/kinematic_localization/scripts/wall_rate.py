#!/usr/bin/env python3
"""Wall alignment metric: fraction of scan points landing on occupied cells.

Each scan of the scan bag is projected into the map frame with the nearest (in
time) estimated pose from the pose bag (/pf/pose/odom), then the fraction of
points within +/-10 cm of an occupied cell is reported per time interval.

Usage (source /opt/ros/jazzy/setup.bash first):
  wall_rate.py --scan-bag run_0803_210100 --pose-bag kicp_out \
      --map map.yaml --intervals 0,11,20,30,45,52
"""
import argparse
import os

import numpy as np
import yaml
from PIL import Image


def load_wall_mask(map_yaml_path, dilation_m=0.1):
    """Occupied cells dilated by +/- dilation_m, plus grid geometry."""
    with open(map_yaml_path) as f:
        info = yaml.safe_load(f)
    image_path = info["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(map_yaml_path), image_path)
    resolution = float(info["resolution"])
    origin = info["origin"]
    occupied_thresh = float(info.get("occupied_thresh", 0.65))
    negate = int(info.get("negate", 0))

    img = np.asarray(Image.open(image_path).convert("L"), dtype=np.float64) / 255.0
    occ_prob = img if negate else (1.0 - img)
    mask = occ_prob >= occupied_thresh
    # Binary dilation with a (2n+1)^2 square structuring element
    n = int(np.ceil(dilation_m / resolution))
    dilated = mask.copy()
    for dy in range(-n, n + 1):
        for dx in range(-n, n + 1):
            if dy == 0 and dx == 0:
                continue
            dilated |= np.roll(np.roll(mask, dy, axis=0), dx, axis=1)
    # np.roll wraps around; clear the wrapped borders (they are never walls)
    dilated[:n, :] = dilated[-n:, :] = False
    dilated[:, :n] = dilated[:, -n:] = False
    return dilated, resolution, origin


def yaw_of(q):
    return np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_bag(uri, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    # Detect the storage id from metadata (sqlite3 / mcap)
    storage_id = "sqlite3"
    meta_path = os.path.join(uri, "metadata.yaml")
    if os.path.isfile(meta_path):
        with open(meta_path) as f:
            meta = yaml.safe_load(f)
        storage_id = meta["rosbag2_bagfile_information"]["storage_identifier"]

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    while reader.has_next():
        topic, data, _ = reader.read_next()
        yield topic, deserialize_message(data, get_message(types[topic]))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scan-bag", required=True, help="bag with /scan and /tf_static")
    parser.add_argument("--pose-bag", required=True, help="bag with /pf/pose/odom")
    parser.add_argument("--map", required=True, help="map_server yaml (pgm/png)")
    parser.add_argument("--intervals", default="0,11,20,30,45,52",
                        help="comma separated interval boundaries in seconds (from 1st scan)")
    parser.add_argument("--min-range", type=float, default=0.1)
    parser.add_argument("--max-range", type=float, default=30.0)
    args = parser.parse_args()

    mask, res, origin = load_wall_mask(args.map)
    height, width = mask.shape
    bounds = [float(v) for v in args.intervals.split(",")]

    # Static laser -> base extrinsic (planar: translation + yaw)
    ext_x = ext_y = ext_yaw = 0.0
    scans = []
    for topic, msg in read_bag(args.scan_bag, ["/scan", "/tf_static"]):
        if topic == "/tf_static":
            for t in msg.transforms:
                if t.child_frame_id in ("laser", "laser_frame"):
                    ext_x = t.transform.translation.x
                    ext_y = t.transform.translation.y
                    ext_yaw = yaw_of(t.transform.rotation)
        else:
            scans.append(msg)
    print(f"extrinsic base->laser: x={ext_x:.3f} y={ext_y:.3f} yaw={ext_yaw:.3f}, "
          f"{len(scans)} scans")

    # Estimated poses
    pt, px, py, pyaw = [], [], [], []
    for _, msg in read_bag(args.pose_bag, ["/pf/pose/odom"]):
        pt.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        px.append(msg.pose.pose.position.x)
        py.append(msg.pose.pose.position.y)
        pyaw.append(yaw_of(msg.pose.pose.orientation))
    pt, px, py, pyaw = map(np.asarray, (pt, px, py, pyaw))
    order = np.argsort(pt)
    pt, px, py, pyaw = pt[order], px[order], py[order], pyaw[order]
    if len(pt) < 2:
        raise SystemExit("no poses found")
    hz = (len(pt) - 1) / (pt[-1] - pt[0])
    print(f"poses: {len(pt)} over {pt[-1] - pt[0]:.1f}s -> mean rate {hz:.1f} Hz")

    t0 = scans[0].header.stamp.sec + scans[0].header.stamp.nanosec * 1e-9
    hits = np.zeros(len(bounds) - 1)
    totals = np.zeros(len(bounds) - 1)
    skipped = 0
    ce, se = np.cos(ext_yaw), np.sin(ext_yaw)
    for scan in scans:
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        rel = stamp - t0
        iv = np.searchsorted(bounds, rel, side="right") - 1
        if not (0 <= iv < len(hits)):
            continue
        i = np.searchsorted(pt, stamp)
        i = np.clip(i, 1, len(pt) - 1)
        if abs(pt[i] - stamp) > abs(pt[i - 1] - stamp):
            i -= 1
        if abs(pt[i] - stamp) > 0.05:
            skipped += 1
            continue
        ranges = np.asarray(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges >= args.min_range) & (ranges <= args.max_range)
        r, a = ranges[valid], angles[valid]
        # laser -> base
        lx, ly = r * np.cos(a), r * np.sin(a)
        bx = ce * lx - se * ly + ext_x
        by = se * lx + ce * ly + ext_y
        # base -> map
        c, s = np.cos(pyaw[i]), np.sin(pyaw[i])
        mx = c * bx - s * by + px[i]
        my = s * bx + c * by + py[i]
        cols = ((mx - origin[0]) / res).astype(int)
        rows = height - 1 - ((my - origin[1]) / res).astype(int)
        inside = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
        totals[iv] += inside.sum()
        hits[iv] += mask[rows[inside], cols[inside]].sum()

    print(f"skipped scans (no pose within 50ms): {skipped}")
    print(f"{'interval [s]':>14} | {'wall rate':>9} | {'points':>8}")
    for i in range(len(hits)):
        rate = 100.0 * hits[i] / totals[i] if totals[i] else 0.0
        print(f"{bounds[i]:>7.0f}-{bounds[i + 1]:<6.0f} | {rate:>8.1f}% | {int(totals[i]):>8}")
    rate = 100.0 * hits.sum() / totals.sum() if totals.sum() else 0.0
    print(f"{'overall':>14} | {rate:>8.1f}% | {int(totals.sum()):>8}")


if __name__ == "__main__":
    main()
