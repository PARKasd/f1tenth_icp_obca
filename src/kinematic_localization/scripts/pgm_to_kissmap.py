#!/usr/bin/env python3
"""Convert an occupancy grid map (map_server yaml + pgm/png) into a .kissmap.

Occupied cells (map_server convention, occ_prob >= occupied_thresh) are reduced
to their *surface* (occupied cells touching free space) — SLAM maps inflate
walls into multi-cell blobs, and aligning scans to blob centers biases the
estimate by ~half the blob thickness. Surface points are emitted at cell
centers, downsampled on a --downsample 2D grid (default 0.1 m: the 1 m voxel
map keeps up to max_points_per_voxel=20 points per voxel, and a wall line
sampled at 0.1 m puts only ~10 points in a 1 m voxel, so nothing is dropped —
a coarser sample leaves ~1.4 m gaps between wall points, which makes ICP snap
to wrong correspondences at speed).

.kissmap binary format (defined by kinematic_localization):
  8 bytes  magic "KISSMAP1"
  double   voxel_size  (metadata only: the node voxel size this map is for)
  double   max_range   (metadata only)
  uint64   point count
  double x, y, z  x count

Usage:
  pgm_to_kissmap.py map.yaml out.kissmap --voxel-size 1.0 --max-range 30 --downsample 0.1
"""
import argparse
import os
import struct

import numpy as np
import yaml
from PIL import Image


def load_occupied_cells(map_yaml_path):
    with open(map_yaml_path) as f:
        info = yaml.safe_load(f)
    image_path = info["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(map_yaml_path), image_path)
    resolution = float(info["resolution"])
    origin = info["origin"]  # [x, y, yaw] of the lower-left cell
    occupied_thresh = float(info.get("occupied_thresh", 0.65))
    negate = int(info.get("negate", 0))

    img = np.asarray(Image.open(image_path).convert("L"), dtype=np.float64) / 255.0
    # map_server convention: occ_prob = (255 - pixel)/255 for negate == 0
    occ_prob = img if negate else (1.0 - img)
    occupied = occ_prob >= occupied_thresh
    # Wall surface = occupied cells with at least one non-occupied 4-neighbor.
    # (Interior of inflated wall blobs is never seen by the lidar and biases
    # point-to-point ICP toward the blob center.)
    interior = (
        np.roll(occupied, 1, axis=0) & np.roll(occupied, -1, axis=0) &
        np.roll(occupied, 1, axis=1) & np.roll(occupied, -1, axis=1)
    )
    # np.roll wraps around; keep border cells as surface (conservative)
    interior[0, :] = interior[-1, :] = False
    interior[:, 0] = interior[:, -1] = False
    surface = occupied & ~interior
    return surface, resolution, origin


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("map_yaml", help="map_server yaml (references pgm/png)")
    parser.add_argument("output", help="output .kissmap path")
    parser.add_argument("--voxel-size", type=float, default=1.0,
                        help="kissmap header metadata (the node voxel size this map is for); "
                             "does NOT control point density")
    parser.add_argument("--max-range", type=float, default=30.0,
                        help="kissmap header metadata")
    parser.add_argument("--downsample", type=float, default=0.1,
                        help="2D grid step [m] for point downsampling (default 0.1)")
    args = parser.parse_args()

    occupied, resolution, origin = load_occupied_cells(args.map_yaml)
    rows, cols = np.nonzero(occupied)
    height = occupied.shape[0]
    # Cell centers in the map frame (pgm row 0 is the top, map y grows upward)
    xs = origin[0] + (cols + 0.5) * resolution
    ys = origin[1] + (height - 1 - rows + 0.5) * resolution
    zs = np.zeros_like(xs)

    # Downsample on a --downsample grid: keep one point per grid cell
    step = args.downsample
    keys = np.stack([np.floor(xs / step), np.floor(ys / step)], axis=1)
    _, unique_idx = np.unique(keys, axis=0, return_index=True)
    points = np.stack([xs[unique_idx], ys[unique_idx], zs[unique_idx]], axis=1)

    with open(args.output, "wb") as f:
        f.write(b"KISSMAP1")
        f.write(struct.pack("<ddQ", args.voxel_size, args.max_range, len(points)))
        f.write(points.astype("<f8").tobytes())
    print(f"{args.map_yaml}: {occupied.sum()} occupied cells -> {len(points)} points "
          f"-> {args.output} (downsample={args.downsample}, voxel_size={args.voxel_size}, "
          f"max_range={args.max_range})")


if __name__ == "__main__":
    main()
