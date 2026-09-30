# Provenance

- `src/kinematic_localization`: copied from the parent `2026_IFAC-transition_global` snapshot;
  package declares MIT. Original source, docs and license notices retained. This repository adds
  manual SLAM initialization and clears the accumulated map on reset. No parent git revision was
  available (the input directory had no `.git`).
- `src/f110_msgs`: parent snapshot, Apache-2.0; messages copied without changes.
- `third_party/kinematic-icp`: PRBonn Kinematic-ICP, MIT, with the parent's existing patches.
- `third_party/kiss-icp`: KISS-ICP v1.2.0 as vendored by the parent, MIT.
- `third_party/sophus`: Sophus 1.22.10 with the parent's patch, see its LICENSE.txt.
- `third_party/robin-map`: tsl::robin_map, MIT; see its LICENSE.
- Ipopt is an external system dependency; its license is not replaced by this repository's license.

New `obca_navigation` code is Apache-2.0. Its OBCA constraints are independently implemented
from Zhang, Liniger and Borrelli, *Optimization-Based Collision Avoidance*,
https://arxiv.org/abs/1711.03449. No GPL Julia implementation was copied from
https://github.com/XiaojingGeorgeZhang/OBCA.

The Windows validation tools and CasADi-distributed Ipopt binaries are temporary tools outside
the repository, not runtime dependencies or redistributable repository content.
