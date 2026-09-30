// §9 scan-end synchronisation (2026-08-22) — pure decision logic.
//
// Split out of localization_node.cpp so the parts that decide *which time* a
// scan is registered at, and *whether* it may be registered yet, can be tested
// without a ROS graph. The node keeps the SE3 interpolation itself (it needs
// Sophus and the live history); everything here is arithmetic on stamps.
//
// ── Why this exists ────────────────────────────────────────────────────────
// The registration prior and the published stamp must both sit at the deskew
// reference, which is the END of the LiDAR sweep. Two things stopped that from
// being true:
//
//  1. kinematic-icp's TimeStampHandler tries to auto-detect whether a scan is
//     begin- or end-stamped by comparing the per-point stamps against the
//     header stamp. laser_geometry hands it RELATIVE per-point stamps
//     (0 .. 0.018750 s) while the header is an ABSOLUTE epoch time, so the
//     comparison is ~1.8e9 and "stamped at the beginning" is *always* true.
//     Its end_stamp is therefore unconditionally header + sweep.
//
//  2. The node's own published stamp and its prior window disagreed: the pose
//     went out at KISS's end_stamp (header + sweep) while the odometry prior
//     ended at the raw header. One of the two had to be wrong by a sweep.
//
// 🔴 Which one — settled by registration A/B, not by reasoning (2026-08-22).
//    Replayed run_20260822_035120 with the localizer seeded from the bag's own
//    first pose, three arms, everything else identical:
//
//      arm                        yaw corr p95  >1 deg  >2 deg  residual p95
//      A  legacy (prior@header)       0.77 deg    182     29       0.567
//      B  sync, convention "end"      0.69 deg    165     28       0.567
//      C  sync, convention "begin"    0.40 deg     80     12       0.417
//
//    "begin" (scan_end = header + sweep) wins on every quality metric, which
//    means the deskew reference really is header + sweep and the header is the
//    time of the FIRST ray — the plain ROS LaserScan convention.
//
// ⚠️ An arrival-lag argument says the opposite and is WRONG here. On this car
//    receive - header is -2.3 ms for /scan (IQR 0.1 ms) against +1.1 ms for
//    /odom and +0.2 ms for /imu, which looks impossible for a begin-stamped
//    scan (it cannot be published before it is acquired). It only looks that
//    way because the urg_node stamp is not on the host clock: its device-clock
//    mapping carries a ~+21 ms bias. The lag test is therefore a *host-clock*
//    heuristic and must never override a registration measurement — that is
//    exactly the mistake this comment exists to stop someone repeating.
//
// The convention stays a parameter so a different LiDAR does not silently
// inherit this car's answer, and so the arms remain reproducible.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>

namespace kinematic_localization::sync {

// Active sweep duration [s]. A 270 deg head at 40 Hz spends 18.75 ms of its
// 25 ms revolution inside the reported field of view, so this is deliberately
// derived from the message rather than from scan_time.
inline double SweepSeconds(std::size_t n_points, double time_increment) {
    if (n_points < 2 || !std::isfinite(time_increment) || time_increment <= 0.0) return 0.0;
    return static_cast<double>(n_points - 1) * time_increment;
}

// Offset from header.stamp to the sweep end, per the configured convention.
// "begin" -> one sweep    (ROS LaserScan convention; this car, per the A/B)
// "end"   -> 0            (rollback / a genuinely end-stamped driver)
// An unknown string falls back to "begin" so a typo cannot silently move the
// registration reference away from the measured answer.
inline double ScanEndOffsetSeconds(const std::string &convention, std::size_t n_points,
                                   double time_increment) {
    if (convention == "end") return 0.0;
    return SweepSeconds(n_points, time_increment);
}

inline bool IsKnownConvention(const std::string &convention) {
    return convention == "end" || convention == "begin";
}

// What the ARRIVAL LAG alone would suggest. `receive - header` must be at
// least one sweep for a begin-stamped scan, because the scan cannot be
// published before it has been acquired.
//
// ⚠️ Only valid when the LiDAR stamp shares the host clock. On this car's
//    urg_node it does not (device-clock mapping bias ~+21 ms), so this returns
//    "end" while the registration A/B says "begin". Report it, never act on
//    it — the caller logs a note when the two disagree.
inline const char *LagSuggestedConvention(double receive_minus_header_ms, double sweep_ms) {
    return (receive_minus_header_ms + 1.0 < sweep_ms) ? "end" : "begin";
}

// Result of asking the odometry history for a pose at a given stamp, without
// ever extrapolating or clamping.
//
// ⚠️ The node's other lookup (OdomAt) deliberately clamps to the newest sample
//    when the request is up to 100 ms in the future. That is right for the
//    watchdog and wrong here: a clamped prior is *truncated* rather than
//    shifted, and a truncated prior is worse than an offset one — measured on
//    run_20260821_021805 (corner yaw correction 2.84 -> 5.18 deg, corner
//    position 15.5 -> 23.4 cm) when the scan-end window was first tried on top
//    of the clamping lookup.
enum class OdomLookup { Ready, WaitingForFuture, TooOld, Empty };

// Bracket test on stamps alone (the caller does the SE3 interpolation).
// `oldest`/`newest` are the history bounds in seconds; `samples` is its size.
inline OdomLookup BracketStatus(double stamp, double oldest, double newest,
                                std::size_t samples) {
    if (samples < 2) return OdomLookup::Empty;
    if (stamp < oldest) return OdomLookup::TooOld;
    if (stamp > newest) return OdomLookup::WaitingForFuture;
    return OdomLookup::Ready;
}

// What the dispatcher should do with the scan at the head of the queue.
enum class SyncAction { Process, Wait, Drop };

// `waited_sec` is how long the head scan has been queued, `queue_size` the
// current depth. A scan is dropped rather than registered against an
// extrapolated prior — a hole in the output is recoverable, a silently wrong
// pose is not.
inline SyncAction DecideSyncAction(OdomLookup status, double waited_sec, std::size_t queue_size,
                                   double max_wait_sec, int max_queue) {
    switch (status) {
        case OdomLookup::Ready:
            return SyncAction::Process;
        case OdomLookup::TooOld:
            return SyncAction::Drop;
        case OdomLookup::WaitingForFuture:
        case OdomLookup::Empty:
            break;
    }
    if (waited_sec > max_wait_sec) return SyncAction::Drop;
    if (max_queue > 0 && static_cast<int>(queue_size) > max_queue) return SyncAction::Drop;
    return SyncAction::Wait;
}

}  // namespace kinematic_localization::sync
