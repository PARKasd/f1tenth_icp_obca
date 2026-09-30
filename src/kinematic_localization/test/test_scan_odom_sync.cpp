// §9 scan-end synchronisation (2026-08-22).
//
// The contracts these lock down:
//   ① The registration lookup NEVER clamps to the newest odom sample. That
//      clamp is what turned the first scan-end attempt into a 2x regression
//      (run_20260821_021805): a truncated prior is worse than an offset one.
//   ② The sweep-end reference follows the *measured* stamp convention, not an
//      assumption, and "begin" reproduces the legacy offset exactly.
//   ③ A scan without bracketing odometry is dropped, never registered against
//      an extrapolated prior — a hole in the output is recoverable, a silently
//      wrong pose is not.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "scan_odom_sync.hpp"

namespace kinematic_localization::sync
{
namespace
{

// This car: 1081 beams, 0.25 deg, 40 Hz, time_increment 1.736e-5 s.
constexpr std::size_t kBeams = 1081;
constexpr double kIncrement = 1.736e-5;
constexpr double kSweep = 0.01875;  // (1081-1) * 1.736e-5

TEST(SweepSeconds, MatchesTheMeasuredHokuyoSweep)
{
  EXPECT_NEAR(SweepSeconds(kBeams, kIncrement), kSweep, 1e-5);
}

TEST(SweepSeconds, DegradesToZeroOnUnusableMetadata)
{
  // A driver that leaves time_increment at 0 must not silently produce a
  // negative or NaN reference — the offset then becomes 0 (= "end").
  EXPECT_DOUBLE_EQ(SweepSeconds(kBeams, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(SweepSeconds(kBeams, -1.0), 0.0);
  EXPECT_DOUBLE_EQ(SweepSeconds(1, kIncrement), 0.0);
  EXPECT_DOUBLE_EQ(SweepSeconds(kBeams, std::nan("")), 0.0);
}

TEST(ScanEndOffset, BeginConventionAddsOneSweep)
{
  // Settled by the 2026-08-22 registration A/B, not by reasoning: "begin" cut
  // the cornering yaw correction p95 from 0.77 to 0.40 deg on run_..._035120.
  EXPECT_NEAR(ScanEndOffsetSeconds("begin", kBeams, kIncrement), kSweep, 1e-5);
}

TEST(ScanEndOffset, EndConventionIsTheRollbackAndAddsNothing)
{
  EXPECT_DOUBLE_EQ(ScanEndOffsetSeconds("end", kBeams, kIncrement), 0.0);
}

TEST(ScanEndOffset, UnknownStringFallsBackToTheMeasuredBeginNotToEnd)
{
  // A typo must not silently move the registration reference off the measured
  // answer, so the fallback is the winning arm rather than the neutral one.
  EXPECT_NEAR(ScanEndOffsetSeconds("BEGIN", kBeams, kIncrement), kSweep, 1e-5);
  EXPECT_NEAR(ScanEndOffsetSeconds("", kBeams, kIncrement), kSweep, 1e-5);
  EXPECT_FALSE(IsKnownConvention("BEGIN"));
  EXPECT_TRUE(IsKnownConvention("end"));
  EXPECT_TRUE(IsKnownConvention("begin"));
}

TEST(LagSuggestedConvention, IsOnlyAHostClockHeuristicAndIsWrongOnThisCar)
{
  // run_20260822_035120 / _015820: receive - header = -2.3 ms, sweep 18.75 ms.
  // The arithmetic says "end" — and the registration A/B says "begin". The
  // urg_node stamp is not on the host clock, so this test locks in that the
  // heuristic is REPORTED, never acted on.
  EXPECT_STREQ(LagSuggestedConvention(-2.3, 18.75), "end");
  EXPECT_STREQ(LagSuggestedConvention(2.0, 18.75), "end");
  // A driver whose stamps do share the host clock shows the full sweep of lag.
  EXPECT_STREQ(LagSuggestedConvention(20.7, 18.75), "begin");
  EXPECT_STREQ(LagSuggestedConvention(18.75, 18.75), "begin");
}

// ── ① never clamp ─────────────────────────────────────────────────────────

TEST(BracketStatus, FutureStampWaitsInsteadOfClampingToTheNewestSample)
{
  // This is the whole point of the strict lookup. OdomAt() would have returned
  // the newest pose here (it clamps anything within 100 ms), which truncates
  // the prior instead of shifting it.
  EXPECT_EQ(BracketStatus(10.030, 9.0, 10.000, 50), OdomLookup::WaitingForFuture);
  EXPECT_EQ(BracketStatus(10.0001, 9.0, 10.000, 50), OdomLookup::WaitingForFuture);
}

TEST(BracketStatus, BracketedStampIsReady)
{
  EXPECT_EQ(BracketStatus(9.5, 9.0, 10.0, 50), OdomLookup::Ready);
  EXPECT_EQ(BracketStatus(9.0, 9.0, 10.0, 50), OdomLookup::Ready);
  EXPECT_EQ(BracketStatus(10.0, 9.0, 10.0, 50), OdomLookup::Ready);
}

TEST(BracketStatus, StampOlderThanTheHistoryIsTooOld)
{
  EXPECT_EQ(BracketStatus(8.9, 9.0, 10.0, 50), OdomLookup::TooOld);
}

TEST(BracketStatus, NeedsTwoSamplesToInterpolate)
{
  EXPECT_EQ(BracketStatus(9.5, 9.0, 9.0, 0), OdomLookup::Empty);
  EXPECT_EQ(BracketStatus(9.5, 9.0, 9.0, 1), OdomLookup::Empty);
}

TEST(BracketStatus, BeginWaitsWhereEndWouldNot)
{
  // The cost side of the measured answer: "begin" pushes the reference one
  // sweep into the future, so the dispatcher waits for the next odom sample.
  // Measured on the replay: 35 ms median vs 15 ms for "end" and a structural
  // 25 ms (one scan of lag) for the legacy path — i.e. +10 ms over legacy,
  // bought with a 48 % cut in cornering yaw correction.
  const double header = 10.000;
  const double newest_odom = 10.002;  // /odom is ~1 ms behind arrival
  EXPECT_EQ(BracketStatus(header + ScanEndOffsetSeconds("end", kBeams, kIncrement), 9.0,
                          newest_odom, 50),
            OdomLookup::Ready);
  EXPECT_EQ(BracketStatus(header + ScanEndOffsetSeconds("begin", kBeams, kIncrement), 9.0,
                          newest_odom, 50),
            OdomLookup::WaitingForFuture);
}

// ── ③ queue policy ────────────────────────────────────────────────────────

TEST(DecideSyncAction, ProcessesAsSoonAsTheBracketExists)
{
  EXPECT_EQ(DecideSyncAction(OdomLookup::Ready, 0.0, 1, 0.08, 4), SyncAction::Process);
  // Even a scan that already sat past the wait cap is processed once bracketed:
  // the cap only bounds *waiting*, it must not throw away usable data.
  EXPECT_EQ(DecideSyncAction(OdomLookup::Ready, 5.0, 9, 0.08, 4), SyncAction::Process);
}

TEST(DecideSyncAction, WaitsOneOdomPeriodForTheFutureSample)
{
  EXPECT_EQ(DecideSyncAction(OdomLookup::WaitingForFuture, 0.000, 1, 0.08, 4), SyncAction::Wait);
  EXPECT_EQ(DecideSyncAction(OdomLookup::WaitingForFuture, 0.020, 1, 0.08, 4), SyncAction::Wait);
  EXPECT_EQ(DecideSyncAction(OdomLookup::Empty, 0.010, 1, 0.08, 4), SyncAction::Wait);
}

TEST(DecideSyncAction, DropsWhenWheelOdometryStalls)
{
  // The 034812 blackout is the real case: /odom stopped for 26 s. Scans must
  // not pile up unboundedly, and must never be registered on a guessed prior.
  EXPECT_EQ(DecideSyncAction(OdomLookup::WaitingForFuture, 0.081, 1, 0.08, 4), SyncAction::Drop);
  EXPECT_EQ(DecideSyncAction(OdomLookup::WaitingForFuture, 0.001, 5, 0.08, 4), SyncAction::Drop);
}

TEST(DecideSyncAction, DropsAScanOlderThanTheHistoryImmediately)
{
  // No amount of further waiting can make an expired stamp bracketable.
  EXPECT_EQ(DecideSyncAction(OdomLookup::TooOld, 0.0, 1, 0.08, 4), SyncAction::Drop);
}

TEST(DecideSyncAction, ZeroQueueCapDisablesTheDepthRule)
{
  // Only the time cap then governs — used to isolate the two limits in replay.
  EXPECT_EQ(DecideSyncAction(OdomLookup::WaitingForFuture, 0.01, 99, 0.08, 0), SyncAction::Wait);
}

}  // namespace
}  // namespace kinematic_localization::sync
