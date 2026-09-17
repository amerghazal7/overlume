// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// test_quality_governor.cpp — VM-040 (Epic 5): QualityGovernor is a pure
// hysteresis state machine over a render_ms stream (quality_governor.hpp's
// own header comment) -- no ROS node, no clock, no renderer, so this is the
// AC's own "test seam: deterministic, CI-safe" load-injection path. Every
// case below uses small, explicit params (never the node's real defaults)
// so a window closes in a handful of asserted record_render_ms() calls.
#include "overlume_ros/quality_governor.hpp"

#include <gtest/gtest.h>

namespace {

using overlume_node::QualityGovernor;
using overlume_node::QualityGovernorParams;
using overlume_node::QualityTransition;

QualityGovernorParams SmallParams() {
    QualityGovernorParams p;
    p.window_size = 4;
    p.drop_threshold_ms = 30.0;
    p.recover_threshold_ms = 20.0;
    p.recover_windows_required = 3;
    p.min_dwell_windows = 2;
    return p;
}

// Feeds `n` copies of `ms` and returns the transition the LAST sample's
// window close (if any) produced -- every case below chooses `n` as an
// exact multiple of window_size so this is always a real window boundary.
QualityTransition FeedWindow(QualityGovernor& gov, double ms, uint32_t n) {
    QualityTransition last = QualityTransition::NONE;
    for (uint32_t i = 0; i < n; ++i) last = gov.record_render_ms(ms);
    return last;
}

}  // namespace

TEST(QualityGovernor, NoTransitionUntilAWindowActuallyCloses) {
    QualityGovernor gov(SmallParams(), /*initial_preset=*/1);
    // window_size == 4 -- three samples never close a window, however
    // extreme.
    EXPECT_EQ(gov.record_render_ms(100.0), QualityTransition::NONE);
    EXPECT_EQ(gov.record_render_ms(100.0), QualityTransition::NONE);
    EXPECT_EQ(gov.record_render_ms(100.0), QualityTransition::NONE);
    EXPECT_EQ(gov.current_preset(), 1u);
}

TEST(QualityGovernor, StaysPutInsideTheHysteresisGap) {
    // Between recover_threshold_ms (20) and drop_threshold_ms (30):
    // neither overloaded nor headroom.
    QualityGovernor gov(SmallParams(), /*initial_preset=*/1);
    EXPECT_EQ(FeedWindow(gov, 25.0, 4), QualityTransition::NONE);
    EXPECT_EQ(gov.current_preset(), 1u);
}

TEST(QualityGovernor, DropsOnePresetWhenAWindowsP95ExceedsTheDropThreshold) {
    QualityGovernor gov(SmallParams(), /*initial_preset=*/2);
    EXPECT_EQ(FeedWindow(gov, 40.0, 4), QualityTransition::DROPPED);
    EXPECT_EQ(gov.current_preset(), 1u);
}

TEST(QualityGovernor, DropNeverGoesBelowLow) {
    QualityGovernor gov(SmallParams(), /*initial_preset=*/0);
    // Already at the floor -- an overloaded window has nowhere to drop to.
    EXPECT_EQ(FeedWindow(gov, 40.0, 4), QualityTransition::NONE);
    EXPECT_EQ(gov.current_preset(), 0u);
}

TEST(QualityGovernor, RecoverRequiresConsecutiveGoodWindowsNotJustOne) {
    QualityGovernorParams params = SmallParams();
    params.min_dwell_windows = 0;  // isolate the recovery-streak behavior
    QualityGovernor gov(params, /*initial_preset=*/0);

    // recover_windows_required == 3: the first two good windows must NOT
    // recover on their own.
    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::NONE);
    EXPECT_EQ(gov.current_preset(), 0u);
    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::NONE);
    EXPECT_EQ(gov.current_preset(), 0u);
    // Third CONSECUTIVE good window recovers.
    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::RECOVERED);
    EXPECT_EQ(gov.current_preset(), 1u);
}

TEST(QualityGovernor, AHysteresisGapWindowBreaksAnInProgressRecoveryStreak) {
    QualityGovernorParams params = SmallParams();
    params.min_dwell_windows = 0;
    QualityGovernor gov(params, /*initial_preset=*/0);

    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::NONE);  // good #1
    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::NONE);  // good #2
    // A window inside the hysteresis gap resets the streak -- it is
    // neither overloaded nor headroom.
    EXPECT_EQ(FeedWindow(gov, 25.0, 4), QualityTransition::NONE);
    // Two more good windows are NOT enough (streak restarted, needs 3).
    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::NONE);  // good #1 (again)
    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::NONE);  // good #2 (again)
    EXPECT_EQ(gov.current_preset(), 0u);
    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::RECOVERED);  // good #3
    EXPECT_EQ(gov.current_preset(), 1u);
}

TEST(QualityGovernor, RecoverNeverGoesAboveHigh) {
    QualityGovernorParams params = SmallParams();
    params.min_dwell_windows = 0;
    QualityGovernor gov(params, /*initial_preset=*/2);
    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::NONE);
    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::NONE);
    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::NONE);  // already at the ceiling
    EXPECT_EQ(gov.current_preset(), 2u);
}

// HYSTERESIS AC: drop reacts within one window; recovery only after a
// SUSTAINED headroom window count -- proven together, one governor
// instance, drop then recover, matching the backlog's own "synthetic-load
// test triggers drop + log; recovers" acceptance criterion (the log is the
// node's own responsibility at the call site -- see overlume_node.cpp).
TEST(QualityGovernor, SyntheticLoadTriggersADropThenARecovery) {
    QualityGovernorParams params = SmallParams();
    params.min_dwell_windows = 0;  // no artificial delay between the two
    QualityGovernor gov(params, /*initial_preset=*/2);

    // Overload: one window over 30ms drops high -> medium immediately.
    EXPECT_EQ(FeedWindow(gov, 45.0, 4), QualityTransition::DROPPED);
    EXPECT_EQ(gov.current_preset(), 1u);

    // A second overloaded window drops medium -> low.
    EXPECT_EQ(FeedWindow(gov, 45.0, 4), QualityTransition::DROPPED);
    EXPECT_EQ(gov.current_preset(), 0u);

    // Real headroom returns: 3 consecutive good windows recover low ->
    // medium.
    EXPECT_EQ(FeedWindow(gov, 5.0, 4), QualityTransition::NONE);
    EXPECT_EQ(FeedWindow(gov, 5.0, 4), QualityTransition::NONE);
    EXPECT_EQ(FeedWindow(gov, 5.0, 4), QualityTransition::RECOVERED);
    EXPECT_EQ(gov.current_preset(), 1u);
}

TEST(QualityGovernor, MinDwellBlocksASecondTransitionTooSoonAfterTheFirst) {
    QualityGovernorParams params = SmallParams();
    params.min_dwell_windows = 2;
    QualityGovernor gov(params, /*initial_preset=*/2);

    // First overloaded window: dwell floor was pre-satisfied at
    // construction (no prior transition), so this drops immediately.
    EXPECT_EQ(FeedWindow(gov, 45.0, 4), QualityTransition::DROPPED);
    EXPECT_EQ(gov.current_preset(), 1u);

    // Immediately overloaded again -- only 1 window has elapsed since the
    // last transition (< min_dwell_windows == 2), so this one is held even
    // though the samples clearly warrant another drop.
    EXPECT_EQ(FeedWindow(gov, 45.0, 4), QualityTransition::NONE);
    EXPECT_EQ(gov.current_preset(), 1u);

    // A second window later, the dwell floor has now elapsed -- the still-
    // overloaded stream drops again.
    EXPECT_EQ(FeedWindow(gov, 45.0, 4), QualityTransition::DROPPED);
    EXPECT_EQ(gov.current_preset(), 0u);
}

TEST(QualityGovernor, ConstructorClampsAnOutOfRangeInitialPreset) {
    QualityGovernor gov(SmallParams(), /*initial_preset=*/99);
    EXPECT_EQ(gov.current_preset(), 2u);
}

// Anti-flap AC, directly: a trace that alternates overload and headroom
// windows must never bounce the preset upward mid-trace -- it can only hold
// or ratchet down, because no run of good windows here is ever long enough
// to satisfy recover_windows_required (a bad window always resets the good
// streak). Uses SmallParams()'s real min_dwell_windows/recover_windows_
// required (not zeroed out, unlike the isolation tests above).
TEST(QualityGovernor, AlternatingOverloadAndHeadroomNeverBouncesThePresetUp) {
    QualityGovernor gov(SmallParams(), /*initial_preset=*/2);
    uint32_t last_preset = gov.current_preset();
    for (int i = 0; i < 10; ++i) {
        const double ms = (i % 2 == 0) ? 45.0 : 5.0;  // alternate overload / headroom
        FeedWindow(gov, ms, 4);
        EXPECT_LE(gov.current_preset(), last_preset) << "preset rose mid-trace at window " << i;
        last_preset = gov.current_preset();
    }
}

// Evidence for the VM-040 backlog Done note (docs/evidence/
// vm040-governor-2026-09-11/quality_governor_defaults.txt): unlike every
// case above, this one uses the SHIPPED defaults (QualityGovernorParams{},
// no overrides) -- window 30, drop 28.0, recover 18.0, 3 recover windows,
// 3 dwell windows -- not SmallParams(). Sustained overload (40ms, > 28.0)
// walks high->medium->low; sustained headroom (12ms, < 18.0) then walks
// low->medium->high, gated by the real recover-streak and dwell floor
// together.
TEST(QualityGovernor, DefaultParamsWalkDownThenUpAcrossASyntheticTrace) {
    QualityGovernor gov(QualityGovernorParams{}, /*initial_preset=*/2);
    EXPECT_EQ(gov.current_preset(), 2u);

    EXPECT_EQ(FeedWindow(gov, 40.0, 30), QualityTransition::DROPPED);
    EXPECT_EQ(gov.current_preset(), 1u);
    EXPECT_EQ(FeedWindow(gov, 40.0, 30), QualityTransition::NONE);
    EXPECT_EQ(FeedWindow(gov, 40.0, 30), QualityTransition::NONE);
    EXPECT_EQ(FeedWindow(gov, 40.0, 30), QualityTransition::DROPPED);
    EXPECT_EQ(gov.current_preset(), 0u);
    EXPECT_EQ(FeedWindow(gov, 40.0, 30), QualityTransition::NONE);
    EXPECT_EQ(gov.current_preset(), 0u);

    EXPECT_EQ(FeedWindow(gov, 12.0, 30), QualityTransition::NONE);
    EXPECT_EQ(FeedWindow(gov, 12.0, 30), QualityTransition::NONE);
    EXPECT_EQ(FeedWindow(gov, 12.0, 30), QualityTransition::RECOVERED);
    EXPECT_EQ(gov.current_preset(), 1u);
    EXPECT_EQ(FeedWindow(gov, 12.0, 30), QualityTransition::NONE);
    EXPECT_EQ(FeedWindow(gov, 12.0, 30), QualityTransition::NONE);
    EXPECT_EQ(FeedWindow(gov, 12.0, 30), QualityTransition::RECOVERED);
    EXPECT_EQ(gov.current_preset(), 2u);
    for (int i = 0; i < 6; ++i) {
        EXPECT_EQ(FeedWindow(gov, 12.0, 30), QualityTransition::NONE);
    }
    EXPECT_EQ(gov.current_preset(), 2u);
}
