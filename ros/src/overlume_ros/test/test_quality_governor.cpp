// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/quality_governor.hpp"

#include <gtest/gtest.h>

namespace {

using overlume::ros::QualityGovernor;
using overlume::ros::QualityGovernorParams;
using overlume::ros::QualityTransition;

QualityGovernorParams SmallParams() {
    QualityGovernorParams p;
    p.window_size = 4;
    p.drop_threshold_ms = 30.0;
    p.recover_threshold_ms = 20.0;
    p.recover_windows_required = 3;
    p.min_dwell_windows = 2;
    return p;
}

QualityTransition FeedWindow(QualityGovernor& gov, double ms, uint32_t n) {
    QualityTransition last = QualityTransition::NONE;
    for (uint32_t i = 0; i < n; ++i) last = gov.record_render_ms(ms);
    return last;
}

}

TEST(QualityGovernor, NoTransitionUntilAWindowActuallyCloses) {
    QualityGovernor gov(SmallParams(), 1);
    EXPECT_EQ(gov.record_render_ms(100.0), QualityTransition::NONE);
    EXPECT_EQ(gov.record_render_ms(100.0), QualityTransition::NONE);
    EXPECT_EQ(gov.record_render_ms(100.0), QualityTransition::NONE);
    EXPECT_EQ(gov.current_preset(), 1u);
}

TEST(QualityGovernor, StaysPutInsideTheHysteresisGap) {
    QualityGovernor gov(SmallParams(), 1);
    EXPECT_EQ(FeedWindow(gov, 25.0, 4), QualityTransition::NONE);
    EXPECT_EQ(gov.current_preset(), 1u);
}

TEST(QualityGovernor, DropsOnePresetWhenAWindowsP95ExceedsTheDropThreshold) {
    QualityGovernor gov(SmallParams(), 2);
    EXPECT_EQ(FeedWindow(gov, 40.0, 4), QualityTransition::DROPPED);
    EXPECT_EQ(gov.current_preset(), 1u);
}

TEST(QualityGovernor, DropNeverGoesBelowLow) {
    QualityGovernor gov(SmallParams(), 0);
    EXPECT_EQ(FeedWindow(gov, 40.0, 4), QualityTransition::NONE);
    EXPECT_EQ(gov.current_preset(), 0u);
}

TEST(QualityGovernor, RecoverRequiresConsecutiveGoodWindowsNotJustOne) {
    QualityGovernorParams params = SmallParams();
    params.min_dwell_windows = 0;
    QualityGovernor gov(params, 0);

    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::NONE);
    EXPECT_EQ(gov.current_preset(), 0u);
    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::NONE);
    EXPECT_EQ(gov.current_preset(), 0u);
    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::RECOVERED);
    EXPECT_EQ(gov.current_preset(), 1u);
}

TEST(QualityGovernor, AHysteresisGapWindowBreaksAnInProgressRecoveryStreak) {
    QualityGovernorParams params = SmallParams();
    params.min_dwell_windows = 0;
    QualityGovernor gov(params, 0);

    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::NONE);
    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::NONE);
    EXPECT_EQ(FeedWindow(gov, 25.0, 4), QualityTransition::NONE);
    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::NONE);
    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::NONE);
    EXPECT_EQ(gov.current_preset(), 0u);
    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::RECOVERED);
    EXPECT_EQ(gov.current_preset(), 1u);
}

TEST(QualityGovernor, RecoverNeverGoesAboveHigh) {
    QualityGovernorParams params = SmallParams();
    params.min_dwell_windows = 0;
    QualityGovernor gov(params, 2);
    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::NONE);
    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::NONE);
    EXPECT_EQ(FeedWindow(gov, 10.0, 4), QualityTransition::NONE);
    EXPECT_EQ(gov.current_preset(), 2u);
}

TEST(QualityGovernor, SyntheticLoadTriggersADropThenARecovery) {
    QualityGovernorParams params = SmallParams();
    params.min_dwell_windows = 0;
    QualityGovernor gov(params, 2);

    EXPECT_EQ(FeedWindow(gov, 45.0, 4), QualityTransition::DROPPED);
    EXPECT_EQ(gov.current_preset(), 1u);

    EXPECT_EQ(FeedWindow(gov, 45.0, 4), QualityTransition::DROPPED);
    EXPECT_EQ(gov.current_preset(), 0u);

    EXPECT_EQ(FeedWindow(gov, 5.0, 4), QualityTransition::NONE);
    EXPECT_EQ(FeedWindow(gov, 5.0, 4), QualityTransition::NONE);
    EXPECT_EQ(FeedWindow(gov, 5.0, 4), QualityTransition::RECOVERED);
    EXPECT_EQ(gov.current_preset(), 1u);
}

TEST(QualityGovernor, MinDwellBlocksASecondTransitionTooSoonAfterTheFirst) {
    QualityGovernorParams params = SmallParams();
    params.min_dwell_windows = 2;
    QualityGovernor gov(params, 2);

    EXPECT_EQ(FeedWindow(gov, 45.0, 4), QualityTransition::DROPPED);
    EXPECT_EQ(gov.current_preset(), 1u);

    EXPECT_EQ(FeedWindow(gov, 45.0, 4), QualityTransition::NONE);
    EXPECT_EQ(gov.current_preset(), 1u);

    EXPECT_EQ(FeedWindow(gov, 45.0, 4), QualityTransition::DROPPED);
    EXPECT_EQ(gov.current_preset(), 0u);
}

TEST(QualityGovernor, ConstructorClampsAnOutOfRangeInitialPreset) {
    QualityGovernor gov(SmallParams(), 99);
    EXPECT_EQ(gov.current_preset(), 2u);
}

TEST(QualityGovernor, AlternatingOverloadAndHeadroomNeverBouncesThePresetUp) {
    QualityGovernor gov(SmallParams(), 2);
    uint32_t last_preset = gov.current_preset();
    for (int i = 0; i < 10; ++i) {
        const double ms = (i % 2 == 0) ? 45.0 : 5.0;
        FeedWindow(gov, ms, 4);
        EXPECT_LE(gov.current_preset(), last_preset) << "preset rose mid-trace at window " << i;
        last_preset = gov.current_preset();
    }
}

TEST(QualityGovernor, DefaultParamsWalkDownThenUpAcrossASyntheticTrace) {
    QualityGovernor gov(QualityGovernorParams{}, 2);
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
