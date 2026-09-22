// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "test_paths.hpp"
#include "bowl_gray_probe.hpp"

#include <algorithm>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

namespace {}

TEST(BowlExposureCalibration, MidGrayRoundTripsWithinToleranceAtShippedDefault) {
    overlume::RenderConfig cfg{overlume::testing::kGrayProbeW, overlume::testing::kGrayProbeH, 1,
                               kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    int out = overlume::testing::render_gray_probe(r, 128);
    ASSERT_GE(out, 0) << "no bowl-surface pixels found in the rendered frame";
    EXPECT_NEAR(out, 128, 8) << "mid-gray no longer round-trips at the shipped "
                                "exposure_compensation default -- see scene.h's "
                                "BowlConfig::exposure_compensation comment";
    overlume::destroy_renderer(r);
}

TEST(BowlExposureCalibration, BrightGrayIsShoulderCompressedNotClipped) {
    overlume::RenderConfig cfg{overlume::testing::kGrayProbeW, overlume::testing::kGrayProbeH, 1,
                               kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    int out = overlume::testing::render_gray_probe(r, 224);
    ASSERT_GE(out, 0) << "no bowl-surface pixels found in the rendered frame";
    EXPECT_LT(out, 224) << "expected the ACES shoulder to compress the bright end, not "
                           "round-trip it exactly";
    EXPECT_LT(out, 250) << "expected no near-white clipping at this compensation";
    overlume::destroy_renderer(r);
}
