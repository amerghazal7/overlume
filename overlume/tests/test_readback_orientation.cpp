// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// Row 0 of the readback must be the TOP row of the image on every back end.

#include "overlume/api.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace {

double RowAverage(const std::vector<uint8_t>& rgb, uint32_t width, uint32_t row) {
    const uint8_t* p = rgb.data() + static_cast<size_t>(row) * width * 3;
    uint64_t sum = 0;
    for (uint32_t i = 0; i < width * 3; ++i) sum += p[i];
    return static_cast<double>(sum) / static_cast<double>(width * 3);
}

}  // namespace

TEST(ReadbackOrientation, Row0IsTopOfImage) {
    constexpr uint32_t kWidth = 128;
    constexpr uint32_t kHeight = 96;

    overlume::RenderConfig config{};
    config.width = kWidth;
    config.height = kHeight;
    config.quality = 1;

    overlume::VisualRenderer* renderer = overlume::create_renderer(config);
    if (renderer == nullptr) GTEST_SKIP() << "No GPU device available on this machine.";

    // Eye above the ground, looking along the horizon: sky on top, ground below.
    overlume::CameraPose pose{};
    pose.eye[0] = 0.0;
    pose.eye[1] = 0.0;
    pose.eye[2] = 2.0;
    pose.target[0] = 10.0;
    pose.target[1] = 0.0;
    pose.target[2] = 2.0;
    pose.vfov_deg = 60.0;

    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, 0);
    overlume::FrameView view{rgb.data(), kWidth, kHeight};
    ASSERT_TRUE(overlume::render_frame(renderer, pose, view));

    const double top = RowAverage(rgb, kWidth, 1);
    const double bottom = RowAverage(rgb, kWidth, kHeight - 2);
    EXPECT_GT(std::fabs(top - bottom), 5.0) << "top=" << top << " bottom=" << bottom;

    // Row 1 is sky: dark_adas palette.sky after the view's colour pipeline.
    const uint8_t expected[3] = {20, 27, 53};
    const uint8_t* px = rgb.data() + static_cast<size_t>(kWidth) * 3 + (kWidth / 2) * 3;
    for (int c = 0; c < 3; ++c) {
        EXPECT_NEAR(px[c], expected[c], 8) << "channel " << c << " of row 1 is not the sky colour";
    }

    overlume::destroy_renderer(renderer);
}
