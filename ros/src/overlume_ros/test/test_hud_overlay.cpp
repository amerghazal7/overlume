// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/hud_overlay.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "overlume/api.h"
#include "overlume/scene.h"

TEST(HudOverlay, PopulateHudCopiesEgoSpeedAndActiveMode) {
    overlume::SceneGraph scene{};
    scene.ego.speed_mps = 12.3;
    overlume::ros::PopulateHud(scene, 3);
    EXPECT_DOUBLE_EQ(scene.hud.speed_mps, 12.3);
    EXPECT_EQ(scene.hud.active_mode, 3);
    EXPECT_EQ(scene.hud.chips, nullptr);
    EXPECT_EQ(scene.hud.chip_count, 0u);
}

namespace {
constexpr uint32_t kWidth = 1280;
constexpr uint32_t kHeight = 720;
constexpr uint8_t kBackground = 40;
}  // namespace

TEST(HudOverlay, CompositesLegibleTextAtLowPreset) {
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
    overlume::ros::HudSnapshot hud{12.3, 3};

    ASSERT_TRUE(overlume::ros::CompositeHud(rgb.data(), kWidth, kHeight, hud, {0.9f, 0.95f, 1.0f},
                                            {0.1f, 1.0f, 0.4f}, 1.0f, OVERLUME_NODE_FONT_PATH));

    constexpr int kRegionX0 = 20, kRegionX1 = 400;
    constexpr int kRegionY0 = 0, kRegionY1 = 100;
    constexpr int kLegibilityThreshold = 20;
    bool found_text_pixel = false;
    for (int y = kRegionY0; y < kRegionY1 && !found_text_pixel; ++y) {
        for (int x = kRegionX0; x < kRegionX1; ++x) {
            const size_t idx = (static_cast<size_t>(y) * kWidth + x) * 3;
            const int diff =
                std::abs(static_cast<int>(rgb[idx]) - static_cast<int>(kBackground)) +
                std::abs(static_cast<int>(rgb[idx + 1]) - static_cast<int>(kBackground)) +
                std::abs(static_cast<int>(rgb[idx + 2]) - static_cast<int>(kBackground));
            if (diff > kLegibilityThreshold) {
                found_text_pixel = true;
                break;
            }
        }
    }
    EXPECT_TRUE(found_text_pixel) << "no pixel in the expected HUD region differs from the "
                                     "uniform background -- text was not drawn";
}

TEST(HudOverlay, MissingOrUnloadableFontIsNonFatalAndLeavesFrameUnchanged) {
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
    const std::vector<uint8_t> before = rgb;
    overlume::ros::HudSnapshot hud{12.3, 3};

    const bool ok =
        overlume::ros::CompositeHud(rgb.data(), kWidth, kHeight, hud, {0.9f, 0.95f, 1.0f},
                                    {0.1f, 1.0f, 0.4f}, 1.0f, "/nonexistent/does_not_exist.ttf");
    EXPECT_FALSE(ok);
    EXPECT_EQ(rgb, before);
}

TEST(HudOverlay, EmptyFontPathIsNonFatal) {
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
    const std::vector<uint8_t> before = rgb;
    overlume::ros::HudSnapshot hud{12.3, 3};
    EXPECT_FALSE(overlume::ros::CompositeHud(rgb.data(), kWidth, kHeight, hud, {0, 0, 0}, {0, 0, 0},
                                             1.0f, ""));
    EXPECT_EQ(rgb, before);
}

TEST(HudOverlay, DrawTextAndDrawLineComposeACallout) {
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);

    overlume::ros::DrawLine(rgb.data(), kWidth, kHeight, 100, 100, 140, 60,
                            overlume::ros::HudRgb{1.0f, 0.2f, 0.2f});
    ASSERT_TRUE(overlume::ros::DrawText(rgb.data(), kWidth, kHeight, "3.2 m", 140, 60,
                                        overlume::ros::HudRgb{1.0f, 0.2f, 0.2f}, 1.0f,
                                        OVERLUME_NODE_FONT_PATH));

    constexpr int kLegibilityThreshold = 20;
    bool found_drawn_pixel = false;
    for (int y = 40; y < 100 && !found_drawn_pixel; ++y) {
        for (int x = 90; x < 300; ++x) {
            const size_t idx = (static_cast<size_t>(y) * kWidth + x) * 3;
            const int diff =
                std::abs(static_cast<int>(rgb[idx]) - static_cast<int>(kBackground)) +
                std::abs(static_cast<int>(rgb[idx + 1]) - static_cast<int>(kBackground)) +
                std::abs(static_cast<int>(rgb[idx + 2]) - static_cast<int>(kBackground));
            if (diff > kLegibilityThreshold) {
                found_drawn_pixel = true;
                break;
            }
        }
    }
    EXPECT_TRUE(found_drawn_pixel) << "neither the leader line nor the chip text drew any pixel";
}

TEST(HudOverlay, DrawTextMissingFontIsNonFatal) {
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
    const std::vector<uint8_t> before = rgb;
    EXPECT_FALSE(overlume::ros::DrawText(rgb.data(), kWidth, kHeight, "3.2 m", 10, 10,
                                         overlume::ros::HudRgb{1, 1, 1}, 1.0f,
                                         "/nonexistent/does_not_exist.ttf"));
    EXPECT_EQ(rgb, before);
}

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

namespace {

double luminance(uint8_t r, uint8_t g, uint8_t b) { return 0.2126 * r + 0.7152 * g + 0.0722 * b; }

double block_ssim(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b, uint32_t width,
                  uint32_t height) {
    constexpr int kBlock = 8;
    constexpr double kC1 = (0.01 * 255) * (0.01 * 255);
    constexpr double kC2 = (0.03 * 255) * (0.03 * 255);
    double total = 0.0;
    int blockCount = 0;
    for (uint32_t by = 0; by + kBlock <= height; by += kBlock) {
        for (uint32_t bx = 0; bx + kBlock <= width; bx += kBlock) {
            double sumA = 0, sumB = 0, sumAA = 0, sumBB = 0, sumAB = 0;
            const int n = kBlock * kBlock;
            for (int y = 0; y < kBlock; ++y) {
                for (int x = 0; x < kBlock; ++x) {
                    const uint32_t px = bx + x, py = by + y;
                    const size_t idx = (static_cast<size_t>(py) * width + px) * 3;
                    const double la = luminance(a[idx], a[idx + 1], a[idx + 2]);
                    const double lb = luminance(b[idx], b[idx + 1], b[idx + 2]);
                    sumA += la;
                    sumB += lb;
                    sumAA += la * la;
                    sumBB += lb * lb;
                    sumAB += la * lb;
                }
            }
            const double meanA = sumA / n, meanB = sumB / n;
            const double varA = sumAA / n - meanA * meanA;
            const double varB = sumBB / n - meanB * meanB;
            const double covAB = sumAB / n - meanA * meanB;
            const double ssim = ((2 * meanA * meanB + kC1) * (2 * covAB + kC2)) /
                                ((meanA * meanA + meanB * meanB + kC1) * (varA + varB + kC2));
            total += ssim;
            ++blockCount;
        }
    }
    return blockCount > 0 ? total / blockCount : 0.0;
}

}  // namespace

TEST(HudOverlayGolden, SyntheticSceneWithHud720pLowPreset) {
    overlume::RenderConfig config{};
    config.width = kWidth;
    config.height = kHeight;
    config.quality = 0;
    config.theme_assets_dir = nullptr;
    config.initial_theme = nullptr;
    overlume::VisualRenderer* r = overlume::create_renderer(config);
    ASSERT_NE(r, nullptr);

    overlume::CameraPose pose{};
    pose.eye[0] = -4.0;
    pose.eye[1] = 0.0;
    pose.eye[2] = 3.5;
    pose.target[0] = 2.0;
    pose.target[1] = 0.0;
    pose.target[2] = -0.5;
    pose.vfov_deg = 80.0;

    overlume::SceneGraph scene{};
    scene.sim_time_sec = 1.0;
    scene.ego.position = {0.0, 0.0, 0.0};
    scene.ego.heading_rad = 0.0;
    scene.ego.speed_mps = 12.3;
    scene.ego.valid = 1;
    overlume::ros::PopulateHud(scene, 3);
    overlume::set_scene(r, scene);

    std::vector<uint8_t> frame(static_cast<size_t>(kWidth) * kHeight * 3, 0);
    overlume::FrameView view{frame.data(), kWidth, kHeight};
    ASSERT_TRUE(overlume::render_frame(r, pose, view));

    const overlume::HudColors colors = overlume::get_hud_colors(r);
    const overlume::ros::HudSnapshot hud{scene.hud.speed_mps, scene.hud.active_mode};
    ASSERT_TRUE(overlume::ros::CompositeHud(
        frame.data(), kWidth, kHeight, hud,
        overlume::ros::HudRgb{colors.text_color[0], colors.text_color[1], colors.text_color[2]},
        overlume::ros::HudRgb{colors.accent_color[0], colors.accent_color[1],
                              colors.accent_color[2]},
        colors.scale, OVERLUME_NODE_FONT_PATH));

    const char* actual_path = "/tmp/hud_overlay_720p_actual.png";
    stbi_write_png(actual_path, static_cast<int>(kWidth), static_cast<int>(kHeight), 3,
                   frame.data(), static_cast<int>(kWidth) * 3);

    const std::string golden_path =
        std::string(OVERLUME_NODE_FIXTURES_DIR) + "/hud_overlay_720p_golden.png";
    int golden_w = 0, golden_h = 0, golden_c = 0;
    uint8_t* golden = stbi_load(golden_path.c_str(), &golden_w, &golden_h, &golden_c, 3);
    double ssim = 0.0;
    if (golden != nullptr && static_cast<uint32_t>(golden_w) == kWidth &&
        static_cast<uint32_t>(golden_h) == kHeight) {
        const std::vector<uint8_t> golden_pixels(
            golden, golden + static_cast<size_t>(golden_w) * golden_h * 3);
        ssim = block_ssim(frame, golden_pixels, kWidth, kHeight);
    }
    if (golden != nullptr) stbi_image_free(golden);

    EXPECT_GT(ssim, 0.98) << "actual frame written to " << actual_path << " -- promote to "
                          << golden_path << " once reviewed";

    overlume::destroy_renderer(r);
}
