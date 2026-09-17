// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

/** @file test_callouts.cpp
 *  @brief Epic 3 Task 4 (VM-031) Step 2: BuildNearestCallout()'s
 *  camera-tracking and behind-camera-suppression behavior. DrawCallout()
 *  itself is exercised visually by the callouts golden, not pixel-asserted
 *  here -- same split test_hud_overlay.cpp uses between CompositeHud()'s own
 *  presence-check tests and its golden.
 */
#include "overlume_ros/callouts.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "overlume/api.h"
#include "overlume/scene.h"

namespace {

overlume::VisualRenderer* MakeRenderer(uint32_t width, uint32_t height) {
    overlume::RenderConfig config{};
    config.width = width;
    config.height = height;
    config.quality = 0;
    config.theme_assets_dir = nullptr;  // compiled-in fallback theme -- no asset dir needed
    config.initial_theme = nullptr;
    return overlume::create_renderer(config);
}

}  // namespace

TEST(Callouts, NearestObstacleChipTracksAcrossCameraMove) {
    constexpr uint32_t kW = 320, kH = 240;
    overlume::VisualRenderer* r = MakeRenderer(kW, kH);
    ASSERT_NE(r, nullptr);

    // Same anchor both ticks -- only the camera moves. Deliberately OFF the
    // look-at target itself: projecting the exact target point always lands
    // near screen center regardless of eye position (see the library's own
    // ProjectToScreen.PointAtCameraTargetProjectsNearCenter test) which
    // would make this test pass by coincidence, not because the anchor
    // actually tracked anything.
    overlume::Vec3 obstacle{2.0, 1.5, -0.5};
    overlume::AlertPolygon alert{};
    alert.points = &obstacle;
    alert.point_count = 1;
    alert.severity = 2;

    std::vector<uint8_t> rgb(static_cast<size_t>(kW) * kH * 3, 0);
    overlume::FrameView view{rgb.data(), kW, kH};

    overlume::CameraPose pose1{};
    pose1.eye[0] = -4.0;
    pose1.eye[1] = 0.0;
    pose1.eye[2] = 3.5;
    pose1.target[0] = 2.0;
    pose1.target[1] = 0.0;
    pose1.target[2] = -0.5;
    pose1.vfov_deg = 80.0;
    ASSERT_TRUE(overlume::render_frame(r, pose1, view));

    overlume_node::Callout c1{};
    const overlume::Vec3 ego1{pose1.eye[0], pose1.eye[1], pose1.eye[2]};
    ASSERT_TRUE(overlume_node::BuildNearestCallout(r, &alert, 1, ego1, c1));

    // Second tick: camera orbits to a different eye, same target/obstacle.
    overlume::CameraPose pose2 = pose1;
    pose2.eye[1] = 3.0;
    ASSERT_TRUE(overlume::render_frame(r, pose2, view));

    overlume_node::Callout c2{};
    const overlume::Vec3 ego2{pose2.eye[0], pose2.eye[1], pose2.eye[2]};
    ASSERT_TRUE(overlume_node::BuildNearestCallout(r, &alert, 1, ego2, c2));

    EXPECT_TRUE(std::fabs(c1.anchor_x - c2.anchor_x) > 0.01f ||
                std::fabs(c1.anchor_y - c2.anchor_y) > 0.01f)
        << "chip screen position did not move across the camera move (c1=(" << c1.anchor_x << ","
        << c1.anchor_y << ") c2=(" << c2.anchor_x << "," << c2.anchor_y << "))";

    overlume::destroy_renderer(r);
}

TEST(Callouts, ChipForAnObjectBehindCameraIsSuppressedNotMisdrawn) {
    constexpr uint32_t kW = 320, kH = 240;
    overlume::VisualRenderer* r = MakeRenderer(kW, kH);
    ASSERT_NE(r, nullptr);

    overlume::CameraPose pose{};
    pose.eye[0] = -4.0;
    pose.eye[1] = 0.0;
    pose.eye[2] = 3.5;
    pose.target[0] = 2.0;
    pose.target[1] = 0.0;
    pose.target[2] = -0.5;
    pose.vfov_deg = 80.0;
    std::vector<uint8_t> rgb(static_cast<size_t>(kW) * kH * 3, 0);
    overlume::FrameView view{rgb.data(), kW, kH};
    ASSERT_TRUE(overlume::render_frame(r, pose, view));

    // Straight behind the eye, opposite the look direction -- same
    // construction as the library's own ProjectToScreen.PointBehindCamera
    // ReturnsFalse test.
    overlume::Vec3 behind{pose.eye[0] + (pose.eye[0] - pose.target[0]),
                          pose.eye[1] + (pose.eye[1] - pose.target[1]),
                          pose.eye[2] + (pose.eye[2] - pose.target[2])};
    overlume::AlertPolygon alert{};
    alert.points = &behind;
    alert.point_count = 1;
    alert.severity = 2;

    overlume_node::Callout c{};
    const overlume::Vec3 ego_pos{pose.eye[0], pose.eye[1], pose.eye[2]};
    EXPECT_FALSE(overlume_node::BuildNearestCallout(r, &alert, 1, ego_pos, c));

    overlume::destroy_renderer(r);
}

TEST(Callouts, NoAlertsReturnsFalse) {
    // alert_count==0 short-circuits before ever touching `renderer` --
    // nullptr here is deliberate, not an oversight.
    overlume_node::Callout c{};
    EXPECT_FALSE(overlume_node::BuildNearestCallout(nullptr, nullptr, 0, overlume::Vec3{}, c));
}

// ── Sanctioned-red callouts golden (Task 4 Step 2's own instruction) ────────
// Renders a real (headless-EGL, low-preset) 720p frame with one AlertPolygon
// in view, builds + draws the callout on top, writes
// /tmp/callouts_720p_actual.png, and SSIMs it against a golden this task
// deliberately does NOT commit -- same "missing golden -> 0.0, never
// silently passes" convention test_hud_overlay.cpp's own golden uses.
// UNPROMOTED, sanctioned red: the user promotes the actual PNG into
// test/fixtures/callouts_720p_golden.png once satisfied with it.
//
// This file's own vendored stb_image/stb_image_write (not overlume's
// -- see test_hud_overlay.cpp's identical comment on why that split exists).
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

namespace {

double luminance(uint8_t r, uint8_t g, uint8_t b) { return 0.2126 * r + 0.7152 * g + 0.0722 * b; }

// ponytail: block-wise (8x8, non-overlapping, luminance-only) mean/
// variance/covariance SSIM -- same deliberately-simplified approximation as
// test_hud_overlay.cpp's own block_ssim (reimplemented here rather than
// shared: separate test binary, same "no shared header between clang/libc++
// and gcc/libstdc++ test trees" reason as everywhere else in this package).
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

TEST(CalloutsGolden, SyntheticFrameWithVisibleCallout720pLowPreset) {
    constexpr uint32_t kW = 1280, kH = 720;
    overlume::RenderConfig config{};
    config.width = kW;
    config.height = kH;
    config.quality = 0;  // low preset -- same AC shape as the HUD golden
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

    // One obstacle, in view and off-center (see the tracking test's own
    // comment on why not the exact look-at target).
    overlume::Vec3 obstacle{2.0, 1.0, -0.5};
    overlume::AlertPolygon alert{};
    alert.points = &obstacle;
    alert.point_count = 1;
    alert.severity = 2;

    overlume::SceneGraph scene{};
    scene.sim_time_sec = 1.0;
    scene.ego.position = {0.0, 0.0, 0.0};
    scene.ego.heading_rad = 0.0;
    scene.ego.speed_mps = 5.0;
    scene.ego.valid = 1;
    scene.alerts = &alert;
    scene.alert_count = 1;
    overlume_node::PopulateHud(scene, /*active_mode=*/3);
    overlume::set_scene(r, scene);

    std::vector<uint8_t> frame(static_cast<size_t>(kW) * kH * 3, 0);
    overlume::FrameView view{frame.data(), kW, kH};
    ASSERT_TRUE(overlume::render_frame(r, pose, view));

    overlume_node::Callout callout{};
    ASSERT_TRUE(overlume_node::BuildNearestCallout(r, &alert, 1, scene.ego.position, callout))
        << "obstacle expected in view for this golden's own fixed pose";

    const overlume::HudColors colors = overlume::get_hud_colors(r);
    overlume_node::DrawCallout(frame.data(), kW, kH, callout,
                               overlume_node::HudRgb{colors.accent_color[0], colors.accent_color[1],
                                                     colors.accent_color[2]},
                               colors.scale, OVERLUME_NODE_FONT_PATH);

    const char* actual_path = "/tmp/callouts_720p_actual.png";
    stbi_write_png(actual_path, static_cast<int>(kW), static_cast<int>(kH), 3, frame.data(),
                   static_cast<int>(kW) * 3);

    const std::string golden_path =
        std::string(OVERLUME_NODE_FIXTURES_DIR) + "/callouts_720p_golden.png";
    int golden_w = 0, golden_h = 0, golden_c = 0;
    uint8_t* golden = stbi_load(golden_path.c_str(), &golden_w, &golden_h, &golden_c, 3);
    double ssim = 0.0;  // no committed golden yet -- same "missing golden -> 0.0" convention
    if (golden != nullptr && static_cast<uint32_t>(golden_w) == kW &&
        static_cast<uint32_t>(golden_h) == kH) {
        const std::vector<uint8_t> golden_pixels(
            golden, golden + static_cast<size_t>(golden_w) * golden_h * 3);
        ssim = block_ssim(frame, golden_pixels, kW, kH);
    }
    if (golden != nullptr) stbi_image_free(golden);

    // SANCTIONED RED (plan's own instruction): unpromoted until a human
    // looks at actual_path and copies it to golden_path.
    EXPECT_GT(ssim, 0.98) << "actual frame written to " << actual_path << " -- promote to "
                          << golden_path << " once reviewed";

    overlume::destroy_renderer(r);
}
