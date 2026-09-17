// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// test_camera_textures.cpp — VM-090 (unified-engine migration Task 1,
// ADR-0005): the POD-boundary camera-texture mechanism (set_bowl_config(),
// set_camera_frame(), set_bowl_visible(), set_camera_motion_delta()). No
// bowl mesh/material exists yet (Task 2) -- these tests only exercise the
// camera-texture allocation/upload/dirty-tracking half.
#include "overlume/api.h"
#include "overlume/scene.h"

#include "camera_textures_test_hooks.hpp"
#include "test_paths.hpp"

#include <vector>

#include <gtest/gtest.h>

namespace {

overlume::BowlConfig one_camera_config(const overlume::CameraExtrinsics& ext,
                                       const overlume::CameraIntrinsics& in, const uint32_t& w,
                                       const uint32_t& h) {
    overlume::BowlConfig bc{};
    bc.camera_count = 1;
    bc.extrinsics = &ext;
    bc.intrinsics = &in;
    bc.cam_width = &w;
    bc.cam_height = &h;
    bc.bowl_R0 = 0.5;
    bc.bowl_k = 1.2;
    bc.bowl_Rmax = 8.0;
    bc.feather_margin = 0.15;
    bc.fill_blind_zone = 1;
    bc.exposure_match = 1;
    bc.sky_color[0] = 0.5f;
    bc.sky_color[1] = 0.7f;
    bc.sky_color[2] = 0.9f;
    return bc;
}

// Release-callback test double: increments a shared counter and frees the
// heap-allocated vector `user` points at. `ptr`/`size` (the wrapped buffer
// itself) are unused -- this only proves the callback fires and hands
// ownership back, not the pixel contents.
int g_release_count = 0;

void count_and_delete_release(void* /*ptr*/, size_t /*size*/, void* user) {
    ++g_release_count;
    delete static_cast<std::vector<uint8_t>*>(user);
}

}  // namespace

// ── Step 0: set_camera_frame() before any BowlConfig is a safe no-op ───────

TEST(CameraTextures, SetCameraFrameBeforeBowlConfigIsNonFatal) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    std::vector<uint8_t> pixels(64 * 48 * 3, 128);
    EXPECT_FALSE(overlume::set_camera_frame(r, 0, pixels.data(), 64, 48, 1));  // no BowlConfig yet
    overlume::destroy_renderer(r);
}

// ── Step 1: set_bowl_config() allocates persistent textures; ───────────────
//    set_camera_frame() uploads into them.

TEST(CameraTextures, SetCameraFrameUploadsAfterBowlConfig) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::CameraExtrinsics ext{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0.5}};
    overlume::CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    uint32_t w = 320, h = 240;
    overlume::BowlConfig bc = one_camera_config(ext, in, w, h);
    ASSERT_TRUE(overlume::set_bowl_config(r, bc));
    std::vector<uint8_t> pixels(320 * 240 * 3, 200);
    EXPECT_TRUE(overlume::set_camera_frame(r, 0, pixels.data(), 320, 240, 1));
    EXPECT_FALSE(
        overlume::set_camera_frame(r, 1, pixels.data(), 320, 240, 1));  // cam_idx out of range

    // Dims-mismatch guard: neither call below matches camera 0's configured
    // 320x240 -- both must be rejected without bumping the upload count (a
    // short buffer wrapped/copied against a larger RGB8 texture is a
    // GPU-side out-of-bounds read of caller memory).
    EXPECT_FALSE(overlume::set_camera_frame(r, 0, pixels.data(), 320, 120, 2));
    EXPECT_FALSE(overlume::set_camera_frame(r, 0, pixels.data(), 160, 240, 3));
    EXPECT_EQ(overlume::testing::camera_frame_upload_count(r, 0), 1u);
    overlume::destroy_renderer(r);
}

// ── Step 2: dirty tracking skips a re-upload when frame_id repeats ─────────

TEST(CameraTextures, RepeatedFrameIdSkipsReupload) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::CameraExtrinsics ext{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0.5}};
    overlume::CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    uint32_t w = 320, h = 240;
    overlume::BowlConfig bc = one_camera_config(ext, in, w, h);
    ASSERT_TRUE(overlume::set_bowl_config(r, bc));
    std::vector<uint8_t> pixels(320 * 240 * 3, 200);

    // hasUploaded pins the very-first upload regardless of frame_id -- a
    // caller whose first real frame_id is 0 (a legitimate starting value,
    // not "unchanged") must still upload once.
    overlume::set_camera_frame(r, 0, pixels.data(), 320, 240, /*frame_id=*/0);
    EXPECT_EQ(overlume::testing::camera_frame_upload_count(r, 0), 1u);

    overlume::set_camera_frame(r, 0, pixels.data(), 320, 240, /*frame_id=*/1);
    auto count_after_first = overlume::testing::camera_frame_upload_count(r, 0);
    overlume::set_camera_frame(r, 0, pixels.data(), 320, 240, /*frame_id=*/1);  // same frame_id
    EXPECT_EQ(overlume::testing::camera_frame_upload_count(r, 0),
              count_after_first);                                               // no re-upload
    overlume::set_camera_frame(r, 0, pixels.data(), 320, 240, /*frame_id=*/2);  // new frame_id
    EXPECT_EQ(overlume::testing::camera_frame_upload_count(r, 0),
              count_after_first + 1);  // uploads
    overlume::destroy_renderer(r);
}

// ── Step 2b: set_camera_motion_delta() gated on BowlConfig ─────────────────

TEST(CameraTextures, SetCameraMotionDeltaGatedOnBowlConfig) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const double identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    EXPECT_FALSE(overlume::set_camera_motion_delta(r, 0, identity));  // no BowlConfig yet

    overlume::CameraExtrinsics ext{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0.5}};
    overlume::CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    uint32_t w = 320, h = 240;
    overlume::BowlConfig bc = one_camera_config(ext, in, w, h);
    ASSERT_TRUE(overlume::set_bowl_config(r, bc));

    EXPECT_TRUE(overlume::set_camera_motion_delta(r, 0, identity));
    EXPECT_FALSE(overlume::set_camera_motion_delta(r, 1, identity));  // cam_idx out of range
    overlume::destroy_renderer(r);
}

// ── set_bowl_visible() gated on BowlConfig (Interfaces block; consumed by
//    Task 4's mode dispatch) -- same no-op-before-config convention as the
//    other three entry points above.

TEST(CameraTextures, SetBowlVisibleGatedOnBowlConfig) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    EXPECT_FALSE(overlume::set_bowl_visible(r, true));  // no BowlConfig yet

    overlume::CameraExtrinsics ext{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0.5}};
    overlume::CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    uint32_t w = 320, h = 240;
    overlume::BowlConfig bc = one_camera_config(ext, in, w, h);
    ASSERT_TRUE(overlume::set_bowl_config(r, bc));

    EXPECT_TRUE(overlume::set_bowl_visible(r, true));
    EXPECT_TRUE(overlume::set_bowl_visible(r, false));
    overlume::destroy_renderer(r);
}

// ── set_bowl_config() rejects an invalid camera_count -- same
//    "missing/invalid config renders nothing" convention as
//    set_environment_source's null-path (Interfaces block).

TEST(CameraTextures, SetBowlConfigRejectsInvalidCameraCount) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::CameraExtrinsics ext{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0.5}};
    overlume::CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    uint32_t w = 320, h = 240;

    overlume::BowlConfig zero = one_camera_config(ext, in, w, h);
    zero.camera_count = 0;
    EXPECT_FALSE(overlume::set_bowl_config(r, zero));

    overlume::BowlConfig too_many = one_camera_config(ext, in, w, h);
    too_many.camera_count = overlume::kMaxBowlCameras + 1;
    EXPECT_FALSE(overlume::set_bowl_config(r, too_many));

    EXPECT_FALSE(overlume::set_bowl_config(nullptr, zero));
    overlume::destroy_renderer(r);
}

// ── Ownership contract: `release` (ADR-0005) fires exactly once when the
//    library actually hands the buffer to Filament ─────────────────────────

TEST(CameraTextures, ReleaseCallbackFiresOncePerHandedInBuffer) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::CameraExtrinsics ext{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0.5}};
    overlume::CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    uint32_t w = 320, h = 240;
    overlume::BowlConfig bc = one_camera_config(ext, in, w, h);
    ASSERT_TRUE(overlume::set_bowl_config(r, bc));

    g_release_count = 0;
    auto* buf = new std::vector<uint8_t>(320 * 240 * 3, 200);
    EXPECT_TRUE(overlume::set_camera_frame(r, 0, buf->data(), 320, 240, /*frame_id=*/1,
                                           &count_and_delete_release, buf));

    // render_frame() flushes/waits for the readback, so by the time it
    // returns the driver has executed the setImage command that consumes
    // (and releases) the buffer.
    overlume::CameraPose pose{{12, -14, 10}, {12, 3, 0}, 70.0};
    std::vector<uint8_t> out(320u * 240u * 3u);
    overlume::FrameView view{out.data(), 320, 240};
    ASSERT_TRUE(overlume::render_frame(r, pose, view));

    EXPECT_EQ(g_release_count, 1);
    overlume::destroy_renderer(r);
}

// ── Ownership contract: `release` fires exactly once even when the call is
//    skipped by the dirty gate or rejected by the dims guard -- ownership
//    transfer is unconditional, not just on the successful-upload path ──────

TEST(CameraTextures, ReleaseCallbackFiresOnSkipAndReject) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::CameraExtrinsics ext{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0.5}};
    overlume::CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    uint32_t w = 320, h = 240;
    overlume::BowlConfig bc = one_camera_config(ext, in, w, h);
    ASSERT_TRUE(overlume::set_bowl_config(r, bc));

    // Prime the dirty gate with a real upload, and force its callback to
    // fire before touching the counter again -- otherwise it could fire
    // during a later render_frame() and pollute the counts below.
    g_release_count = 0;
    auto* primer = new std::vector<uint8_t>(320 * 240 * 3, 200);
    ASSERT_TRUE(overlume::set_camera_frame(r, 0, primer->data(), 320, 240, /*frame_id=*/1,
                                           &count_and_delete_release, primer));
    overlume::CameraPose pose{{12, -14, 10}, {12, 3, 0}, 70.0};
    std::vector<uint8_t> out(320u * 240u * 3u);
    overlume::FrameView view{out.data(), 320, 240};
    ASSERT_TRUE(overlume::render_frame(r, pose, view));
    ASSERT_EQ(g_release_count, 1);

    // Repeated frame_id -- dirty-gate skip -- still releases exactly once.
    g_release_count = 0;
    auto* repeat_buf = new std::vector<uint8_t>(320 * 240 * 3, 200);
    EXPECT_TRUE(overlume::set_camera_frame(r, 0, repeat_buf->data(), 320, 240, /*frame_id=*/1,
                                           &count_and_delete_release, repeat_buf));
    EXPECT_EQ(g_release_count, 1);

    // Dims mismatch -- rejected call -- still releases exactly once.
    g_release_count = 0;
    auto* mismatched_buf = new std::vector<uint8_t>(320 * 120 * 3, 200);
    EXPECT_FALSE(overlume::set_camera_frame(r, 0, mismatched_buf->data(), 320, 120, /*frame_id=*/2,
                                            &count_and_delete_release, mismatched_buf));
    EXPECT_EQ(g_release_count, 1);

    overlume::destroy_renderer(r);
}
