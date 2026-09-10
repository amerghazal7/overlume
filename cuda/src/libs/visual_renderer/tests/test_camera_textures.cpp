// test_camera_textures.cpp — VM-090 (unified-engine migration Task 1,
// ADR-0005): the POD-boundary camera-texture mechanism (set_bowl_config(),
// set_camera_frame(), set_bowl_visible(), set_camera_motion_delta()). No
// bowl mesh/material exists yet (Task 2) -- these tests only exercise the
// camera-texture allocation/upload/dirty-tracking half.
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

#include "camera_textures_test_hooks.hpp"
#include "test_paths.hpp"

#include <vector>

#include <gtest/gtest.h>

namespace {

mpviz::BowlConfig one_camera_config(const mpviz::CameraExtrinsics& ext,
                                     const mpviz::CameraIntrinsics& in, const uint32_t& w,
                                     const uint32_t& h) {
    mpviz::BowlConfig bc{};
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

}  // namespace

// ── Step 0: set_camera_frame() before any BowlConfig is a safe no-op ───────

TEST(CameraTextures, SetCameraFrameBeforeBowlConfigIsNonFatal) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    std::vector<uint8_t> pixels(64 * 48 * 3, 128);
    EXPECT_FALSE(mpviz::set_camera_frame(r, 0, pixels.data(), 64, 48, 1));  // no BowlConfig yet
    mpviz::destroy_renderer(r);
}

// ── Step 1: set_bowl_config() allocates persistent textures; ───────────────
//    set_camera_frame() uploads into them.

TEST(CameraTextures, SetCameraFrameUploadsAfterBowlConfig) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    mpviz::CameraExtrinsics ext{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0.5}};
    mpviz::CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    uint32_t w = 320, h = 240;
    mpviz::BowlConfig bc = one_camera_config(ext, in, w, h);
    ASSERT_TRUE(mpviz::set_bowl_config(r, bc));
    std::vector<uint8_t> pixels(320 * 240 * 3, 200);
    EXPECT_TRUE(mpviz::set_camera_frame(r, 0, pixels.data(), 320, 240, 1));
    EXPECT_FALSE(mpviz::set_camera_frame(r, 1, pixels.data(), 320, 240, 1));  // cam_idx out of range
    mpviz::destroy_renderer(r);
}

// ── Step 2: dirty tracking skips a re-upload when frame_id repeats ─────────

TEST(CameraTextures, RepeatedFrameIdSkipsReupload) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    mpviz::CameraExtrinsics ext{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0.5}};
    mpviz::CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    uint32_t w = 320, h = 240;
    mpviz::BowlConfig bc = one_camera_config(ext, in, w, h);
    ASSERT_TRUE(mpviz::set_bowl_config(r, bc));
    std::vector<uint8_t> pixels(320 * 240 * 3, 200);

    mpviz::set_camera_frame(r, 0, pixels.data(), 320, 240, /*frame_id=*/1);
    auto count_after_first = mpviz::testing::camera_frame_upload_count(r, 0);
    mpviz::set_camera_frame(r, 0, pixels.data(), 320, 240, /*frame_id=*/1);  // same frame_id
    EXPECT_EQ(mpviz::testing::camera_frame_upload_count(r, 0), count_after_first);  // no re-upload
    mpviz::set_camera_frame(r, 0, pixels.data(), 320, 240, /*frame_id=*/2);  // new frame_id
    EXPECT_EQ(mpviz::testing::camera_frame_upload_count(r, 0), count_after_first + 1);  // uploads
    mpviz::destroy_renderer(r);
}

// ── Step 2b: set_camera_motion_delta() gated on BowlConfig ─────────────────

TEST(CameraTextures, SetCameraMotionDeltaGatedOnBowlConfig) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const double identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    EXPECT_FALSE(mpviz::set_camera_motion_delta(r, 0, identity));  // no BowlConfig yet

    mpviz::CameraExtrinsics ext{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0.5}};
    mpviz::CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    uint32_t w = 320, h = 240;
    mpviz::BowlConfig bc = one_camera_config(ext, in, w, h);
    ASSERT_TRUE(mpviz::set_bowl_config(r, bc));

    EXPECT_TRUE(mpviz::set_camera_motion_delta(r, 0, identity));
    EXPECT_FALSE(mpviz::set_camera_motion_delta(r, 1, identity));  // cam_idx out of range
    mpviz::destroy_renderer(r);
}

// ── set_bowl_visible() gated on BowlConfig (Interfaces block; consumed by
//    Task 4's mode dispatch) -- same no-op-before-config convention as the
//    other three entry points above.

TEST(CameraTextures, SetBowlVisibleGatedOnBowlConfig) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    EXPECT_FALSE(mpviz::set_bowl_visible(r, true));  // no BowlConfig yet

    mpviz::CameraExtrinsics ext{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0.5}};
    mpviz::CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    uint32_t w = 320, h = 240;
    mpviz::BowlConfig bc = one_camera_config(ext, in, w, h);
    ASSERT_TRUE(mpviz::set_bowl_config(r, bc));

    EXPECT_TRUE(mpviz::set_bowl_visible(r, true));
    EXPECT_TRUE(mpviz::set_bowl_visible(r, false));
    mpviz::destroy_renderer(r);
}

// ── set_bowl_config() rejects an invalid camera_count -- same
//    "missing/invalid config renders nothing" convention as
//    set_environment_source's null-path (Interfaces block).

TEST(CameraTextures, SetBowlConfigRejectsInvalidCameraCount) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    mpviz::CameraExtrinsics ext{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0.5}};
    mpviz::CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    uint32_t w = 320, h = 240;

    mpviz::BowlConfig zero = one_camera_config(ext, in, w, h);
    zero.camera_count = 0;
    EXPECT_FALSE(mpviz::set_bowl_config(r, zero));

    mpviz::BowlConfig too_many = one_camera_config(ext, in, w, h);
    too_many.camera_count = mpviz::kMaxBowlCameras + 1;
    EXPECT_FALSE(mpviz::set_bowl_config(r, too_many));

    EXPECT_FALSE(mpviz::set_bowl_config(nullptr, zero));
    mpviz::destroy_renderer(r);
}
