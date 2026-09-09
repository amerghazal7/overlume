// tests/test_renderer_projection.cpp — Epic 3 Task 4 (VM-031) Step 1:
// mpviz::project_to_screen() against the camera state the most recent
// render_frame() call set. Same "create_renderer() must succeed on this
// box" convention as tests/test_hud_overlay.cpp's own golden (this repo's
// CI box has a working GPU/EGL device) -- not the older GTEST_SKIP-on-no-GPU
// shape test_hello_frame.cpp uses.
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace {

mpviz::VisualRenderer* MakeRenderer(uint32_t width, uint32_t height) {
    mpviz::RenderConfig config{};
    config.width = width;
    config.height = height;
    config.quality = 0;
    config.theme_assets_dir = nullptr;  // compiled-in fallback theme -- no asset dir needed
    config.initial_theme = nullptr;
    return mpviz::create_renderer(config);
}

// The node's own default hello-frame pose (visual-mode.md Task 2 Step 1),
// same one test_hello_frame.cpp uses -- an arbitrary-but-known eye/target/fov.
mpviz::CameraPose HelloFramePose() {
    mpviz::CameraPose pose{};
    pose.eye[0] = -4.0;
    pose.eye[1] = 0.0;
    pose.eye[2] = 3.5;
    pose.target[0] = 2.0;
    pose.target[1] = 0.0;
    pose.target[2] = -0.5;
    pose.vfov_deg = 80.0;
    return pose;
}

}  // namespace

TEST(ProjectToScreen, PointAtCameraTargetProjectsNearCenter) {
    constexpr uint32_t kW = 320, kH = 240;
    mpviz::VisualRenderer* r = MakeRenderer(kW, kH);
    ASSERT_NE(r, nullptr);

    const mpviz::CameraPose pose = HelloFramePose();
    std::vector<uint8_t> rgb(static_cast<size_t>(kW) * kH * 3, 0);
    mpviz::FrameView view{rgb.data(), kW, kH};
    ASSERT_TRUE(mpviz::render_frame(r, pose, view));

    const mpviz::Vec3 target{pose.target[0], pose.target[1], pose.target[2]};
    float x = -1.0f, y = -1.0f;
    ASSERT_TRUE(mpviz::project_to_screen(r, target, &x, &y));
    EXPECT_NEAR(x, 0.5f, 0.02f);
    EXPECT_NEAR(y, 0.5f, 0.02f);

    mpviz::destroy_renderer(r);
}

TEST(ProjectToScreen, PointBehindCameraReturnsFalse) {
    constexpr uint32_t kW = 320, kH = 240;
    mpviz::VisualRenderer* r = MakeRenderer(kW, kH);
    ASSERT_NE(r, nullptr);

    const mpviz::CameraPose pose = HelloFramePose();
    std::vector<uint8_t> rgb(static_cast<size_t>(kW) * kH * 3, 0);
    mpviz::FrameView view{rgb.data(), kW, kH};
    ASSERT_TRUE(mpviz::render_frame(r, pose, view));

    // eye + (eye - target): straight behind the eye, opposite the look
    // direction, at the same distance the target sits in front of it.
    const mpviz::Vec3 behind{pose.eye[0] + (pose.eye[0] - pose.target[0]),
                              pose.eye[1] + (pose.eye[1] - pose.target[1]),
                              pose.eye[2] + (pose.eye[2] - pose.target[2])};
    float x = 0.0f, y = 0.0f;
    EXPECT_FALSE(mpviz::project_to_screen(r, behind, &x, &y));

    mpviz::destroy_renderer(r);
}

TEST(ProjectToScreen, PointOutsideFrustumReturnsFalse) {
    constexpr uint32_t kW = 320, kH = 240;
    mpviz::VisualRenderer* r = MakeRenderer(kW, kH);
    ASSERT_NE(r, nullptr);

    const mpviz::CameraPose pose = HelloFramePose();
    std::vector<uint8_t> rgb(static_cast<size_t>(kW) * kH * 3, 0);
    mpviz::FrameView view{rgb.data(), kW, kH};
    ASSERT_TRUE(mpviz::render_frame(r, pose, view));

    // Same depth as the target (in front of the camera, w > 0) but 500m off
    // to the side -- far outside an 80deg vfov's horizontal extent at ~7m.
    const mpviz::Vec3 way_off_to_the_side{pose.target[0], pose.target[1] + 500.0,
                                           pose.target[2]};
    float x = 0.0f, y = 0.0f;
    EXPECT_FALSE(mpviz::project_to_screen(r, way_off_to_the_side, &x, &y));

    mpviz::destroy_renderer(r);
}
