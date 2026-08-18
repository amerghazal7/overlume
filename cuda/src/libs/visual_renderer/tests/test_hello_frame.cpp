// test_hello_frame.cpp — Epic 0 Task 2 failing-first gtest
// (docs/superpowers/plans/2026-08-18-visual-mode.md).
//
// Renders one frame from the node's default pose and checks that something
// plausible came out: the buffer isn't all-zero, and the sky (top rows,
// nothing drawn there) is visibly different from the ground (bottom rows,
// the lit ground plane). GTEST_SKIP()s cleanly on machines with no GPU/EGL
// device instead of failing, matching the repo's GPU-test convention.
#include "visual_renderer/api.h"

#include <EGL/egl.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace {

bool HasGpuEglDevice() {
    EGLDisplay display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (display == EGL_NO_DISPLAY) return false;
    EGLint major = 0;
    EGLint minor = 0;
    return eglInitialize(display, &major, &minor) == EGL_TRUE;
}

double RowAverage(const std::vector<uint8_t>& rgb, uint32_t width, uint32_t row) {
    const uint8_t* rowPtr = rgb.data() + static_cast<size_t>(row) * width * 3;
    uint64_t sum = 0;
    for (uint32_t i = 0; i < width * 3; ++i) sum += rowPtr[i];
    return static_cast<double>(sum) / static_cast<double>(width * 3);
}

}  // namespace

TEST(HelloFrame, RendersDistinctSkyAndGround) {
    constexpr uint32_t kWidth = 320;
    constexpr uint32_t kHeight = 240;

    mpviz::RenderConfig config{};
    config.width = kWidth;
    config.height = kHeight;
    config.quality = 1;

    mpviz::VisualRenderer* renderer = mpviz::create_renderer(config);
    if (renderer == nullptr) {
        if (!HasGpuEglDevice()) {
            GTEST_SKIP() << "No GPU/EGL device available on this machine.";
        }
        FAIL() << "create_renderer() returned nullptr despite an available GPU/EGL device.";
        return;
    }

    // The node's default pose (docs/superpowers/plans/2026-08-18-visual-mode.md, Task 2 Step 1).
    mpviz::CameraPose pose{};
    pose.eye[0] = -4.0;
    pose.eye[1] = 0.0;
    pose.eye[2] = 3.5;
    pose.target[0] = 2.0;
    pose.target[1] = 0.0;
    pose.target[2] = -0.5;
    // eye->target pitches down ~33.7 deg from horizontal; vfov must exceed
    // 2x that (~67.4 deg) or the whole frustum stays below the horizon and
    // no sky pixels exist to compare against the ground at all.
    pose.vfov_deg = 80.0;

    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, 0);
    mpviz::FrameView view{rgb.data(), kWidth, kHeight};

    ASSERT_TRUE(mpviz::render_frame(renderer, pose, view));

    const bool anyNonZero = std::any_of(rgb.begin(), rgb.end(), [](uint8_t v) { return v != 0; });
    EXPECT_TRUE(anyNonZero) << "Rendered frame buffer is entirely zero.";

    const double skyAvg = RowAverage(rgb, kWidth, 5);              // near top
    const double groundAvg = RowAverage(rgb, kWidth, kHeight - 5);  // near bottom
    EXPECT_GT(std::fabs(skyAvg - groundAvg), 5.0)
        << "sky rows and ground rows look indistinguishable (sky_avg=" << skyAvg
        << ", ground_avg=" << groundAvg << ")";

    mpviz::destroy_renderer(renderer);
}
