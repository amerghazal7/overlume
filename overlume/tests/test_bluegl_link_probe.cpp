// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"

#include <EGL/egl.h>
#include <gtest/gtest.h>

#include <algorithm>
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

}  // namespace

TEST(BlueglLinkProbe, BindReturnsZeroOnSuccessfulContextAndFrameRenders) {
    constexpr uint32_t kWidth = 64;
    constexpr uint32_t kHeight = 64;

    overlume::RenderConfig config{};
    config.width = kWidth;
    config.height = kHeight;
    config.quality = 0;

    overlume::VisualRenderer* renderer = overlume::create_renderer(config);
    if (renderer == nullptr) {
        if (!HasGpuEglDevice()) {
            GTEST_SKIP() << "No GPU/EGL device available on this machine.";
        }
        FAIL() << "create_renderer() returned nullptr despite an available GPU/EGL device.";
        return;
    }

    overlume::CameraPose pose{};
    pose.eye[0] = -4.0;
    pose.eye[1] = 0.0;
    pose.eye[2] = 3.5;
    pose.target[0] = 2.0;
    pose.target[1] = 0.0;
    pose.target[2] = -0.5;
    pose.vfov_deg = 80.0;

    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, 0);
    overlume::FrameView view{rgb.data(), kWidth, kHeight};

    ASSERT_TRUE(overlume::render_frame(renderer, pose, view));

    const bool anyNonZero = std::any_of(rgb.begin(), rgb.end(), [](uint8_t v) { return v != 0; });
    EXPECT_TRUE(anyNonZero) << "Rendered frame buffer is entirely zero -- a corrupted GL function "
                               "table from a bluegl signature mismatch is one plausible cause.";

    overlume::destroy_renderer(renderer);
}
