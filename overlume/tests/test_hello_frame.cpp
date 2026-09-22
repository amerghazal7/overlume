// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"

#include <EGL/egl.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

namespace {

bool HasGpuEglDevice() {
    EGLDisplay display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (display == EGL_NO_DISPLAY) return false;
    EGLint major = 0;
    EGLint minor = 0;
    return eglInitialize(display, &major, &minor) == EGL_TRUE;
}

class StderrCapture {
public:
    StderrCapture() {
        std::snprintf(path_, sizeof(path_), "/tmp/test_hello_frame_stderr_XXXXXX");
        fd_ = mkstemp(path_);
        savedStderr_ = dup(fileno(stderr));
        std::fflush(stderr);
        dup2(fd_, fileno(stderr));
    }
    std::string Read() {
        std::fflush(stderr);
        dup2(savedStderr_, fileno(stderr));
        close(savedStderr_);
        close(fd_);
        std::ifstream in(path_);
        std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        std::remove(path_);
        return text;
    }

private:
    char path_[64];
    int fd_ = -1;
    int savedStderr_ = -1;
};

size_t CountOccurrences(const std::string& haystack, const std::string& needle) {
    size_t count = 0;
    size_t pos = 0;
    while ((pos = haystack.find(needle, pos)) != std::string::npos) {
        ++count;
        pos += needle.size();
    }
    return count;
}

double RowAverage(const std::vector<uint8_t>& rgb, uint32_t width, uint32_t row) {
    const uint8_t* rowPtr = rgb.data() + static_cast<size_t>(row) * width * 3;
    uint64_t sum = 0;
    for (uint32_t i = 0; i < width * 3; ++i) sum += rowPtr[i];
    return static_cast<double>(sum) / static_cast<double>(width * 3);
}

}

TEST(HelloFrame, RendersDistinctSkyAndGround) {
    constexpr uint32_t kWidth = 320;
    constexpr uint32_t kHeight = 240;

    overlume::RenderConfig config{};
    config.width = kWidth;
    config.height = kHeight;
    config.quality = 1;

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
    EXPECT_TRUE(anyNonZero) << "Rendered frame buffer is entirely zero.";

    const double skyAvg = RowAverage(rgb, kWidth, 5);
    const double groundAvg = RowAverage(rgb, kWidth, kHeight - 5);
    EXPECT_GT(std::fabs(skyAvg - groundAvg), 5.0)
        << "sky rows and ground rows look indistinguishable (sky_avg=" << skyAvg
        << ", ground_avg=" << groundAvg << ")";

    overlume::destroy_renderer(renderer);
}

TEST(CreateRenderer, LogsGlVendorRendererVersionOnce) {
    overlume::RenderConfig config{};
    config.width = 64;
    config.height = 64;
    config.quality = 0;

    StderrCapture capture;
    overlume::VisualRenderer* renderer = overlume::create_renderer(config);
    const std::string captured = capture.Read();

    if (renderer == nullptr) {
        if (!HasGpuEglDevice()) {
            GTEST_SKIP() << "No GPU/EGL device available on this machine.";
        }
        FAIL() << "create_renderer() returned nullptr despite an available GPU/EGL device.";
        return;
    }

    EXPECT_EQ(CountOccurrences(captured, "GL_VENDOR"), 1u) << captured;
    EXPECT_EQ(CountOccurrences(captured, "GL_RENDERER"), 1u) << captured;
    EXPECT_EQ(CountOccurrences(captured, "GL_VERSION"), 1u) << captured;
    EXPECT_EQ(captured.find("GL_VENDOR: (null)"), std::string::npos) << captured;
    EXPECT_EQ(captured.find("GL_RENDERER: (null)"), std::string::npos) << captured;
    EXPECT_EQ(captured.find("GL_VERSION: (null)"), std::string::npos) << captured;
    EXPECT_EQ(captured.find("GL_VENDOR: \n"), std::string::npos) << captured;
    EXPECT_EQ(captured.find("GL_RENDERER: \n"), std::string::npos) << captured;
    EXPECT_EQ(captured.find("GL_VERSION: \n"), std::string::npos) << captured;

    overlume::destroy_renderer(renderer);
}
