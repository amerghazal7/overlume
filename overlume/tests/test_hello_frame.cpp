// test_hello_frame.cpp — renders one frame from the node's default pose
// and checks that something plausible came out: the buffer isn't all-zero,
// and the sky (top rows, nothing drawn there) is visibly different from
// the ground (bottom rows, the lit ground plane). GTEST_SKIP()s cleanly on
// machines with no GPU/EGL device instead of failing, matching the repo's
// GPU-test convention.
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

// Redirects stderr to a temp file for the lifetime of the object, restoring
// it (and reading the captured text) on Read(). Process-local (fd-level
// dup2), safe here because gtest_discover_tests runs each TEST in its own
// process invocation (--gtest_filter), never two tests in one process.
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

}  // namespace

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

    // The node's default pose (docs/plans/2026-08-18-visual-mode.md, Task 2 Step 1).
    overlume::CameraPose pose{};
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
    overlume::FrameView view{rgb.data(), kWidth, kHeight};

    ASSERT_TRUE(overlume::render_frame(renderer, pose, view));

    const bool anyNonZero = std::any_of(rgb.begin(), rgb.end(), [](uint8_t v) { return v != 0; });
    EXPECT_TRUE(anyNonZero) << "Rendered frame buffer is entirely zero.";

    const double skyAvg = RowAverage(rgb, kWidth, 5);              // near top
    const double groundAvg = RowAverage(rgb, kWidth, kHeight - 5);  // near bottom
    EXPECT_GT(std::fabs(skyAvg - groundAvg), 5.0)
        << "sky rows and ground rows look indistinguishable (sky_avg=" << skyAvg
        << ", ground_avg=" << groundAvg << ")";

    overlume::destroy_renderer(renderer);
}

// Step (h), VM-037: create_renderer() (via HeadlessEglPlatform::createDriver(),
// the only place the GL context is current on the same thread as the bluegl
// binding) logs GL_VENDOR/GL_RENDERER/GL_VERSION once, so a recorded
// render_ms budget can be attributed to real hardware vs. Mesa llvmpipe
// (which also passes HasGpuEglDevice()). GTEST_SKIP()s per HasGpuEglDevice(),
// same convention as RendersDistinctSkyAndGround above.
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
    // Each label must be followed by a real (non-null, non-empty) value on
    // its own line -- catches the label being logged while the underlying
    // bluegl_glGetString() call itself silently returns null (e.g. from the
    // wrong thread; see the call site's own comment for that exact history).
    EXPECT_EQ(captured.find("GL_VENDOR: (null)"), std::string::npos) << captured;
    EXPECT_EQ(captured.find("GL_RENDERER: (null)"), std::string::npos) << captured;
    EXPECT_EQ(captured.find("GL_VERSION: (null)"), std::string::npos) << captured;
    EXPECT_EQ(captured.find("GL_VENDOR: \n"), std::string::npos) << captured;
    EXPECT_EQ(captured.find("GL_RENDERER: \n"), std::string::npos) << captured;
    EXPECT_EQ(captured.find("GL_VERSION: \n"), std::string::npos) << captured;

    overlume::destroy_renderer(renderer);
}
