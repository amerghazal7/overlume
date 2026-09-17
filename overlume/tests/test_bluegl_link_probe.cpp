// test_bluegl_link_probe.cpp — named runtime probe on bluegl::bind()'s
// hand-declared signature (Step (i), VM-037).
//
// renderer.cpp hand-declares `namespace bluegl { int bind(); void unbind(); }`
// at global scope so the linker resolves the real ::bluegl::bind()/unbind()
// symbols without vendoring the generated BlueGL.h (see that file's own
// comment for why). Nothing checks that the hand-written declarations still
// match the real library's signature -- the Itanium ABI does not mangle
// return types, so an upstream bluegl change to e.g. void/bool would link
// silently and leave `if (bluegl::bind() != 0)` (renderer.cpp) reading
// garbage instead of failing to compile or link.
//
// A true compile-time signature check isn't possible without vendoring the
// real BlueGL.h (the whole point of hand-declaring was to avoid that), so
// this is a named RUNTIME probe: it exercises the exact call path
// renderer.cpp uses (create_renderer() -> HeadlessEglPlatform::createDriver()
// -> bluegl::bind()) and asserts the result is consistent with the assumed
// signature -- render_frame() must produce real, non-garbage pixels. A
// silently-mismatched calling convention would corrupt the GL function
// table and most likely crash or produce an all-black frame, not a clean
// nonzero return, so this test names the assumption explicitly rather than
// relying on HelloFrame to catch it as a side effect.
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

    // create_renderer() returns nullptr on ANY failure inside
    // HeadlessEglPlatform::createDriver() -- including `bluegl::bind() != 0`
    // -- so a nullptr here on a machine with a real GPU/EGL device already
    // means bind() (or something upstream of it) misbehaved.
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
    EXPECT_TRUE(anyNonZero)
        << "Rendered frame buffer is entirely zero -- a corrupted GL function "
           "table from a bluegl signature mismatch is one plausible cause.";

    overlume::destroy_renderer(renderer);
}
