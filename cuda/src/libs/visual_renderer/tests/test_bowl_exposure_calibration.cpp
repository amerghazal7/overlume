// test_bowl_exposure_calibration.cpp — bowl color fidelity fix (2026-09-11).
// Locks in the MEASURED BowlConfig::exposure_compensation default
// (tools/bowl_exposure_probe.cpp's gray-ramp binary search; see scene.h's
// own comment) against regressions: a future change to camera_textures.cpp's
// SRGB8 format, renderer.cpp's fixed exposure/ACES tonemap, or bowl.mat's
// exposureCompensation wiring that silently re-breaks the round trip should
// fail here, not just look "a bit washed" in a golden diff.
//
// Same overhead-camera-over-a-small-bowl geometry as
// RenderFrameWithBowlConfiguredProducesSentinelPixels (test_bowl.cpp) --
// proven to put a large, easily-isolated fraction of the frame on the
// bowl's sampled surface.
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

#include "test_paths.hpp"

#include <algorithm>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

namespace {

constexpr uint32_t kW = 320, kH = 240;

// Renders one flat sRGB `gray_byte` camera frame through the bowl at the
// SHIPPED default exposure_compensation (BowlConfig{}'s own default member
// initializer -- not overridden here, so this test breaks if that default
// ever drifts) and returns the mean output byte over the bowl's sampled
// (near-neutral) pixels. -1 if none found.
int render_gray(mpviz::VisualRenderer* r, uint8_t gray_byte) {
    mpviz::CameraExtrinsics ext{{1, 0, 0, 0, -1, 0, 0, 0, -1}, {0, 0, 10.0}};
    mpviz::CameraIntrinsics in{200, 200, 160, 120, {0, 0, 0, 0, 0}};
    uint32_t w = kW, h = kH;
    mpviz::BowlConfig bc{};  // exposure_compensation left at its shipped default
    bc.camera_count = 1;
    bc.extrinsics = &ext;
    bc.intrinsics = &in;
    bc.cam_width = &w;
    bc.cam_height = &h;
    bc.bowl_R0 = 0.5;
    bc.bowl_k = 0.3;
    bc.bowl_Rmax = 4.0;
    bc.feather_margin = 5.0;
    // Saturated green sky: cleanly distinguishable from any gray bowl
    // sample, so "green-dominant" unambiguously means "sky, not bowl".
    bc.sky_color[0] = 0.0f;
    bc.sky_color[1] = 1.0f;
    bc.sky_color[2] = 0.0f;

    if (!mpviz::set_bowl_config(r, bc)) return -1;
    if (!mpviz::set_bowl_visible(r, true)) return -1;

    std::vector<uint8_t> cam_pixels(static_cast<size_t>(w) * h * 3, gray_byte);
    if (!mpviz::set_camera_frame(r, 0, cam_pixels.data(), w, h, /*frame_id=*/1)) return -1;

    mpviz::CameraPose pose{{0, -6, 6}, {0, 0, 0}, 70.0};
    std::vector<uint8_t> buf(static_cast<size_t>(kW) * kH * 3);
    mpviz::FrameView view{buf.data(), kW, kH};
    if (!mpviz::render_frame(r, pose, view)) return -1;

    long sum = 0, count = 0;
    for (size_t i = 0; i < buf.size(); i += 3) {
        int R = buf[i], G = buf[i + 1], B = buf[i + 2];
        int lo = std::min({R, G, B});
        int hi = std::max({R, G, B});
        // Widened from `hi - lo <= 6`: that tighter band silently dropped
        // most of the bowl above ~gray 200 (the ACES shoulder pushes a flat
        // gray bowl 7-20 bytes off neutral there), keeping a <1% edge
        // subsample one AA/driver nudge from failing spuriously -- see
        // tools/bowl_exposure_probe.cpp's render_gray_probe() comment for the
        // measured counts and for why a pure "exclude the green sky" filter
        // is NOT a safe alternative (this render target's uncovered area is
        // the renderer's own non-green clear color, not sky_color). 24 is
        // still comfortably under that clear color's own hi-lo (measured 45).
        if (hi - lo <= 24) {
            sum += (R + G + B);
            count += 3;
        }
    }
    if (count == 0) return -1;
    return static_cast<int>(sum / count);
}

}  // namespace

// The measurement this default is calibrated against: mid-gray (sRGB byte
// 128) round-trips through SRGB8 camera-texture decode + this renderer's
// fixed exposure + ACES tonemap + output OETF to within a few bytes of
// itself. +/-8 (not the probe's own tighter +/-1) gives this regression
// test headroom against ordinary driver/AA noise while still catching a
// real regression (the pre-fix pipeline missed by ~50+ bytes here, per the
// scene.h/bowl.mat measurement comments).
TEST(BowlExposureCalibration, MidGrayRoundTripsWithinToleranceAtShippedDefault) {
    mpviz::RenderConfig cfg{kW, kH, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    int out = render_gray(r, 128);
    ASSERT_GE(out, 0) << "no bowl-surface pixels found in the rendered frame";
    EXPECT_NEAR(out, 128, 8) << "mid-gray no longer round-trips at the shipped "
                                "exposure_compensation default -- see scene.h's "
                                "BowlConfig::exposure_compensation comment";
    mpviz::destroy_renderer(r);
}

// ACES shoulder behavior, recorded honestly (task requirement): the bright
// end does NOT round-trip the way mid-gray does -- this asserts the
// DIRECTION (compressed toward mid-gray, i.e. undershoots 224) rather than
// a tight tolerance, so a real tonemap/compensation change is still free to
// move this measured number without breaking a test asserting the wrong
// physics.
TEST(BowlExposureCalibration, BrightGrayIsShoulderCompressedNotClipped) {
    mpviz::RenderConfig cfg{kW, kH, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    int out = render_gray(r, 224);
    ASSERT_GE(out, 0) << "no bowl-surface pixels found in the rendered frame";
    // Measured 224 -> 208 (bowl_exposure_probe.cpp): well below a clipped
    // 255, and below the input itself -- the ACES shoulder rolling off
    // highlights, not a bug.
    EXPECT_LT(out, 224) << "expected the ACES shoulder to compress the bright end, not "
                            "round-trip it exactly";
    EXPECT_LT(out, 250) << "expected no near-white clipping at this compensation";
    mpviz::destroy_renderer(r);
}
