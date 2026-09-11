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
#include "bowl_gray_probe.hpp"

#include <algorithm>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

namespace {

}  // namespace

// The measurement this default is calibrated against: mid-gray (sRGB byte
// 128) round-trips through SRGB8 camera-texture decode + this renderer's
// fixed exposure + ACES tonemap + output OETF to within a few bytes of
// itself. +/-8 (not the probe's own tighter +/-1) gives this regression
// test headroom against ordinary driver/AA noise while still catching a
// real regression (the pre-fix pipeline missed by ~50+ bytes here, per the
// scene.h/bowl.mat measurement comments).
TEST(BowlExposureCalibration, MidGrayRoundTripsWithinToleranceAtShippedDefault) {
    mpviz::RenderConfig cfg{mpviz::testing::kGrayProbeW, mpviz::testing::kGrayProbeH, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    int out = mpviz::testing::render_gray_probe(r, 128);
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
    mpviz::RenderConfig cfg{mpviz::testing::kGrayProbeW, mpviz::testing::kGrayProbeH, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    int out = mpviz::testing::render_gray_probe(r, 224);
    ASSERT_GE(out, 0) << "no bowl-surface pixels found in the rendered frame";
    // Measured 224 -> 208 (bowl_exposure_probe.cpp): well below a clipped
    // 255, and below the input itself -- the ACES shoulder rolling off
    // highlights, not a bug.
    EXPECT_LT(out, 224) << "expected the ACES shoulder to compress the bright end, not "
                            "round-trip it exactly";
    EXPECT_LT(out, 250) << "expected no near-white clipping at this compensation";
    mpviz::destroy_renderer(r);
}
