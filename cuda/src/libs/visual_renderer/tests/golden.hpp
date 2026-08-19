// golden.hpp — Epic 1 Task 2 Step 9: shared render+compare helper used by
// every later epic's golden-image tests. NOT a gtest file itself (see
// CMakeLists.txt: golden.cpp is compiled as a plain extra source into every
// other test binary instead of its own gtest executable).
#pragma once

#include "visual_renderer/api.h"

namespace mpviz::testing {

// Renders ONE frame of `r`'s current active scene/theme state from `pose`
// at a fixed 320x240 (this epic's goldens are all committed at that size —
// spec: "goldens render at a fixed 320x240 purely for CI speed/
// determinism"), writes it to `out_png_path`, and returns a block-SSIM
// score in [0,1] against `golden_png_path` (0.0 if the golden doesn't exist
// yet or doesn't match 320x240 — first run of a new golden always fails
// loudly, never silently "passes" with nothing to compare against).
//
// The harness does NOT create a renderer, and does NOT call set_scene/
// set_theme — it only calls render_frame(r, pose, ...) and compares the
// result. The caller owns create_renderer()/destroy_renderer(), and must
// have already driven `r` into whatever scene/theme/transition state it
// wants a golden of via set_scene()/set_theme() BEFORE calling this.
//
// Returns -1.0 if `r` is null — same no-GPU/EGL convention as
// test_hello_frame.cpp; callers are expected to GTEST_SKIP() right after
// create_renderer() returns null, before ever reaching this call, same as
// every other renderer test in this codebase.
double render_and_compare(mpviz::VisualRenderer* r, const mpviz::CameraPose& pose,
                           const char* golden_png_path, const char* out_png_path);

// Legibility stats for a rendered frame -- the numeric form of the plan's
// Step 7a AC ("clay surfaces read as mid-gray-ish, not clipped white or
// crushed black"). `top_third_mean`/`bottom_third_mean` are luminance means
// of the top/bottom thirds of the frame (sky-ish vs. ground-ish for this
// epic's fixed camera pose looking at the horizon) -- not a scene-aware
// segmentation, just enough to catch "the flat sky backdrop is brighter
// than the supposedly sunlit ground" the way a human glancing at the image
// would. Returns all-zero stats if the PNG can't be loaded.
struct FrameStats {
    double mean = 0.0;
    int distinct_levels = 0;
    double top_third_mean = 0.0;
    double bottom_third_mean = 0.0;
    // sky_row_mean: rows [10,40) -- deep in the flat sky/clear-color
    // backdrop, above any horizon effect, for this epic's fixed 320x240 /
    // CameraPose{{0,-8,4},{0,0,0},60} test setup (same "not scene-aware,
    // just matches this fixed pose" caveat as top/bottom_third_mean above).
    // horizon_row_mean: rows [50,60) -- the far edge of the ground plane
    // (kGroundHalfExtent, only 40m across) as it meets the sky, where
    // distance-fog opacity is at its highest for any on-plane ray (though,
    // being a *finite* plane, never near-total extinction the way a true
    // infinite ground would give). Every shipped theme authors palette.fog
    // == palette.sky (spec §4.3 treats them as one token), so a correctly-
    // scaled fog should pull this band noticeably toward sky_row_mean --
    // this is the guard the epic1 Task 2 fog-scale review round asked for
    // (a flat mean-band check can't express "the ground fades toward the
    // sky", only "isn't crushed/clipped"; see renderer.cpp's setFogOptions
    // comment for why exact equality isn't reachable by color scale alone).
    double sky_row_mean = 0.0;
    double horizon_row_mean = 0.0;
};
FrameStats analyze_png(const char* png_path);

}  // namespace mpviz::testing
