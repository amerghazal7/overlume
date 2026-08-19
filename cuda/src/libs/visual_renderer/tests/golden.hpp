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

}  // namespace mpviz::testing
