// bowl_exposure_probe.cpp — measures the bowl.mat `exposureCompensation`
// value this renderer's ACES tonemap + output OETF actually need to bring an
// UNLIT material's raw baseColor back into a display-referred range (NOT
// renderer.cpp's setExposure(16, 1/500, 100) — bowl.mat is shadingModel:
// unlit, and matinfo confirms its compiled fragment shader never reads
// frameUniforms.exposure; a photometric recompute of that camera's EV100
// gives a ~6.5e-6 Filament exposure factor, five orders of magnitude away
// from 1.56, which is itself the proof setExposure isn't in this path), now
// that camera_textures.cpp samples camera pixels as SRGB8 (decoded to linear
// at sample time) instead of a linear internal format. Replaces the
// empirical eyeball-against-a-bag guess bowl.mat/scene.h's comments
// documented (10.0 -> 1.5) with a real measurement: feed a flat sRGB gray
// frame through the real set_bowl_config+set_camera_frame+render_frame
// pipeline, read the rendered bowl pixels back, and binary-search
// exposureCompensation until mid-gray (sRGB byte 128) round-trips back to
// ~128 in the output. Prints the full 32/64/128/192/224 ramp at the found
// value afterward so the ACES shoulder compression at the bright end is on
// the record, not assumed — shadows are compressed too (not just highlights):
// measured 32->18 and 64->48, ~40% dark, at this same mid-gray-matched value
// (a single scalar compensation cannot invert a nonlinear tonemap curve).
//
// GPU-less box: create_renderer() returns nullptr; this prints one line and
// exits nonzero rather than fabricating a number.
#include "../tests/bowl_gray_probe.hpp"
#include "overlume/api.h"
#include "overlume/scene.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace {

constexpr uint32_t kW = 320, kH = 240;
constexpr int kMidGrayTarget = 128;
constexpr int kToleranceBytes = 1;  // acceptance bar for the search

}  // namespace

int main() {
    overlume::RenderConfig cfg{kW, kH, 1, nullptr, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) {
        std::printf("bowl_exposure_probe: no GPU/EGL, cannot measure\n");
        return 1;
    }

    // Binary search on exposureCompensation for the mid-gray (sRGB byte
    // 128) round trip. The renderer's exposure+ACES chain is monotonically
    // increasing in this input range (verified: no overshoot-then-recede
    // observed across the probed range), so bisection is valid.
    double lo = 0.1, hi = 30.0;
    double best = lo;
    int best_out = -1;
    for (int iter = 0; iter < 20; ++iter) {
        double mid = 0.5 * (lo + hi);
        int out = overlume::testing::render_gray_probe(r, kMidGrayTarget, static_cast<float>(mid));
        if (out < 0) {
            std::printf("bowl_exposure_probe: render failed at compensation=%.4f\n", mid);
            overlume::destroy_renderer(r);
            return 1;
        }
        std::printf("search iter %2d: compensation=%.4f -> mid-gray out=%d\n", iter, mid, out);
        best = mid;
        best_out = out;
        if (out < kMidGrayTarget) {
            lo = mid;
        } else {
            hi = mid;
        }
        if (std::abs(out - kMidGrayTarget) <= kToleranceBytes) break;
    }

    std::printf("\nMEASURED exposure_compensation = %.4f (mid-gray 128 -> %d, tolerance +/-%d)\n",
                best, best_out, kToleranceBytes);

    std::printf("\nFull ramp at compensation=%.4f:\n", best);
    for (int g : {32, 64, 128, 192, 224}) {
        int out = overlume::testing::render_gray_probe(r, static_cast<uint8_t>(g), static_cast<float>(best));
        std::printf("  in=%3d (sRGB byte) -> out=%3d\n", g, out);
    }

    overlume::destroy_renderer(r);
    return 0;
}
