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
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace {

constexpr uint32_t kW = 320, kH = 240;
constexpr int kMidGrayTarget = 128;
constexpr int kToleranceBytes = 1;  // acceptance bar for the search

// Same overhead-camera-over-a-small-bowl geometry
// tests/test_bowl.cpp's RenderFrameWithBowlConfiguredProducesSentinelPixels
// uses (a wide-FOV eye straight down onto the whole bowl) -- proven to put
// a large, easily-isolated fraction of the frame on the bowl's sampled
// surface.
mpviz::BowlConfig make_bowl_config(const mpviz::CameraExtrinsics& ext,
                                    const mpviz::CameraIntrinsics& in, const uint32_t& w,
                                    const uint32_t& h, float exposure_compensation) {
    mpviz::BowlConfig bc{};
    bc.camera_count = 1;
    bc.extrinsics = &ext;
    bc.intrinsics = &in;
    bc.cam_width = &w;
    bc.cam_height = &h;
    bc.bowl_R0 = 0.5;
    bc.bowl_k = 0.3;
    bc.bowl_Rmax = 4.0;
    bc.feather_margin = 5.0;
    // Pure saturated green: nothing this test samples through the bowl
    // (flat gray camera frames) ever renders green-dominant, so "green
    // dominant" cleanly means "sky, not bowl" below -- unlike gray
    // sky_color values used elsewhere, which would be indistinguishable
    // from a dim gray bowl sample.
    bc.sky_color[0] = 0.0f;
    bc.sky_color[1] = 1.0f;
    bc.sky_color[2] = 0.0f;
    bc.exposure_compensation = exposure_compensation;
    return bc;
}

// Renders one flat sRGB `gray_byte` camera frame through the bowl at
// `exposure_compensation` and returns the mean output byte over pixels that
// are the bowl's sampled surface. The naive near-neutral filter (`hi - lo <=
// 6`) silently drops most of the bowl at the bright end -- the ACES shoulder
// pushes a flat gray bowl 7-20 bytes off neutral there, so at gray=224 it
// kept only 563 of ~26,300 bowl pixels (0.7% of the frame, mean 192, an
// edge/fringe subsample) instead of the bulk 25,769 px (mean 208), one AA/
// driver nudge from keeping zero pixels and failing spuriously. A pure
// "exclude the green sky" filter is NOT a safe replacement here: this probe's
// render target is NOT fully covered by the bowl mesh + its sky_color fill --
// a substantial fraction (measured ~66% at this pose) is the renderer's own
// clear color (kFallbackTheme's dark navy sky, since this probe runs with no
// theme_assets_dir), which is not green-dominant and would be silently
// counted as "bowl" by a green-only exclusion (verified: it moved the
// mid-gray round-trip target from 1.56 to ~18.8, a 12x error). Widening the
// near-neutral band to `hi - lo <= 24` instead is what actually holds: the
// measured background corner is (35, 43, 80) (hi-lo=45, safely excluded)
// while the bowl's own bright-end shoulder deviation tops out at 20 (safely
// included) across the whole 32/64/128/192/224 ramp.
// Returns -1 if no such pixel is found (config rejected, or GPU-less).
int render_gray_probe(mpviz::VisualRenderer* r, const mpviz::CameraExtrinsics& ext,
                       const mpviz::CameraIntrinsics& in, uint8_t gray_byte,
                       float exposure_compensation) {
    uint32_t w = kW, h = kH;
    mpviz::BowlConfig bc = make_bowl_config(ext, in, w, h, exposure_compensation);
    if (!mpviz::set_bowl_config(r, bc)) return -1;
    if (!mpviz::set_bowl_visible(r, true)) return -1;

    std::vector<uint8_t> cam_pixels(static_cast<size_t>(w) * h * 3, gray_byte);
    if (!mpviz::set_camera_frame(r, 0, cam_pixels.data(), w, h, /*frame_id=*/1)) return -1;

    mpviz::CameraPose pose{{0, -6, 6}, {0, 0, 0}, 70.0};
    std::vector<uint8_t> buf(static_cast<size_t>(kW) * kH * 3);
    mpviz::FrameView view{buf.data(), kW, kH};
    if (!mpviz::render_frame(r, pose, view)) return -1;

    long sum = 0;
    long count = 0;
    for (size_t i = 0; i < buf.size(); i += 3) {
        int R = buf[i], G = buf[i + 1], B = buf[i + 2];
        int lo = std::min({R, G, B});
        int hi = std::max({R, G, B});
        if (hi - lo <= 24) {
            sum += (R + G + B);
            count += 3;
        }
    }
    if (count == 0) return -1;
    return static_cast<int>(sum / count);
}

}  // namespace

int main() {
    mpviz::RenderConfig cfg{kW, kH, 1, nullptr, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) {
        std::printf("bowl_exposure_probe: no GPU/EGL, cannot measure\n");
        return 1;
    }

    mpviz::CameraExtrinsics ext{{1, 0, 0, 0, -1, 0, 0, 0, -1}, {0, 0, 10.0}};
    mpviz::CameraIntrinsics in{200, 200, 160, 120, {0, 0, 0, 0, 0}};

    // Binary search on exposureCompensation for the mid-gray (sRGB byte
    // 128) round trip. The renderer's exposure+ACES chain is monotonically
    // increasing in this input range (verified: no overshoot-then-recede
    // observed across the probed range), so bisection is valid.
    double lo = 0.1, hi = 30.0;
    double best = lo;
    int best_out = -1;
    for (int iter = 0; iter < 20; ++iter) {
        double mid = 0.5 * (lo + hi);
        int out = render_gray_probe(r, ext, in, kMidGrayTarget, static_cast<float>(mid));
        if (out < 0) {
            std::printf("bowl_exposure_probe: render failed at compensation=%.4f\n", mid);
            mpviz::destroy_renderer(r);
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
        int out = render_gray_probe(r, ext, in, static_cast<uint8_t>(g), static_cast<float>(best));
        std::printf("  in=%3d (sRGB byte) -> out=%3d\n", g, out);
    }

    mpviz::destroy_renderer(r);
    return 0;
}
