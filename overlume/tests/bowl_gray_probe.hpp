// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// bowl_gray_probe.hpp — the ONE gray-ramp probe harness shared by
// tests/test_bowl_exposure_calibration.cpp (the regression pin) and
// tools/bowl_exposure_probe.cpp (the measurement tool), hoisted per the
// test_paths.hpp precedent so the two can't drift apart. Filament-free:
// api.h/scene.h only.
//
// Geometry: overhead camera over a small bowl (the test_bowl.cpp sentinel
// pose) — proven to put a large, easily-isolated fraction of the frame on
// the bowl's sampled surface.
//
// kNearNeutralBand (hi-lo <= 24): the naive `<= 6` filter silently drops
// most of the bowl at the bright end — the ACES shoulder pushes a flat gray
// bowl 7-20 bytes off neutral there (at gray=224 it kept 563 of ~26,300
// bowl pixels, an edge/fringe subsample one AA/driver nudge from empty). A
// pure "exclude the green sky" filter is NOT a safe alternative: this
// probe's render target is ~66% the renderer's own clear color (dark navy,
// not green-dominant; measured corner (35,43,80), hi-lo=45) — counting it
// as "bowl" moved the mid-gray round-trip target from 1.56 to ~18.8, a 12x
// error. 24 includes the bowl's worst shoulder deviation (20) and excludes
// the clear color (45).
#pragma once

#include "overlume/api.h"
#include "overlume/scene.h"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace overlume::testing {

inline constexpr uint32_t kGrayProbeW = 320, kGrayProbeH = 240;
inline constexpr int kNearNeutralBand = 24;

inline overlume::CameraExtrinsics gray_probe_extrinsics() {
    return {{1, 0, 0, 0, -1, 0, 0, 0, -1}, {0, 0, 10.0}};
}
inline overlume::CameraIntrinsics gray_probe_intrinsics() {
    return {200, 200, 160, 120, {0, 0, 0, 0, 0}};
}

// Renders one flat sRGB `gray_byte` camera frame through the bowl and
// returns the mean output byte over the bowl's sampled (near-neutral)
// pixels; -1 if none found (config rejected / GPU-less).
// `exposure_compensation` < 0 leaves BowlConfig's SHIPPED default member
// initializer untouched (the regression test's mode); >= 0 overrides it
// (the measurement tool's binary-search mode).
inline int render_gray_probe(overlume::VisualRenderer* r, uint8_t gray_byte,
                             float exposure_compensation = -1.0f) {
    overlume::CameraExtrinsics ext = gray_probe_extrinsics();
    overlume::CameraIntrinsics in = gray_probe_intrinsics();
    uint32_t w = kGrayProbeW, h = kGrayProbeH;
    overlume::BowlConfig bc{};
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
    // sample AND from the renderer's dark-navy clear color.
    bc.sky_color[0] = 0.0f;
    bc.sky_color[1] = 1.0f;
    bc.sky_color[2] = 0.0f;
    if (exposure_compensation >= 0.0f) bc.exposure_compensation = exposure_compensation;

    if (!overlume::set_bowl_config(r, bc)) return -1;
    if (!overlume::set_bowl_visible(r, true)) return -1;

    std::vector<uint8_t> cam_pixels(static_cast<size_t>(w) * h * 3, gray_byte);
    // frame_id must advance across calls -- the dirty gate skips repeats.
    static uint64_t frame_id = 0;
    if (!overlume::set_camera_frame(r, 0, cam_pixels.data(), w, h, ++frame_id)) return -1;

    overlume::CameraPose pose{{0, -6, 6}, {0, 0, 0}, 70.0};
    std::vector<uint8_t> buf(static_cast<size_t>(kGrayProbeW) * kGrayProbeH * 3);
    overlume::FrameView view{buf.data(), kGrayProbeW, kGrayProbeH};
    if (!overlume::render_frame(r, pose, view)) return -1;

    long sum = 0, count = 0;
    for (size_t i = 0; i < buf.size(); i += 3) {
        const int R = buf[i], G = buf[i + 1], B = buf[i + 2];
        const int lo = std::min({R, G, B});
        const int hi = std::max({R, G, B});
        if (hi - lo <= kNearNeutralBand) {
            sum += (R + G + B);
            count += 3;
        }
    }
    if (count == 0) return -1;
    return static_cast<int>(sum / count);
}

}  // namespace overlume::testing
