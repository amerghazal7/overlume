// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

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
constexpr int kToleranceBytes = 1;

}

int main() {
    overlume::RenderConfig cfg{kW, kH, 1, nullptr, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) {
        std::printf("bowl_exposure_probe: no GPU/EGL, cannot measure\n");
        return 1;
    }

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
        int out = overlume::testing::render_gray_probe(r, static_cast<uint8_t>(g),
                                                       static_cast<float>(best));
        std::printf("  in=%3d (sRGB byte) -> out=%3d\n", g, out);
    }

    overlume::destroy_renderer(r);
    return 0;
}
