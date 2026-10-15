// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

// Mapbox terrain-DEM tiles (512 px terrain-RGB PNGs) synthesized in the test process from a height
// function, so no Mapbox bytes enter the tree. Header-only C++17.

#include "imagery_fixture.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace overlume::testing::dem {

inline constexpr int kDemSide = 512;  // pixels per side, as mapbox::kDemSize

// A 512 x 512 RGB PNG whose pixel (px, py) encodes height_m(u = px + 0.5, v = py + 0.5) with the
// Mapbox terrain-RGB formula (stb_image_write). Heights are rounded to 0.1 m.
inline std::vector<uint8_t> encode_dem_png(
    const std::function<double(double u, double v)>& height_m) {
    std::vector<uint8_t> rgb(size_t{kDemSide} * kDemSide * 3);
    for (int py = 0; py < kDemSide; ++py) {
        for (int px = 0; px < kDemSide; ++px) {
            // height = -10000 + code * 0.1, code = R * 65536 + G * 256 + B
            const double code = std::round((height_m(px + 0.5, py + 0.5) + 10000.0) * 10.0);
            const auto c = static_cast<uint32_t>(std::clamp(code, 0.0, 16777215.0));
            uint8_t* p = &rgb[(size_t(py) * kDemSide + size_t(px)) * 3];
            p[0] = static_cast<uint8_t>(c >> 16);
            p[1] = static_cast<uint8_t>(c >> 8);
            p[2] = static_cast<uint8_t>(c);
        }
    }
    return imagery::png(kDemSide, kDemSide, 3, rgb);
}

// Named scenes used by several tasks (u east, v south, in tile pixels 0..512).
// A ramp rising 0..20 m west to east.
inline double ramp_height_m(double u, double /*v*/) { return 20.0 * u / kDemSide; }

// A hill of 40 m at the tile centre (gaussian, sigma 400 px).
inline double hill_height_m(double u, double v) {
    const double du = u - kDemSide / 2.0, dv = v - kDemSide / 2.0;
    return 40.0 * std::exp(-(du * du + dv * dv) / (2.0 * 400.0 * 400.0));
}

// A shelf 10 m high on the north half.
inline double shelf_height_m(double /*u*/, double v) { return v < kDemSide / 2.0 ? 10.0 : 0.0; }

// Sea: 0 everywhere.
inline double sea_height_m(double /*u*/, double /*v*/) { return 0.0; }

}
