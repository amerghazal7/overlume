// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

// Imagery tiles synthesized in the test process, so no Mapbox or ion imagery enters the tree: one
// flat colour per tile, which makes attachment and UV mapping pixel-checkable. Header-only C++17.
// stb_image_write's implementation comes from golden.cpp, which every test binary compiles.

#include "stb_image_write.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace overlume::testing::imagery {

// 8-bit pixels (row-major, `channels` per pixel) as PNG bytes.
inline std::vector<uint8_t> png(int width, int height, int channels,
                                const std::vector<uint8_t>& pixels) {
    std::vector<uint8_t> out;
    stbi_write_png_to_func(
        [](void* context, void* data, int size) {
            auto* bytes = static_cast<std::vector<uint8_t>*>(context);
            bytes->insert(bytes->end(), static_cast<uint8_t*>(data),
                          static_cast<uint8_t*>(data) + size);
        },
        &out, width, height, channels, pixels.data(), width * channels);
    return out;
}

// A size x size RGB PNG in one colour (JPEG is not needed: the overlay reads any image cesium
// decodes).
inline std::vector<uint8_t> flat_png(uint32_t size, uint8_t r, uint8_t g, uint8_t b) {
    std::vector<uint8_t> rgb(size_t{size} * size * 3);
    for (size_t i = 0; i < rgb.size(); i += 3) {
        rgb[i] = r;
        rgb[i + 1] = g;
        rgb[i + 2] = b;
    }
    return png(static_cast<int>(size), static_cast<int>(size), 3, rgb);
}

}
