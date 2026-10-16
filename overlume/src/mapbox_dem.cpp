// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_dem.hpp"

#include <algorithm>
#include <climits>
#include <cmath>

// stb_image from the vcpkg prefix cesium-native builds against (on overlume_stream's include
// path). Static and PNG-only, as cesium's own CesiumImage/src/ImageDecoder.cpp and the node's
// hud_overlay.cpp compile it, so these symbols never meet another stb_image at link time.
#define STB_IMAGE_STATIC
#define STBI_ONLY_PNG
#define STBI_NO_STDIO
#define STB_IMAGE_IMPLEMENTATION
// Resolves only through usage requirements: the overlume_stream block in overlume/CMakeLists.txt
// links Cesium3DTilesSelection, whose target puts the vcpkg prefix on the include path.
#include <stb_image.h>

namespace overlume::mapbox {

namespace {

constexpr size_t kDemPixels = size_t{kDemSize} * kDemSize;

DemTile failed(const char* reason) {
    DemTile out;
    out.error = reason;
    return out;
}

}

DemTile decode_dem_png(const uint8_t* bytes, size_t size) {
    if (bytes == nullptr || size == 0) return failed("DEM tile is empty");
    int w = 0, h = 0, channels = 0;
    if (size > static_cast<size_t>(INT_MAX) ||
        stbi_info_from_memory(bytes, static_cast<int>(size), &w, &h, &channels) == 0) {
        return failed("DEM tile does not decode as PNG");
    }
    const int len = static_cast<int>(size);
    if (w != static_cast<int>(kDemSize) || h != static_cast<int>(kDemSize)) {
        return failed("DEM tile is not 512 x 512 pixels");
    }
    if (stbi_is_16_bit_from_memory(bytes, len) != 0) return failed("DEM tile is not 8-bit");
    if (channels < 3) return failed("DEM tile has fewer than 3 channels");
    stbi_uc* rgb = stbi_load_from_memory(bytes, len, &w, &h, &channels, 3);
    if (rgb == nullptr) return failed("DEM tile does not decode as PNG");
    DemTile out;
    out.height_m.resize(kDemPixels);
    for (size_t i = 0; i < kDemPixels; ++i) {
        const stbi_uc* p = rgb + 3 * i;
        out.height_m[i] =
            static_cast<float>(-10000.0 + (p[0] * 65536.0 + p[1] * 256.0 + p[2]) * 0.1);
    }
    stbi_image_free(rgb);
    out.ok = true;
    return out;
}

double dem_sample(const DemTile& dem, double u, double v) {
    if (dem.height_m.size() != kDemPixels) return 0.0;
    // Pixel (px, py) holds the height at (px + 0.5, py + 0.5); fmin/fmax also map NaN to the edge.
    const double x = std::fmin(std::fmax(u - 0.5, 0.0), kDemSize - 1.0);
    const double y = std::fmin(std::fmax(v - 0.5, 0.0), kDemSize - 1.0);
    const uint32_t x0 = std::min(static_cast<uint32_t>(x), kDemSize - 2);
    const uint32_t y0 = std::min(static_cast<uint32_t>(y), kDemSize - 2);
    const double fx = x - x0, fy = y - y0;
    const double north = dem.at(x0, y0) * (1.0 - fx) + dem.at(x0 + 1, y0) * fx;
    const double south = dem.at(x0, y0 + 1) * (1.0 - fx) + dem.at(x0 + 1, y0 + 1) * fx;
    return north * (1.0 - fy) + south * fy;
}

DemMesh mesh_dem(const DemTile& dem, const TileFrame& frame, const TileId& t14, double z_ref_m,
                 double skirt_m) {
    DemMesh m;
    if (!dem.ok || dem.height_m.size() != kDemPixels) return m;
    constexpr uint32_t n = kDemGridCells + 1;  // vertices per side
    constexpr double step = static_cast<double>(kDemSize) / kDemGridCells;
    const auto place = [&](uint32_t i, uint32_t j, double down_m) {
        const double u = i * step, v = j * step;
        return project_to_frame(frame, t14, u, v, kDemSize,
                                dem_sample(dem, u, v) - z_ref_m - down_m);
    };
    const size_t vertices = size_t{n} * n + 4 * kDemGridCells;  // the grid, then the skirt ring
    m.positions.reserve(vertices);
    m.normals.reserve(vertices);  // so the skirt's copies below never reallocate under a reference
    m.uvs.reserve(vertices);
    std::vector<std::array<double, 3>> grid;
    grid.reserve(size_t{n} * n);
    for (uint32_t j = 0; j < n; ++j) {
        for (uint32_t i = 0; i < n; ++i) grid.push_back(place(i, j, 0.0));
    }
    const auto at = [&grid](uint32_t i, uint32_t j) { return grid[size_t{j} * n + i]; };
    for (uint32_t j = 0; j < n; ++j) {
        for (uint32_t i = 0; i < n; ++i) {
            const std::array<double, 3> p = at(i, j);
            m.positions.push_back(
                {static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2])});
            m.uvs.push_back(
                {static_cast<float>(i) / kDemGridCells, static_cast<float>(j) / kDemGridCells});
            // Central differences of the grid (one-sided on the border): east x north.
            const std::array<double, 3> e = at(std::min(i + 1, n - 1), j), w = at(i ? i - 1 : 0, j);
            const std::array<double, 3> no = at(i, j ? j - 1 : 0),
                                        s = at(i, std::min(j + 1, n - 1));
            const double ex = e[0] - w[0], ey = e[1] - w[1], ez = e[2] - w[2];
            const double nx = no[0] - s[0], ny = no[1] - s[1], nz = no[2] - s[2];
            const double cx = ey * nz - ez * ny, cy = ez * nx - ex * nz, cz = ex * ny - ey * nx;
            const double len = std::sqrt(cx * cx + cy * cy + cz * cz);
            m.normals.push_back({static_cast<float>(cx / len), static_cast<float>(cy / len),
                                 static_cast<float>(cz / len)});
        }
    }
    m.grid_vertex_count = n * n;
    for (uint32_t j = 0; j < kDemGridCells; ++j) {
        for (uint32_t i = 0; i < kDemGridCells; ++i) {
            const uint32_t nw = j * n + i, ne = nw + 1, sw = nw + n, se = sw + 1;
            m.indices.insert(m.indices.end(), {nw, sw, se, nw, se, ne});
        }
    }
    // The border, counter-clockwise seen from above: down the west edge, east along the south,
    // up the east, west along the north. Each border vertex once.
    std::vector<uint32_t> ring;
    for (uint32_t j = 0; j < kDemGridCells; ++j) ring.push_back(j * n);
    for (uint32_t i = 0; i < kDemGridCells; ++i) ring.push_back(kDemGridCells * n + i);
    for (uint32_t j = kDemGridCells; j > 0; --j) ring.push_back(j * n + kDemGridCells);
    for (uint32_t i = kDemGridCells; i > 0; --i) ring.push_back(i);
    const auto base = static_cast<uint32_t>(m.positions.size());
    const auto count = static_cast<uint32_t>(ring.size());
    for (const uint32_t r : ring) {
        const std::array<double, 3> p = place(r % n, r / n, skirt_m);
        m.positions.push_back(
            {static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2])});
        m.normals.push_back(m.normals[r]);
        m.uvs.push_back(m.uvs[r]);
    }
    // One quad per border segment, wound to face out of the tile.
    for (uint32_t k = 0; k < count; ++k) {
        const uint32_t e0 = ring[k], e1 = ring[(k + 1) % count];
        const uint32_t s0 = base + k, s1 = base + (k + 1) % count;
        m.indices.insert(m.indices.end(), {e0, s0, s1, e0, s1, e1});
    }
    return m;
}

}
