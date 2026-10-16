// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include "mapbox_tile_frame.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace overlume::mapbox {

inline constexpr uint32_t kDemSize = 512;      // pixels per side of a terrain-DEM tile
inline constexpr uint32_t kDemGridCells = 64;  // mesh cells per side (one vertex every 8 pixels)
inline constexpr double kDemSkirtM = 20.0;     // skirt depth below the edge

struct DemTile {
    bool ok = false;
    std::string error;            // one line when !ok
    std::vector<float> height_m;  // kDemSize * kDemSize, row-major, row 0 at the tile's north edge
    float at(uint32_t px, uint32_t py) const {  // px east, py south, both < kDemSize
        return height_m[static_cast<size_t>(py) * kDemSize + px];
    }
};

// Decodes a terrain-DEM PNG (stb_image, 8-bit, 3 or 4 channels, exactly 512 x 512) with
// height = -10000 + (R * 65536 + G * 256 + B) * 0.1. Any other size, depth or decode failure:
// ok = false.
DemTile decode_dem_png(const uint8_t* bytes, size_t size);

// Bilinear height at pixel-centre coordinates (u east, v south, 0..512, clamped to the edge).
double dem_sample(const DemTile& dem, double u, double v);

struct DemMesh {
    // Tile frame metres, grid vertices first (row-major from the north-west corner), then the
    // skirt ring.
    std::vector<std::array<float, 3>> positions;
    // Unit, from central differences; a skirt vertex copies its edge vertex's.
    std::vector<std::array<float, 3>> normals;
    // u east 0..1, v south 0..1 across the tile.
    std::vector<std::array<float, 2>> uvs;
    // CCW seen from above, grid then skirt (the skirt faces out of the tile).
    std::vector<uint32_t> indices;
    uint32_t grid_vertex_count = 0;  // (kDemGridCells + 1)^2
};

// Heights become z = height - z_ref_m; vertices sit at u, v = 0, 8, ..., 512 (pixel coordinates),
// so neighbouring tiles share their edge vertices and heights are bilinear samples at those points;
// positions through project_to_frame(frame, t14, u, v, kDemSize, z). Empty when !dem.ok.
DemMesh mesh_dem(const DemTile& dem, const TileFrame& frame, const TileId& t14, double z_ref_m,
                 double skirt_m = kDemSkirtM);

}
