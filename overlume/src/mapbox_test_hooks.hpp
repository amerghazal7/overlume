// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

// Task 5. Its block goes directly after `namespace overlume::testing {`, above Task 4's block, so
// that Task 13's block (appended at the end, possibly before Task 5 merges) merges cleanly.
// inside_any_footprint for C++17 tests (the real one is declared in C++20 mapbox_landmarks.hpp).
bool mapbox_inside_any_footprint_probe(const std::vector<std::array<double, 6>>& triangles,
                                       double x, double y);
struct MapboxLandmarkProbe {
    bool ok = false;
    std::string error;
    uint32_t buildings_kept = 0;
    uint32_t nodes_skipped = 0;
    uint32_t scenes = 0, nodes = 0, meshes = 0, accessors = 0, buffer_views = 0, buffers = 0;
    uint32_t images = 0, textures = 0, samplers = 0, materials = 0;
    uint32_t footprint_triangles = 0;
    uint64_t buffer_bytes = 0;            // summed size of every buffer's data
    bool meshopt_extension_left = false;  // EXT_meshopt_compression anywhere after reading
    std::vector<std::array<double, 16>> node_matrices;  // kept nodes, column-major, in order
};
MapboxLandmarkProbe probe_mapbox_landmark_tile(const std::vector<uint8_t>& glb, uint32_t x14,
                                               uint32_t y14, double origin_height_m,
                                               bool keep_materials);

// Task 4. A point (u, v) in tile (z, x, y) units (`extent` per edge) at z_m metres above the
// anchor's ground, through the Mapbox placement chain (tile frame of its z14 tile, ECEF, the map
// frame of `anchor`, the stream's height correction) → map metres.
bool mapbox_tile_point_to_map_probe(overlume::GeoAnchor anchor, uint32_t z, uint32_t x, uint32_t y,
                                    double u, double v, double extent, double z_m,
                                    double out_map_xyz[3]);

}
