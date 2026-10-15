// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

static_assert(__cplusplus >= 202002L, "mapbox_landmarks.hpp is C++20-only (overlume_stream)");

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include <CesiumGltf/Model.h>

#include "mapbox_tile_frame.hpp"

namespace overlume::mapbox {

struct LandmarkTile {
    // The kept building nodes only, all in scene 0; each node's matrix replaced by its
    // local-metres → tile-frame map. No nodes when the GLB held none that were usable.
    CesiumGltf::Model model;
    // Landmark footprint triangles in tile-frame metres: x0, y0, x1, y1, x2, y2.
    std::vector<std::array<double, 6>> footprint_triangles;
    uint32_t buildings_kept = 0;
    uint32_t nodes_skipped = 0;  // a node with an unexpected matrix
    bool ok = false;             // false: unreadable or undecodable; `error` says why
    std::string error;
};

// Reads one landmark GLB: cesium decodes meshopt and quantization, images are not decoded. Keeps
// the "Default Scene" scene (else `scene`, else scene 0), collects footprint triangles from nodes
// whose extras carry `mapbox:footprint:id`, and drops footprint nodes, the LOD scene and everything
// only they used, including meshopt's source and fallback buffers. keep_materials = false (clay)
// also drops textures, images and samplers. A bufferView still carrying EXT_meshopt_compression
// after reading, a required extension the reader does not decode, or a pruned model with a buffer,
// view or texture reference out of range makes the whole tile ok = false. A footprint node with a
// valid matrix yields its triangles even when its building node was skipped.
LandmarkTile read_landmark_tile(std::span<const std::byte> glb, const TileFrame& frame,
                                bool keep_materials);

// True when (x, y) in tile-frame metres lies inside any footprint triangle.
bool inside_any_footprint(const std::vector<std::array<double, 6>>& triangles, double x, double y);

}
