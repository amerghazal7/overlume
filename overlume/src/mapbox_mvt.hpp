// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace overlume::mapbox {

// A vertex in MVT tile units: x right, y down, 0..extent inside the tile; the buffer lies outside.
struct MvtPoint {
    int32_t x = 0;
    int32_t y = 0;
};

// An exterior ring and its holes, classified by the MVT v2 winding rule (an exterior ring has a
// positive surveyor's-formula area in tile coordinates). Rings are open: the first vertex is not
// repeated at the end.
struct MvtPolygon {
    std::vector<MvtPoint> outer;
    std::vector<std::vector<MvtPoint>> holes;
};

// One feature of the streets-v8 `building` layer.
struct MvtBuilding {
    std::optional<uint64_t> id;
    std::vector<MvtPolygon> polygons;
    std::optional<double> height;      // `height` (m), from any numeric value type
    std::optional<double> min_height;  // `min_height` (m)
    std::optional<bool> extrude;       // `extrude`: the string "true"/"false" or a bool value
    std::optional<bool> underground;   // `underground`: likewise
    std::string type;                  // `type`, empty when absent
};

struct MvtBuildingLayer {
    bool ok = false;  // false: the tile itself is malformed and nothing in it is usable
    uint32_t extent = 4096;
    std::vector<MvtBuilding> buildings;
    uint32_t skipped_features = 0;  // malformed or non-polygon features left out
};

// Decodes the layer named "building" from an uncompressed MVT tile. Never reads outside
// [data, data + size). A well-formed tile without that layer gives ok = true and no buildings.
MvtBuildingLayer decode_mvt_building_layer(const uint8_t* data, size_t size);

// Mapbox Standard's 3D filter: `extrude` is true and `underground` is false; a missing
// `underground` fails, as the style's `== "false"` does.
bool passes_standard_3d_filter(const MvtBuilding& b);

}
