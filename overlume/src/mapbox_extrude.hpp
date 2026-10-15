// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace overlume::mapbox {

// A ring in tile units (x right, y down). Open: the first vertex is not repeated.
using TileRing = std::vector<std::array<double, 2>>;

struct TilePolygon {
    TileRing outer;
    std::vector<TileRing> holes;
};

// Maps a point in the polygon's tile units, at z_m metres above the anchor's ground, to output
// metres (x east, y north, z up). Called for every ring vertex left after clipping, once at each
// height; walls and roof share the results.
using ProjectFn = std::function<std::array<double, 3>(double u, double v, double z_m)>;

// Triangles with per-face normals: faces of different orientation never share a vertex.
struct TriangleMesh {
    std::vector<float> positions;   // x, y, z per vertex, output metres
    std::vector<float> normals;     // unit length, x, y, z per vertex
    std::vector<uint32_t> indices;  // counter-clockwise seen from outside
};

struct ExtrudeStats {
    uint32_t polygons_in = 0;
    uint32_t polygons_built = 0;  // non-empty after clipping
    uint32_t wall_quads = 0;
    uint32_t roof_triangles = 0;
};

// Clips each polygon to the square [0, clip_extent]², then extrudes it from min_height_m to
// height_m: a roof at height_m (earcut, holes included) and walls between the two heights. Edges
// that lie on the clip square get no wall; there is no floor. Rings are re-oriented in output
// space (exterior counter-clockwise, holes clockwise) before walls and roofs are built. Appends to
// `out`. Requires finite heights with height_m > min_height_m; returns zero stats otherwise. A
// polygon with a non-finite coordinate is skipped.
ExtrudeStats extrude_polygons(const std::vector<TilePolygon>& polygons, double clip_extent,
                              double min_height_m, double height_m, const ProjectFn& project,
                              TriangleMesh& out);

// Area-weighted centroid of the exterior rings (tile units), before clipping; exteriors with a
// non-finite coordinate are left out. nullopt when the total area is zero.
std::optional<std::array<double, 2>> footprint_centroid(const std::vector<TilePolygon>& polygons);

}
