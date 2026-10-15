// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace overlume::mapbox {

inline constexpr uint32_t kLandmarkZoom = 14;
inline constexpr uint32_t kFootprintZoom = 16;
inline constexpr double kLandmarkExtent = 8192.0;          // landmark node units per z14 tile
inline constexpr double kMapboxSphereRadiusM = 6371008.8;  // the radius Mapbox's scale uses

// Column-major 4×4, as glm and glTF store matrices.
using Mat4d = std::array<double, 16>;

struct TileId {
    uint32_t z = 0;
    uint32_t x = 0;
    uint32_t y = 0;  // XYZ: row 0 at the north edge
};

struct LonLatDeg {
    double lon = 0.0;
    double lat = 0.0;
};

struct LonLatRect {
    double west = 0.0, south = 0.0, east = 0.0, north = 0.0;  // degrees
};

// Spherical Web Mercator: (u, v) in units of tile t, `extent` units per tile edge → lon/lat.
LonLatDeg tile_point_to_lonlat(const TileId& t, double u, double v, double extent);

// The zoom-z tile containing lon/lat; latitude is clamped to the Web Mercator limit.
TileId tile_containing(double lon_deg, double lat_deg, uint32_t z);

LonLatRect tile_bounds(const TileId& t);

// The 16 z16 tiles of z14 tile t, row-major from the north-west corner.
std::array<TileId, 16> footprint_tiles_of(const TileId& z14);

// The z14 tile's frame: east-north-up at the tile centre, on the WGS84 ellipsoid at
// origin_height_m (cesium's eastNorthUpToFixedFrame).
struct TileFrame {
    TileId tile;
    double origin_height_m = 0.0;
    Mat4d frame_to_ecef{};
    Mat4d ecef_to_frame{};
};

TileFrame make_tile_frame(const TileId& z14, double origin_height_m);

// The exact chain: (u, v) in units of tile `src` (`extent` per edge), z_m metres above
// origin_height_m → longitude and latitude (inverse Web Mercator) → ECEF at origin_height_m + z_m
// → tile-frame metres.
std::array<double, 3> project_to_frame(const TileFrame& f, const TileId& src, double u, double v,
                                       double extent, double z_m);

// Mapbox's landmark scale at a latitude: 8192·2^14 / (2π·6371008.8·cos lat), tile units per metre.
double mapbox_node_scale(double lat_deg);

// A landmark node matrix (local metres → z14 tile units: diag(s, −s, 1) plus (tx, ty, 0)) → the
// affine map from local metres into f, linearised at (tx, ty). nullopt unless the matrix has that
// form with s within 1% of mapbox_node_scale at the node's latitude.
std::optional<Mat4d> landmark_node_to_frame(const TileFrame& f, const Mat4d& node_matrix);

}
