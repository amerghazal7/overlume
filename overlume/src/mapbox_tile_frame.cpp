// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_tile_frame.hpp"

#include "environment_stream.hpp"
#include "mapbox_test_hooks.hpp"

#include <CesiumGeospatial/Cartographic.h>
#include <CesiumGeospatial/Ellipsoid.h>
#include <CesiumGeospatial/GlobeTransforms.h>

#include <glm/mat4x4.hpp>
#include <glm/matrix.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace overlume::mapbox {

namespace {

constexpr double kDegToRad = std::numbers::pi / 180.0;

glm::dmat4 to_glm(const Mat4d& m) {
    glm::dmat4 out(1.0);
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) out[c][r] = m[static_cast<size_t>(c * 4 + r)];
    }
    return out;
}

Mat4d to_mat4d(const glm::dmat4& m) {
    Mat4d out{};
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) out[static_cast<size_t>(c * 4 + r)] = m[c][r];
    }
    return out;
}

}

LonLatDeg tile_point_to_lonlat(const TileId& t, double u, double v, double extent) {
    const double n = std::ldexp(1.0, static_cast<int>(t.z));
    const double mercator = std::numbers::pi * (1.0 - 2.0 * (t.y + v / extent) / n);
    return {(t.x + u / extent) / n * 360.0 - 180.0, std::atan(std::sinh(mercator)) / kDegToRad};
}

TileId tile_containing(double lon_deg, double lat_deg, uint32_t z) {
    constexpr double kMaxLatDeg = 85.0511287798;
    const double lat = std::clamp(lat_deg, -kMaxLatDeg, kMaxLatDeg) * kDegToRad;
    const double n = std::ldexp(1.0, static_cast<int>(z));
    // NaN and anything west or north of the square land in column or row 0.
    const auto cell = [n](double f) {
        return f >= 0.0 ? static_cast<uint32_t>(std::min(std::floor(f), n - 1.0)) : 0u;
    };
    return {z, cell((lon_deg + 180.0) / 360.0 * n),
            cell((1.0 - std::asinh(std::tan(lat)) / std::numbers::pi) / 2.0 * n)};
}

LonLatRect tile_bounds(const TileId& t) {
    const LonLatDeg nw = tile_point_to_lonlat(t, 0.0, 0.0, 1.0);
    const LonLatDeg se = tile_point_to_lonlat(t, 1.0, 1.0, 1.0);
    return {nw.lon, se.lat, se.lon, nw.lat};
}

std::array<TileId, 16> footprint_tiles_of(const TileId& z14) {
    std::array<TileId, 16> out;
    for (uint32_t j = 0; j < 4; ++j) {
        for (uint32_t i = 0; i < 4; ++i) out[j * 4 + i] = {z14.z + 2, 4 * z14.x + i, 4 * z14.y + j};
    }
    return out;
}

TileFrame make_tile_frame(const TileId& z14, double origin_height_m) {
    const LonLatDeg centre = tile_point_to_lonlat(z14, 0.5, 0.5, 1.0);
    const glm::dvec3 origin = CesiumGeospatial::Ellipsoid::WGS84.cartographicToCartesian(
        CesiumGeospatial::Cartographic::fromDegrees(centre.lon, centre.lat, origin_height_m));
    const glm::dmat4 frameToEcef = CesiumGeospatial::GlobeTransforms::eastNorthUpToFixedFrame(
        origin, CesiumGeospatial::Ellipsoid::WGS84);
    TileFrame f;
    f.tile = z14;
    f.origin_height_m = origin_height_m;
    f.frame_to_ecef = to_mat4d(frameToEcef);
    f.ecef_to_frame = to_mat4d(glm::inverse(frameToEcef));
    return f;
}

std::array<double, 3> project_to_frame(const TileFrame& f, const TileId& src, double u, double v,
                                       double extent, double z_m) {
    const LonLatDeg ll = tile_point_to_lonlat(src, u, v, extent);
    const glm::dvec3 ecef = CesiumGeospatial::Ellipsoid::WGS84.cartographicToCartesian(
        CesiumGeospatial::Cartographic::fromDegrees(ll.lon, ll.lat, f.origin_height_m + z_m));
    const glm::dvec4 p = to_glm(f.ecef_to_frame) * glm::dvec4(ecef, 1.0);
    return {p.x, p.y, p.z};
}

double mapbox_node_scale(double lat_deg) {
    return kLandmarkExtent * std::ldexp(1.0, static_cast<int>(kLandmarkZoom)) /
           (2.0 * std::numbers::pi * kMapboxSphereRadiusM * std::cos(lat_deg * kDegToRad));
}

std::optional<Mat4d> landmark_node_to_frame(const TileFrame& f, const Mat4d& node_matrix) {
    const Mat4d& m = node_matrix;
    const double s = m[0], tx = m[12], ty = m[13];
    if (!(s > 0.0) || !std::isfinite(s) || !std::isfinite(tx) || !std::isfinite(ty)) {
        return std::nullopt;
    }
    // diag(s, -s, 1) plus (tx, ty, 0): no rotation, shear, projection or z translation.
    for (const int i : {1, 2, 3, 4, 6, 7, 8, 9, 11, 14}) {
        if (!(std::abs(m[i]) < 1e-9 * s)) return std::nullopt;
    }
    if (!(std::abs(-m[5] - s) <= 1e-6 * s) || !(std::abs(m[10] - 1.0) < 1e-9) ||
        !(std::abs(m[15] - 1.0) < 1e-9)) {
        return std::nullopt;
    }
    const double lat = tile_point_to_lonlat(f.tile, tx, ty, kLandmarkExtent).lat;
    if (!(std::abs(s / mapbox_node_scale(lat) - 1.0) <= 0.01)) return std::nullopt;

    // The exact chain, linearised at the anchor by central differences (1 tile unit, 1 m). Across
    // a 500 m × 500 m × 300 m building it stays within 2.3 cm of the exact chain: the Mercator
    // scale changes by 1.8e-5 over 250 m, the ellipsoid sags 9.8 mm at the 354 m corner, and the up
    // direction tilts by d/R, 1.7 cm at 300 m. So one node matrix places the whole model and its
    // vertices and accessor bounds stay untouched.
    const auto at = [&f](double u, double v, double w) {
        return project_to_frame(f, f.tile, u, v, kLandmarkExtent, w);
    };
    const std::array<double, 3> origin = at(tx, ty, 0.0);
    const std::array<double, 3> east = at(tx + 1.0, ty, 0.0), west = at(tx - 1.0, ty, 0.0);
    const std::array<double, 3> south = at(tx, ty + 1.0, 0.0), north = at(tx, ty - 1.0, 0.0);
    const std::array<double, 3> up = at(tx, ty, 1.0), down = at(tx, ty, -1.0);
    Mat4d a{};
    for (size_t r = 0; r < 3; ++r) {
        a[0 + r] = (east[r] - west[r]) / 2.0 * m[0];
        a[4 + r] = (south[r] - north[r]) / 2.0 * m[5];
        a[8 + r] = (up[r] - down[r]) / 2.0 * m[10];
        a[12 + r] = origin[r];
    }
    a[15] = 1.0;
    return a;
}

}

namespace overlume::testing {

bool mapbox_tile_point_to_map_probe(overlume::GeoAnchor anchor, uint32_t z, uint32_t x, uint32_t y,
                                    double u, double v, double extent, double z_m,
                                    double out_map_xyz[3]) {
    namespace mb = overlume::mapbox;
    if (out_map_xyz == nullptr || z < mb::kLandmarkZoom || z > 30) return false;
    const uint32_t shift = z - mb::kLandmarkZoom;
    const mb::TileFrame f =
        mb::make_tile_frame({mb::kLandmarkZoom, x >> shift, y >> shift}, anchor.origin_height_m);
    // As the stream places a vertex: the tile's transform (frame → ECEF), then ecefToMap_ and the
    // height correction of prepareInLoadThread.
    const std::array<double, 3> p = mb::project_to_frame(f, {z, x, y}, u, v, extent, z_m);
    const glm::dvec4 ecef = mb::to_glm(f.frame_to_ecef) * glm::dvec4(p[0], p[1], p[2], 1.0);
    const glm::dvec3 map = overlume::correct_ecef_point_height(
        glm::dvec3(ecef), overlume::compute_ecef_to_map(anchor), anchor.origin_height_m);
    out_map_xyz[0] = map.x;
    out_map_xyz[1] = map.y;
    out_map_xyz[2] = map.z;
    return true;
}

}
