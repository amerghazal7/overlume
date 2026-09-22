// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "overlume/scene.h"

namespace overlume::detail {

std::vector<Vec3> extrude_polyline(const Vec3* pts, uint32_t n, float half_width, float z_lift);

std::vector<uint16_t> extrude_polyline_indices(uint32_t point_count);

std::vector<Vec3> triangulate_convex_polygon(const Vec3* pts, uint32_t n, float z_lift);

inline constexpr uint32_t kMaxPointsPerMesh = 32000;
std::vector<std::pair<uint32_t, uint32_t>> polyline_chunks(uint32_t n);

std::pair<double, double> closest_arc_station(const Vec3* pts, uint32_t n, const Vec3& ego);

inline constexpr float kPolylineEgoClipLateralM = 5.0f;
inline constexpr float kPolylineClipQuantizeM = 0.05f;

struct PolylineClip {
    bool active = false;
    int64_t quantized_units = 0;
    double station_m = 0.0;
};

PolylineClip compute_polyline_clip(const Vec3* pts, uint32_t n, const Vec3& ego_position);

std::vector<double> clean_polyline_stations(const Vec3* pts, uint32_t n);

void collapse_clipped_positions(std::vector<Vec3>& positions, const std::vector<double>& stations,
                                bool clip_active, double clip_station_m);

std::vector<Vec3> build_crosswalk_hatch(const Vec3* pts, uint32_t n, float z_lift);

}  // namespace overlume::detail
