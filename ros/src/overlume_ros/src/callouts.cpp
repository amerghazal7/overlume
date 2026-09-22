// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/callouts.hpp"

#include <cmath>
#include <cstdio>
#include <limits>

namespace overlume::ros {
namespace {

double Dist(const overlume::Vec3& a, const overlume::Vec3& b) {
    const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

constexpr float kChipOffsetXPx = 24.0f;
constexpr float kChipOffsetYPx = -24.0f;

}  // namespace

bool BuildNearestCallout(overlume::VisualRenderer* renderer, const overlume::AlertPolygon* alerts,
                         uint32_t alert_count, overlume::Vec3 ego_pos, Callout& out) {
    bool found = false;
    double best_dist = std::numeric_limits<double>::infinity();
    overlume::Vec3 best_anchor{};
    for (uint32_t i = 0; i < alert_count; ++i) {
        const overlume::AlertPolygon& poly = alerts[i];
        for (uint32_t j = 0; j < poly.point_count; ++j) {
            const double d = Dist(ego_pos, poly.points[j]);
            if (d < best_dist) {
                best_dist = d;
                best_anchor = poly.points[j];
                found = true;
            }
        }
    }
    if (!found) return false;

    float x = 0.0f, y = 0.0f;
    if (!overlume::project_to_screen(renderer, best_anchor, &x, &y)) return false;

    std::snprintf(out.text, sizeof(out.text), "%.1f m", best_dist);
    out.anchor_x = x;
    out.anchor_y = y;
    return true;
}

void DrawCallout(uint8_t* rgb, uint32_t width, uint32_t height, const Callout& callout,
                 HudRgb chip_rgb, float scale, const char* font_path) {
    const float ax = callout.anchor_x * static_cast<float>(width);
    const float ay = callout.anchor_y * static_cast<float>(height);
    const float tx = ax + kChipOffsetXPx * scale;
    const float ty = ay + kChipOffsetYPx * scale;
    DrawLine(rgb, width, height, ax, ay, tx, ty, chip_rgb);
    DrawText(rgb, width, height, callout.text, tx, ty, chip_rgb, scale, font_path);
}

}  // namespace overlume::ros
