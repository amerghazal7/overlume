/** @file callouts.cpp
 *  @brief See callouts.hpp.
 */
#include "micropilot_visualization_node/callouts.hpp"

#include <cmath>
#include <cstdio>
#include <limits>

namespace mpviz_node
{
namespace
{

// Plain point-to-point distance -- same one-line formula every adapter file
// already keeps its own local copy of (e.g. collision.cpp's own Dist());
// not shared across translation units by convention in this package.
double Dist(const mpviz::Vec3& a, const mpviz::Vec3& b)
{
    const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Pixel offset from the anchor's own projected point to where the chip
// text/leader-line start actually sits -- purely cosmetic (keeps the label
// from sitting directly on top of the thing it's labeling).
constexpr float kChipOffsetXPx = 24.0f;
constexpr float kChipOffsetYPx = -24.0f;

}  // namespace

bool BuildNearestCallout(mpviz::VisualRenderer* renderer, const mpviz::AlertPolygon* alerts,
                          uint32_t alert_count, mpviz::Vec3 ego_pos, Callout& out)
{
    bool found = false;
    double best_dist = std::numeric_limits<double>::infinity();
    mpviz::Vec3 best_anchor{};
    for (uint32_t i = 0; i < alert_count; ++i)
    {
        const mpviz::AlertPolygon& poly = alerts[i];
        for (uint32_t j = 0; j < poly.point_count; ++j)
        {
            const double d = Dist(ego_pos, poly.points[j]);
            if (d < best_dist)
            {
                best_dist = d;
                best_anchor = poly.points[j];
                found = true;
            }
        }
    }
    if (!found) return false;  // no alert data this tick -- no chip to build

    float x = 0.0f, y = 0.0f;
    // Behind the camera / outside the frustum -- suppressed, not misdrawn
    // (this task's own AC).
    if (!mpviz::project_to_screen(renderer, best_anchor, &x, &y)) return false;

    std::snprintf(out.text, sizeof(out.text), "%.1f m", best_dist);
    out.anchor_x = x;
    out.anchor_y = y;
    return true;
}

void DrawCallout(uint8_t* rgb, uint32_t width, uint32_t height, const Callout& callout,
                  HudRgb chip_rgb, float scale, const char* font_path)
{
    const float ax = callout.anchor_x * static_cast<float>(width);
    const float ay = callout.anchor_y * static_cast<float>(height);
    // Offsets scale with the theme's hud.scale (review 2026-09-09): fixed
    // 24px offsets under grown glyphs would let the chip overlap the thing
    // it labels.
    const float tx = ax + kChipOffsetXPx * scale;
    const float ty = ay + kChipOffsetYPx * scale;
    DrawLine(rgb, width, height, ax, ay, tx, ty, chip_rgb);
    DrawText(rgb, width, height, callout.text, tx, ty, chip_rgb, scale, font_path);
}

}  // namespace mpviz_node
