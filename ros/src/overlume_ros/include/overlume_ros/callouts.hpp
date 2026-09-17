#pragma once
/** @file callouts.hpp
 *  @brief Epic 3 Task 4 (VM-031): node-side alert callouts -- one text chip
 *  + leader line for the single nearest live obstacle, drawn through
 *  hud_overlay's font/text/line primitives (Task 3, extended this task) --
 *  NOT a second compositor (plan's own instruction).
 *
 *  Split in two so "does the anchor track across camera moves" / "an
 *  anchor behind the camera is suppressed, not misdrawn" is testable on
 *  plain data, with no pixel inspection needed:
 *   - BuildNearestCallout(): picks the AlertPolygon vertex nearest `ego_pos`
 *     across every polygon in `alerts[0..alert_count)` -- plain
 *     point-to-point Euclidean distance, the same one-line formula every
 *     adapter file already keeps its own copy of (e.g. collision.cpp's own
 *     Dist()); no polygon-edge projection or other new geometry math.
 *     Formats it as "<n.n> m" and projects it via overlume::project_to_screen()
 *     against the camera state the most recent overlume::render_frame() call
 *     set. Returns false (chip untouched) when there is no alert data at
 *     all, or the nearest anchor is behind the camera / outside the
 *     frustum -- exactly project_to_screen()'s own false case, propagated
 *     up ("suppressed, not misdrawn").
 *   - DrawCallout(): given a built chip, draws a short leader line from the
 *     anchor's own projected screen point to a small fixed pixel offset
 *     above-right of it, then the chip text at that offset -- both via
 *     hud_overlay's DrawLine()/DrawText().
 */

#include <cstdint>

#include "overlume_ros/hud_overlay.hpp"
#include "overlume/scene.h"

namespace overlume_node
{

struct Callout
{
    char text[16];              // "%.1f m", nul-terminated -- fixed buffer, no std::string needed
    float anchor_x, anchor_y;   // the projected anchor itself, screen fraction [0, 1]
};

// See file comment. `ego_pos` is the map-frame ego position
// (SceneGraph::ego.position). `alerts`/`alert_count` is this tick's live
// AlertPolygon list (SceneAssembly::alerts / SceneGraph::alerts, same data
// either way).
bool BuildNearestCallout(overlume::VisualRenderer* renderer, const overlume::AlertPolygon* alerts,
                          uint32_t alert_count, overlume::Vec3 ego_pos, Callout& out);

// See file comment. `rgb`/`width`/`height` is the same frame_buf_
// hud_overlay's CompositeHud() already composited onto. `chip_rgb` colors
// both the chip text and the leader line (STANDING directive's style
// token: theme `hud.accent_color` -- the same live theme color
// overlume::get_hud_colors() already returns for the HUD's own accent, reused
// here rather than adding a second theme.hud field just for this).
void DrawCallout(uint8_t* rgb, uint32_t width, uint32_t height, const Callout& callout,
                  HudRgb chip_rgb, float scale, const char* font_path);

}  // namespace overlume_node
