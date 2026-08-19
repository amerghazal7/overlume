#pragma once
/** @file ego_anchor.hpp
 *  @brief Ego-anchored virtual-camera pose composition (2026-08-19 user
 *  directive). Pulled out of visualization_node.cpp's anonymous namespace
 *  (review finding: the yaw convention already regressed once -- a pi/2
 *  broadside bug caught only by a human on a live frame -- so it needs
 *  automated coverage, which a translation-unit-local function can't get)
 *  so test_ego_anchor.cpp can include and exercise it directly.
 */

#include <cmath>

#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

namespace micropilot::visualization_app
{

// The whole vcam pipeline (presets_, cur_/src_/dst_ tween, on_set_look,
// ~/vcam_state) stays in an EGO-RELATIVE OFFSET frame: eye/target are
// defined as if the ego sat at the world origin facing world +X
// (heading_rad == 0).
//
// Concrete case, verified by hand against the node's REAL default offset
// (virtual_pose param: eye (-4, 0, 3.5), target (2, 0, -0.5)) — the offset
// frame already looks along +X, i.e. its "forward" IS +X, so kEgoForwardYaw
// is 0 and offsets rotate by heading_rad directly. heading_rad = 0 (ego
// facing world +X): rotation = 0, eye stays (-4, 0) = directly behind the
// ego, target (2, 0) = ahead of it. heading_rad = -pi/2 (facing -Y):
// rotation -pi/2 puts the eye at (0, +4) — behind an ego driving toward -Y,
// looking along its heading. (An earlier revision used pi/2, derived from a
// -Y-facing example offset {0,-8,4} that is NOT this node's default — it
// framed the ego broadside-on, verified live against the fixture bag.)
constexpr double kEgoForwardYaw = 0.0;

// Yaw-rotate a rig-relative offset pose about +Z by (ego.heading_rad -
// kEgoForwardYaw), then translate by ego.position (z included -- callers
// must never assume ego z == 0, see EgoState::position doc). Pure function:
// takes the tweened offset pose as input, returns a new pose for the
// renderer -- never mutates any node state, so it cannot feed back into the
// tween/telemetry state. Caller's responsibility to gate on ego.valid.
inline mpviz::CameraPose compose_ego_anchored_pose(const mpviz::CameraPose& offset_pose,
                                                    const mpviz::EgoState& ego)
{
    const double rot = ego.heading_rad - kEgoForwardYaw;
    const double c = std::cos(rot);
    const double s = std::sin(rot);
    auto rotate_and_translate = [&](const double in[3], double out[3])
    {
        out[0] = in[0] * c - in[1] * s + ego.position.x;
        out[1] = in[0] * s + in[1] * c + ego.position.y;
        out[2] = in[2] + ego.position.z;
    };
    mpviz::CameraPose out = offset_pose;
    rotate_and_translate(offset_pose.eye, out.eye);
    rotate_and_translate(offset_pose.target, out.target);
    return out;
}

}  // namespace micropilot::visualization_app
