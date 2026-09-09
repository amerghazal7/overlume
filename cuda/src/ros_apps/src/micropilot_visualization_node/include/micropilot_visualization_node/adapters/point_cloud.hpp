#pragma once
/** @file point_cloud.hpp
 *  @brief PointCloudAdapter (Epic 3 Task 6 / VM-035): sensor_msgs/PointCloud2
 *  -> mpviz::PointCloud.
 *
 *  Ctor shape matches every adapter in this node (row, tf).
 *
 *  FIXTURE GAP (this task, epic3 plan): zero sensor_msgs/PointCloud2 topics
 *  exist in any recording -- every fixture this adapter's tests use is
 *  hand-built synthetic (rgb-carrying / intensity-only / bare-XYZ), and no
 *  shipped profile carries a live `adapter: point_cloud` row.
 *
 *  Field scan: reuses the offset-scan technique already proven in
 *  micropilot_rendering_node/src/rendering_node.cpp:448-462 (locate FLOAT32
 *  x/y/z by name, memcpy per point using msg.point_step), extended to also
 *  look for FLOAT32 `rgb`/`rgba` (PCL's packed-float convention -- the
 *  field's bit pattern, reinterpreted as a uint32_t, is 0x00RRGGBB /
 *  0xAARRGGBB) and FLOAT32 `intensity`. Only FLOAT32-typed fields are
 *  recognized (same subset the cited prior art handles) -- a field present
 *  under a different wire datatype reads as absent, same as a missing
 *  field, and falls through this adapter's own tier fallback.
 *
 *  x/y/z missing (any of the three) -> the WHOLE message is malformed, no
 *  coherent geometry to render (++dropped_malformed, previously-stored
 *  cloud, if any, keeps rendering). A NaN/Inf in a surviving point's x/y/z
 *  drops just that POINT (++dropped_malformed once per message, not once
 *  per point, same convention as ogm.hpp's ConvertCell).
 *
 *  color_mode tiers (row_.color_mode, profile.hpp):
 *   - `rgb`: uses the rgb/rgba field if present, else falls back exactly
 *     like `auto` below.
 *   - `auto`: rgb field present -> RGB; else intensity field present ->
 *     INTENSITY (auto-ranged per message, see below); else -> HEIGHT.
 *   - `intensity`: INTENSITY if the field is present, else HEIGHT.
 *   - `height`: always HEIGHT (z is always present -- x/y/z are required).
 *   - `flat`: bakes the alpha==0 sentinel (scene.h's PointCloudPoint
 *     comment) on every point -- point_cloud.cpp substitutes the theme's
 *     neutral token at mesh-build time. The ONE tier that actually reaches
 *     a soft-defaulted theme token per the 2026-09-09 standing directive.
 *
 *  INTENSITY/HEIGHT ramp endpoints, STATED DEVIATION: the standing
 *  directive asks for soft-defaulted theme tokens on any fixed color that
 *  ships; `flat` gets exactly that (above). INTENSITY/HEIGHT need a
 *  genuine per-point gradient, which has to be baked HERE (the frozen
 *  PointCloudPoint carries only a final rgba, no separate fraction
 *  channel the renderer could re-color at draw time) -- and no
 *  cross-toolchain path exists today for a gcc/libstdc++ node adapter to
 *  read the clang/libc++ library's parsed theme.yaml (get_hud_colors() is
 *  the one narrow exception that exists, and it exposes HUD tokens only,
 *  not a general palette read). kIntensityLowRgb/kIntensityHighRgb/
 *  kHeightLowRgb/kHeightHighRgb below are therefore compile-time constants
 *  standing in for that token, same class of gap as the HUD font path's
 *  VM-044 default (docs/visual_mode_project_backlog.md) -- replace with a
 *  real theme lookup if/when that read path is ever added.
 *
 *  Intensity/height auto-range: min/max is computed from the points THIS
 *  message actually carries (after stride/max_points decimation) -- no
 *  profile-level override field exists (unlike every other tunable on this
 *  row), since a fixed range would need re-deriving per sensor/scene
 *  anyway and this task's own Files list adds no such key.
 *
 *  Decimation: `stride` keeps every Nth point (1 = every point); the
 *  survivors are then capped at `max_points` (0 = no cap) -- both applied
 *  before color computation, so ramps auto-range over exactly the points
 *  that will actually render.
 *
 *  flatten_z is DELIBERATELY NOT applied to point-cloud geometry -- every
 *  other adapter zeros z to sit on the 2D HD-map plane (frame_transform.hpp
 *  convention), but a point cloud's height IS load-bearing 3D data (the
 *  `height` color tier needs real z variance), so the map-frame transform
 *  is applied in full, z included, regardless of tf_.flatten_z().
 *
 *  last_update_sec is stamped from ingest time, per message (PointCloud
 *  carries its own field) -- the library fades a stale cloud via
 *  point_cloud.mat's own settable alpha (point_cloud.cpp), same
 *  "stop filling past timeout_sec, mark_stale_tick() instead" shape as
 *  every other category. fill()'s PointCloud::points aliases this
 *  object's OWN storage_ and stays valid exactly as long as this instance
 *  is not destroyed and does not run another ingest() call.
 */

#include <cstdint>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "micropilot_visualization_node/adapter_stats.hpp"
#include "micropilot_visualization_node/frame_transform.hpp"
#include "micropilot_visualization_node/profile.hpp"
#include "micropilot_visualization_node/scene_assembly.hpp"
#include "visual_renderer/scene.h"

namespace mpviz_node
{

// Packs r,g,b,a (each 0..255) into scene.h's documented PointCloudPoint::
// rgba convention (byte0=r, byte1=g, byte2=b, byte3=a). Exposed (not
// file-local) so this task's adapter tests can assert on the exact packed
// value without re-deriving the convention themselves.
inline uint32_t PackRgba(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    return static_cast<uint32_t>(r) | (static_cast<uint32_t>(g) << 8) |
           (static_cast<uint32_t>(b) << 16) | (static_cast<uint32_t>(a) << 24);
}

class PointCloudAdapter
{
public:
    PointCloudAdapter(const ProfileRow& row,
                      const micropilot::visualization_app::FrameTransformer& tf);

    // ROS callback thread. See this file's header comment for the full
    // field-scan/tier/decimation/malformed rules.
    void ingest(const sensor_msgs::msg::PointCloud2& msg, double sim_time_sec);

    // Timer thread, before set_scene(). APPENDS at most ONE PointCloud into
    // out.point_clouds (never assigns/replaces it) -- same "multiple rows
    // must not erase each other" shape as every other category. Emits
    // nothing if this adapter has never received a valid message.
    void fill(micropilot::visualization_app::SceneAssembly& out) const;

    const AdapterStats& stats() const { return stats_; }

    // Node-side staleness bookkeeping -- same shape as every other adapter.
    void mark_stale_tick() { ++stats_.dropped_stale; }

private:
    ProfileRow row_;
    const micropilot::visualization_app::FrameTransformer& tf_;

    bool has_data_{false};
    std::vector<mpviz::PointCloudPoint> storage_;
    double last_update_sec_{0.0};
    AdapterStats stats_;
};

}  // namespace mpviz_node
