#pragma once
/** @file trajectory_carpet.hpp
 *  @brief TrajectoryCarpetAdapter (VM-077): MarkerArray -> mpviz::TrajectoryCarpet.
 *
 *  Ctor shape matches every adapter in this node (row, tf).
 *
 *  Measured against the real recorded `output_trajectory_carpet` (VM-077
 *  measurement report §1): ONE persistent marker
 *  (`ns='output_trajectory_carpet' id=0`), refreshed every tick via
 *  `DELETE_ALL` + one `ADD`, never fanned across ns/id -- so, unlike
 *  CollisionAdapter/GenericMarkerAdapter, this adapter needs no `Key{ns,id}`
 *  map: it tracks exactly one stored carpet, wholesale-replaced on every
 *  valid ingest (same "REPLACES, never merges" contract as PathAdapter).
 *
 *  ingest(): `type==TRIANGLE_LIST && points.size()%3==0 && points.size()>=3`
 *  (else ++dropped_malformed, previous carpet -- if any -- keeps
 *  rendering). Marker pose composition and frame transform identical to
 *  every other marker adapter (points[] RELATIVE to marker.pose, composed
 *  BEFORE the frame transform; a zero/degenerate orientation quaternion is
 *  identity, matching rviz); flatten_z applies (frame_transform.hpp) -- the
 *  carpet's own z (~0.150 m, measured) sits on the same 2D plane every other
 *  category flattens to.
 *
 *  Color: `colors[i]` packed via `PackRgba()` (reused from
 *  point_cloud.hpp's adapter -- point_cloud.cpp's `resolve_rgba()`
 *  alpha-zero-sentinel convention is what the RENDERER does with the
 *  result, not this adapter) with alpha forced to 255 ("a real color was
 *  supplied" sentinel) WHEN `colors.size() == points.size()`; otherwise
 *  every point's packed rgba is `0` (alpha byte 0 -- "no real per-point
 *  color was computed", a WHOLE-MESSAGE fallback since a length mismatch
 *  means the whole array is suspect, not a per-point one).
 *
 *  DELETE_ALL clears the stored carpet. last_update_sec is stamped from
 *  ingest time -- TrajectoryCarpet carries its own, so the library fades it
 *  via the shared staleness_alpha() ramp (trajectory_carpet.cpp), same
 *  "stop filling past timeout_sec, mark_stale_tick() instead" shape as
 *  every other category. Storage outlives fill() (same alias-lifetime
 *  contract as every other adapter).
 */

#include <cstdint>
#include <vector>

#include <visualization_msgs/msg/marker_array.hpp>

#include "micropilot_visualization_node/adapter_stats.hpp"
#include "micropilot_visualization_node/adapters/point_cloud.hpp"  // PackRgba()
#include "micropilot_visualization_node/frame_transform.hpp"
#include "micropilot_visualization_node/profile.hpp"
#include "micropilot_visualization_node/scene_assembly.hpp"
#include "visual_renderer/scene.h"

namespace mpviz_node
{

class TrajectoryCarpetAdapter
{
public:
    TrajectoryCarpetAdapter(const ProfileRow& row,
                            const micropilot::visualization_app::FrameTransformer& tf);

    // ROS callback thread. See this file's header comment for the full
    // frame/malformed/replace rules.
    void ingest(const visualization_msgs::msg::MarkerArray& msg, double sim_time_sec);

    // Timer thread, before set_scene(). APPENDS at most ONE TrajectoryCarpet
    // into out.trajectory_carpets (never assigns/replaces it) -- same
    // "multiple rows must not erase each other" shape as every other
    // category, even though today's shipped profile carries exactly one
    // row. Emits nothing if this adapter has never received a valid
    // message. Returned TrajectoryCarpet::points aliases this object's OWN
    // storage_ and stays valid exactly as long as this instance is not
    // destroyed and does not run another ingest() call.
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
