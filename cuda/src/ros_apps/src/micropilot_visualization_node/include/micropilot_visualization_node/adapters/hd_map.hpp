#pragma once
/** @file hd_map.hpp
 *  @brief HdMapAdapter (Epic 2 Task 2 / VM-024): MarkerArray -> MapElement.
 *
 *  Ctor shape matches every Epic 2 adapter (epic2 plan, "Node: adapter
 *  shape"): (row, tf). No node handle, no subscription -- visualization_
 *  node.cpp owns every create_subscription() call and routes the callback
 *  into ingest(). One instance per profile row -- urban ships THREE
 *  (/hd_map_local_elements, /hd_map_global_elements, /road_markers), the
 *  sim profile adds a fourth (/sim/hd_map/markers, latched) -- and every
 *  instance's fill() APPENDS into the shared SceneAssembly (see that
 *  header's own comment on why: the last adapter to run must not erase the
 *  others).
 *
 *  Namespace rules (profile.hpp's classify()) decide is_polygon
 *  (NsRender::kPolygon) vs polyline (kPolyline) vs dropped (kDrop) -- see
 *  the shipped urban_profile.yaml / sim_profile.yaml hd_map rows. This
 *  class never reads a namespace string itself beyond classify()'s
 *  verdict -- MapElement carries no namespace and no "kind" (scene.h:56-59
 *  is frozen at {points, point_count, is_polygon}).
 *
 *  Frames: every marker's points are moved into the map frame via `tf`
 *  BEFORE being stored -- ONE lookup per ingest() call (from the first
 *  marker's header), never per marker (epic2 plan, "Frames": the common
 *  case, frame_id == "map", costs nothing; a non-map frame costs one
 *  lookup for the whole message, not one per marker).
 *
 *  Marker.msg semantics (rviz-parity fix, user report 2026-08-20):
 *  points[] on a LINE_STRIP are RELATIVE to marker.pose, composed as
 *  frame_transform * (marker_pose * point) -- the marker pose is IN the
 *  header frame, so it composes INSIDE the frame transform. Applied per
 *  marker, before dash chopping. Identity pose (bag data today) skips the
 *  multiply entirely; a NaN pose is malformed (dropped_malformed), a
 *  non-identity pose is not.
 *
 *  Staleness: node-side only, by design (epic2 plan, "Staleness" --
 *  STATED DEVIATION: MapElement is frozen with no last_update_sec, so the
 *  library cannot fade map geometry; past row.timeout_sec every lane and
 *  crosswalk vanishes in one frame). This adapter has no timeout logic of
 *  its own -- visualization_node.cpp checks stats().last_msg_sec against
 *  the row's timeout_sec and simply stops calling fill() for a stale row.
 */

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <visualization_msgs/msg/marker_array.hpp>

#include "micropilot_visualization_node/adapter_stats.hpp"
#include "micropilot_visualization_node/frame_transform.hpp"
#include "micropilot_visualization_node/profile.hpp"
#include "micropilot_visualization_node/scene_assembly.hpp"
#include "visual_renderer/scene.h"

namespace mpviz_node
{

// AdapterStats now lives in adapter_stats.hpp (epic2 plan, "Diagnostics
// counters") -- promoted out of here the day a second adapter (Task 3's
// DynamicObjectsAdapter) needed the identical shape, exactly as this
// comment used to say it would be.

class HdMapAdapter
{
public:
    HdMapAdapter(const ProfileRow& row, const micropilot::visualization_app::FrameTransformer& tf);

    // ROS callback thread. Transforms into the map frame, applies the
    // row's namespace rules, and replaces/updates this adapter's own
    // storage in place (DELETEALL clears it, DELETE removes one id, ADD/
    // MODIFY inserts-or-overwrites by (ns, id) -- exactly ROS Marker
    // semantics). Malformed markers (points.size() < 2, a NaN point, a
    // non-LINE_STRIP type) are dropped and counted, never partially
    // stored. Rule-dropped namespaces (classify() == kDrop) are counted
    // separately (dropped_by_rule) and never touch storage_ at all. A TF
    // lookup failure drops the WHOLE message (dropped_no_tf) and leaves
    // storage_ untouched, so previously-rendered elements keep rendering.
    // row.max_rate_hz (>0) rate-limits REBUILDS, not receipt: a message
    // arriving before the last accepted rebuild's cooldown elapses is
    // dropped in full (storage_ untouched), matching the shipped urban
    // row's max_rate_hz: 2.0 against an ~18 Hz publisher.
    void ingest(const visualization_msgs::msg::MarkerArray& msg, double sim_time_sec);

    // Timer thread, before set_scene(). APPENDS into out.map_elements
    // (never assigns/replaces it) -- see SceneAssembly's own header
    // comment: this node runs multiple HdMapAdapter instances (one per
    // hd_map profile row), all filling the same category, and the last
    // one to run must not erase what the others already appended.
    // Returned MapElement::points pointers alias this object's OWN
    // storage_ and stay valid exactly as long as this object is not
    // destroyed and does not run another ingest() -- the same contract
    // SceneAssembly's own header documents for every adapter.
    void fill(micropilot::visualization_app::SceneAssembly& out) const;

    const AdapterStats& stats() const { return stats_; }

private:
    struct StoredElement
    {
        std::vector<mpviz::Vec3> points;
        uint8_t is_polygon{0};
    };
    // One marker key can now expand into several MapElements (a dashed
    // centerline's dash pieces) -- see ingest()'s dash-chopping. Every
    // OTHER marker (non-dashed, or a polygon) still stores exactly one
    // StoredElement per key; the vector is size 1 for those, never fanned
    // out. DELETE(m.ns, m.id) erases the whole vector, i.e. every dash
    // piece, in one shot -- ROS Marker semantics don't have a "half a
    // marker" concept and neither does this.
    using Key = std::pair<std::string, int32_t>;  // (marker.ns, marker.id)
    struct KeyHash
    {
        size_t operator()(const Key& k) const noexcept
        {
            return std::hash<std::string>{}(k.first) ^
                   (std::hash<int32_t>{}(k.second) << 1);
        }
    };

    ProfileRow row_;
    const micropilot::visualization_app::FrameTransformer& tf_;
    std::unordered_map<Key, std::vector<StoredElement>, KeyHash> storage_;
    AdapterStats stats_;
    // Separate from stats_.last_msg_sec: this tracks the last ACCEPTED
    // rebuild for max_rate_hz gating, not the last message merely
    // received (a rate-limited-away message still bumps stats_.msgs but
    // must not reset this, or the cooldown would never actually apply).
    double last_rebuild_sec_{-1.0};
};

}  // namespace mpviz_node
