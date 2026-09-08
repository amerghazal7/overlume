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
 *  Namespace rules (profile.hpp's classify()/match_rule()) decide is_polygon
 *  (NsRender::kPolygon) vs polyline (kPolyline) vs dropped (kDrop) -- see
 *  the shipped urban_profile.yaml / sim_profile.yaml hd_map rows. Epic 3
 *  Task 1 (VM-036, ADR-0004): the matched NsRule's `kind` field (MapKind,
 *  scene.h) is carried straight onto the stored/emitted MapElement, and
 *  `lane_id` is the marker's own `id` for CENTERLINE/LEFT_BOUNDARY/
 *  RIGHT_BOUNDARY/ROAD_EDGE (0 for everything else -- crosswalk/stopline/
 *  junction/other aren't lane-paired). This class still never parses a
 *  namespace STRING itself beyond match_rule()'s verdict.
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
    // storage_ (or, for synthesized ROAD_SURFACE elements, this object's
    // OWN road_surface_points_ cache, rebuilt fresh on every call) and stay
    // valid exactly as long as this object is not destroyed and does not
    // run another ingest()/fill() -- the same contract SceneAssembly's own
    // header documents for every adapter.
    //
    // Road-edge detection (user directive 2026-09-08): before road-surface
    // fill runs, a LEFT_BOUNDARY/RIGHT_BOUNDARY element whose polyline has
    // no near-coincident opposite-side boundary from a DIFFERENT lane_id
    // has its EMITTED kind promoted to ROAD_EDGE (the underlying LEFT_
    // BOUNDARY/RIGHT_BOUNDARY pairing used by road-surface fill, below, is
    // unaffected -- see hd_map.cpp's IsRoadEdge()/fill() for the measured
    // threshold).
    //
    // Road-surface fill (Epic 3 Task 1 / VM-036, decision #5): pairs every
    // LEFT_BOUNDARY/RIGHT_BOUNDARY element sharing a lane_id, resamples
    // both rails to kRoadFillSamples stations by normalized arc length
    // (rail point counts are NOT guaranteed to match -- verified on real
    // data), and emits one extra kind==ROAD_SURFACE MapElement per paired
    // lane (point_count == 2*kRoadFillSamples: points[0..16) left rail,
    // points[16..32) right rail, index-parallel by station). A lane_id
    // present on only one rail emits no ROAD_SURFACE element for it --
    // silently dropped, not malformed (spec §9).
    void fill(micropilot::visualization_app::SceneAssembly& out) const;

    const AdapterStats& stats() const { return stats_; }

private:
    struct StoredElement
    {
        std::vector<mpviz::Vec3> points;
        uint8_t is_polygon{0};
        mpviz::MapKind kind{mpviz::MapKind::OTHER};  // NEW (VM-036): from the matched NsRule
        uint32_t lane_id{0};                          // NEW (VM-036): marker.id, lane kinds only
    };
    // Epic 3 Task 1 (VM-036, decision #3): dashing moved renderer-side, so
    // the adapter never chops a marker into pieces any more -- every key
    // maps to exactly one StoredElement now. The vector wrapper is kept
    // (not collapsed to a bare StoredElement) purely to avoid touching
    // ingest()/fill()'s existing shape for zero behavioural gain; it is
    // always size 1. DELETE(m.ns, m.id) erases it in one shot.
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
    // Road-surface fill (VM-036, decision #5): rebuilt from scratch at the
    // START of every fill() call (never touched by ingest()) -- holds the
    // resampled two-rail point buffers the synthesized ROAD_SURFACE
    // MapElements point into. `mutable` because fill() is const (same
    // "logically read-only, physically caches a derived buffer" shape as
    // every other adapter's fill()-time geometry synthesis).
    mutable std::vector<std::vector<mpviz::Vec3>> road_surface_points_;
    AdapterStats stats_;
    // Separate from stats_.last_msg_sec: this tracks the last ACCEPTED
    // rebuild for max_rate_hz gating, not the last message merely
    // received (a rate-limited-away message still bumps stats_.msgs but
    // must not reset this, or the cooldown would never actually apply).
    double last_rebuild_sec_{-1.0};
};

}  // namespace mpviz_node
