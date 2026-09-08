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
 *  Staleness (Epic 3 Task 2 / VM-034, closes the STATED DEVIATION this
 *  comment used to record): every emitted MapElement now carries a real
 *  MapElement::last_update_sec (appended, ADR-0004), so the map layer fades
 *  via the library's shared staleness_alpha() like every other category --
 *  it no longer pops. The library's fade BEGINS at kStaleFadeStartSec =
 *  0.5s of age and COMPLETES (alpha 0) at kStaleFadeTimeoutSec = 1.0s
 *  (renderer_internal.hpp) -- not "only 1.0s of silence fades it," a claim
 *  this comment used to make and got wrong; a straight last_update_sec ==
 *  last_recv_sec_ stamp sawtoothed the whole layer between alpha 1.0 and
 *  0.0 once per receipt on any row received slower than ~2 Hz, and left a
 *  publish-once/transient_local row (last_recv_sec_ frozen at its one
 *  receipt while sim_time keeps climbing) faded to alpha 0 within 1s of
 *  that receipt (2026-09-08 review fix, round 2).
 *
 *  Fixed by stamping e.last_update_sec = last_recv_sec_ + (row_.timeout_sec
 *  - kMapFadeWindowSec) (see hd_map.cpp's kMapFadeWindowSec comment) --
 *  last_recv_sec_ is still the sim time of this row's last TF-lookup-
 *  succeeded ingest() call (set BEFORE the max_rate_hz rebuild gate, see
 *  ingest()'s own comment), deliberately NOT stats_.last_msg_sec, the last
 *  ACCEPTED rebuild: a throttled row (e.g. /hd_map_global_elements'
 *  max_rate_hz: 0.5) keeps receiving at its real publish rate between
 *  rebuilds, and gating the fade on the rebuild cadence would blink the
 *  layer dark for most of every rebuild interval even while the topic is
 *  genuinely alive. The (timeout_sec - kMapFadeWindowSec) offset places
 *  that same liveness stamp's fade window in the last kMapFadeWindowSec
 *  before this row's own hard cutoff instead of right after last_recv_sec_
 *  itself, so a row received often enough stays continuously opaque and a
 *  row that goes silent (genuinely, or because it only ever published
 *  once) ramps out right before the node stops calling fill() for it,
 *  rather than popping or fading early while still being shown. This
 *  adapter still has no timeout logic of its own for the hard cutoff --
 *  visualization_node.cpp separately checks stats().last_msg_sec against
 *  the row's timeout_sec and stops calling fill() for a stale row.
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
    // Junction cleanup (same directive, same-day refinement): a promoted
    // ROAD_EDGE polyline crossing through a JUNCTION-kind ring in this
    // message gets clipped to stop at the ring's boundary and resume past
    // it; independently, any two ROAD_EDGE polylines from different
    // lane_ids that cross in 2D (the common case on a row with no JUNCTION
    // geometry at all) get trimmed back from their crossing point. Interior
    // LEFT_/RIGHT_BOUNDARY separators are never crossing-cut, and are only
    // ring-clipped when `row.junction_interior_boundaries` is false
    // (default true) -- see hd_map.cpp's junction-cleanup block and
    // ProfileRow::junction_interior_boundaries for the full rationale.
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
    // Junction cleanup (user directive 2026-09-08): rebuilt from scratch at
    // the START of every fill() call, same "mutable, fill()-time cache"
    // shape as road_surface_points_ above -- holds every clipped/cut
    // ROAD_EDGE and (junction_interior_boundaries: false) LEFT_/RIGHT_
    // BOUNDARY sub-polyline the MapElements below point into.
    mutable std::vector<std::vector<mpviz::Vec3>> junction_cut_points_;
    AdapterStats stats_;
    // Separate from stats_.last_msg_sec: this tracks the last ACCEPTED
    // rebuild for max_rate_hz gating, not the last message merely
    // received (a rate-limited-away message still bumps stats_.msgs but
    // must not reset this, or the cooldown would never actually apply).
    double last_rebuild_sec_{-1.0};
    // VM-034 review fix: the sim time of the last ingest() call whose TF
    // lookup succeeded, set BEFORE the max_rate_hz gate above so a
    // rate-limited-away message still advances it -- fill() stamps
    // MapElement::last_update_sec from this PLUS an offset into the row's
    // own timeout_sec (see this class's own Staleness doc comment for the
    // offset and for why the base must NOT be last_rebuild_sec_/
    // stats_.last_msg_sec instead).
    double last_recv_sec_{-1.0};
};

}  // namespace mpviz_node
