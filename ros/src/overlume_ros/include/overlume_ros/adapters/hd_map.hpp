// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once
/** @file hd_map.hpp
 *  @brief HdMapAdapter (Epic 2 Task 2 / VM-024): MarkerArray -> MapElement.
 *
 *  Ctor shape (row, tf) matches every Epic 2 adapter. No node handle, no
 *  subscription -- overlume_node.cpp owns create_subscription() and
 *  routes the callback into ingest(). One instance per profile row; every
 *  instance's fill() APPENDS into the shared SceneAssembly (see that
 *  header: the last adapter to run must not erase the others).
 *
 *  Namespace rules (profile.hpp's classify()/match_rule()) decide is_polygon
 *  vs polyline vs dropped. The matched NsRule's `kind` field (MapKind,
 *  scene.h) is carried straight onto the stored/emitted MapElement
 *  (ADR-0004), and `lane_id` is the marker's own `id` for CENTERLINE/
 *  LEFT_BOUNDARY/RIGHT_BOUNDARY/ROAD_EDGE (0 otherwise). This class never
 *  parses a namespace STRING itself beyond match_rule()'s verdict.
 *
 *  Frames: every marker's points are moved into the map frame via `tf`
 *  BEFORE being stored -- ONE lookup per ingest() call (from the first
 *  marker's header), never per marker.
 *
 *  Marker.msg points[] on a LINE_STRIP are RELATIVE to marker.pose, composed
 *  as frame_transform * (marker_pose * point) -- the marker pose is IN the
 *  header frame, so it composes INSIDE the frame transform. Applied per
 *  marker, before dash chopping. A NaN pose is malformed (dropped_malformed).
 *
 *  Staleness (VM-034): every emitted MapElement carries MapElement::
 *  last_update_sec so the map layer fades via the library's shared
 *  staleness_alpha() like every other category, fading between
 *  kStaleFadeStartSec and kStaleFadeTimeoutSec (renderer_internal.hpp).
 *  fill() stamps e.last_update_sec = last_recv_sec_ + (row_.timeout_sec -
 *  kMapFadeWindowSec) (kMapFadeWindowSec, hd_map.cpp, MIRRORS the library's
 *  kStaleFadeTimeoutSec -- if that moves, this moves). last_recv_sec_ is the
 *  sim time of this row's last TF-lookup-succeeded ingest() call (set
 *  BEFORE the max_rate_hz rebuild gate), deliberately NOT stats_.last_msg_sec
 *  (the last ACCEPTED rebuild): a throttled row keeps receiving at its real
 *  publish rate between rebuilds, and gating the fade on rebuild cadence
 *  would blink the layer dark while the topic is genuinely alive. This
 *  adapter has no hard-cutoff logic of its own -- overlume_node.cpp
 *  checks stats().last_msg_sec against the row's timeout_sec and stops
 *  calling fill() for a stale row.
 *
 *  Road-edge promotion, junction-cleanup clipping/cutting, and road-surface
 *  fill: see hd_map.cpp and plan 2026-08-18-visual-mode-epic3.md (decisions
 *  #1-#5, junction addenda) for the algorithms and measured thresholds.
 */

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <visualization_msgs/msg/marker_array.hpp>

#include "overlume_ros/adapter_stats.hpp"
#include "overlume_ros/frame_transform.hpp"
#include "overlume_ros/profile.hpp"
#include "overlume_ros/scene_assembly.hpp"
#include "overlume/scene.h"

namespace overlume::ros {

class HdMapAdapter {
public:
    HdMapAdapter(const ProfileRow& row, const overlume::ros::FrameTransformer& tf);

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
    // (never assigns/replaces it) -- see SceneAssembly's own header: this
    // node runs multiple HdMapAdapter instances (one per hd_map profile
    // row), all filling the same category, and the last one to run must
    // not erase what the others already appended. Returned
    // MapElement::points pointers alias this object's OWN storage_ (or,
    // for synthesized ROAD_SURFACE elements, road_surface_points_,
    // rebuilt fresh on every call) and stay valid exactly as long as this
    // object is not destroyed and does not run another ingest()/fill().
    //
    // Road-edge promotion, junction-cleanup clipping/cutting, and
    // road-surface fill (pairs LEFT_BOUNDARY/RIGHT_BOUNDARY by lane_id,
    // resampled to kRoadFillSamples stations by normalized arc length --
    // rail point counts are NOT guaranteed to match; point_count ==
    // 2*kRoadFillSamples, left rail then right rail, index-parallel by
    // station; a lane_id on only one rail emits no ROAD_SURFACE element,
    // silently dropped per spec §9): see hd_map.cpp and plan
    // 2026-08-18-visual-mode-epic3.md (decisions #1-#5, junction addenda)
    // for the algorithms and measured thresholds.
    void fill(overlume::ros::SceneAssembly& out) const;

    const AdapterStats& stats() const { return stats_; }

private:
    struct StoredElement {
        std::vector<overlume::Vec3> points;
        uint8_t is_polygon{0};
        overlume::MapKind kind{overlume::MapKind::OTHER};  // from the matched NsRule
        uint32_t lane_id{0};                               // marker.id; lane kinds only, else 0
    };
    // Dashing moved renderer-side (VM-036); every key maps to exactly one
    // StoredElement now (vector wrapper kept only to avoid reshaping
    // ingest()/fill(); always size 1). DELETE(m.ns, m.id) erases it in one
    // shot.
    using Key = std::pair<std::string, int32_t>;  // (marker.ns, marker.id)
    struct KeyHash {
        size_t operator()(const Key& k) const noexcept {
            return std::hash<std::string>{}(k.first) ^ (std::hash<int32_t>{}(k.second) << 1);
        }
    };

    ProfileRow row_;
    const overlume::ros::FrameTransformer& tf_;
    std::unordered_map<Key, std::vector<StoredElement>, KeyHash> storage_;
    // Rebuilt from scratch at the START of every fill() call (never touched
    // by ingest()) -- holds the resampled two-rail point buffers the
    // synthesized ROAD_SURFACE MapElements point into. `mutable` because
    // fill() is const.
    mutable std::vector<std::vector<overlume::Vec3>> road_surface_points_;
    // Same "mutable, fill()-time cache" shape as road_surface_points_ above
    // -- holds every clipped/cut ROAD_EDGE and (junction_interior_boundaries:
    // false) LEFT_/RIGHT_BOUNDARY sub-polyline the MapElements below point
    // into.
    mutable std::vector<std::vector<overlume::Vec3>> junction_cut_points_;
    AdapterStats stats_;
    // Separate from stats_.last_msg_sec: this tracks the last ACCEPTED
    // rebuild for max_rate_hz gating, not the last message merely
    // received (a rate-limited-away message still bumps stats_.msgs but
    // must not reset this, or the cooldown would never actually apply).
    double last_rebuild_sec_{-1.0};
    // Sim time of the last ingest() call whose TF lookup succeeded, set
    // BEFORE the max_rate_hz gate above so a rate-limited-away message
    // still advances it -- see this file's Staleness doc comment for why
    // fill() bases MapElement::last_update_sec on this rather than
    // last_rebuild_sec_/stats_.last_msg_sec.
    double last_recv_sec_{-1.0};
};

}  // namespace overlume::ros
