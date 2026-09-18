// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once
/** @file collision.hpp
 *  @brief CollisionAdapter (Epic 2 Task 7 / VM-026): MarkerArray ->
 *  overlume::AlertPolygon.
 *
 *  Ctor shape matches every Epic 2 adapter (epic2 plan, "Node: adapter
 *  shape"): (row, tf). UNLIKE HdMapAdapter, severity here is NOT a
 *  per-namespace classify() verdict -- the five shipped collision profile
 *  rows carry no `namespaces:` rules at all (config/urban_profile.yaml).
 *  Severity is driven entirely by the ROW'S OWN `role` field, through the
 *  ONE role -> severity table in this file's .cpp (Task 1 Step 3's shipped
 *  mapping, sourced from config/rviz/urban_config.rviz:175-223 -- see the
 *  epic2 plan's Task 7 table): `collision` -> 2 critical, `predicted`/
 *  `merged_object` -> 1 warning, `sweep`/`merged_ego` -> 0 info. Both
 *  ego-side rows (`sweep`, `merged_ego`) land on severity 0 for a second
 *  reason beyond "debug geometry": `AlertPolygon` is frozen at {points,
 *  point_count, severity, last_update_sec} with NO role, so the renderer
 *  cannot tell a sweep from a merged polygon -- "the ego sweep gets a
 *  ghost alpha" is therefore only expressible as "severity 0 does" (see
 *  the plan's "…and the material that can actually do it").
 *  severity_for_role() THROWS on any role outside those five -- Task 1's
 *  profile validator (profile.cpp's RoleSets) already rejects an unknown
 *  role for `adapter: collision` before a row ever reaches this ctor; this
 *  function must never paper over that with a silent info default.
 *
 *  FIXTURE GAP 4 (epic2 plan): the five collision-checker topics were
 *  SILENT in the recorded bag (a calm scenario, zero messages) -- every
 *  fixture this adapter's tests use is synthetic, hand-written,
 *  unvalidated against a live publisher.
 *
 *  ingest(): one AlertPolygon per LINE_STRIP marker. Marker.msg semantics
 *  identical to HdMapAdapter's own: points[] are RELATIVE to marker.pose,
 *  composed BEFORE the frame transform; a zero/degenerate orientation
 *  quaternion is treated as
 *  identity (matching rviz), never NaN'd through tf2; flatten_z on stored
 *  points (frame_transform.hpp). Producers may or may not repeat the
 *  first point as the last -- this adapter normalizes storage to always
 *  be CLOSED (first == last), so the library's fan triangulation
 *  (triangulate_convex_polygon) always sees a genuinely closed loop
 *  regardless of which convention the producer used. Malformed (a NaN
 *  anywhere, or fewer than 3 DISTINCT points once consecutive duplicates
 *  are collapsed) -> dropped_malformed, neighbours in the same message
 *  still come through (spec §9). DELETEALL clears every polygon this
 *  adapter is tracking; DELETE(ns, id) removes one. last_update_sec is
 *  stamped from ingest time, per polygon -- AlertPolygon (unlike
 *  MapElement) carries its own, so the library fades this category rather
 *  than the node popping it (see overlume_node.cpp's collision-row
 *  loop: same "stop filling past timeout_sec, mark_stale_tick() instead"
 *  shape as dynamic_objects/path/ogm). Storage outlives fill() (same
 *  alias-lifetime contract as every other Epic 2 adapter).
 */

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

// Task 7's ONE role -> severity table. Throws std::invalid_argument on any
// role outside the five shipped ones (`collision`, `predicted`,
// `merged_object`, `sweep`, `merged_ego`) -- see this file's own header
// comment for why that must never become a silent info default.
uint8_t severity_for_role(const std::string& role);

class CollisionAdapter {
public:
    CollisionAdapter(const ProfileRow& row, const overlume::ros::FrameTransformer& tf);

    // ROS callback thread. See this file's header comment for the full
    // malformed/close/drop rules.
    void ingest(const visualization_msgs::msg::MarkerArray& msg, double sim_time_sec);

    // Timer thread, before set_scene(). APPENDS into out.alerts (never
    // assigns/replaces it) -- SceneAssembly's own header: this node runs
    // FIVE CollisionAdapter instances (one per profile row), all filling
    // the same category, and the last one to run must not erase what the
    // others already appended. Returned AlertPolygon::points pointers
    // alias this object's OWN storage_ and stay valid exactly as long as
    // this instance is not destroyed and does not run another ingest().
    void fill(overlume::ros::SceneAssembly& out) const;

    const AdapterStats& stats() const { return stats_; }

    // Node-side staleness bookkeeping (epic2 plan, "Staleness" -- the node
    // owns timeout_sec, not the adapter): the node calls this once per
    // timer tick that it withholds this row's alerts for being past
    // row.timeout_sec, instead of calling fill(). Same ponytail as
    // DynamicObjectsAdapter's/PathAdapter's identical method: counts
    // withheld TICKS, not withheld polygons.
    void mark_stale_tick() { ++stats_.dropped_stale; }

private:
    using Key = std::pair<std::string, int32_t>;  // (marker.ns, marker.id)
    struct KeyHash {
        size_t operator()(const Key& k) const noexcept {
            return std::hash<std::string>{}(k.first) ^ (std::hash<int32_t>{}(k.second) << 1);
        }
    };
    struct StoredPolygon {
        std::vector<overlume::Vec3> points;  // always CLOSED: points.front() == points.back()
        double last_update_sec{0.0};
    };

    ProfileRow row_;
    const overlume::ros::FrameTransformer& tf_;
    uint8_t severity_;  // computed once at construction from row_.role
    std::unordered_map<Key, StoredPolygon, KeyHash> storage_;
    AdapterStats stats_;
};

}  // namespace overlume::ros
