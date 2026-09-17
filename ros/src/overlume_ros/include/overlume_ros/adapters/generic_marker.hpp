// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once
/** @file generic_marker.hpp
 *  @brief GenericMarkerAdapter (Epic 2 Task 8 / VM-027): the spec §7
 *  parity guarantee -- ANY MarkerArray topic renders via one YAML row.
 *  MarkerArray -> overlume::GenericMarker[].
 *
 *  Ctor shape matches every Epic 2 adapter (epic2 plan, "Node: adapter
 *  shape"): (row, tf). One instance per profile row with `adapter:
 *  generic` (today's shipped profiles carry exactly one:
 *  `/sim/ground_truth/boxes`, role `neutral`, `best_effort: true` -- the
 *  bag's only non-map-frame, only lifetime-expiring topic).
 *
 *  ROS Marker::type -> overlume::MarkerPrimitive, all 12 ROS types over the
 *  FROZEN 10-value enum (scene.h:22-25, Epic 2 may not add to it):
 *    ARROW(0)->ARROW, CUBE(1)->CUBE, SPHERE(2)->SPHERE, CYLINDER(3)->
 *    CYLINDER, LINE_STRIP(4)->LINE_STRIP, LINE_LIST(5)->LINE_LIST,
 *    POINTS(8)->POINTS, TEXT_VIEW_FACING(9)->TEXT, MESH_RESOURCE(10)->
 *    MESH, TRIANGLE_LIST(11)->TRIANGLE_LIST. CUBE_LIST(6)/SPHERE_LIST(7)
 *    are the two ROS types without a slot -- the ONLY freeze-respecting
 *    route is adapter-side FAN-OUT: one N-point CUBE_LIST/SPHERE_LIST
 *    becomes N GenericMarker CUBE/SPHERE entries, one per point, all
 *    sharing marker.scale, each taking colors[i] when
 *    colors.size()==points.size() else marker.color. Without this fan-out
 *    a published CUBE_LIST/SPHERE_LIST is a silent
 *    UnknownOrUnsupportedPrimitiveIsSkippedAndCounted increment instead of
 *    the parity guarantee.
 *
 *  Namespace rules (profile.hpp's classify()) decide render vs. drop
 *  (dropped_by_rule), same ONE rule every marker adapter in this node
 *  uses -- `is_polygon`/`kind` have no meaning here (GenericMarker
 *  carries neither), only the render/drop verdict matters.
 *
 *  Frames: marker.pose composition (points[]/CUBE_LIST/SPHERE_LIST points
 *  are RELATIVE to marker.pose, composed BEFORE the frame transform,
 *  zero/degenerate quaternion treated as identity) + ONE FrameTransformer
 *  lookup per ingest() call (from the first marker's header) + flatten_z
 *  on stored points/positions -- identical conventions to hd_map.cpp/
 *  dynamic_objects.cpp/collision.cpp. `/sim/ground_truth/boxes` publishes
 *  in base_link, the bag's one non-map frame -- BaseLinkMarkersLandAround
 *  TheEgoNotTheMapOrigin asserts POSITIONS against a hand-built TF, not
 *  pixels (a pixel-diff alone would pass even with the frame transform
 *  skipped entirely).
 *
 *  Non-zero `lifetime` (builtin_interfaces/Duration) expiry: judged
 *  against INGEST sim time, swept at the top of every ingest() call
 *  (before that message's own markers are processed) -- a stored entry
 *  whose `expires_at_sec` (ingest_sim_time_of_its_own_ADD + lifetime) is
 *  at or before the CURRENT message's sim_time_sec is dropped. `lifetime
 *  == 0` (ROS convention: forever) never expires this way -- only
 *  DELETE/DELETEALL removes it. `/sim/ground_truth/boxes` uses 0.2 s --
 *  the bag's one lifetime exerciser.
 *
 *  Malformed (spec §9, per-primitive, dropped + counted, neighbours in the
 *  same message still come through): a marker.type outside the 12 known
 *  ROS values; a NaN pose; a zero-or-negative scale axis on a
 *  pose+scale primitive (CUBE/SPHERE/CYLINDER/ARROW/MESH/CUBE_LIST/
 *  SPHERE_LIST); LINE_STRIP/LINE_LIST/TRIANGLE_LIST/CUBE_LIST/SPHERE_LIST
 *  with too few points for their shape; MESH_RESOURCE with an empty
 *  mesh_resource.
 *
 *  Staleness: unlike HdMapAdapter's map_elements (which are frozen with no
 *  last_update_sec, spec §5's stated deviation), GenericMarker DOES carry
 *  last_update_sec, so this category gets the library's staleness FADE --
 *  overlume_node.cpp's generic-row loop is the "stop filling past
 *  timeout_sec, mark_stale_tick() instead" shape every other faded
 *  category already uses.
 *
 *  text/mesh_path string storage (TEXT/MESH_RESOURCE only) must outlive
 *  fill() -- owned by this adapter's own storage_, same alias-lifetime
 *  contract as every other Epic 2 adapter's points/label storage.
 */

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <visualization_msgs/msg/marker_array.hpp>

#include "overlume_ros/adapter_stats.hpp"
#include "overlume_ros/frame_transform.hpp"
#include "overlume_ros/profile.hpp"
#include "overlume_ros/scene_assembly.hpp"
#include "overlume/scene.h"

namespace overlume_node {

class GenericMarkerAdapter {
public:
    GenericMarkerAdapter(const ProfileRow& row, const overlume::ros::FrameTransformer& tf);

    // ROS callback thread. See this file's own header comment for the
    // full type-mapping/fan-out/frame/lifetime/malformed rules.
    void ingest(const visualization_msgs::msg::MarkerArray& msg, double sim_time_sec);

    // Timer thread, before set_scene(). APPENDS into out.markers (never
    // assigns/replaces it) -- SceneAssembly's own header: multiple
    // adapters (this one, plus a future second `generic` row) all fill
    // the same category. Returned GenericMarker::points/text/mesh_path
    // pointers alias this object's OWN storage_ and stay valid exactly as
    // long as this instance is not destroyed and does not run another
    // ingest().
    void fill(overlume::ros::SceneAssembly& out) const;

    const AdapterStats& stats() const { return stats_; }

    // Node-side staleness bookkeeping (epic2 plan, "Staleness" -- the node
    // owns timeout_sec, not the adapter), identical shape to every other
    // fill()-appends adapter's own mark_stale_tick().
    void mark_stale_tick() { ++stats_.dropped_stale; }

private:
    using Key = std::pair<std::string, int32_t>;  // (marker.ns, marker.id)

    // One stored entry per (ns, id) key. Exactly one of `points` (LINE_*/
    // POINTS/TRIANGLE_LIST) or `fan_positions` (CUBE_LIST/SPHERE_LIST
    // fan-out source) is ever populated; neither is for a plain pose+scale
    // primitive (CUBE/SPHERE/CYLINDER/ARROW/MESH/TEXT), which uses
    // `position`/`heading_rad`/`scale` directly. fill() reconstructs the
    // actual GenericMarker(s) fresh from this data every call (same
    // "storage holds raw geometry, fill() builds the output struct"
    // pattern as HdMapAdapter's StoredElement/CollisionAdapter's
    // StoredPolygon).
    struct StoredMarker {
        overlume::MarkerPrimitive primitive{overlume::MarkerPrimitive::CUBE};
        overlume::Vec3 position{};
        double heading_rad{0.0};
        overlume::Vec3 scale{1.0, 1.0, 1.0};
        float color[4]{0.0f, 0.0f, 0.0f, 0.0f};
        std::string text;                    // TEXT only
        std::string mesh_path;               // MESH only
        std::vector<overlume::Vec3> points;  // LINE_*/POINTS/TRIANGLE_LIST only

        // CUBE_LIST/SPHERE_LIST fan-out source: one GenericMarker CUBE/
        // SPHERE per entry in `fan_positions`, all sharing `scale`/`color`
        // above UNLESS `fan_colors` is non-empty (colors[i], one RGBA
        // per fanned point).
        std::vector<overlume::Vec3> fan_positions;
        std::vector<float> fan_colors;  // empty, or 4*fan_positions.size()

        double last_update_sec{0.0};
        // 0.0 == never expires (ROS Marker::lifetime convention: zero
        // duration means forever). See this file's own header comment for
        // the sweep contract.
        double expires_at_sec{0.0};
    };

    ProfileRow row_;
    const overlume::ros::FrameTransformer& tf_;
    // ORDERED map on purpose: fill() emits in iteration order and the
    // renderer keys its pooled slots by INDEX -- an unordered_map's rehash
    // after erase/insert churn reordered emissions, re-homing slots and
    // forcing per-tick mesh rebuilds (a glTF re-parse for MESH markers),
    // which spec 4.2's per-frame allocation forbids. std::map keeps a given
    // (ns,id) in the same slot every tick; Key is pair<string,int32_t>,
    // ordered for free.
    std::map<Key, StoredMarker> storage_;
    AdapterStats stats_;
};

}  // namespace overlume_node
