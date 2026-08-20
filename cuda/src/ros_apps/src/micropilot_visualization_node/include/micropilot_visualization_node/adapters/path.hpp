#pragma once
/** @file path.hpp
 *  @brief PathAdapter (Epic 2 Task 5 / VM-023): nav_msgs/Path -> PathRibbon.
 *
 *  Ctor shape matches every Epic 2 adapter (epic2 plan, "Node: adapter
 *  shape"): (row, tf). One instance per profile row -- both shipped
 *  profiles ship FOUR path rows over THREE roles (BEHAVIOR, LOCAL x2,
 *  GLOBAL -- fixture gap 2), and every instance's fill() APPENDS into the
 *  shared SceneAssembly (see that header's own comment on why).
 *
 *  Role comes from `row.role` ("behavior"|"global"|"local"), NEVER the
 *  topic name -- renaming a topic is a YAML edit, not a code edit
 *  (plan Task 5 Step 1, RoleComesFromTheProfileRowNotTheTopicName).
 *
 *  Positions ONLY: PathRibbon is frozen {role, points, point_count,
 *  last_update_sec} -- no orientation field, and the recorded
 *  /behavior_path_planner/output_path_visualization poses carry IDENTITY
 *  orientations for every pose anyway, so reading pose.orientation for
 *  anything would be silently reading nothing. Each pose's
 *  pose.pose.position is transformed into the map frame via `tf` (ONE
 *  lookup per ingest() call, from the Path's own header -- never per
 *  pose, same rule as every other Epic 2 adapter) and stored.
 *
 *  Malformed handling (spec §9): an empty Path or a single-pose Path has no
 *  polyline to draw -- dropped whole, ++dropped_malformed, previously
 *  stored ribbon (if any) keeps rendering (same "drop the primitive, don't
 *  propagate, don't erase what still renders" rule as hd_map/dynamic_objects).
 *  A TF lookup failure drops the whole message (++dropped_no_tf), same
 *  effect.
 *
 *  A new (valid) Path REPLACES the stored one wholesale, never appends/
 *  merges -- e.g. an 803-pose path followed by a 33-pose path on the same
 *  topic yields point_count == 33 on the next fill() (Task 5 Step 1,
 *  PathChangeReplacesRatherThanAppends).
 *
 *  Staleness: node-side only (epic2 plan, "Staleness"). This adapter has no
 *  timeout logic of its own -- visualization_node.cpp checks
 *  stats().last_msg_sec against the row's timeout_sec and stops calling
 *  fill() for a stale row, mirroring mark_stale_tick()'s use in the
 *  dynamic_objects loop. Unlike MapElement, PathRibbon DOES carry
 *  last_update_sec, so the library fades it via the shared
 *  staleness_alpha() ramp (ribbon.cpp) rather than popping.
 */

#include <cstdint>
#include <string>
#include <vector>

#include <nav_msgs/msg/path.hpp>

#include "micropilot_visualization_node/adapter_stats.hpp"
#include "micropilot_visualization_node/frame_transform.hpp"
#include "micropilot_visualization_node/profile.hpp"
#include "micropilot_visualization_node/scene_assembly.hpp"
#include "visual_renderer/scene.h"

namespace mpviz_node
{

class PathAdapter
{
public:
    PathAdapter(const ProfileRow& row, const micropilot::visualization_app::FrameTransformer& tf);

    // ROS callback thread. See this file's header comment for the full
    // frame/malformed/replace rules.
    void ingest(const nav_msgs::msg::Path& msg, double sim_time_sec);

    // Timer thread, before set_scene(). APPENDS at most ONE PathRibbon into
    // out.paths (never assigns/replaces it) -- see SceneAssembly's own
    // header comment: multiple path rows (four, over three roles) all fill
    // the same category, and the last one to run must not erase the
    // others. Emits nothing if this adapter has never received a valid
    // (>=2 pose) Path. Returned PathRibbon::points aliases this object's
    // OWN storage_ and stays valid exactly as long as this instance is not
    // destroyed and does not run another ingest() -- the same contract
    // every other Epic 2 adapter's fill() documents.
    void fill(micropilot::visualization_app::SceneAssembly& out) const;

    const AdapterStats& stats() const { return stats_; }

    // Node-side staleness bookkeeping (epic2 plan, "Staleness" -- the node
    // owns timeout_sec, not the adapter), identical shape to
    // DynamicObjectsAdapter::mark_stale_tick().
    void mark_stale_tick() { ++stats_.dropped_stale; }

private:
    ProfileRow row_;
    const micropilot::visualization_app::FrameTransformer& tf_;
    mpviz::PathRole role_{mpviz::PathRole::LOCAL};
    std::vector<mpviz::Vec3> points_;
    double last_update_sec_{0.0};
    AdapterStats stats_;
};

}  // namespace mpviz_node
