// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once
/** @file ogm.hpp
 *  @brief OgmAdapter (Epic 2 Task 6 / VM-025): nav_msgs/OccupancyGrid +
 *  map_msgs/OccupancyGridUpdate -> GroundGridLayer.
 *
 *  FIXTURE GAP 3 (epic2 plan, Task 6): there are ZERO OccupancyGrid topics
 *  in the recorded bag or stack -- every fixture and golden this task ships
 *  is SYNTHETIC. The ACs below are proven against hand-built data only;
 *  re-validate when an OGM-enabled recording exists.
 *
 *  Ctor shape matches every Epic 2 adapter (epic2 plan, "Node: adapter
 *  shape"): (row, tf). UNLIKE every other adapter, this one has TWO ingest
 *  entry points, not one -- ingest() (the full grid) and ingest_update()
 *  (a map_msgs::msg::OccupancyGridUpdate patch) -- because
 *  subscriptions_for() (profile.hpp, Task 1) returns TWO SubSpecs for an
 *  `ogm` row (the base topic + `update_topic`), and ONE adapter instance
 *  must consume both: an _updates-only row (no base topic ever arriving)
 *  would have nothing to patch, forever -- see
 *  OneRowYieldsTwoSubscriptionsAndOneAdapter, this task's regression test
 *  for exactly that trap. Adapters never subscribe and never hold an
 *  rclcpp::Node* (see "Node: adapter shape") -- the NODE
 *  (overlume_node.cpp) walks subscriptions_for(row) and binds each
 *  spec's `type` to the matching overload below.
 *
 *  kUnknownCell = 255: the sentinel this adapter writes for nav_msgs/
 *  OccupancyGrid's `-1` ("unknown"), and for any OTHER out-of-range int8
 *  value (anything outside 0..100) -- the frozen POD boundary
 *  (GroundGridLayer::cells is a bare uint8_t*, scene.h) cannot carry a
 *  shared enum/constant across the ABI, so this number is mirrored by
 *  comment in TWO other places: ground_grid.cpp (the renderer's caller) and
 *  ground_grid.mat's own header (the transfer function this sentinel
 *  feeds). 255 is reachable ONLY via a malformed/unknown cell -- 0..100 are
 *  the sole legal wire values -- so no legitimate occupancy reading can
 *  ever be confused with "unknown".
 *
 *  Conversion, not a copy (int8_t -> uint8_t): -1 -> kUnknownCell, 0..100
 *  pass through unchanged, anything else -> kUnknownCell +
 *  ++dropped_malformed ONCE PER MESSAGE (not per cell -- see
 *  UnknownCellsBecomeTheSentinelNotTwoFiftyFive). The SAME conversion runs
 *  on ingest_update()'s patch path -- a patch that skipped it would
 *  reintroduce raw `-1`s into an already-converted grid.
 *
 *  ingest_update() before any ingest() has ever succeeded (no base grid
 *  yet) is dropped and counted (dropped_malformed) -- no allocation, no
 *  crash, base untouched (there IS no base yet). An out-of-bounds update
 *  rect (x/y/width/height overrunning the base grid's own dims) is
 *  likewise dropped and counted, base untouched -- a partial patch is
 *  never applied to some of a rect and not the rest of it.
 *
 *  Role -> kind (scene.h's GroundGridLayer::kind, 0/1): `dynamic_ogm` -> 0,
 *  `gradient_ogm` -> 1 (the closed role set profile.cpp's ValidateRow
 *  already enforces for adapter: ogm rows).
 *
 *  fill()'s GroundGridLayer::cells points at THIS object's own storage_ (the
 *  full converted grid, patched in place by ingest_update()) and stays
 *  valid exactly as long as this instance is not destroyed and does not run
 *  another ingest()/ingest_update() call afterward -- the same contract
 *  every other Epic 2 adapter's fill() documents. A dims change (a fresh
 *  ingest() at a different width/height) reallocates storage_; a patch
 *  (ingest_update()) never does -- see PartialUpdatePatchesInPlaceWithout
 *  Resizing.
 *
 *  Staleness: node-side only (epic2 plan, "Staleness"), same split as
 *  every other adapter -- overlume_node.cpp checks stats().
 *  last_msg_sec against the row's timeout_sec and stops calling fill() for
 *  a stale row. GroundGridLayer DOES carry last_update_sec, so the library
 *  fades it via ground_grid.mat's own settable alpha (ground_grid.cpp)
 *  rather than popping, same as PathRibbon/TrackedObject.
 */

#include <cstdint>
#include <string>
#include <vector>

#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "overlume_ros/adapter_stats.hpp"
#include "overlume_ros/frame_transform.hpp"
#include "overlume_ros/profile.hpp"
#include "overlume_ros/scene_assembly.hpp"
#include "overlume/scene.h"

namespace overlume_node {

// See this file's header comment: the ONE number shared across the frozen
// POD boundary that nothing enforces except this comment mirror (here,
// ground_grid.cpp, ground_grid.mat).
inline constexpr uint8_t kUnknownCell = 255;

class OgmAdapter {
public:
    OgmAdapter(const ProfileRow& row, const overlume::ros::FrameTransformer& tf);

    // ROS callback thread, bound to the ROW'S OWN topic (subscriptions_
    // for(row)[0]). A full nav_msgs/OccupancyGrid REPLACES this adapter's
    // stored grid wholesale (dims, origin, resolution, every cell) -- see
    // this file's header comment for the int8->uint8 conversion rule.
    // Malformed guards (spec §9): a zero-sized grid, or `data.size()` not
    // matching `width*height`, drops the WHOLE message (++dropped_malformed,
    // previously-stored grid, if any, keeps rendering); a TF lookup failure
    // on `msg.header` drops it too (++dropped_no_tf), same effect.
    void ingest(const nav_msgs::msg::OccupancyGrid& msg, double sim_time_sec);

    // ROS callback thread, bound to the row's `update_topic`
    // (subscriptions_for(row)[1]). Rewrites EXACTLY the (x,y,width,height)
    // sub-rectangle in place -- cells outside it are byte-identical
    // afterward, and the grid is never resized/reallocated by a patch (see
    // this file's header comment). No base grid yet, or a rect that
    // overruns the base grid's own width/height -> dropped + counted
    // (++dropped_malformed), base untouched, no allocation, no crash.
    void ingest_update(const map_msgs::msg::OccupancyGridUpdate& msg, double sim_time_sec);

    // Timer thread, before set_scene(). APPENDS at most ONE GroundGridLayer
    // into out.grids (never assigns/replaces it) -- same "multiple ogm rows
    // must not erase each other" shape as every other category. Emits
    // nothing if this adapter has never received a valid full grid.
    // Returned GroundGridLayer::cells aliases this object's OWN storage_ and
    // stays valid exactly as long as this instance is not destroyed and does
    // not run another ingest()/ingest_update() call -- the same contract
    // every other Epic 2 adapter's fill() documents.
    void fill(overlume::ros::SceneAssembly& out) const;

    const AdapterStats& stats() const { return stats_; }

    // Node-side staleness bookkeeping (epic2 plan, "Staleness" -- the node
    // owns timeout_sec, not the adapter), identical shape to every other
    // adapter's mark_stale_tick().
    void mark_stale_tick() { ++stats_.dropped_stale; }

private:
    ProfileRow row_;
    const overlume::ros::FrameTransformer& tf_;
    uint8_t kind_{0};  // from row_.role: dynamic_ogm -> 0, gradient_ogm -> 1

    bool has_grid_{false};
    overlume::Vec3 origin_{};
    double resolution_m_{0.0};
    uint32_t width_cells_{0};
    uint32_t height_cells_{0};
    std::vector<uint8_t> cells_;  // width_cells_ * height_cells_, row-major, CONVERTED bytes
    double last_update_sec_{0.0};
    AdapterStats stats_;
};

}  // namespace overlume_node
