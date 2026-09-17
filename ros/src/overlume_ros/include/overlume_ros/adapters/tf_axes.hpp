#pragma once
/** @file tf_axes.hpp
 *  @brief TfAxesAdapter (Epic 2 Task 8 / VM-027 Step 7): the "full TF axes"
 *  debug layer spec §5/§7 name under the generic-marker fallback -- a
 *  PRODUCER, not a subscriber. Nothing in this stack publishes TF frames as
 *  MarkerArray messages (not the recorded bag, not either rviz config), so
 *  there is no topic to subscribe to; this adapter walks the node's own
 *  tf2_ros::Buffer directly instead.
 *
 *  Ctor takes (row, buffer) -- NOT (row, FrameTransformer&) like every
 *  other Epic 2 adapter, even though the POSITIONAL shape matches
 *  ("(row, tf)"): FrameTransformer only exposes a single-target-frame,
 *  header-stamped lookup() (one lookup per MESSAGE, from a marker's own
 *  header.frame_id), which is the wrong shape for "enumerate every frame
 *  the buffer currently knows about and look each one up against `map`
 *  directly". Widening FrameTransformer's contract for this one caller
 *  isn't worth it; this class takes the raw `tf2_ros::Buffer` (the same
 *  object overlume_node.cpp already owns) plus its own
 *  `target_frame` (defaults to "map", matching every other adapter's
 *  frame_transformer_ target).
 *
 *  `adapter: tf_axes` rows carry NO `topic`/`type` (Task 1 Step 2's loader
 *  enforces this -- profile.cpp's ValidateRow) since there is nothing to
 *  subscribe to. This class has NO ingest() at all (unlike every other
 *  adapter, which has one) -- the node's wiring loop for `adapter:
 *  tf_axes` rows has no subscription branch; it just constructs this
 *  adapter and calls fill() every tick.
 *
 *  fill() is NOT const (unlike every ingest()-based adapter's fill(),
 *  which only reads storage_ that ingest() already built ahead of time):
 *  this class has no ingest() event to build storage_ from, so fill()
 *  itself does the live tf2_ros::Buffer walk, updates stats_, and rebuilds
 *  its own storage_ fresh every call. `sim_time_sec` is stamped onto every
 *  emitted GenericMarker's last_update_sec AS-OF THIS CALL -- unlike an
 *  ingest()-based category (whose last_update_sec is frozen at message
 *  receipt and fades once messages stop arriving), a TF frame is
 *  regenerated fresh from the LIVE tree every single fill() call, so it
 *  must never read as stale: stamping "now" every time is what keeps
 *  staleness_alpha() reading fully-fresh permanently, which is the
 *  correct rendering for a continuously-live debug layer. The lifetime
 *  contract for the returned GenericMarker::points pointers is otherwise
 *  identical to every other adapter's fill(): they alias this object's
 *  OWN storage and stay valid exactly as long as this instance is not
 *  destroyed and does not run another fill().
 *
 *  Per frame: three GenericMarker LINE_LISTs (X red, Y green, Z blue),
 *  each a single 2-point segment from the frame's origin to a point 0.5 m
 *  out along that axis in the frame's own orientation, color[3] (alpha)
 *  == 1. A frame whose map<-frame lookup throws (a TF tree mid-startup is
 *  normal, not an error) is skipped and counted in dropped_no_tf, never
 *  fatal.
 *
 *  Skipped per the plan: per-frame labels (that's TEXT, i.e. VM-030/Epic
 *  3) and a frame filter (add one when somebody's tree is big enough to
 *  hurt).
 */

#include <string>
#include <vector>

#include <tf2_ros/buffer.h>

#include "overlume_ros/adapter_stats.hpp"
#include "overlume_ros/profile.hpp"
#include "overlume_ros/scene_assembly.hpp"
#include "overlume/scene.h"

namespace overlume_node
{

class TfAxesAdapter
{
public:
    explicit TfAxesAdapter(const ProfileRow& row, const tf2_ros::Buffer& buffer,
                           std::string target_frame = "map");

    // Timer thread. See this file's own header comment for why this is
    // NOT const and has no ingest() counterpart. APPENDS into out.markers
    // (never assigns/replaces it), same "category can have multiple
    // contributing adapters" contract as every other adapter's fill() --
    // though today's shipped profiles carry at most one tf_axes row (Task
    // 1 Step 3, still commented out).
    void fill(overlume::ros::SceneAssembly& out, double sim_time_sec);

    const AdapterStats& stats() const { return stats_; }

private:
    ProfileRow row_;
    const tf2_ros::Buffer& buffer_;
    std::string target_frame_;
    std::vector<overlume::Vec3> point_storage_;
    std::vector<overlume::GenericMarker> axis_markers_;
    AdapterStats stats_;
};

}  // namespace overlume_node
