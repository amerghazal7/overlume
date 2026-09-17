#pragma once
/** @file trajectory_carpet.hpp
 *  @brief TrajectoryCarpetAdapter (VM-077, REDIRECTED 2026-09-10): MarkerArray
 *         -> overlume::TrajectoryCarpet, now carrying CENTERLINE STATIONS
 *         (one per dual-rail quad) instead of the raw wire triangle list.
 *
 *  Ctor shape matches every adapter in this node (row, tf).
 *
 *  REDIRECT (user directive, 2026-09-10, verbatim: "I just noticed a major
 *  flickering for the local path ribbon after adding the velocity profile
 *  rendering this way, I think i still prefer to treat it as [a] ribbon
 *  that can be stacked on top of local ribbon with margin"): the library
 *  side now renders this as an extruded RIBBON (trajectory_carpet.cpp), not
 *  a flat triangle list -- it needs a CENTERLINE, not the raw dual-rail
 *  geometry. The VM-077 flicker measurement report's own vertex-pairing
 *  finding supplies the exact extraction: every 6 wire points form one
 *  "quad" = two triangles `(A,B,C)` then `(A,C,D)`, where `{A,D}`/`{B,C}`
 *  are the two rails and consecutive quads share their trailing rail-pair
 *  as the next quad's leading pair. For quad `k` (flat indices `6k..6k+5`
 *  = `[A,B,C,A,C,D]`):
 *    - `station_0 = midpoint(quad_0.A, quad_0.B)`, color from `quad_0.A`
 *      (measured: A and B are bit-identical in color, every sampled quad).
 *    - `station_{k+1} = midpoint(quad_k.D, quad_k.C)`, color from
 *      `quad_k.D`.
 *  `n_quads` quads (`points.size() == n_quads*6`, ALWAYS a multiple of 6,
 *  not just 3 -- measured, 1647/1647 ADD markers) yield `n_quads+1`
 *  stations. Each corner point is transformed individually (marker pose,
 *  then the one per-message TF lookup, then flatten_z) before midpointing
 *  -- same per-point composition every other adapter uses, just applied to
 *  the two corners a station needs rather than every raw point.
 *
 *  ingest(): `type==TRIANGLE_LIST && points.size()>=6 && points.size()%6==0`
 *  (else ++dropped_malformed, previous carpet -- if any -- keeps
 *  rendering). One persistent marker (`ns='output_trajectory_carpet' id=0`),
 *  refreshed every tick via `DELETE_ALL` + one `ADD`, never fanned across
 *  ns/id -- so, unlike CollisionAdapter/GenericMarkerAdapter, this adapter
 *  needs no `Key{ns,id}` map: it tracks exactly one stored carpet,
 *  wholesale-replaced on every valid ingest (same "REPLACES, never merges"
 *  contract as PathAdapter).
 *
 *  Color: `colors[i]` packed via `PackRgba()` (reused from
 *  point_cloud.hpp's adapter -- point_cloud.cpp's `resolve_rgba()`
 *  alpha-zero-sentinel convention is what the RENDERER does with the
 *  result, not this adapter) with alpha forced to 255 ("a real color was
 *  supplied" sentinel) WHEN `colors.size() == points.size()`; otherwise
 *  every station's packed rgba is `0` (alpha byte 0 -- "no real per-station
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

#include "overlume_ros/adapter_stats.hpp"
#include "overlume_ros/adapters/point_cloud.hpp"  // PackRgba()
#include "overlume_ros/frame_transform.hpp"
#include "overlume_ros/profile.hpp"
#include "overlume_ros/scene_assembly.hpp"
#include "overlume/scene.h"

namespace overlume_node
{

class TrajectoryCarpetAdapter
{
public:
    TrajectoryCarpetAdapter(const ProfileRow& row,
                            const overlume::ros::FrameTransformer& tf);

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
    void fill(overlume::ros::SceneAssembly& out) const;

    const AdapterStats& stats() const { return stats_; }

    // Node-side staleness bookkeeping -- same shape as every other adapter.
    void mark_stale_tick() { ++stats_.dropped_stale; }

private:
    ProfileRow row_;
    const overlume::ros::FrameTransformer& tf_;

    bool has_data_{false};
    std::vector<overlume::PointCloudPoint> storage_;
    double last_update_sec_{0.0};
    AdapterStats stats_;
};

}  // namespace overlume_node
