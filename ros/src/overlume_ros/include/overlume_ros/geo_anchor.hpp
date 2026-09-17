#pragma once
/** @file geo_anchor.hpp
 *  @brief WGS84 <-> map-frame geo-anchor (VM-050, Epic 4 Task 1).
 *
 *  Spec §4.5/§9: the robot publishes `sensor_msgs/msg/NavSatFix` in WGS84
 *  (`/sim/feedback/gps`) -- the environment layer's PRIMARY datum source,
 *  paired with the `gps_link`/`base_link` TF (Epic 4 plan Decision 7:
 *  `base_link -> gps_link` is an identity transform in both recordings, so
 *  this samples `(map-frame base_link pose, WGS84 fix)` directly, no lever
 *  arm correction). `geo_datum_*` node params are a manual override only,
 *  for GPS-denied replays. No anchor from either source -> environment
 *  layer disabled with one WARN (Task 3 checks `solved()`); everything
 *  else in the node is unaffected.
 *
 *  Pure math below (SolveAnchor/WgsToMap/MapToWgs/GreatCircleDistanceM) is
 *  ROS-free and GPU-free -- testable standalone, and reused verbatim by
 *  Task 3's verification overlay reasoning and bake_environment.py's own
 *  (independently re-implemented, Python-side) copy of the same formulas.
 *
 *  GeoAnchorSolver is the ROS-facing accumulator: it reuses the node's ONE
 *  tf2_ros::Buffer (the same one TfAdapter reads for ego pose -- no second
 *  TransformListener spun up), the same non-fatal "no TF yet" philosophy as
 *  tf_adapter.cpp:20-27 (tf2::TransformException caught, sample dropped,
 *  never a crash).
 */

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/buffer.h>

#include "overlume/scene.h"

namespace overlume::ros
{

using overlume::GeoAnchor;  // re-export, not a new type -- unqualified GeoAnchor
                          // below and in test_geo_anchor.cpp is this same
                          // type Task 3 (VM-052) appends set_environment_source
                          // to consume.

// Pure math, no ROS types -- testable without a node/executor. WGS84 <->
// local-ENU via a standard equirectangular approximation around the
// anchor's own origin: sufficient at the ~2 km scale the AC names (this is
// a clay-context layer, not survey-grade). `heading_rad` rotates local-ENU
// so the map frame's own +X aligns with it (GeoAnchor's own field comment:
// "bearing of map-frame +X from true north" -- at heading_rad == 0, map +X
// points true north, map +Y points true west, the unique right-handed (Z
// up) orientation consistent with that bearing definition).
overlume::GeoAnchor SolveAnchor(const std::vector<std::pair<double, double>>& fixes,
                             const std::vector<std::pair<double, double>>& map_xy);

overlume::Vec3 WgsToMap(const overlume::GeoAnchor& anchor, double lat_deg, double lon_deg,
                     double alt_m = 0.0);
std::pair<double, double> MapToWgs(const overlume::GeoAnchor& anchor, overlume::Vec3 map_xy);

// Small-angle great-circle (haversine) distance in meters between two
// WGS84 points -- the units the round-trip test's own 0.1 m bar is stated
// in. Anchor-independent: pure lat/lon geometry, used by test_geo_anchor.cpp
// to check round-trip error, not by any anchor math itself.
double GreatCircleDistanceM(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg);

// Rate -> minimum sample count: 500 samples at the confirmed 50 Hz
// /sim/feedback/gps rate (Epic 4 plan Decision 6) = 10 s of real robot
// motion before the anchor is even considered -- short enough that the
// environment layer activates well within a normal startup window.
//
// Measured speed -> minimum displacement -> bearing error: a sample-count
// gate alone does not guarantee movement -- stack_v2_fixtures_2026-09-09's
// mean speed is ~0.5 m/s (102 m net over 205.7 s) but its first ~13 s are
// stationary to well under a millimetre, so a count-only gate can fire on a
// near-zero baseline and reduce SolveAnchor()'s heading to atan2() over
// GPS/localization noise (a 1.0 m^2 covariance placeholder, Decision 6) --
// an arbitrary bearing. kMinAnchorBaselineM requires real motion before
// solving: 20.0 m keeps the resulting bearing error from a 1 m-sigma fix to
// a few degrees, well under the recording's own displacement budget.
inline constexpr uint32_t kMinAnchorSamples = 500;
inline constexpr double kMinAnchorBaselineM = 20.0;

// All-or-nothing classification of the geo_datum_lat_deg/lon_deg/heading_deg
// override params (spec: the override is complete or absent, never partial
// -- see set_override()'s own doc comment below).
enum class GeoDatumOverride
{
    None,      // All three NaN -- sample from NavSatFix+TF instead.
    Complete,  // All three finite -- set_override() applies immediately.
    Partial,   // 1 or 2 finite -- config ERROR: ignored, sampling proceeds.
};

GeoDatumOverride ClassifyGeoDatum(double lat_deg, double lon_deg, double heading_deg);

// ROS-facing accumulator: subscribes NavSatFix (node owns the subscription,
// forwards each sample via on_fix()), looks up (map_frame, base_frame) via
// the SAME tf2_ros::Buffer the ego TfAdapter already reads, and calls
// SolveAnchor() once kMinAnchorSamples accumulate. geo_datum_* params (if
// all three are finite) short-circuit sampling entirely via set_override()
// -- the spec's "manual override for GPS-denied replays" -- and, once set,
// win for the node's lifetime: a later on_fix() is a no-op (real fixes
// never silently creep back in over a deliberate override).
class GeoAnchorSolver
{
public:
    GeoAnchorSolver(tf2_ros::Buffer& buffer, std::string map_frame, std::string base_frame);

    void set_override(double lat_deg, double lon_deg, double heading_deg);

    // Looks up map_frame->base_frame "now" (non-fatal on
    // tf2::TransformException -- sample dropped, same "no data yet"
    // philosophy as tf_adapter.cpp), pairs it with `fix`, and accumulates.
    // No-op once overridden or already solved.
    void on_fix(const sensor_msgs::msg::NavSatFix& fix);

    bool solved() const { return solved_; }
    // Undefined (returns a default-constructed GeoAnchor) if !solved().
    overlume::GeoAnchor anchor() const { return anchor_; }

private:
    tf2_ros::Buffer& buffer_;
    std::string map_frame_;
    std::string base_frame_;

    bool overridden_{false};
    bool solved_{false};
    overlume::GeoAnchor anchor_{};

    std::vector<std::pair<double, double>> fixes_;   // (lat_deg, lon_deg)
    std::vector<std::pair<double, double>> map_xy_;  // paired map-frame (x, y)
};

}  // namespace overlume::ros
