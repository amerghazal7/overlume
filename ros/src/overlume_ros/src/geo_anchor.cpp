// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/geo_anchor.hpp"

#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

namespace overlume::ros {

namespace {
constexpr double kDegToRad = M_PI / 180.0;
// Mean Earth radius (meters) -- a spherical approximation is the AC's own
// stated sufficiency bound ("~2 km area"), not a survey-grade ellipsoid
// model. Used consistently by WgsToMap/MapToWgs (forward/inverse of the
// SAME linear rotation, so the exact value cancels out of round-trip
// accuracy) and by GreatCircleDistanceM (where it does set the real-world
// scale of the reported distance).
constexpr double kEarthRadiusM = 6371000.0;
}  // namespace

overlume::Vec3 WgsToMap(const overlume::GeoAnchor& anchor, double lat_deg, double lon_deg,
                        double alt_m) {
    const double lat0_rad = anchor.origin_lat_deg * kDegToRad;
    const double east =
        kEarthRadiusM * std::cos(lat0_rad) * (lon_deg - anchor.origin_lon_deg) * kDegToRad;
    const double north = kEarthRadiusM * (lat_deg - anchor.origin_lat_deg) * kDegToRad;

    const double h = anchor.heading_rad;
    const double s = std::sin(h), c = std::cos(h);
    // Rotates ENU into the map frame: at heading_rad == 0, map +X == true
    // north (GeoAnchor's own field comment), map +Y == true west (the
    // unique right-handed, Z-up choice consistent with that). See
    // geo_anchor.hpp's header comment for the derivation.
    return overlume::Vec3{east * s + north * c, -east * c + north * s, alt_m};
}

std::pair<double, double> MapToWgs(const overlume::GeoAnchor& anchor, overlume::Vec3 map_xy) {
    const double h = anchor.heading_rad;
    const double s = std::sin(h), c = std::cos(h);
    // Inverse of WgsToMap's rotation -- the forward matrix is orthonormal
    // (rows [s, c] / [-c, s], each unit-length and mutually perpendicular),
    // so its inverse is its transpose.
    const double east = map_xy.x * s - map_xy.y * c;
    const double north = map_xy.x * c + map_xy.y * s;

    const double lat0_rad = anchor.origin_lat_deg * kDegToRad;
    const double lat = anchor.origin_lat_deg + north / (kEarthRadiusM * kDegToRad);
    const double lon =
        anchor.origin_lon_deg + east / (kEarthRadiusM * kDegToRad * std::cos(lat0_rad));
    return {lat, lon};
}

double GreatCircleDistanceM(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg) {
    const double phi1 = lat1_deg * kDegToRad;
    const double phi2 = lat2_deg * kDegToRad;
    const double dphi = (lat2_deg - lat1_deg) * kDegToRad;
    const double dlambda = (lon2_deg - lon1_deg) * kDegToRad;
    const double a = std::sin(dphi / 2.0) * std::sin(dphi / 2.0) + std::cos(phi1) * std::cos(phi2) *
                                                                       std::sin(dlambda / 2.0) *
                                                                       std::sin(dlambda / 2.0);
    const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return kEarthRadiusM * c;
}

overlume::GeoAnchor SolveAnchor(const std::vector<std::pair<double, double>>& fixes,
                                const std::vector<std::pair<double, double>>& map_xy) {
    overlume::GeoAnchor out{};
    if (fixes.empty() || fixes.size() != map_xy.size()) return out;

    // ponytail: heading from the first/last sample pair alone (a
    // single-segment bearing estimate), not an average over every
    // consecutive pair -- the simplest thing that works for a roughly
    // straight baseline drive, per kMinAnchorSamples' own "~10s+ baseline
    // of movement for a stable bearing" reasoning. Upgrade to a
    // circular-mean-of-incremental-bearings estimate if the real operating
    // area's initial approach turns out not to be roughly straight.
    if (fixes.size() >= 2) {
        const auto& [lat1, lon1] = fixes.front();
        const auto& [lat2, lon2] = fixes.back();
        const auto& [mx1, my1] = map_xy.front();
        const auto& [mx2, my2] = map_xy.back();

        const double phi1 = lat1 * kDegToRad, phi2 = lat2 * kDegToRad;
        const double dlon = (lon2 - lon1) * kDegToRad;
        // Standard initial-bearing formula: clockwise from true north.
        const double bearing = std::atan2(
            std::sin(dlon) * std::cos(phi2),
            std::cos(phi1) * std::sin(phi2) - std::sin(phi1) * std::cos(phi2) * std::cos(dlon));
        // Angle of the same displacement in the map frame, CCW from map +X
        // (standard math convention).
        const double angle_map = std::atan2(my2 - my1, mx2 - mx1);

        double heading = bearing + angle_map;
        while (heading > M_PI) heading -= 2.0 * M_PI;
        while (heading <= -M_PI) heading += 2.0 * M_PI;
        out.heading_rad = heading;
    }

    // Origin: the WGS84 position of map (0,0) -- what WgsToMap/MapToWgs
    // actually mean by "anchor origin" -- NOT the mean fix (the ego's own
    // mean position, which sits wherever the ego's map-frame track
    // averaged to, not at the map origin; those differ by the ego's mean
    // map offset). Per sample: rotate its map (x, y) back to an ENU offset
    // using the heading solved above (this is WgsToMap's own east/north,
    // inverted the same way MapToWgs already does it), then subtract that
    // offset from the sample's OWN fix to get that sample's estimate of the
    // origin; average the per-sample estimates.
    const double s = std::sin(out.heading_rad), c = std::cos(out.heading_rad);
    double sum_lat0 = 0.0, sum_lon0 = 0.0;
    for (std::size_t i = 0; i < fixes.size(); ++i) {
        const auto& [lat, lon] = fixes[i];
        const auto& [x, y] = map_xy[i];
        const double east = x * s - y * c;
        const double north = x * c + y * s;
        sum_lat0 += lat - north / (kEarthRadiusM * kDegToRad);
        sum_lon0 += lon - east / (kEarthRadiusM * kDegToRad * std::cos(lat * kDegToRad));
    }
    out.origin_lat_deg = sum_lat0 / static_cast<double>(fixes.size());
    out.origin_lon_deg = sum_lon0 / static_cast<double>(fixes.size());

    return out;
}

GeoDatumOverride ClassifyGeoDatum(double lat_deg, double lon_deg, double heading_deg) {
    const int finite_count = (std::isfinite(lat_deg) ? 1 : 0) + (std::isfinite(lon_deg) ? 1 : 0) +
                             (std::isfinite(heading_deg) ? 1 : 0);
    if (finite_count == 3) return GeoDatumOverride::Complete;
    if (finite_count == 0) return GeoDatumOverride::None;
    return GeoDatumOverride::Partial;
}

GeoAnchorSolver::GeoAnchorSolver(tf2_ros::Buffer& buffer, std::string map_frame,
                                 std::string base_frame)
    : buffer_(buffer), map_frame_(std::move(map_frame)), base_frame_(std::move(base_frame)) {}

void GeoAnchorSolver::set_override(double lat_deg, double lon_deg, double heading_deg) {
    anchor_ = overlume::GeoAnchor{lat_deg, lon_deg, heading_deg * kDegToRad};
    overridden_ = true;
    solved_ = true;
}

void GeoAnchorSolver::on_fix(const sensor_msgs::msg::NavSatFix& fix) {
    // Override wins for the node's lifetime (spec's "manual override for
    // GPS-denied replays") -- real fixes never creep back in once set.
    // Already-solved-by-sampling is likewise final: no re-solving mid-run.
    if (overridden_ || solved_) return;

    geometry_msgs::msg::TransformStamped t;
    try {
        t = buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException&) {
        // No TF yet -- non-fatal, same "no data" philosophy as
        // tf_adapter.cpp:20-27. Drop this fix, keep accumulating on the
        // next callback.
        return;
    }

    fixes_.emplace_back(fix.latitude, fix.longitude);
    map_xy_.emplace_back(t.transform.translation.x, t.transform.translation.y);

    if (fixes_.size() >= kMinAnchorSamples) {
        // Sample count alone is necessary but not sufficient (see
        // kMinAnchorBaselineM's own comment in geo_anchor.hpp) -- also
        // require the accumulated track to have actually moved.
        const auto& [x0, y0] = map_xy_.front();
        const auto& [x1, y1] = map_xy_.back();
        if (std::hypot(x1 - x0, y1 - y0) >= kMinAnchorBaselineM) {
            anchor_ = SolveAnchor(fixes_, map_xy_);
            solved_ = true;
        }
    }
}

}  // namespace overlume::ros
