// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/geo_anchor.hpp"

#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

namespace overlume::ros {

namespace {
constexpr double kDegToRad = M_PI / 180.0;
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
    return overlume::Vec3{east * s + north * c, -east * c + north * s, alt_m};
}

std::pair<double, double> MapToWgs(const overlume::GeoAnchor& anchor, overlume::Vec3 map_xy) {
    const double h = anchor.heading_rad;
    const double s = std::sin(h), c = std::cos(h);
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
                                const std::vector<std::pair<double, double>>& map_xy,
                                const std::vector<double>& altitudes_m) {
    overlume::GeoAnchor out{};
    if (fixes.empty() || fixes.size() != map_xy.size()) return out;

    if (fixes.size() >= 2) {
        const auto& [lat1, lon1] = fixes.front();
        const auto& [lat2, lon2] = fixes.back();
        const auto& [mx1, my1] = map_xy.front();
        const auto& [mx2, my2] = map_xy.back();

        const double phi1 = lat1 * kDegToRad, phi2 = lat2 * kDegToRad;
        const double dlon = (lon2 - lon1) * kDegToRad;
        const double bearing = std::atan2(
            std::sin(dlon) * std::cos(phi2),
            std::cos(phi1) * std::sin(phi2) - std::sin(phi1) * std::cos(phi2) * std::cos(dlon));
        const double angle_map = std::atan2(my2 - my1, mx2 - mx1);

        double heading = bearing + angle_map;
        while (heading > M_PI) heading -= 2.0 * M_PI;
        while (heading <= -M_PI) heading += 2.0 * M_PI;
        out.heading_rad = heading;
    }

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

    if (!altitudes_m.empty()) {
        double sum_alt = 0.0;
        for (const double alt : altitudes_m) sum_alt += alt;
        out.origin_height_m = sum_alt / static_cast<double>(altitudes_m.size());
    }

    return out;
}

double ChooseAnchorHeightM(double datum_height_m, double sampled_height_m, double offset_m) {
    return (std::isfinite(datum_height_m) ? datum_height_m : sampled_height_m) + offset_m;
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
    if (overridden_ || solved_) return;

    geometry_msgs::msg::TransformStamped t;
    try {
        t = buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException&) {
        return;
    }

    fixes_.emplace_back(fix.latitude, fix.longitude);
    map_xy_.emplace_back(t.transform.translation.x, t.transform.translation.y);
    if (std::isfinite(fix.altitude)) altitudes_.push_back(fix.altitude);

    if (fixes_.size() >= kMinAnchorSamples) {
        const auto& [x0, y0] = map_xy_.front();
        const auto& [x1, y1] = map_xy_.back();
        if (std::hypot(x1 - x0, y1 - y0) >= kMinAnchorBaselineM) {
            anchor_ = SolveAnchor(fixes_, map_xy_, altitudes_);
            solved_ = true;
        }
    }
}

}  // namespace overlume::ros
