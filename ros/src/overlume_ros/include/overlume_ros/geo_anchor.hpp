// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/buffer.h>

#include "overlume/scene.h"

namespace overlume::ros {

using overlume::GeoAnchor;

overlume::GeoAnchor SolveAnchor(const std::vector<std::pair<double, double>>& fixes,
                                const std::vector<std::pair<double, double>>& map_xy,
                                const std::vector<double>& altitudes_m = {});

overlume::Vec3 WgsToMap(const overlume::GeoAnchor& anchor, double lat_deg, double lon_deg,
                        double alt_m = 0.0);
std::pair<double, double> MapToWgs(const overlume::GeoAnchor& anchor, overlume::Vec3 map_xy);

double GreatCircleDistanceM(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg);

inline constexpr uint32_t kMinAnchorSamples = 500;
inline constexpr double kMinAnchorBaselineM = 20.0;

enum class GeoDatumOverride {
    None,
    Complete,
    Partial,
};

GeoDatumOverride ClassifyGeoDatum(double lat_deg, double lon_deg, double heading_deg);

double ChooseAnchorHeightM(double datum_height_m, double sampled_height_m, double offset_m);

class GeoAnchorSolver {
public:
    GeoAnchorSolver(tf2_ros::Buffer& buffer, std::string map_frame, std::string base_frame);

    void set_override(double lat_deg, double lon_deg, double heading_deg);

    void on_fix(const sensor_msgs::msg::NavSatFix& fix);

    bool solved() const { return solved_; }
    overlume::GeoAnchor anchor() const { return anchor_; }

    void set_origin_height_m(double origin_height_m) { anchor_.origin_height_m = origin_height_m; }

private:
    tf2_ros::Buffer& buffer_;
    std::string map_frame_;
    std::string base_frame_;

    bool overridden_{false};
    bool solved_{false};
    overlume::GeoAnchor anchor_{};

    std::vector<std::pair<double, double>> fixes_;
    std::vector<std::pair<double, double>> map_xy_;
    std::vector<double> altitudes_;
};

}  // namespace overlume::ros
