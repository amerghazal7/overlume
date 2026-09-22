// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/path.hpp"

#include <cmath>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace overlume::ros {
namespace {

constexpr double kPathFadeWindowSec = 1.0;

overlume::PathRole RoleFromString(const std::string& s) {
    if (s == "behavior") return overlume::PathRole::BEHAVIOR;
    if (s == "global") return overlume::PathRole::GLOBAL;
    return overlume::PathRole::LOCAL;
}

bool HasNan(double v) { return std::isnan(v); }

}

PathAdapter::PathAdapter(const ProfileRow& row, const overlume::ros::FrameTransformer& tf)
    : row_(row), tf_(tf), role_(RoleFromString(row.role)) {}

void PathAdapter::ingest(const nav_msgs::msg::Path& msg, double sim_time_sec) {
    ++stats_.msgs;

    if (msg.poses.size() < 2) {
        ++stats_.dropped_malformed;
        return;
    }

    tf2::Transform xform;
    if (!tf_.lookup(msg.header, xform)) {
        ++stats_.dropped_no_tf;
        return;
    }

    std::vector<overlume::Vec3> next;
    next.reserve(msg.poses.size());
    for (const auto& ps : msg.poses) {
        const auto& p = ps.pose.position;
        const tf2::Vector3 v = xform * tf2::Vector3(p.x, p.y, p.z);
        if (HasNan(v.x()) || HasNan(v.y()) || HasNan(v.z())) {
            ++stats_.dropped_malformed;
            return;
        }
        next.push_back({v.x(), v.y(), tf_.flatten_z() ? 0.0 : v.z()});
    }

    points_ = std::move(next);
    last_update_sec_ = sim_time_sec + (row_.timeout_sec - kPathFadeWindowSec);
    stats_.last_msg_sec = sim_time_sec;
}

void PathAdapter::fill(overlume::ros::SceneAssembly& out) const {
    if (points_.size() < 2) return;

    overlume::PathRibbon r{};
    r.role = role_;
    r.points = points_.data();
    r.point_count = static_cast<uint32_t>(points_.size());
    r.last_update_sec = last_update_sec_;
    out.paths.push_back(r);
}

}
