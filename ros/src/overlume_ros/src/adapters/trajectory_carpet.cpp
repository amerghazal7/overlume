// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/trajectory_carpet.hpp"

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace overlume::ros {
namespace {

constexpr int32_t kActionAdd = 0;
constexpr int32_t kActionDeleteAll = 3;
constexpr int32_t kTypeTriangleList = 11;

bool HasNan(double x, double y, double z) {
    return std::isnan(x) || std::isnan(y) || std::isnan(z);
}

bool MarkerPoseIsIdentity(const geometry_msgs::msg::Pose& p) {
    constexpr double kEps = 1e-12;
    return std::abs(p.position.x) < kEps && std::abs(p.position.y) < kEps &&
           std::abs(p.position.z) < kEps && std::abs(p.orientation.x) < kEps &&
           std::abs(p.orientation.y) < kEps && std::abs(p.orientation.z) < kEps &&
           std::abs(p.orientation.w - 1.0) < kEps;
}

bool MarkerPoseHasNan(const geometry_msgs::msg::Pose& p) {
    return std::isnan(p.position.x) || std::isnan(p.position.y) || std::isnan(p.position.z) ||
           std::isnan(p.orientation.x) || std::isnan(p.orientation.y) ||
           std::isnan(p.orientation.z) || std::isnan(p.orientation.w);
}

}

TrajectoryCarpetAdapter::TrajectoryCarpetAdapter(const ProfileRow& row,
                                                 const overlume::ros::FrameTransformer& tf)
    : row_(row), tf_(tf) {}

void TrajectoryCarpetAdapter::ingest(const visualization_msgs::msg::MarkerArray& msg,
                                     double sim_time_sec) {
    ++stats_.msgs;
    if (msg.markers.empty()) return;

    tf2::Transform xform;
    if (!tf_.lookup(msg.markers.front().header, xform)) {
        ++stats_.dropped_no_tf;
        return;
    }

    for (const auto& m : msg.markers) {
        if (m.action == kActionDeleteAll) {
            has_data_ = false;
            storage_.clear();
            continue;
        }
        if (m.action != kActionAdd) continue;

        if (m.type != kTypeTriangleList || m.points.size() < 6 || m.points.size() % 6 != 0) {
            ++stats_.dropped_malformed;
            continue;
        }

        const bool identity_pose = MarkerPoseIsIdentity(m.pose);
        tf2::Transform marker_tf;
        if (!identity_pose) {
            if (MarkerPoseHasNan(m.pose)) {
                ++stats_.dropped_malformed;
                continue;
            }
            tf2::Quaternion q(m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
                              m.pose.orientation.w);
            if (q.length2() < 1e-12) q = tf2::Quaternion::getIdentity();
            marker_tf = tf2::Transform(
                q, tf2::Vector3(m.pose.position.x, m.pose.position.y, m.pose.position.z));
        }

        const bool per_point_colors = m.colors.size() == m.points.size();

        auto transform_point = [&](size_t idx, tf2::Vector3* out) -> bool {
            const auto& p = m.points[idx];
            const tf2::Vector3 local(p.x, p.y, p.z);
            const tf2::Vector3 posed = identity_pose ? local : marker_tf * local;
            const tf2::Vector3 tp = xform * posed;
            const double z = tf_.flatten_z() ? 0.0 : tp.z();
            if (HasNan(tp.x(), tp.y(), z)) return false;
            *out = tf2::Vector3(tp.x(), tp.y(), z);
            return true;
        };

        auto make_station = [&](size_t idx_a, size_t idx_b, size_t color_idx,
                                overlume::PointCloudPoint* out) -> bool {
            tf2::Vector3 a, b;
            if (!transform_point(idx_a, &a) || !transform_point(idx_b, &b)) return false;
            out->position = {(a.x() + b.x()) / 2.0, (a.y() + b.y()) / 2.0, (a.z() + b.z()) / 2.0};
            out->rgba =
                per_point_colors
                    ? PackRgba(static_cast<uint8_t>(m.colors[color_idx].r * 255.0f + 0.5f),
                               static_cast<uint8_t>(m.colors[color_idx].g * 255.0f + 0.5f),
                               static_cast<uint8_t>(m.colors[color_idx].b * 255.0f + 0.5f), 255)
                    : 0u;
            return true;
        };

        const size_t n_quads = m.points.size() / 6;
        std::vector<overlume::PointCloudPoint> stations;
        stations.reserve(n_quads + 1);
        bool ok = true;

        overlume::PointCloudPoint s0{};
        if (!make_station(0, 1, 0, &s0)) {
            ok = false;
        } else {
            stations.push_back(s0);
        }

        for (size_t k = 0; ok && k < n_quads; ++k) {
            const size_t base = 6 * k;
            overlume::PointCloudPoint sk1{};
            if (!make_station(base + 5, base + 4, base + 5, &sk1)) {
                ok = false;
                break;
            }
            stations.push_back(sk1);
        }
        if (!ok) {
            ++stats_.dropped_malformed;
            continue;
        }

        storage_ = std::move(stations);
        has_data_ = true;
        last_update_sec_ = sim_time_sec;
    }

    stats_.last_msg_sec = sim_time_sec;
}

void TrajectoryCarpetAdapter::fill(overlume::ros::SceneAssembly& out) const {
    if (!has_data_) return;
    overlume::TrajectoryCarpet tc{};
    tc.points = storage_.data();
    tc.point_count = static_cast<uint32_t>(storage_.size());
    tc.last_update_sec = last_update_sec_;
    out.trajectory_carpets.push_back(tc);
}

}
