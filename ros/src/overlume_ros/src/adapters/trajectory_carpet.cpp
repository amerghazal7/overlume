// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/trajectory_carpet.hpp"

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace overlume::ros {
namespace {

// visualization_msgs/msg/Marker.msg action + type constants -- not worth a
// dependency on the generated enum names for values used once each (same
// convention as collision.cpp/generic_marker.cpp).
constexpr int32_t kActionAdd = 0;
constexpr int32_t kActionDeleteAll = 3;
constexpr int32_t kTypeTriangleList = 11;

bool HasNan(double x, double y, double z) {
    return std::isnan(x) || std::isnan(y) || std::isnan(z);
}

// A marker's own pose is RELATIVE to the header frame -- same pair as every
// other adapter's own copy (hd_map.cpp/dynamic_objects.cpp/collision.cpp/
// generic_marker.cpp; each file keeps its own rather than a shared header).
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

}  // namespace

TrajectoryCarpetAdapter::TrajectoryCarpetAdapter(const ProfileRow& row,
                                                 const overlume::ros::FrameTransformer& tf)
    : row_(row), tf_(tf) {}

void TrajectoryCarpetAdapter::ingest(const visualization_msgs::msg::MarkerArray& msg,
                                     double sim_time_sec) {
    ++stats_.msgs;
    if (msg.markers.empty()) return;

    // ONE lookup for the whole message -- same convention as every other
    // marker adapter in this node.
    tf2::Transform xform;
    if (!tf_.lookup(msg.markers.front().header, xform)) {
        ++stats_.dropped_no_tf;
        return;  // whole message dropped; previously-stored carpet stays
    }

    for (const auto& m : msg.markers) {
        if (m.action == kActionDeleteAll) {
            has_data_ = false;
            storage_.clear();
            continue;
        }
        if (m.action != kActionAdd) continue;  // ignore DELETE(ns,id) -- one persistent marker

        // Always a multiple of 6 (measured, 1647/1647 ADD markers) -- two
        // triangles per dual-rail quad, not just "a multiple of 3" (this
        // file's own header comment has the full pairing algorithm).
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
            if (q.length2() < 1e-12) q = tf2::Quaternion::getIdentity();  // rviz parity
            marker_tf = tf2::Transform(
                q, tf2::Vector3(m.pose.position.x, m.pose.position.y, m.pose.position.z));
        }

        // A length mismatch means the WHOLE colors[] array is suspect (not
        // a per-station concern) -- every station falls back to the
        // alpha==0 sentinel together, same "whole-message fallback" rule
        // this file's header comment states.
        const bool per_point_colors = m.colors.size() == m.points.size();

        // Transforms ONE raw wire point (marker pose, then the message's
        // one TF lookup, then flatten_z) -- same per-point composition
        // every other adapter uses. Returns false (NaN) rather than
        // throwing; the whole message is dropped on any NaN, same "no
        // partial carpet" rule the old flat path used.
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

        // One centerline station from two raw corner indices (position:
        // their midpoint) + one color index (colors[] alpha documented
        // "not yet used"; force 255 so a supplied color always reads as
        // "real", same convention GenericMarkerAdapter's fan_colors uses).
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

        // station_0 = midpoint(quad_0.A, quad_0.B) = midpoint(points[0],
        // points[1]), color from A (points[0]/colors[0]).
        overlume::PointCloudPoint s0{};
        if (!make_station(0, 1, 0, &s0)) {
            ok = false;
        } else {
            stations.push_back(s0);
        }

        for (size_t k = 0; ok && k < n_quads; ++k) {
            const size_t base = 6 * k;
            // station_{k+1} = midpoint(quad_k.D, quad_k.C) = midpoint(
            // points[base+5], points[base+4]), color from D (points[base+5]).
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

        // REPLACES wholesale, never appends/merges -- same contract as
        // PathAdapter (this file's own header comment).
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

}  // namespace overlume::ros
