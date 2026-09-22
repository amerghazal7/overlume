// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/generic_marker.hpp"

#include <cmath>
#include <optional>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>

namespace overlume::ros {
namespace {

constexpr int32_t kActionAdd = 0;
constexpr int32_t kActionDelete = 2;
constexpr int32_t kActionDeleteAll = 3;

constexpr int32_t kTypeArrow = 0;
constexpr int32_t kTypeCube = 1;
constexpr int32_t kTypeSphere = 2;
constexpr int32_t kTypeCylinder = 3;
constexpr int32_t kTypeLineStrip = 4;
constexpr int32_t kTypeLineList = 5;
constexpr int32_t kTypeCubeList = 6;
constexpr int32_t kTypeSphereList = 7;
constexpr int32_t kTypePoints = 8;
constexpr int32_t kTypeTextViewFacing = 9;
constexpr int32_t kTypeMeshResource = 10;
constexpr int32_t kTypeTriangleList = 11;

bool HasNan(const overlume::Vec3& p) {
    return std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z);
}

bool MarkerPoseIsIdentity(const geometry_msgs::msg::Pose& p) {
    constexpr double kEps = 1e-12;
    return std::abs(p.position.x) < kEps && std::abs(p.position.y) < kEps &&
           std::abs(p.position.z) < kEps && std::abs(p.orientation.x) < kEps &&
           std::abs(p.orientation.y) < kEps && std::abs(p.orientation.z) < kEps &&
           std::abs(p.orientation.w - 1.0) < kEps;
}

bool BuildMarkerPoseTransform(const geometry_msgs::msg::Pose& p, tf2::Transform& out) {
    const tf2::Vector3 pos(p.position.x, p.position.y, p.position.z);
    tf2::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    if (std::isnan(pos.x()) || std::isnan(pos.y()) || std::isnan(pos.z()) || std::isnan(q.x()) ||
        std::isnan(q.y()) || std::isnan(q.z()) || std::isnan(q.w())) {
        return false;
    }
    if (q.length2() < 1e-12) q = tf2::Quaternion::getIdentity();
    out = tf2::Transform(q, pos);
    return true;
}

std::string ToMeshPath(const std::string& uri) {
    constexpr const char* kFilePrefix = "file://";
    if (uri.rfind(kFilePrefix, 0) == 0) return uri.substr(std::string(kFilePrefix).size());
    return uri;
}

}  // namespace

GenericMarkerAdapter::GenericMarkerAdapter(const ProfileRow& row,
                                           const overlume::ros::FrameTransformer& tf)
    : row_(row), tf_(tf) {}

void GenericMarkerAdapter::ingest(const visualization_msgs::msg::MarkerArray& msg,
                                  double sim_time_sec) {
    ++stats_.msgs;
    if (msg.markers.empty()) return;

    for (auto it = storage_.begin(); it != storage_.end();) {
        if (it->second.expires_at_sec > 0.0 && it->second.expires_at_sec <= sim_time_sec) {
            it = storage_.erase(it);
        } else {
            ++it;
        }
    }

    tf2::Transform xform;
    if (!tf_.lookup(msg.markers.front().header, xform)) {
        ++stats_.dropped_no_tf;
        return;
    }

    for (const auto& m : msg.markers) {
        if (m.action == kActionDeleteAll) {
            storage_.clear();
            continue;
        }
        if (m.action == kActionDelete) {
            storage_.erase(Key{m.ns, m.id});
            continue;
        }
        if (m.action != kActionAdd) continue;

        const NsRender verdict = classify(row_, m.ns);
        if (verdict == NsRender::kDrop) {
            ++stats_.dropped_by_rule;
            continue;
        }

        std::optional<overlume::MarkerPrimitive> single_primitive;
        bool is_fan_out = false;
        switch (m.type) {
            case kTypeArrow:
                single_primitive = overlume::MarkerPrimitive::ARROW;
                break;
            case kTypeCube:
                single_primitive = overlume::MarkerPrimitive::CUBE;
                break;
            case kTypeSphere:
                single_primitive = overlume::MarkerPrimitive::SPHERE;
                break;
            case kTypeCylinder:
                single_primitive = overlume::MarkerPrimitive::CYLINDER;
                break;
            case kTypeLineStrip:
                single_primitive = overlume::MarkerPrimitive::LINE_STRIP;
                break;
            case kTypeLineList:
                single_primitive = overlume::MarkerPrimitive::LINE_LIST;
                break;
            case kTypePoints:
                single_primitive = overlume::MarkerPrimitive::POINTS;
                break;
            case kTypeTextViewFacing:
                single_primitive = overlume::MarkerPrimitive::TEXT;
                break;
            case kTypeMeshResource:
                single_primitive = overlume::MarkerPrimitive::MESH;
                break;
            case kTypeTriangleList:
                single_primitive = overlume::MarkerPrimitive::TRIANGLE_LIST;
                break;
            case kTypeCubeList:
            case kTypeSphereList:
                is_fan_out = true;
                break;
            default:
                ++stats_.dropped_malformed;
                continue;
        }

        const bool identity_pose = MarkerPoseIsIdentity(m.pose);
        tf2::Transform marker_tf;
        if (!identity_pose && !BuildMarkerPoseTransform(m.pose, marker_tf)) {
            ++stats_.dropped_malformed;
            continue;
        }

        if (is_fan_out) {
            if (m.points.empty() || m.scale.x <= 0.0 || m.scale.y <= 0.0 || m.scale.z <= 0.0) {
                ++stats_.dropped_malformed;
                continue;
            }
            const tf2::Transform world_rot_tf = identity_pose ? xform : xform * marker_tf;
            const double heading = tf2::getYaw(world_rot_tf.getRotation());

            StoredMarker entry;
            entry.primitive = m.type == kTypeCubeList ? overlume::MarkerPrimitive::CUBE
                                                      : overlume::MarkerPrimitive::SPHERE;
            entry.heading_rad = heading;
            entry.scale = {m.scale.x, m.scale.y, m.scale.z};
            entry.color[0] = m.color.r;
            entry.color[1] = m.color.g;
            entry.color[2] = m.color.b;
            entry.color[3] = m.color.a;
            entry.fan_positions.reserve(m.points.size());
            const bool per_point_colors = m.colors.size() == m.points.size();
            if (per_point_colors) entry.fan_colors.reserve(m.points.size() * 4);

            bool ok = true;
            for (size_t i = 0; i < m.points.size(); ++i) {
                const tf2::Vector3 local(m.points[i].x, m.points[i].y, m.points[i].z);
                const tf2::Vector3 posed = identity_pose ? local : marker_tf * local;
                const tf2::Vector3 tp = xform * posed;
                const overlume::Vec3 v{tp.x(), tp.y(), tf_.flatten_z() ? 0.0 : tp.z()};
                if (HasNan(v)) {
                    ok = false;
                    break;
                }
                entry.fan_positions.push_back(v);
                if (per_point_colors) {
                    entry.fan_colors.push_back(m.colors[i].r);
                    entry.fan_colors.push_back(m.colors[i].g);
                    entry.fan_colors.push_back(m.colors[i].b);
                    entry.fan_colors.push_back(1.0f);
                }
            }
            if (!ok) {
                ++stats_.dropped_malformed;
                continue;
            }

            entry.last_update_sec = sim_time_sec;
            const double lifetime_sec = m.lifetime.sec + m.lifetime.nanosec * 1e-9;
            entry.expires_at_sec = lifetime_sec > 0.0 ? sim_time_sec + lifetime_sec : 0.0;
            storage_[Key{m.ns, m.id}] = std::move(entry);
            continue;
        }

        const overlume::MarkerPrimitive primitive = *single_primitive;
        StoredMarker entry;
        entry.primitive = primitive;
        entry.color[0] = m.color.r;
        entry.color[1] = m.color.g;
        entry.color[2] = m.color.b;
        entry.color[3] = m.color.a;

        if (primitive == overlume::MarkerPrimitive::LINE_STRIP ||
            primitive == overlume::MarkerPrimitive::LINE_LIST ||
            primitive == overlume::MarkerPrimitive::POINTS ||
            primitive == overlume::MarkerPrimitive::TRIANGLE_LIST) {
            const size_t min_points = primitive == overlume::MarkerPrimitive::TRIANGLE_LIST ? 3
                                      : primitive == overlume::MarkerPrimitive::POINTS      ? 1
                                                                                            : 2;
            if (m.points.size() < min_points) {
                ++stats_.dropped_malformed;
                continue;
            }
            if ((primitive == overlume::MarkerPrimitive::TRIANGLE_LIST &&
                 m.points.size() % 3 != 0) ||
                (primitive == overlume::MarkerPrimitive::LINE_LIST && m.points.size() % 2 != 0)) {
                ++stats_.dropped_malformed;
                continue;
            }
            std::vector<overlume::Vec3> pts;
            pts.reserve(m.points.size());
            bool ok = true;
            for (const auto& p : m.points) {
                const tf2::Vector3 local(p.x, p.y, p.z);
                const tf2::Vector3 posed = identity_pose ? local : marker_tf * local;
                const tf2::Vector3 tp = xform * posed;
                const overlume::Vec3 v{tp.x(), tp.y(), tf_.flatten_z() ? 0.0 : tp.z()};
                if (HasNan(v)) {
                    ok = false;
                    break;
                }
                pts.push_back(v);
            }
            if (!ok) {
                ++stats_.dropped_malformed;
                continue;
            }
            entry.points = std::move(pts);
        } else {
            if (primitive == overlume::MarkerPrimitive::MESH) {
                if (m.mesh_resource.empty()) {
                    ++stats_.dropped_malformed;
                    continue;
                }
                entry.mesh_path = ToMeshPath(m.mesh_resource);
            }
            if (primitive == overlume::MarkerPrimitive::TEXT) {
                entry.text = m.text;
            } else if (m.scale.x <= 0.0 || m.scale.y <= 0.0 || m.scale.z <= 0.0) {
                ++stats_.dropped_malformed;
                continue;
            }

            const tf2::Transform world_tf = identity_pose ? xform : xform * marker_tf;
            const tf2::Vector3 origin = world_tf.getOrigin();
            entry.position = {origin.x(), origin.y(), tf_.flatten_z() ? 0.0 : origin.z()};
            entry.heading_rad = tf2::getYaw(world_tf.getRotation());
            entry.scale = {m.scale.x, m.scale.y, m.scale.z};
        }

        entry.last_update_sec = sim_time_sec;
        const double lifetime_sec = m.lifetime.sec + m.lifetime.nanosec * 1e-9;
        entry.expires_at_sec = lifetime_sec > 0.0 ? sim_time_sec + lifetime_sec : 0.0;
        storage_[Key{m.ns, m.id}] = std::move(entry);
    }

    stats_.last_msg_sec = sim_time_sec;
}

void GenericMarkerAdapter::fill(overlume::ros::SceneAssembly& out) const {
    for (const auto& [key, entry] : storage_) {
        (void)key;
        if (!entry.fan_positions.empty()) {
            const bool per_point_colors = !entry.fan_colors.empty();
            for (size_t i = 0; i < entry.fan_positions.size(); ++i) {
                overlume::GenericMarker g{};
                g.primitive = entry.primitive;
                g.position = entry.fan_positions[i];
                g.heading_rad = entry.heading_rad;
                g.scale = entry.scale;
                if (per_point_colors) {
                    g.color[0] = entry.fan_colors[i * 4 + 0];
                    g.color[1] = entry.fan_colors[i * 4 + 1];
                    g.color[2] = entry.fan_colors[i * 4 + 2];
                    g.color[3] = entry.fan_colors[i * 4 + 3];
                } else {
                    g.color[0] = entry.color[0];
                    g.color[1] = entry.color[1];
                    g.color[2] = entry.color[2];
                    g.color[3] = entry.color[3];
                }
                g.last_update_sec = entry.last_update_sec;
                out.markers.push_back(g);
            }
            continue;
        }

        overlume::GenericMarker g{};
        g.primitive = entry.primitive;
        g.color[0] = entry.color[0];
        g.color[1] = entry.color[1];
        g.color[2] = entry.color[2];
        g.color[3] = entry.color[3];
        g.last_update_sec = entry.last_update_sec;

        if (!entry.points.empty()) {
            g.points = entry.points.data();
            g.point_count = static_cast<uint32_t>(entry.points.size());
        } else {
            g.position = entry.position;
            g.heading_rad = entry.heading_rad;
            g.scale = entry.scale;
            if (entry.primitive == overlume::MarkerPrimitive::TEXT) {
                g.text = entry.text.c_str();
            } else if (entry.primitive == overlume::MarkerPrimitive::MESH) {
                g.mesh_path = entry.mesh_path.c_str();
            }
        }
        out.markers.push_back(g);
    }
}

}  // namespace overlume::ros
