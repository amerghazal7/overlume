// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/dynamic_objects.hpp"

#include <cmath>
#include <string_view>
#include <unordered_set>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>
#include <yaml-cpp/yaml.h>

namespace overlume::ros {
namespace {

constexpr int32_t kActionAdd = 0;
constexpr int32_t kActionModify = 1;
constexpr int32_t kActionDelete = 2;
constexpr int32_t kActionDeleteAll = 3;

constexpr double kMinArrowM = 1e-3;
constexpr double kChainToleranceM = 1e-6;

bool HasNan(double v) { return std::isnan(v); }

bool HasNanVec(const tf2::Vector3& v) { return HasNan(v.x()) || HasNan(v.y()) || HasNan(v.z()); }

bool HasNanQuat(const tf2::Quaternion& q) {
    return HasNan(q.x()) || HasNan(q.y()) || HasNan(q.z()) || HasNan(q.w());
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
    if (HasNanVec(pos) || HasNanQuat(q)) return false;
    if (q.length2() < 1e-12) q = tf2::Quaternion::getIdentity();
    out = tf2::Transform(q, pos);
    return true;
}

overlume::Vec3 ToVec3(const geometry_msgs::msg::Point& p) { return {p.x, p.y, p.z}; }

overlume::Vec3 ToVec3(const tf2::Vector3& v) { return {v.x(), v.y(), v.z()}; }

std::optional<overlume::ObjectClass> ParseObjectClass(const std::string& s) {
    if (s == "CAR") return overlume::ObjectClass::CAR;
    if (s == "TRUCK_VAN") return overlume::ObjectClass::TRUCK_VAN;
    if (s == "BUS") return overlume::ObjectClass::BUS;
    if (s == "PEDESTRIAN") return overlume::ObjectClass::PEDESTRIAN;
    if (s == "CYCLIST") return overlume::ObjectClass::CYCLIST;
    if (s == "UNKNOWN") return overlume::ObjectClass::UNKNOWN;
    return std::nullopt;
}

}

std::optional<ClassInferenceTable> load_class_inference(const std::string& path,
                                                        std::vector<std::string>& errors) {
    errors.clear();
    ClassInferenceTable table;
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        errors.push_back(path + ": yaml parse error: " + e.what());
        return std::nullopt;
    } catch (const std::exception& e) {
        errors.push_back(path + ": " + e.what());
        return std::nullopt;
    }

    if (const auto prefix_node = root["prefix"]) {
        for (auto it = prefix_node.begin(); it != prefix_node.end(); ++it) {
            const std::string key = it->first.as<std::string>();
            const std::string val = it->second.as<std::string>();
            if (const auto cls = ParseObjectClass(val)) {
                table.prefix[key] = *cls;
            } else {
                errors.push_back(path + ": prefix '" + key + "': unknown class '" + val + "'");
            }
        }
    }

    if (const auto footprint_node = root["footprint"]) {
        for (const auto& item : footprint_node) {
            FootprintBand band;
            band.max_length_m = item["max_length_m"] ? item["max_length_m"].as<double>() : 1e300;
            band.max_width_m = item["max_width_m"] ? item["max_width_m"].as<double>() : 1e300;
            band.min_height_m = item["min_height_m"] ? item["min_height_m"].as<double>() : 0.0;
            const std::string cls_str = item["class"] ? item["class"].as<std::string>() : "";
            const auto cls = ParseObjectClass(cls_str);
            if (!cls) {
                errors.push_back(path + ": footprint[] entry: unknown/missing class '" + cls_str +
                                 "'");
                continue;
            }
            band.cls = *cls;
            table.footprint.push_back(band);
        }
    }

    const std::string default_str = root["default"] ? root["default"].as<std::string>() : "UNKNOWN";
    if (const auto def = ParseObjectClass(default_str)) {
        table.default_cls = *def;
    } else {
        errors.push_back(path + ": default '" + default_str + "': unknown class");
        table.default_cls = overlume::ObjectClass::UNKNOWN;
    }

    return table;
}

overlume::ObjectClass infer(const ClassInferenceTable& cfg, const char* label,
                            overlume::Vec3 dims) {
    if (label != nullptr) {
        const std::string_view sv(label);
        const auto underscore = sv.find('_');
        const std::string prefix(sv.substr(0, underscore));
        if (const auto it = cfg.prefix.find(prefix); it != cfg.prefix.end()) {
            return it->second;
        }
    }

    if (dims.x <= 0.0 || dims.y <= 0.0) return cfg.default_cls;

    for (const auto& band : cfg.footprint) {
        if (dims.x <= band.max_length_m && dims.y <= band.max_width_m &&
            dims.z >= band.min_height_m) {
            return band.cls;
        }
    }
    return cfg.default_cls;
}

bool line_list_to_polyline(const std::vector<geometry_msgs::msg::Point>& in,
                           std::vector<overlume::Vec3>& out) {
    if (in.size() < 2 || in.size() % 2 != 0) return false;

    const size_t n_segments = in.size() / 2;
    std::vector<overlume::Vec3> result;
    result.reserve(n_segments + 1);
    result.push_back(ToVec3(in[0]));
    if (HasNan(result.back().x) || HasNan(result.back().y) || HasNan(result.back().z)) return false;

    for (size_t i = 0; i < n_segments; ++i) {
        const auto& seg_end = in[2 * i + 1];
        const overlume::Vec3 v = ToVec3(seg_end);
        if (HasNan(v.x) || HasNan(v.y) || HasNan(v.z)) return false;

        if (i + 1 < n_segments) {
            const auto& next_start = in[2 * i + 2];
            const double dx = seg_end.x - next_start.x;
            const double dy = seg_end.y - next_start.y;
            const double dz = seg_end.z - next_start.z;
            if (std::sqrt(dx * dx + dy * dy + dz * dz) > kChainToleranceM) return false;
        }
        result.push_back(v);
    }

    out = std::move(result);
    return true;
}

DynamicObjectsAdapter::DynamicObjectsAdapter(const ProfileRow& row,
                                             const overlume::ros::FrameTransformer& tf,
                                             const ClassInferenceTable& classes)
    : row_(row), tf_(tf), classes_(classes) {}

void DynamicObjectsAdapter::ingest(const visualization_msgs::msg::MarkerArray& msg,
                                   double sim_time_sec) {
    ++stats_.msgs;
    if (msg.markers.empty()) return;

    tf2::Transform xform;
    if (!tf_.lookup(msg.markers.front().header, xform)) {
        ++stats_.dropped_no_tf;
        return;
    }

    std::unordered_set<int32_t> touched;

    for (const auto& m : msg.markers) {
        if (m.action == kActionDeleteAll) {
            tracks_.clear();
            continue;
        }
        if (m.action == kActionDelete) {
            tracks_.erase(m.id);
            continue;
        }
        if (m.action != kActionAdd && m.action != kActionModify) continue;

        const NsRender verdict = classify(row_, m.ns);
        if (verdict == NsRender::kDrop) {
            ++stats_.dropped_by_rule;
            continue;
        }

        Track& t = tracks_[m.id];

        if (m.ns == "dynamic_objects_bbox") {
            touched.insert(m.id);
            if (m.scale.x <= 0.0 || m.scale.y <= 0.0 || m.scale.z <= 0.0) {
                ++stats_.dropped_malformed;
                continue;
            }
            const tf2::Vector3 pos(m.pose.position.x, m.pose.position.y, m.pose.position.z);
            tf2::Quaternion q(m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
                              m.pose.orientation.w);
            if (HasNanVec(pos) || HasNanQuat(q)) {
                ++stats_.dropped_malformed;
                continue;
            }
            if (q.length2() < 1e-12) q = tf2::Quaternion::getIdentity();
            const tf2::Transform marker_tf(q, pos);
            const tf2::Transform world_tf = xform * marker_tf;
            t.position = ToVec3(world_tf.getOrigin());
            if (tf_.flatten_z()) t.position.z = 0.0;
            t.heading_rad = tf2::getYaw(world_tf.getRotation());
            t.dimensions = {m.scale.x, m.scale.y, m.scale.z};
            t.last_update_sec = sim_time_sec;
            t.has_bbox = true;
        } else if (m.ns == "dynamic_objects_text") {
            touched.insert(m.id);
            if (!m.text.empty()) {
                t.label = m.text;
                t.has_text = true;
            }
        } else if (m.ns == "dynamic_objects_arrow") {
            if (m.points.size() < 2) {
                continue;
            }
            const bool identity_pose = MarkerPoseIsIdentity(m.pose);
            tf2::Transform marker_tf;
            if (!identity_pose && !BuildMarkerPoseTransform(m.pose, marker_tf)) {
                ++stats_.dropped_malformed;
                continue;
            }
            const tf2::Vector3 lp0(m.points[0].x, m.points[0].y, m.points[0].z);
            const tf2::Vector3 lp1(m.points[1].x, m.points[1].y, m.points[1].z);
            const tf2::Vector3 hp0 = identity_pose ? lp0 : marker_tf * lp0;
            const tf2::Vector3 hp1 = identity_pose ? lp1 : marker_tf * lp1;
            const tf2::Vector3 p0 = xform * hp0;
            const tf2::Vector3 p1 = xform * hp1;
            const tf2::Vector3 diff = p1 - p0;
            t.velocity =
                (diff.length() < kMinArrowM) ? overlume::Vec3{0.0, 0.0, 0.0} : ToVec3(diff);
            if (tf_.flatten_z()) t.velocity.z = 0.0;
        } else if (m.ns == "dynamic_objects_hd_map_path") {
            const bool identity_pose = MarkerPoseIsIdentity(m.pose);
            tf2::Transform marker_tf;
            if (!identity_pose && !BuildMarkerPoseTransform(m.pose, marker_tf)) {
                ++stats_.dropped_malformed;
                continue;
            }
            std::vector<geometry_msgs::msg::Point> posed_points;
            posed_points.reserve(m.points.size());
            for (const auto& p : m.points) {
                if (identity_pose) {
                    posed_points.push_back(p);
                    continue;
                }
                const tf2::Vector3 v = marker_tf * tf2::Vector3(p.x, p.y, p.z);
                geometry_msgs::msg::Point pp;
                pp.x = v.x();
                pp.y = v.y();
                pp.z = v.z();
                posed_points.push_back(pp);
            }

            std::vector<overlume::Vec3> polyline;
            if (!line_list_to_polyline(posed_points, polyline)) {
                ++stats_.dropped_malformed;
                continue;
            }
            t.predicted_path.clear();
            t.predicted_path.reserve(polyline.size());
            for (const auto& v : polyline) {
                const tf2::Vector3 tv = xform * tf2::Vector3(v.x, v.y, v.z);
                overlume::Vec3 pv = ToVec3(tv);
                if (tf_.flatten_z()) pv.z = 0.0;
                t.predicted_path.push_back(pv);
            }
        }
    }

    for (const int32_t id : touched) {
        const auto it = tracks_.find(id);
        if (it != tracks_.end() && it->second.has_bbox && !it->second.has_text) {
            ++stats_.dropped_malformed;
            tracks_.erase(it);
        }
    }

    stats_.last_msg_sec = sim_time_sec;
}

void DynamicObjectsAdapter::fill(overlume::ros::SceneAssembly& out) const {
    for (const auto& [id, t] : tracks_) {
        if (!t.has_bbox || !t.has_text) continue;

        overlume::TrackedObject o{};
        o.id = static_cast<uint32_t>(id);
        o.cls = infer(classes_, t.label.c_str(), t.dimensions);
        o.position = t.position;
        o.heading_rad = t.heading_rad;
        o.dimensions = t.dimensions;
        o.velocity = t.velocity;
        o.predicted_path = t.predicted_path.empty() ? nullptr : t.predicted_path.data();
        o.predicted_path_count = static_cast<uint32_t>(t.predicted_path.size());
        o.label = t.label.c_str();
        o.last_update_sec = t.last_update_sec;
        out.objects.push_back(o);
    }
}

}
