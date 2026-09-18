// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "fixture_msgs.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace overlume::ros::testing {
namespace {

std::string FixturePath(const std::string& fixture_name) {
#ifndef OVERLUME_NODE_FIXTURES_DIR
#error "OVERLUME_NODE_FIXTURES_DIR not defined -- see CMakeLists.txt's overlume_node_test_paths"
#endif
    return std::string(OVERLUME_NODE_FIXTURES_DIR) + "/" + fixture_name;
}

YAML::Node LoadFixtureYaml(const std::string& fixture_name) {
    const std::string path = FixturePath(fixture_name);
    std::ifstream in(path);
    if (!in) throw std::runtime_error("fixture_msgs: could not open fixture '" + path + "'");
    try {
        return YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("fixture_msgs: '" + path + "': " + e.what());
    }
}

double GetD(const YAML::Node& n, const char* key, double def = 0.0) {
    return n[key] ? n[key].as<double>() : def;
}

int64_t GetI(const YAML::Node& n, const char* key, int64_t def = 0) {
    return n[key] ? n[key].as<int64_t>() : def;
}

std::string GetS(const YAML::Node& n, const char* key, const std::string& def = "") {
    return n[key] ? n[key].as<std::string>() : def;
}

bool GetB(const YAML::Node& n, const char* key, bool def = false) {
    return n[key] ? n[key].as<bool>() : def;
}

void FillHeader(const YAML::Node& n, std_msgs::msg::Header& out) {
    if (!n) return;
    out.frame_id = GetS(n, "frame_id");
    if (const auto stamp = n["stamp"]) {
        out.stamp.sec = static_cast<int32_t>(GetI(stamp, "sec"));
        out.stamp.nanosec = static_cast<uint32_t>(GetI(stamp, "nanosec"));
    }
}

void FillPoint(const YAML::Node& n, geometry_msgs::msg::Point& out) {
    if (!n) return;
    out.x = GetD(n, "x");
    out.y = GetD(n, "y");
    out.z = GetD(n, "z");
}

void FillQuaternion(const YAML::Node& n, geometry_msgs::msg::Quaternion& out) {
    if (!n) return;
    out.x = GetD(n, "x");
    out.y = GetD(n, "y");
    out.z = GetD(n, "z");
    out.w = GetD(n, "w", 1.0);
}

void FillPose(const YAML::Node& n, geometry_msgs::msg::Pose& out) {
    if (!n) return;
    FillPoint(n["position"], out.position);
    FillQuaternion(n["orientation"], out.orientation);
}

void FillVector3(const YAML::Node& n, geometry_msgs::msg::Vector3& out) {
    if (!n) return;
    out.x = GetD(n, "x");
    out.y = GetD(n, "y");
    out.z = GetD(n, "z");
}

void FillColor(const YAML::Node& n, std_msgs::msg::ColorRGBA& out) {
    if (!n) return;
    out.r = static_cast<float>(GetD(n, "r"));
    out.g = static_cast<float>(GetD(n, "g"));
    out.b = static_cast<float>(GetD(n, "b"));
    out.a = static_cast<float>(GetD(n, "a"));
}

visualization_msgs::msg::Marker ParseMarker(const YAML::Node& n) {
    visualization_msgs::msg::Marker m;
    FillHeader(n["header"], m.header);
    m.ns = GetS(n, "ns");
    m.id = static_cast<int32_t>(GetI(n, "id"));
    m.type = static_cast<int32_t>(GetI(n, "type"));
    m.action = static_cast<int32_t>(GetI(n, "action"));
    FillPose(n["pose"], m.pose);
    FillVector3(n["scale"], m.scale);
    FillColor(n["color"], m.color);
    if (const auto lt = n["lifetime"]) {
        m.lifetime.sec = static_cast<int32_t>(GetI(lt, "sec"));
        m.lifetime.nanosec = static_cast<uint32_t>(GetI(lt, "nanosec"));
    }
    m.frame_locked = GetB(n, "frame_locked");
    if (const auto pts = n["points"]) {
        for (const auto& p : pts) {
            geometry_msgs::msg::Point pt;
            FillPoint(p, pt);
            m.points.push_back(pt);
        }
    }
    if (const auto cols = n["colors"]) {
        for (const auto& c : cols) {
            std_msgs::msg::ColorRGBA col;
            FillColor(c, col);
            m.colors.push_back(col);
        }
    }
    m.text = GetS(n, "text");
    m.mesh_resource = GetS(n, "mesh_resource");
    return m;
}

}  // namespace

visualization_msgs::msg::MarkerArray load_marker_array(const std::string& fixture_name) {
    const YAML::Node root = LoadFixtureYaml(fixture_name);
    visualization_msgs::msg::MarkerArray arr;
    if (const auto markers = root["markers"]) {
        for (const auto& m : markers) arr.markers.push_back(ParseMarker(m));
    }
    return arr;
}

nav_msgs::msg::Path load_path(const std::string& fixture_name) {
    const YAML::Node root = LoadFixtureYaml(fixture_name);
    nav_msgs::msg::Path path;
    FillHeader(root["header"], path.header);
    if (const auto poses = root["poses"]) {
        for (const auto& p : poses) {
            geometry_msgs::msg::PoseStamped ps;
            FillHeader(p["header"], ps.header);
            FillPose(p["pose"], ps.pose);
            path.poses.push_back(ps);
        }
    }
    return path;
}

nav_msgs::msg::OccupancyGrid load_occupancy_grid(const std::string& fixture_name) {
    const YAML::Node root = LoadFixtureYaml(fixture_name);
    nav_msgs::msg::OccupancyGrid grid;
    FillHeader(root["header"], grid.header);
    if (const auto info = root["info"]) {
        grid.info.resolution = static_cast<float>(GetD(info, "resolution"));
        grid.info.width = static_cast<uint32_t>(GetI(info, "width"));
        grid.info.height = static_cast<uint32_t>(GetI(info, "height"));
        FillPose(info["origin"], grid.info.origin);
    }
    if (const auto data = root["data"]) {
        for (const auto& v : data) grid.data.push_back(static_cast<int8_t>(v.as<int>()));
    }
    return grid;
}

// Same shape as load_occupancy_grid() just above -- x/y/width/height are
// plain scalars, data is int8[] (msg's own on-wire type; OgmAdapter's
// ConvertCell(), not this loader, does the -1/0..100/malformed conversion).
map_msgs::msg::OccupancyGridUpdate load_occupancy_grid_update(const std::string& fixture_name) {
    const YAML::Node root = LoadFixtureYaml(fixture_name);
    map_msgs::msg::OccupancyGridUpdate update;
    FillHeader(root["header"], update.header);
    update.x = static_cast<int32_t>(GetI(root, "x"));
    update.y = static_cast<int32_t>(GetI(root, "y"));
    update.width = static_cast<uint32_t>(GetI(root, "width"));
    update.height = static_cast<uint32_t>(GetI(root, "height"));
    if (const auto data = root["data"]) {
        for (const auto& v : data) update.data.push_back(static_cast<int8_t>(v.as<int>()));
    }
    return update;
}

namespace {

ProfileRow RowFromProfile(const char* profile_stem, const std::string& topic) {
#ifndef TEST_CONFIG_DIR
#error "TEST_CONFIG_DIR not defined -- see CMakeLists.txt's overlume_node_test_paths"
#endif
    const std::string path = std::string(TEST_CONFIG_DIR) + "/" + profile_stem + "_profile.yaml";
    std::vector<std::string> errs;
    auto profile = load_profile(path, errs);
    if (!profile) {
        std::ostringstream os;
        os << "fixture_msgs: shipped profile '" << path << "' failed to load:";
        for (const auto& e : errs) os << "\n  " << e;
        throw std::runtime_error(os.str());
    }
    const auto* row = find_row(*profile, topic);
    if (row == nullptr) {
        throw std::runtime_error("fixture_msgs: '" + path + "' has no row for topic '" + topic +
                                 "'");
    }
    return *row;
}

}  // namespace

ProfileRow urban_row(const std::string& topic) { return RowFromProfile("urban", topic); }
ProfileRow sim_row(const std::string& topic) { return RowFromProfile("sim", topic); }

ClassInferenceTable inference_table() {
    const std::string path = std::string(TEST_CONFIG_DIR) + "/class_inference.yaml";
    std::vector<std::string> errs;
    auto table = load_class_inference(path, errs);
    if (!table) {
        std::ostringstream os;
        os << "fixture_msgs: shipped class inference table '" << path << "' failed to load:";
        for (const auto& e : errs) os << "\n  " << e;
        throw std::runtime_error(os.str());
    }
    return *table;
}

}  // namespace overlume::ros::testing
