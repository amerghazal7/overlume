// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "overlume_ros/adapter_stats.hpp"
#include "overlume_ros/frame_transform.hpp"
#include "overlume_ros/profile.hpp"
#include "overlume_ros/scene_assembly.hpp"
#include "overlume/scene.h"

namespace overlume::ros {

struct FootprintBand {
    double max_length_m{1e300};
    double max_width_m{1e300};
    double min_height_m{0.0};
    overlume::ObjectClass cls{overlume::ObjectClass::UNKNOWN};
};

struct ClassInferenceTable {
    std::unordered_map<std::string, overlume::ObjectClass> prefix;
    std::vector<FootprintBand> footprint;
    overlume::ObjectClass default_cls{overlume::ObjectClass::UNKNOWN};
};

std::optional<ClassInferenceTable> load_class_inference(const std::string& path,
                                                        std::vector<std::string>& errors);

overlume::ObjectClass infer(const ClassInferenceTable& cfg, const char* label, overlume::Vec3 dims);

bool line_list_to_polyline(const std::vector<geometry_msgs::msg::Point>& in,
                           std::vector<overlume::Vec3>& out);

class DynamicObjectsAdapter {
public:
    DynamicObjectsAdapter(const ProfileRow& row, const overlume::ros::FrameTransformer& tf,
                          const ClassInferenceTable& classes);

    void ingest(const visualization_msgs::msg::MarkerArray& msg, double sim_time_sec);

    void fill(overlume::ros::SceneAssembly& out) const;

    const AdapterStats& stats() const { return stats_; }

    void mark_stale_tick() { ++stats_.dropped_stale; }

private:
    struct Track {
        bool has_bbox{false};
        bool has_text{false};
        overlume::Vec3 position{0.0, 0.0, 0.0};
        double heading_rad{0.0};
        overlume::Vec3 dimensions{0.0, 0.0, 0.0};
        overlume::Vec3 velocity{0.0, 0.0, 0.0};
        std::string label;
        std::vector<overlume::Vec3> predicted_path;
        double last_update_sec{0.0};
    };

    ProfileRow row_;
    const overlume::ros::FrameTransformer& tf_;
    const ClassInferenceTable& classes_;
    std::unordered_map<int32_t, Track> tracks_;
    AdapterStats stats_;
};

}
