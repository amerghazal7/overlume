// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <visualization_msgs/msg/marker_array.hpp>

#include "overlume_ros/adapter_stats.hpp"
#include "overlume_ros/frame_transform.hpp"
#include "overlume_ros/profile.hpp"
#include "overlume_ros/scene_assembly.hpp"
#include "overlume/scene.h"

namespace overlume::ros {

class GenericMarkerAdapter {
public:
    GenericMarkerAdapter(const ProfileRow& row, const overlume::ros::FrameTransformer& tf);

    void ingest(const visualization_msgs::msg::MarkerArray& msg, double sim_time_sec);

    void fill(overlume::ros::SceneAssembly& out) const;

    const AdapterStats& stats() const { return stats_; }

    void mark_stale_tick() { ++stats_.dropped_stale; }

private:
    using Key = std::pair<std::string, int32_t>;

    struct StoredMarker {
        overlume::MarkerPrimitive primitive{overlume::MarkerPrimitive::CUBE};
        overlume::Vec3 position{};
        double heading_rad{0.0};
        overlume::Vec3 scale{1.0, 1.0, 1.0};
        float color[4]{0.0f, 0.0f, 0.0f, 0.0f};
        std::string text;
        std::string mesh_path;
        std::vector<overlume::Vec3> points;

        std::vector<overlume::Vec3> fan_positions;
        std::vector<float> fan_colors;

        double last_update_sec{0.0};
        double expires_at_sec{0.0};
    };

    ProfileRow row_;
    const overlume::ros::FrameTransformer& tf_;
    std::map<Key, StoredMarker> storage_;
    AdapterStats stats_;
};

}
