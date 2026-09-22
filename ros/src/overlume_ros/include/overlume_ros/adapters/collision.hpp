// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <visualization_msgs/msg/marker_array.hpp>

#include "overlume_ros/adapter_stats.hpp"
#include "overlume_ros/frame_transform.hpp"
#include "overlume_ros/profile.hpp"
#include "overlume_ros/scene_assembly.hpp"
#include "overlume/scene.h"

namespace overlume::ros {

uint8_t severity_for_role(const std::string& role);

class CollisionAdapter {
public:
    CollisionAdapter(const ProfileRow& row, const overlume::ros::FrameTransformer& tf);

    void ingest(const visualization_msgs::msg::MarkerArray& msg, double sim_time_sec);

    void fill(overlume::ros::SceneAssembly& out) const;

    const AdapterStats& stats() const { return stats_; }

    void mark_stale_tick() { ++stats_.dropped_stale; }

private:
    using Key = std::pair<std::string, int32_t>;
    struct KeyHash {
        size_t operator()(const Key& k) const noexcept {
            return std::hash<std::string>{}(k.first) ^ (std::hash<int32_t>{}(k.second) << 1);
        }
    };
    struct StoredPolygon {
        std::vector<overlume::Vec3> points;
        double last_update_sec{0.0};
    };

    ProfileRow row_;
    const overlume::ros::FrameTransformer& tf_;
    uint8_t severity_;
    std::unordered_map<Key, StoredPolygon, KeyHash> storage_;
    AdapterStats stats_;
};

}  // namespace overlume::ros
