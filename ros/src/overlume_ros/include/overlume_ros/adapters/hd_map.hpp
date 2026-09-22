// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstddef>
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

class HdMapAdapter {
public:
    HdMapAdapter(const ProfileRow& row, const overlume::ros::FrameTransformer& tf);

    void ingest(const visualization_msgs::msg::MarkerArray& msg, double sim_time_sec);

    void fill(overlume::ros::SceneAssembly& out) const;

    const AdapterStats& stats() const { return stats_; }

private:
    struct StoredElement {
        std::vector<overlume::Vec3> points;
        uint8_t is_polygon{0};
        overlume::MapKind kind{overlume::MapKind::OTHER};
        uint32_t lane_id{0};
    };
    using Key = std::pair<std::string, int32_t>;
    struct KeyHash {
        size_t operator()(const Key& k) const noexcept {
            return std::hash<std::string>{}(k.first) ^ (std::hash<int32_t>{}(k.second) << 1);
        }
    };

    struct RoadFillCacheEntry {
        std::vector<overlume::Vec3> left;
        std::vector<overlume::Vec3> right;
        double cached_recv_sec{-1.0};
    };
    mutable std::unordered_map<uint32_t, RoadFillCacheEntry> road_fill_cache_;

    ProfileRow row_;
    const overlume::ros::FrameTransformer& tf_;
    std::unordered_map<Key, std::vector<StoredElement>, KeyHash> storage_;
    mutable std::vector<std::vector<overlume::Vec3>> road_surface_points_;
    mutable std::vector<std::vector<overlume::Vec3>> junction_cut_points_;
    AdapterStats stats_;
    double last_rebuild_sec_{-1.0};
    double last_recv_sec_{-1.0};
};

}  // namespace overlume::ros
