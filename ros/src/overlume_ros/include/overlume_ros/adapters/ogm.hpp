// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "overlume_ros/adapter_stats.hpp"
#include "overlume_ros/frame_transform.hpp"
#include "overlume_ros/profile.hpp"
#include "overlume_ros/scene_assembly.hpp"
#include "overlume/scene.h"

namespace overlume::ros {

inline constexpr uint8_t kUnknownCell = 255;

class OgmAdapter {
public:
    OgmAdapter(const ProfileRow& row, const overlume::ros::FrameTransformer& tf);

    void ingest(const nav_msgs::msg::OccupancyGrid& msg, double sim_time_sec);

    void ingest_update(const map_msgs::msg::OccupancyGridUpdate& msg, double sim_time_sec);

    void fill(overlume::ros::SceneAssembly& out) const;

    const AdapterStats& stats() const { return stats_; }

    void mark_stale_tick() { ++stats_.dropped_stale; }

private:
    ProfileRow row_;
    const overlume::ros::FrameTransformer& tf_;
    uint8_t kind_{0};

    bool has_grid_{false};
    overlume::Vec3 origin_{};
    double resolution_m_{0.0};
    uint32_t width_cells_{0};
    uint32_t height_cells_{0};
    std::vector<uint8_t> cells_;
    double last_update_sec_{0.0};
    AdapterStats stats_;
};

}  // namespace overlume::ros
