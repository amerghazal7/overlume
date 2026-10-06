// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "overlume_ros/adapter_stats.hpp"
#include "overlume_ros/frame_transform.hpp"
#include "overlume_ros/profile.hpp"
#include "overlume_ros/scene_assembly.hpp"
#include "overlume/scene.h"

namespace overlume::ros {

// One OccupancyGrid cell -> metres above the grid origin plane. -1 is unknown (NaN, not
// malformed). 0..100 decodes linearly (min + v/100*(max-min)) or, when `normalized`, on the
// 1..100 scale (min + clamp((v-1)/99, 0, 1)*(max-min)). Anything else is NaN and sets
// `malformed`.
float decode_height_cell(int8_t v, bool normalized, double min_m, double max_m, bool& malformed);

class HeightGridAdapter {
public:
    HeightGridAdapter(const ProfileRow& row, const overlume::ros::FrameTransformer& tf);

    void ingest(const nav_msgs::msg::OccupancyGrid& msg, double sim_time_sec);

    void fill(overlume::ros::SceneAssembly& out) const;

    const AdapterStats& stats() const { return stats_; }

    void mark_stale_tick() { ++stats_.dropped_stale; }

private:
    ProfileRow row_;
    const overlume::ros::FrameTransformer& tf_;
    bool normalized_{false};

    bool has_grid_{false};
    overlume::Vec3 origin_{};
    double yaw_rad_ = 0.0;
    double resolution_m_{0.0};
    uint32_t width_cells_{0};
    uint32_t height_cells_{0};
    std::vector<float> heights_;
    double last_update_sec_{0.0};
    AdapterStats stats_;
};

}
