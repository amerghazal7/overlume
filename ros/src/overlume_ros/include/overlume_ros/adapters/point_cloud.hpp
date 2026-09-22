// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "overlume_ros/adapter_stats.hpp"
#include "overlume_ros/frame_transform.hpp"
#include "overlume_ros/profile.hpp"
#include "overlume_ros/scene_assembly.hpp"
#include "overlume/scene.h"

namespace overlume::ros {

inline uint32_t PackRgba(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    return static_cast<uint32_t>(r) | (static_cast<uint32_t>(g) << 8) |
           (static_cast<uint32_t>(b) << 16) | (static_cast<uint32_t>(a) << 24);
}

class PointCloudAdapter {
public:
    PointCloudAdapter(const ProfileRow& row, const overlume::ros::FrameTransformer& tf);

    void ingest(const sensor_msgs::msg::PointCloud2& msg, double sim_time_sec);

    void fill(overlume::ros::SceneAssembly& out) const;

    const AdapterStats& stats() const { return stats_; }

    void mark_stale_tick() { ++stats_.dropped_stale; }

    uint64_t dropped_below_min_z() const { return dropped_below_min_z_; }

private:
    ProfileRow row_;
    const overlume::ros::FrameTransformer& tf_;

    bool has_data_{false};
    std::vector<overlume::PointCloudPoint> storage_;
    double last_update_sec_{0.0};
    AdapterStats stats_;
    uint64_t dropped_below_min_z_{0};
};

}
