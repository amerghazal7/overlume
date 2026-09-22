// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <vector>

#include <visualization_msgs/msg/marker_array.hpp>

#include "overlume_ros/adapter_stats.hpp"
#include "overlume_ros/adapters/point_cloud.hpp"
#include "overlume_ros/frame_transform.hpp"
#include "overlume_ros/profile.hpp"
#include "overlume_ros/scene_assembly.hpp"
#include "overlume/scene.h"

namespace overlume::ros {

class TrajectoryCarpetAdapter {
public:
    TrajectoryCarpetAdapter(const ProfileRow& row, const overlume::ros::FrameTransformer& tf);

    void ingest(const visualization_msgs::msg::MarkerArray& msg, double sim_time_sec);

    void fill(overlume::ros::SceneAssembly& out) const;

    const AdapterStats& stats() const { return stats_; }

    void mark_stale_tick() { ++stats_.dropped_stale; }

private:
    ProfileRow row_;
    const overlume::ros::FrameTransformer& tf_;

    bool has_data_{false};
    std::vector<overlume::PointCloudPoint> storage_;
    double last_update_sec_{0.0};
    AdapterStats stats_;
};

}
