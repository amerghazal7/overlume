// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <nav_msgs/msg/path.hpp>

#include "overlume_ros/adapter_stats.hpp"
#include "overlume_ros/frame_transform.hpp"
#include "overlume_ros/profile.hpp"
#include "overlume_ros/scene_assembly.hpp"
#include "overlume/scene.h"

namespace overlume::ros {

class PathAdapter {
public:
    PathAdapter(const ProfileRow& row, const overlume::ros::FrameTransformer& tf);

    void ingest(const nav_msgs::msg::Path& msg, double sim_time_sec);

    void fill(overlume::ros::SceneAssembly& out) const;

    const AdapterStats& stats() const { return stats_; }

    void mark_stale_tick() { ++stats_.dropped_stale; }

private:
    ProfileRow row_;
    const overlume::ros::FrameTransformer& tf_;
    overlume::PathRole role_{overlume::PathRole::LOCAL};
    std::vector<overlume::Vec3> points_;
    double last_update_sec_{0.0};
    AdapterStats stats_;
};

}  // namespace overlume::ros
