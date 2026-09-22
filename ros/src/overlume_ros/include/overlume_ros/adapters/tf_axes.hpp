// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <string>
#include <vector>

#include <tf2_ros/buffer.h>

#include "overlume_ros/adapter_stats.hpp"
#include "overlume_ros/profile.hpp"
#include "overlume_ros/scene_assembly.hpp"
#include "overlume/scene.h"

namespace overlume::ros {

class TfAxesAdapter {
public:
    explicit TfAxesAdapter(const ProfileRow& row, const tf2_ros::Buffer& buffer,
                           std::string target_frame = "map");

    void fill(overlume::ros::SceneAssembly& out, double sim_time_sec);

    const AdapterStats& stats() const { return stats_; }

private:
    ProfileRow row_;
    const tf2_ros::Buffer& buffer_;
    std::string target_frame_;
    std::vector<overlume::Vec3> point_storage_;
    std::vector<overlume::GenericMarker> axis_markers_;
    AdapterStats stats_;
};

}
