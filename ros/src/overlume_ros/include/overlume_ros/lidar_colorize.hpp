// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <vector>

#include "overlume/scene.h"

namespace overlume::ros {

std::vector<overlume::PointCloudPoint> ColorizeFromCameras(
    const std::vector<overlume::Vec3>& lidar_points_rig_frame, const overlume::BowlConfig& cameras,
    const std::vector<const uint8_t*>& camera_rgb_buffers);

}
