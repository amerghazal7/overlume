// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <vector>

#include "overlume/scene.h"

namespace overlume::ros {

// p' = Rd^T (p - pd), z unchanged: lidar cloud advanced from its stamp to the image reference time.
void CompensateCloud(std::vector<overlume::Vec3>& rig_pts, double th, double px, double py);

// rgba holds the camera's raw sRGB bytes (alpha 255); the splat material decodes them.
std::vector<overlume::PointCloudPoint> ColorizeFromCameras(
    const std::vector<overlume::Vec3>& lidar_points_rig_frame, const overlume::BowlConfig& cameras,
    const std::vector<const uint8_t*>& camera_rgb_buffers);

}
