// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/lidar_colorize.hpp"

#include "bowl_projection.hpp"

#include <array>
#include <cmath>

namespace overlume::ros {

namespace {
const std::array<uint8_t, 256>& srgb_to_linear_lut() {
    static const std::array<uint8_t, 256> lut = [] {
        std::array<uint8_t, 256> t{};
        for (int i = 0; i < 256; ++i) {
            const double c = i / 255.0;
            const double lin = c <= 0.04045 ? c / 12.92 : std::pow((c + 0.055) / 1.055, 2.4);
            t[static_cast<size_t>(i)] = static_cast<uint8_t>(std::lround(lin * 255.0));
        }
        return t;
    }();
    return lut;
}

uint32_t pack_rgba(uint8_t r, uint8_t g, uint8_t b) {
    const auto& lut = srgb_to_linear_lut();
    return static_cast<uint32_t>(lut[r]) | (static_cast<uint32_t>(lut[g]) << 8) |
           (static_cast<uint32_t>(lut[b]) << 16) | (static_cast<uint32_t>(255) << 24);
}
}

std::vector<overlume::PointCloudPoint> ColorizeFromCameras(
    const std::vector<overlume::Vec3>& lidar_points_rig_frame, const overlume::BowlConfig& cameras,
    const std::vector<const uint8_t*>& camera_rgb_buffers) {
    std::vector<overlume::PointCloudPoint> out;
    out.reserve(lidar_points_rig_frame.size());

    for (const auto& p : lidar_points_rig_frame) {
        for (uint32_t cam = 0; cam < cameras.camera_count; ++cam) {
            if (cam >= camera_rgb_buffers.size() || camera_rgb_buffers[cam] == nullptr) continue;
            const uint32_t w = cameras.cam_width[cam];
            const uint32_t h = cameras.cam_height[cam];
            if (w == 0 || h == 0) continue;

            float u, v;
            if (!overlume::bowl::ProjectToCameraUv(cameras.extrinsics[cam], cameras.intrinsics[cam],
                                                   w, h, p, &u, &v)) {
                continue;
            }

            uint32_t px = static_cast<uint32_t>(u * static_cast<float>(w));
            uint32_t py = static_cast<uint32_t>(v * static_cast<float>(h));
            if (px >= w) px = w - 1;
            if (py >= h) py = h - 1;
            const uint8_t* pixel = camera_rgb_buffers[cam] + (static_cast<size_t>(py) * w + px) * 3;

            overlume::PointCloudPoint pt{};
            pt.position = p;
            pt.rgba = pack_rgba(pixel[0], pixel[1], pixel[2]);
            out.push_back(pt);
            break;
        }
    }
    return out;
}

}
