// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

/** @file lidar_colorize.cpp
 *  @brief See lidar_colorize.hpp. VM-094 (unified-engine migration Task 5)
 *  Steps 0-2.
 */
#include "overlume_ros/lidar_colorize.hpp"

// overlume's own private, portable, host-only projection math
// (Task 2 Step 0) -- not part of its public include/ surface, so this
// package's CMakeLists.txt adds OVERLUME_DIR/src as an extra
// PRIVATE include dir for this one file's sake (see that file's own
// comment). Pure-POD signature (Vec3/CameraExtrinsics/CameraIntrinsics by
// value/pointer, no std:: types crossing), so calling into the symbol
// already compiled into liboverlume.a from this gcc/libstdc++ TU is
// exactly as safe as every other api.h/scene.h POD-boundary call this node
// already makes (Global Constraints).
#include "bowl_projection.hpp"

#include <array>
#include <cmath>

namespace overlume::ros {

namespace {
// scene.h's PointCloudPoint::rgba convention (point_cloud.cpp's own
// comment): byte0=r, byte1=g, byte2=b, byte3=a. Every point this file
// returns has a real camera sample, so alpha is always 255 -- Decision 5's
// "uncovered points are dropped, never emitted at alpha 0" rule.
// sRGB -> linear, 256-entry LUT: point_cloud.mat binds this byte as a plain
// normalized vertex COLOR (no sampler, so no hardware sRGB decode -- unlike
// the bowl's SRGB8 camera textures, fixed 2026-09-11), and the renderer's
// output OETF re-encodes at the end -- packing raw sRGB camera bytes here
// double-encodes them, the same washed-out bug the bowl had. Adapter-baked
// point-cloud colors (intensity/height colormaps) are NOT linearized: they
// were authored against the existing pipeline and are not camera samples.
// 8-bit linear loses some shadow precision (mild banding in darks) --
// acceptable for lidar speckle.
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
}  // namespace

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
                continue;  // this camera doesn't cover this point -- try the next configured one
            }

            // Nearest-neighbor sample: first-match has no blend weight to
            // interpolate against, so bilinear buys nothing here (Decision 5).
            uint32_t px = static_cast<uint32_t>(u * static_cast<float>(w));
            uint32_t py = static_cast<uint32_t>(v * static_cast<float>(h));
            if (px >= w) px = w - 1;
            if (py >= h) py = h - 1;
            const uint8_t* pixel = camera_rgb_buffers[cam] + (static_cast<size_t>(py) * w + px) * 3;

            overlume::PointCloudPoint pt{};
            pt.position = p;
            pt.rgba = pack_rgba(pixel[0], pixel[1], pixel[2]);
            out.push_back(pt);
            break;  // first-match wins (Decision 5 / Step 2) -- never try a later camera
        }
        // No configured camera covered this point: DROPPED (Step 1), not
        // appended with any sentinel color.
    }
    return out;
}

}  // namespace overlume::ros
