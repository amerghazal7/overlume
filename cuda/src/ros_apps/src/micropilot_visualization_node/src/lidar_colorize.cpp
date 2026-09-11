/** @file lidar_colorize.cpp
 *  @brief See lidar_colorize.hpp. VM-094 (unified-engine migration Task 5)
 *  Steps 0-2.
 */
#include "micropilot_visualization_node/lidar_colorize.hpp"

// visual_renderer's own private, portable, host-only projection math
// (Task 2 Step 0) -- not part of its public include/ surface, so this
// package's CMakeLists.txt adds VISUAL_RENDERER_DIR/src as an extra
// PRIVATE include dir for this one file's sake (see that file's own
// comment). Pure-POD signature (Vec3/CameraExtrinsics/CameraIntrinsics by
// value/pointer, no std:: types crossing), so calling into the symbol
// already compiled into libvisual_renderer.a from this gcc/libstdc++ TU is
// exactly as safe as every other api.h/scene.h POD-boundary call this node
// already makes (Global Constraints).
#include "bowl_projection.hpp"

namespace micropilot::visualization_app
{

namespace
{
// scene.h's PointCloudPoint::rgba convention (point_cloud.cpp's own
// comment): byte0=r, byte1=g, byte2=b, byte3=a. Every point this file
// returns has a real camera sample, so alpha is always 255 -- Decision 5's
// "uncovered points are dropped, never emitted at alpha 0" rule.
uint32_t pack_rgba(uint8_t r, uint8_t g, uint8_t b)
{
    return static_cast<uint32_t>(r) | (static_cast<uint32_t>(g) << 8) |
           (static_cast<uint32_t>(b) << 16) | (static_cast<uint32_t>(255) << 24);
}
}  // namespace

std::vector<mpviz::PointCloudPoint> ColorizeFromCameras(
    const std::vector<mpviz::Vec3>& lidar_points_rig_frame, const mpviz::BowlConfig& cameras,
    const std::vector<const uint8_t*>& camera_rgb_buffers)
{
    std::vector<mpviz::PointCloudPoint> out;
    out.reserve(lidar_points_rig_frame.size());

    for (const auto& p : lidar_points_rig_frame)
    {
        for (uint32_t cam = 0; cam < cameras.camera_count; ++cam)
        {
            if (cam >= camera_rgb_buffers.size() || camera_rgb_buffers[cam] == nullptr) continue;
            const uint32_t w = cameras.cam_width[cam];
            const uint32_t h = cameras.cam_height[cam];
            if (w == 0 || h == 0) continue;

            float u, v;
            if (!mpviz::bowl::ProjectToCameraUv(cameras.extrinsics[cam], cameras.intrinsics[cam], w,
                                                 h, p, &u, &v))
            {
                continue;  // this camera doesn't cover this point -- try the next configured one
            }

            // Nearest-neighbor sample: first-match has no blend weight to
            // interpolate against, so bilinear buys nothing here (Decision 5).
            uint32_t px = static_cast<uint32_t>(u * static_cast<float>(w));
            uint32_t py = static_cast<uint32_t>(v * static_cast<float>(h));
            if (px >= w) px = w - 1;
            if (py >= h) py = h - 1;
            const uint8_t* pixel = camera_rgb_buffers[cam] + (static_cast<size_t>(py) * w + px) * 3;

            mpviz::PointCloudPoint pt{};
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

}  // namespace micropilot::visualization_app
