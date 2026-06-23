#pragma once
/** @file reprojector.hpp @brief CUDA virtual-camera reprojection (bowl/depth/hybrid). */

#include <cstddef>
#include <vector>

#include "rendering_reprojector/types.hpp"

namespace micropilot::rendering
{
/**
 * @class Reprojector
 * @brief Synthesizes a virtual camera view by reprojecting N real cameras on the GPU.
 *
 * Upload calibrations and images once (static scene), then render per virtual pose.
 * Output is host (H,W,4) float row-major, row 0 = top; RGB = color, A = valid mask.
 */
class Reprojector
{
public:
    Reprojector(int out_width, int out_height);
    ~Reprojector();
    Reprojector(const Reprojector&) = delete;
    Reprojector& operator=(const Reprojector&) = delete;

    void set_cameras(const std::vector<CameraParams>& cams);
    void upload_images(const float* nhwc, int n, int h, int w);
    void upload_depth(const float* nhw, int n, int h, int w);

    void render_bowl(const CameraParams& vcam, const BowlParams& bowl, float* out_rgba);
    void render_depth(const CameraParams& vcam, int splat_radius, float* out_rgba);
    void render_hybrid(const CameraParams& vcam, const BowlParams& bowl, int splat_radius,
                       float* out_rgba);

private:
    struct Impl;
    Impl* impl_;
};
}  // namespace micropilot::rendering
