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

    /** Upload the robot proxy mesh (persistent; call once). verts = n_tris*9
     *  rig-frame xyz, cols = n_tris*3 baked RGB (see mesh_loader.hpp).
     *  n_tris == 0 disables robot compositing. */
    void upload_robot_mesh(const float* verts, const float* cols, std::size_t n_tris);

    void render_bowl(const CameraParams& vcam, const BowlParams& bowl, float* out_rgba);
    void render_depth(const CameraParams& vcam, int splat_radius, float* out_rgba);
    void render_hybrid(const CameraParams& vcam, const BowlParams& bowl, int splat_radius,
                       float* out_rgba);

    /// Diagnostics/tests: cumulative number of device-buffer (re)allocations.
    /// Device buffers are persistent and grow-only, so after the working set is
    /// reached this stays constant — the steady-state render path allocates nothing.
    std::size_t device_alloc_count() const;

private:
    struct Impl;
    Impl* impl_;
};
}  // namespace micropilot::rendering
