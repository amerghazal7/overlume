#pragma once
/** @file camdev.hpp @brief CamDev struct + launch_bowl/launch_fill declarations (shared
 * between reproject.cu and reprojector.cpp). */
#include <cuda_runtime.h>

namespace micropilot::rendering
{
/** Per-camera device record: K split + R columns + center. */
struct CamDev
{
    float fx, fy, cx, cy;
    float3 right, down, fwd, t;
    int w, h;
};

/** Toolchain smoke kernel (Task 2 scaffolding). */
void launch_fill(float* d_out, int n, float r, float g, float b, float a);

/** Real bowl reprojection kernel (Task 3). */
void launch_bowl(float* d_out, int OW, int OH, const float* d_images, const CamDev* d_cams,
                 int ncam, CamDev v, int surf_type, float flat_z0, float R0, float k, float Rmax,
                 float feather_margin, float fr, float fg, float fb);

/** Forward point-cloud splat with packed-atomicMin z-buffer (Task 4). */
void launch_splat(unsigned long long* d_zbuf, float* d_out, int OW, int OH, const float* d_pts,
                  const float* d_cols, int npts, CamDev v, int radius, float fr, float fg,
                  float fb);
}  // namespace micropilot::rendering
