#pragma once
/** @file camdev.hpp @brief CamDev struct + launch_bowl/launch_fill declarations (shared
 * between reproject.cu and reprojector.cpp). */
#include <cuda_runtime.h>

namespace micropilot::rendering
{
/** Per-camera device record: K split + R columns + center + plumb_bob distortion. */
struct CamDev
{
    float fx, fy, cx, cy;
    float3 right, down, fwd, t;
    int w, h;
    float k1, k2, p1, p2, k3;   // plumb_bob; all-zero = pinhole
};

/** Toolchain smoke kernel (Task 2 scaffolding). */
void launch_fill(float* d_out, int n, float r, float g, float b, float a);

/** Real bowl reprojection kernel (Task 3). mark_uncovered != 0 writes alpha 0.5
 *  (instead of 0) to surface-hit pixels no camera covers, for launch_hole_fill.
 *  d_selfmask (nullable): per-camera (ncam,H,W) uint8 body masks — nonzero
 *  source pixels see the robot itself and are skipped like out-of-bounds. */
void launch_bowl(float* d_out, int OW, int OH, const float* d_images, const CamDev* d_cams,
                 int ncam, CamDev v, int surf_type, float flat_z0, float R0, float k, float Rmax,
                 float feather_margin, float fr, float fg, float fb, int mark_uncovered,
                 const unsigned char* d_selfmask);

/** Blind-zone fill: iteratively grows valid (alpha 1) colors into alpha-0.5
 *  marked pixels (Jacobi, ping-pong via d_scratch); leftovers revert to alpha 0.
 *  Result ends in d_buf. Run BEFORE the robot overlay so it can't bleed in. */
void launch_hole_fill(float* d_buf, float* d_scratch, int OW, int OH, int iters);

/** Forward point-cloud splat with packed-atomicMin z-buffer (Task 4). */
void launch_splat(unsigned long long* d_zbuf, float* d_out, int OW, int OH, const float* d_pts,
                  const float* d_cols, int npts, CamDev v, int radius, float fr, float fg,
                  float fb);

/** Composite depth-where-valid else bowl, per pixel (Task 5).
 *  n = OW * OH (number of pixels); all buffers are device RGBA (4 floats/pixel). */
void launch_composite(const float* d_depth, const float* d_bowl, float* d_out, int n);

/** Robot proxy triangle raster: z-buffered mesh layer for compositing (robot proxy). */
void launch_robot(unsigned long long* d_zbuf, float* d_layer, int OW, int OH,
                  const float* d_verts, const float* d_cols, int ntris, CamDev v);
}  // namespace micropilot::rendering
