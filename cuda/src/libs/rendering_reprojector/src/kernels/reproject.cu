/** @file reproject.cu @brief Backward bowl reprojection kernel (port of NumpyRenderer). */
#include <cuda_runtime.h>

#include "kernels/blend.cuh"
#include "kernels/camdev.hpp"
#include "kernels/sampling.cuh"
#include "kernels/surface.cuh"
#include "kernels/vecmath.cuh"

namespace micropilot::rendering
{
__global__ void bowl_kernel(float* out, int OW, int OH, const float* images, const CamDev* cams,
                            int ncam, CamDev v, int surf_type, float flat_z0, float R0, float k,
                            float Rmax, float feather_margin, float fr, float fg, float fb)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= OW || y >= OH) return;
    int idx = (y * OW + x) * 4;

    float xc = (x - v.cx) / v.fx;
    float yc = (y - v.cy) / v.fy;
    float3 dc = make_float3(xc, yc, 1.0f);
    float3 dir = vnorm(make_float3(v.right.x * dc.x + v.down.x * dc.y + v.fwd.x * dc.z,
                                   v.right.y * dc.x + v.down.y * dc.y + v.fwd.y * dc.z,
                                   v.right.z * dc.x + v.down.z * dc.y + v.fwd.z * dc.z));
    float3 o = v.t;
    float3 P;
    if (!intersect(o, dir, surf_type, flat_z0, R0, k, Rmax, P))
    {
        out[idx] = fr; out[idx+1] = fg; out[idx+2] = fb; out[idx+3] = 0.0f;
        return;
    }
    float ar = 0, ag = 0, ab = 0, wsum = 0;
    for (int i = 0; i < ncam; ++i)
    {
        CamDev c = cams[i];
        float3 rel = vsub(P, c.t);
        float z = vdot(c.fwd, rel);
        if (z <= 1e-9f) continue;
        float xp = c.fx * vdot(c.right, rel) / z + c.cx;
        float yp = c.fy * vdot(c.down, rel) / z + c.cy;
        if (xp < 0 || xp > c.w - 1 || yp < 0 || yp > c.h - 1) continue;
        float r, g, b;
        bilinear(images + (size_t)i * c.w * c.h * 3, c.w, c.h, xp, yp, r, g, b);
        float align = fmaxf(0.0f, fminf(1.0f, vdot(vnorm(rel), c.fwd)));
        float w = border_feather(xp, yp, c.w, c.h, feather_margin) * align * align;
        ar += w * r; ag += w * g; ab += w * b; wsum += w;
    }
    if (wsum > 0.0f)
    {
        out[idx] = ar / wsum; out[idx+1] = ag / wsum; out[idx+2] = ab / wsum; out[idx+3] = 1.0f;
    }
    else { out[idx] = fr; out[idx+1] = fg; out[idx+2] = fb; out[idx+3] = 0.0f; }
}

void launch_bowl(float* d_out, int OW, int OH, const float* d_images, const CamDev* d_cams,
                 int ncam, CamDev v, int surf_type, float flat_z0, float R0, float k, float Rmax,
                 float feather_margin, float fr, float fg, float fb)
{
    dim3 block(16, 16);
    dim3 grid((OW + 15) / 16, (OH + 15) / 16);
    bowl_kernel<<<grid, block>>>(d_out, OW, OH, d_images, d_cams, ncam, v, surf_type, flat_z0, R0,
                                 k, Rmax, feather_margin, fr, fg, fb);
}
}  // namespace micropilot::rendering
