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
                            float Rmax, float feather_margin, float fr, float fg, float fb,
                            int mark_uncovered)
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
    else
    {
        // Surface hit but no camera coverage (blind zone). alpha 0.5 marks it
        // for launch_hole_fill; without marking it stays plain-invalid (legacy).
        out[idx] = fr; out[idx+1] = fg; out[idx+2] = fb;
        out[idx+3] = mark_uncovered ? 0.5f : 0.0f;
    }
}

// One Jacobi step of blind-zone fill: an unfilled hole pixel (alpha == 0.5)
// takes the average color of its valid (alpha == 1) 8-neighbours and becomes
// valid; everything else passes through. Ping-pong src -> dst.
__global__ void hole_fill_kernel(const float* src, float* dst, int OW, int OH)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= OW || y >= OH) return;
    int idx = (y * OW + x) * 4;
    float a = src[idx + 3];
    if (a != 0.5f)
    {
        dst[idx] = src[idx]; dst[idx+1] = src[idx+1];
        dst[idx+2] = src[idx+2]; dst[idx+3] = a;
        return;
    }
    float r = 0, g = 0, b = 0;
    int n = 0;
    for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx)
        {
            int nx = x + dx, ny = y + dy;
            if (nx < 0 || nx >= OW || ny < 0 || ny >= OH) continue;
            int nidx = (ny * OW + nx) * 4;
            if (src[nidx + 3] == 1.0f)
            {
                r += src[nidx]; g += src[nidx + 1]; b += src[nidx + 2];
                ++n;
            }
        }
    if (n > 0)
    {
        dst[idx] = r / n; dst[idx+1] = g / n; dst[idx+2] = b / n; dst[idx+3] = 1.0f;
    }
    else
    {
        dst[idx] = src[idx]; dst[idx+1] = src[idx+1];
        dst[idx+2] = src[idx+2]; dst[idx+3] = 0.5f;
    }
}

// Any hole pixel still unfilled after the iteration budget reverts to plain
// invalid (alpha 0) so the consumer's sky fill applies as before.
__global__ void hole_finalize_kernel(float* buf, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    if (buf[i * 4 + 3] == 0.5f) buf[i * 4 + 3] = 0.0f;
}

void launch_bowl(float* d_out, int OW, int OH, const float* d_images, const CamDev* d_cams,
                 int ncam, CamDev v, int surf_type, float flat_z0, float R0, float k, float Rmax,
                 float feather_margin, float fr, float fg, float fb, int mark_uncovered)
{
    dim3 block(16, 16);
    dim3 grid((OW + 15) / 16, (OH + 15) / 16);
    bowl_kernel<<<grid, block>>>(d_out, OW, OH, d_images, d_cams, ncam, v, surf_type, flat_z0, R0,
                                 k, Rmax, feather_margin, fr, fg, fb, mark_uncovered);
}

void launch_hole_fill(float* d_buf, float* d_scratch, int OW, int OH, int iters)
{
    dim3 block(16, 16);
    dim3 grid((OW + 15) / 16, (OH + 15) / 16);
    // ponytail: fixed Jacobi budget fills holes up to ~iters px from a valid
    // border (~2m ring at top_down zoom); add a device done-flag early-exit if
    // profiling ever shows these launches mattering.
    float* src = d_buf;
    float* dst = d_scratch;
    for (int i = 0; i < iters; ++i)
    {
        hole_fill_kernel<<<grid, block>>>(src, dst, OW, OH);
        float* tmp = src; src = dst; dst = tmp;
    }
    // result lives in src; move it home if the iteration count was odd
    if (src != d_buf)
        cudaMemcpy(d_buf, src, sizeof(float) * OW * OH * 4, cudaMemcpyDeviceToDevice);
    int n = OW * OH, t = 256;
    hole_finalize_kernel<<<(n + t - 1) / t, t>>>(d_buf, n);
}
}  // namespace micropilot::rendering
