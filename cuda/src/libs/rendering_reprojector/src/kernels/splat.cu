/** @file splat.cu @brief Forward point-cloud splat with packed-atomicMin z-buffer. */
#include <cuda_runtime.h>
#include <stdint.h>

#include "kernels/camdev.hpp"
#include "kernels/vecmath.cuh"

namespace micropilot::rendering
{
// Pack (float depth bits | point index). For z>0, IEEE float bits are monotonic,
// so atomicMin on the u64 selects the nearest point; low 32 bits index the winner.
__global__ void zbuf_init(unsigned long long* zbuf, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) zbuf[i] = 0xFFFFFFFFFFFFFFFFULL;
}

__global__ void splat_kernel(unsigned long long* zbuf, int OW, int OH, const float* pts,
                             int npts, CamDev v, int radius)
{
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if (p >= npts) return;
    float3 P = make_float3(pts[p * 3], pts[p * 3 + 1], pts[p * 3 + 2]);
    float3 rel = vsub(P, v.t);
    float z = vdot(v.fwd, rel);
    if (z <= 1e-6f) return;
    float xp = v.fx * vdot(v.right, rel) / z + v.cx;
    float yp = v.fy * vdot(v.down, rel) / z + v.cy;
    int cx = (int)lrintf(xp), cy = (int)lrintf(yp);
    unsigned int zbits = __float_as_uint(z);
    unsigned long long packed = ((unsigned long long)zbits << 32) | (unsigned int)p;
    for (int dy = -radius; dy <= radius; ++dy)
        for (int dx = -radius; dx <= radius; ++dx)
        {
            int x = cx + dx, y = cy + dy;
            if (x < 0 || x >= OW || y < 0 || y >= OH) continue;
            atomicMin(&zbuf[y * OW + x], packed);
        }
}

__global__ void resolve_kernel(const unsigned long long* zbuf, const float* cols, float* out,
                               int n, float fr, float fg, float fb)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    unsigned long long z = zbuf[i];
    if (z == 0xFFFFFFFFFFFFFFFFULL)
    {
        out[i * 4] = fr;
        out[i * 4 + 1] = fg;
        out[i * 4 + 2] = fb;
        out[i * 4 + 3] = 0.0f;
        return;
    }
    unsigned int p = (unsigned int)(z & 0xFFFFFFFFULL);
    out[i * 4] = cols[p * 3];
    out[i * 4 + 1] = cols[p * 3 + 1];
    out[i * 4 + 2] = cols[p * 3 + 2];
    out[i * 4 + 3] = 1.0f;
}

void launch_splat(unsigned long long* d_zbuf, float* d_out, int OW, int OH, const float* d_pts,
                  const float* d_cols, int npts, CamDev v, int radius, float fr, float fg, float fb)
{
    int n = OW * OH, t = 256;
    zbuf_init<<<(n + t - 1) / t, t>>>(d_zbuf, n);
    splat_kernel<<<(npts + t - 1) / t, t>>>(d_zbuf, OW, OH, d_pts, npts, v, radius);
    resolve_kernel<<<(n + t - 1) / t, t>>>(d_zbuf, d_cols, d_out, n, fr, fg, fb);
}

// ---------------------------------------------------------------------------
// Composite kernel: depth-where-valid else bowl (Task 5).
// Per pixel: if depth alpha > 0.5 use depth RGB; else use bowl RGB.
// Output alpha = 1 if either input is valid, 0 otherwise.
// ---------------------------------------------------------------------------
__global__ void composite_kernel(const float* depth, const float* bowl, float* out, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    float dA = depth[i * 4 + 3];
    bool dv = dA > 0.5f, bv = bowl[i * 4 + 3] > 0.5f;
    const float* src = dv ? &depth[i * 4] : &bowl[i * 4];
    out[i * 4 + 0] = src[0];
    out[i * 4 + 1] = src[1];
    out[i * 4 + 2] = src[2];
    out[i * 4 + 3] = (dv || bv) ? 1.0f : 0.0f;
}

void launch_composite(const float* d_depth, const float* d_bowl, float* d_out, int n)
{
    int t = 256;
    composite_kernel<<<(n + t - 1) / t, t>>>(d_depth, d_bowl, d_out, n);
}
}  // namespace micropilot::rendering
