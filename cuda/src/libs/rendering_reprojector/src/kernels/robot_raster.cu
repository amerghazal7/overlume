/** @file robot_raster.cu @brief Robot proxy triangle raster into a packed z-buffer.
 *
 *  Same packing as splat.cu (depth float bits << 32 | index): for z > 0, IEEE
 *  float bits are monotonic, so atomicMin selects the nearest triangle; the low
 *  32 bits index the winning triangle's color. zbuf_init/resolve_kernel are
 *  reused from splat.cu (external linkage; host-side launches only).
 */
#include <cuda_runtime.h>
#include <stdint.h>

#include "kernels/camdev.hpp"
#include "kernels/vecmath.cuh"

namespace micropilot::rendering
{
// defined in splat.cu
__global__ void zbuf_init(unsigned long long* zbuf, int n);
__global__ void resolve_kernel(const unsigned long long* zbuf, const float* cols, float* out,
                               int n, float fr, float fg, float fb);

// One thread per triangle: project the 3 vertices with the virtual camera,
// clamp the screen bbox, test pixel centers with barycentric edge functions
// (signed sub-area / signed area — winding-independent), atomicMin the packed
// affine-interpolated depth. Affine (not perspective-correct) depth across a
// triangle is fine here: the M02P triangles are millimetre-scale.
// ponytail: a near triangle spanning the screen would walk a huge bbox; the
// robot is always >= 1.5 m from the vcam (orbit DIST_MIN), so bboxes stay tiny.
__global__ void robot_raster_kernel(unsigned long long* zbuf, int OW, int OH,
                                    const float* verts, int ntris, CamDev v)
{
    int tri = blockIdx.x * blockDim.x + threadIdx.x;
    if (tri >= ntris) return;
    const float* q = verts + tri * 9;
    float sx[3], sy[3], sz[3];
    for (int k = 0; k < 3; ++k)
    {
        float3 P = make_float3(q[k * 3], q[k * 3 + 1], q[k * 3 + 2]);
        float3 rel = vsub(P, v.t);
        float z = vdot(v.fwd, rel);
        if (z <= 1e-4f) return;  // any vertex at/behind the camera -> drop triangle
        float xn = vdot(v.right, rel) / z;
        float yn = vdot(v.down, rel) / z;
        if (v.k1 != 0.0f || v.k2 != 0.0f || v.p1 != 0.0f || v.p2 != 0.0f || v.k3 != 0.0f)
        {
            // plumb_bob forward distortion per vertex (piecewise-linear across
            // mm-scale triangles) — used when rasterizing self-view masks from a
            // real distorted camera. Same out-of-field guard as reproject.cu.
            float r2 = xn * xn + yn * yn;
            if (r2 > 3.0f) return;
            float radial = 1.0f + r2 * (v.k1 + r2 * (v.k2 + r2 * v.k3));
            float xd = xn * radial + 2.0f * v.p1 * xn * yn + v.p2 * (r2 + 2.0f * xn * xn);
            float yd = yn * radial + v.p1 * (r2 + 2.0f * yn * yn) + 2.0f * v.p2 * xn * yn;
            xn = xd; yn = yd;
        }
        sx[k] = v.fx * xn + v.cx;
        sy[k] = v.fy * yn + v.cy;
        sz[k] = z;
    }
    float area = (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sy[1] - sy[0]) * (sx[2] - sx[0]);
    if (fabsf(area) < 1e-12f) return;  // degenerate/edge-on
    int x0 = max(0, (int)floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
    int x1 = min(OW - 1, (int)ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
    int y0 = max(0, (int)floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
    int y1 = min(OH - 1, (int)ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x)
        {
            float px = x + 0.5f, py = y + 0.5f;
            float w0 = ((sx[1] - px) * (sy[2] - py) - (sy[1] - py) * (sx[2] - px)) / area;
            float w1 = ((sx[2] - px) * (sy[0] - py) - (sy[2] - py) * (sx[0] - px)) / area;
            float w2 = 1.0f - w0 - w1;
            if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) continue;
            float z = w0 * sz[0] + w1 * sz[1] + w2 * sz[2];
            unsigned long long packed =
                ((unsigned long long)__float_as_uint(z) << 32) | (unsigned int)tri;
            atomicMin(&zbuf[y * OW + x], packed);
        }
}

void launch_robot(unsigned long long* d_zbuf, float* d_layer, int OW, int OH,
                  const float* d_verts, const float* d_cols, int ntris, CamDev v)
{
    int n = OW * OH, t = 256;
    zbuf_init<<<(n + t - 1) / t, t>>>(d_zbuf, n);
    robot_raster_kernel<<<(ntris + t - 1) / t, t>>>(d_zbuf, OW, OH, d_verts, ntris, v);
    // fill = black with alpha 0 -> launch_composite keeps the environment there
    resolve_kernel<<<(n + t - 1) / t, t>>>(d_zbuf, d_cols, d_layer, n, 0.0f, 0.0f, 0.0f);
}
}  // namespace micropilot::rendering
