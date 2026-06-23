#pragma once
#include "kernels/vecmath.cuh"

namespace micropilot::rendering
{
__device__ __forceinline__ float bowl_height(float r, float R0, float k, float Rmax)
{
    float d = fminf(fmaxf(r - R0, 0.0f), Rmax - R0);
    return k * d * d;
}
__device__ __forceinline__ float bowl_g(float3 o, float3 dir, float t, float R0, float k, float Rmax)
{
    float3 P = vadd(o, vscale(dir, t));
    return P.z - bowl_height(sqrtf(P.x * P.x + P.y * P.y), R0, k, Rmax);
}
__device__ __forceinline__ bool intersect(float3 o, float3 dir, int surf_type, float flat_z0,
                                           float R0, float k, float Rmax, float3& P)
{
    if (surf_type == 0)
    {
        float dz = dir.z;
        if (fabsf(dz) <= 1e-12f) return false;
        float t = (flat_z0 - o.z) / dz;
        if (t <= 1e-9f) return false;
        P = vadd(o, vscale(dir, t));
        return true;
    }
    float eps = 1e-6f, tmax = 1.0e4f;
    if (!(bowl_g(o, dir, eps, R0, k, Rmax) > 0.0f && bowl_g(o, dir, tmax, R0, k, Rmax) < 0.0f))
        return false;
    float lo = eps, hi = tmax;
    for (int i = 0; i < 60; i++)
    {
        float mid = 0.5f * (lo + hi);
        if (bowl_g(o, dir, mid, R0, k, Rmax) > 0.0f) lo = mid;
        else hi = mid;
    }
    P = vadd(o, vscale(dir, 0.5f * (lo + hi)));
    return true;
}
}  // namespace micropilot::rendering
