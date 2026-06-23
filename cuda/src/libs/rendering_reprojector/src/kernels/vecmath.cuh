#pragma once
#include <cuda_runtime.h>

namespace micropilot::rendering
{
__device__ __forceinline__ float3 vsub(float3 a, float3 b) { return make_float3(a.x-b.x,a.y-b.y,a.z-b.z); }
__device__ __forceinline__ float3 vadd(float3 a, float3 b) { return make_float3(a.x+b.x,a.y+b.y,a.z+b.z); }
__device__ __forceinline__ float3 vscale(float3 a, float s) { return make_float3(a.x*s,a.y*s,a.z*s); }
__device__ __forceinline__ float vdot(float3 a, float3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
__device__ __forceinline__ float3 vnorm(float3 a)
{
    float n = sqrtf(vdot(a, a)) + 1e-12f;
    return vscale(a, 1.0f / n);
}
}  // namespace micropilot::rendering
