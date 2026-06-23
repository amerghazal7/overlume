#pragma once
namespace micropilot::rendering
{
__device__ __forceinline__ float smoothstep01(float x)
{
    x = fminf(fmaxf(x, 0.0f), 1.0f);
    return x * x * (3.0f - 2.0f * x);
}
__device__ __forceinline__ float border_feather(float u, float v, int W, int H, float margin)
{
    float d = fminf(fminf(u, (W - 1) - u), fminf(v, (H - 1) - v));
    if (margin <= 0.0f) return d >= 0.0f ? 1.0f : 0.0f;
    return smoothstep01(d / margin);
}
}  // namespace micropilot::rendering
