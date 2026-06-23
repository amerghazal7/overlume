/** @file fill.cu @brief Toolchain smoke kernel: write a constant RGBA per pixel. */
#include <cuda_runtime.h>

namespace micropilot::rendering
{
__global__ void fill_kernel(float* out, int n, float r, float g, float b, float a)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    out[i * 4 + 0] = r;
    out[i * 4 + 1] = g;
    out[i * 4 + 2] = b;
    out[i * 4 + 3] = a;
}

void launch_fill(float* d_out, int n, float r, float g, float b, float a)
{
    int threads = 256;
    fill_kernel<<<(n + threads - 1) / threads, threads>>>(d_out, n, r, g, b, a);
}
}  // namespace micropilot::rendering
