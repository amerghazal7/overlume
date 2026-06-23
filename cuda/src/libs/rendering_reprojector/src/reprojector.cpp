/** @file reprojector.cpp @brief Host orchestration + device buffer management (skeleton). */

#include "rendering_reprojector/reprojector.hpp"

#include <cuda_runtime.h>

#include <cstring>
#include <vector>

namespace micropilot::rendering
{
// Declared in fill.cu — toolchain smoke kernel (Task 2 scaffolding; replaced in Task 3).
void launch_fill(float* d_out, int n, float r, float g, float b, float a);

struct Reprojector::Impl
{
    int out_w;
    int out_h;
    std::vector<CameraParams> cams;
    int img_n = 0, img_h = 0, img_w = 0;
};

Reprojector::Reprojector(int out_width, int out_height) : impl_(new Impl)
{
    impl_->out_w = out_width;
    impl_->out_h = out_height;
}

Reprojector::~Reprojector() { delete impl_; }

void Reprojector::set_cameras(const std::vector<CameraParams>& cams) { impl_->cams = cams; }

void Reprojector::upload_images(const float*, int n, int h, int w)
{
    impl_->img_n = n;
    impl_->img_h = h;
    impl_->img_w = w;
}

void Reprojector::upload_depth(const float*, int, int, int) {}

void Reprojector::render_bowl(const CameraParams&, const BowlParams&, float* out_rgba)
{
    // Temporary scaffolding (Task 2): fill every pixel with a constant RGBA via CUDA.
    // Task 3 replaces this with the real reprojection kernel.
    int n = impl_->out_w * impl_->out_h;
    float* d_out = nullptr;
    cudaMalloc(&d_out, sizeof(float) * n * 4);
    launch_fill(d_out, n, 0.1f, 0.2f, 0.3f, 1.0f);
    cudaDeviceSynchronize();
    cudaMemcpy(out_rgba, d_out, sizeof(float) * n * 4, cudaMemcpyDeviceToHost);
    cudaFree(d_out);
}

void Reprojector::render_depth(const CameraParams&, int, float* out_rgba)
{
    std::memset(out_rgba, 0, sizeof(float) * impl_->out_w * impl_->out_h * 4);
}

void Reprojector::render_hybrid(const CameraParams&, const BowlParams&, int, float* out_rgba)
{
    std::memset(out_rgba, 0, sizeof(float) * impl_->out_w * impl_->out_h * 4);
}
}  // namespace micropilot::rendering
