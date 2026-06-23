/** @file reprojector.cpp @brief Host orchestration + device buffer management. */

#include "rendering_reprojector/reprojector.hpp"

#include <cuda_runtime.h>

#include <cassert>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "kernels/camdev.hpp"

// ---------------------------------------------------------------------------
// CUDA_CHECK macro (carry-forward #1 from Task-2 review)
// ---------------------------------------------------------------------------
#define CUDA_CHECK(call)                                                               \
    do                                                                                 \
    {                                                                                  \
        cudaError_t _e = (call);                                                       \
        if (_e != cudaSuccess)                                                         \
        {                                                                              \
            throw std::runtime_error(std::string("CUDA error in " #call ": ") +       \
                                     cudaGetErrorString(_e));                          \
        }                                                                              \
    } while (0)

namespace micropilot::rendering
{
// Convert a CameraParams → CamDev (device record).
static CamDev to_camdev(const CameraParams& c)
{
    CamDev d;
    d.fx = c.K[0]; d.fy = c.K[4]; d.cx = c.K[2]; d.cy = c.K[5];
    // R is row-major; columns = (right, down, fwd)
    d.right = make_float3(c.R[0], c.R[3], c.R[6]);
    d.down  = make_float3(c.R[1], c.R[4], c.R[7]);
    d.fwd   = make_float3(c.R[2], c.R[5], c.R[8]);
    d.t = make_float3(c.t[0], c.t[1], c.t[2]);
    d.w = c.width; d.h = c.height;
    return d;
}

struct Reprojector::Impl
{
    int out_w;
    int out_h;

    // Per-camera host metadata (for to_camdev conversion).
    std::vector<CameraParams> cams;

    // Device buffers (owned; null until upload).
    float*   d_images = nullptr;   // (N, H, W, 3) flat float
    CamDev*  d_cams   = nullptr;   // N × CamDev

    int img_n = 0, img_h = 0, img_w = 0;

    void free_device()
    {
        if (d_images) { cudaFree(d_images); d_images = nullptr; }
        if (d_cams)   { cudaFree(d_cams);   d_cams   = nullptr; }
    }

    ~Impl() { free_device(); }
};

Reprojector::Reprojector(int out_width, int out_height) : impl_(new Impl)
{
    impl_->out_w = out_width;
    impl_->out_h = out_height;
}

Reprojector::~Reprojector() { delete impl_; }

void Reprojector::set_cameras(const std::vector<CameraParams>& cams)
{
    impl_->cams = cams;
    // If images were already uploaded, rebuild device camera records.
    if (impl_->img_n == static_cast<int>(cams.size()) && impl_->d_images)
    {
        std::vector<CamDev> hcams;
        hcams.reserve(cams.size());
        for (auto& c : cams) hcams.push_back(to_camdev(c));
        if (impl_->d_cams) { cudaFree(impl_->d_cams); impl_->d_cams = nullptr; }
        CUDA_CHECK(cudaMalloc(&impl_->d_cams, hcams.size() * sizeof(CamDev)));
        CUDA_CHECK(cudaMemcpy(impl_->d_cams, hcams.data(),
                              hcams.size() * sizeof(CamDev), cudaMemcpyHostToDevice));
    }
}

void Reprojector::upload_images(const float* nhwc, int n, int h, int w)
{
    impl_->img_n = n;
    impl_->img_h = h;
    impl_->img_w = w;

    // Release old device buffers.
    impl_->free_device();

    // Upload image data.
    size_t img_bytes = static_cast<size_t>(n) * h * w * 3 * sizeof(float);
    CUDA_CHECK(cudaMalloc(&impl_->d_images, img_bytes));
    CUDA_CHECK(cudaMemcpy(impl_->d_images, nhwc, img_bytes, cudaMemcpyHostToDevice));

    // Build device camera records (requires set_cameras to have been called).
    if (!impl_->cams.empty())
    {
        std::vector<CamDev> hcams;
        hcams.reserve(impl_->cams.size());
        for (auto& c : impl_->cams) hcams.push_back(to_camdev(c));
        CUDA_CHECK(cudaMalloc(&impl_->d_cams, hcams.size() * sizeof(CamDev)));
        CUDA_CHECK(cudaMemcpy(impl_->d_cams, hcams.data(),
                              hcams.size() * sizeof(CamDev), cudaMemcpyHostToDevice));
    }
}

void Reprojector::upload_depth(const float*, int, int, int) {}

void Reprojector::render_bowl(const CameraParams& vcam, const BowlParams& bowl, float* out_rgba)
{
    // Carry-forward #2: enforce output-resolution contract.
    assert(vcam.width == impl_->out_w && vcam.height == impl_->out_h &&
           "render_bowl: vcam dimensions must match Reprojector constructor dims");

    int OW = impl_->out_w;
    int OH = impl_->out_h;
    int ncam = impl_->img_n;

    float* d_out = nullptr;
    CUDA_CHECK(cudaMalloc(&d_out, sizeof(float) * OW * OH * 4));

    CamDev v = to_camdev(vcam);

    launch_bowl(d_out, OW, OH, impl_->d_images, impl_->d_cams, ncam, v,
                /*surf_type=*/1, /*flat_z0=*/0.0f,
                bowl.R0, bowl.k, bowl.Rmax,
                /*feather_margin=*/30.0f,
                /*fr=*/0.0f, /*fg=*/0.0f, /*fb=*/0.0f);

    CUDA_CHECK(cudaDeviceSynchronize());
    CUDA_CHECK(cudaMemcpy(out_rgba, d_out, sizeof(float) * OW * OH * 4, cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaFree(d_out));
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
