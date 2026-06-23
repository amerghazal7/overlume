/** @file reprojector.cpp @brief Host orchestration + device buffer management. */

#include "rendering_reprojector/reprojector.hpp"

#include <cuda_runtime.h>

#include <cassert>
#include <cmath>
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

    // Point cloud (host) built during upload_depth.
    std::vector<float> h_pts;   // (M, 3) xyz flat
    std::vector<float> h_cols;  // (M, 3) rgb flat

    // Device point cloud buffers (owned; null until upload_depth).
    float* d_pts  = nullptr;  // (M, 3)
    float* d_cols = nullptr;  // (M, 3)
    int    npts   = 0;

    void free_device()
    {
        if (d_images) { cudaFree(d_images); d_images = nullptr; }
        if (d_cams)   { cudaFree(d_cams);   d_cams   = nullptr; }
    }

    void free_depth_device()
    {
        if (d_pts)  { cudaFree(d_pts);  d_pts  = nullptr; }
        if (d_cols) { cudaFree(d_cols); d_cols = nullptr; }
        npts = 0;
    }

    ~Impl()
    {
        free_device();
        free_depth_device();
    }
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

void Reprojector::upload_depth(const float* nhw, int n, int h, int w)
{
    // Build host point cloud exactly mirroring DepthRenderer._point_cloud.
    // For each camera, each finite-depth pixel: back-project (u,v,depth) to world.
    impl_->h_pts.clear();
    impl_->h_cols.clear();

    const float* images = nullptr;
    // h_images is not kept on host; we need it for colors.
    // If d_images is uploaded, we need image data — keep host copy.
    // Fallback: we stored img_n/img_h/img_w; colors come from d_images copy.
    // We must download d_images to get colors since we don't keep a host copy.
    // Download once for the point cloud build.
    int img_n = impl_->img_n;
    int img_h = impl_->img_h;
    int img_w = impl_->img_w;

    std::vector<float> host_images;
    if (impl_->d_images && img_n > 0)
    {
        size_t sz = static_cast<size_t>(img_n) * img_h * img_w * 3;
        host_images.resize(sz);
        CUDA_CHECK(cudaMemcpy(host_images.data(), impl_->d_images, sz * sizeof(float),
                              cudaMemcpyDeviceToHost));
        images = host_images.data();
    }

    int ncam = static_cast<int>(impl_->cams.size());
    for (int ci = 0; ci < n && ci < ncam; ++ci)
    {
        const CameraParams& cam = impl_->cams[ci];
        float fx = cam.K[0], fy = cam.K[4], cx_k = cam.K[2], cy_k = cam.K[5];
        // R columns (row-major R): right=(R[0],R[3],R[6]), down=(R[1],R[4],R[7]),
        // fwd=(R[2],R[5],R[8]), t=center
        float rx = cam.R[0], ry = cam.R[3], rz = cam.R[6];  // right col
        float dx = cam.R[1], dy2 = cam.R[4], dz = cam.R[7];  // down col
        float fx2 = cam.R[2], fy2 = cam.R[5], fz = cam.R[8]; // fwd col
        float tx = cam.t[0], ty = cam.t[1], tz = cam.t[2];

        int dh = h, dw = w;
        const float* depth_plane = nhw + static_cast<size_t>(ci) * dh * dw;
        const float* img_plane = nullptr;
        int ih = img_h, iw = img_w;
        if (images) img_plane = images + static_cast<size_t>(ci) * ih * iw * 3;

        for (int v = 0; v < dh; ++v)
        {
            for (int u = 0; u < dw; ++u)
            {
                float depth = depth_plane[v * dw + u];
                if (!std::isfinite(depth)) continue;

                // Back-project: x=(u-cx)/fx*z; y=(v-cy)/fy*z
                float xc = (u - cx_k) / fx * depth;
                float yc = (v - cy_k) / fy * depth;
                float zc = depth;

                // world = R @ [xc, yc, zc] + t
                float wx = rx * xc + dx * yc + fx2 * zc + tx;
                float wy = ry * xc + dy2 * yc + fy2 * zc + ty;
                float wz = rz * xc + dz * yc + fz * zc + tz;

                impl_->h_pts.push_back(wx);
                impl_->h_pts.push_back(wy);
                impl_->h_pts.push_back(wz);

                // Color from image at same (u,v) — use cam image dims
                if (img_plane && u < iw && v < ih)
                {
                    const float* px = img_plane + (v * iw + u) * 3;
                    impl_->h_cols.push_back(px[0]);
                    impl_->h_cols.push_back(px[1]);
                    impl_->h_cols.push_back(px[2]);
                }
                else
                {
                    impl_->h_cols.push_back(0.0f);
                    impl_->h_cols.push_back(0.0f);
                    impl_->h_cols.push_back(0.0f);
                }
            }
        }
    }

    // Upload point cloud to device.
    impl_->free_depth_device();
    impl_->npts = static_cast<int>(impl_->h_pts.size() / 3);
    if (impl_->npts > 0)
    {
        size_t pts_bytes  = static_cast<size_t>(impl_->npts) * 3 * sizeof(float);
        size_t cols_bytes = static_cast<size_t>(impl_->npts) * 3 * sizeof(float);
        CUDA_CHECK(cudaMalloc(&impl_->d_pts,  pts_bytes));
        CUDA_CHECK(cudaMalloc(&impl_->d_cols, cols_bytes));
        CUDA_CHECK(cudaMemcpy(impl_->d_pts,  impl_->h_pts.data(),  pts_bytes,
                              cudaMemcpyHostToDevice));
        CUDA_CHECK(cudaMemcpy(impl_->d_cols, impl_->h_cols.data(), cols_bytes,
                              cudaMemcpyHostToDevice));
    }
}

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

void Reprojector::render_depth(const CameraParams& vcam, int splat_radius, float* out_rgba)
{
    assert(vcam.width == impl_->out_w && vcam.height == impl_->out_h &&
           "render_depth: vcam dimensions must match Reprojector constructor dims");

    int OW = impl_->out_w;
    int OH = impl_->out_h;
    int npts = impl_->npts;

    float* d_out = nullptr;
    unsigned long long* d_zbuf = nullptr;
    CUDA_CHECK(cudaMalloc(&d_out,  sizeof(float) * OW * OH * 4));
    CUDA_CHECK(cudaMalloc(&d_zbuf, sizeof(unsigned long long) * OW * OH));

    CamDev v = to_camdev(vcam);

    launch_splat(d_zbuf, d_out, OW, OH,
                 impl_->d_pts, impl_->d_cols, npts, v, splat_radius,
                 /*fr=*/0.0f, /*fg=*/0.0f, /*fb=*/0.0f);

    CUDA_CHECK(cudaDeviceSynchronize());
    CUDA_CHECK(cudaMemcpy(out_rgba, d_out, sizeof(float) * OW * OH * 4, cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaFree(d_out));
    CUDA_CHECK(cudaFree(d_zbuf));
}

void Reprojector::render_hybrid(const CameraParams&, const BowlParams&, int, float* out_rgba)
{
    std::memset(out_rgba, 0, sizeof(float) * impl_->out_w * impl_->out_h * 4);
}
}  // namespace micropilot::rendering
