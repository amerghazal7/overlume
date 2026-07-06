/** @file reprojector.cpp @brief Host orchestration + device buffer management.
 *
 *  Device buffers are PERSISTENT and grow-only: they are (re)allocated only when
 *  a larger size is needed, never per frame. Per-frame cudaMalloc/cudaFree are
 *  device-serializing calls — when this node shares a GPU with the CARLA server,
 *  churning the ~96 MB image buffer every frame stalled the simulator's render
 *  pipeline (server 30->8 fps, cameras 10->3). Allocate once, reuse forever.
 */

#include "rendering_reprojector/reprojector.hpp"

#include <cuda_runtime.h>

#include <cmath>
#include <cstddef>
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
    bool cams_dirty = false;  // host cams changed → re-upload d_cams before next render

    // ── Persistent device buffers (grow-only; freed only in dtor) ────────────
    // Each tracks its allocated capacity in BYTES; ensure_bytes() reallocates
    // only when a larger size is requested.
    float*  d_images = nullptr;  size_t d_images_cap = 0;
    CamDev* d_cams   = nullptr;  size_t d_cams_cap   = 0;
    float*  d_out    = nullptr;  size_t d_out_cap    = 0;  // shared RGBA output (OW*OH*4)

    // Depth / hybrid scratch (fixed OW*OH; allocated once on first use).
    float*              d_bowl  = nullptr;  size_t d_bowl_cap  = 0;
    float*              d_depth = nullptr;  size_t d_depth_cap = 0;
    unsigned long long* d_zbuf  = nullptr;  size_t d_zbuf_cap  = 0;

    int img_n = 0, img_h = 0, img_w = 0;

    // Point cloud (host) built during upload_depth.
    std::vector<float> h_pts;   // (M, 3) xyz flat
    std::vector<float> h_cols;  // (M, 3) rgb flat

    // Device point cloud buffers (persistent, grow-only).
    float* d_pts  = nullptr;  size_t d_pts_cap  = 0;
    float* d_cols = nullptr;  size_t d_cols_cap = 0;
    int    npts   = 0;

    // Robot proxy mesh (persistent, uploaded once; n_rtris == 0 -> no robot).
    float* d_rverts = nullptr;  size_t d_rverts_cap = 0;
    float* d_rcols  = nullptr;  size_t d_rcols_cap  = 0;
    int    n_rtris  = 0;

    // Cumulative device (re)allocation count — exposed for diagnostics/tests.
    size_t alloc_count = 0;

    // Grow-only device allocation: (re)allocate *ptr only when need > cap.
    // Frees the old buffer first so capacity can shrink-then-grow safely.
    void ensure_bytes(void** ptr, size_t& cap, size_t need)
    {
        if (need <= cap) return;
        if (*ptr) cudaFree(*ptr);
        *ptr = nullptr;
        CUDA_CHECK(cudaMalloc(ptr, need));
        cap = need;
        ++alloc_count;
    }

    // Upload device camera records iff the host cams changed since last upload.
    // d_cams persists; only a small (ncam × CamDev) memcpy runs per dirty frame.
    void sync_cams()
    {
        if (cams.empty()) return;
        ensure_bytes(reinterpret_cast<void**>(&d_cams), d_cams_cap, cams.size() * sizeof(CamDev));
        if (cams_dirty)
        {
            std::vector<CamDev> hcams;
            hcams.reserve(cams.size());
            for (auto& c : cams) hcams.push_back(to_camdev(c));
            CUDA_CHECK(cudaMemcpy(d_cams, hcams.data(), hcams.size() * sizeof(CamDev),
                                  cudaMemcpyHostToDevice));
            cams_dirty = false;
        }
    }

    // Rasterize the robot proxy and overlay it on the environment already in
    // d_out. Reuses d_zbuf + d_bowl as scratch (both free at this point in
    // every render path: bowl/depth don't use them afterwards, hybrid has
    // already consumed them into d_out). No-op without a mesh.
    void composite_robot(const CamDev& v)
    {
        if (n_rtris == 0) return;
        int npx = out_w * out_h;
        ensure_bytes(reinterpret_cast<void**>(&d_zbuf), d_zbuf_cap,
                     sizeof(unsigned long long) * npx);
        ensure_bytes(reinterpret_cast<void**>(&d_bowl), d_bowl_cap,
                     sizeof(float) * npx * 4);
        launch_robot(d_zbuf, d_bowl, out_w, out_h, d_rverts, d_rcols, n_rtris, v);
        // robot-where-valid else env: exactly launch_composite's contract
        launch_composite(d_bowl, d_out, d_out, npx);
    }

    void free_all()
    {
        if (d_images) cudaFree(d_images);
        if (d_cams)   cudaFree(d_cams);
        if (d_out)    cudaFree(d_out);
        if (d_bowl)   cudaFree(d_bowl);
        if (d_depth)  cudaFree(d_depth);
        if (d_zbuf)   cudaFree(d_zbuf);
        if (d_pts)    cudaFree(d_pts);
        if (d_cols)   cudaFree(d_cols);
        if (d_rverts) cudaFree(d_rverts);
        if (d_rcols)  cudaFree(d_rcols);
    }

    ~Impl() { free_all(); }
};

Reprojector::Reprojector(int out_width, int out_height) : impl_(new Impl)
{
    impl_->out_w = out_width;
    impl_->out_h = out_height;
}

Reprojector::~Reprojector() { delete impl_; }

std::size_t Reprojector::device_alloc_count() const { return impl_->alloc_count; }

void Reprojector::set_cameras(const std::vector<CameraParams>& cams)
{
    // Store host params and mark dirty; the (cheap) device upload is deferred to
    // the next render via sync_cams(). No per-call cudaMalloc/cudaFree.
    impl_->cams = cams;
    impl_->cams_dirty = true;
}

void Reprojector::upload_images(const float* nhwc, int n, int h, int w)
{
    impl_->img_n = n;
    impl_->img_h = h;
    impl_->img_w = w;

    size_t img_bytes = static_cast<size_t>(n) * h * w * 3 * sizeof(float);
    impl_->ensure_bytes(reinterpret_cast<void**>(&impl_->d_images), impl_->d_images_cap, img_bytes);
    CUDA_CHECK(cudaMemcpy(impl_->d_images, nhwc, img_bytes, cudaMemcpyHostToDevice));
}

void Reprojector::upload_depth(const float* nhw, int n, int h, int w)
{
    // Build host point cloud exactly mirroring DepthRenderer._point_cloud.
    // For each camera, each finite-depth pixel: back-project (u,v,depth) to world.
    impl_->h_pts.clear();
    impl_->h_cols.clear();

    const float* images = nullptr;
    // d_images is not mirrored on host; download once to source point colors.
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

    // Upload point cloud to (persistent, grow-only) device buffers.
    impl_->npts = static_cast<int>(impl_->h_pts.size() / 3);
    if (impl_->npts > 0)
    {
        size_t pts_bytes  = static_cast<size_t>(impl_->npts) * 3 * sizeof(float);
        size_t cols_bytes = static_cast<size_t>(impl_->npts) * 3 * sizeof(float);
        impl_->ensure_bytes(reinterpret_cast<void**>(&impl_->d_pts),  impl_->d_pts_cap,  pts_bytes);
        impl_->ensure_bytes(reinterpret_cast<void**>(&impl_->d_cols), impl_->d_cols_cap, cols_bytes);
        CUDA_CHECK(cudaMemcpy(impl_->d_pts,  impl_->h_pts.data(),  pts_bytes,
                              cudaMemcpyHostToDevice));
        CUDA_CHECK(cudaMemcpy(impl_->d_cols, impl_->h_cols.data(), cols_bytes,
                              cudaMemcpyHostToDevice));
    }
}

void Reprojector::upload_robot_mesh(const float* verts, const float* cols, std::size_t n_tris)
{
    impl_->n_rtris = static_cast<int>(n_tris);
    if (n_tris == 0) return;
    size_t vb = n_tris * 9 * sizeof(float);
    size_t cb = n_tris * 3 * sizeof(float);
    impl_->ensure_bytes(reinterpret_cast<void**>(&impl_->d_rverts), impl_->d_rverts_cap, vb);
    impl_->ensure_bytes(reinterpret_cast<void**>(&impl_->d_rcols), impl_->d_rcols_cap, cb);
    CUDA_CHECK(cudaMemcpy(impl_->d_rverts, verts, vb, cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(impl_->d_rcols, cols, cb, cudaMemcpyHostToDevice));
}

void Reprojector::render_bowl(const CameraParams& vcam, const BowlParams& bowl, float* out_rgba)
{
    // Output-resolution contract: enforced in all build configs (Release + Debug).
    if (vcam.width != impl_->out_w || vcam.height != impl_->out_h)
        throw std::invalid_argument(
            "render_bowl: vcam dimensions must match Reprojector constructor dims");

    int OW = impl_->out_w;
    int OH = impl_->out_h;
    int ncam = impl_->img_n;

    impl_->sync_cams();
    impl_->ensure_bytes(reinterpret_cast<void**>(&impl_->d_out), impl_->d_out_cap,
                        sizeof(float) * OW * OH * 4);

    CamDev v = to_camdev(vcam);

    launch_bowl(impl_->d_out, OW, OH, impl_->d_images, impl_->d_cams, ncam, v,
                /*surf_type=*/1, /*flat_z0=*/0.0f,
                bowl.R0, bowl.k, bowl.Rmax,
                bowl.feather_margin,
                /*fr=*/0.0f, /*fg=*/0.0f, /*fb=*/0.0f);
    CUDA_CHECK(cudaGetLastError());  // surface launch-config errors immediately

    impl_->composite_robot(v);
    CUDA_CHECK(cudaGetLastError());

    // Synchronous D2H copy on the default stream already orders after (and waits
    // for) the kernel — no separate cudaDeviceSynchronize() needed.
    CUDA_CHECK(cudaMemcpy(out_rgba, impl_->d_out, sizeof(float) * OW * OH * 4,
                          cudaMemcpyDeviceToHost));
}

void Reprojector::render_depth(const CameraParams& vcam, int splat_radius, float* out_rgba)
{
    if (vcam.width != impl_->out_w || vcam.height != impl_->out_h)
        throw std::invalid_argument(
            "render_depth: vcam dimensions must match Reprojector constructor dims");

    int OW = impl_->out_w;
    int OH = impl_->out_h;
    int npts = impl_->npts;

    impl_->ensure_bytes(reinterpret_cast<void**>(&impl_->d_out), impl_->d_out_cap,
                        sizeof(float) * OW * OH * 4);
    impl_->ensure_bytes(reinterpret_cast<void**>(&impl_->d_zbuf), impl_->d_zbuf_cap,
                        sizeof(unsigned long long) * OW * OH);

    CamDev v = to_camdev(vcam);

    launch_splat(impl_->d_zbuf, impl_->d_out, OW, OH,
                 impl_->d_pts, impl_->d_cols, npts, v, splat_radius,
                 /*fr=*/0.0f, /*fg=*/0.0f, /*fb=*/0.0f);
    CUDA_CHECK(cudaGetLastError());

    impl_->composite_robot(v);
    CUDA_CHECK(cudaGetLastError());

    CUDA_CHECK(cudaMemcpy(out_rgba, impl_->d_out, sizeof(float) * OW * OH * 4,
                          cudaMemcpyDeviceToHost));
}

void Reprojector::render_hybrid(const CameraParams& vcam, const BowlParams& bowl,
                                int splat_radius, float* out_rgba)
{
    if (vcam.width != impl_->out_w || vcam.height != impl_->out_h)
        throw std::invalid_argument(
            "render_hybrid: vcam dimensions must match Reprojector constructor dims");

    int OW   = impl_->out_w;
    int OH   = impl_->out_h;
    int ncam = impl_->img_n;
    int npts = impl_->npts;
    int npx  = OW * OH;

    impl_->sync_cams();
    impl_->ensure_bytes(reinterpret_cast<void**>(&impl_->d_bowl),  impl_->d_bowl_cap,
                        sizeof(float) * npx * 4);
    impl_->ensure_bytes(reinterpret_cast<void**>(&impl_->d_depth), impl_->d_depth_cap,
                        sizeof(float) * npx * 4);
    impl_->ensure_bytes(reinterpret_cast<void**>(&impl_->d_out),   impl_->d_out_cap,
                        sizeof(float) * npx * 4);
    impl_->ensure_bytes(reinterpret_cast<void**>(&impl_->d_zbuf),  impl_->d_zbuf_cap,
                        sizeof(unsigned long long) * npx);

    CamDev v = to_camdev(vcam);

    // --- Bowl pass ---
    launch_bowl(impl_->d_bowl, OW, OH, impl_->d_images, impl_->d_cams, ncam, v,
                /*surf_type=*/1, /*flat_z0=*/0.0f,
                bowl.R0, bowl.k, bowl.Rmax,
                bowl.feather_margin,
                /*fr=*/0.0f, /*fg=*/0.0f, /*fb=*/0.0f);

    // --- Depth (splat) pass ---
    launch_splat(impl_->d_zbuf, impl_->d_depth, OW, OH,
                 impl_->d_pts, impl_->d_cols, npts, v, splat_radius,
                 /*fr=*/0.0f, /*fg=*/0.0f, /*fb=*/0.0f);

    // --- Composite: depth-where-valid else bowl ---
    launch_composite(impl_->d_depth, impl_->d_bowl, impl_->d_out, npx);
    CUDA_CHECK(cudaGetLastError());

    impl_->composite_robot(v);
    CUDA_CHECK(cudaGetLastError());

    CUDA_CHECK(cudaMemcpy(out_rgba, impl_->d_out, sizeof(float) * npx * 4,
                          cudaMemcpyDeviceToHost));
}
}  // namespace micropilot::rendering
