/**
 * @file benchmark.cpp
 * @brief Standalone benchmark for micropilot::rendering::Reprojector.
 *
 * Builds a synthetic 6-camera ring rig with constant-color 128x96 images
 * entirely in-process (no fixture files), constructs a 1280x720 Reprojector,
 * uploads once, and times render_bowl over N=100 iterations.
 *
 * Warm-up: 1 call before timing.
 * render_bowl already calls cudaDeviceSynchronize() + device→host copy, so
 * each iteration measures the full realistic library-call cost.
 *
 * Output to stdout:
 *   bench_reprojector: 1280x720, 6 cams, N=100 iters
 *   warm-up done
 *   elapsed: 42.315 ms total  =>  2.362 ms/frame  =>  423.4 fps
 */

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "rendering_reprojector/reprojector.hpp"
#include "rendering_reprojector/types.hpp"

using micropilot::rendering::BowlParams;
using micropilot::rendering::CameraParams;
using micropilot::rendering::Reprojector;

// ---------------------------------------------------------------------------
// Minimal math helpers (no external deps)
// ---------------------------------------------------------------------------
static std::array<float, 3> cross3(std::array<float, 3> a, std::array<float, 3> b)
{
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
}
static float dot3(std::array<float, 3> a, std::array<float, 3> b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
static std::array<float, 3> norm3(std::array<float, 3> v)
{
    float d = std::sqrt(dot3(v, v));
    if (d < 1e-12f) return v;
    return {v[0] / d, v[1] / d, v[2] / d};
}

// ---------------------------------------------------------------------------
// Build one ring camera (CV convention: +Z fwd, +X right, +Y down)
// ---------------------------------------------------------------------------
static CameraParams make_ring_cam(float theta_rad, float radius, float mount_h,
                                  float tilt_rad, float fx, float fy, float cx,
                                  float cy, int w, int h)
{
    float ct = std::cos(theta_rad), st = std::sin(theta_rad);

    // Forward direction: outward + downward tilt (Z points up in world)
    std::array<float, 3> fwd_raw{ct * std::cos(tilt_rad), st * std::cos(tilt_rad),
                                  -std::sin(tilt_rad)};
    std::array<float, 3> fwd = norm3(fwd_raw);

    std::array<float, 3> world_up{0.f, 0.f, -1.f};  // +Z up → down for CV
    std::array<float, 3> right = norm3(cross3(world_up, fwd));
    std::array<float, 3> down = cross3(fwd, right);

    CameraParams cam{};
    cam.K[0] = fx;
    cam.K[1] = 0.f;
    cam.K[2] = cx;
    cam.K[3] = 0.f;
    cam.K[4] = fy;
    cam.K[5] = cy;
    cam.K[6] = 0.f;
    cam.K[7] = 0.f;
    cam.K[8] = 1.f;
    // R row-major: R[i,:] = [right[i], down[i], fwd[i]]
    cam.R[0] = right[0];
    cam.R[1] = down[0];
    cam.R[2] = fwd[0];
    cam.R[3] = right[1];
    cam.R[4] = down[1];
    cam.R[5] = fwd[1];
    cam.R[6] = right[2];
    cam.R[7] = down[2];
    cam.R[8] = fwd[2];
    cam.t[0] = radius * ct;
    cam.t[1] = radius * st;
    cam.t[2] = mount_h;
    cam.width = w;
    cam.height = h;
    return cam;
}

// ---------------------------------------------------------------------------
// Build virtual overhead camera (bird's-eye, looking straight down)
// ---------------------------------------------------------------------------
static CameraParams make_overhead_vcam(int out_w, int out_h)
{
    const float hfov_rad = 90.f * 3.14159265f / 180.f;
    const float vfx = (out_w / 2.f) / std::tan(hfov_rad / 2.f);
    const float vfy = vfx;
    const float vcx = out_w / 2.f, vcy = out_h / 2.f;

    CameraParams vcam{};
    vcam.K[0] = vfx;
    vcam.K[1] = 0.f;
    vcam.K[2] = vcx;
    vcam.K[3] = 0.f;
    vcam.K[4] = vfy;
    vcam.K[5] = vcy;
    vcam.K[6] = 0.f;
    vcam.K[7] = 0.f;
    vcam.K[8] = 1.f;
    // Looking straight down: right=+X, down=+Y, fwd=+Z (CV +Z pointing down)
    // In world: fwd = (0,0,-1) → pointing downward from height
    // Use identity R to keep it simple (standard bird's-eye)
    vcam.R[0] = 1.f;
    vcam.R[4] = 1.f;
    vcam.R[8] = 1.f;
    // Camera placed above origin at 5m
    vcam.t[0] = 0.f;
    vcam.t[1] = 0.f;
    vcam.t[2] = 5.f;
    vcam.width = out_w;
    vcam.height = out_h;
    return vcam;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main()
{
    const int OUT_W = 1280, OUT_H = 720;
    const int N_CAM = 6;
    const int CAM_W = 128, CAM_H = 96;
    const int N_ITERS = 100;

    std::printf("bench_reprojector: %dx%d output, %d cams (%dx%d each), N=%d iters\n",
                OUT_W, OUT_H, N_CAM, CAM_W, CAM_H, N_ITERS);
    std::fflush(stdout);

    // ---- Build 6 ring cameras -----------------------------------------------
    const float radius = 0.25f, mount_h = 0.55f;
    const float tilt_deg = 15.f;
    const float tilt_rad = tilt_deg * 3.14159265f / 180.f;
    const float hfov_rad = 85.f * 3.14159265f / 180.f;
    const float fx = (CAM_W / 2.f) / std::tan(hfov_rad / 2.f);
    const float fy = fx;
    const float cx = CAM_W / 2.f, cy = CAM_H / 2.f;

    // Distinct solid colors for each camera
    const float colors[N_CAM][3] = {{0.9f, 0.2f, 0.2f}, {0.2f, 0.8f, 0.2f},
                                     {0.2f, 0.3f, 0.9f}, {0.9f, 0.8f, 0.1f},
                                     {0.8f, 0.3f, 0.8f}, {0.2f, 0.8f, 0.8f}};

    std::vector<CameraParams> cams;
    std::vector<float> images_nhwc(N_CAM * CAM_H * CAM_W * 3, 0.f);

    for (int i = 0; i < N_CAM; ++i)
    {
        float theta = 2.f * 3.14159265f * i / N_CAM;
        cams.push_back(make_ring_cam(theta, radius, mount_h, tilt_rad, fx, fy, cx, cy,
                                     CAM_W, CAM_H));
        size_t img_start = static_cast<size_t>(i) * CAM_H * CAM_W * 3;
        for (int px = 0; px < CAM_H * CAM_W; ++px)
        {
            images_nhwc[img_start + px * 3 + 0] = colors[i][0];
            images_nhwc[img_start + px * 3 + 1] = colors[i][1];
            images_nhwc[img_start + px * 3 + 2] = colors[i][2];
        }
    }

    // ---- Build reprojector + upload (one-time cost, not benchmarked) ---------
    Reprojector rep(OUT_W, OUT_H);
    rep.set_cameras(cams);
    rep.upload_images(images_nhwc.data(), N_CAM, CAM_H, CAM_W);

    CameraParams vcam = make_overhead_vcam(OUT_W, OUT_H);
    BowlParams bowl{6.0f, 0.08f, 20.0f};
    std::vector<float> out(OUT_W * OUT_H * 4, 0.f);

    // ---- Warm-up (1 call, not timed) -----------------------------------------
    rep.render_bowl(vcam, bowl, out.data());
    std::printf("warm-up done\n");
    std::fflush(stdout);

    // ---- Timed loop -----------------------------------------------------------
    // render_bowl already calls cudaDeviceSynchronize() + device→host copy.
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < N_ITERS; ++i)
        rep.render_bowl(vcam, bowl, out.data());
    auto t1 = std::chrono::high_resolution_clock::now();

    double elapsed_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    double ms_per_frame = elapsed_ms / N_ITERS;
    double fps = 1000.0 / ms_per_frame;

    std::printf("elapsed: %.3f ms total  =>  %.3f ms/frame  =>  %.1f fps\n",
                elapsed_ms, ms_per_frame, fps);
    std::fflush(stdout);
    return 0;
}
