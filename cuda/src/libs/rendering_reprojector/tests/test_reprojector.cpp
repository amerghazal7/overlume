#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <vector>

#include "rendering_reprojector/reprojector.hpp"

using micropilot::rendering::BowlParams;
using micropilot::rendering::CameraParams;
using micropilot::rendering::Reprojector;

// ---------------------------------------------------------------------------
// Smoke test (carry-forward #3 from Task-2 review):
// The fill-constant assertions were removed because render_bowl now runs the
// real reprojection kernel which no longer writes (0.1, 0.2, 0.3, 1.0).
// This test verifies:
//   1. The output buffer is fully written (no uninitialised -1.0 values remain).
//   2. All pixels are finite.
//   3. Alpha (validity) is 0.0 or 1.0 for every pixel.
// The rigorous correctness gate (PSNR > 40 dB vs NumpyRenderer) lives in the
// Python parity test: tests/test_cuda_parity.py::test_cuda_bowl_matches_numpy.
// ---------------------------------------------------------------------------
TEST(Reprojector, ConstructsAndRendersBufferOfCorrectSize)
{
    const int W = 8, H = 6;
    Reprojector r(W, H);

    // One trivial camera: identity intrinsics/extrinsics, small image (4x4).
    CameraParams cam{};
    cam.K[0] = 4.0f;
    cam.K[4] = 4.0f;
    cam.K[2] = 2.0f;
    cam.K[5] = 2.0f;
    cam.K[8] = 1.0f;
    // R = identity (row-major).
    cam.R[0] = 1.0f;
    cam.R[4] = 1.0f;
    cam.R[8] = 1.0f;
    // Camera center slightly above the bowl, looking down.
    cam.t[0] = 0.0f;
    cam.t[1] = 0.0f;
    cam.t[2] = 3.0f;
    cam.width = 4;
    cam.height = 4;

    // Upload a plain red image (4x4x3).
    std::vector<float> img(1 * 4 * 4 * 3, 0.0f);
    for (int i = 0; i < 4 * 4; ++i) img[i * 3] = 1.0f;  // R=1, G=0, B=0

    r.set_cameras({cam});
    r.upload_images(img.data(), 1, 4, 4);

    CameraParams vcam{};
    vcam.K[0] = 4.0f;
    vcam.K[4] = 4.0f;
    vcam.K[2] = 4.0f;
    vcam.K[5] = 3.0f;
    vcam.K[8] = 1.0f;
    vcam.R[0] = 1.0f;
    vcam.R[4] = 1.0f;
    vcam.R[8] = 1.0f;
    vcam.t[0] = 0.0f;
    vcam.t[1] = 0.0f;
    vcam.t[2] = 3.0f;
    vcam.width = W;
    vcam.height = H;

    BowlParams bowl{6.0f, 0.08f, 20.0f};
    std::vector<float> out(W * H * 4, -1.0f);  // sentinel: -1 = uninitialised
    r.render_bowl(vcam, bowl, out.data());

    for (int i = 0; i < W * H; ++i)
    {
        // Buffer must have been fully written (no sentinel -1 remains).
        EXPECT_NE(out[i * 4 + 0], -1.0f) << "pixel " << i << " R not written";
        EXPECT_NE(out[i * 4 + 3], -1.0f) << "pixel " << i << " A not written";
        // All values must be finite.
        EXPECT_TRUE(std::isfinite(out[i * 4 + 0])) << "pixel " << i << " R not finite";
        EXPECT_TRUE(std::isfinite(out[i * 4 + 1])) << "pixel " << i << " G not finite";
        EXPECT_TRUE(std::isfinite(out[i * 4 + 2])) << "pixel " << i << " B not finite";
        EXPECT_TRUE(std::isfinite(out[i * 4 + 3])) << "pixel " << i << " A not finite";
        // Alpha must be 0 (invalid) or 1 (valid).
        float a = out[i * 4 + 3];
        EXPECT_TRUE(a == 0.0f || a == 1.0f) << "pixel " << i << " alpha=" << a;
    }
}

// ---------------------------------------------------------------------------
// Regression performance floor (720p, 20 iters).
// render_bowl includes cudaDeviceSynchronize() + device→host copy — this is
// the full realistic per-call cost.  The floor is set conservatively at 200
// fps; a Debug build on an RTX 3090 should comfortably exceed this.
// ---------------------------------------------------------------------------
static std::array<float, 3> bench_cross3(std::array<float, 3> a, std::array<float, 3> b)
{
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
static float bench_dot3(std::array<float, 3> a, std::array<float, 3> b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
static std::array<float, 3> bench_norm3(std::array<float, 3> v)
{
    float d = std::sqrt(bench_dot3(v, v));
    if (d < 1e-12f) return v;
    return {v[0] / d, v[1] / d, v[2] / d};
}

static CameraParams make_bench_ring_cam(float theta_rad)
{
    const float radius = 0.25f, mount_h = 0.55f;
    const float tilt_rad = 15.f * 3.14159265f / 180.f;
    const int CAM_W = 128, CAM_H = 96;
    const float hfov_rad = 85.f * 3.14159265f / 180.f;
    const float fx = (CAM_W / 2.f) / std::tan(hfov_rad / 2.f);
    const float fy = fx;
    const float cx = CAM_W / 2.f, cy = CAM_H / 2.f;

    float ct = std::cos(theta_rad), st = std::sin(theta_rad);
    std::array<float, 3> fwd_raw{ct * std::cos(tilt_rad), st * std::cos(tilt_rad),
                                 -std::sin(tilt_rad)};
    std::array<float, 3> fwd = bench_norm3(fwd_raw);
    std::array<float, 3> world_up{0.f, 0.f, -1.f};
    std::array<float, 3> right = bench_norm3(bench_cross3(world_up, fwd));
    std::array<float, 3> down = bench_cross3(fwd, right);

    CameraParams cam{};
    cam.K[0] = fx;
    cam.K[2] = cx;
    cam.K[4] = fy;
    cam.K[5] = cy;
    cam.K[8] = 1.f;
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
    cam.width = CAM_W;
    cam.height = CAM_H;
    return cam;
}

TEST(Reprojector, BowlRenderFloor720p)
{
    const int OUT_W = 1280, OUT_H = 720;
    const int N_CAM = 6;
    const int CAM_W = 128, CAM_H = 96;
    const int N_ITERS = 20;

    std::vector<CameraParams> cams;
    std::vector<float> images_nhwc(N_CAM * CAM_H * CAM_W * 3, 0.f);

    const float colors[N_CAM][3] = {{0.9f, 0.2f, 0.2f}, {0.2f, 0.8f, 0.2f}, {0.2f, 0.3f, 0.9f},
                                    {0.9f, 0.8f, 0.1f}, {0.8f, 0.3f, 0.8f}, {0.2f, 0.8f, 0.8f}};

    for (int i = 0; i < N_CAM; ++i)
    {
        float theta = 2.f * 3.14159265f * i / N_CAM;
        cams.push_back(make_bench_ring_cam(theta));
        size_t img_start = static_cast<size_t>(i) * CAM_H * CAM_W * 3;
        for (int px = 0; px < CAM_H * CAM_W; ++px)
        {
            images_nhwc[img_start + px * 3 + 0] = colors[i][0];
            images_nhwc[img_start + px * 3 + 1] = colors[i][1];
            images_nhwc[img_start + px * 3 + 2] = colors[i][2];
        }
    }

    Reprojector rep(OUT_W, OUT_H);
    rep.set_cameras(cams);
    rep.upload_images(images_nhwc.data(), N_CAM, CAM_H, CAM_W);

    CameraParams vcam{};
    const float hfov_rad = 90.f * 3.14159265f / 180.f;
    const float vfx = (OUT_W / 2.f) / std::tan(hfov_rad / 2.f);
    vcam.K[0] = vfx;
    vcam.K[2] = OUT_W / 2.f;
    vcam.K[4] = vfx;
    vcam.K[5] = OUT_H / 2.f;
    vcam.K[8] = 1.f;
    vcam.R[0] = 1.f;
    vcam.R[4] = 1.f;
    vcam.R[8] = 1.f;
    vcam.t[0] = 0.f;
    vcam.t[1] = 0.f;
    vcam.t[2] = 5.f;
    vcam.width = OUT_W;
    vcam.height = OUT_H;

    BowlParams bowl{6.0f, 0.08f, 20.0f};
    std::vector<float> out(OUT_W * OUT_H * 4, 0.f);

    // Warm-up
    rep.render_bowl(vcam, bowl, out.data());

    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < N_ITERS; ++i) rep.render_bowl(vcam, bowl, out.data());
    auto t1 = std::chrono::high_resolution_clock::now();

    double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    double fps = 1000.0 / (elapsed_ms / N_ITERS);

    std::printf("[BowlRenderFloor720p] %.3f ms/frame  =>  %.1f fps (Debug, %d iters)\n",
                elapsed_ms / N_ITERS, fps, N_ITERS);

    EXPECT_GT(fps, 200.0) << "fps " << fps
                          << " < 200 fps floor. Check GPU availability or kernel regression.";
}
