#include <gtest/gtest.h>

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
    cam.K[0] = 4.0f; cam.K[4] = 4.0f; cam.K[2] = 2.0f; cam.K[5] = 2.0f; cam.K[8] = 1.0f;
    // R = identity (row-major).
    cam.R[0] = 1.0f; cam.R[4] = 1.0f; cam.R[8] = 1.0f;
    // Camera center slightly above the bowl, looking down.
    cam.t[0] = 0.0f; cam.t[1] = 0.0f; cam.t[2] = 3.0f;
    cam.width = 4; cam.height = 4;

    // Upload a plain red image (4x4x3).
    std::vector<float> img(1 * 4 * 4 * 3, 0.0f);
    for (int i = 0; i < 4 * 4; ++i) img[i * 3] = 1.0f;  // R=1, G=0, B=0

    r.set_cameras({cam});
    r.upload_images(img.data(), 1, 4, 4);

    CameraParams vcam{};
    vcam.K[0] = 4.0f; vcam.K[4] = 4.0f; vcam.K[2] = 4.0f; vcam.K[5] = 3.0f; vcam.K[8] = 1.0f;
    vcam.R[0] = 1.0f; vcam.R[4] = 1.0f; vcam.R[8] = 1.0f;
    vcam.t[0] = 0.0f; vcam.t[1] = 0.0f; vcam.t[2] = 3.0f;
    vcam.width = W; vcam.height = H;

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
