// test_bowl_projection.cpp — VM-091 (unified-engine migration Task 2),
// Steps 0-2: bowl_projection.hpp's portable pinhole + plumb_bob projection
// and bowl-surface math. No Filament/GPU involved -- these link against
// visual_renderer but only exercise the Filament-free bowl_projection.cpp
// translation unit.
#include "bowl_projection.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace {

using mpviz::CameraExtrinsics;
using mpviz::CameraIntrinsics;
using mpviz::Vec3;
namespace bowl = mpviz::bowl;

constexpr CameraExtrinsics kIdentityExt{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}};

}  // namespace

// ---- Step 0 ----------------------------------------------------------

TEST(BowlProjection, PinholeCenterPointProjectsToImageCenter) {
    CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};  // zero distortion
    float u, v;
    // A point straight ahead on the camera's optical axis, at some depth,
    // must project to the principal point (cx,cy) normalized -> (0.5,0.5).
    ASSERT_TRUE(bowl::ProjectToCameraUv(kIdentityExt, in, 320, 240, {0, 0, 5}, &u, &v));
    EXPECT_NEAR(u, 0.5f, 1e-3f);
    EXPECT_NEAR(v, 0.5f, 1e-3f);
}

TEST(BowlProjection, PointBehindCameraFailsProjection) {
    CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    float u, v;
    EXPECT_FALSE(bowl::ProjectToCameraUv(kIdentityExt, in, 320, 240, {0, 0, -5}, &u, &v));
}

TEST(BowlProjection, PointOutsidePixelBoundsFailsProjection) {
    // Far off-axis in x at a shallow depth -> lands outside [0, width-1].
    CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    float u, v;
    EXPECT_FALSE(bowl::ProjectToCameraUv(kIdentityExt, in, 320, 240, {50, 0, 1}, &u, &v));
}

// ---- Step 1: nonzero plumb_bob distortion ----------------------------
//
// Neither default_params.yaml nor m2o1_params.yaml carries a `dist` row
// (intrinsics/distortion are not ROS parameters at all -- Global
// Constraints), and this epic's own Decision resolutions record that the
// real fixture bag's six cameras carry ZERO distortion coefficients. Per
// Task 2 Step 1's option (b): a hand-written, clearly-labeled `dist` row,
// NOT derived from any file this repo ships, chosen as a plausible small
// plumb_bob distortion (k1/k2 only, zero p1/p2/k3). The expected UV is
// computed independently here via the plain plumb_bob formula, not by
// calling the function under test and checking self-agreement.
TEST(BowlProjection, NonzeroRadialDistortionMatchesIndependentPlumbBobComputation) {
    const CameraIntrinsics in{400, 400, 160, 120, {-0.12, 0.02, 0, 0, 0}};  // k1, k2 only
    const Vec3 p{0.3, 0.1, 5.0};  // off-axis rig-frame point, camera at rig origin

    // Independent reference computation (same formula, computed by hand
    // here rather than delegated to the function under test).
    const double xn0 = p.x / p.z;  // 0.06
    const double yn0 = p.y / p.z;  // 0.02
    const double r2 = xn0 * xn0 + yn0 * yn0;
    const double radial = 1.0 + r2 * (in.dist[0] + r2 * in.dist[1]);
    const double xd = xn0 * radial;
    const double yd = yn0 * radial;
    const double expected_u = (in.fx * xd + in.cx) / 320.0;
    const double expected_v = (in.fy * yd + in.cy) / 240.0;

    float u, v;
    ASSERT_TRUE(bowl::ProjectToCameraUv(kIdentityExt, in, 320, 240, p, &u, &v));
    EXPECT_NEAR(u, expected_u, 1e-6);
    EXPECT_NEAR(v, expected_v, 1e-6);
    // Sanity: distortion actually moved the point off the pure-pinhole UV.
    EXPECT_GT(std::abs(u - (in.fx * xn0 + in.cx) / 320.0), 1e-5);
}

TEST(BowlProjection, ExtremeOffAxisPointRejectedByCalibratedFieldGuard) {
    // r2 > 3.0 (>~60 deg off-axis) -- the same guard reproject.cu's
    // bowl_kernel applies before trusting the plumb_bob polynomial.
    const CameraIntrinsics in{400, 400, 160, 120, {0.01, 0, 0, 0, 0}};
    float u, v;
    EXPECT_FALSE(bowl::ProjectToCameraUv(kIdentityExt, in, 320, 240, {5, 5, 1}, &u, &v));
}

// ---- Step 2: BowlSurfacePoint boundary conditions --------------------

TEST(BowlProjection, SurfacePointAtR0IsFlatFloor) {
    // r == R0 -> d == 0 -> z == 0, regardless of k, matching the flat-floor
    // region inside R0 (types.hpp/surface.cuh's own documented shape).
    const Vec3 p = bowl::BowlSurfacePoint(/*R0=*/6.0, /*k=*/0.2, /*Rmax=*/22.0, /*theta=*/0.7,
                                          /*r=*/6.0);
    EXPECT_NEAR(p.z, 0.0, 1e-9);
    EXPECT_NEAR(p.x, 6.0 * std::cos(0.7), 1e-9);
    EXPECT_NEAR(p.y, 6.0 * std::sin(0.7), 1e-9);
}

TEST(BowlProjection, SurfacePointAtRmaxMatchesWallRiseFormula) {
    const double R0 = 6.0, k = 0.2, Rmax = 22.0;
    const Vec3 p = bowl::BowlSurfacePoint(R0, k, Rmax, /*theta=*/1.2, /*r=*/Rmax);
    const double expected_z = k * (Rmax - R0) * (Rmax - R0);
    EXPECT_NEAR(p.z, expected_z, 1e-9);
}

TEST(BowlProjection, SurfacePointBeyondRmaxClampsToWallTop) {
    // r > Rmax must clamp to the same height as r == Rmax (the "flat again,
    // capped" region bowl_height's own clamp documents).
    const double R0 = 6.0, k = 0.2, Rmax = 22.0;
    const Vec3 at_rmax = bowl::BowlSurfacePoint(R0, k, Rmax, 0.0, Rmax);
    const Vec3 beyond = bowl::BowlSurfacePoint(R0, k, Rmax, 0.0, Rmax + 10.0);
    EXPECT_NEAR(at_rmax.z, beyond.z, 1e-9);
}
