// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "bowl_projection.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace {

using overlume::CameraExtrinsics;
using overlume::CameraIntrinsics;
using overlume::Vec3;
namespace bowl = overlume::bowl;

constexpr CameraExtrinsics kIdentityExt{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}};

}  // namespace

TEST(BowlProjection, PinholeCenterPointProjectsToImageCenter) {
    CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    float u, v;
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
    CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    float u, v;
    EXPECT_FALSE(bowl::ProjectToCameraUv(kIdentityExt, in, 320, 240, {50, 0, 1}, &u, &v));
}

TEST(BowlProjection, NonzeroRadialDistortionMatchesIndependentPlumbBobComputation) {
    const CameraIntrinsics in{400, 400, 160, 120, {-0.12, 0.02, 0, 0, 0}};
    const Vec3 p{0.3, 0.1, 5.0};

    const double xn0 = p.x / p.z;
    const double yn0 = p.y / p.z;
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
    EXPECT_GT(std::abs(u - (in.fx * xn0 + in.cx) / 320.0), 1e-5);
}

TEST(BowlProjection, ExtremeOffAxisPointRejectedByCalibratedFieldGuard) {
    const CameraIntrinsics in{400, 400, 160, 120, {0.01, 0, 0, 0, 0}};
    float u, v;
    EXPECT_FALSE(bowl::ProjectToCameraUv(kIdentityExt, in, 320, 240, {5, 5, 1}, &u, &v));
}

TEST(BowlProjection, SurfacePointAtR0IsFlatFloor) {
    const Vec3 p = bowl::BowlSurfacePoint(6.0, 0.2, 22.0, 0.7, 6.0);
    EXPECT_NEAR(p.z, 0.0, 1e-9);
    EXPECT_NEAR(p.x, 6.0 * std::cos(0.7), 1e-9);
    EXPECT_NEAR(p.y, 6.0 * std::sin(0.7), 1e-9);
}

TEST(BowlProjection, SurfacePointAtRmaxMatchesWallRiseFormula) {
    const double R0 = 6.0, k = 0.2, Rmax = 22.0;
    const Vec3 p = bowl::BowlSurfacePoint(R0, k, Rmax, 1.2, Rmax);
    const double expected_z = k * (Rmax - R0) * (Rmax - R0);
    EXPECT_NEAR(p.z, expected_z, 1e-9);
}

TEST(BowlProjection, SurfacePointBeyondRmaxClampsToWallTop) {
    const double R0 = 6.0, k = 0.2, Rmax = 22.0;
    const Vec3 at_rmax = bowl::BowlSurfacePoint(R0, k, Rmax, 0.0, Rmax);
    const Vec3 beyond = bowl::BowlSurfacePoint(R0, k, Rmax, 0.0, Rmax + 10.0);
    EXPECT_NEAR(at_rmax.z, beyond.z, 1e-9);
}
