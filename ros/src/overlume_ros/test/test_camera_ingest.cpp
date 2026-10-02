// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/camera_ingest.hpp"

#include <chrono>
#include <cmath>

#include <gtest/gtest.h>

namespace overlume_test = overlume::ros;

TEST(TwistAt, EmptyBufferReturnsFalse) {
    std::deque<overlume_test::StampedTwist> twists;
    overlume_test::StampedTwist out;
    EXPECT_FALSE(overlume_test::twist_at(twists, 1.0, out));
}

TEST(TwistAt, ClampsToBufferEnds) {
    std::deque<overlume_test::StampedTwist> twists{{1.0, 1.0, 0.0, 0.0}, {2.0, 2.0, 0.0, 0.0}};
    overlume_test::StampedTwist out;
    ASSERT_TRUE(overlume_test::twist_at(twists, 0.0, out));
    EXPECT_DOUBLE_EQ(out.vx, 1.0);
    ASSERT_TRUE(overlume_test::twist_at(twists, 5.0, out));
    EXPECT_DOUBLE_EQ(out.vx, 2.0);
}

TEST(TwistAt, InterpolatesLinearlyBetweenSamples) {
    std::deque<overlume_test::StampedTwist> twists{{1.0, 0.0, 0.0, 0.0}, {2.0, 2.0, 0.0, 0.0}};
    overlume_test::StampedTwist out;
    ASSERT_TRUE(overlume_test::twist_at(twists, 1.5, out));
    EXPECT_NEAR(out.vx, 1.0, 1e-9);
}

TEST(RigDelta, NoOdometryReturnsFalse) {
    std::deque<overlume_test::StampedTwist> twists;
    double th, px, py;
    EXPECT_FALSE(overlume_test::rig_delta(twists, 10.0, 10.08, th, px, py));
}

TEST(RigDelta, DegenerateSpanReturnsFalse) {
    std::deque<overlume_test::StampedTwist> twists{{10.0, 1.0, 0.0, 0.0}};
    double th, px, py;
    EXPECT_FALSE(overlume_test::rig_delta(twists, 10.0, 10.0, th, px, py));
}

TEST(RigDelta, ConstantForwardVelocityIntegratesToLinearDisplacement) {
    std::deque<overlume_test::StampedTwist> twists{{9.0, 1.0, 0.0, 0.0}, {11.0, 1.0, 0.0, 0.0}};
    double th, px, py;
    ASSERT_TRUE(overlume_test::rig_delta(twists, 10.0, 10.08, th, px, py));
    EXPECT_NEAR(th, 0.0, 1e-9);
    EXPECT_NEAR(px, 0.08, 1e-6);
    EXPECT_NEAR(py, 0.0, 1e-9);
}

TEST(RigDelta, ConstantYawRateIntegratesToRotation) {
    std::deque<overlume_test::StampedTwist> twists{{9.0, 0.0, 0.0, 1.0}, {11.0, 0.0, 0.0, 1.0}};
    double th, px, py;
    ASSERT_TRUE(overlume_test::rig_delta(twists, 10.0, 10.5, th, px, py));
    EXPECT_NEAR(th, 0.5, 1e-6);
}

TEST(CompensationDelta4x4, IdentityWhenNoOdometry) {
    std::deque<overlume_test::StampedTwist> twists;
    double delta[16];
    overlume_test::compensation_delta_4x4(twists, 10.0, 10.08, delta);
    const double I[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    for (int i = 0; i < 16; ++i) EXPECT_NEAR(delta[i], I[i], 1e-12) << "index " << i;
}

TEST(CompensationDelta4x4, IdentityWhenCameraStampEqualsReferenceTime) {
    std::deque<overlume_test::StampedTwist> twists{{9.0, 1.0, 0.0, 0.3}, {11.0, 1.0, 0.0, 0.3}};
    double delta[16];
    overlume_test::compensation_delta_4x4(twists, 10.08, 10.08, delta);
    const double I[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    for (int i = 0; i < 16; ++i) EXPECT_NEAR(delta[i], I[i], 1e-12) << "index " << i;
}

TEST(CompensationDelta4x4, MatchesRigDeltaForTwoStraddlingCameraStamps) {
    std::deque<overlume_test::StampedTwist> twists{{9.0, 0.5, 0.1, 0.2}, {11.0, 0.5, 0.1, 0.2}};
    const double t_cam_a = 10.00;
    const double t_max = 10.08;

    double th, px, py;
    ASSERT_TRUE(overlume_test::rig_delta(twists, t_cam_a, t_max, th, px, py));
    double delta_a[16];
    overlume_test::compensation_delta_4x4(twists, t_cam_a, t_max, delta_a);
    const double c = std::cos(th), s = std::sin(th);
    const double expected_a[16] = {c, -s, 0, px, s, c, 0, py, 0, 0, 1, 0, 0, 0, 0, 1};
    for (int i = 0; i < 16; ++i) EXPECT_NEAR(delta_a[i], expected_a[i], 1e-9) << "index " << i;

    double delta_b[16];
    overlume_test::compensation_delta_4x4(twists, t_max, t_max, delta_b);
    const double I[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    for (int i = 0; i < 16; ++i) EXPECT_NEAR(delta_b[i], I[i], 1e-12) << "index " << i;
}

TEST(OrthonormalizeExtrinsics, AlreadyOrthonormalPassesThroughWithNegligibleCorrection) {
    overlume::CameraExtrinsics ext{{1, 0, 0, 0, -1, 0, 0, 0, -1}, {1, 2, 3}};
    double max_corr = -1.0;
    const overlume::CameraExtrinsics out = overlume_test::OrthonormalizeExtrinsics(ext, &max_corr);
    EXPECT_LT(max_corr, 1e-9);
    for (int i = 0; i < 9; ++i) EXPECT_NEAR(out.R[i], ext.R[i], 1e-9) << "R[" << i << "]";
    for (int i = 0; i < 3; ++i) EXPECT_DOUBLE_EQ(out.t[i], ext.t[i]);
}

TEST(OrthonormalizeExtrinsics, SkewedRIsCorrectedToAnOrthonormalRightHandedBasis) {
    overlume::CameraExtrinsics ext{{1, 0.05, 0, 0, -1, 0.03, 0, 0, -1}, {0, 0, 0}};
    double max_corr = 0.0;
    const overlume::CameraExtrinsics out = overlume_test::OrthonormalizeExtrinsics(ext, &max_corr);
    EXPECT_GT(max_corr, 0.0) << "a skewed R should report a nonzero correction";
    EXPECT_LT(max_corr, overlume_test::kOrthonormalizeWarnThresholdRad)
        << "this test's skew is deliberately small -- should not itself cross the WARN bar";

    auto col = [&](int j) { return std::array<double, 3>{out.R[j], out.R[3 + j], out.R[6 + j]}; };
    auto dot = [](std::array<double, 3> a, std::array<double, 3> b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    };
    const auto right = col(0), down = col(1), fwd = col(2);
    EXPECT_NEAR(dot(right, right), 1.0, 1e-9);
    EXPECT_NEAR(dot(down, down), 1.0, 1e-9);
    EXPECT_NEAR(dot(fwd, fwd), 1.0, 1e-9);
    EXPECT_NEAR(dot(right, down), 0.0, 1e-9);
    EXPECT_NEAR(dot(right, fwd), 0.0, 1e-9);
    EXPECT_NEAR(dot(down, fwd), 0.0, 1e-9);
    const std::array<double, 3> cross{right[1] * down[2] - right[2] * down[1],
                                      right[2] * down[0] - right[0] * down[2],
                                      right[0] * down[1] - right[1] * down[0]};
    for (int i = 0; i < 3; ++i) EXPECT_NEAR(cross[i], fwd[i], 1e-9) << "axis " << i;
}

TEST(OrthonormalizeExtrinsics, GrosslyNonOrthogonalRCrossesTheWarnThreshold) {
    overlume::CameraExtrinsics ext{{1, 0.36, 0, 0, -1, 0, 0, 0, -1}, {0, 0, 0}};
    double max_corr = 0.0;
    overlume_test::OrthonormalizeExtrinsics(ext, &max_corr);
    EXPECT_GT(max_corr, overlume_test::kOrthonormalizeWarnThresholdRad);
}

TEST(IngestState, AllInfoReadyFalseUntilEveryConfiguredCameraReports) {
    std::vector<overlume::CameraExtrinsics> ext(2);
    overlume_test::IngestState state(2, ext);
    EXPECT_FALSE(state.all_info_ready());

    overlume::CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    EXPECT_FALSE(state.record_camera_info(0, in, 320, 240))
        << "camera 1 still missing -- not complete yet";
    EXPECT_FALSE(state.all_info_ready());
    EXPECT_TRUE(state.record_camera_info(1, in, 320, 240))
        << "the LAST camera's CameraInfo should signal completion";
    EXPECT_TRUE(state.all_info_ready());
}

TEST(IngestState, SubsequentCameraInfoOnlySignalsRebakeWhenSomethingActuallyChanged) {
    std::vector<overlume::CameraExtrinsics> ext(1);
    overlume_test::IngestState state(1, ext);
    overlume::CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    ASSERT_TRUE(state.record_camera_info(0, in, 320, 240));

    EXPECT_FALSE(state.record_camera_info(0, in, 320, 240));

    EXPECT_TRUE(state.record_camera_info(0, in, 640, 480));
}

TEST(IngestState, ImageArrivalBumpsAMonotonicPerCameraFrameId) {
    std::vector<overlume::CameraExtrinsics> ext(2);
    overlume_test::IngestState state(2, ext);
    EXPECT_EQ(state.record_image_stamp(0, 10.0), 1u);
    EXPECT_EQ(state.record_image_stamp(0, 10.1), 2u);
    EXPECT_EQ(state.record_image_stamp(1, 10.0), 1u) << "each camera's counter is independent";
    EXPECT_EQ(state.record_image_stamp(0, 10.2), 3u);
}

TEST(IngestState, RgbReturnsNullptrWhenStoredBufferDimsDisagreeWithCameraInfo) {
    std::vector<overlume::CameraExtrinsics> ext(1);
    overlume_test::IngestState state(1, ext);
    overlume::CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    ASSERT_TRUE(state.record_camera_info(0, in, 320, 240));
    std::vector<uint8_t> frame(160 * 120 * 3, 42);
    state.store_rgb(0, frame.data(), 160, 120);
    EXPECT_EQ(state.rgb(0), nullptr)
        << "mismatched dims must read as 'no image', not sample past the buffer's end";
}

TEST(IngestState, RgbReturnsTheBufferWhenDimsMatchCameraInfo) {
    std::vector<overlume::CameraExtrinsics> ext(1);
    overlume_test::IngestState state(1, ext);
    overlume::CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    ASSERT_TRUE(state.record_camera_info(0, in, 320, 240));
    std::vector<uint8_t> frame(320 * 240 * 3, 42);
    state.store_rgb(0, frame.data(), 320, 240);
    EXPECT_NE(state.rgb(0), nullptr);
}

TEST(IngestState, NewestStampIsTheFrameSyncGatesTMax) {
    std::vector<overlume::CameraExtrinsics> ext(2);
    overlume_test::IngestState state(2, ext);
    double t_max = -1.0;
    EXPECT_FALSE(state.newest_stamp(t_max)) << "no image has arrived for either camera yet";

    state.record_image_stamp(0, 10.00);
    state.record_image_stamp(1, 10.08);
    ASSERT_TRUE(state.newest_stamp(t_max));
    EXPECT_NEAR(t_max, 10.08, 1e-9);
}

TEST(ApplyMotionDelta, IdentityIsNoOp) {
    overlume::CameraExtrinsics e{{0, -1, 0, 1, 0, 0, 0, 0, 1}, {1.0, 2.0, 3.0}};
    const double I[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    const auto o = overlume_test::ApplyMotionDelta(e, I);
    for (int i = 0; i < 9; ++i) EXPECT_DOUBLE_EQ(o.R[i], e.R[i]);
    for (int i = 0; i < 3; ++i) EXPECT_DOUBLE_EQ(o.t[i], e.t[i]);
}

TEST(ApplyMotionDelta, MatchesUpdateBowlFormula) {
    // bowl.cpp: right/fwd' = Rd^T v ; t' = Rd^T (t - pd)
    overlume::CameraExtrinsics e{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {2.0, 0.5, 1.0}};
    const double th = 0.3, px = 0.4, py = -0.1;
    const double c = std::cos(th), s = std::sin(th);
    const double d[16] = {c, -s, 0, px, s, c, 0, py, 0, 0, 1, 0, 0, 0, 0, 1};
    const auto o = overlume_test::ApplyMotionDelta(e, d);
    // right = (1,0,0): Rd^T right = (c, -s, 0)
    EXPECT_NEAR(o.R[0], c, 1e-12);
    EXPECT_NEAR(o.R[3], -s, 1e-12);
    // t - pd = (1.6, 0.6, 1): Rd^T = (c*1.6 + s*0.6, -s*1.6 + c*0.6, 1)
    EXPECT_NEAR(o.t[0], c * 1.6 + s * 0.6, 1e-12);
    EXPECT_NEAR(o.t[1], -s * 1.6 + c * 0.6, 1e-12);
    EXPECT_NEAR(o.t[2], 1.0, 1e-12);
    const double dx[16] = {1, 0, 0, 0.5, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    EXPECT_NEAR(overlume_test::ApplyMotionDelta(e, dx).t[0], 1.5, 1e-12);
}

TEST(CloudCompDelta, MatchesRigDeltaOverTheSpan) {
    std::deque<overlume_test::StampedTwist> tw{{9.0, 1.0, 0.2, 0.3}, {11.0, 1.0, 0.2, 0.3}};
    double th, px, py, th2, px2, py2;
    ASSERT_TRUE(overlume_test::cloud_comp_delta(tw, 10.0, 10.1, th, px, py));
    ASSERT_TRUE(overlume_test::rig_delta(tw, 10.0, 10.1, th2, px2, py2));
    EXPECT_DOUBLE_EQ(th, th2);
    EXPECT_DOUBLE_EQ(px, px2);
    EXPECT_DOUBLE_EQ(py, py2);
}

TEST(CloudCompDelta, SpanBoundedAndFast) {
    std::deque<overlume_test::StampedTwist> tw{{9.0, 1.0, 0.0, 0.1}, {11.0, 1.0, 0.0, 0.1}};
    double th, px, py;
    const auto t0 = std::chrono::steady_clock::now();
    EXPECT_FALSE(overlume_test::rig_delta(tw, 0.0, 1e6, th, px, py));
    EXPECT_EQ(th, 0.0);
    EXPECT_EQ(px, 0.0);
    EXPECT_EQ(py, 0.0);
    EXPECT_FALSE(overlume_test::cloud_comp_delta(tw, 10.0 - 1e6, 10.0, th, px, py));
    EXPECT_EQ(px, 0.0);
    const auto ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0);
    EXPECT_LT(ms.count(), 10.0) << "unbounded rig_delta loop";
    EXPECT_FALSE(overlume_test::cloud_comp_delta(tw, 10.0 - 0.6, 10.0, th, px, py));
    EXPECT_TRUE(overlume_test::cloud_comp_delta(tw, 10.0 - 0.4, 10.0, th, px, py));
    EXPECT_FALSE(overlume_test::cloud_comp_delta(tw, 0.0, 10.0, th, px, py));  // zero stamp
}
