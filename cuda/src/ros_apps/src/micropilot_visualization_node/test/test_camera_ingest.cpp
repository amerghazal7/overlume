/** @file test_camera_ingest.cpp
 *  @brief VM-091 Task 2 Step 6. Exercises camera_ingest.hpp's pure, ROS/GPU-
 *  free math and bookkeeping directly -- no rclcpp node/spin, no GPU, same
 *  "no full VisualizationNode/rclcpp harness in this suite" shape
 *  test_ego_anchor.cpp/test_scene_layout.cpp already use. The ROS-facing
 *  CameraIngest wrapper itself (subscriptions, cv_bridge, the mpviz:: call
 *  sites) is exercised end-to-end only by actually running the node (this
 *  epic's Step 5 perf-gate bag run) -- there is no rclcpp-node-harness
 *  convention in this package to stand one up here, and doing so just for
 *  this test would be new scaffolding this package's own test suite
 *  deliberately avoids elsewhere (see test_profile.cpp's comment on why
 *  even GPU-linking tests stay header/library-call-only).
 */
#include "micropilot_visualization_node/camera_ingest.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace mpviz_test = micropilot::visualization_app;

// ---- twist_at ---------------------------------------------------------

TEST(TwistAt, EmptyBufferReturnsFalse) {
    std::deque<mpviz_test::StampedTwist> twists;
    mpviz_test::StampedTwist out;
    EXPECT_FALSE(mpviz_test::twist_at(twists, 1.0, out));
}

TEST(TwistAt, ClampsToBufferEnds) {
    std::deque<mpviz_test::StampedTwist> twists{{1.0, 1.0, 0.0, 0.0}, {2.0, 2.0, 0.0, 0.0}};
    mpviz_test::StampedTwist out;
    ASSERT_TRUE(mpviz_test::twist_at(twists, 0.0, out));
    EXPECT_DOUBLE_EQ(out.vx, 1.0);
    ASSERT_TRUE(mpviz_test::twist_at(twists, 5.0, out));
    EXPECT_DOUBLE_EQ(out.vx, 2.0);
}

TEST(TwistAt, InterpolatesLinearlyBetweenSamples) {
    std::deque<mpviz_test::StampedTwist> twists{{1.0, 0.0, 0.0, 0.0}, {2.0, 2.0, 0.0, 0.0}};
    mpviz_test::StampedTwist out;
    ASSERT_TRUE(mpviz_test::twist_at(twists, 1.5, out));
    EXPECT_NEAR(out.vx, 1.0, 1e-9);
}

// ---- rig_delta ----------------------------------------------------------

TEST(RigDelta, NoOdometryReturnsFalse) {
    std::deque<mpviz_test::StampedTwist> twists;
    double th, px, py;
    EXPECT_FALSE(mpviz_test::rig_delta(twists, 10.0, 10.08, th, px, py));
}

TEST(RigDelta, DegenerateSpanReturnsFalse) {
    std::deque<mpviz_test::StampedTwist> twists{{10.0, 1.0, 0.0, 0.0}};
    double th, px, py;
    EXPECT_FALSE(mpviz_test::rig_delta(twists, 10.0, 10.0, th, px, py));
}

TEST(RigDelta, ConstantForwardVelocityIntegratesToLinearDisplacement) {
    // Constant vx=1 m/s, no yaw, over an 80ms span (this bag's real
    // camera-phase spread, per the plan's own m2o1 note) -> ~0.08m of
    // forward travel, zero yaw.
    std::deque<mpviz_test::StampedTwist> twists{{9.0, 1.0, 0.0, 0.0}, {11.0, 1.0, 0.0, 0.0}};
    double th, px, py;
    ASSERT_TRUE(mpviz_test::rig_delta(twists, 10.0, 10.08, th, px, py));
    EXPECT_NEAR(th, 0.0, 1e-9);
    EXPECT_NEAR(px, 0.08, 1e-6);
    EXPECT_NEAR(py, 0.0, 1e-9);
}

TEST(RigDelta, ConstantYawRateIntegratesToRotation) {
    std::deque<mpviz_test::StampedTwist> twists{{9.0, 0.0, 0.0, 1.0}, {11.0, 0.0, 0.0, 1.0}};
    double th, px, py;
    ASSERT_TRUE(mpviz_test::rig_delta(twists, 10.0, 10.5, th, px, py));
    EXPECT_NEAR(th, 0.5, 1e-6);
}

// ---- compensation_delta_4x4 ----------------------------------------------

TEST(CompensationDelta4x4, IdentityWhenNoOdometry) {
    std::deque<mpviz_test::StampedTwist> twists;
    double delta[16];
    mpviz_test::compensation_delta_4x4(twists, 10.0, 10.08, delta);
    const double I[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    for (int i = 0; i < 16; ++i) EXPECT_NEAR(delta[i], I[i], 1e-12) << "index " << i;
}

TEST(CompensationDelta4x4, IdentityWhenCameraStampEqualsReferenceTime) {
    // The "straddling cameras" pinning case for the camera whose stamp
    // already IS t_max -- Step 6's own requirement (b)/(c): a camera at
    // t_max needs no compensation, same as if odometry were absent.
    std::deque<mpviz_test::StampedTwist> twists{{9.0, 1.0, 0.0, 0.3}, {11.0, 1.0, 0.0, 0.3}};
    double delta[16];
    mpviz_test::compensation_delta_4x4(twists, 10.08, 10.08, delta);
    const double I[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    for (int i = 0; i < 16; ++i) EXPECT_NEAR(delta[i], I[i], 1e-12) << "index " << i;
}

TEST(CompensationDelta4x4, MatchesRigDeltaForTwoStraddlingCameraStamps) {
    // Step 6's required pinning case: two cameras whose stamps straddle
    // max_sync_latency (this bag's own ~80ms real spread) -- assert BOTH
    // cameras' deltas equal rig_delta(stamp_i, t_max), independently
    // recomputed, not merely "returns true".
    std::deque<mpviz_test::StampedTwist> twists{{9.0, 0.5, 0.1, 0.2}, {11.0, 0.5, 0.1, 0.2}};
    const double t_cam_a = 10.00;  // older camera
    const double t_max = 10.08;    // newer camera == the tick's reference time

    double th, px, py;
    ASSERT_TRUE(mpviz_test::rig_delta(twists, t_cam_a, t_max, th, px, py));
    double delta_a[16];
    mpviz_test::compensation_delta_4x4(twists, t_cam_a, t_max, delta_a);
    const double c = std::cos(th), s = std::sin(th);
    const double expected_a[16] = {c, -s, 0, px, s, c, 0, py, 0, 0, 1, 0, 0, 0, 0, 1};
    for (int i = 0; i < 16; ++i) EXPECT_NEAR(delta_a[i], expected_a[i], 1e-9) << "index " << i;

    // The newer camera (already at t_max) gets identity -- covered by
    // IdenticalWhenCameraStampEqualsReferenceTime above; repeated here
    // inline so this one test documents both halves of the straddling pair
    // together.
    double delta_b[16];
    mpviz_test::compensation_delta_4x4(twists, t_max, t_max, delta_b);
    const double I[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    for (int i = 0; i < 16; ++i) EXPECT_NEAR(delta_b[i], I[i], 1e-12) << "index " << i;
}

// ---- OrthonormalizeExtrinsics ---------------------------------------------

TEST(OrthonormalizeExtrinsics, AlreadyOrthonormalPassesThroughWithNegligibleCorrection) {
    mpviz::CameraExtrinsics ext{{1, 0, 0, 0, -1, 0, 0, 0, -1}, {1, 2, 3}};
    double max_corr = -1.0;
    const mpviz::CameraExtrinsics out = mpviz_test::OrthonormalizeExtrinsics(ext, &max_corr);
    EXPECT_LT(max_corr, 1e-9);
    for (int i = 0; i < 9; ++i) EXPECT_NEAR(out.R[i], ext.R[i], 1e-9) << "R[" << i << "]";
    for (int i = 0; i < 3; ++i) EXPECT_DOUBLE_EQ(out.t[i], ext.t[i]);
}

TEST(OrthonormalizeExtrinsics, SkewedRIsCorrectedToAnOrthonormalRightHandedBasis) {
    // right/down/fwd nominally (1,0,0)/(0,-1,0)/(0,0,-1), but `down` is
    // nudged off-orthogonal by a small amount -- plausible calibration
    // noise, not an extreme case.
    mpviz::CameraExtrinsics ext{{1, 0.05, 0, 0, -1, 0.03, 0, 0, -1}, {0, 0, 0}};
    double max_corr = 0.0;
    const mpviz::CameraExtrinsics out = mpviz_test::OrthonormalizeExtrinsics(ext, &max_corr);
    EXPECT_GT(max_corr, 0.0) << "a skewed R should report a nonzero correction";
    EXPECT_LT(max_corr, mpviz_test::kOrthonormalizeWarnThresholdRad)
        << "this test's skew is deliberately small -- should not itself cross the WARN bar";

    auto col = [&](int j) { return std::array<double, 3>{out.R[j], out.R[3 + j], out.R[6 + j]}; };
    auto dot = [](std::array<double, 3> a, std::array<double, 3> b)
    { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
    const auto right = col(0), down = col(1), fwd = col(2);
    EXPECT_NEAR(dot(right, right), 1.0, 1e-9);
    EXPECT_NEAR(dot(down, down), 1.0, 1e-9);
    EXPECT_NEAR(dot(fwd, fwd), 1.0, 1e-9);
    EXPECT_NEAR(dot(right, down), 0.0, 1e-9);
    EXPECT_NEAR(dot(right, fwd), 0.0, 1e-9);
    EXPECT_NEAR(dot(down, fwd), 0.0, 1e-9);
    // Right-handed: right x down == fwd (bowl.mat's own down=cross(fwd,right)
    // reconstruction, cyclically).
    const std::array<double, 3> cross{right[1] * down[2] - right[2] * down[1],
                                       right[2] * down[0] - right[0] * down[2],
                                       right[0] * down[1] - right[1] * down[0]};
    for (int i = 0; i < 3; ++i) EXPECT_NEAR(cross[i], fwd[i], 1e-9) << "axis " << i;
}

TEST(OrthonormalizeExtrinsics, GrosslyNonOrthogonalRCrossesTheWarnThreshold) {
    // down nudged by ~20 degrees worth of skew -- large enough that a real
    // caller (CameraIngest's ctor) would WARN.
    mpviz::CameraExtrinsics ext{{1, 0.36, 0, 0, -1, 0, 0, 0, -1}, {0, 0, 0}};
    double max_corr = 0.0;
    mpviz_test::OrthonormalizeExtrinsics(ext, &max_corr);
    EXPECT_GT(max_corr, mpviz_test::kOrthonormalizeWarnThresholdRad);
}

// ---- IngestState: info_ready gate + monotonic frame_id --------------------

TEST(IngestState, AllInfoReadyFalseUntilEveryConfiguredCameraReports) {
    std::vector<mpviz::CameraExtrinsics> ext(2);
    mpviz_test::IngestState state(2, ext);
    EXPECT_FALSE(state.all_info_ready());

    mpviz::CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    EXPECT_FALSE(state.record_camera_info(0, in, 320, 240))
        << "camera 1 still missing -- not complete yet";
    EXPECT_FALSE(state.all_info_ready());
    EXPECT_TRUE(state.record_camera_info(1, in, 320, 240))
        << "the LAST camera's CameraInfo should signal completion";
    EXPECT_TRUE(state.all_info_ready());
}

TEST(IngestState, SubsequentCameraInfoOnlySignalsRebakeWhenSomethingActuallyChanged) {
    std::vector<mpviz::CameraExtrinsics> ext(1);
    mpviz_test::IngestState state(1, ext);
    mpviz::CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    ASSERT_TRUE(state.record_camera_info(0, in, 320, 240));  // first completion

    // Identical CameraInfo again -- nothing changed, no re-bake needed.
    EXPECT_FALSE(state.record_camera_info(0, in, 320, 240));

    // A real change (driver reconnect / live extrinsics edit changing dims).
    EXPECT_TRUE(state.record_camera_info(0, in, 640, 480));
}

TEST(IngestState, ImageArrivalBumpsAMonotonicPerCameraFrameId) {
    std::vector<mpviz::CameraExtrinsics> ext(2);
    mpviz_test::IngestState state(2, ext);
    EXPECT_EQ(state.record_image_stamp(0, 10.0), 1u);
    EXPECT_EQ(state.record_image_stamp(0, 10.1), 2u);
    EXPECT_EQ(state.record_image_stamp(1, 10.0), 1u)
        << "each camera's counter is independent";
    EXPECT_EQ(state.record_image_stamp(0, 10.2), 3u);
}

// ---- IngestState::rgb: CameraInfo/image-stream dim-mismatch guard ---------
// VM-094 review round 1 finding 2: CameraInfo can advertise different dims
// than the image stream actually publishes (calibration-res CameraInfo +
// a downscaled stream) -- rgb() must not hand out a buffer ingested at one
// size once width(i)/height(i) says another, or ColorizeFromCameras' raw
// pointer indexing reads past the end of it.
TEST(IngestState, RgbReturnsNullptrWhenStoredBufferDimsDisagreeWithCameraInfo) {
    std::vector<mpviz::CameraExtrinsics> ext(1);
    mpviz_test::IngestState state(1, ext);
    mpviz::CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    // CameraInfo advertises 320x240 (e.g. the sensor's calibration resolution)...
    ASSERT_TRUE(state.record_camera_info(0, in, 320, 240));
    std::vector<uint8_t> frame(160 * 120 * 3, 42);
    // ...but the actual published image stream is downscaled to 160x120.
    state.store_rgb(0, frame.data(), 160, 120);
    EXPECT_EQ(state.rgb(0), nullptr)
        << "mismatched dims must read as 'no image', not sample past the buffer's end";
}

TEST(IngestState, RgbReturnsTheBufferWhenDimsMatchCameraInfo) {
    std::vector<mpviz::CameraExtrinsics> ext(1);
    mpviz_test::IngestState state(1, ext);
    mpviz::CameraIntrinsics in{400, 400, 160, 120, {0, 0, 0, 0, 0}};
    ASSERT_TRUE(state.record_camera_info(0, in, 320, 240));
    std::vector<uint8_t> frame(320 * 240 * 3, 42);
    state.store_rgb(0, frame.data(), 320, 240);
    EXPECT_NE(state.rgb(0), nullptr);
}

TEST(IngestState, NewestStampIsTheFrameSyncGatesTMax) {
    std::vector<mpviz::CameraExtrinsics> ext(2);
    mpviz_test::IngestState state(2, ext);
    double t_max = -1.0;
    EXPECT_FALSE(state.newest_stamp(t_max)) << "no image has arrived for either camera yet";

    state.record_image_stamp(0, 10.00);
    state.record_image_stamp(1, 10.08);  // this bag's own ~80ms real spread
    ASSERT_TRUE(state.newest_stamp(t_max));
    EXPECT_NEAR(t_max, 10.08, 1e-9);
}
