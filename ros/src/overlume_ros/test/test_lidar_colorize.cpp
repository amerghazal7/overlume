// test_lidar_colorize.cpp -- VM-094 (unified-engine migration Task 5)
// Steps 0-2. Pure camera math, no rclcpp/ROS involvement (same "no full
// OverlumeNode/rclcpp harness in this suite" shape as
// test_camera_ingest.cpp) -- ColorizeFromCameras() takes plain vectors and
// a BowlConfig, nothing ROS-shaped.
#include "overlume_ros/lidar_colorize.hpp"

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

namespace
{
using overlume::ros::ColorizeFromCameras;
using overlume::CameraExtrinsics;
using overlume::CameraIntrinsics;

// Identity extrinsics at the rig origin: right=(1,0,0), down=(0,1,0),
// fwd=(0,0,1) -- R columns per bowl_projection.hpp's own convention
// (row-major R, column j = (R[j],R[3+j],R[6+j])). A point at rig-frame
// (0,0,depth) sits dead ahead.
CameraExtrinsics IdentityCameraAt(double tx, double ty, double tz)
{
    return CameraExtrinsics{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {tx, ty, tz}};
}

// No distortion, principal point centered -- (0,0,depth) projects exactly
// to (cx,cy) regardless of depth.
CameraIntrinsics ZeroDistIntrinsics(double fx, double fy, double cx, double cy)
{
    return CameraIntrinsics{fx, fy, cx, cy, {0, 0, 0, 0, 0}};
}

// Tightly-packed WxHx3 RGB8 buffer, every pixel the same color -- a
// "test-pattern" solid fill, same fixture shape the Interfaces block's own
// Step 0 comment describes.
std::vector<uint8_t> SolidRgbBuffer(uint32_t w, uint32_t h, uint8_t r, uint8_t g, uint8_t b)
{
    std::vector<uint8_t> buf(static_cast<size_t>(w) * h * 3);
    for (size_t i = 0; i < buf.size(); i += 3)
    {
        buf[i] = r;
        buf[i + 1] = g;
        buf[i + 2] = b;
    }
    return buf;
}

void Unpack(uint32_t rgba, uint8_t& r, uint8_t& g, uint8_t& b, uint8_t& a)
{
    r = static_cast<uint8_t>(rgba & 0xFF);
    g = static_cast<uint8_t>((rgba >> 8) & 0xFF);
    b = static_cast<uint8_t>((rgba >> 16) & 0xFF);
    a = static_cast<uint8_t>((rgba >> 24) & 0xFF);
}
}  // namespace

// Step 0: a lidar point visible to exactly one configured camera gets that
// camera's pixel color.
TEST(LidarColorize, PointInSingleCameraFovGetsThatCamerasColor)
{
    const CameraExtrinsics ext = IdentityCameraAt(0, 0, 0);
    const CameraIntrinsics in = ZeroDistIntrinsics(500, 500, 320, 240);
    const uint32_t w = 640, h = 480;
    const auto rgb = SolidRgbBuffer(w, h, 10, 20, 30);

    overlume::BowlConfig cfg{};
    cfg.camera_count = 1;
    cfg.extrinsics = &ext;
    cfg.intrinsics = &in;
    cfg.cam_width = &w;
    cfg.cam_height = &h;

    const std::vector<overlume::Vec3> points{{0, 0, 5}};  // straight ahead, depth 5
    const std::vector<const uint8_t*> bufs{rgb.data()};

    const auto out = ColorizeFromCameras(points, cfg, bufs);
    ASSERT_EQ(out.size(), 1u);
    uint8_t r, g, b, a;
    Unpack(out[0].rgba, r, g, b, a);
    // Expected bytes are the sRGB->linear LUT of the sampled camera bytes
    // (pack_rgba linearizes camera samples -- see lidar_colorize.cpp).
    EXPECT_EQ(r, 1);
    EXPECT_EQ(g, 2);
    EXPECT_EQ(b, 3);
    EXPECT_EQ(a, 255);
    EXPECT_DOUBLE_EQ(out[0].position.x, 0.0);
    EXPECT_DOUBLE_EQ(out[0].position.y, 0.0);
    EXPECT_DOUBLE_EQ(out[0].position.z, 5.0);
}

// Step 1: a point outside every configured camera's FOV is DROPPED from the
// output, not emitted with any sentinel color.
TEST(LidarColorize, PointOutsideEveryCameraFovIsDropped)
{
    const CameraExtrinsics ext = IdentityCameraAt(0, 0, 0);
    const CameraIntrinsics in = ZeroDistIntrinsics(500, 500, 320, 240);
    const uint32_t w = 640, h = 480;
    const auto rgb = SolidRgbBuffer(w, h, 1, 2, 3);

    overlume::BowlConfig cfg{};
    cfg.camera_count = 1;
    cfg.extrinsics = &ext;
    cfg.intrinsics = &in;
    cfg.cam_width = &w;
    cfg.cam_height = &h;

    // point 0: covered (straight ahead). point 1: same depth but far
    // off-axis -- xn=yn=200 at z=5, xp = 500*200+320 lands nowhere near
    // [0,640) -- outside this camera's pixel bounds (ProjectToCameraUv's
    // own bounds check).
    const std::vector<overlume::Vec3> points{{0, 0, 5}, {1000, 1000, 5}};
    const std::vector<const uint8_t*> bufs{rgb.data()};

    const auto out = ColorizeFromCameras(points, cfg, bufs);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_DOUBLE_EQ(out[0].position.z, 5.0);
    EXPECT_DOUBLE_EQ(out[0].position.x, 0.0);
}

// Step 2: a point visible to two cameras picks the first configured match,
// deterministically -- the named parity-exception tie-break rule (first-
// match-wins, not blended, per Decision 5/lidar_colorize.hpp's own doc).
TEST(LidarColorize, PointVisibleToTwoCamerasPicksFirstConfiguredMatch)
{
    // Both cameras identical geometry (co-located, identical intrinsics) so
    // both genuinely cover the same point -- isolates the tie-break rule
    // from any FOV-overlap-geometry concern.
    const CameraExtrinsics ext0 = IdentityCameraAt(0, 0, 0);
    const CameraExtrinsics ext1 = IdentityCameraAt(0, 0, 0);
    const CameraIntrinsics in = ZeroDistIntrinsics(500, 500, 320, 240);
    const uint32_t w = 640, h = 480;
    const auto rgb0 = SolidRgbBuffer(w, h, 111, 112, 113);
    const auto rgb1 = SolidRgbBuffer(w, h, 200, 201, 202);

    const std::vector<CameraExtrinsics> ext{ext0, ext1};
    const std::vector<CameraIntrinsics> in_v{in, in};
    const std::vector<uint32_t> w_v{w, w};
    const std::vector<uint32_t> h_v{h, h};

    overlume::BowlConfig cfg{};
    cfg.camera_count = 2;
    cfg.extrinsics = ext.data();
    cfg.intrinsics = in_v.data();
    cfg.cam_width = w_v.data();
    cfg.cam_height = h_v.data();

    const std::vector<overlume::Vec3> points{{0, 0, 5}};
    const std::vector<const uint8_t*> bufs{rgb0.data(), rgb1.data()};

    const auto out = ColorizeFromCameras(points, cfg, bufs);
    ASSERT_EQ(out.size(), 1u);
    uint8_t r, g, b, a;
    Unpack(out[0].rgba, r, g, b, a);
    // Expected bytes are the sRGB->linear LUT of the sampled camera bytes
    // (pack_rgba linearizes camera samples -- see lidar_colorize.cpp).
    EXPECT_EQ(r, 41);  // camera 0's color (sRGB 111,112,113 linearized), not camera 1's
    EXPECT_EQ(g, 41);
    EXPECT_EQ(b, 42);
}

// A null buffer entry (camera never delivered a frame) is skipped, not
// dereferenced -- the point falls through to the next configured camera
// (or is dropped if none remain), never a crash.
TEST(LidarColorize, NullBufferForACoveringCameraFallsThroughToTheNextOne)
{
    const CameraExtrinsics ext0 = IdentityCameraAt(0, 0, 0);
    const CameraExtrinsics ext1 = IdentityCameraAt(0, 0, 0);
    const CameraIntrinsics in = ZeroDistIntrinsics(500, 500, 320, 240);
    const uint32_t w = 640, h = 480;
    const auto rgb1 = SolidRgbBuffer(w, h, 7, 8, 9);

    const std::vector<CameraExtrinsics> ext{ext0, ext1};
    const std::vector<CameraIntrinsics> in_v{in, in};
    const std::vector<uint32_t> w_v{w, w};
    const std::vector<uint32_t> h_v{h, h};

    overlume::BowlConfig cfg{};
    cfg.camera_count = 2;
    cfg.extrinsics = ext.data();
    cfg.intrinsics = in_v.data();
    cfg.cam_width = w_v.data();
    cfg.cam_height = h_v.data();

    const std::vector<overlume::Vec3> points{{0, 0, 5}};
    const std::vector<const uint8_t*> bufs{nullptr, rgb1.data()};  // camera 0 has no frame yet

    const auto out = ColorizeFromCameras(points, cfg, bufs);
    ASSERT_EQ(out.size(), 1u);
    uint8_t r, g, b, a;
    Unpack(out[0].rgba, r, g, b, a);
    // Expected bytes are the sRGB->linear LUT of the sampled camera bytes
    // (pack_rgba linearizes camera samples -- see lidar_colorize.cpp).
    EXPECT_EQ(r, 1);
    EXPECT_EQ(g, 1);
    EXPECT_EQ(b, 1);
}
