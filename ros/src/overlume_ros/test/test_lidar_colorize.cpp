// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/lidar_colorize.hpp"

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

namespace {
using overlume::ros::ColorizeFromCameras;
using overlume::CameraExtrinsics;
using overlume::CameraIntrinsics;

CameraExtrinsics IdentityCameraAt(double tx, double ty, double tz) {
    return CameraExtrinsics{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {tx, ty, tz}};
}

CameraIntrinsics ZeroDistIntrinsics(double fx, double fy, double cx, double cy) {
    return CameraIntrinsics{fx, fy, cx, cy, {0, 0, 0, 0, 0}};
}

std::vector<uint8_t> SolidRgbBuffer(uint32_t w, uint32_t h, uint8_t r, uint8_t g, uint8_t b) {
    std::vector<uint8_t> buf(static_cast<size_t>(w) * h * 3);
    for (size_t i = 0; i < buf.size(); i += 3) {
        buf[i] = r;
        buf[i + 1] = g;
        buf[i + 2] = b;
    }
    return buf;
}

void Unpack(uint32_t rgba, uint8_t& r, uint8_t& g, uint8_t& b, uint8_t& a) {
    r = static_cast<uint8_t>(rgba & 0xFF);
    g = static_cast<uint8_t>((rgba >> 8) & 0xFF);
    b = static_cast<uint8_t>((rgba >> 16) & 0xFF);
    a = static_cast<uint8_t>((rgba >> 24) & 0xFF);
}
}

TEST(LidarColorize, PointInSingleCameraFovGetsThatCamerasColor) {
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

    const std::vector<overlume::Vec3> points{{0, 0, 5}};
    const std::vector<const uint8_t*> bufs{rgb.data()};

    const auto out = ColorizeFromCameras(points, cfg, bufs);
    ASSERT_EQ(out.size(), 1u);
    uint8_t r, g, b, a;
    Unpack(out[0].rgba, r, g, b, a);
    EXPECT_EQ(r, 1);
    EXPECT_EQ(g, 2);
    EXPECT_EQ(b, 3);
    EXPECT_EQ(a, 255);
    EXPECT_DOUBLE_EQ(out[0].position.x, 0.0);
    EXPECT_DOUBLE_EQ(out[0].position.y, 0.0);
    EXPECT_DOUBLE_EQ(out[0].position.z, 5.0);
}

TEST(LidarColorize, PointOutsideEveryCameraFovIsDropped) {
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

    const std::vector<overlume::Vec3> points{{0, 0, 5}, {1000, 1000, 5}};
    const std::vector<const uint8_t*> bufs{rgb.data()};

    const auto out = ColorizeFromCameras(points, cfg, bufs);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_DOUBLE_EQ(out[0].position.z, 5.0);
    EXPECT_DOUBLE_EQ(out[0].position.x, 0.0);
}

TEST(LidarColorize, PointVisibleToTwoCamerasPicksFirstConfiguredMatch) {
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
    EXPECT_EQ(r, 41);
    EXPECT_EQ(g, 41);
    EXPECT_EQ(b, 42);
}

TEST(LidarColorize, NullBufferForACoveringCameraFallsThroughToTheNextOne) {
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
    const std::vector<const uint8_t*> bufs{nullptr, rgb1.data()};

    const auto out = ColorizeFromCameras(points, cfg, bufs);
    ASSERT_EQ(out.size(), 1u);
    uint8_t r, g, b, a;
    Unpack(out[0].rgba, r, g, b, a);
    EXPECT_EQ(r, 1);
    EXPECT_EQ(g, 1);
    EXPECT_EQ(b, 1);
}
