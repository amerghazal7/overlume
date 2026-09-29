// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/point_cloud.hpp"

#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/clock.hpp>
#include <tf2_ros/buffer.h>

#include "overlume/api.h"
#include "overlume/scene.h"

using overlume::ros::FrameTransformer;
using overlume::ros::SceneAssembly;

namespace {

struct TfFixture {
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer{clock};
    FrameTransformer tf{buffer};
};

overlume::ros::ProfileRow MakeRow(const std::string& color_mode = "auto", uint32_t max_points = 0,
                                  uint32_t stride = 1) {
    overlume::ros::ProfileRow row;
    row.topic = "/lidar/points";
    row.type = "sensor_msgs/msg/PointCloud2";
    row.adapter = "point_cloud";
    row.role = "points";
    row.color_mode = color_mode;
    row.max_points = max_points;
    row.stride = stride;
    return row;
}

sensor_msgs::msg::PointField MakeField(const std::string& name, uint32_t offset) {
    sensor_msgs::msg::PointField f;
    f.name = name;
    f.offset = offset;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    return f;
}

float BitsAsFloat(uint32_t bits) {
    float f;
    std::memcpy(&f, &bits, 4);
    return f;
}

float PackRgbFloat(uint8_t r, uint8_t g, uint8_t b) {
    const uint32_t bits = (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) |
                          static_cast<uint32_t>(b);
    return BitsAsFloat(bits);
}

struct SyntheticPoint {
    float x, y, z;
    float extra = 0.0f;
};

sensor_msgs::msg::PointCloud2 BuildCloud(const std::vector<SyntheticPoint>& pts,
                                         const std::string& extra_name) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = "map";
    msg.height = 1;
    msg.width = static_cast<uint32_t>(pts.size());
    msg.fields = {MakeField("x", 0), MakeField("y", 4), MakeField("z", 8)};
    uint32_t point_step = 12;
    const bool has_extra = !extra_name.empty();
    if (has_extra) {
        msg.fields.push_back(MakeField(extra_name, 12));
        point_step = 16;
    }
    msg.point_step = point_step;
    msg.row_step = point_step * msg.width;
    msg.is_dense = true;
    msg.is_bigendian = false;
    msg.data.resize(static_cast<size_t>(point_step) * pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
        uint8_t* rec = msg.data.data() + i * point_step;
        std::memcpy(rec + 0, &pts[i].x, 4);
        std::memcpy(rec + 4, &pts[i].y, 4);
        std::memcpy(rec + 8, &pts[i].z, 4);
        if (has_extra) std::memcpy(rec + 12, &pts[i].extra, 4);
    }
    return msg;
}

void Unpack(uint32_t rgba, uint8_t* r, uint8_t* g, uint8_t* b, uint8_t* a) {
    *r = static_cast<uint8_t>(rgba & 0xFFu);
    *g = static_cast<uint8_t>((rgba >> 8) & 0xFFu);
    *b = static_cast<uint8_t>((rgba >> 16) & 0xFFu);
    *a = static_cast<uint8_t>((rgba >> 24) & 0xFFu);
}

const overlume::PointCloud* OnlyCloud(const SceneAssembly& asm_) {
    return asm_.point_clouds.size() == 1 ? &asm_.point_clouds[0] : nullptr;
}

}

TEST(PointCloudAdapter, AutoModePicksRgbWhenFieldPresent) {
    std::vector<SyntheticPoint> pts = {
        {0.0f, 0.0f, 0.0f, PackRgbFloat(200, 10, 10)},
        {1.0f, 0.0f, 0.0f, PackRgbFloat(10, 200, 10)},
    };
    auto msg = BuildCloud(pts, "rgb");
    TfFixture kTf;
    overlume::ros::PointCloudAdapter a(MakeRow("auto"), kTf.tf);
    a.ingest(msg, 1.0);
    ASSERT_EQ(a.stats().msgs, 1u);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::PointCloud* pc = OnlyCloud(asm_);
    ASSERT_NE(pc, nullptr);
    ASSERT_EQ(pc->point_count, 2u);
    uint8_t r, g, b, alpha;
    Unpack(pc->points[0].rgba, &r, &g, &b, &alpha);
    EXPECT_EQ(r, 200);
    EXPECT_EQ(g, 10);
    EXPECT_EQ(b, 10);
    EXPECT_EQ(alpha, 255) << "a real per-point color is always opaque (a==0 is the flat sentinel)";
}

TEST(PointCloudAdapter, AutoModeFallsBackToIntensityRampWhenNoRgbFieldExists) {
    std::vector<SyntheticPoint> pts = {
        {0.0f, 0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f, 10.0f},
    };
    auto msg = BuildCloud(pts, "intensity");
    TfFixture kTf;
    overlume::ros::PointCloudAdapter a(MakeRow("auto"), kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::PointCloud* pc = OnlyCloud(asm_);
    ASSERT_NE(pc, nullptr);
    ASSERT_EQ(pc->point_count, 2u);
    uint8_t r0, g0, b0, a0, r1, g1, b1, a1;
    Unpack(pc->points[0].rgba, &r0, &g0, &b0, &a0);
    Unpack(pc->points[1].rgba, &r1, &g1, &b1, &a1);
    EXPECT_EQ(a0, 255);
    EXPECT_EQ(a1, 255);
    EXPECT_FALSE(r0 == r1 && g0 == g1 && b0 == b1)
        << "min/max intensity points must land at different ramp colors";
}

TEST(PointCloudAdapter, AutoModeFallsBackToHeightRampWhenNeitherRgbNorIntensityExists) {
    std::vector<SyntheticPoint> pts = {
        {0.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 10.0f, 0.0f},
    };
    auto msg = BuildCloud(pts, "");
    TfFixture kTf;
    overlume::ros::PointCloudAdapter a(MakeRow("auto"), kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::PointCloud* pc = OnlyCloud(asm_);
    ASSERT_NE(pc, nullptr);
    ASSERT_EQ(pc->point_count, 2u);
    uint8_t r0, g0, b0, a0, r1, g1, b1, a1;
    Unpack(pc->points[0].rgba, &r0, &g0, &b0, &a0);
    Unpack(pc->points[1].rgba, &r1, &g1, &b1, &a1);
    EXPECT_EQ(a0, 255);
    EXPECT_EQ(a1, 255);
    EXPECT_FALSE(r0 == r1 && g0 == g1 && b0 == b1)
        << "min/max height points must land at different ramp colors";
}

TEST(PointCloudAdapter, IntensityRangeAutoRangesWhenRowLeavesItUnset) {
    std::vector<SyntheticPoint> pts = {
        {0.0f, 0.0f, 0.0f, 500.0f},
        {1.0f, 0.0f, 0.0f, 750.0f},
        {2.0f, 0.0f, 0.0f, 1000.0f},
    };
    auto msg = BuildCloud(pts, "intensity");
    TfFixture kTf;
    overlume::ros::PointCloudAdapter a(MakeRow("intensity"), kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::PointCloud* pc = OnlyCloud(asm_);
    ASSERT_NE(pc, nullptr);
    ASSERT_EQ(pc->point_count, 3u);
    uint8_t rLo, gLo, bLo, aLo, rMid, gMid, bMid, aMid, rHi, gHi, bHi, aHi;
    Unpack(pc->points[0].rgba, &rLo, &gLo, &bLo, &aLo);
    Unpack(pc->points[1].rgba, &rMid, &gMid, &bMid, &aMid);
    Unpack(pc->points[2].rgba, &rHi, &gHi, &bHi, &aHi);
    EXPECT_NEAR(static_cast<double>(rMid), (static_cast<double>(rLo) + rHi) / 2.0, 2.0);
    EXPECT_NEAR(static_cast<double>(gMid), (static_cast<double>(gLo) + gHi) / 2.0, 2.0);
    EXPECT_NEAR(static_cast<double>(bMid), (static_cast<double>(bLo) + bHi) / 2.0, 2.0);
}

TEST(PointCloudAdapter, DecimationRespectsMaxPointsAndStride) {
    std::vector<SyntheticPoint> pts;
    for (int i = 0; i < 10; ++i) {
        pts.push_back(
            {static_cast<float>(i), 0.0f, 0.0f, PackRgbFloat(static_cast<uint8_t>(i * 20), 0, 0)});
    }
    auto msg = BuildCloud(pts, "rgb");
    TfFixture kTf;
    overlume::ros::PointCloudAdapter a(MakeRow("rgb", 3, 2), kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::PointCloud* pc = OnlyCloud(asm_);
    ASSERT_NE(pc, nullptr);
    ASSERT_EQ(pc->point_count, 3u);
    EXPECT_DOUBLE_EQ(pc->points[0].position.x, 0.0);
    EXPECT_DOUBLE_EQ(pc->points[1].position.x, 2.0);
    EXPECT_DOUBLE_EQ(pc->points[2].position.x, 4.0);
}

TEST(PointCloudAdapter, MissingXyzFieldDropsWholeMessage) {
    std::vector<SyntheticPoint> pts = {{0, 0, 0, 0}};
    auto msg = BuildCloud(pts, "");
    msg.fields.erase(msg.fields.begin() + 2);
    TfFixture kTf;
    overlume::ros::PointCloudAdapter a(MakeRow(), kTf.tf);
    a.ingest(msg, 1.0);
    EXPECT_EQ(a.stats().dropped_malformed, 1u);

    SceneAssembly asm_;
    a.fill(asm_);
    EXPECT_TRUE(asm_.point_clouds.empty());
}

TEST(PointCloudAdapter, MinZDropsPointsBelowTheThreshold) {
    std::vector<SyntheticPoint> pts = {
        {0.0f, 0.0f, -0.5f, 0.0f},
        {1.0f, 0.0f, 0.1f, 0.0f},
        {2.0f, 0.0f, 0.5f, 0.0f},
    };
    auto msg = BuildCloud(pts, "");
    TfFixture kTf;
    auto row = MakeRow();
    row.min_z_m = 0.2;
    overlume::ros::PointCloudAdapter a(row, kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::PointCloud* pc = OnlyCloud(asm_);
    ASSERT_NE(pc, nullptr);
    ASSERT_EQ(pc->point_count, 1u);
    EXPECT_DOUBLE_EQ(pc->points[0].position.z, 0.5);
    EXPECT_EQ(a.dropped_below_min_z(), 2u);
}

TEST(PointCloudAdapter, MinZNaNDefaultKeepsEveryPoint) {
    std::vector<SyntheticPoint> pts = {
        {0.0f, 0.0f, -0.5f, 0.0f},
        {1.0f, 0.0f, 0.1f, 0.0f},
        {2.0f, 0.0f, 0.5f, 0.0f},
    };
    auto msg = BuildCloud(pts, "");
    TfFixture kTf;
    overlume::ros::PointCloudAdapter a(MakeRow(), kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::PointCloud* pc = OnlyCloud(asm_);
    ASSERT_NE(pc, nullptr);
    EXPECT_EQ(pc->point_count, 3u);
    EXPECT_EQ(a.dropped_below_min_z(), 0u);
}

TEST(PointCloudAdapter, FlatModeBakesTheAlphaZeroSentinel) {
    std::vector<SyntheticPoint> pts = {{0, 0, 0, 0}, {1, 0, 0, 0}};
    auto msg = BuildCloud(pts, "");
    TfFixture kTf;
    overlume::ros::PointCloudAdapter a(MakeRow("flat"), kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::PointCloud* pc = OnlyCloud(asm_);
    ASSERT_NE(pc, nullptr);
    for (uint32_t i = 0; i < pc->point_count; ++i) {
        EXPECT_EQ(pc->points[i].rgba, 0u)
            << "flat mode bakes rgba==0 -- point_cloud.cpp substitutes the theme token";
    }
}

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

namespace {

constexpr uint32_t kGoldenWidth = 320;
constexpr uint32_t kGoldenHeight = 240;

double Luminance(uint8_t r, uint8_t g, uint8_t b) { return 0.2126 * r + 0.7152 * g + 0.0722 * b; }

double BlockSsim(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b, uint32_t width,
                 uint32_t height) {
    constexpr int kBlock = 8;
    constexpr double kC1 = (0.01 * 255) * (0.01 * 255);
    constexpr double kC2 = (0.03 * 255) * (0.03 * 255);
    double total = 0.0;
    int blockCount = 0;
    for (uint32_t by = 0; by + kBlock <= height; by += kBlock) {
        for (uint32_t bx = 0; bx + kBlock <= width; bx += kBlock) {
            double sumA = 0, sumB = 0, sumAA = 0, sumBB = 0, sumAB = 0;
            const int n = kBlock * kBlock;
            for (int y = 0; y < kBlock; ++y) {
                for (int x = 0; x < kBlock; ++x) {
                    const uint32_t px = bx + x, py = by + y;
                    const size_t idx = (static_cast<size_t>(py) * width + px) * 3;
                    const double la = Luminance(a[idx], a[idx + 1], a[idx + 2]);
                    const double lb = Luminance(b[idx], b[idx + 1], b[idx + 2]);
                    sumA += la;
                    sumB += lb;
                    sumAA += la * la;
                    sumBB += lb * lb;
                    sumAB += la * lb;
                }
            }
            const double meanA = sumA / n, meanB = sumB / n;
            const double varA = sumAA / n - meanA * meanA;
            const double varB = sumBB / n - meanB * meanB;
            const double covAB = sumAB / n - meanA * meanB;
            const double ssim = ((2 * meanA * meanB + kC1) * (2 * covAB + kC2)) /
                                ((meanA * meanA + meanB * meanB + kC1) * (varA + varB + kC2));
            total += ssim;
            ++blockCount;
        }
    }
    return blockCount > 0 ? total / blockCount : 0.0;
}

std::vector<SyntheticPoint> MakeGoldenGrid(const std::string& extra_name) {
    std::vector<SyntheticPoint> pts;
    constexpr int kN = 25;
    constexpr float kExtentM = 6.0f;
    for (int iy = 0; iy < kN; ++iy) {
        for (int ix = 0; ix < kN; ++ix) {
            const float x = (static_cast<float>(ix) / (kN - 1) * 2.0f - 1.0f) * kExtentM;
            const float y = (static_cast<float>(iy) / (kN - 1) * 2.0f - 1.0f) * kExtentM;
            const float dist = std::sqrt(x * x + y * y);
            const float z = 2.0f * std::exp(-dist * dist / 12.0f);
            float extra = 0.0f;
            if (extra_name == "rgb") {
                const uint8_t r = static_cast<uint8_t>(128 + 127 * std::sin(x));
                const uint8_t g = static_cast<uint8_t>(128 + 127 * std::sin(y));
                const uint8_t b = static_cast<uint8_t>(128 + 127 * std::cos(dist));
                extra = PackRgbFloat(r, g, b);
            } else if (extra_name == "intensity") {
                extra = dist;
            }
            pts.push_back({x, y, z, extra});
        }
    }
    return pts;
}

void RenderGoldenAndAssert(const std::string& tier_name, const std::string& extra_field_name) {
    auto msg = BuildCloud(MakeGoldenGrid(extra_field_name), extra_field_name);
    TfFixture kTf;
    overlume::ros::PointCloudAdapter adapter(MakeRow("auto"), kTf.tf);
    adapter.ingest(msg, 1.0);

    SceneAssembly asm_;
    adapter.fill(asm_);
    ASSERT_EQ(asm_.point_clouds.size(), 1u);

    overlume::RenderConfig cfg{};
    cfg.width = kGoldenWidth;
    cfg.height = kGoldenHeight;
    cfg.quality = 1;
    cfg.theme_assets_dir = nullptr;
    cfg.initial_theme = nullptr;
    overlume::VisualRenderer* r = overlume::create_renderer(cfg);
    ASSERT_NE(r, nullptr) << "no GPU/EGL";

    overlume::SceneGraph scene{};
    scene.sim_time_sec = 1.0;
    scene.ego.valid = 0;
    asm_.point_at(scene);
    overlume::set_scene(r, scene);

    std::vector<uint8_t> frame(static_cast<size_t>(kGoldenWidth) * kGoldenHeight * 3, 0);
    overlume::FrameView view{frame.data(), kGoldenWidth, kGoldenHeight};
    overlume::CameraPose pose{{0.0, -14.0, 12.0}, {0.0, 0.0, 0.0}, 60.0};
    ASSERT_TRUE(overlume::render_frame(r, pose, view));

    const std::string actual_path = "/tmp/point_cloud_" + tier_name + "_actual.png";
    stbi_write_png(actual_path.c_str(), static_cast<int>(kGoldenWidth),
                   static_cast<int>(kGoldenHeight), 3, frame.data(),
                   static_cast<int>(kGoldenWidth) * 3);

    const std::string golden_path =
        std::string(OVERLUME_NODE_FIXTURES_DIR) + "/point_cloud_" + tier_name + "_golden.png";
    int golden_w = 0, golden_h = 0, golden_c = 0;
    uint8_t* golden = stbi_load(golden_path.c_str(), &golden_w, &golden_h, &golden_c, 3);
    double ssim = 0.0;
    if (golden != nullptr && static_cast<uint32_t>(golden_w) == kGoldenWidth &&
        static_cast<uint32_t>(golden_h) == kGoldenHeight) {
        const std::vector<uint8_t> golden_pixels(
            golden, golden + static_cast<size_t>(golden_w) * golden_h * 3);
        ssim = BlockSsim(frame, golden_pixels, kGoldenWidth, kGoldenHeight);
    }
    if (golden != nullptr) stbi_image_free(golden);

    EXPECT_GT(ssim, 0.98) << "actual frame written to " << actual_path << " -- promote to "
                          << golden_path << " once reviewed";

    overlume::destroy_renderer(r);
}

}

TEST(PointCloudGolden, RgbTier_DarkAdas) { RenderGoldenAndAssert("rgb", "rgb"); }
TEST(PointCloudGolden, IntensityTier_DarkAdas) { RenderGoldenAndAssert("intensity", "intensity"); }
TEST(PointCloudGolden, HeightTier_DarkAdas) { RenderGoldenAndAssert("height", ""); }

TEST(PointCloudAdapter, FrameIdOverrideReplacesAMislabelledHeaderFrame) {
    std::vector<SyntheticPoint> pts = {{2.0f, 1.0f, 0.5f, 0.0f}};
    auto msg = BuildCloud(pts, "");
    msg.header.frame_id = "base_link";
    TfFixture kTf;
    geometry_msgs::msg::TransformStamped lidar;
    lidar.header.frame_id = "map";
    lidar.child_frame_id = "seyond";
    lidar.transform.rotation.z = 1.0;
    lidar.transform.rotation.w = 0.0;
    kTf.buffer.setTransform(lidar, "test", true);
    geometry_msgs::msg::TransformStamped base;
    base.header.frame_id = "map";
    base.child_frame_id = "base_link";
    base.transform.rotation.w = 1.0;
    kTf.buffer.setTransform(base, "test", true);

    auto labelled = MakeRow();
    overlume::ros::PointCloudAdapter plain(labelled, kTf.tf);
    plain.ingest(msg, 1.0);
    SceneAssembly a1;
    plain.fill(a1);
    ASSERT_NE(OnlyCloud(a1), nullptr);
    EXPECT_NEAR(OnlyCloud(a1)->points[0].position.x, 2.0, 1e-6) << "trusts the header frame";

    auto overridden = MakeRow();
    overridden.frame_id = "seyond";
    overlume::ros::PointCloudAdapter fixed(overridden, kTf.tf);
    fixed.ingest(msg, 1.0);
    SceneAssembly a2;
    fixed.fill(a2);
    const overlume::PointCloud* pc = OnlyCloud(a2);
    ASSERT_NE(pc, nullptr);
    EXPECT_NEAR(pc->points[0].position.x, -2.0, 1e-6)
        << "frame_id: seyond must apply the 180-degree seyond transform, not base_link's";
    EXPECT_NEAR(pc->points[0].position.y, -1.0, 1e-6);
}
