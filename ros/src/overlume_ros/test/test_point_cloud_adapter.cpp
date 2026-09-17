/** @file test_point_cloud_adapter.cpp
 *  @brief PointCloudAdapter tests + the three synthetic-scene point-cloud
 *  goldens (Epic 3 Task 6 / VM-035, Steps 3-4).
 *
 *  FIXTURE GAP (this task, epic3 plan): zero sensor_msgs/PointCloud2 topics
 *  exist in any recording -- every fixture below is hand-built synthetic,
 *  over three named PointCloud2 shapes (rgb-carrying / intensity-only /
 *  bare-XYZ), since none exist in any recording.
 */
#include "overlume_ros/adapters/point_cloud.hpp"

#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/clock.hpp>
#include <tf2_ros/buffer.h>

#include "overlume/api.h"
#include "overlume/scene.h"

using overlume::ros::FrameTransformer;
using overlume::ros::SceneAssembly;

namespace
{

// Hand-built tf2_ros::Buffer + FrameTransformer -- an empty buffer is
// enough (every fixture below stays in the "map" frame, FrameTransformer's
// identity shortcut). Same fixture style as test_ogm_adapter.cpp/
// test_collision_adapter.cpp.
struct TfFixture
{
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer{clock};
    FrameTransformer tf{buffer};
};

// No shipped profile carries a live adapter: point_cloud row (FIXTURE GAP)
// -- hand-built directly here, same "no urban_row()/sim_row() to borrow"
// shape test_tf_axes_adapter.cpp already uses for its own row.
overlume_node::ProfileRow MakeRow(const std::string& color_mode = "auto", uint32_t max_points = 0,
                               uint32_t stride = 1)
{
    overlume_node::ProfileRow row;
    row.topic = "/lidar/points";
    row.type = "sensor_msgs/msg/PointCloud2";
    row.adapter = "point_cloud";
    row.role = "points";
    row.color_mode = color_mode;
    row.max_points = max_points;
    row.stride = stride;
    return row;
}

sensor_msgs::msg::PointField MakeField(const std::string& name, uint32_t offset)
{
    sensor_msgs::msg::PointField f;
    f.name = name;
    f.offset = offset;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    return f;
}

float BitsAsFloat(uint32_t bits)
{
    float f;
    std::memcpy(&f, &bits, 4);
    return f;
}

// PCL packed-float convention (this adapter's own header comment): the
// field's bit pattern, reinterpreted as a uint32_t, is 0x00RRGGBB.
float PackRgbFloat(uint8_t r, uint8_t g, uint8_t b)
{
    const uint32_t bits = (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) |
                          static_cast<uint32_t>(b);
    return BitsAsFloat(bits);
}

struct SyntheticPoint
{
    float x, y, z;
    float extra = 0.0f;  // rgb (packed float) or intensity, per `extra_name`
};

// Builds a synthetic PointCloud2: always x/y/z (FLOAT32), plus one more
// FLOAT32 field named `extra_name` when non-empty (rgb/rgba/intensity) --
// the three named fixture shapes this task's FIXTURE GAP requires (rgb-
// carrying / intensity-only / bare-XYZ) are just this helper called with
// "rgb", "intensity", or "" respectively.
sensor_msgs::msg::PointCloud2 BuildCloud(const std::vector<SyntheticPoint>& pts,
                                          const std::string& extra_name)
{
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = "map";
    msg.height = 1;
    msg.width = static_cast<uint32_t>(pts.size());
    msg.fields = {MakeField("x", 0), MakeField("y", 4), MakeField("z", 8)};
    uint32_t point_step = 12;
    const bool has_extra = !extra_name.empty();
    if (has_extra)
    {
        msg.fields.push_back(MakeField(extra_name, 12));
        point_step = 16;
    }
    msg.point_step = point_step;
    msg.row_step = point_step * msg.width;
    msg.is_dense = true;
    msg.is_bigendian = false;
    msg.data.resize(static_cast<size_t>(point_step) * pts.size());
    for (size_t i = 0; i < pts.size(); ++i)
    {
        uint8_t* rec = msg.data.data() + i * point_step;
        std::memcpy(rec + 0, &pts[i].x, 4);
        std::memcpy(rec + 4, &pts[i].y, 4);
        std::memcpy(rec + 8, &pts[i].z, 4);
        if (has_extra) std::memcpy(rec + 12, &pts[i].extra, 4);
    }
    return msg;
}

void Unpack(uint32_t rgba, uint8_t* r, uint8_t* g, uint8_t* b, uint8_t* a)
{
    *r = static_cast<uint8_t>(rgba & 0xFFu);
    *g = static_cast<uint8_t>((rgba >> 8) & 0xFFu);
    *b = static_cast<uint8_t>((rgba >> 16) & 0xFFu);
    *a = static_cast<uint8_t>((rgba >> 24) & 0xFFu);
}

const overlume::PointCloud* OnlyCloud(const SceneAssembly& asm_)
{
    return asm_.point_clouds.size() == 1 ? &asm_.point_clouds[0] : nullptr;
}

}  // namespace

// ── Step 3: color_mode: auto tier fallback ──────────────────────────────────

TEST(PointCloudAdapter, AutoModePicksRgbWhenFieldPresent)
{
    std::vector<SyntheticPoint> pts = {
        {0.0f, 0.0f, 0.0f, PackRgbFloat(200, 10, 10)},
        {1.0f, 0.0f, 0.0f, PackRgbFloat(10, 200, 10)},
    };
    auto msg = BuildCloud(pts, "rgb");
    TfFixture kTf;
    overlume_node::PointCloudAdapter a(MakeRow("auto"), kTf.tf);
    a.ingest(msg, /*sim_time_sec=*/1.0);
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

TEST(PointCloudAdapter, AutoModeFallsBackToIntensityRampWhenNoRgbFieldExists)
{
    std::vector<SyntheticPoint> pts = {
        {0.0f, 0.0f, 0.0f, 0.0f},    // min intensity -> the ramp's low endpoint
        {1.0f, 0.0f, 0.0f, 10.0f},   // max intensity -> the ramp's high endpoint
    };
    auto msg = BuildCloud(pts, "intensity");
    TfFixture kTf;
    overlume_node::PointCloudAdapter a(MakeRow("auto"), kTf.tf);
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
    // The two endpoints must differ -- proves a real ramp was applied, not
    // one flat color regardless of intensity.
    EXPECT_FALSE(r0 == r1 && g0 == g1 && b0 == b1)
        << "min/max intensity points must land at different ramp colors";
}

TEST(PointCloudAdapter, AutoModeFallsBackToHeightRampWhenNeitherRgbNorIntensityExists)
{
    std::vector<SyntheticPoint> pts = {
        {0.0f, 0.0f, 0.0f, 0.0f},     // z=0, the ramp's low endpoint
        {0.0f, 0.0f, 10.0f, 0.0f},    // z=10, the ramp's high endpoint
    };
    auto msg = BuildCloud(pts, "");  // bare XYZ -- no rgb, no intensity field at all
    TfFixture kTf;
    overlume_node::PointCloudAdapter a(MakeRow("auto"), kTf.tf);
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

// ── Step 3: intensity auto-ranges per message, not over a fixed assumption ──

TEST(PointCloudAdapter, IntensityRangeAutoRangesWhenRowLeavesItUnset)
{
    // An arbitrary, non-[0,1]/non-[0,100] intensity range -- proves the
    // ramp is stretched over THIS message's own observed min/max, not some
    // hardcoded assumed scale (no profile-level intensity_range key exists
    // -- this task's own Files list adds no such field, see point_cloud.
    // hpp's header comment).
    std::vector<SyntheticPoint> pts = {
        {0.0f, 0.0f, 0.0f, 500.0f},
        {1.0f, 0.0f, 0.0f, 750.0f},
        {2.0f, 0.0f, 0.0f, 1000.0f},
    };
    auto msg = BuildCloud(pts, "intensity");
    TfFixture kTf;
    overlume_node::PointCloudAdapter a(MakeRow("intensity"), kTf.tf);
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
    // The midpoint (750, exactly halfway between the observed 500..1000
    // range) must land roughly halfway between the low/high endpoint
    // colors -- not near either extreme, which is what a wrong (e.g.
    // fixed-scale) range would produce here.
    EXPECT_NEAR(static_cast<double>(rMid), (static_cast<double>(rLo) + rHi) / 2.0, 2.0);
    EXPECT_NEAR(static_cast<double>(gMid), (static_cast<double>(gLo) + gHi) / 2.0, 2.0);
    EXPECT_NEAR(static_cast<double>(bMid), (static_cast<double>(bLo) + bHi) / 2.0, 2.0);
}

// ── Step 3: decimation ───────────────────────────────────────────────────────

TEST(PointCloudAdapter, DecimationRespectsMaxPointsAndStride)
{
    // 10 points, distinctly colored (index*20 in the red channel) so
    // survivors are identifiable; stride: 2 keeps indices 0,2,4,6,8;
    // max_points: 3 then caps that to the first three (0,2,4).
    std::vector<SyntheticPoint> pts;
    for (int i = 0; i < 10; ++i)
    {
        pts.push_back({static_cast<float>(i), 0.0f, 0.0f,
                       PackRgbFloat(static_cast<uint8_t>(i * 20), 0, 0)});
    }
    auto msg = BuildCloud(pts, "rgb");
    TfFixture kTf;
    overlume_node::PointCloudAdapter a(MakeRow("rgb", /*max_points=*/3, /*stride=*/2), kTf.tf);
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

// ── Step 3: malformed / no-tf guards ─────────────────────────────────────────

TEST(PointCloudAdapter, MissingXyzFieldDropsWholeMessage)
{
    std::vector<SyntheticPoint> pts = {{0, 0, 0, 0}};
    auto msg = BuildCloud(pts, "");
    msg.fields.erase(msg.fields.begin() + 2);  // drop "z"
    TfFixture kTf;
    overlume_node::PointCloudAdapter a(MakeRow(), kTf.tf);
    a.ingest(msg, 1.0);
    EXPECT_EQ(a.stats().dropped_malformed, 1u);

    SceneAssembly asm_;
    a.fill(asm_);
    EXPECT_TRUE(asm_.point_clouds.empty());
}

TEST(PointCloudAdapter, FlatModeBakesTheAlphaZeroSentinel)
{
    std::vector<SyntheticPoint> pts = {{0, 0, 0, 0}, {1, 0, 0, 0}};
    auto msg = BuildCloud(pts, "");
    TfFixture kTf;
    overlume_node::PointCloudAdapter a(MakeRow("flat"), kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::PointCloud* pc = OnlyCloud(asm_);
    ASSERT_NE(pc, nullptr);
    for (uint32_t i = 0; i < pc->point_count; ++i)
    {
        EXPECT_EQ(pc->points[i].rgba, 0u)
            << "flat mode bakes rgba==0 -- point_cloud.cpp substitutes the theme token";
    }
}

// ── Step 4: three synthetic goldens, one per auto tier ──────────────────────
// UNPROMOTED, sanctioned red (same convention as test_hud_overlay.cpp's
// own golden): renders a real headless-EGL frame from the adapter's own
// output, writes it to /tmp/point_cloud_<tier>_actual.png, and SSIMs it
// against a golden this task deliberately does NOT commit. The user
// promotes each actual PNG into test/fixtures/point_cloud_<tier>_golden.png
// once satisfied with it.
//
// The stb_image/stb_image_write use below is this test binary's own
// vendored copy (OVERLUME_NODE_STB_DIR, CMakeLists.txt) -- NOT visual_
// renderer's, a different build tree entirely, pinned to the same upstream
// commit so the two are byte-identical (test_hud_overlay.cpp's own
// comment explains the split).
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

namespace
{

constexpr uint32_t kGoldenWidth = 320;
constexpr uint32_t kGoldenHeight = 240;

double Luminance(uint8_t r, uint8_t g, uint8_t b) { return 0.2126 * r + 0.7152 * g + 0.0722 * b; }

// ponytail: block-wise (8x8, non-overlapping, luminance-only) mean/
// variance/covariance SSIM -- same deliberately-simplified approximation
// as test_hud_overlay.cpp's own block_ssim (not shared code, see that
// file's comment for why each vendored-stb build tree reimplements this
// small helper rather than sharing one).
double BlockSsim(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b, uint32_t width,
                 uint32_t height)
{
    constexpr int kBlock = 8;
    constexpr double kC1 = (0.01 * 255) * (0.01 * 255);
    constexpr double kC2 = (0.03 * 255) * (0.03 * 255);
    double total = 0.0;
    int blockCount = 0;
    for (uint32_t by = 0; by + kBlock <= height; by += kBlock)
    {
        for (uint32_t bx = 0; bx + kBlock <= width; bx += kBlock)
        {
            double sumA = 0, sumB = 0, sumAA = 0, sumBB = 0, sumAB = 0;
            const int n = kBlock * kBlock;
            for (int y = 0; y < kBlock; ++y)
            {
                for (int x = 0; x < kBlock; ++x)
                {
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

// A 25x25 grid of points over a gentle "bump" surface (-6..6 m in x/y),
// carrying the named extra field -- one synthetic scene per FIXTURE GAP
// fixture shape, large enough to read as an actual point cloud rather than
// a couple of dots.
std::vector<SyntheticPoint> MakeGoldenGrid(const std::string& extra_name)
{
    std::vector<SyntheticPoint> pts;
    constexpr int kN = 25;
    constexpr float kExtentM = 6.0f;
    for (int iy = 0; iy < kN; ++iy)
    {
        for (int ix = 0; ix < kN; ++ix)
        {
            const float x = (static_cast<float>(ix) / (kN - 1) * 2.0f - 1.0f) * kExtentM;
            const float y = (static_cast<float>(iy) / (kN - 1) * 2.0f - 1.0f) * kExtentM;
            const float dist = std::sqrt(x * x + y * y);
            const float z = 2.0f * std::exp(-dist * dist / 12.0f);  // a gentle central bump
            float extra = 0.0f;
            if (extra_name == "rgb")
            {
                // Position-driven hue so the golden reads as an actual
                // colored surface, not a flat tint.
                const uint8_t r = static_cast<uint8_t>(128 + 127 * std::sin(x));
                const uint8_t g = static_cast<uint8_t>(128 + 127 * std::sin(y));
                const uint8_t b = static_cast<uint8_t>(128 + 127 * std::cos(dist));
                extra = PackRgbFloat(r, g, b);
            }
            else if (extra_name == "intensity")
            {
                extra = dist;  // auto-ranged by the adapter itself
            }
            pts.push_back({x, y, z, extra});
        }
    }
    return pts;
}

void RenderGoldenAndAssert(const std::string& tier_name, const std::string& extra_field_name)
{
    auto msg = BuildCloud(MakeGoldenGrid(extra_field_name), extra_field_name);
    TfFixture kTf;
    overlume_node::PointCloudAdapter adapter(MakeRow("auto"), kTf.tf);
    adapter.ingest(msg, /*sim_time_sec=*/1.0);

    SceneAssembly asm_;
    adapter.fill(asm_);
    ASSERT_EQ(asm_.point_clouds.size(), 1u);

    overlume::RenderConfig cfg{};
    cfg.width = kGoldenWidth;
    cfg.height = kGoldenHeight;
    cfg.quality = 1;
    cfg.theme_assets_dir = nullptr;  // compiled-in fallback theme (dark_adas-equivalent)
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
    double ssim = 0.0;  // no committed golden yet -- "missing golden -> 0.0" convention
    if (golden != nullptr && static_cast<uint32_t>(golden_w) == kGoldenWidth &&
        static_cast<uint32_t>(golden_h) == kGoldenHeight)
    {
        const std::vector<uint8_t> golden_pixels(
            golden, golden + static_cast<size_t>(golden_w) * golden_h * 3);
        ssim = BlockSsim(frame, golden_pixels, kGoldenWidth, kGoldenHeight);
    }
    if (golden != nullptr) stbi_image_free(golden);

    // SANCTIONED RED: unpromoted until a human looks at actual_path and
    // copies it to golden_path.
    EXPECT_GT(ssim, 0.98) << "actual frame written to " << actual_path << " -- promote to "
                          << golden_path << " once reviewed";

    overlume::destroy_renderer(r);
}

}  // namespace

TEST(PointCloudGolden, RgbTier_DarkAdas) { RenderGoldenAndAssert("rgb", "rgb"); }
TEST(PointCloudGolden, IntensityTier_DarkAdas) { RenderGoldenAndAssert("intensity", "intensity"); }
TEST(PointCloudGolden, HeightTier_DarkAdas) { RenderGoldenAndAssert("height", ""); }
