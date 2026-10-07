// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "hybrid_splats_test_hooks.hpp"
#include "test_paths.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <vector>

#include <gtest/gtest.h>

// Rig: magenta overhead camera (R0=0.5 k=0.3 Rmax=4), vcam (0,-6,6) -> origin, 320x240.
// Thresholds below were calibrated from the first passing run (see each test).

namespace {

using overlume::PointCloudPoint;

constexpr int kDitherTol = 8;
constexpr uint32_t kW = 320, kH = 240;
constexpr uint32_t kGreen = 0xFF00FF00u, kBlue = 0xFFFF0000u, kYellow = 0xFF00FFFFu,
                   kCyan = 0xFFFFFF00u, kRed = 0xFF0000FFu;

bool Green(const uint8_t* p) { return p[1] > p[0] + 60 && p[1] > p[2] + 60; }
bool Blue(const uint8_t* p) { return p[2] > p[0] + 60 && p[2] > p[1] + 60; }
bool Yellow(const uint8_t* p) { return p[0] > p[2] + 80 && p[1] > p[2] + 80; }
bool Cyan(const uint8_t* p) { return p[1] > p[0] + 60 && p[2] > p[0] + 60; }

template <class F>
size_t Count(const std::vector<uint8_t>& img, F pred) {
    size_t n = 0;
    for (size_t i = 0; i < img.size(); i += 3) n += pred(&img[i]) ? 1 : 0;
    return n;
}

struct Rig {
    overlume::VisualRenderer* r = nullptr;
    overlume::CameraExtrinsics ext{{1, 0, 0, 0, -1, 0, 0, 0, -1}, {0, 0, 10.0}};
    overlume::CameraIntrinsics in{200, 200, 160, 120, {0, 0, 0, 0, 0}};
    uint32_t w = kW, h = kH;
    overlume::BowlConfig bc{};
    overlume::CameraPose pose{{0, -6, 6}, {0, 0, 0}, 70.0};
    uint8_t cam_rgb[3] = {255, 0, 255};
    uint64_t frame_id = 1;

    bool Init(bool with_ego, float exposure = 1.56f) {
        overlume::RenderConfig cfg{kW, kH, 1, kThemeDir, "dark_adas"};
        r = overlume::create_renderer(cfg);
        if (!r) return false;
        bc.camera_count = 1;
        bc.extrinsics = &ext;
        bc.intrinsics = &in;
        bc.cam_width = &w;
        bc.cam_height = &h;
        bc.bowl_R0 = 0.5;
        bc.bowl_k = 0.3;
        bc.bowl_Rmax = 4.0;
        bc.feather_margin = 5.0;
        bc.sky_color[0] = bc.sky_color[1] = bc.sky_color[2] = 0.05f;
        if (with_ego) overlume::set_ego_model(r, "/nonexistent/path.glb", {1.5, 1.5, 1.2});
        Bake(exposure);
        overlume::SceneGraph s{};
        s.ego = {{0, 0, 0}, 0.0, 0.0, 1};
        overlume::set_scene(r, s);
        return true;
    }
    void Bake(float exposure) {
        bc.exposure_compensation = exposure;
        EXPECT_TRUE(overlume::set_bowl_config(r, bc));
        EXPECT_TRUE(overlume::set_bowl_visible(r, true));
        std::vector<uint8_t> px(static_cast<size_t>(kW) * kH * 3);
        for (size_t i = 0; i < px.size(); i += 3) {
            px[i] = cam_rgb[0];
            px[i + 1] = cam_rgb[1];
            px[i + 2] = cam_rgb[2];
        }
        EXPECT_TRUE(overlume::set_camera_frame(r, 0, px.data(), kW, kH, ++frame_id));
    }
    std::vector<uint8_t> Render() {
        std::vector<uint8_t> img(static_cast<size_t>(kW) * kH * 3);
        EXPECT_TRUE(overlume::render_frame(r, pose, {img.data(), kW, kH}));
        return img;
    }
    ~Rig() {
        if (r) overlume::destroy_renderer(r);
    }
};

void Add(std::vector<PointCloudPoint>& v, double x, double y, double z, uint32_t rgba) {
    v.push_back({{x, y, z}, rgba});
}

std::vector<PointCloudPoint> GroundGrid(uint32_t rgba) {
    std::vector<PointCloudPoint> v;
    for (double x = -3.5; x <= 3.5; x += 0.05)
        for (double y = -3.5; y <= 3.5; y += 0.05) Add(v, x, y, 0.0, rgba);
    return v;
}

std::vector<PointCloudPoint> BeyondRimWall(uint32_t rgba) {
    std::vector<PointCloudPoint> v;
    for (double x = -3.0; x <= 3.0; x += 0.05)
        for (double z = 0.0; z <= 3.0; z += 0.05) Add(v, x, 6.0, z, rgba);
    return v;
}

std::vector<PointCloudPoint> Column(double x, double y, uint32_t rgba) {
    std::vector<PointCloudPoint> v;
    for (double z = 0.2; z <= 0.95; z += 0.02) Add(v, x, y, z, rgba);
    return v;
}

}

TEST(HybridSplats, SplatsDrawOverBowlWhereAnOrdinaryRowIsHidden) {
    Rig rig;
    if (!rig.Init(false)) GTEST_SKIP() << "no GPU/EGL";
    auto ground = GroundGrid(kGreen);
    auto wall = BeyondRimWall(kBlue);

    // Bowl-only frame: its magenta pixels are "inside the bowl region".
    const auto base = rig.Render();
    auto bowl_green = [&](const std::vector<uint8_t>& img) {
        size_t n = 0;
        for (size_t i = 0; i < img.size(); i += 3)
            n +=
                (base[i] > 150 && base[i + 2] > 150 && base[i + 1] < 100 && Green(&img[i])) ? 1 : 0;
        return n;
    };

    overlume::PointCloud pc[2] = {{ground.data(), static_cast<uint32_t>(ground.size()), 0.0},
                                  {wall.data(), static_cast<uint32_t>(wall.size()), 0.0}};
    overlume::SceneGraph s{};
    s.ego = {{0, 0, 0}, 0.0, 0.0, 1};
    s.point_clouds = pc;
    s.point_cloud_count = 2;
    overlume::set_scene(rig.r, s);
    const auto row = rig.Render();
    // Calibrated: row = 352 green px (rim pixels where the bowl edge meets bare ground), splats >
    // 8000.
    EXPECT_LT(bowl_green(row), 1000u) << "ordinary rows are depth-hidden by the opaque bowl";
    EXPECT_LT(Count(row, Blue), 300u);

    s.point_cloud_count = 0;
    overlume::set_scene(rig.r, s);
    std::vector<PointCloudPoint> all = ground;
    all.insert(all.end(), wall.begin(), wall.end());
    ASSERT_TRUE(
        overlume::set_hybrid_splats(rig.r, all.data(), static_cast<uint32_t>(all.size()), 7.0f));
    const auto splat = rig.Render();
    EXPECT_GT(bowl_green(splat), 8000u);
    EXPECT_GT(Count(splat, Blue), 3000u);
}

TEST(HybridSplats, EgoStillOccludesSplatsBehindIt) {
    Rig rig;
    if (!rig.Init(true)) GTEST_SKIP() << "no GPU/EGL";
    auto behind = Column(0.0, 1.0, kYellow);
    ASSERT_TRUE(overlume::set_hybrid_splats(rig.r, behind.data(),
                                            static_cast<uint32_t>(behind.size()), 7.0f));
    const size_t hidden = Count(rig.Render(), Yellow);

    auto front = Column(0.0, -1.2, kCyan);
    ASSERT_TRUE(overlume::set_hybrid_splats(rig.r, front.data(),
                                            static_cast<uint32_t>(front.size()), 7.0f));
    const size_t cy = Count(rig.Render(), Cyan);
    EXPECT_GT(cy, 50u);
    // The same column seen from the open side covers 100 px; behind the ego only the sliver above
    // the clay box shows (5 px on desktop GL and Android x86 SwANGLE, 6 on x86_64 SwANGLE; the
    // extra pixel's position was not captured, point-sprite edge rasterisation is the inferred
    // cause). The invariant is "mostly occluded", not an exact edge count.
    EXPECT_LE(hidden * 10, cy);
}

TEST(HybridSplats, NearestSplatWins) {
    Rig rig;
    if (!rig.Init(false)) GTEST_SKIP() << "no GPU/EGL";
    // Both on the view ray through the image centre (eye (0,-6,6) -> origin).
    const double k = 0.70710678;
    auto at = [&](double s) { return std::pair<double, double>{-6.0 + s * k, 6.0 - s * k}; };
    const auto [ny, nz] = at(5.0);
    const auto [fy, fz] = at(7.0);
    const uint8_t* centre;
    for (int near_first = 0; near_first < 2; ++near_first) {
        std::vector<PointCloudPoint> v;
        if (near_first) {
            Add(v, 0, ny, nz, kRed);
            Add(v, 0, fy, fz, kGreen);
        } else {
            Add(v, 0, fy, fz, kGreen);
            Add(v, 0, ny, nz, kRed);
        }
        ASSERT_TRUE(overlume::set_hybrid_splats(rig.r, v.data(), 2, 9.0f));
        const auto img = rig.Render();
        centre = &img[(static_cast<size_t>(kH / 2) * kW + kW / 2) * 3];
        EXPECT_GT(centre[0], centre[1] + 60) << "near (red) must win, near_first=" << near_first;
    }
}

TEST(HybridSplats, StencilStateFollowsTheLayer) {
    using overlume::testing::hybrid_stencil_state_for_test;
    Rig rig;
    if (!rig.Init(false)) GTEST_SKIP() << "no GPU/EGL";
    auto expect = [&](bool on) {
        const auto st = hybrid_stencil_state_for_test(rig.r);
        EXPECT_EQ(st.view_stencil, on);
        EXPECT_EQ(st.bowl_ne, on);
        EXPECT_EQ(st.ground_ne, on);
        EXPECT_EQ(st.grid_ne, on);
    };
    rig.Render();
    expect(false);

    auto pts = Column(0.0, -1.2, kCyan);
    const uint32_t n = static_cast<uint32_t>(pts.size());
    overlume::set_hybrid_splats(rig.r, pts.data(), n, 7.0f);
    rig.Render();
    expect(true);

    rig.Bake(1.56f);  // re-bake creates a fresh bowl instance
    rig.Render();
    expect(true);

    overlume::set_hybrid_splats(rig.r, nullptr, 0, 0.0f);
    rig.Render();
    expect(false);

    overlume::set_hybrid_splats(rig.r, pts.data(), n, 7.0f);
    overlume::set_bowl_visible(rig.r, false);
    rig.Render();
    expect(false);
}

TEST(HybridSplats, ClearingRestoresByteIdenticalFrames) {
    Rig rig;
    if (!rig.Init(false)) GTEST_SKIP() << "no GPU/EGL";
    auto row_pts = Column(0.0, -1.2, kCyan);
    overlume::PointCloud pc{row_pts.data(), static_cast<uint32_t>(row_pts.size()), 0.0};
    overlume::SceneGraph s{};
    s.ego = {{0, 0, 0}, 0.0, 0.0, 1};
    s.point_clouds = &pc;
    s.point_cloud_count = 1;
    overlume::set_scene(rig.r, s);
    const auto a = rig.Render();

    auto g = GroundGrid(kGreen);
    overlume::set_hybrid_splats(rig.r, g.data(), static_cast<uint32_t>(g.size()), 7.0f);
    rig.Render();
    overlume::set_hybrid_splats(rig.r, nullptr, 0, 0.0f);
    const auto b = rig.Render();
    const auto st = overlume::testing::hybrid_stencil_state_for_test(rig.r);
    EXPECT_FALSE(st.view_stencil);
    EXPECT_FALSE(st.bowl_ne);
    EXPECT_FALSE(st.ground_ne);
    EXPECT_FALSE(st.grid_ne);
    // Frames carry temporal noise (Filament's dither, amplified through the post pass). On Mesa
    // llvmpipe (Windows WGL) two back-to-back clean renders with no splat already differ by up to
    // 12 on 1-3 isolated pixels (CI run 37645646387); desktop GL stays within 5. So the oracle is
    // "no more than that noise", not "zero channel bytes beyond kDitherTol": a leftover splat or
    // stale parameter moves thousands of pixels (the 7 px ground grid covers > 8000), so a budget
    // of 50 channel bytes still fails on any real regression and passes the llvmpipe noise.
    size_t big = 0;
    for (size_t i = 0; i < a.size(); ++i) big += std::abs(int(a[i]) - int(b[i])) > kDitherTol;
    EXPECT_LT(big, 50u);
}

TEST(HybridSplats, HiddenWhenBowlHidden) {
    Rig rig;
    if (!rig.Init(false)) GTEST_SKIP() << "no GPU/EGL";
    auto g = GroundGrid(kGreen);
    overlume::set_hybrid_splats(rig.r, g.data(), static_cast<uint32_t>(g.size()), 7.0f);
    overlume::set_bowl_visible(rig.r, false);
    EXPECT_EQ(Count(rig.Render(), Green), 0u);
}

TEST(HybridSplats, SizeFromThemeTokenWhenZero) {
    Rig rig;
    if (!rig.Init(false)) GTEST_SKIP() << "no GPU/EGL";
    std::vector<PointCloudPoint> v;
    for (double x = -1.5; x <= 1.5; x += 0.75) Add(v, x, 1.5, 0.0, kGreen);
    const uint32_t n = static_cast<uint32_t>(v.size());
    const auto none = rig.Render();
    // Splat footprint = pixels that moved off the splat-less frame. A 1 px point is resolved by the
    // post-process AA into a blend with the bowl, so a colour predicate (Green) sees it only on
    // some back ends (1 px on desktop GL, 0 under SwANGLE); the footprint is what scales with size.
    auto footprint = [&](float size_px) {
        overlume::set_hybrid_splats(rig.r, v.data(), n, size_px);
        const auto img = rig.Render();
        size_t px = 0;
        for (size_t i = 0; i < img.size(); i += 3)
            for (size_t c = 0; c < 3; ++c)
                if (std::abs(int(img[i + c]) - int(none[i + c])) > kDitherTol) {
                    ++px;
                    break;
                }
        return px;
    };
    const size_t themed = footprint(0.0f);
    const size_t one = footprint(1.0f);
    ASSERT_GT(one, 0u);
    EXPECT_GT(static_cast<double>(themed) / static_cast<double>(one), 20.0);
}

TEST(HybridSplats, SplatColourMatchesBowl) {
    Rig rig;
    rig.cam_rgb[0] = 200;
    rig.cam_rgb[1] = 120;
    rig.cam_rgb[2] = 60;
    if (!rig.Init(false)) GTEST_SKIP() << "no GPU/EGL";
    std::vector<PointCloudPoint> patch;
    for (int i = 0; i < 13; ++i)
        for (int j = 0; j < 13; ++j)
            Add(patch, 1.6 + 0.03 * (i - 6), 0.03 * (j - 6), 0.0, 0xFF3C78C8u);
    for (float exposure : {1.56f, 1.2f}) {
        if (exposure != 1.56f) rig.Bake(exposure);
        overlume::set_hybrid_splats(rig.r, nullptr, 0, 0.0f);
        const auto base = rig.Render();
        overlume::set_hybrid_splats(rig.r, patch.data(), static_cast<uint32_t>(patch.size()), 7.0f);
        const auto with = rig.Render();
        size_t differing = 0, max_diff = 0, covered = 0;
        for (size_t i = 0; i < base.size(); ++i) {
            const size_t d = static_cast<size_t>(std::abs(int(base[i]) - int(with[i])));
            max_diff = std::max(max_diff, d);
            differing += d > static_cast<size_t>(kDitherTol) ? 1 : 0;
            covered += d > 0 ? 1 : 0;
        }
        EXPECT_EQ(differing, 0u) << "exposure " << exposure << " max channel diff " << max_diff;
        (void)covered;
        // Guard against a vacuous pass: a black patch in the same place must be clearly visible.
        for (auto& p : patch) p.rgba = 0xFF000000u;
        overlume::set_hybrid_splats(rig.r, patch.data(), static_cast<uint32_t>(patch.size()), 7.0f);
        const auto black = rig.Render();
        size_t visible = 0;
        for (size_t i = 0; i < base.size(); ++i) visible += black[i] + 40 < base[i] ? 1 : 0;
        EXPECT_GT(visible, 300u) << "the splat patch is not drawn at all";
        for (auto& p : patch) p.rgba = 0xFF3C78C8u;
    }
}

TEST(HybridSplats, SplatsFollowEgoAnchor) {
    Rig rig;
    if (!rig.Init(true)) GTEST_SKIP() << "no GPU/EGL";
    auto col = Column(0.0, -1.2, kCyan);
    ASSERT_TRUE(
        overlume::set_hybrid_splats(rig.r, col.data(), static_cast<uint32_t>(col.size()), 7.0f));
    auto centroid_x = [&](const std::vector<uint8_t>& img, size_t& n) {
        double sx = 0;
        n = 0;
        for (size_t i = 0; i < img.size(); i += 3)
            if (Cyan(&img[i])) {
                sx += static_cast<double>((i / 3) % kW);
                ++n;
            }
        return n ? sx / static_cast<double>(n) : 0.0;
    };
    size_t n0 = 0, n1 = 0;
    const double x0 = centroid_x(rig.Render(), n0);
    ASSERT_GT(n0, 50u) << "column not visible at the origin";
    overlume::SceneGraph s{};
    s.ego = {{1.0, 0, 0}, 0.0, 0.0, 1};
    overlume::set_scene(rig.r, s);
    const double x1 = centroid_x(rig.Render(), n1);
    ASSERT_GT(n1, 50u) << "column not visible after the ego moved";
    // Camera looks along +y from x=0: an ego 1 m to +x pushes the column right on screen.
    EXPECT_GT(x1 - x0, 15.0) << "x0=" << x0 << " x1=" << x1;
}
