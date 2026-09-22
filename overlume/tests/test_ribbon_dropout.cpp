// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "ribbon_test_hooks.hpp"
#include "trajectory_carpet_test_hooks.hpp"
#include "test_paths.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

using overlume::PathRibbon;
using overlume::PathRole;
using overlume::PointCloudPoint;
using overlume::TrajectoryCarpet;
using overlume::Vec3;

std::vector<uint8_t> render_once(overlume::VisualRenderer* r, const overlume::CameraPose& pose) {
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    overlume::FrameView view{pixels.data(), 320, 240};
    EXPECT_TRUE(overlume::render_frame(r, pose, view));
    return pixels;
}

uint32_t pack_rgba(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    return static_cast<uint32_t>(r) | (static_cast<uint32_t>(g) << 8) |
           (static_cast<uint32_t>(b) << 16) | (static_cast<uint32_t>(a) << 24);
}

struct Rgb {
    int r = 0, g = 0, b = 0;
};

Rgb pixel_at(const std::vector<uint8_t>& px, int width, int x, int y) {
    const size_t i = (static_cast<size_t>(y) * width + x) * 3;
    return {px[i], px[i + 1], px[i + 2]};
}

std::vector<Vec3> make_spine(double startX, uint32_t n, double spacingM) {
    std::vector<Vec3> pts(n);
    for (uint32_t i = 0; i < n; ++i) pts[i] = Vec3{startX + i * spacingM, 0.0, 0.0};
    return pts;
}

std::vector<PointCloudPoint> make_carpet_points(const std::vector<Vec3>& spine,
                                                uint32_t colorTweak) {
    std::vector<PointCloudPoint> pts(spine.size());
    const uint32_t rgba = pack_rgba(255, static_cast<uint8_t>(120 + (colorTweak % 30)), 0, 255);
    for (size_t i = 0; i < spine.size(); ++i) {
        pts[i].position = spine[i];
        pts[i].rgba = rgba;
    }
    return pts;
}

struct ProbeLayout {
    int row = 0;
    int rimLeftCol = 0;
    int centerCol = 0;
    int rimRightCol = 0;
};

bool looks_like_corridor(const Rgb& c) { return c.r > c.b; }

ProbeLayout locate_probe(const std::vector<uint8_t>& px, int width, int height) {
    ProbeLayout best{};
    int bestRunLen = 0;
    for (int y = height / 4; y < (3 * height) / 4; y += 2) {
        int runStart = -1, runLen = 0, bestRowRunStart = -1, bestRowRunLen = 0;
        for (int x = 0; x < width; ++x) {
            const bool bright = looks_like_corridor(pixel_at(px, width, x, y));
            if (bright) {
                if (runStart < 0) runStart = x;
                ++runLen;
            } else {
                if (runLen > bestRowRunLen) {
                    bestRowRunLen = runLen;
                    bestRowRunStart = runStart;
                }
                runStart = -1;
                runLen = 0;
            }
        }
        if (runLen > bestRowRunLen) {
            bestRowRunLen = runLen;
            bestRowRunStart = runStart;
        }
        if (bestRowRunLen > bestRunLen) {
            bestRunLen = bestRowRunLen;
            best.row = y;
            best.rimLeftCol = bestRowRunStart + 2;
            best.rimRightCol = bestRowRunStart + bestRowRunLen - 3;
            best.centerCol = bestRowRunStart + bestRowRunLen / 2;
        }
    }
    return best;
}

constexpr size_t kRibbonRoleCount = 3;

struct FrameRecord {
    int frame = 0;
    bool ribbonRebuilt = false;
    bool carpetRebuilt = false;
    size_t ribbonMeshes[kRibbonRoleCount] = {};
    size_t ribbonVerts[kRibbonRoleCount] = {};
    size_t carpetMeshes = 0;
    size_t carpetVerts = 0;
    Rgb center{}, rimLeft{}, rimRight{};
    bool centerIsBackground = false;
    bool rimLeftIsBackground = false;
    bool rimRightIsBackground = false;
};

void print_trace(const FrameRecord& f) {
    fprintf(stderr,
            "frame %2d: ribbonRebuilt=%d carpetRebuilt=%d ribbonMesh=[%zu,%zu,%zu] "
            "carpetMesh=%zu ribbonVerts=[%zu,%zu,%zu] carpetVerts=%zu center=(%d,%d,%d)%s "
            "rimL=(%d,%d,%d)%s rimR=(%d,%d,%d)%s\n",
            f.frame, f.ribbonRebuilt, f.carpetRebuilt, f.ribbonMeshes[0], f.ribbonMeshes[1],
            f.ribbonMeshes[2], f.carpetMeshes, f.ribbonVerts[0], f.ribbonVerts[1], f.ribbonVerts[2],
            f.carpetVerts, f.center.r, f.center.g, f.center.b, f.centerIsBackground ? " BG!" : "",
            f.rimLeft.r, f.rimLeft.g, f.rimLeft.b, f.rimLeftIsBackground ? " BG!" : "",
            f.rimRight.r, f.rimRight.g, f.rimRight.b, f.rimRightIsBackground ? " BG!" : "");
}

struct RunResult {
    std::vector<FrameRecord> frames;
    uint64_t ribbonRebuildsTotal = 0;
    uint64_t carpetRebuildsTotal = 0;
    bool anyDropout = false;
};

RunResult drive(overlume::VisualRenderer* r, int numFrames, double stepMinM, double stepMaxM,
                uint32_t seed) {
    RunResult result;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> step(stepMinM, stepMaxM);

    std::vector<Vec3> spine = make_spine(0.0, 301, 0.5);
    double egoX = 5.0;
    ProbeLayout probe{};
    uint64_t prevRibbonRebuilds = 0, prevCarpetRebuilds = 0;

    for (int i = 0; i < numFrames; ++i) {
        egoX += step(rng);

        if (i > 0 && i % 8 == 0) {
            spine.back().x += 0.01;
        }
        std::vector<Vec3> spineCopy = spine;
        std::vector<PointCloudPoint> carpetPts =
            make_carpet_points(spineCopy, static_cast<uint32_t>(i));

        PathRibbon ribbons[kRibbonRoleCount]{};
        ribbons[0].role = PathRole::BEHAVIOR;
        ribbons[1].role = PathRole::GLOBAL;
        ribbons[2].role = PathRole::LOCAL;
        for (auto& ribbon : ribbons) {
            ribbon.points = spineCopy.data();
            ribbon.point_count = static_cast<uint32_t>(spineCopy.size());
            ribbon.last_update_sec = 0.0;
        }

        TrajectoryCarpet carpet{};
        carpet.points = carpetPts.data();
        carpet.point_count = static_cast<uint32_t>(carpetPts.size());
        carpet.last_update_sec = 0.0;

        overlume::SceneGraph s{};
        s.sim_time_sec = 0.0;
        s.ego = {{egoX, 0.0, 0.0}, 0.0, 0.0, 1};
        s.paths = ribbons;
        s.path_count = kRibbonRoleCount;
        s.trajectory_carpets = &carpet;
        s.trajectory_carpet_count = 1;
        overlume::set_scene(r, s);

        overlume::CameraPose pose{{egoX - 4.0, -8.0, 6.0}, {egoX + 4.0, 1.0, 0.0}, 60.0};
        std::vector<uint8_t> px = render_once(r, pose);

        const uint64_t ribbonRebuilds = overlume::testing::ribbon_rebuild_count(r);
        const uint64_t carpetRebuilds = overlume::testing::trajectory_carpet_rebuild_count(r);

        FrameRecord rec;
        rec.frame = i;
        rec.ribbonRebuilt = ribbonRebuilds != prevRibbonRebuilds;
        rec.carpetRebuilt = carpetRebuilds != prevCarpetRebuilds;
        for (size_t slot = 0; slot < kRibbonRoleCount; ++slot) {
            rec.ribbonMeshes[slot] = overlume::testing::ribbon_mesh_count(r, slot);
            rec.ribbonVerts[slot] = overlume::testing::ribbon_vertex_count(r, slot);
        }
        rec.carpetMeshes = overlume::testing::trajectory_carpet_mesh_count(r, 0);
        rec.carpetVerts = overlume::testing::trajectory_carpet_vertex_count(r, 0);

        if (i == 0) {
            probe = locate_probe(px, 320, 240);
        }
        rec.center = pixel_at(px, 320, probe.centerCol, probe.row);
        rec.rimLeft = pixel_at(px, 320, probe.rimLeftCol, probe.row);
        rec.rimRight = pixel_at(px, 320, probe.rimRightCol, probe.row);
        rec.centerIsBackground = !looks_like_corridor(rec.center);
        rec.rimLeftIsBackground = !looks_like_corridor(rec.rimLeft);
        rec.rimRightIsBackground = !looks_like_corridor(rec.rimRight);

        const bool anyRibbonSlotEmpty =
            std::any_of(std::begin(rec.ribbonMeshes), std::end(rec.ribbonMeshes),
                        [](size_t n) { return n == 0; });
        const bool emptySlotDespiteContent = (anyRibbonSlotEmpty || rec.carpetMeshes == 0) &&
                                             ribbons[0].point_count >= 2 && carpet.point_count >= 2;
        const bool visiblyEmptyFrame =
            rec.centerIsBackground && rec.rimLeftIsBackground && rec.rimRightIsBackground;
        if (emptySlotDespiteContent || visiblyEmptyFrame) {
            result.anyDropout = true;
        }

        print_trace(rec);
        result.frames.push_back(rec);
        prevRibbonRebuilds = ribbonRebuilds;
        prevCarpetRebuilds = carpetRebuilds;
    }
    result.ribbonRebuildsTotal = prevRibbonRebuilds;
    result.carpetRebuildsTotal = prevCarpetRebuilds;
    return result;
}

}

TEST(RibbonDropout, RealisticDrivingAtQuantize005DoesNotDropASingleThreadedFrame) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    fprintf(stderr,
            "\n=== RealisticDrivingAtQuantize005 (0.03-0.08m/frame, churn every 8th) ===\n");
    RunResult res = drive(r, 60, 0.03, 0.08, 1234);

    static constexpr const char* kRoleNames[kRibbonRoleCount] = {"BEHAVIOR", "GLOBAL", "LOCAL"};
    for (const auto& f : res.frames) {
        for (size_t slot = 0; slot < kRibbonRoleCount; ++slot) {
            EXPECT_GT(f.ribbonMeshes[slot], 0u)
                << "frame " << f.frame << ": " << kRoleNames[slot]
                << " ribbon slot has ZERO "
                   "meshes despite live content -- the exact reported dropout";
            EXPECT_GT(f.ribbonVerts[slot], 0u) << "frame " << f.frame << ": " << kRoleNames[slot]
                                               << " ribbon slot has ZERO "
                                                  "vertices despite live content";
        }
        EXPECT_GT(f.carpetMeshes, 0u) << "frame " << f.frame
                                      << ": trajectory carpet slot has ZERO "
                                         "meshes despite live content";
        EXPECT_FALSE(f.centerIsBackground && f.rimLeftIsBackground && f.rimRightIsBackground)
            << "frame " << f.frame << ": all 3 probes read background -- whole corridor vanished";
    }
    fprintf(stderr,
            "ribbon rebuilds: %llu/%d frames, carpet rebuilds: %llu/%d frames, dropout=%d\n",
            static_cast<unsigned long long>(res.ribbonRebuildsTotal), 60,
            static_cast<unsigned long long>(res.carpetRebuildsTotal), 60, res.anyDropout);
    EXPECT_FALSE(res.anyDropout)
        << "single-threaded sequential set_scene()+render_frame() reproduced the dropout -- "
           "see the per-frame trace on stderr above";
    EXPECT_LE(res.ribbonRebuildsTotal, 15u * kRibbonRoleCount)
        << "ribbon rebuilds tracked ego motion, not just content -- the clip is feeding the "
           "content signature again (the exact mechanism this fix removed)";
    EXPECT_GE(res.ribbonRebuildsTotal, 5u * kRibbonRoleCount)
        << "fewer rebuilds than the message-churn cadence (every 8th frame) should produce -- "
           "the driving loop isn't actually exercising a content change";
    overlume::destroy_renderer(r);
}

namespace {

int count_teal_pixels(const std::vector<uint8_t>& px, int width, int height) {
    int n = 0;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const Rgb c = pixel_at(px, width, x, y);
            if (c.g > c.r + 20 && c.g > 40) ++n;
        }
    }
    return n;
}

}

TEST(RibbonDropout, BehaviorRibbonNeverVanishesUnderCoLocatedCarpetChurn) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<Vec3> ribbonSpine = make_spine(0.0, 301, 0.5);
    std::vector<Vec3> carpetSpine = ribbonSpine;
    double egoX = 5.0;
    std::mt19937 rng(99);
    std::uniform_real_distribution<double> step(0.03, 0.08);

    int framesWithTeal = 0;
    constexpr int kFrames = 32;
    for (int i = 0; i < kFrames; ++i) {
        egoX += step(rng);
        if (i % 8 == 4) ribbonSpine.back().x += 0.01;
        if (i % 8 == 0) carpetSpine.back().x += 0.01;

        std::vector<PointCloudPoint> carpetPts(carpetSpine.size());
        for (size_t j = 0; j < carpetSpine.size(); ++j) {
            carpetPts[j].position = carpetSpine[j];
            carpetPts[j].rgba = pack_rgba(255, 0, 0, 255);
        }

        PathRibbon ribbon{};
        ribbon.role = PathRole::BEHAVIOR;
        ribbon.points = ribbonSpine.data();
        ribbon.point_count = static_cast<uint32_t>(ribbonSpine.size());
        ribbon.last_update_sec = 0.0;

        TrajectoryCarpet carpet{};
        carpet.points = carpetPts.data();
        carpet.point_count = static_cast<uint32_t>(carpetPts.size());
        carpet.last_update_sec = 0.0;

        overlume::SceneGraph s{};
        s.sim_time_sec = 0.0;
        s.ego = {{egoX, 0.0, 0.0}, 0.0, 0.0, 1};
        s.paths = &ribbon;
        s.path_count = 1;
        s.trajectory_carpets = &carpet;
        s.trajectory_carpet_count = 1;
        overlume::set_scene(r, s);

        overlume::CameraPose pose{{egoX - 4.0, -8.0, 6.0}, {egoX + 4.0, 1.0, 0.0}, 60.0};
        std::vector<uint8_t> px = render_once(r, pose);

        const int teal = count_teal_pixels(px, 320, 240);
        if (teal > 0) ++framesWithTeal;
        fprintf(stderr, "frame %2d: teal_pixels=%d\n", i, teal);
        EXPECT_GT(teal, 0) << "frame " << i
                           << ": the teal BEHAVIOR ribbon (z 0.058, above the "
                              "carpet's 0.052) vanished -- the co-located strip drawn after it "
                              "overpainted it (blended-queue order dependence)";
    }
    fprintf(stderr, "teal visible on %d/%d frames\n", framesWithTeal, kFrames);
    overlume::destroy_renderer(r);
}

TEST(RibbonDropout, ParkedEgoRebuildRateIsNearZeroByContrast) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    fprintf(stderr, "\n=== ParkedEgoRebuildRateIsNearZeroByContrast (~0m/frame) ===\n");
    RunResult res = drive(r, 60, 0.0, 0.001, 1234);

    fprintf(stderr, "ribbon rebuilds: %llu/60 frames (parked) vs the driving run's rate above\n",
            static_cast<unsigned long long>(res.ribbonRebuildsTotal));
    EXPECT_FALSE(res.anyDropout);
    EXPECT_LT(res.ribbonRebuildsTotal, 10u * kRibbonRoleCount)
        << "a near-parked ego still rebuilt almost every frame -- the quantized clip station "
           "isn't stable the way ribbon.cpp's own comment claims";
    overlume::destroy_renderer(r);
}
