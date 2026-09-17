// test_ribbon_dropout.cpp — regression pass for the live flicker report
// ("the green LOCAL path disappears and reappears randomly"). Drives a
// PathRibbon per role (BEHAVIOR/GLOBAL/LOCAL) plus a TrajectoryCarpet, all
// on the same spine, through realistic per-frame ego motion (single-
// threaded set_scene()+render_frame(), the only contract scene.h documents
// as supported) and pins two properties: no ribbon/carpet slot ever reads
// empty while its content is live, and the rebuild count tracks CONTENT
// churn (message updates), not ego motion — see the "Rebuild/clip
// decoupling" 2026-09-10 entry in
// docs/plans/2026-09-09-vm077-new-stack-rendering.md for the
// full investigation and its still-open BEHAVIOR-ribbon staleness question.
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

// One straight spine, spaced 0.5m, both categories read the SAME centerline
// -- "on the same spine" per the repro instructions, mirroring how the two
// categories visually stack in ribbon.cpp/trajectory_carpet.cpp's z-order
// (LOCAL ribbon wider+lower, carpet narrower+higher, both centered on the
// same points).
std::vector<Vec3> make_spine(double startX, uint32_t n, double spacingM) {
    std::vector<Vec3> pts(n);
    for (uint32_t i = 0; i < n; ++i) pts[i] = Vec3{startX + i * spacingM, 0.0, 0.0};
    return pts;
}

std::vector<PointCloudPoint> make_carpet_points(const std::vector<Vec3>& spine, uint32_t colorTweak) {
    std::vector<PointCloudPoint> pts(spine.size());
    // Bright, saturated, and unlike the theme's amber ribbon_local tint --
    // "orange VELOCITY" per the user's own description of the flicker.
    const uint32_t rgba = pack_rgba(255, static_cast<uint8_t>(120 + (colorTweak % 30)), 0, 255);
    for (size_t i = 0; i < spine.size(); ++i) {
        pts[i].position = spine[i];
        pts[i].rgba = rgba;
    }
    return pts;
}

// Fixed 3-position probe (found once, off frame 0, reused every frame): a
// horizontal scan-line across the corridor's cross-section. All three
// positions track the ego because the camera offset below is ego-relative
// -- the corridor should sit in roughly the same screen region every frame
// regardless of ego.x.
struct ProbeLayout {
    int row = 0;
    int rimLeftCol = 0;
    int centerCol = 0;
    int rimRightCol = 0;
};

// dark_adas' sun/palette are blue-leaning (sun color [0.55,0.6,0.75], road a
// dark neutral) while BOTH corridor categories here are warm (carpet:
// authored r=255,g~120-150,b=0; ribbon_local theme tint [0.95,0.70,0.15]).
// r>b is a theme-derived but robust "corridor vs road/background"
// discriminator -- a generic brightness-vs-corner-sample test was tried
// first and mis-fired (the corner-relative threshold caught the whole lit
// road surface, not just the corridor, so "center"/"rim" landed on bare
// road instead of the ribbon).
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
                if (runLen > bestRowRunLen) { bestRowRunLen = runLen; bestRowRunStart = runStart; }
                runStart = -1;
                runLen = 0;
            }
        }
        if (runLen > bestRowRunLen) { bestRowRunLen = runLen; bestRowRunStart = runStart; }
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

// Ribbon slot count/order this file drives every frame: BEHAVIOR, GLOBAL,
// LOCAL (matches test_ribbon.cpp's ThreeRoles scene shape) -- Criterion 2
// requires all four ribbon-shaped slots (these three roles + the carpet)
// covered, not just the one (LOCAL) the original repro pass measured.
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
            f.ribbonMeshes[2], f.carpetMeshes, f.ribbonVerts[0], f.ribbonVerts[1],
            f.ribbonVerts[2], f.carpetVerts, f.center.r, f.center.g, f.center.b,
            f.centerIsBackground ? " BG!" : "", f.rimLeft.r, f.rimLeft.g, f.rimLeft.b,
            f.rimLeftIsBackground ? " BG!" : "", f.rimRight.r, f.rimRight.g, f.rimRight.b,
            f.rimRightIsBackground ? " BG!" : "");
}

// Drives N frames of ego motion (stepMinM..stepMaxM per frame, seeded RNG
// for reproducibility) with periodic "message churn" (tail point nudged +
// fresh heap allocation, simulating a new planner publish) and returns the
// per-frame trace plus whether any dropout was observed.
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

    std::vector<Vec3> spine = make_spine(/*startX=*/0.0, /*n=*/301, /*spacingM=*/0.5);  // 150m
    double egoX = 5.0;  // comfortably inside the spine, far from either end
    ProbeLayout probe{};
    uint64_t prevRibbonRebuilds = 0, prevCarpetRebuilds = 0;

    for (int i = 0; i < numFrames; ++i) {
        egoX += step(rng);

        // Message churn every 8th frame: fresh heap allocation, tail point
        // nudged -- simulates the planner extending/redrawing its horizon,
        // independent of the ego-driven clip-station churn.
        if (i > 0 && i % 8 == 0) {
            spine.back().x += 0.01;
        }
        std::vector<Vec3> spineCopy = spine;  // fresh vector each frame either way
        std::vector<PointCloudPoint> carpetPts = make_carpet_points(spineCopy, static_cast<uint32_t>(i));

        // Same spine for all three roles -- BEHAVIOR/GLOBAL/LOCAL routinely
        // trace the same route live (ribbon.cpp's own z-stagger comment), and
        // this file's whole point is exercising every role through the exact
        // per-tick clip path, not just LOCAL.
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
        s.ego = {{egoX, 0.0, 0.0}, 0.0, 0.0, /*valid=*/1};
        s.paths = ribbons;
        s.path_count = kRibbonRoleCount;
        s.trajectory_carpets = &carpet;
        s.trajectory_carpet_count = 1;
        overlume::set_scene(r, s);

        // Camera tracks the ego with a fixed relative offset (same shape as
        // test_ribbon.cpp's ThreeRoles golden) so the corridor stays in
        // roughly the same screen region every frame regardless of egoX.
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
            // Locate the probe positions once (r>b corridor scan, see
            // looks_like_corridor) -- reused verbatim every later frame.
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
        const bool emptySlotDespiteContent =
            (anyRibbonSlotEmpty || rec.carpetMeshes == 0) &&
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

}  // namespace

// ── Primary reproduction: realistic driving churns the clip station nearly
//    every frame (kPolylineClipQuantizeM=0.05) -- does that alone produce a
//    frame where content exists but nothing renders? ────────────────────────
TEST(RibbonDropout, RealisticDrivingAtQuantize005DoesNotDropASingleThreadedFrame) {
    overlume::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    fprintf(stderr, "\n=== RealisticDrivingAtQuantize005 (0.03-0.08m/frame, churn every 8th) ===\n");
    RunResult res = drive(r, /*numFrames=*/60, /*stepMinM=*/0.03, /*stepMaxM=*/0.08, /*seed=*/1234);

    static constexpr const char* kRoleNames[kRibbonRoleCount] = {"BEHAVIOR", "GLOBAL", "LOCAL"};
    for (const auto& f : res.frames) {
        for (size_t slot = 0; slot < kRibbonRoleCount; ++slot) {
            EXPECT_GT(f.ribbonMeshes[slot], 0u)
                << "frame " << f.frame << ": " << kRoleNames[slot] << " ribbon slot has ZERO "
                   "meshes despite live content -- the exact reported dropout";
            EXPECT_GT(f.ribbonVerts[slot], 0u)
                << "frame " << f.frame << ": " << kRoleNames[slot] << " ribbon slot has ZERO "
                   "vertices despite live content";
        }
        EXPECT_GT(f.carpetMeshes, 0u) << "frame " << f.frame << ": trajectory carpet slot has ZERO "
                                          "meshes despite live content";
        EXPECT_FALSE(f.centerIsBackground && f.rimLeftIsBackground && f.rimRightIsBackground)
            << "frame " << f.frame << ": all 3 probes read background -- whole corridor vanished";
    }
    fprintf(stderr, "ribbon rebuilds: %llu/%d frames, carpet rebuilds: %llu/%d frames, dropout=%d\n",
            static_cast<unsigned long long>(res.ribbonRebuildsTotal), 60,
            static_cast<unsigned long long>(res.carpetRebuildsTotal), 60, res.anyDropout);
    EXPECT_FALSE(res.anyDropout)
        << "single-threaded sequential set_scene()+render_frame() reproduced the dropout -- "
           "see the per-frame trace on stderr above";
    // POST-FIX SANITY (VM-0xx): the ego-clip no longer feeds the content
    // signature (ribbon.cpp/trajectory_carpet.cpp), so driving alone must
    // NOT churn the rebuild count anymore -- only the message churn this
    // loop injects every 8th frame should (~60/8 = 7-8 rebuilds). Before the
    // fix this was >30 (most frames rebuilt from clip-station churn alone,
    // per this test's own header); a regression back to that shape would
    // mean the destroy-then-rebuild path came back.
    // Thresholds scaled by kRibbonRoleCount: ribbon_rebuild_count() is one
    // counter shared across every ribbon slot, and all three roles share the
    // same spine/churn cadence here, so each churn (or the initial build)
    // increments it once per role, not once total.
    EXPECT_LE(res.ribbonRebuildsTotal, 15u * kRibbonRoleCount)
        << "ribbon rebuilds tracked ego motion, not just content -- the clip is feeding the "
           "content signature again (the exact mechanism this fix removed)";
    EXPECT_GE(res.ribbonRebuildsTotal, 5u * kRibbonRoleCount)
        << "fewer rebuilds than the message-churn cadence (every 8th frame) should produce -- "
           "the driving loop isn't actually exercising a content change";
    overlume::destroy_renderer(r);
}

// ── The BINARY teal toggle from the live burst (teal fraction exactly 0.0
//    on 15/24 frames, ~0.027 on the rest): BEHAVIOR (ribbon_emissive.mat)
//    and the velocity carpet (trajectory_carpet.mat) were BOTH fade-blended
//    -- Filament's blended queue sorts per-renderable and writes no depth,
//    so with the two strips co-located on one spine the later-drawn one
//    fully overpaints the other at alpha 1. Rebuilds re-enter the queue, so
//    alternating ribbon/carpet content churn (exactly the live ~8Hz publish
//    pattern) flips which is drawn last. This test alternates those rebuilds
//    and asserts the teal BEHAVIOR strip -- z-lifted ABOVE the carpet
//    (0.058 vs 0.052) and therefore rightfully visible -- never vanishes.
//    The carpet is authored pure red and BEHAVIOR is the theme's cold
//    green/teal, so "any green-dominant pixel exists" is the discriminator
//    (the drive() probe above is warm-only, r>b, and is blind to teal --
//    which is why the tests above passed while the live scene flickered). ──
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

}  // namespace

TEST(RibbonDropout, BehaviorRibbonNeverVanishesUnderCoLocatedCarpetChurn) {
    overlume::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
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
        // Alternate WHICH renderable rebuilds -- the live publish pattern
        // (behavior path and carpet arrive from different topics at
        // different moments). The nudged tail is ~145m away, far outside
        // the probed screen region.
        if (i % 8 == 4) ribbonSpine.back().x += 0.01;
        if (i % 8 == 0) carpetSpine.back().x += 0.01;

        std::vector<PointCloudPoint> carpetPts(carpetSpine.size());
        for (size_t j = 0; j < carpetSpine.size(); ++j) {
            carpetPts[j].position = carpetSpine[j];
            carpetPts[j].rgba = pack_rgba(255, 0, 0, 255);  // pure red -- never green-dominant
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
        s.ego = {{egoX, 0.0, 0.0}, 0.0, 0.0, /*valid=*/1};
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
        EXPECT_GT(teal, 0) << "frame " << i << ": the teal BEHAVIOR ribbon (z 0.058, above the "
                              "carpet's 0.052) vanished -- the co-located strip drawn after it "
                              "overpainted it (blended-queue order dependence)";
    }
    fprintf(stderr, "teal visible on %d/%d frames\n", framesWithTeal, kFrames);
    overlume::destroy_renderer(r);
}

// ── Control: a parked ego (zero clip-station churn) is the same condition
//    the retired kPolylineClipQuantizeM=0.5 constant existed to approximate
//    -- confirms rebuild rate (and therefore any rebuild-driven artifact)
//    scales with ego motion, per the repro instructions' explicit
//    alternative to recompiling with the old quantize value. ───────────────
TEST(RibbonDropout, ParkedEgoRebuildRateIsNearZeroByContrast) {
    overlume::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    fprintf(stderr, "\n=== ParkedEgoRebuildRateIsNearZeroByContrast (~0m/frame) ===\n");
    // Effectively parked: a step small enough that quantize=0.05 almost
    // never crosses a bin boundary (not exactly 0.0 -- still real motion,
    // same "GPS jitter" shape compute_polyline_clip's quantization exists to
    // absorb).
    RunResult res = drive(r, /*numFrames=*/60, /*stepMinM=*/0.0, /*stepMaxM=*/0.001, /*seed=*/1234);

    fprintf(stderr, "ribbon rebuilds: %llu/60 frames (parked) vs the driving run's rate above\n",
            static_cast<unsigned long long>(res.ribbonRebuildsTotal));
    EXPECT_FALSE(res.anyDropout);
    // Scaled by kRibbonRoleCount -- see the driving test's own comment on why.
    EXPECT_LT(res.ribbonRebuildsTotal, 10u * kRibbonRoleCount)
        << "a near-parked ego still rebuilt almost every frame -- the quantized clip station "
           "isn't stable the way ribbon.cpp's own comment claims";
    overlume::destroy_renderer(r);
}
