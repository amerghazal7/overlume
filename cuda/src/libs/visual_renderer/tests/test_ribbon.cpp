// test_ribbon.cpp — Epic 2 Task 5 (VM-023): path ribbons in three roles
// (BEHAVIOR/GLOBAL/LOCAL), the behavior ribbon as the emissive bloom hero.
// Same "no Filament type" boundary as every other tests/*.cpp -- see
// ribbon_test_hooks.hpp. polyline.hpp is Filament-free (its own header
// comment) so this file, like test_polyline.cpp, may include it directly
// for polyline_chunks()'s own math (LongPathSplitsAcrossMeshesWithoutTruncation).
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

#include "golden.hpp"
#include "polyline.hpp"
#include "ribbon_test_hooks.hpp"
#include "test_paths.hpp"
#include "theme.hpp"

#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

std::vector<uint8_t> render_once(mpviz::VisualRenderer* r, const mpviz::CameraPose& pose) {
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    mpviz::FrameView view{pixels.data(), 320, 240};
    EXPECT_TRUE(mpviz::render_frame(r, pose, view));
    return pixels;
}

}  // namespace

// ── Step 3: three roles at once, BEHAVIOR the bloom hero ───────────────────

TEST(RibbonGolden, ThreeRoles_DarkAdas) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // Synthetic three-ribbon scene (FIXTURE GAP 2: no global-path publisher
    // exists in the recorded stack, so this golden cannot come from the
    // bag -- see docs/superpowers/plans/2026-08-18-visual-mode-epic2.md,
    // Task 5 Step 3). Points array kept alive by RibbonScene across
    // set_scene() (golden.cpp's move-only owner pattern).
    mpviz::testing::RibbonScene ribbons = mpviz::testing::make_three_role_ribbons(/*now=*/10.0);
    mpviz::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.ego = {{0, 0, 0}, 0.0, 0.0, /*valid=*/1};
    s.paths = ribbons.ribbons.data();
    s.path_count = static_cast<uint32_t>(ribbons.ribbons.size());
    mpviz::set_scene(r, s);

    mpviz::CameraPose pose{{-14, -14, 10}, {0, 0, 0}, 60.0};
    double ssim = mpviz::testing::render_and_compare(
        r, pose, MPVIZ_TEST_DATA_DIR "/tests/goldens/ribbons_three_roles_dark_adas.png",
        "/tmp/ribbons_three_roles_dark_adas_actual.png");
    EXPECT_GT(ssim, 0.98);
    mpviz::destroy_renderer(r);
}

// ── Step 4: a path change rebuilds geometry, not a stale cache ─────────────

TEST(Ribbon, PathChangeRebuildsGeometry) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const mpviz::Vec3 longPts[] = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}};
    mpviz::PathRibbon ribbon{};
    ribbon.role = mpviz::PathRole::LOCAL;
    ribbon.points = longPts;
    ribbon.point_count = 5;
    ribbon.last_update_sec = 1.0;
    mpviz::SceneGraph s{};
    s.sim_time_sec = 1.0;
    s.paths = &ribbon;
    s.path_count = 1;
    mpviz::set_scene(r, s);
    mpviz::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};
    render_once(r, pose);
    EXPECT_EQ(mpviz::testing::ribbon_vertex_count(r, 0), 5u * 2);

    const mpviz::Vec3 shortPts[] = {{0, 0, 0}, {1, 0, 0}};
    ribbon.points = shortPts;
    ribbon.point_count = 2;
    ribbon.last_update_sec = 2.0;
    s.sim_time_sec = 2.0;
    mpviz::set_scene(r, s);
    render_once(r, pose);
    EXPECT_EQ(mpviz::testing::ribbon_vertex_count(r, 0), 2u * 2)
        << "publishing a shorter path did not shrink slot 0's geometry -- a stale-cache bug";
    mpviz::destroy_renderer(r);
}

// ── ITEM 1 (user directive 2026-08-20): theme.ribbon.width_m joins the slot
//    content signature ─────────────────────────────────────────────────────

TEST(Ribbon, WidthChangeRebuildsGeometry) {
    // ribbon_width_a.yaml/ribbon_width_b.yaml are byte-for-byte identical
    // except `ribbon.width_m` (0.24 vs 0.60) -- same fixture-dir convention
    // as test_theme.cpp's sun_dir_a/b. The PathRibbon's own point data is
    // NEVER touched across the set_theme() call below: if width didn't
    // join the slot's content signature (ribbon.cpp's ribbon_signature()),
    // the slot would consider itself unchanged and silently keep rendering
    // the OLD width forever.
    const std::string fixtureDir = std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/themes";
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, fixtureDir.c_str(), "ribbon_width_a"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const mpviz::Vec3 pts[] = {{-5, 0, 0}, {-3, 0, 0}, {-1, 0, 0}, {1, 0, 0}, {3, 0, 0}};
    mpviz::PathRibbon ribbon{};
    ribbon.role = mpviz::PathRole::LOCAL;
    ribbon.points = pts;
    ribbon.point_count = 5;
    ribbon.last_update_sec = 0.0;
    mpviz::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.paths = &ribbon;
    s.path_count = 1;
    mpviz::set_scene(r, s);
    mpviz::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};
    render_once(r, pose);
    EXPECT_NEAR(mpviz::testing::ribbon_slot_half_width_m(r, 0), 0.12f, 1e-4f);
    const size_t vertsBefore = mpviz::testing::ribbon_vertex_count(r, 0);

    // set_theme() to the wider fixture, then advance sim_time_sec past the
    // transition's own duration (deterministic-clock pattern, test_theme_
    // transition.cpp's DeterministicClock_MatchesTargetAtDuration) so the
    // blend has fully settled on ribbon_width_b's 0.60 width_m, not some
    // partially-blended value.
    ASSERT_TRUE(mpviz::set_theme(r, "ribbon_width_b", /*at_sec=*/0.0, /*transition_sec=*/0.2));
    s.sim_time_sec = 0.2;
    mpviz::set_scene(r, s);
    render_once(r, pose);

    EXPECT_NEAR(mpviz::testing::ribbon_slot_half_width_m(r, 0), 0.30f, 1e-4f)
        << "a width-only theme change (no PathRibbon point data touched) did not rebuild "
           "slot 0's geometry at the new width -- width isn't part of the slot signature";
    // Vertex COUNT is width-independent (2 per point, always) -- unchanged
    // here is the EXPECTED, correct outcome, not evidence either way about
    // whether a rebuild happened (see ribbon_slot_half_width_m()'s own
    // comment for why that hook, not this count, is the real signal).
    EXPECT_EQ(mpviz::testing::ribbon_vertex_count(r, 0), vertsBefore);

    mpviz::destroy_renderer(r);
}

// ── Step 4: THE keying test -- two LOCAL ribbons, both live ────────────────

TEST(Ribbon, TwoLocalRibbonsBothRender) {
    // Both shipped profiles put TWO rows on role LOCAL (/local_vel_path AND
    // /local_path, spec §7 / fixture gap 2) -- SceneAssembly::paths holds
    // FOUR ribbons over THREE roles in a real frame. Keyed by role, the
    // second LOCAL ribbon would silently overwrite the first's entity every
    // frame; keyed by slot index (this task's decision), both render with
    // their own vertex counts.
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const mpviz::Vec3 ptsA[] = {{-5, 0, 0}, {-3, 0, 0}, {-1, 0, 0}};
    const mpviz::Vec3 ptsB[] = {{1, 0, 0}, {3, 0, 0}, {5, 0, 0}, {7, 0, 0}};
    mpviz::PathRibbon ribbons[2]{};
    ribbons[0].role = mpviz::PathRole::LOCAL;
    ribbons[0].points = ptsA;
    ribbons[0].point_count = 3;
    ribbons[0].last_update_sec = 1.0;
    ribbons[1].role = mpviz::PathRole::LOCAL;
    ribbons[1].points = ptsB;
    ribbons[1].point_count = 4;
    ribbons[1].last_update_sec = 1.0;
    mpviz::SceneGraph s{};
    s.sim_time_sec = 1.0;
    s.paths = ribbons;
    s.path_count = 2;
    mpviz::set_scene(r, s);
    mpviz::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};
    render_once(r, pose);

    EXPECT_GT(mpviz::testing::ribbon_mesh_count(r, 0), 0u);
    EXPECT_GT(mpviz::testing::ribbon_mesh_count(r, 1), 0u);
    EXPECT_EQ(mpviz::testing::ribbon_vertex_count(r, 0), 3u * 2)
        << "slot 0 (the first LOCAL row) has no geometry of its own -- keyed by role, not slot";
    EXPECT_EQ(mpviz::testing::ribbon_vertex_count(r, 1), 4u * 2)
        << "slot 1 (the second LOCAL row) has no geometry of its own -- overwritten by slot 0";
    mpviz::destroy_renderer(r);
}

// ── Staleness: shared clay_translucent.mat swap for GLOBAL/LOCAL, direct
//    alpha for BEHAVIOR's own emissive material ──────────────────────────

TEST(Ribbon, StaleRibbonFadesViaSharedStalenessAlpha) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const mpviz::Vec3 behPts[] = {{0, 0, 0}, {2, 0, 0}};
    const mpviz::Vec3 locPts[] = {{0, 5, 0}, {2, 5, 0}};
    mpviz::PathRibbon ribbons[2]{};
    ribbons[0].role = mpviz::PathRole::BEHAVIOR;
    ribbons[0].points = behPts;
    ribbons[0].point_count = 2;
    ribbons[0].last_update_sec = 9.25;  // 0.75s stale at sim_time 10.0: mid-fade (0.5 <= t < 1.0)
    ribbons[1].role = mpviz::PathRole::LOCAL;
    ribbons[1].points = locPts;
    ribbons[1].point_count = 2;
    ribbons[1].last_update_sec = 9.25;
    mpviz::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.paths = ribbons;
    s.path_count = 2;
    mpviz::set_scene(r, s);
    mpviz::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};
    render_once(r, pose);

    // BEHAVIOR: alpha IS the staleness knob on ribbon_emissive.mat's OWN
    // instance -- never an instance swap.
    const auto beh = mpviz::testing::ribbon_slot_material_info(r, 0);
    EXPECT_FALSE(beh.bound_to_translucent);
    EXPECT_GT(beh.alpha, 0.0f);
    EXPECT_LT(beh.alpha, 1.0f);

    // LOCAL: swaps to a per-slot clay_translucent.mat instance (Task 4's
    // mechanism) -- NOT clay_faded.mat (no settable alpha, requires a COLOR
    // attribute extrude_polyline doesn't produce).
    const auto loc = mpviz::testing::ribbon_slot_material_info(r, 1);
    EXPECT_TRUE(loc.bound_to_translucent);
    EXPECT_GT(loc.alpha, 0.0f);
    EXPECT_LT(loc.alpha, 1.0f);
    mpviz::destroy_renderer(r);
}

// ── The uint16 index-buffer ceiling actually triggers ───────────────────────

TEST(Ribbon, LongPathSplitsAcrossMeshesWithoutTruncation) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    constexpr uint32_t kN = 40000;
    std::vector<mpviz::Vec3> pts(kN);
    for (uint32_t i = 0; i < kN; ++i) pts[i] = {static_cast<double>(i) * 0.1, 0.0, 0.0};

    mpviz::PathRibbon ribbon{};
    ribbon.role = mpviz::PathRole::GLOBAL;
    ribbon.points = pts.data();
    ribbon.point_count = kN;
    ribbon.last_update_sec = 1.0;
    mpviz::SceneGraph s{};
    s.sim_time_sec = 1.0;
    s.paths = &ribbon;
    s.path_count = 1;
    mpviz::set_scene(r, s);
    mpviz::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};
    render_once(r, pose);

    const auto chunks = mpviz::detail::polyline_chunks(kN);
    ASSERT_EQ(chunks.size(), 2u) << "40000 points should split into exactly 2 chunks at "
                                     "kMaxPointsPerMesh=32000 -- fixture assumption changed?";
    size_t expectedVerts = 0;
    for (const auto& [a, b] : chunks) expectedVerts += 2 * static_cast<size_t>(b - a);

    EXPECT_EQ(mpviz::testing::ribbon_mesh_count(r, 0), chunks.size());
    EXPECT_EQ(mpviz::testing::ribbon_vertex_count(r, 0), expectedVerts)
        << "vertex count doesn't match polyline_chunks()'s own math -- a point was lost "
           "at the chunk-split seam";
    mpviz::destroy_renderer(r);
}

// ── Step 8a's exemplar, copied: themed on first data, no transition ────────

TEST(Ribbon, MaterialIsThemedOnFirstDataWithNoTransition) {
    const auto theme = mpviz::detail::load_theme(kThemeDir, "dark_adas");
    ASSERT_TRUE(theme.has_value());

    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // First-ever path data, all three roles. Nothing calls set_theme().
    const mpviz::Vec3 pts[] = {{0, 0, 0}, {5, 0, 0}};
    mpviz::PathRibbon ribbons[3]{};
    ribbons[0].role = mpviz::PathRole::BEHAVIOR;
    ribbons[0].points = pts;
    ribbons[0].point_count = 2;
    ribbons[1].role = mpviz::PathRole::GLOBAL;
    ribbons[1].points = pts;
    ribbons[1].point_count = 2;
    ribbons[2].role = mpviz::PathRole::LOCAL;
    ribbons[2].points = pts;
    ribbons[2].point_count = 2;
    mpviz::SceneGraph s{};
    s.paths = ribbons;
    s.path_count = 3;
    mpviz::set_scene(r, s);
    mpviz::CameraPose pose{{0, -8, 4}, {0, 0, 0}, 60.0};
    render_once(r, pose);

    const auto beh = mpviz::testing::ribbon_role_base_color(r, mpviz::PathRole::BEHAVIOR);
    EXPECT_NEAR(beh.r, theme->palette.ribbon_core.r, 1e-4);
    EXPECT_NEAR(beh.g, theme->palette.ribbon_core.g, 1e-4);
    EXPECT_NEAR(beh.b, theme->palette.ribbon_core.b, 1e-4);

    // GLOBAL/LOCAL each have their OWN dedicated theme token as of user
    // directive 2026-08-20 (ITEM 1) -- ribbon_global/ribbon_local
    // (theme.hpp), no longer a reuse of the BEHAVIOR/hero ribbon's own
    // ribbon_core/ribbon_glow (see push_theme_to_scene()'s own comment in
    // renderer.cpp).
    const auto glob = mpviz::testing::ribbon_role_base_color(r, mpviz::PathRole::GLOBAL);
    EXPECT_NEAR(glob.r, theme->palette.ribbon_global.r, 1e-4);
    EXPECT_NEAR(glob.g, theme->palette.ribbon_global.g, 1e-4);
    EXPECT_NEAR(glob.b, theme->palette.ribbon_global.b, 1e-4);

    const auto loc = mpviz::testing::ribbon_role_base_color(r, mpviz::PathRole::LOCAL);
    EXPECT_NEAR(loc.r, theme->palette.ribbon_local.r, 1e-4);
    EXPECT_NEAR(loc.g, theme->palette.ribbon_local.g, 1e-4);
    EXPECT_NEAR(loc.b, theme->palette.ribbon_local.b, 1e-4);
    mpviz::destroy_renderer(r);
}
