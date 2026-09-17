// test_alert_polygons.cpp — translucent collision alert polygons. Same
// "no Filament type" boundary as every other tests/*.cpp -- see
// alert_polygons_test_hooks.hpp.
//
// The five collision-checker topics were silent in the recorded bag --
// AlertGolden.SweepPlusPredicted_DarkAdas's scene is entirely synthetic
// (golden.cpp's make_sweep_and_predicted_alerts()).
#include "overlume/api.h"
#include "overlume/scene.h"

#include "alert_polygons_test_hooks.hpp"
#include "golden.hpp"
#include "test_paths.hpp"
#include "theme.hpp"

#include <vector>

#include <gtest/gtest.h>

namespace {

std::vector<uint8_t> render_once(overlume::VisualRenderer* r, const overlume::CameraPose& pose) {
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    overlume::FrameView view{pixels.data(), 320, 240};
    EXPECT_TRUE(overlume::render_frame(r, pose, view));
    return pixels;
}

}  // namespace

// ── Step 3: a ghost sweep trail + a warning predicted polygon ──────────────

TEST(AlertGolden, SweepPlusPredicted_DarkAdas) {
    overlume::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // Synthetic scene: no live collision-checker publisher exists in the
    // recorded stack. Points array kept alive by AlertScene across
    // set_scene() (golden.cpp's move-only owner pattern).
    overlume::testing::AlertScene alerts = overlume::testing::make_sweep_and_predicted_alerts(/*now=*/10.0);
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.ego = {{0, 0, 0}, 0.0, 0.0, /*valid=*/1};
    s.alerts = alerts.alerts.data();
    s.alert_count = static_cast<uint32_t>(alerts.alerts.size());
    overlume::set_scene(r, s);

    overlume::CameraPose pose{{0, -12, 12}, {2, 2, 0}, 60.0};
    double ssim = overlume::testing::render_and_compare(
        r, pose, OVERLUME_TEST_DATA_DIR "/tests/goldens/alerts_warning_dark_adas.png",
        "/tmp/alerts_warning_dark_adas_actual.png");
    EXPECT_GT(ssim, 0.98);
    overlume::destroy_renderer(r);
}

// ── Step 4: severity picks the theme's alert ramp, not a hand-picked color ─

TEST(Alerts, SeverityPicksTheThemeAlertRamp) {
    const auto theme = overlume::detail::load_theme(kThemeDir, "dark_adas");
    ASSERT_TRUE(theme.has_value());

    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // One fresh polygon per severity -- each must be bound to ITS OWN
    // severity's template instance (not e.g. warning's polygon silently
    // sharing critical's), and that template's rgb must be the matching
    // palette.alert.* token.
    const overlume::Vec3 pts[] = {{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}};
    overlume::AlertPolygon polys[3]{};
    for (int i = 0; i < 3; ++i) {
        polys[i].points = pts;
        polys[i].point_count = 4;
        polys[i].severity = static_cast<uint8_t>(i);
        polys[i].last_update_sec = 0.0;
    }
    overlume::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.alerts = polys;
    s.alert_count = 3;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{1, -8, 6}, {1, 1, 0}, 60.0};
    render_once(r, pose);

    const auto info = overlume::testing::alert_slot_material_info(r, 0);
    EXPECT_TRUE(info.bound_to_severity_template);
    const auto warn = overlume::testing::alert_slot_material_info(r, 1);
    EXPECT_TRUE(warn.bound_to_severity_template);
    const auto crit = overlume::testing::alert_slot_material_info(r, 2);
    EXPECT_TRUE(crit.bound_to_severity_template);

    const auto infoColor = overlume::testing::alert_severity_base_color(r, 0);
    EXPECT_NEAR(infoColor.r, theme->palette.alert.info.r, 1e-4);
    EXPECT_NEAR(infoColor.g, theme->palette.alert.info.g, 1e-4);
    EXPECT_NEAR(infoColor.b, theme->palette.alert.info.b, 1e-4);

    const auto warnColor = overlume::testing::alert_severity_base_color(r, 1);
    EXPECT_NEAR(warnColor.r, theme->palette.alert.warning.r, 1e-4);
    EXPECT_NEAR(warnColor.g, theme->palette.alert.warning.g, 1e-4);
    EXPECT_NEAR(warnColor.b, theme->palette.alert.warning.b, 1e-4);

    const auto critColor = overlume::testing::alert_severity_base_color(r, 2);
    EXPECT_NEAR(critColor.r, theme->palette.alert.critical.r, 1e-4);
    EXPECT_NEAR(critColor.g, theme->palette.alert.critical.g, 1e-4);
    EXPECT_NEAR(critColor.b, theme->palette.alert.critical.b, 1e-4);
    overlume::destroy_renderer(r);
}

// ── Staleness: shared clay_translucent.mat swap, MULTIPLYING the severity's
//    constant alpha down -- a fading critical must not brighten past it ────

TEST(Alerts, StaleAlertFadesViaSharedStalenessAlpha) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 freshPts[] = {{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}};
    const overlume::Vec3 stalePts[] = {{5, 0, 0}, {7, 0, 0}, {7, 2, 0}, {5, 2, 0}};
    overlume::AlertPolygon polys[2]{};
    polys[0].points = freshPts;
    polys[0].point_count = 4;
    polys[0].severity = 2;  // critical
    polys[0].last_update_sec = 10.0;  // fresh at sim_time 10.0
    polys[1].points = stalePts;
    polys[1].point_count = 4;
    polys[1].severity = 2;  // same severity -- isolates the fade, not a ramp difference
    polys[1].last_update_sec = 9.25;  // 0.75s stale: mid-fade (0.5 <= t < 1.0)
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.alerts = polys;
    s.alert_count = 2;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{1, -8, 6}, {1, 1, 0}, 60.0};
    render_once(r, pose);

    const auto fresh = overlume::testing::alert_slot_material_info(r, 0);
    EXPECT_TRUE(fresh.bound_to_severity_template);

    const auto stale = overlume::testing::alert_slot_material_info(r, 1);
    EXPECT_FALSE(stale.bound_to_severity_template)
        << "a stale alert must swap to its own clay_translucent.mat instance, not clay_faded.mat "
           "(no settable alpha) and not the shared severity template (whose alpha never changes)";
    EXPECT_GT(stale.alpha, 0.0f);
    EXPECT_LT(stale.alpha, fresh.alpha)
        << "a fading critical must not brighten past its constant alpha";
    overlume::destroy_renderer(r);
}

// ── Step 8a's exemplar, copied: themed on first data, no transition ────────

TEST(Alerts, MaterialIsThemedOnFirstDataWithNoTransition) {
    const auto theme = overlume::detail::load_theme(kThemeDir, "dark_adas");
    ASSERT_TRUE(theme.has_value());

    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // First-ever alert data, all three severities. Nothing calls
    // set_theme().
    const overlume::Vec3 pts[] = {{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}};
    overlume::AlertPolygon polys[3]{};
    for (int i = 0; i < 3; ++i) {
        polys[i].points = pts;
        polys[i].point_count = 4;
        polys[i].severity = static_cast<uint8_t>(i);
    }
    overlume::SceneGraph s{};
    s.alerts = polys;
    s.alert_count = 3;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{1, -8, 6}, {1, 1, 0}, 60.0};
    render_once(r, pose);

    const auto info = overlume::testing::alert_severity_base_color(r, 0);
    EXPECT_NEAR(info.r, theme->palette.alert.info.r, 1e-4);
    EXPECT_NEAR(info.g, theme->palette.alert.info.g, 1e-4);
    EXPECT_NEAR(info.b, theme->palette.alert.info.b, 1e-4);

    const auto warn = overlume::testing::alert_severity_base_color(r, 1);
    EXPECT_NEAR(warn.r, theme->palette.alert.warning.r, 1e-4);
    EXPECT_NEAR(warn.g, theme->palette.alert.warning.g, 1e-4);
    EXPECT_NEAR(warn.b, theme->palette.alert.warning.b, 1e-4);

    const auto crit = overlume::testing::alert_severity_base_color(r, 2);
    EXPECT_NEAR(crit.r, theme->palette.alert.critical.r, 1e-4);
    EXPECT_NEAR(crit.g, theme->palette.alert.critical.g, 1e-4);
    EXPECT_NEAR(crit.b, theme->palette.alert.critical.b, 1e-4);
    overlume::destroy_renderer(r);
}
