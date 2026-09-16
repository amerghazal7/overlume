// test_theme_showcase.cpp — palette-iteration harness (2026-09-16 session):
// a single, fast, repeatable capture that exercises the WHOLE theme palette
// in ONE frame, so a human can judge "which candidate theme looks best" by
// eye rather than by scrolling between a dozen narrow per-feature goldens.
//
// Opt-in and env-gated EXACTLY like test_environment_stream.cpp's own
// capture tests (EnvSourceCapture.Baked/Osm/Google) -- GTEST_SKIP()s by
// default, never requires network/token, never runs under a plain
// ctest/ci_visual_mode.sh invocation.
//
// Env contract:
//   MPVIZ_SHOWCASE=1              -- enables the capture (else GTEST_SKIP)
//   MPVIZ_SHOWCASE_THEME=<name>   -- theme name to load (default dark_adas)
//   MPVIZ_SHOWCASE_THEME_DIR=<dir> -- theme assets dir (default the shipped
//                                     assets/themes/ dir) -- point this at
//                                     assets/theme_variants/ (or any other
//                                     directory) to render a candidate theme
//                                     that isn't shipped
//   MPVIZ_SHOWCASE_OUT=<path.png> -- where to write the PNG (default
//                                     /tmp/theme_showcase_<theme>.png)
//
// Re-render loop this harness is FOR: edit a theme YAML, re-run this one
// gtest binary with the right env vars, look at the PNG -- no rebuild of
// the renderer itself needed (YAML is read at runtime), no other test
// suite touched, no golden comparison to keep in sync (this test never
// asserts pixel content -- it reports, honestly, the same way
// EnvSourceCapture's own captures do).
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

#include "golden.hpp"
#include "test_paths.hpp"

#include "stb_image_write.h"  // declarations only -- golden.cpp already
                              // defines STB_IMAGE_WRITE_IMPLEMENTATION and
                              // is linked into every test binary as an
                              // extra source (CMakeLists.txt's
                              // `_golden_cpp`), same shape
                              // test_environment_stream.cpp's own capture
                              // tests rely on.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

using mpviz::Vec3;

const std::string kTestTownDir =
    std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/environment_test_town_0";

// Same fixture anchor every other baked-environment test in this suite uses
// (test_environment.cpp, test_environment_stream.cpp) -- the baked backend
// doesn't re-derive placement from this at runtime (it's pre-baked in map
// frame), but set_environment_source() still takes one.
constexpr mpviz::GeoAnchor kAnchor{25.0803, 55.3910, 0.0};

// The fixture's own real footprint centroid (test_environment.cpp's
// kBuildingsCentroid, re-typed here rather than re-derived -- same
// "measured directly from the fixture .glb" fact, not a new measurement).
constexpr Vec3 kBuildingsCentroid{-109.2, -17.1, 3.0};

// The scene's local origin: where every reused golden.hpp helper's own
// baked-in "ego sits at (0,0,0)" assumption gets translated to. Chosen 20m
// behind and 6m off the buildings' own centroid Y, so ego + road + ribbons
// + objects all land in the same neighborhood as the ONE thing in this
// scene that can't be moved (the baked buildings, already placed in map
// frame by bake_environment.py) -- and specifically so the buildings sit
// AHEAD of the ego along the road, echoing ref-2's forward-looking framing.
constexpr Vec3 kSceneOrigin{kBuildingsCentroid.x - 20.0, kBuildingsCentroid.y - 6.0, 0.0};

Vec3 translated(const Vec3& p, double dx, double dy) {
    return Vec3{p.x + dx, p.y + dy, p.z};
}

// Renders at 1280x960 -- human eyes, not SSIM (no golden comparison in this
// file at all; see the header's own "never asserts pixel content" note).
constexpr uint32_t kWidth = 1280;
constexpr uint32_t kHeight = 960;

// Resolution-independent, no-golden-needed content check -- same shape as
// test_environment_stream.cpp's own EnvSourceCapture::analyze_capture
// (mean luminance instead of stddev here, since this report wants "mean
// luminance" specifically per the task, plus the same non-background
// fraction test).
struct ContentStats {
    double mean_luminance = 0.0;
    double non_background_fraction = 0.0;
};

double luminance(uint8_t r, uint8_t g, uint8_t b) {
    return 0.2126 * r + 0.7152 * g + 0.0722 * b;
}

ContentStats analyze_capture(const std::vector<uint8_t>& rgb, uint32_t width, uint32_t height) {
    const size_t n = static_cast<size_t>(width) * height;
    double sum = 0.0;
    for (size_t i = 0; i < n; ++i) sum += luminance(rgb[i * 3], rgb[i * 3 + 1], rgb[i * 3 + 2]);
    const double mean = sum / static_cast<double>(n);

    const double bgL = luminance(rgb[0], rgb[1], rgb[2]);  // top-left corner: sky, reliably
    constexpr double kTolerance = 10.0;
    size_t differing = 0;
    for (size_t i = 0; i < n; ++i) {
        const double l = luminance(rgb[i * 3], rgb[i * 3 + 1], rgb[i * 3 + 2]);
        if (std::abs(l - bgL) > kTolerance) ++differing;
    }
    return ContentStats{mean, static_cast<double>(differing) / static_cast<double>(n)};
}

std::string env_or(const char* name, const std::string& fallback) {
    const char* v = std::getenv(name);
    return (v && *v) ? std::string(v) : fallback;
}

}  // namespace

TEST(ThemeShowcase, Capture) {
    if (std::getenv("MPVIZ_SHOWCASE") == nullptr) {
        GTEST_SKIP() << "opt-in palette-iteration capture -- set MPVIZ_SHOWCASE=1 to run "
                        "(never required by ctest); see this file's header comment for the "
                        "full env contract (MPVIZ_SHOWCASE_THEME/_THEME_DIR/_OUT)";
    }

    const std::string themeName = env_or("MPVIZ_SHOWCASE_THEME", "dark_adas");
    const std::string themeDir = env_or("MPVIZ_SHOWCASE_THEME_DIR", kThemeDir);
    const std::string outPath =
        env_or("MPVIZ_SHOWCASE_OUT", "/tmp/theme_showcase_" + themeName + ".png");

    // Quality 2 (high, per RenderConfig::quality's 0/1/2 convention) --
    // this is a manual, opt-in, one-frame capture whose entire point is a
    // human judging "well lighted" (shadows, SSAO), not CI speed; every
    // other golden in this suite trades that off for quality 1 because it
    // renders every ctest run, this one never does.
    mpviz::RenderConfig cfg{kWidth, kHeight, /*quality=*/2, themeDir.c_str(), themeName.c_str()};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    if (!mpviz::theme_assets_loaded(r)) {
        std::cerr << "[ThemeShowcase] WARNING: '" << themeName << "' failed to load from '"
                  << themeDir << "' -- rendering the compiled-in fallback theme instead, NOT "
                  << "the requested candidate. Check the theme name/dir.\n";
    }

    // ── Baked environment buildings (task requirement 1/6) ────────────────
    ASSERT_TRUE(mpviz::set_environment_source(r, kTestTownDir.c_str(), kAnchor))
        << "baked fixture town failed to open -- see tests/fixtures/environment_test_town_0";

    // ── Ego model (task requirement 2/6) ───────────────────────────────────
    // No dedicated ego glTF ships anywhere in this repo (grep confirms: the
    // node's own robot_model_path param defaults empty) -- the shipped,
    // in-production look for the ego IS the themed clay-box fallback
    // build_ego_fallback() builds, keyed on palette.ego (theme.hpp's own
    // comment: "a dedicated clay.mat instance themed with its own
    // palette.ego token"). set_ego_model() must be called at least once for
    // that fallback box to exist at all (ego.cpp never builds it
    // automatically) -- same "/nonexistent path forces the fallback" shape
    // test_ego.cpp's own WrongDims-class tests use.
    constexpr Vec3 kEgoDims{4.5, 1.9, 1.6};  // sedan-scale clay box
    mpviz::set_ego_model(r, "/nonexistent/theme_showcase_ego.glb", kEgoDims);

    const double now = 42.0;  // arbitrary sim clock -- staleness fades below are all relative to it

    // ── Road surface + lane boundaries + centerline + crosswalk + road
    //    edges (task requirement 3/6) -- hand-built the same way
    //    test_map_elements.cpp builds individual MapElements (no existing
    //    helper assembles a whole road cross-section at once; every
    //    existing one exercises ONE MapKind in isolation for a unit
    //    assertion, not a full scene). ─────────────────────────────────────
    // Road runs along local +X (kSceneOrigin's own local frame), a
    // realistic 2-lane cross-section: lane_width 3.5m each side of the
    // centerline (matches the shipped theme schema's own
    // ribbon.lane_width_m convention for "a lane"), 0.6m shoulder out to
    // road_edge.
    constexpr double kHalfWidth = 3.5;
    constexpr double kEdgeOffset = kHalfWidth + 0.6;
    constexpr double kRoadX0 = -15.0;  // behind the ego
    constexpr double kRoadX1 = 45.0;   // ahead of the ego, past the buildings
    constexpr double kCrosswalkX = 12.0;  // ahead of the ego, between it and the buildings

    auto world = [](double lx, double ly) {
        return Vec3{kSceneOrigin.x + lx, kSceneOrigin.y + ly, 0.0};
    };

    const std::vector<Vec3> roadSurfacePts = {
        world(kRoadX0, -kHalfWidth), world(kRoadX1, -kHalfWidth),
        world(kRoadX1, kHalfWidth), world(kRoadX0, kHalfWidth)};
    const std::vector<Vec3> leftBoundaryPts = {world(kRoadX0, -kHalfWidth), world(kRoadX1, -kHalfWidth)};
    const std::vector<Vec3> rightBoundaryPts = {world(kRoadX0, kHalfWidth), world(kRoadX1, kHalfWidth)};
    const std::vector<Vec3> centerlinePts = {world(kRoadX0, 0.0), world(kRoadX1, 0.0)};
    const std::vector<Vec3> roadEdgeLeftPts = {world(kRoadX0, -kEdgeOffset), world(kRoadX1, -kEdgeOffset)};
    const std::vector<Vec3> roadEdgeRightPts = {world(kRoadX0, kEdgeOffset), world(kRoadX1, kEdgeOffset)};
    const std::vector<Vec3> crosswalkPts = {
        world(kCrosswalkX - 1.5, -kEdgeOffset - 0.3), world(kCrosswalkX + 1.5, -kEdgeOffset - 0.3),
        world(kCrosswalkX + 1.5, kEdgeOffset + 0.3), world(kCrosswalkX - 1.5, kEdgeOffset + 0.3)};

    auto make_elem = [&](const std::vector<Vec3>& pts, uint8_t isPolygon, mpviz::MapKind kind) {
        mpviz::MapElement e{};
        e.points = pts.data();
        e.point_count = static_cast<uint32_t>(pts.size());
        e.is_polygon = isPolygon;
        e.kind = kind;
        e.last_update_sec = now;
        return e;
    };
    const std::vector<mpviz::MapElement> mapElements = {
        make_elem(roadSurfacePts, 1, mpviz::MapKind::ROAD_SURFACE),
        make_elem(leftBoundaryPts, 0, mpviz::MapKind::LEFT_BOUNDARY),
        make_elem(rightBoundaryPts, 0, mpviz::MapKind::RIGHT_BOUNDARY),
        make_elem(centerlinePts, 0, mpviz::MapKind::CENTERLINE),
        make_elem(roadEdgeLeftPts, 0, mpviz::MapKind::ROAD_EDGE),
        make_elem(roadEdgeRightPts, 0, mpviz::MapKind::ROAD_EDGE),
        make_elem(crosswalkPts, 1, mpviz::MapKind::CROSSWALK),
    };

    // ── TrackedObjects across classes (task requirement 4/6) -- reuse
    //    golden.hpp's own make_mixed_class_objects() (CAR/TRUCK_VAN/BUS/
    //    PEDESTRIAN/CYCLIST/UNKNOWN, one of each) verbatim, then translate
    //    every position/path point from its own "near world origin" frame
    //    into this scene's frame. ─────────────────────────────────────────
    mpviz::testing::ObjectScene objects = mpviz::testing::make_mixed_class_objects(now);
    for (auto& p : objects.path_points) p = translated(p, kSceneOrigin.x, kSceneOrigin.y);
    for (auto& obj : objects.objects) obj.position = translated(obj.position, kSceneOrigin.x, kSceneOrigin.y);

    // ── Critical alert on one TrackedObject (task requirement 4/6, "coral
    //    accent") -- no existing golden.hpp helper produces a CRITICAL
    //    (severity 2) alert (make_sweep_and_predicted_alerts() only covers
    //    info/warning), so this one is hand-built, same shape as that
    //    helper's own AlertPolygon construction. A 5m box straddling
    //    objects.objects[0] (the CAR, per make_mixed_class_objects's own
    //    documented ordering) -- alert_polygons.cpp maps severity 2 to
    //    r->alertTint[2], theme.palette.alert.critical (verified against
    //    renderer.cpp's alertTints[] array order: info/warning/critical). */
    const Vec3 carPos = objects.objects[0].position;
    const std::vector<Vec3> criticalAlertPts = {
        {carPos.x - 2.5, carPos.y - 2.5, 0.0}, {carPos.x + 2.5, carPos.y - 2.5, 0.0},
        {carPos.x + 2.5, carPos.y + 2.5, 0.0}, {carPos.x - 2.5, carPos.y + 2.5, 0.0}};
    mpviz::AlertPolygon criticalAlert{};
    criticalAlert.points = criticalAlertPts.data();
    criticalAlert.point_count = static_cast<uint32_t>(criticalAlertPts.size());
    criticalAlert.severity = 2;  // critical -> theme.palette.alert.critical (the coral accent)
    criticalAlert.last_update_sec = now;  // fresh -- full severity alpha, not mid-fade

    // ── All three ribbon roles (task requirement 5/6) -- reuse
    //    golden.hpp's make_three_role_ribbons() verbatim (BEHAVIOR/GLOBAL/
    //    LOCAL, one corridor, all fresh), translated the same way the
    //    objects above are: that helper's own doc states its ego sits at
    //    local (0,0,0), which is exactly kSceneOrigin's definition, so
    //    translating its points by kSceneOrigin reproduces the SAME
    //    corridor-relative-to-ego shape, just moved to sit on this scene's
    //    road (the corridor already runs along the helper's local +X,
    //    matching the road's own +X layout above). ──────────────────────
    mpviz::testing::RibbonScene ribbons = mpviz::testing::make_three_role_ribbons(now);
    for (auto& p : ribbons.point_storage) p = translated(p, kSceneOrigin.x, kSceneOrigin.y);

    // ── Ground grid (task requirement 6/6) -- reuse golden.hpp's
    //    make_two_layer_grids() verbatim; GroundGridLayer::origin is a
    //    plain field (not a pointer into shared storage), so it translates
    //    directly. ─────────────────────────────────────────────────────────
    mpviz::testing::GridScene grids = mpviz::testing::make_two_layer_grids(now);
    for (auto& g : grids.grids) g.origin = translated(g.origin, kSceneOrigin.x, kSceneOrigin.y);

    mpviz::SceneGraph s{};
    s.sim_time_sec = now;
    s.ego = mpviz::EgoState{kSceneOrigin, /*heading_rad=*/0.0, /*speed_mps=*/8.0, /*valid=*/1};
    s.objects = objects.objects.data();
    s.object_count = static_cast<uint32_t>(objects.objects.size());
    s.paths = ribbons.ribbons.data();
    s.path_count = static_cast<uint32_t>(ribbons.ribbons.size());
    s.map_elements = mapElements.data();
    s.map_element_count = static_cast<uint32_t>(mapElements.size());
    s.grids = grids.grids.data();
    s.grid_count = static_cast<uint32_t>(grids.grids.size());
    s.alerts = &criticalAlert;
    s.alert_count = 1;
    mpviz::set_scene(r, s);

    // Elevated 3/4 chase-cam pose, echoing ref-2's own framing: behind and
    // above the ego, looking forward-and-across along the road toward the
    // buildings ahead -- NOT a top-down survey shot and NOT a ground-level
    // bumper cam, the same "elevated three-quarter" read ref-2 itself uses.
    // eye is 14m behind/14m to the side of the ego and 12m up (a steep
    // enough angle to also read the ground grid/road surface, not just
    // vertical faces); target sits 18m ahead and 5m across from the ego, at
    // ego-eye height (1.2m) rather than the ground, so the horizon/building
    // fronts are framed rather than the pavement immediately underfoot --
    // this lands just short of kBuildingsCentroid itself (by design: aiming
    // exactly at it would center the buildings and crop the ego/road out of
    // frame instead of "framing ego + road ahead + buildings" together).
    const mpviz::CameraPose pose{
        {kSceneOrigin.x - 14.0, kSceneOrigin.y - 14.0, 12.0},
        {kSceneOrigin.x + 18.0, kSceneOrigin.y + 5.0, 1.2},
        /*vfov_deg=*/55.0};

    // One warm-up frame to trigger the baked chunk load (same "one render
    // before the capture frame" shape as EnvironmentGolden.TestTown_DarkAdas
    // -- the baked backend's load is synchronous-by-next-frame, no network
    // pump loop needed, unlike the streaming/cesium backend).
    std::vector<uint8_t> warm(static_cast<size_t>(kWidth) * kHeight * 3);
    ASSERT_TRUE(mpviz::render_frame(r, pose, {warm.data(), kWidth, kHeight}));

    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3);
    ASSERT_TRUE(mpviz::render_frame(r, pose, {rgb.data(), kWidth, kHeight}));
    stbi_write_png(outPath.c_str(), static_cast<int>(kWidth), static_cast<int>(kHeight), 3, rgb.data(),
                   static_cast<int>(kWidth) * 3);

    const ContentStats stats = analyze_capture(rgb, kWidth, kHeight);
    std::cerr << "[ThemeShowcase] theme='" << themeName << "' theme_dir='" << themeDir << "' -> "
              << outPath << " mean_luminance=" << stats.mean_luminance
              << " non_background_fraction=" << stats.non_background_fraction << "\n";

    mpviz::destroy_renderer(r);
}
