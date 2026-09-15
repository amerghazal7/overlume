// test_environment_stream.cpp — StreamingEnvironmentSource behind the VM-052
// seam (VM-062). Same "no Filament/cesium type" boundary as every other
// tests/*.cpp: this is a plain C++17 TU with no cesium include dirs at all
// (CMakeLists.txt's per-test `-I src` grants environment_test_hooks.hpp,
// nothing cesium-flavored) -- Decision 3's quarantine holds even here.
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

#include "environment_test_hooks.hpp"
#include "golden.hpp"
#include "test_paths.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

const std::string kTestTownDir =
    std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/environment_test_town_0";
const std::string kIonFixtureDir =
    std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/environment_ion_fixture_0";

constexpr mpviz::Vec3 kChunk0Center{-128.0, -128.0, 0.0};

// PROVENANCE.md: the anchor lies inside tile_root.b3dm's own region, so the
// map-frame block center IS the map origin, by WgsToMap's own definition.
constexpr mpviz::Vec3 kFixtureBlockCenterMap{0.0, 0.0, 0.0};
constexpr mpviz::GeoAnchor kFixtureAnchor{25.0803, 55.3910, 0.0};

mpviz::CameraPose kStdPose{{0, -8, 3}, {0, 0, 0.5}, 60};

// Pumps render_frame() until either the streaming source has something
// loaded or `max_frames` ticks pass (cesium's own async load pipeline needs
// several ticks: request -> load thread -> main thread prepare -> next
// tick's tilesToRenderThisFrame).
uint64_t pump_until_loaded(mpviz::VisualRenderer* r, const mpviz::CameraPose& pose,
                           std::vector<uint8_t>& buf, int max_frames = 200) {
    uint64_t count = 0;
    for (int i = 0; i < max_frames; ++i) {
        if (!mpviz::render_frame(r, pose, {buf.data(), 320, 240})) return count;
        count = mpviz::testing::environment_loaded_chunk_count(r);
        if (count > 0) return count;
    }
    return count;
}

}  // namespace

// ── Step 1: dispatch + factory skeleton ─────────────────────────────────
TEST(EnvironmentStream, IonUriWithoutTokenIsNonFatalFalse) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ::unsetenv("CESIUM_ION_TOKEN");  // this test binary's env only
    mpviz::GeoAnchor a{25.0803, 55.3910, 0.0};
    EXPECT_FALSE(mpviz::set_environment_source(r, "ion://96188", a));
    std::vector<uint8_t> buf(320u * 240u * 3u);
    EXPECT_TRUE(mpviz::render_frame(r, kStdPose, {buf.data(), 320, 240}));
    mpviz::destroy_renderer(r);
}

TEST(EnvironmentStream, NonIonUriStillOpensBakedSource) {
    // Regression pin for Decision 5's "byte-identical baked path": the
    // existing fixture town still opens through the SAME entry point.
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    mpviz::GeoAnchor a{25.0803, 55.3910, 0.0};
    EXPECT_TRUE(mpviz::set_environment_source(r, kTestTownDir.c_str(), a));

    mpviz::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kChunk0Center;
    mpviz::set_scene(r, s);
    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_TRUE(mpviz::render_frame(r, kStdPose, {buf.data(), 320, 240}));
    EXPECT_GT(mpviz::testing::environment_loaded_chunk_count(r), 0u);
    mpviz::destroy_renderer(r);
}

// ── Step 2: the real cesium plumbing, fixture-served ────────────────────
TEST(EnvironmentStream, FixtureTilesLoadRenderAsClayAndCount) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(
        mpviz::testing::install_fixture_streaming_source(r, kIonFixtureDir.c_str(), kFixtureAnchor));

    mpviz::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    mpviz::set_scene(r, s);

    std::vector<uint8_t> buf(320u * 240u * 3u);
    const uint64_t loaded = pump_until_loaded(r, kStdPose, buf);
    EXPECT_GT(loaded, 0u);
    mpviz::destroy_renderer(r);
}

// ── Step 3: geo placement cross-pin (Epic 4's committed pin fixture) ─────
// Same truth table as test_geo_anchor.cpp's own cross-language pin, applied
// to the STREAMING transform chain (ecef_to_map * wgs_to_ecef) instead of
// WgsToMap directly -- three implementations, one truth table.
TEST(EnvironmentStream, EcefToMapAgreesWithCppPinWithinHalfMeter) {
    const std::string path = std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/geo_anchor_cpp_pin_0.json";
    std::ifstream file(path);
    ASSERT_TRUE(file) << "missing fixture: " << path;
    std::stringstream ss;
    ss << file.rdbuf();
    const std::string json = ss.str();

    // ponytail: a handful of numeric literals pulled straight from the
    // committed fixture (values re-typed here, not re-derived) -- a real
    // JSON parser is not worth adding to this test binary for 5 fixed
    // probes; if the fixture's numbers ever change, this test's literals
    // must be updated alongside it (same convention as the fixture's own
    // provenance note: "not re-derived independently").
    ASSERT_NE(json.find("\"origin_lat_deg\": 25.0803"), std::string::npos);
    ASSERT_NE(json.find("\"origin_lon_deg\": 55.391"), std::string::npos);

    struct Probe {
        double lat, lon, heading_rad, map_x, map_y, map_z;
    };
    const std::vector<Probe> probes = {
        {25.0803, 55.391, 0.0, 0.0, 0.0, 0.0},
        {25.0903, 55.391, 0.0, 1111.9492664458, 0.0, 0.0},
        {25.0803, 55.401, 0.0, 0.0, -1007.1086827547, 0.0},
        {25.0653, 55.411, 0.0, -1667.9238996684, -2014.2173655101, 0.0},
        // probes_nonzero_heading -- pins the heading-rotation term the
        // heading-0 probes above cannot see.
        {25.085, 55.395, 0.35, 629.0654991903, -199.2162324121, 0.0},
    };
    // VM-062 gate round 1, Finding 1: compute_ecef_to_map() now rescales
    // cesium-native's true-ellipsoid ENU down to the SAME fixed-sphere
    // model geo_anchor.cpp's WgsToMap uses (kEarthRadiusM,
    // geo_anchor.cpp:14-20) before applying the shared heading rotation --
    // see compute_ecef_to_map's own comment. The residual here is
    // curvature/rounding only, so the plan's original flat 0.5 m bar
    // (restored, not widened) is what Step 3 actually measures.
    //
    // Error is measured over (x, y) only, not the probe's z: WgsToMap
    // itself never models height curvature -- it returns map z = alt_m
    // verbatim regardless of distance from the anchor (geo_anchor.cpp:36),
    // so `want.z` is trivially 0 at every probe here and was never part of
    // what this cross-pin validates. cesium's tangent-plane ENU, by
    // contrast, DOES report the true geometric sag below the tangent plane
    // (~0.54 m at this fixture's ~2.6 km probe, d^2/(2R) as expected) --
    // real curvature, not a bug, and not something the east/north sphere
    // rescale above touches or should. NOTE (gate round 2): the sag is
    // d^2/(2R) -- 0.08 m at 1 km, 0.54 m at 2.6 km, 7.85 m at 10 km --
    // quadratic in distance from the anchor, so the CONSTANT
    // `kStreamHeightOffsetM` cannot cancel it, and it is not expressible in
    // the single rigid 4x4 the design commits to; if streamed tiles visibly
    // sink at long range, that is a Task 4 / follow-up item, not a knob
    // that already exists. z IS asserted below, against the sag model
    // itself, so an up-axis/z-scale regression still fails this test.
    for (const Probe& p : probes) {
        double x = 0, y = 0, z = 0;
        ASSERT_TRUE(mpviz::testing::ecef_to_map_probe(25.0803, 55.391, p.heading_rad, p.lat, p.lon,
                                                       0.0, &x, &y, &z));
        const double err = std::sqrt((x - p.map_x) * (x - p.map_x) + (y - p.map_y) * (y - p.map_y));
        EXPECT_LT(err, 0.5) << "lat=" << p.lat << " lon=" << p.lon << " heading=" << p.heading_rad
                             << " got=(" << x << "," << y << "," << z << ") want=(" << p.map_x << ","
                             << p.map_y << "," << p.map_z << ") err=" << err;
        // z is asserted against the tangent-plane sag model itself,
        // -d^2/(2R) (VM-062 gate round 2 minor): a genuine up-axis
        // inversion or z-scale regression in compute_ecef_to_map must fail
        // HERE -- the horizontal bar above cannot see it.
        const double want_z = -(p.map_x * p.map_x + p.map_y * p.map_y) / (2.0 * 6371000.0);
        EXPECT_NEAR(z, want_z, 0.05) << "lat=" << p.lat << " lon=" << p.lon
                                     << " z=" << z << " want_z(sag)=" << want_z;
    }
}

// ── Step 4: disk cache proves itself (offline second-run reload) ────────
TEST(EnvironmentStream, DiskCacheServesTilesWithNetworkDead) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // Run 1: healthy fixture accessor, warms the on-disk sqlite cache
    // (Decision 10) in the per-process temp dir test_cache_dir() names
    // (under std::filesystem::temp_directory_path(), NOT the committed
    // fixture tree -- gate round 1 finding 4).
    auto* handle1 = mpviz::testing::install_fixture_streaming_source_with_fallback(
        r, kIonFixtureDir.c_str(), /*fallback_baked_dir=*/nullptr, kFixtureAnchor);
    ASSERT_NE(handle1, nullptr);
    mpviz::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    mpviz::set_scene(r, s);
    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GT(pump_until_loaded(r, kStdPose, buf), 0u);
    // The sqlite cache write for each response happens on a background
    // thread pool, independent of (and not gated by) content ingestion --
    // pump extra ticks so those writes settle before tearing this source
    // down, or the SECOND source below could race an incomplete cache.
    for (int i = 0; i < 30; ++i) mpviz::render_frame(r, kStdPose, {buf.data(), 320, 240});

    // Run 2: shares run 1's cache because test_cache_dir() is static
    // within the process (same per-process temp path, Decision 10), but the
    // "network" killed from tick 0 -- any tile that still loads came from
    // the sqlite cache, not a live fetch.
    auto* handle2 = mpviz::testing::install_fixture_streaming_source_with_fallback(
        r, kIonFixtureDir.c_str(), /*fallback_baked_dir=*/nullptr, kFixtureAnchor);
    ASSERT_NE(handle2, nullptr);
    mpviz::testing::kill_fixture_network(handle2);
    mpviz::set_scene(r, s);  // re-publish ego (install_* only swaps r->environmentSource)
    const uint64_t loadedOffline = pump_until_loaded(r, kStdPose, buf);
    EXPECT_GT(loadedOffline, 0u);
    mpviz::destroy_renderer(r);
}

// ── Step 5: golden + theming (P3 golden scoping, one theme) ─────────────
TEST(EnvironmentStreamGolden, FixtureBlock_DarkAdas) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(
        mpviz::testing::install_fixture_streaming_source(r, kIonFixtureDir.c_str(), kFixtureAnchor));

    mpviz::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    mpviz::set_scene(r, s);

    // Framed on the fixture tiles' own real footprint centroid -- measured
    // directly from the placed FilamentAsset's world-space bounding box
    // (~(69, 472, -3)), NOT the anchor point or the tile's nominal region
    // center (same "frame on the real footprint" lesson
    // test_environment.cpp's own kBuildingsCentroid comment documents).
    mpviz::CameraPose pose{{149, 352, 90}, {69, 472, 0}, 60.0};
    std::vector<uint8_t> warm(320u * 240u * 3u);
    ASSERT_GT(pump_until_loaded(r, pose, warm), 0u);
    // Keep pumping so the fixture's other two tiles (not just whichever
    // loads first) have a chance to finish loading too before the golden
    // comparison frame -- more of the real content on screen.
    for (int i = 0; i < 60; ++i) mpviz::render_frame(r, pose, {warm.data(), 320, 240});

    double ssim = mpviz::testing::render_and_compare(
        r, pose, MPVIZ_TEST_DATA_DIR "/tests/goldens/environment_stream_dark_adas.png",
        "/tmp/environment_stream_dark_adas_actual.png");
    EXPECT_GT(ssim, 0.98);
    mpviz::destroy_renderer(r);
}

// ── Step 6: perf check (dev-box proxy, same honesty class as
//    EnvironmentPerf.RenderMsDeltaWithTestTownLoaded) ─────────────────────
TEST(EnvironmentStreamPerf, RenderMsDeltaAndWorstFrameWithFixtureLoaded) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    mpviz::CameraPose pose{{149, 352, 90}, {69, 472, 0}, 60.0};
    std::vector<uint8_t> buf(320u * 240u * 3u);
    constexpr int kFrames = 60;

    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(
        mpviz::testing::install_fixture_streaming_source(r, kIonFixtureDir.c_str(), kFixtureAnchor));
    mpviz::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    mpviz::set_scene(r, s);
    ASSERT_GT(pump_until_loaded(r, pose, buf), 0u);  // warm: tiles already loaded before timing

    double worstMs = 0.0;
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < kFrames; ++i) {
        const auto f0 = std::chrono::steady_clock::now();
        mpviz::render_frame(r, pose, {buf.data(), 320, 240});
        const auto f1 = std::chrono::steady_clock::now();
        worstMs = std::max(worstMs, std::chrono::duration<double, std::milli>(f1 - f0).count());
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double meanWithMs = std::chrono::duration<double, std::milli>(t1 - t0).count() / kFrames;
    mpviz::destroy_renderer(r);

    auto* r2 = mpviz::create_renderer(cfg);
    ASSERT_NE(r2, nullptr);
    mpviz::set_scene(r2, mpviz::SceneGraph{});
    mpviz::render_frame(r2, pose, {buf.data(), 320, 240});
    const auto b0 = std::chrono::steady_clock::now();
    for (int i = 0; i < kFrames; ++i) mpviz::render_frame(r2, pose, {buf.data(), 320, 240});
    const auto b1 = std::chrono::steady_clock::now();
    const double meanWithoutMs = std::chrono::duration<double, std::milli>(b1 - b0).count() / kFrames;
    mpviz::destroy_renderer(r2);

    std::cerr << "[EnvironmentStreamPerf] mean render_ms without=" << meanWithoutMs
              << " with=" << meanWithMs << " delta=" << (meanWithMs - meanWithoutMs)
              << " worst_single_frame_ms=" << worstMs << "\n";
    SUCCEED();
}

// Fixture provenance (Step 0): tests/fixtures/environment_ion_fixture_0/ --
// see that directory's own PROVENANCE.md (real ion OSM Buildings tiles,
// fetched once with the live token, tileset.json hand-pruned to a
// self-contained 3-tile subtree with relative local uris; token appears
// nowhere in the fixture, verified by grep before commit).
