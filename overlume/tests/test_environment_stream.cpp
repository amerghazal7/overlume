// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "environment_test_hooks.hpp"
#include "golden.hpp"
#include "imagery_fixture.hpp"
#include "mapbox_test_hooks.hpp"
#include "quantized_mesh_fixture.hpp"
#include "test_paths.hpp"

#include "stb_image_write.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <sstream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace {

const std::string kTestTownDir =
    std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/environment_test_town_0";
const std::string kTilesFixtureDir =
    std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/environment_tiles_fixture_0";
const std::string kTilesFixtureFallbackDir =
    std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/environment_tiles_fixture_fallback_0";
const std::string kSessionGroundFixtureDir =
    std::string(OVERLUME_TEST_DATA_DIR) +
    "/tests/fixtures/environment_tiles_fixture_session_ground_0";
const overlume::GeoAnchor kSessionGroundAnchor{25.08001258, 55.38847719, 90.1178 * M_PI / 180.0};

constexpr overlume::Vec3 kChunk0Center{-128.0, -128.0, 0.0};

constexpr overlume::Vec3 kFixtureBlockCenterMap{0.0, 0.0, 0.0};
constexpr overlume::GeoAnchor kFixtureAnchor{25.0803, 55.3910, 0.0};

overlume::CameraPose kStdPose{{0, -8, 3}, {0, 0, 0.5}, 60};

#ifdef _WIN32
// MSVC's CRT has no setenv/unsetenv; _putenv_s with "" removes the variable.
int setenv(const char* name, const char* value, int) { return _putenv_s(name, value); }
int unsetenv(const char* name) { return _putenv_s(name, ""); }
#endif

class ScopedUnsetEnv {
public:
    explicit ScopedUnsetEnv(const char* name) : name_(name) {
        const char* v = std::getenv(name);
        if (v) {
            had_value_ = true;
            saved_ = v;
        }
        ::unsetenv(name);
    }
    ~ScopedUnsetEnv() {
        if (had_value_)
            ::setenv(name_, saved_.c_str(), 1);
        else
            ::unsetenv(name_);
    }
    ScopedUnsetEnv(const ScopedUnsetEnv&) = delete;
    ScopedUnsetEnv& operator=(const ScopedUnsetEnv&) = delete;

private:
    const char* name_;
    bool had_value_ = false;
    std::string saved_;
};

uint64_t pump_until_loaded(overlume::VisualRenderer* r, const overlume::CameraPose& pose,
                           std::vector<uint8_t>& buf, int max_frames = 200, uint64_t want = 1) {
    uint64_t count = 0;
    for (int i = 0; i < max_frames; ++i) {
        if (!overlume::render_frame(r, pose, {buf.data(), 320, 240})) return count;
        count = overlume::testing::environment_loaded_chunk_count(r);
        if (count >= want) return count;
    }
    return count;
}

}

TEST(EnvironmentStream, IonUriWithoutTokenIsNonFatalFalse) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ScopedUnsetEnv no_token("CESIUM_ION_TOKEN");
    overlume::GeoAnchor a{25.0803, 55.3910, 0.0};
    EXPECT_FALSE(overlume::set_environment_source(r, "ion://96188", a));
    std::vector<uint8_t> buf(320u * 240u * 3u);
    EXPECT_TRUE(overlume::render_frame(r, kStdPose, {buf.data(), 320, 240}));
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, NonIonUriStillOpensBakedSource) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::GeoAnchor a{25.0803, 55.3910, 0.0};
    EXPECT_TRUE(overlume::set_environment_source(r, kTestTownDir.c_str(), a));

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kChunk0Center;
    overlume::set_scene(r, s);
    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_TRUE(overlume::render_frame(r, kStdPose, {buf.data(), 320, 240}));
    EXPECT_GT(overlume::testing::environment_loaded_chunk_count(r), 0u);
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, FixtureTilesLoadRenderAsClayAndCount) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(r, kTilesFixtureDir.c_str(),
                                                                    kFixtureAnchor));

    std::vector<uint8_t> buf(320u * 240u * 3u);
    auto settle_count = [&](overlume::Vec3 pos) -> uint64_t {
        overlume::SceneGraph s{};
        s.ego.valid = 1;
        s.ego.position = pos;
        overlume::set_scene(r, s);
        const overlume::CameraPose pose{
            {pos.x, pos.y, pos.z + 5000.0}, {pos.x + 0.001, pos.y, pos.z + 5001.0}, 60};
        uint64_t count = 0;
        for (int tick = 0, stableTicks = 0; tick < 500 && stableTicks < 10; ++tick) {
            overlume::render_frame(r, pose, {buf.data(), 320, 240});
            const uint64_t next = overlume::testing::environment_loaded_chunk_count(r);
            if (next == count && count > 0) {
                ++stableTicks;
            } else {
                count = next;
                stableTicks = 0;
            }
        }
        return count;
    };

    EXPECT_EQ(settle_count(kFixtureBlockCenterMap), 1u)
        << "tile_root (single-buffer+NORMAL, +_BATCHID) must load from the anchor's own "
           "position";

    double xA = 0, yA = 0, zA = 0;
    ASSERT_TRUE(overlume::testing::ecef_to_map_probe(
        kFixtureAnchor.origin_lat_deg, kFixtureAnchor.origin_lon_deg, 0.0, 0.0, 25.114938650000003,
        55.39303970000001, 0.0, &xA, &yA, &zA));
    EXPECT_EQ(settle_count(overlume::Vec3{xA, yA, zA}), 1u)
        << "tile_a (multi-buffer) must load once ego sits inside its own region -- a "
           "consolidate_buffers() regression on the multi-buffer merge would drop this to 0";

    double xB = 0, yB = 0, zB = 0;
    ASSERT_TRUE(overlume::testing::ecef_to_map_probe(
        kFixtureAnchor.origin_lat_deg, kFixtureAnchor.origin_lon_deg, 0.0, 0.0, 25.127951550000006,
        55.430998900000006, 0.0, &xB, &yB, &zB));
    EXPECT_EQ(settle_count(overlume::Vec3{xB, yB, zB}), 1u)
        << "tile_b (no NORMAL) must load once ego sits inside its own region -- an "
           "ensure_flat_normals() regression on the streamed (not baked-chunk) path would "
           "drop this to 0";

    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, RadiusExcludesTileOutsideConfiguredRadius) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    std::vector<uint8_t> buf(320u * 240u * 3u);
    overlume::SceneGraph farScene{};
    farScene.ego.valid = 1;
    farScene.ego.position = overlume::Vec3{0.0, -8000.0, 0.0};

    auto* wide = overlume::create_renderer(cfg);
    if (!wide) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(
        wide, kTilesFixtureDir.c_str(), kFixtureAnchor, false, false, 1.0e9));
    overlume::set_scene(wide, farScene);
    for (int i = 0; i < 60; ++i) overlume::render_frame(wide, kStdPose, {buf.data(), 320, 240});
    EXPECT_GT(overlume::testing::environment_loaded_chunk_count(wide), 0u)
        << "with an effectively unbounded radius the render camera's own selection must still "
           "bring the fixture tiles into the scene, ego position notwithstanding";
    overlume::destroy_renderer(wide);

    auto* tight = overlume::create_renderer(cfg);
    if (!tight) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(
        tight, kTilesFixtureDir.c_str(), kFixtureAnchor, false, false, 0.001));
    overlume::set_scene(tight, farScene);
    for (int i = 0; i < 60; ++i) overlume::render_frame(tight, kStdPose, {buf.data(), 320, 240});
    EXPECT_EQ(overlume::testing::environment_loaded_chunk_count(tight), 0u)
        << "same ego, same camera, near-zero radius: every selected tile must be culled";
    overlume::destroy_renderer(tight);
}

TEST(EnvironmentStream, ThemeTileRadiusGatesUnlessUriOverrides) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    std::vector<uint8_t> buf(320u * 240u * 3u);
    overlume::SceneGraph farScene{};
    farScene.ego.valid = 1;
    farScene.ego.position = overlume::Vec3{0.0, -8000.0, 0.0};
    auto loaded_after_settle = [&](const char* theme, std::optional<double> uriRadius) {
        overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), theme};
        auto* r = overlume::create_renderer(cfg);
        if (!r) return std::optional<uint64_t>{};
        EXPECT_TRUE(overlume::testing::install_fixture_streaming_source(
            r, kTilesFixtureDir.c_str(), kFixtureAnchor, false, false, uriRadius));
        overlume::set_scene(r, farScene);
        for (int i = 0; i < 60; ++i) overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});
        const uint64_t n = overlume::testing::environment_loaded_chunk_count(r);
        overlume::destroy_renderer(r);
        return std::optional<uint64_t>{n};
    };

    const auto wideTheme = loaded_after_settle("tile_radius_wide", std::nullopt);
    if (!wideTheme) GTEST_SKIP() << "no GPU/EGL";
    EXPECT_GT(*wideTheme, 0u)
        << "no radius= on the URI: the theme's 1e9 m tile_radius_m must let 8 km-away tiles "
           "in (a hard-coded 700 m would cull them all)";

    const auto tinyTheme = loaded_after_settle("tile_radius_tiny", std::nullopt);
    ASSERT_TRUE(tinyTheme);
    EXPECT_EQ(*tinyTheme, 0u) << "the theme's 0.001 m tile_radius_m culls every tile";

    const auto tinyOverridden = loaded_after_settle("tile_radius_tiny", 1.0e9);
    ASSERT_TRUE(tinyOverridden);
    EXPECT_GT(*tinyOverridden, 0u) << "an explicit radius= on the URI overrides the theme value";
}

TEST(EnvironmentStream, TileSelectionUsesRenderCameraAsSecondFrustum) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(r, kTilesFixtureDir.c_str(),
                                                                    kFixtureAnchor));
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);

    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_TRUE(overlume::render_frame(r, kStdPose, {buf.data(), 320, 240}));
    EXPECT_EQ(overlume::testing::environment_stream_last_view_frustum_count(r), 1);
    ASSERT_TRUE(overlume::render_frame(r, kStdPose, {buf.data(), 320, 240}));

    EXPECT_EQ(overlume::testing::environment_stream_last_view_frustum_count(r), 2);

    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, NodeMatrixEncodingRendersIdenticallyToEcefEncoding) {
    const std::string kSessionEcefDir = std::string(OVERLUME_TEST_DATA_DIR) +
                                        "/tests/fixtures/environment_tiles_fixture_session_ecef_0";
    const std::string kSessionNodeMatrixDir =
        std::string(OVERLUME_TEST_DATA_DIR) +
        "/tests/fixtures/environment_tiles_fixture_session_nodematrix_0";
    const overlume::GeoAnchor kSessionAnchor{25.08001258, 55.38847719, 90.1178 * M_PI / 180.0};
    double targetX = 0, targetY = 0, targetZ = 0;
    ASSERT_TRUE(overlume::testing::ecef_to_map_probe(
        kSessionAnchor.origin_lat_deg, kSessionAnchor.origin_lon_deg, kSessionAnchor.heading_rad,
        0.0, 25.080637330000002, 55.38380269000002, 0.0, &targetX, &targetY, &targetZ));
    overlume::CameraPose pose{
        {targetX + 80, targetY - 120, targetZ + 90}, {targetX, targetY, targetZ}, 60.0};

    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};

    auto* rA = overlume::create_renderer(cfg);
    if (!rA) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(rA, kSessionEcefDir.c_str(),
                                                                    kSessionAnchor));
    overlume::SceneGraph sA{};
    sA.ego.valid = 1;
    sA.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(rA, sA);
    std::vector<uint8_t> bufA(320u * 240u * 3u);
    ASSERT_GT(pump_until_loaded(rA, pose, bufA), 0u)
        << "session_ecef fixture must load from its own (shifted) anchor's block center";
    for (int i = 0; i < 60; ++i) overlume::render_frame(rA, pose, {bufA.data(), 320, 240});
    overlume::testing::render_and_compare(rA, pose, "/nonexistent_no_golden.png",
                                          OVERLUME_TMP_DIR
                                          "/environment_stream_session_ecef_actual.png");
    overlume::destroy_renderer(rA);

    auto* rB = overlume::create_renderer(cfg);
    if (!rB) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(
        rB, kSessionNodeMatrixDir.c_str(), kSessionAnchor));
    overlume::SceneGraph sB{};
    sB.ego.valid = 1;
    sB.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(rB, sB);
    std::vector<uint8_t> bufB(320u * 240u * 3u);
    ASSERT_GT(pump_until_loaded(rB, pose, bufB), 0u)
        << "session_nodematrix fixture must load too -- loading doesn't inspect vertex "
           "correctness, only the render below can see the mis-corrected positions";
    for (int i = 0; i < 60; ++i) overlume::render_frame(rB, pose, {bufB.data(), 320, 240});

    const double ssim = overlume::testing::render_and_compare(
        rB, pose, OVERLUME_TMP_DIR "/environment_stream_session_ecef_actual.png",
        OVERLUME_TMP_DIR "/environment_stream_session_nodematrix_actual.png");
    EXPECT_GE(ssim, 0.97)
        << "node-matrix-encoded tiles (Google's own convention) must render identically to "
           "the SAME geometry's identity-node encoding -- strip_attributes_and_correct_heights() "
           "ignoring the primitive's own node transform sends node-matrix vertices to the wrong "
           "ECEF position (root-transform-only correction), got ssim="
        << ssim;
    overlume::destroy_renderer(rB);
}

TEST(EnvironmentStream, EcefToMapAgreesWithCppPinWithinHalfMeter) {
    const std::string path =
        std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/geo_anchor_cpp_pin_0.json";
    std::ifstream file(path);
    ASSERT_TRUE(file) << "missing fixture: " << path;
    std::stringstream ss;
    ss << file.rdbuf();
    const std::string json = ss.str();

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
        {25.085, 55.395, 0.35, 629.0654991903, -199.2162324121, 0.0},
    };
    for (const Probe& p : probes) {
        double x = 0, y = 0, z = 0;
        ASSERT_TRUE(overlume::testing::ecef_to_map_probe(25.0803, 55.391, p.heading_rad, 0.0, p.lat,
                                                         p.lon, 0.0, &x, &y, &z));
        const double err = std::sqrt((x - p.map_x) * (x - p.map_x) + (y - p.map_y) * (y - p.map_y));
        EXPECT_LT(err, 0.5) << "lat=" << p.lat << " lon=" << p.lon << " heading=" << p.heading_rad
                            << " got=(" << x << "," << y << "," << z << ") want=(" << p.map_x << ","
                            << p.map_y << "," << p.map_z << ") err=" << err;
        const double want_z = -(p.map_x * p.map_x + p.map_y * p.map_y) / (2.0 * 6371000.0);
        EXPECT_NEAR(z, want_z, 0.05)
            << "lat=" << p.lat << " lon=" << p.lon << " z=" << z << " want_z(sag)=" << want_z;
    }
}

TEST(EnvironmentStream, EllipsoidHeightCorrectionRemovesTangentPlaneSag) {
    constexpr double kAnchorLat = 25.0803, kAnchorLon = 55.391;
    constexpr double kSphereRadiusM = 6371000.0;
    const std::vector<double> offsets_m = {0.0, 1000.0, 2600.0, 10000.0};
    for (const double offset_m : offsets_m) {
        const double lat = kAnchorLat + (offset_m / kSphereRadiusM) * (180.0 / M_PI);

        double zUncorrected = 0.0, zCorrected = 0.0;
        ASSERT_TRUE(overlume::testing::ecef_height_correction_probe(
            kAnchorLat, kAnchorLon, 0.0, 0.0, lat, kAnchorLon, 0.0, &zUncorrected, &zCorrected));

        const double wantSag = -(offset_m * offset_m) / (2.0 * kSphereRadiusM);
        EXPECT_NEAR(zUncorrected, wantSag, 0.05)
            << "offset=" << offset_m << "m: uncorrected z=" << zUncorrected << " should sag ~"
            << wantSag << "m below the tangent plane -- the bug this "
            << "follow-up fixes";
        EXPECT_LT(std::abs(zCorrected), 0.005)
            << "offset=" << offset_m << "m: corrected z=" << zCorrected
            << " should sit on the map plane within 5mm";
        EXPECT_NEAR(zCorrected - zUncorrected, -wantSag, 0.05)
            << "offset=" << offset_m << "m: correction should remove exactly the sag amount";
    }
}

TEST(EnvironmentStream, AnchorHeightShiftsCorrectedZ) {
    constexpr double kAnchorLat = 25.0803, kAnchorLon = 55.391;
    constexpr double kOriginHeightM = 25.0;

    double zUncorrected = 0.0, zCorrected = 0.0;
    ASSERT_TRUE(overlume::testing::ecef_height_correction_probe(
        kAnchorLat, kAnchorLon, 0.0, kOriginHeightM, kAnchorLat, kAnchorLon, 25.0, &zUncorrected,
        &zCorrected));
    EXPECT_NEAR(zUncorrected, 0.0, 1e-3)
        << "the ENU origin must sit at origin_height_m, not on the ellipsoid; got " << zUncorrected;
    EXPECT_NEAR(zCorrected, 0.0, 1e-3)
        << "a point at the anchor's own altitude (25.0 m) must correct to z=0 when "
           "origin_height_m=25.0, got "
        << zCorrected;

    ASSERT_TRUE(overlume::testing::ecef_height_correction_probe(
        kAnchorLat, kAnchorLon, 0.0, kOriginHeightM, kAnchorLat, kAnchorLon, 30.0, &zUncorrected,
        &zCorrected));
    EXPECT_NEAR(zCorrected, 5.0, 1e-3)
        << "a point 5 m above the anchor's own altitude must correct to z=5.0 when "
           "origin_height_m=25.0, got "
        << zCorrected;
}

TEST(EnvironmentStream, DiskCacheServesTilesWithNetworkDead) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    auto* handle1 = overlume::testing::install_fixture_streaming_source_with_fallback(
        r, kTilesFixtureDir.c_str(), nullptr, kFixtureAnchor);
    ASSERT_NE(handle1, nullptr);
    EXPECT_TRUE(overlume::testing::environment_stream_has_disk_cache(r))
        << "a source opened with a cache directory reads through the cache; this is the positive "
           "control for the mapbox:// no-disk-cache check";
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);
    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GT(pump_until_loaded(r, kStdPose, buf), 0u);
    for (int i = 0; i < 30; ++i) overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});

    auto* handle2 = overlume::testing::install_fixture_streaming_source_with_fallback(
        r, kTilesFixtureDir.c_str(), nullptr, kFixtureAnchor);
    ASSERT_NE(handle2, nullptr);
    overlume::testing::kill_fixture_network(handle2);
    overlume::set_scene(r, s);
    const uint64_t loadedOffline = pump_until_loaded(r, kStdPose, buf);
    EXPECT_GT(loadedOffline, 0u);
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamGolden, FixtureBlock_DarkAdas) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(r, kTilesFixtureDir.c_str(),
                                                                    kFixtureAnchor));

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);

    overlume::CameraPose pose{{149, 352, 90}, {69, 472, 0}, 60.0};
    std::vector<uint8_t> warm(320u * 240u * 3u);
    ASSERT_GT(pump_until_loaded(r, pose, warm), 0u);
    for (int i = 0; i < 60; ++i) overlume::render_frame(r, pose, {warm.data(), 320, 240});

    double ssim = overlume::testing::render_and_compare(
        r, pose, OVERLUME_TEST_DATA_DIR "/tests/goldens/environment_stream_dark_adas.png",
        OVERLUME_TMP_DIR "/environment_stream_dark_adas_actual.png");
    EXPECT_GT(ssim, overlume::testing::kSsimMin);
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamGolden, SetVisibleFalseHidesFixtureTilesWithoutTearingDown) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(r, kTilesFixtureDir.c_str(),
                                                                    kFixtureAnchor));

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);
    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GT(pump_until_loaded(r, kStdPose, buf), 0u);
    for (int i = 0; i < 30; ++i) overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});
    const uint64_t loadedBefore = overlume::testing::environment_loaded_chunk_count(r);
    ASSERT_GT(loadedBefore, 0u);
    ASSERT_EQ(overlume::testing::environment_scene_membership_count(r), loadedBefore)
        << "every loaded tile should start out actually in the scene";

    ASSERT_TRUE(overlume::set_environment_visible(r, false));
    ASSERT_TRUE(overlume::render_frame(r, kStdPose, {buf.data(), 320, 240}));
    EXPECT_EQ(overlume::testing::environment_loaded_chunk_count(r), loadedBefore);
    EXPECT_EQ(overlume::testing::environment_scene_membership_count(r), 0u)
        << "streamed tiles are still in the Filament scene after "
           "set_environment_visible(r, false)";

    ASSERT_TRUE(overlume::set_environment_visible(r, true));
    ASSERT_TRUE(overlume::render_frame(r, kStdPose, {buf.data(), 320, 240}));
    EXPECT_EQ(overlume::testing::environment_loaded_chunk_count(r), loadedBefore);
    EXPECT_EQ(overlume::testing::environment_scene_membership_count(r), loadedBefore)
        << "re-showing did not put every already-loaded tile back into the scene";
    overlume::destroy_renderer(r);
}

namespace {
size_t count_differing_bytes_with_tolerance(const std::vector<uint8_t>& a,
                                            const std::vector<uint8_t>& b, int tolerance) {
    size_t diff = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::abs(static_cast<int>(a[i]) - static_cast<int>(b[i])) > tolerance) ++diff;
    }
    return diff;
}
}

TEST(EnvironmentStreamGolden, TileLoadedWhileHiddenDoesNotPopIntoView) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(r, kTilesFixtureDir.c_str(),
                                                                    kFixtureAnchor));
    ASSERT_TRUE(overlume::set_environment_visible(r, false));

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{149, 352, 90}, {69, 472, 0}, 60.0};
    const size_t nBytes = 320u * 240u * 3u;
    std::vector<uint8_t> hiddenBuf(nBytes);
    ASSERT_GT(pump_until_loaded(r, pose, hiddenBuf), 0u)
        << "loading itself must still happen while hidden";
    for (int i = 0; i < 30; ++i) overlume::render_frame(r, pose, {hiddenBuf.data(), 320, 240});
    ASSERT_TRUE(overlume::render_frame(r, pose, {hiddenBuf.data(), 320, 240}));
    EXPECT_EQ(overlume::testing::environment_scene_membership_count(r), 0u)
        << "a tile loaded while hidden was added to the Filament scene anyway";

    auto* rNoEnv = overlume::create_renderer(cfg);
    ASSERT_NE(rNoEnv, nullptr);
    overlume::set_scene(rNoEnv, s);
    std::vector<uint8_t> noEnvBuf(nBytes);
    ASSERT_TRUE(overlume::render_frame(rNoEnv, pose, {noEnvBuf.data(), 320, 240}));
    overlume::destroy_renderer(rNoEnv);

    constexpr int kTolerance = 8;
    const size_t diffFromNoEnv =
        count_differing_bytes_with_tolerance(hiddenBuf, noEnvBuf, kTolerance);
    EXPECT_LT(diffFromNoEnv, 100u)
        << "a tile loaded while hidden popped into view (" << diffFromNoEnv << "/" << nBytes
        << " bytes differ beyond rendering noise from a renderer with no environment source at "
           "all)";

    ASSERT_TRUE(overlume::set_environment_visible(r, true));
    std::vector<uint8_t> shownBuf(nBytes);
    ASSERT_TRUE(overlume::render_frame(r, pose, {shownBuf.data(), 320, 240}));
    const size_t diffShown = count_differing_bytes_with_tolerance(hiddenBuf, shownBuf, kTolerance);
    EXPECT_GT(diffShown, 50u) << "showing again produced no visible change (only " << diffShown
                              << "/" << nBytes << " bytes changed beyond rendering noise)";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamPerf, RenderMsDeltaAndWorstFrameWithFixtureLoaded) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{149, 352, 90}, {69, 472, 0}, 60.0};
    std::vector<uint8_t> buf(320u * 240u * 3u);
    constexpr int kFrames = 60;

    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(r, kTilesFixtureDir.c_str(),
                                                                    kFixtureAnchor));
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);
    ASSERT_GT(pump_until_loaded(r, pose, buf), 0u);

    double worstMs = 0.0;
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < kFrames; ++i) {
        const auto f0 = std::chrono::steady_clock::now();
        overlume::render_frame(r, pose, {buf.data(), 320, 240});
        const auto f1 = std::chrono::steady_clock::now();
        worstMs = std::max(worstMs, std::chrono::duration<double, std::milli>(f1 - f0).count());
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double meanWithMs = std::chrono::duration<double, std::milli>(t1 - t0).count() / kFrames;
    overlume::destroy_renderer(r);

    auto* r2 = overlume::create_renderer(cfg);
    ASSERT_NE(r2, nullptr);
    overlume::set_scene(r2, overlume::SceneGraph{});
    overlume::render_frame(r2, pose, {buf.data(), 320, 240});
    const auto b0 = std::chrono::steady_clock::now();
    for (int i = 0; i < kFrames; ++i) overlume::render_frame(r2, pose, {buf.data(), 320, 240});
    const auto b1 = std::chrono::steady_clock::now();
    const double meanWithoutMs =
        std::chrono::duration<double, std::milli>(b1 - b0).count() / kFrames;
    overlume::destroy_renderer(r2);

    std::cerr << "[EnvironmentStreamPerf] mean render_ms without=" << meanWithoutMs
              << " with=" << meanWithMs << " delta=" << (meanWithMs - meanWithoutMs)
              << " worst_single_frame_ms=" << worstMs << "\n";
    SUCCEED();
}

TEST(TokenRedaction, NeverLeaksIntoLogText) {
    constexpr const char* kBogusToken = "bogus-token-for-redaction-test";
    ASSERT_TRUE(overlume::testing::drive_ion_token_redaction_probe(kBogusToken, 123456, 200))
        << "the probe never logged anything at all -- test infrastructure issue, not a pass";

    const std::string logText = overlume::testing::captured_cesium_log_text();
    ASSERT_NE(logText.find("access_token="), std::string::npos)
        << "probe never logged the ion endpoint URL -- redaction path was never exercised";
    const auto leakPos = logText.find(kBogusToken);
    EXPECT_EQ(leakPos, std::string::npos)
        << "the bogus token leaked verbatim into this library's own log output near: "
        << (leakPos == std::string::npos ? std::string() : logText.substr(leakPos, 48));

    size_t pos = 0;
    while ((pos = logText.find("access_token=", pos)) != std::string::npos) {
        const std::string rest = logText.substr(pos + std::strlen("access_token="));
        EXPECT_EQ(rest.rfind("<redacted>", 0), 0u)
            << "access_token= was followed by something other than <redacted> (" << rest.size()
            << " chars follow; content withheld -- it may be a credential)";
        pos += std::strlen("access_token=");
    }
}

namespace {

void pump_until_state(overlume::VisualRenderer* r, const overlume::CameraPose& pose,
                      std::vector<uint8_t>& buf, overlume::EnvironmentSourceState stop_state,
                      int max_ticks = 30) {
    for (int i = 0; i < max_ticks; ++i) {
        ASSERT_TRUE(overlume::render_frame(r, pose, {buf.data(), 320, 240}));
        if (overlume::environment_source_state(r) == stop_state) return;
    }
}

}

TEST(EnvironmentStream, NetworkDeadFromFirstRequestFallsBackToBakedChunksOnce) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    auto* killable = overlume::testing::install_fixture_streaming_source_with_fallback(
        r, kTilesFixtureFallbackDir.c_str(), kTestTownDir.c_str(), kFixtureAnchor);
    ASSERT_NE(killable, nullptr);

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);

    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::STREAMING);

    std::vector<uint8_t> buf(320u * 240u * 3u);
    overlume::testing::kill_fixture_network(killable);
    pump_until_state(r, kStdPose, buf, overlume::EnvironmentSourceState::STREAMING_FALLBACK);

    EXPECT_EQ(overlume::environment_source_state(r),
              overlume::EnvironmentSourceState::STREAMING_FALLBACK);
    EXPECT_GT(overlume::testing::environment_loaded_chunk_count(r), 0u);

    overlume::testing::revive_fixture_network(killable);
    for (int i = 0; i < 10; ++i) overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});
    EXPECT_EQ(overlume::environment_source_state(r),
              overlume::EnvironmentSourceState::STREAMING_FALLBACK)
        << "a revived network must not un-do the fallback (Decision 11: one-way)";
    EXPECT_GT(overlume::testing::environment_loaded_chunk_count(r), 0u)
        << "the baked chunks must stay resident after the network returns";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, NetworkLossWithNoFallbackDirStillTransitionsAndStaysEmpty) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    auto* killable = overlume::testing::install_fixture_streaming_source_with_fallback(
        r, kTilesFixtureFallbackDir.c_str(), nullptr, kFixtureAnchor);
    ASSERT_NE(killable, nullptr);

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);

    std::vector<uint8_t> buf(320u * 240u * 3u);
    overlume::testing::kill_fixture_network(killable);
    pump_until_state(r, kStdPose, buf, overlume::EnvironmentSourceState::STREAMING_FALLBACK);

    EXPECT_EQ(overlume::environment_source_state(r),
              overlume::EnvironmentSourceState::STREAMING_FALLBACK);
    EXPECT_EQ(overlume::testing::environment_loaded_chunk_count(r), 0u);
    EXPECT_TRUE(overlume::render_frame(r, kStdPose, {buf.data(), 320, 240}));
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, TransportErrorsCountAsNetworkLossAndFallBack) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    auto* handle = overlume::testing::install_fixture_streaming_source_with_fallback(
        r, kTilesFixtureFallbackDir.c_str(), kTestTownDir.c_str(), kFixtureAnchor);
    ASSERT_NE(handle, nullptr);

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);

    std::vector<uint8_t> buf(320u * 240u * 3u);
    overlume::testing::break_fixture_network_transport(handle);
    pump_until_state(r, kStdPose, buf, overlume::EnvironmentSourceState::STREAMING_FALLBACK);

    EXPECT_EQ(overlume::environment_source_state(r),
              overlume::EnvironmentSourceState::STREAMING_FALLBACK)
        << "rejected requests (curl's DNS failure, refusal or timeout) never reached the "
           "consecutive-failure counter, so a real loss of connectivity never falls back";
    EXPECT_GT(overlume::testing::environment_loaded_chunk_count(r), 0u)
        << "the baked fallback chunks must load";
    const std::string logText = overlume::testing::captured_cesium_log_text();
    EXPECT_NE(logText.find("fixture transport error: "), std::string::npos)
        << "the counter must rethrow the transport error with its original text";
    EXPECT_NE(logText.find("access_token=<redacted>"), std::string::npos)
        << "the transport error's text, URL and token included, must reach the redacting logger";
    EXPECT_EQ(logText.find("fixture-transport-token"), std::string::npos)
        << "a token in a transport error's text must never reach the log";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, MaterialsOriginalDefaultsFalse) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(r, kTilesFixtureDir.c_str(),
                                                                    kFixtureAnchor));
    EXPECT_FALSE(overlume::testing::environment_stream_materials_original(r));
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, MaterialsOriginalTrueIsMirroredByHook) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(r, kTilesFixtureDir.c_str(),
                                                                    kFixtureAnchor, true));
    EXPECT_TRUE(overlume::testing::environment_stream_materials_original(r));
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, ParseMaterialsOriginalRecognizesTheRealKeyAndValue) {
    EXPECT_TRUE(
        overlume::testing::environment_stream_parse_materials_original("96188?materials=original"));
    EXPECT_FALSE(overlume::testing::environment_stream_parse_materials_original("96188"));
    EXPECT_FALSE(
        overlume::testing::environment_stream_parse_materials_original("96188?materials=clay"));
    EXPECT_FALSE(
        overlume::testing::environment_stream_parse_materials_original("96188?materials=bogus"));
}

TEST(EnvironmentStream, UnknownMaterialsValueOnIonUriIsNonFatalFalse) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ScopedUnsetEnv no_token("CESIUM_ION_TOKEN");
    overlume::GeoAnchor a{25.0803, 55.3910, 0.0};
    EXPECT_FALSE(overlume::set_environment_source(r, "ion://96188?materials=bogus", a));
    std::vector<uint8_t> buf(320u * 240u * 3u);
    EXPECT_TRUE(overlume::render_frame(r, kStdPose, {buf.data(), 320, 240}));
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, ParseIonSpecFollowTerrain) {
    bool ok = false;
    EXPECT_TRUE(
        overlume::testing::environment_stream_parse_follow_terrain("96188?follow_terrain=on", &ok));
    EXPECT_TRUE(ok);
    EXPECT_FALSE(overlume::testing::environment_stream_parse_follow_terrain("96188", &ok));
    EXPECT_TRUE(ok);
    overlume::testing::environment_stream_parse_follow_terrain("96188?follow_terrain=maybe", &ok);
    EXPECT_FALSE(ok);
}

TEST(EnvironmentStream, ParseIonSpecMaxTiltDeg) {
    bool ok = false;
    EXPECT_DOUBLE_EQ(
        overlume::testing::environment_stream_parse_max_tilt_deg("96188?max_tilt_deg=5", &ok), 5.0);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(overlume::testing::environment_stream_parse_max_tilt_deg("96188", &ok), 2.0)
        << "absent key -- parses fine, defaults to 2.0 (kTerrainMaxTiltRad)";
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(
        overlume::testing::environment_stream_parse_max_tilt_deg("96188?max_tilt_deg=0", &ok), 0.0)
        << "0 is a valid value -- the offset-only escape hatch";
    EXPECT_TRUE(ok);
    overlume::testing::environment_stream_parse_max_tilt_deg("96188?max_tilt_deg=bogus", &ok);
    EXPECT_FALSE(ok) << "non-numeric -- the WHOLE spec fails to parse";
    overlume::testing::environment_stream_parse_max_tilt_deg("96188?max_tilt_deg=-1", &ok);
    EXPECT_FALSE(ok) << "negative -- a tilt clamp can't be negative, same 'fails the whole "
                        "parse' shape";
}

TEST(EnvironmentStream, ParseIonSpecBrightness) {
    bool ok = false;
    EXPECT_DOUBLE_EQ(
        overlume::testing::environment_stream_parse_brightness("96188?brightness=2.5", &ok), 2.5);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(overlume::testing::environment_stream_parse_brightness("96188", &ok), 1.0)
        << "absent key -- parses fine, defaults to 1.0 (no-op gain)";
    EXPECT_TRUE(ok);
    overlume::testing::environment_stream_parse_brightness("96188?brightness=0", &ok);
    EXPECT_FALSE(ok) << "0 -- a zero gain has no safe silent meaning, fails the whole parse";
    overlume::testing::environment_stream_parse_brightness("96188?brightness=-1", &ok);
    EXPECT_FALSE(ok) << "negative -- a gain can't be negative, same 'fails the whole parse' shape";
    overlume::testing::environment_stream_parse_brightness("96188?brightness=bogus", &ok);
    EXPECT_FALSE(ok) << "non-numeric -- the WHOLE spec fails to parse";
}

TEST(EnvironmentStream, ParseIonSpecRadius) {
    bool ok = false;
    EXPECT_DOUBLE_EQ(overlume::testing::environment_stream_parse_radius("96188", &ok), 700.0)
        << "absent key -- parses fine, probe reports the 700.0 default";
    EXPECT_TRUE(ok);
    EXPECT_FALSE(overlume::testing::environment_stream_parse_has_radius("96188"))
        << "absent key means defer to the theme, not an explicit 700";
    EXPECT_TRUE(overlume::testing::environment_stream_parse_has_radius("96188?radius=700"));
    EXPECT_DOUBLE_EQ(overlume::testing::environment_stream_parse_radius("96188?radius=250", &ok),
                     250.0);
    EXPECT_TRUE(ok);
    overlume::testing::environment_stream_parse_radius("96188?radius=0", &ok);
    EXPECT_FALSE(ok) << "0 -- an empty radius has no safe silent meaning, fails the whole parse";
    overlume::testing::environment_stream_parse_radius("96188?radius=-5", &ok);
    EXPECT_FALSE(ok)
        << "negative -- a radius can't be negative, same 'fails the whole parse' shape";
    overlume::testing::environment_stream_parse_radius("96188?radius=abc", &ok);
    EXPECT_FALSE(ok) << "non-numeric -- the WHOLE spec fails to parse";
}

TEST(EnvironmentStream, ParseIonSpecTerrainAndImageryAssets) {
    struct Case {
        const char* spec;
        int64_t terrain, imagery;
    };
    for (const Case& c :
         {Case{"96188", 0, 0}, Case{"96188?terrain=off&imagery=off", 0, 0},
          Case{"96188?terrain=1", 1, 0}, Case{"96188?terrain=1&imagery=2", 1, 2},
          Case{"96188?imagery=2&terrain=1", 1, 2}, Case{"96188?terrain=1&imagery=off", 1, 0},
          Case{"96188?terrain=1&terrain=off", 0, 0}, Case{"96188?terrain=off&terrain=3", 3, 0},
          Case{"96188?terrain=1&imagery=2&imagery=3", 1, 3},
          Case{"96188?follow_terrain=on", 0, 0}}) {
        int64_t terrain = -1;
        int64_t imagery = -1;
        bool ok = false;
        const bool on = overlume::testing::environment_stream_parse_terrain_assets(c.spec, &terrain,
                                                                                   &imagery, &ok);
        EXPECT_TRUE(ok) << c.spec;
        EXPECT_EQ(on, c.terrain != 0) << c.spec;
        EXPECT_EQ(terrain, c.terrain) << c.spec;
        EXPECT_EQ(imagery, c.imagery) << c.spec;
    }
    bool ok = false;
    EXPECT_DOUBLE_EQ(overlume::testing::environment_stream_parse_radius(
                         "96188?terrain=1&imagery=2&radius=250", &ok),
                     250.0)
        << "the other keys still parse beside terrain and imagery";
    EXPECT_TRUE(ok);
}

TEST(EnvironmentStream, ParseIonSpecImageryNeedsTerrain) {
    for (const char* spec : {"96188?imagery=2", "96188?imagery=2&terrain=off",
                             "96188?terrain=1&imagery=2&terrain=off"}) {
        int64_t terrain = -1;
        int64_t imagery = -1;
        bool ok = true;
        EXPECT_FALSE(overlume::testing::environment_stream_parse_terrain_assets(spec, &terrain,
                                                                                &imagery, &ok))
            << spec;
        EXPECT_FALSE(ok) << spec << ": imagery without terrain fails the whole parse";
        EXPECT_EQ(terrain, 0) << spec;
        EXPECT_EQ(imagery, 0) << spec;
    }
    bool ok = false;
    overlume::testing::environment_stream_parse_terrain_assets("96188?imagery=off", nullptr,
                                                               nullptr, &ok);
    EXPECT_TRUE(ok) << "imagery=off needs no terrain";
}

TEST(EnvironmentStream, ParseIonSpecBadAssetIdFailsTheWholeParse) {
    for (const char* value :
         {"", "0", "-1", "+1", " 1", "1.5", "1x", "abc", "on", "OFF", "99999999999999999999"}) {
        for (const std::string& spec : {std::string("96188?terrain=") + value,
                                        std::string("96188?terrain=1&imagery=") + value}) {
            bool ok = true;
            overlume::testing::environment_stream_parse_terrain_assets(spec.c_str(), nullptr,
                                                                       nullptr, &ok);
            EXPECT_FALSE(ok) << "'" << spec << "'";
        }
    }
}

TEST(EnvironmentStream, ParseIonSpecRefusalLogsOneLine) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ScopedUnsetEnv no_token("CESIUM_ION_TOKEN");  // a spec that parses stops at the token
    const overlume::GeoAnchor a{25.0803, 55.3910, 0.0};
    struct Case {
        const char* uri;
        const char* names;  // what the reason after "refused: " names: the bad key and value
    };
    for (const Case& c : {Case{"ion://96188?imagery=2", "imagery=2 needs terrain="},
                          Case{"ion://96188?terrain=1x", "terrain='1x'"},
                          Case{"ion://96188?terrain=1&imagery=0", "imagery='0'"},
                          Case{"ion://96188?radius=abc", "radius='abc'"}}) {
        const size_t from = overlume::testing::captured_cesium_log_text().size();
        EXPECT_FALSE(overlume::set_environment_source(r, c.uri, a)) << c.uri;
        const std::string text = overlume::testing::captured_cesium_log_text();
        const std::string logged = text.size() > from ? text.substr(from) : std::string();
        const std::string prefix = std::string(c.uri) + " refused: ";
        const size_t at = logged.find(prefix);
        EXPECT_NE(at, std::string::npos) << c.uri << ": '" << logged << "'";
        // Only the text after the prefix counts: the uri echoed before it holds every key already.
        const std::string reason =
            at == std::string::npos ? std::string() : logged.substr(at + prefix.size());
        EXPECT_NE(reason.find(c.names), std::string::npos)
            << c.uri << ": the reason must name '" << c.names << "', got '" << reason << "'";
        EXPECT_EQ(logged.find("refused"), logged.rfind("refused")) << "one line: " << logged;
    }
    const size_t from = overlume::testing::captured_cesium_log_text().size();
    EXPECT_FALSE(overlume::set_environment_source(r, "ion://96188?terrain=1&imagery=2", a));
    EXPECT_EQ(overlume::testing::captured_cesium_log_text().find("refused", from),
              std::string::npos)
        << "a spec that parses is not refused (with no token it opens nothing, silently)";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, TerrainPlaneFitProbe) {
    {
        const double s[5] = {-20.0, -10.0, 0.0, 10.0, 20.0};
        double h[5];
        for (int i = 0; i < 5; ++i) h[i] = 0.02 * s[i] + 1.5;
        double slope = 0.0, intercept = 0.0, rms = 0.0;
        overlume::testing::terrain_plane_fit_probe(s, h, 5, &slope, &intercept, &rms);
        EXPECT_NEAR(slope, 0.02, 1e-9);
        EXPECT_NEAR(intercept, 1.5, 1e-9);
        EXPECT_NEAR(rms, 0.0, 1e-9);
    }
    {
        const double s[5] = {-20.0, -10.0, 0.0, 10.0, 20.0};
        const double noise[5] = {0.1, -0.1, 0.1, -0.1, 0.1};
        double h[5];
        for (int i = 0; i < 5; ++i) h[i] = 0.02 * s[i] + 1.5 + noise[i];
        double slope = 0.0, intercept = 0.0, rms = 0.0;
        overlume::testing::terrain_plane_fit_probe(s, h, 5, &slope, &intercept, &rms);
        EXPECT_NEAR(slope, 0.02, 0.02);
        EXPECT_NEAR(intercept, 1.5, 0.1);
        EXPECT_GT(rms, 0.0);
    }
    {
        const double s2[2] = {-10.0, 10.0};
        const double h2[2] = {2.0, 4.0};
        double slope = 0.0, intercept = 0.0, rms = 0.0;
        overlume::testing::terrain_plane_fit_probe(s2, h2, 2, &slope, &intercept, &rms);
        EXPECT_DOUBLE_EQ(slope, 0.0);
        EXPECT_DOUBLE_EQ(intercept, 3.0);

        const double s1[1] = {0.0};
        const double h1[1] = {7.0};
        overlume::testing::terrain_plane_fit_probe(s1, h1, 1, &slope, &intercept, &rms);
        EXPECT_DOUBLE_EQ(slope, 0.0);
        EXPECT_DOUBLE_EQ(intercept, 7.0);
        EXPECT_DOUBLE_EQ(rms, 0.0);
    }
}

TEST(EnvironmentStream, TerrainTransformProbeCancelsGradeOnFittedLine) {
    constexpr double kSlope = 0.02;
    constexpr double kIntercept = 3.0;
    constexpr double kAnchorHeightM = 0.0;
    constexpr double kGroundBiasM = 0.3;
    constexpr double kMaxTiltRad = 2.0 * M_PI / 180.0;
    constexpr double kHeadingRad = 0.0;
    for (const double s : {-20.0, 0.0, 20.0}) {
        const double z = overlume::testing::terrain_transform_probe(
            kSlope, kIntercept, kAnchorHeightM, kGroundBiasM, kMaxTiltRad, kHeadingRad, 0.0, 0.0,
            s);
        EXPECT_NEAR(z, -kGroundBiasM, 0.02)
            << "s=" << s << ": the fitted tilt must cancel the grade, landing near -ground_bias";
    }
}

TEST(EnvironmentStream, TerrainTransformProbeClampsSteepGrade) {
    constexpr double kSlope = 0.20;
    constexpr double kIntercept = 3.0;
    constexpr double kAnchorHeightM = 0.0;
    constexpr double kGroundBiasM = 0.3;
    constexpr double kMaxTiltRad = 2.0 * M_PI / 180.0;
    constexpr double kHeadingRad = 0.0;
    constexpr double kS = 20.0;

    ASSERT_GT(std::atan(kSlope), kMaxTiltRad) << "test setup: this slope must actually clamp";
    const double clampedTheta = kMaxTiltRad;
    const double expectedResidual = kS * (kSlope * std::cos(clampedTheta) - std::sin(clampedTheta));
    const double z = overlume::testing::terrain_transform_probe(
        kSlope, kIntercept, kAnchorHeightM, kGroundBiasM, kMaxTiltRad, kHeadingRad, 0.0, 0.0, kS);
    EXPECT_NEAR(z, -kGroundBiasM * std::cos(clampedTheta) + expectedResidual, 1e-5);
    EXPECT_GT(std::abs(expectedResidual), 0.01);
}

TEST(EnvironmentStream, TerrainTransformProbeZeroMaxTiltMatchesOldOffsetOnlyBehaviour) {
    constexpr double kSlope = 0.02;
    constexpr double kIntercept = 3.0;
    constexpr double kAnchorHeightM = 0.0;
    constexpr double kGroundBiasM = 0.3;
    constexpr double kHeadingRad = 0.0;
    for (const double s : {-20.0, 0.0, 20.0}) {
        const double z = overlume::testing::terrain_transform_probe(
            kSlope, kIntercept, kAnchorHeightM, kGroundBiasM, 0.0, kHeadingRad, 0.0, 0.0, s);
        EXPECT_NEAR(z, -kGroundBiasM + kSlope * s, 1e-5)
            << "max_tilt_deg=0 must reproduce the OLD offset-only error exactly";
    }
    const double zAt20 = overlume::testing::terrain_transform_probe(
        kSlope, kIntercept, kAnchorHeightM, kGroundBiasM, 0.0, kHeadingRad, 0.0, 0.0, 20.0);
    EXPECT_NEAR(zAt20 - (-kGroundBiasM), 0.4, 1e-5);
}

TEST(EnvironmentStream, TerrainGroundOffsetSmoothingMath) {
    EXPECT_NEAR(overlume::testing::terrain_ground_offset_probe(3.0, 0.0, 0.0, 0.0, 0.0f, true),
                -3.0, 1e-9);
    EXPECT_NEAR(overlume::testing::terrain_ground_offset_probe(3.0, 0.0, 0.3, 0.0, 0.0f, true),
                -3.3, 1e-9);
    EXPECT_NEAR(overlume::testing::terrain_ground_offset_probe(500.0, 0.0, 0.0, 0.0, 0.0f, true),
                -30.0, 1e-9);
    const double stepped =
        overlume::testing::terrain_ground_offset_probe(3.0, 0.0, 0.0, 0.0, 0.5f, false);
    EXPECT_GT(stepped, -3.0);
    EXPECT_LT(stepped, 0.0);
}

TEST(EnvironmentStream, FollowTerrainShiftsEnvironmentToGroundUnderEgo) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(
        r, kSessionGroundFixtureDir.c_str(), kSessionGroundAnchor, false, true));

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);

    const overlume::CameraPose awayPose{{0, 0, 5000}, {0.001, 0, 5001}, 60};

    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GT(pump_until_loaded(r, awayPose, buf), 0u)
        << "session_ground fixture must load from its own anchor's block center";
    const double worldZBefore = overlume::testing::environment_terrain_first_tile_world_z(r);
    ASSERT_FALSE(std::isnan(worldZBefore))
        << "a tracked tile must expose a real Filament transform once loaded";

    constexpr double kHeightSamplePrecisionM = 0.1;
    double offset = overlume::testing::environment_terrain_offset_z(r);
    for (int i = 0; i < 600 && std::abs(offset - (-3.0)) > kHeightSamplePrecisionM; ++i) {
        overlume::render_frame(r, awayPose, {buf.data(), 320, 240});
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        offset = overlume::testing::environment_terrain_offset_z(r);
    }
    EXPECT_NEAR(offset, -3.0, kHeightSamplePrecisionM)
        << "groundOffsetZ_ must converge on -(3.0 - 0.0) m";

    const double worldZAfter = overlume::testing::environment_terrain_first_tile_world_z(r);
    ASSERT_FALSE(std::isnan(worldZAfter));
    EXPECT_NEAR(worldZAfter - worldZBefore, -3.0, kHeightSamplePrecisionM)
        << "the REAL Filament world transform of the tracked tile must have shifted by the "
           "same amount -- see this test's own top comment";

    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, FollowTerrainOffStaysAtZero) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(
        r, kSessionGroundFixtureDir.c_str(), kSessionGroundAnchor, false, false));
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);
    std::vector<uint8_t> buf(320u * 240u * 3u);
    for (int i = 0; i < 600; ++i) {
        overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_EQ(overlume::testing::environment_terrain_offset_z(r), 0.0);
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, GroundPlaneHiddenWhenTilesetSuppliesGround) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(
        r, kSessionGroundFixtureDir.c_str(), kSessionGroundAnchor, false, true));

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);

    const overlume::CameraPose awayPose{{0, 0, 5000}, {0.001, 0, 5001}, 60};
    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GT(pump_until_loaded(r, awayPose, buf), 0u)
        << "session_ground fixture must load from its own anchor's block center";

    bool hit = overlume::testing::environment_stream_provides_ground(r);
    for (int i = 0; i < 600 && !hit; ++i) {
        overlume::render_frame(r, awayPose, {buf.data(), 320, 240});
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        hit = overlume::testing::environment_stream_provides_ground(r);
    }
    ASSERT_TRUE(hit) << "sampleHeightMostDetailed() must hit the ground quad under the ego";
    EXPECT_FALSE(overlume::testing::renderer_ground_plane_in_scene(r))
        << "evidence of streamed ground must take the clay plane out of the scene";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, GroundPlaneKeptWhenTilesetIsBuildingsOnly) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(r, kTilesFixtureDir.c_str(),
                                                                    kFixtureAnchor, false, true));
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);
    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GT(pump_until_loaded(r, kStdPose, buf), 0u);
    for (int i = 0; i < 200; ++i) {
        overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_FALSE(overlume::testing::environment_stream_provides_ground(r));
    EXPECT_TRUE(overlume::testing::renderer_ground_plane_in_scene(r));
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, GroundPlaneReturnsAfterNetworkLossFallback) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* killable = overlume::testing::install_fixture_streaming_source_with_fallback(
        r, kSessionGroundFixtureDir.c_str(), kTestTownDir.c_str(), kSessionGroundAnchor, true);
    ASSERT_NE(killable, nullptr);

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);

    const overlume::CameraPose awayPose{{0, 0, 5000}, {0.001, 0, 5001}, 60};
    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GT(pump_until_loaded(r, awayPose, buf), 0u)
        << "session_ground fixture must load from its own anchor's block center";

    bool hit = overlume::testing::environment_stream_provides_ground(r);
    for (int i = 0; i < 600 && !hit; ++i) {
        overlume::render_frame(r, awayPose, {buf.data(), 320, 240});
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        hit = overlume::testing::environment_stream_provides_ground(r);
    }
    ASSERT_TRUE(hit) << "sampleHeightMostDetailed() must hit the ground quad under the ego";
    ASSERT_FALSE(overlume::testing::renderer_ground_plane_in_scene(r))
        << "clay plane must be hidden once the tileset proves it has ground";

    ASSERT_TRUE(overlume::testing::environment_stream_force_fall_back(r));
    overlume::render_frame(r, awayPose, {buf.data(), 320, 240});

    EXPECT_TRUE(overlume::testing::renderer_ground_plane_in_scene(r))
        << "the clay plane must return once the streamed source has fallen back -- the ground "
           "hit latch must not survive Decision 11's network-loss fallback";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, ParseIonSpecReplacesGround) {
    bool ok = false;
    EXPECT_FALSE(overlume::testing::environment_stream_parse_replaces_ground(
        "96188?replaces_ground=off", &ok));
    EXPECT_TRUE(ok);
    EXPECT_TRUE(overlume::testing::environment_stream_parse_replaces_ground("96188", &ok));
    EXPECT_TRUE(ok);
    overlume::testing::environment_stream_parse_replaces_ground("96188?replaces_ground=maybe", &ok);
    EXPECT_FALSE(ok);
}

TEST(EnvironmentStream, ClayModeRemapsFirstPrimitiveToBuildingMaterial) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(r, kTilesFixtureDir.c_str(),
                                                                    kFixtureAnchor));
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);
    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GT(pump_until_loaded(r, kStdPose, buf), 0u);
    EXPECT_TRUE(overlume::testing::environment_stream_first_primitive_is_clay(r));
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, OriginalModeSkipsClayRemapOnFirstPrimitive) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(r, kTilesFixtureDir.c_str(),
                                                                    kFixtureAnchor, true));
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);
    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GT(pump_until_loaded(r, kStdPose, buf), 0u);
    EXPECT_FALSE(overlume::testing::environment_stream_first_primitive_is_clay(r));
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamPerf, GooglePresetLiveRenderMsDeltaVsOsmClay) {
    if (std::getenv("CESIUM_ION_TOKEN") == nullptr ||
        std::getenv("OVERLUME_LIVE_ION_PERF") == nullptr) {
        GTEST_SKIP() << "opt-in live-network perf check -- set CESIUM_ION_TOKEN and "
                        "OVERLUME_LIVE_ION_PERF=1 to run (never required by ctest)";
    }
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -300, 300}, {0, 0, 0}, 60.0};
    std::vector<uint8_t> buf(320u * 240u * 3u);
    overlume::GeoAnchor anchor{25.0803, 55.3910, 0.0};

    auto pump_live = [&](const char* source_uri, std::vector<uint8_t>& out_buf) -> uint64_t {
        auto* r = overlume::create_renderer(cfg);
        if (!r) return 0;
        if (!overlume::set_environment_source(r, source_uri, anchor)) {
            overlume::destroy_renderer(r);
            return 0;
        }
        overlume::SceneGraph s{};
        s.ego.valid = 1;
        s.ego.position = overlume::Vec3{0, 0, 0};
        overlume::set_scene(r, s);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
        uint64_t loaded = 0;
        while (std::chrono::steady_clock::now() < deadline) {
            if (!overlume::render_frame(r, pose, {out_buf.data(), 320, 240})) break;
            loaded = overlume::testing::environment_loaded_chunk_count(r);
            if (loaded > 0) break;
        }
        if (loaded == 0) {
            overlume::destroy_renderer(r);
            return 0;
        }
        double worstMs = 0.0;
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < 60; ++i) {
            const auto f0 = std::chrono::steady_clock::now();
            overlume::render_frame(r, pose, {out_buf.data(), 320, 240});
            worstMs = std::max(worstMs, std::chrono::duration<double, std::milli>(
                                            std::chrono::steady_clock::now() - f0)
                                            .count());
        }
        const double meanMs =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0)
                .count() /
            60.0;
        std::cerr << "[EnvironmentStreamPerf/live] source='" << source_uri << "' loaded=" << loaded
                  << " mean_render_ms=" << meanMs << " worst_single_frame_ms=" << worstMs << "\n";
        overlume::testing::render_and_compare(r, pose, "/nonexistent_no_golden.png",
                                              OVERLUME_TMP_DIR
                                              "/environment_stream_live_actual.png");
        overlume::destroy_renderer(r);
        return loaded;
    };

    std::vector<uint8_t> clayBuf(320u * 240u * 3u);
    const uint64_t clayLoaded = pump_live("ion://96188", clayBuf);
    const uint64_t googleLoaded = pump_live("ion://2275207?materials=original&cache=off", buf);
    std::cerr << "[EnvironmentStreamPerf/live] clay(96188) loaded=" << clayLoaded
              << " google(2275207,original) loaded=" << googleLoaded << "\n";
    SUCCEED();
}

namespace {

constexpr overlume::Vec3 kCaptureBuildingsCentroid{-109.2, -17.1, 3.0};
const overlume::CameraPose kCapturePose{
    {kCaptureBuildingsCentroid.x + 50, kCaptureBuildingsCentroid.y - 70, 40},
    {kCaptureBuildingsCentroid.x, kCaptureBuildingsCentroid.y, kCaptureBuildingsCentroid.z},
    60.0};

constexpr uint32_t kCaptureWidth = 960;
constexpr uint32_t kCaptureHeight = 720;

std::string capture_out_path(const char* name) {
    const char* dir = std::getenv("OVERLUME_CAPTURE_OUT_DIR");
    return std::string(dir && *dir ? dir : OVERLUME_TMP_DIR) + "/env_source_" + name + ".png";
}

struct ContentStats {
    double luminance_stddev = 0.0;
    double non_background_fraction = 0.0;
};

ContentStats analyze_capture(const std::vector<uint8_t>& rgb, uint32_t width, uint32_t height) {
    auto luminance = [](uint8_t r, uint8_t g, uint8_t b) {
        return 0.2126 * r + 0.7152 * g + 0.0722 * b;
    };
    const size_t n = static_cast<size_t>(width) * height;
    double sum = 0.0, sumSq = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double l = luminance(rgb[i * 3], rgb[i * 3 + 1], rgb[i * 3 + 2]);
        sum += l;
        sumSq += l * l;
    }
    const double mean = sum / static_cast<double>(n);
    const double variance = sumSq / static_cast<double>(n) - mean * mean;

    const double bgL = luminance(rgb[0], rgb[1], rgb[2]);
    constexpr double kTolerance = 10.0;
    size_t differing = 0;
    for (size_t i = 0; i < n; ++i) {
        const double l = luminance(rgb[i * 3], rgb[i * 3 + 1], rgb[i * 3 + 2]);
        if (std::abs(l - bgL) > kTolerance) ++differing;
    }
    ContentStats stats;
    stats.luminance_stddev = std::sqrt(std::max(0.0, variance));
    stats.non_background_fraction = static_cast<double>(differing) / static_cast<double>(n);
    return stats;
}

uint64_t pump_and_settle(overlume::VisualRenderer* r, const overlume::CameraPose& pose,
                         std::vector<uint8_t>& buf, int initial_deadline_sec, int settle_seconds) {
    const auto phase1Deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(initial_deadline_sec);
    uint64_t loaded = 0;
    while (std::chrono::steady_clock::now() < phase1Deadline) {
        if (!overlume::render_frame(r, pose, {buf.data(), kCaptureWidth, kCaptureHeight}))
            return loaded;
        loaded = overlume::testing::environment_loaded_chunk_count(r);
        if (loaded > 0) break;
    }
    if (loaded == 0) return 0;
    const auto settleDeadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(settle_seconds);
    while (std::chrono::steady_clock::now() < settleDeadline) {
        overlume::render_frame(r, pose, {buf.data(), kCaptureWidth, kCaptureHeight});
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return overlume::testing::environment_loaded_chunk_count(r);
}

void capture_and_report(overlume::VisualRenderer* r, const char* name) {
    std::vector<uint8_t> buf(static_cast<size_t>(kCaptureWidth) * kCaptureHeight * 3);
    ASSERT_TRUE(
        overlume::render_frame(r, kCapturePose, {buf.data(), kCaptureWidth, kCaptureHeight}));
    const std::string outPath = capture_out_path(name);
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(outPath).parent_path(), ec);
    const int wrote = stbi_write_png(outPath.c_str(), static_cast<int>(kCaptureWidth),
                                     static_cast<int>(kCaptureHeight), 3, buf.data(),
                                     static_cast<int>(kCaptureWidth) * 3);
    ASSERT_NE(wrote, 0) << "failed to write capture PNG to " << outPath
                        << " (does OVERLUME_CAPTURE_OUT_DIR exist?)";
    const ContentStats stats = analyze_capture(buf, kCaptureWidth, kCaptureHeight);
    std::cerr << "[EnvSourceCapture] " << name << " -> " << outPath
              << " luminance_stddev=" << stats.luminance_stddev
              << " non_background_fraction=" << stats.non_background_fraction << "\n";
}

}

TEST(EnvSourceCapture, Baked) {
    if (std::getenv("OVERLUME_CAPTURE_ENV_SOURCES") == nullptr) {
        GTEST_SKIP()
            << "opt-in visual comparison capture -- set OVERLUME_CAPTURE_ENV_SOURCES=1 to run "
               "(never required by ctest)";
    }
    overlume::RenderConfig cfg{kCaptureWidth, kCaptureHeight, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::set_environment_source(r, kTestTownDir.c_str(), kFixtureAnchor));

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = overlume::Vec3{0, 0, 0};
    overlume::set_scene(r, s);
    std::vector<uint8_t> warm(static_cast<size_t>(kCaptureWidth) * kCaptureHeight * 3);
    ASSERT_GT(pump_and_settle(r, kCapturePose, warm, 5, 1), 0u)
        << "baked fixture chunk never loaded at ego {0,0,0}";
    capture_and_report(r, "baked");
    overlume::destroy_renderer(r);
}

TEST(EnvSourceCapture, Osm) {
    if (std::getenv("OVERLUME_CAPTURE_ENV_SOURCES") == nullptr ||
        std::getenv("CESIUM_ION_TOKEN") == nullptr) {
        GTEST_SKIP() << "opt-in LIVE-network visual comparison capture -- set "
                        "OVERLUME_CAPTURE_ENV_SOURCES=1 and CESIUM_ION_TOKEN to run "
                        "(never required by ctest)";
    }
    overlume::RenderConfig cfg{kCaptureWidth, kCaptureHeight, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::set_environment_source(r, "ion://96188", kFixtureAnchor));

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = overlume::Vec3{0, 0, 0};
    overlume::set_scene(r, s);
    std::vector<uint8_t> warm(static_cast<size_t>(kCaptureWidth) * kCaptureHeight * 3);
    const uint64_t loaded = pump_and_settle(r, kCapturePose, warm, 30, 60);
    EXPECT_GT(loaded, 0u)
        << "osm preset (ion://96188) loaded no tiles at kFixtureAnchor within 30s "
           "-- would ship an empty/near-empty capture, reporting rather than faking "
           "it";
    capture_and_report(r, "osm");
    overlume::destroy_renderer(r);
}

TEST(EnvSourceCapture, Google) {
    if (std::getenv("OVERLUME_CAPTURE_ENV_SOURCES") == nullptr ||
        std::getenv("CESIUM_ION_TOKEN") == nullptr) {
        GTEST_SKIP() << "opt-in LIVE-network visual comparison capture -- set "
                        "OVERLUME_CAPTURE_ENV_SOURCES=1 and CESIUM_ION_TOKEN to run "
                        "(never required by ctest)";
    }
    overlume::RenderConfig cfg{kCaptureWidth, kCaptureHeight, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::set_environment_source(r, "ion://2275207?materials=original&cache=off",
                                                 kFixtureAnchor));

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = overlume::Vec3{0, 0, 0};
    overlume::set_scene(r, s);
    std::vector<uint8_t> warm(static_cast<size_t>(kCaptureWidth) * kCaptureHeight * 3);
    const uint64_t loaded = pump_and_settle(r, kCapturePose, warm, 30, 60);
    EXPECT_GT(loaded, 0u)
        << "google preset (ion://2275207) loaded no tiles at kFixtureAnchor within "
           "30s -- would ship an empty/near-empty capture, reporting rather than "
           "faking it";
    capture_and_report(r, "google");
    overlume::destroy_renderer(r);
}

TEST(EnvSourceCapture, Clipped) {
    if (std::getenv("OVERLUME_CAPTURE_ENV_SOURCES") == nullptr) {
        GTEST_SKIP()
            << "opt-in visual comparison capture -- set OVERLUME_CAPTURE_ENV_SOURCES=1 to run "
               "(never required by ctest)";
    }
    GTEST_SKIP()
        << "'clipped' preset resolves to the node's environment_own_asset_uri, which is NOT "
           "configured on this box (empty default) -- no real ion asset id exists to render, "
           "so this case is reported as not-renderable rather than faked with a made-up id";
}

namespace {

using GridCells = std::vector<std::pair<int, int>>;

// The grid fixture in metres east and north of the anchor: cell (c, r) spans east
// [(c - 2) km, (c - 1) km] and north [(1 - r) km, (2 - r) km]. Returns the cells whose rectangle
// lies within radius_m of (east_m, north_m), in (c, r) order.
GridCells grid_cells_within(double east_m, double north_m, double radius_m) {
    GridCells cells;
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            const double west = (c - 2) * 1000.0, south = (1 - r) * 1000.0;
            const double dx = std::max({west - east_m, 0.0, east_m - (west + 1000.0)});
            const double dy = std::max({south - north_m, 0.0, north_m - (south + 1000.0)});
            if (std::hypot(dx, dy) < radius_m) cells.emplace_back(c, r);
        }
    }
    return cells;
}

GridCells sorted(GridCells cells) {
    std::sort(cells.begin(), cells.end());
    return cells;
}

// kFixtureAnchor has heading 0: map +x is north and +y is west. Map metres differ from ground
// metres by under 0.4% here, and every test point keeps 100 m or more from a cell decision.
void set_ego_east_north(overlume::VisualRenderer* r, double east_m, double north_m) {
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = overlume::Vec3{north_m, -east_m, 0.0};
    overlume::set_scene(r, s);
}

}

TEST(EnvironmentStream, AreaSelectionLoadsOnlyTheCellsInsideTheRadius) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* grid = overlume::testing::install_fixture_grid_source(r, kFixtureAnchor, 500.0, true);
    ASSERT_NE(grid, nullptr);
    set_ego_east_north(r, 0.0, 0.0);

    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GE(pump_until_loaded(r, kStdPose, buf, 300, 4), 4u)
        << "the four cells around the ego never loaded";
    for (int i = 0; i < 30; ++i) overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});

    const GridCells central = {{1, 1}, {1, 2}, {2, 1}, {2, 2}};
    EXPECT_EQ(sorted(overlume::testing::fixture_grid_requested_leaves(grid)), central)
        << "a 500 m sphere at the shared corner touches only the four central cells; any other "
           "request came from a screen-space view or a preloaded sibling";
    EXPECT_EQ(overlume::testing::environment_loaded_chunk_count(r), 4u);
    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::STREAMING);
    EXPECT_EQ(overlume::testing::environment_stream_last_view_frustum_count(r), 1)
        << "the area view must be the only view";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, AreaSelectionRequestsEveryCellTheSphereTouches) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* grid = overlume::testing::install_fixture_grid_source(r, kFixtureAnchor, 1200.0, true);
    ASSERT_NE(grid, nullptr);
    set_ego_east_north(r, 0.0, 0.0);
    const GridCells expected = grid_cells_within(0.0, 0.0, 1200.0);
    ASSERT_EQ(expected.size(), 12u)
        << "test setup: 1200 m reaches the edge cells (1000 m away), not the corners (1414 m)";

    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GE(pump_until_loaded(r, kStdPose, buf, 300, 12), 12u);
    for (int i = 0; i < 30; ++i) overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});

    EXPECT_EQ(sorted(overlume::testing::fixture_grid_requested_leaves(grid)), expected);
    EXPECT_EQ(overlume::testing::environment_loaded_chunk_count(r), 12u);
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, AreaSelectionFollowsTheEgo) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* grid = overlume::testing::install_fixture_grid_source(r, kFixtureAnchor, 500.0, true);
    ASSERT_NE(grid, nullptr);
    set_ego_east_north(r, 0.0, 0.0);
    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GE(pump_until_loaded(r, kStdPose, buf, 300, 4), 4u);
    const size_t before = overlume::testing::fixture_grid_requested_leaves(grid).size();

    set_ego_east_north(r, 1600.0, 0.0);
    const GridCells expected = grid_cells_within(1600.0, 0.0, 500.0);
    ASSERT_EQ(expected, (GridCells{{3, 1}, {3, 2}})) << "test setup";
    uint64_t loaded = 0;
    for (int i = 0; i < 300 && loaded != 2; ++i) {
        overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});
        loaded = overlume::testing::environment_loaded_chunk_count(r);
    }
    for (int i = 0; i < 30; ++i) overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});

    const GridCells all = overlume::testing::fixture_grid_requested_leaves(grid);
    ASSERT_GE(all.size(), before);
    EXPECT_EQ(sorted(GridCells(all.begin() + before, all.end())), expected)
        << "1600 m east of the anchor the sphere touches only the two eastern cells either side "
           "of the ego's row boundary";
    EXPECT_EQ(overlume::testing::environment_loaded_chunk_count(r), 2u)
        << "the central cells must leave the scene once the sphere leaves them";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, WithoutAreaSelectionACustomTilesetKeepsTheScreenSpaceViews) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* grid = overlume::testing::install_fixture_grid_source(r, kFixtureAnchor, 500.0, false);
    ASSERT_NE(grid, nullptr);
    set_ego_east_north(r, 0.0, 0.0);
    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GE(pump_until_loaded(r, kStdPose, buf, 300, 4), 4u);
    for (int i = 0; i < 30; ++i) overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});

    EXPECT_GT(overlume::testing::fixture_grid_requested_leaves(grid).size(), 4u)
        << "without area selection cesium keeps today's two views and preloads culled siblings";
    EXPECT_EQ(overlume::testing::environment_stream_last_view_frustum_count(r), 2);
    overlume::destroy_renderer(r);
}

namespace {

// The golden pose of EnvironmentStreamGolden.FixtureBlock_DarkAdas: the fixture block covers about
// 2,600 of the frame's 76,800 pixels (counted in environment_stream_dark_adas.png).
const overlume::CameraPose kBlockPose{{149, 352, 90}, {69, 472, 0}, 60.0};

// Two copies of light_clay (thin fog at the pose's 150 m, so hues reach the frame) that differ
// only in environment.terrain_color: terrain_green and terrain_red. Returns their directory.
std::string write_terrain_themes() {
    const std::string dir = std::string(OVERLUME_TMP_DIR) + "/overlume_terrain_themes";
    std::ifstream in(std::string(kThemeDir) + "/light_clay.yaml");
    std::stringstream text;
    text << in.rdbuf();
    const std::string yaml = text.str();
    const size_t at = yaml.find("environment:");
    if (at == std::string::npos) {
        // The renderer then fails to load the theme; the test already carries this failure.
        ADD_FAILURE() << "light_clay.yaml has no environment: line";
        return dir;
    }
    std::filesystem::create_directories(dir);
    for (const auto& [name, rgb] :
         {std::pair<std::string, std::string>{"terrain_green", "[0.05, 0.6, 0.05]"},
          std::pair<std::string, std::string>{"terrain_red", "[0.6, 0.05, 0.05]"}}) {
        std::string variant = yaml;
        variant.replace(at, yaml.find('\n', at) - at,
                        "environment: { tile_radius_m: 100.0, terrain_color: " + rgb + " }");
        std::ofstream(dir + "/" + name + ".yaml") << variant;
    }
    return dir;
}

struct Rgb {
    uint8_t r, g, b;
};

// 256-px imagery tiles at <dir>/{z}/{x}/{y}.png for every web-mercator tile of levels
// 0..max_level within one tile of the fixture tileset's region (lon 55.3709..55.4485,
// lat 25.0499..25.1368), rows counted from the south as cesium fills {y}. Each pixel row takes
// colour(latitude of the row's centre, degrees); row 0 is the tile's north edge. With decodable
// false each file holds bytes no image decoder reads: every request answers 200, no raster lands.
template <typename Colour>
void write_fixture_imagery(const std::string& dir, bool decodable, int max_level, Colour colour) {
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
    std::vector<uint8_t> rgb(256u * 256u * 3u);
    for (int z = 0; z <= max_level; ++z) {
        const int n = 1 << z;
        const auto column = [n](double lon) { return static_cast<int>((lon + 180.0) / 360.0 * n); };
        const auto row = [n](double lat) {
            const double fromNorth =
                (1.0 - std::asinh(std::tan(lat * M_PI / 180.0)) / M_PI) / 2.0 * n;
            return n - 1 - static_cast<int>(fromNorth);
        };
        for (int x = std::max(0, column(55.3709) - 1); x <= std::min(n - 1, column(55.4485) + 1);
             ++x) {
            const std::string folder = dir + "/" + std::to_string(z) + "/" + std::to_string(x);
            std::filesystem::create_directories(folder);
            for (int y = std::max(0, row(25.0499) - 1); y <= std::min(n - 1, row(25.1368) + 1);
                 ++y) {
                const std::string path = folder + "/" + std::to_string(y) + ".png";
                if (!decodable) {
                    std::ofstream(path, std::ios::binary) << "not an image";
                    continue;
                }
                for (size_t j = 0; j < 256; ++j) {
                    // Row j's centre as a fraction of the mercator square from its north edge.
                    const double t = (n - 1 - y + (static_cast<double>(j) + 0.5) / 256.0) / n;
                    const Rgb c =
                        colour(std::atan(std::sinh(M_PI * (1.0 - 2.0 * t))) * 180.0 / M_PI);
                    for (size_t i = 0; i < 256; ++i) {
                        uint8_t* p = &rgb[(j * 256u + i) * 3u];
                        p[0] = c.r;
                        p[1] = c.g;
                        p[2] = c.b;
                    }
                }
                ASSERT_NE(stbi_write_png(path.c_str(), 256, 256, 3, rgb.data(), 256 * 3), 0)
                    << path;
            }
        }
    }
}

// One colour everywhere, levels 0..10 (the hook's default cap).
void write_fixture_imagery(const std::string& dir, uint8_t red, uint8_t green, uint8_t blue,
                           bool decodable) {
    write_fixture_imagery(dir, decodable, 10, [&](double) { return Rgb{red, green, blue}; });
}

// Pixels whose red (green) channel leads both others by more than 40, and the frame's red sum.
struct Hues {
    size_t red = 0;
    size_t green = 0;
    uint64_t red_sum = 0;
};

Hues count_hues(const std::vector<uint8_t>& rgb) {
    Hues h;
    for (size_t i = 0; i + 2 < rgb.size(); i += 3) {
        const int r = rgb[i], g = rgb[i + 1], b = rgb[i + 2];
        if (r > g + 40 && r > b + 40) ++h.red;
        if (g > r + 40 && g > b + 40) ++h.green;
        h.red_sum += static_cast<uint64_t>(r);
    }
    return h;
}

// The JSON chunk of a GLB: a 12-byte header, the chunk's length and type, then the text.
std::string glb_json(const std::vector<uint8_t>& glb) {
    if (glb.size() < 20) return {};
    uint32_t length = 0;
    std::memcpy(&length, glb.data() + 12, sizeof(length));
    if (20 + static_cast<size_t>(length) > glb.size()) return {};
    return std::string(reinterpret_cast<const char*>(glb.data() + 20), length);
}

// Which channel leads the other two by more than 40 (on average) in the 3x3 pixels around where
// the map point `p` lands in `frame`, a 320x240 render through the renderer's current camera:
// 'r', 'g' or 'b'; '-' for none, and off-screen. The window is golden_luma's
// (test_mapbox_buildings.cpp), copied because the two tests build separate binaries.
char hue_at(overlume::VisualRenderer* r, const std::vector<uint8_t>& frame,
            const overlume::Vec3& p) {
    float u = 0.0f, v = 0.0f;
    if (!overlume::project_to_screen(r, p, &u, &v)) return '-';
    const int cx = std::clamp(static_cast<int>(u * 320.0f), 1, 318);
    const int cy = std::clamp(static_cast<int>(v * 240.0f), 1, 238);
    int sum[3] = {0, 0, 0};
    for (int y = cy - 1; y <= cy + 1; ++y) {
        for (int x = cx - 1; x <= cx + 1; ++x) {
            const size_t i = (static_cast<size_t>(y) * 320u + static_cast<size_t>(x)) * 3u;
            for (int c = 0; c < 3; ++c) sum[c] += frame[i + static_cast<size_t>(c)];
        }
    }
    for (int c = 0; c < 3; ++c) {
        if (sum[c] > sum[(c + 1) % 3] + 9 * 40 && sum[c] > sum[(c + 2) % 3] + 9 * 40) {
            return "rgb"[c];
        }
    }
    return '-';
}

// The fixture tileset without RTC_CENTER, copied to the tmp dir. make_tile_fixture.py writes
// absolute ECEF positions and an RTC_CENTER the stream ignores, but cesium folds RTC_CENTER into
// the positions it generates overlay UVs from (RasterOverlayUtilities.cpp:98): every vertex then
// sits at about twice its ECEF vector, 0.07 degrees south of tile_root, and v clamps to 0. The
// feature table JSON keeps its length (padded with spaces), so the b3dm header stays valid.
std::string tiles_fixture_without_rtc() {
    const std::string dir = std::string(OVERLUME_TMP_DIR) + "/overlume_tiles_fixture_no_rtc";
    std::filesystem::create_directories(dir);
    std::filesystem::copy_file(kTilesFixtureDir + "/tileset.json", dir + "/tileset.json",
                               std::filesystem::copy_options::overwrite_existing);
    for (const char* name : {"tile_root.b3dm", "tile_a.b3dm", "tile_b.b3dm"}) {
        std::ifstream in(kTilesFixtureDir + "/" + name, std::ios::binary);
        std::string b3dm((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        uint32_t jsonLength = 0;
        if (b3dm.size() >= 28) std::memcpy(&jsonLength, b3dm.data() + 12, sizeof(jsonLength));
        if (jsonLength < 18 || b3dm.size() < 28 + static_cast<size_t>(jsonLength)) {
            ADD_FAILURE() << name << " is not a b3dm with a feature table";
            continue;
        }
        std::string json = "{\"BATCH_LENGTH\":1}";
        json.resize(jsonLength, ' ');
        b3dm.replace(28, jsonLength, json);
        std::ofstream(dir + "/" + name, std::ios::binary) << b3dm;
    }
    return dir;
}

// A renderer on `theme` with the fixture tileset in `tiles_dir` open as terrain under the imagery
// in `imagery_dir` and the ego at the fixture block; nullptr without a GPU.
overlume::VisualRenderer* terrain_renderer(const std::string& theme_dir, const char* theme,
                                           const std::string& imagery_dir, double brightness = 1.0,
                                           uint32_t maximum_level = 10,
                                           const std::string& tiles_dir = kTilesFixtureDir) {
    overlume::RenderConfig cfg{320, 240, 1, theme_dir.c_str(), theme};
    auto* r = overlume::create_renderer(cfg);
    if (r == nullptr) return nullptr;
    EXPECT_TRUE(overlume::testing::install_fixture_terrain_with_file_imagery(
        r, tiles_dir.c_str(), imagery_dir.c_str(), brightness, maximum_level));
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);
    return r;
}

// Renders kBlockPose until `done()` or 600 frames, then 30 more so the frame settles.
template <typename Done>
void render_block_until(overlume::VisualRenderer* r, std::vector<uint8_t>& frame, Done done) {
    for (int i = 0; i < 600 && !done(); ++i) {
        overlume::render_frame(r, kBlockPose, {frame.data(), 320, 240});
    }
    for (int i = 0; i < 30; ++i) overlume::render_frame(r, kBlockPose, {frame.data(), 320, 240});
}

bool tile_in_scene(overlume::VisualRenderer* r) {
    return overlume::testing::environment_loaded_chunk_count(r) >= 1;
}

bool raster_attached(overlume::VisualRenderer* r) {
    return overlume::testing::environment_stream_overlay_probe(r).rasters_attached >= 1;
}

}

TEST(EnvironmentStreamOverlay, TerrainColorShowsWhileNoRasterHasAttached) {
    const std::string imagery = std::string(OVERLUME_TMP_DIR) + "/overlume_imagery_undecodable";
    write_fixture_imagery(imagery, 0, 0, 0, false);
    auto* r = terrain_renderer(write_terrain_themes(), "terrain_green", imagery);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    std::vector<uint8_t> frame(320u * 240u * 3u);
    render_block_until(r, frame, [&] { return tile_in_scene(r); });
    ASSERT_TRUE(tile_in_scene(r)) << "the fixture tile never reached the scene";

    const overlume::testing::OverlayProbe probe =
        overlume::testing::environment_stream_overlay_probe(r);
    EXPECT_EQ(probe.rasters_attached, 0u) << "every imagery tile is undecodable";
    EXPECT_EQ(probe.textures_alive, 0u);
    const Hues hues = count_hues(frame);
    EXPECT_GT(hues.green, 800u) << "the terrain tile must show the theme's terrain_color (green) "
                                   "while no raster is attached";
    EXPECT_LT(hues.red, 20u);
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamOverlay, RasterAttachesAndItsColourCoversTheTile) {
    const std::string imagery = std::string(OVERLUME_TMP_DIR) + "/overlume_imagery_red";
    write_fixture_imagery(imagery, 255, 0, 0, true);
    overlume::testing::capture_stream_glbs(true);
    auto* r = terrain_renderer(write_terrain_themes(), "terrain_green", imagery);
    if (!r) {
        overlume::testing::capture_stream_glbs(false);
        GTEST_SKIP() << "no GPU/EGL";
    }
    std::vector<uint8_t> frame(320u * 240u * 3u);
    render_block_until(r, frame, [&] { return raster_attached(r); });
    const std::vector<std::vector<uint8_t>> glbs = overlume::testing::captured_stream_glbs();
    overlume::testing::capture_stream_glbs(false);

    const overlume::testing::OverlayProbe probe =
        overlume::testing::environment_stream_overlay_probe(r);
    ASSERT_GE(probe.rasters_attached, 1u) << "no raster attached within 600 frames";
    EXPECT_GE(probe.textures_alive, 1u);
    const auto glbs_with = [&](const char* text) {
        return std::count_if(glbs.begin(), glbs.end(), [&](const std::vector<uint8_t>& glb) {
            return glb_json(glb).find(text) != std::string::npos;
        });
    };
    EXPECT_GE(glbs_with("\"TEXCOORD_0\""), 1)
        << "the GLB step must copy _CESIUMOVERLAY_0 into TEXCOORD_0 for terrain tiles";
    EXPECT_EQ(glbs_with("_CESIUMOVERLAY_"), 0) << "gltfio refuses attributes outside the core set";
    const Hues hues = count_hues(frame);
    EXPECT_GT(hues.red, 800u) << "the fixture tile must show the imagery's red";
    EXPECT_LT(hues.green, 20u) << "terrain_color must be gone once the raster attaches";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamOverlay, TeardownDetachesEveryRasterAndFreesEveryTexture) {
    const std::string imagery = std::string(OVERLUME_TMP_DIR) + "/overlume_imagery_red";
    write_fixture_imagery(imagery, 255, 0, 0, true);
    auto* r = terrain_renderer(write_terrain_themes(), "terrain_green", imagery);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    std::vector<uint8_t> frame(320u * 240u * 3u);
    render_block_until(r, frame, [&] { return raster_attached(r); });
    ASSERT_TRUE(raster_attached(r)) << "no raster attached within 600 frames";

    // Tears the tileset down and keeps the source, so its counters stay readable.
    ASSERT_TRUE(overlume::testing::environment_stream_force_fall_back(r));
    const overlume::testing::OverlayProbe probe =
        overlume::testing::environment_stream_overlay_probe(r);
    EXPECT_EQ(probe.rasters_detached, probe.rasters_attached)
        << "every attached raster must be detached before its tile is freed";
    EXPECT_EQ(probe.textures_alive, 0u) << "a raster texture outlived its tileset";
    EXPECT_GE(probe.textures_freed, 1u);
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamOverlay, RemovingTheOverlayDetachesLiveTilesBackToTerrainColor) {
    const std::string imagery = std::string(OVERLUME_TMP_DIR) + "/overlume_imagery_red";
    write_fixture_imagery(imagery, 255, 0, 0, true);
    auto* r = terrain_renderer(write_terrain_themes(), "terrain_green", imagery);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    std::vector<uint8_t> frame(320u * 240u * 3u);
    render_block_until(r, frame, [&] { return raster_attached(r); });
    ASSERT_TRUE(raster_attached(r)) << "no raster attached within 600 frames";
    ASSERT_GT(count_hues(frame).red, 800u) << "test setup: the red imagery on the tile";

    ASSERT_TRUE(overlume::testing::environment_stream_remove_overlays(r));
    render_block_until(r, frame, [&] {
        return overlume::testing::environment_stream_overlay_probe(r).textures_alive == 0;
    });
    ASSERT_TRUE(tile_in_scene(r)) << "the terrain tile must survive the overlay's removal";
    const overlume::testing::OverlayProbe probe =
        overlume::testing::environment_stream_overlay_probe(r);
    EXPECT_EQ(probe.rasters_detached, probe.rasters_attached)
        << "every attached raster of a live tile must be detached when its overlay is removed";
    EXPECT_EQ(probe.textures_alive, 0u) << "a detached raster's texture must be freed by the drain";
    const Hues hues = count_hues(frame);
    EXPECT_GT(hues.green, 800u)
        << "a live tile whose raster detached must show terrain_color again (hasImagery 0)";
    EXPECT_LT(hues.red, 20u);
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamOverlay, BrightnessScalesTheImagery) {
    const std::string imagery = std::string(OVERLUME_TMP_DIR) + "/overlume_imagery_dim_red";
    write_fixture_imagery(imagery, 100, 0, 0, true);
    const std::string themes = write_terrain_themes();
    const auto red_sum_at = [&](double brightness) -> std::optional<uint64_t> {
        auto* r = terrain_renderer(themes, "terrain_green", imagery, brightness);
        if (!r) return std::nullopt;
        std::vector<uint8_t> frame(320u * 240u * 3u);
        render_block_until(r, frame, [&] { return raster_attached(r); });
        EXPECT_TRUE(raster_attached(r)) << "brightness " << brightness;
        const uint64_t sum = count_hues(frame).red_sum;
        overlume::destroy_renderer(r);
        return sum;
    };
    const std::optional<uint64_t> dim = red_sum_at(1.0);
    if (!dim) GTEST_SKIP() << "no GPU/EGL";
    const std::optional<uint64_t> bright = red_sum_at(3.0);
    ASSERT_TRUE(bright.has_value());
    EXPECT_GT(*bright, *dim + 40000u)
        << "brightness 3 must lift the imagery's red on the tile's ~2,600 pixels (frame red sum "
        << *dim << " at 1, " << *bright << " at 3)";
    // The imagery texture is sRGB (SRGB8_A8): red 100 is the albedo 0.127. As RGBA8 it would be
    // 0.39, three times brighter, which lifts the tile's ~2,600 pixels by about 150,000 in the
    // frame's red sum. Measured at 1x: 11,151,722 with SRGB8_A8 (llvmpipe: 11,158,234), 11,302,078
    // with RGBA8. The ceiling is their midpoint, 11,230,000: 78,000 above the sRGB sum, 72,000
    // below the RGBA8 one, and ten times the 6,500 the two GPUs differ by.
    EXPECT_LT(*dim, 11230000u) << "the imagery must be sampled as sRGB (SRGB8_A8): RGBA8 reads "
                                  "three times brighter (frame red sum "
                               << *dim << " at 1)";
}

TEST(EnvironmentStreamOverlay, TerrainColorFollowsAThemeSwitchOnLiveTiles) {
    const std::string imagery = std::string(OVERLUME_TMP_DIR) + "/overlume_imagery_undecodable";
    write_fixture_imagery(imagery, 0, 0, 0, false);
    auto* r = terrain_renderer(write_terrain_themes(), "terrain_green", imagery);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    std::vector<uint8_t> frame(320u * 240u * 3u);
    render_block_until(r, frame, [&] { return tile_in_scene(r); });
    ASSERT_GT(count_hues(frame).green, 800u) << "test setup: the green terrain tile";

    // A transition that ended 9.9 s before the scene's sim time 0: the next frame is terrain_red.
    ASSERT_TRUE(overlume::set_theme(r, "terrain_red", -10.0, 0.1));
    for (int i = 0; i < 3; ++i) overlume::render_frame(r, kBlockPose, {frame.data(), 320, 240});
    const Hues hues = count_hues(frame);
    EXPECT_GT(hues.red, 800u) << "a live terrain tile must take the new theme's terrain_color";
    EXPECT_LT(hues.green, 20u);
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamOverlay, ImageryKeepsNorthUp) {
    // tile_root's region centre, where make_tile_fixture.py stands its five buildings (at the
    // region's mid height); the imagery is red north of this latitude and blue south of it.
    constexpr double kCentreLatDeg = 25.080924750000005;
    constexpr double kCentreLonDeg = 55.386325500000005;
    constexpr double kBaseHeightM = -2.7436;
    double x0 = 0, y0 = 0, z0 = 0;
    ASSERT_TRUE(overlume::testing::ecef_to_map_probe(
        kFixtureAnchor.origin_lat_deg, kFixtureAnchor.origin_lon_deg, 0.0, 0.0, kCentreLatDeg,
        kCentreLonDeg, kBaseHeightM, &x0, &y0, &z0));
    // Map x is north and y west. Each point is a building's centre at half its height, on that
    // building's own faces from any side. North: E 30.4, N 29.5, 11.8 x 16.0 m, 26.1 m tall.
    // South: E -13.1, N -25.4, 14.4 x 8.2 m, 11.2 m tall.
    const overlume::Vec3 northBuilding{x0 + 29.5, y0 - 30.4, z0 + 13.0};
    const overlume::Vec3 southBuilding{x0 - 25.4, y0 + 13.1, z0 + 5.6};

    // Level 14: 8.7 m texels, so both buildings lie more than two texels from the colour line.
    const std::string imagery = std::string(OVERLUME_TMP_DIR) + "/overlume_imagery_north_red";
    write_fixture_imagery(imagery, true, 14, [](double lat_deg) {
        return lat_deg > kCentreLatDeg ? Rgb{255, 0, 0} : Rgb{0, 0, 255};
    });
    // On the shipped fixture, cesium folds RTC_CENTER into the overlay UVs and both probes read
    // the raster's south edge (API notes); this copy drops it.
    auto* r = terrain_renderer(write_terrain_themes(), "terrain_green", imagery, 1.0, 14,
                               tiles_fixture_without_rtc());
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    std::vector<uint8_t> frame(320u * 240u * 3u);
    render_block_until(r, frame, [&] { return raster_attached(r); });
    ASSERT_TRUE(raster_attached(r)) << "no raster attached within 600 frames";

    EXPECT_EQ(hue_at(r, frame, northBuilding), 'r')
        << "the building north of the colour line must show the imagery's north (red); 'b' means "
           "the raster is mirrored north-south, 'g' that terrain_color still shows";
    EXPECT_EQ(hue_at(r, frame, southBuilding), 'b')
        << "the building south of the colour line must show the imagery's south (blue); 'r' means "
           "the raster is mirrored or sampled at one corner (translation and scale swapped)";
    overlume::destroy_renderer(r);
}

// Task 11: an ion:// source with a second tileset (layer.json terrain with an ion imagery overlay)
// over a fixture network. Asset 96188 is a 3D Tiles endpoint whose tileset is a file fixture; asset
// 1 is terrain, the two level-0 tiles of the geographic scheme flat at 0 m (Task 2's encoder);
// asset 2 is TMS imagery whose every tile is one flat 512-px PNG. Each test uses its own token:
// cesium-native caches ion endpoints per URL, token included, for the life of the process.
namespace ion_terrain_fixture {

namespace qmesh = overlume::testing::qmesh;
using overlume::testing::IonFixtureNetwork;

const std::string kContentBase = "https://assets.ion.fixture/96188/";
const std::string kTerrainBase = "https://assets.ion.fixture/1/";
const std::string kImageryBase = "https://assets.ion.fixture/2/";
const std::string kAttributedImageryEndpoint =
    R"({"type":"IMAGERY","url":")" + kImageryBase +
    R"(","accessToken":"fixture-asset-token","attributions":[{"html":"<span><a href=\"https://fixture.example/terms\">Fixture Imagery</a> &amp; Co.</span>","collapsible":false}]})";
constexpr char kTerrainDidNotOpen401[] =
    "environment source: terrain tileset did not open (HTTP 401) -- continuing without terrain";
constexpr size_t kFrameBytes = 320u * 240u * 3u;

std::string endpoint_url(int64_t asset_id) {
    return "https://api.cesium.com/v1/assets/" + std::to_string(asset_id) + "/endpoint";
}

std::vector<uint8_t> bytes_of(const std::string& s) {
    return std::vector<uint8_t>(s.begin(), s.end());
}

std::vector<uint8_t> file_bytes(const std::filesystem::path& path) {
    std::ifstream f(path, std::ios::binary);
    return std::vector<uint8_t>(std::istreambuf_iterator<char>(f),
                                std::istreambuf_iterator<char>());
}

// Its attribution is the content tileset's own credit, which environment_source_credits omits.
void serve_content_endpoint(IonFixtureNetwork* n) {
    overlume::testing::ion_fixture_serve(
        n, endpoint_url(96188), 200,
        bytes_of(
            R"({"type":"3DTILES","url":")" + kContentBase +
            R"(tileset.json","accessToken":"fixture-asset-token","attributions":[{"html":"Fixture Buildings","collapsible":false}]})"));
}

// Asset 96188: the endpoint, then the fixture's tileset.json and every tile at its own URL.
void serve_content(IonFixtureNetwork* n, const std::string& fixture_dir) {
    serve_content_endpoint(n);
    for (const auto& entry : std::filesystem::directory_iterator(fixture_dir)) {
        const std::string name = entry.path().filename().string();
        if (name == "PROVENANCE.md") continue;
        overlume::testing::ion_fixture_serve(n, kContentBase + name, 200, file_bytes(entry.path()));
    }
}

// Asset 1: Task 2's terrain endpoint, a layer.json listing the two level-0 tiles and those tiles at
// layer.json's "{z}/{x}/{y}.terrain". A layer.json refused with `layer_json_status` serves no tile.
void serve_terrain(IonFixtureNetwork* n, uint16_t layer_json_status = 200) {
    overlume::testing::ion_fixture_serve(n, endpoint_url(1), 200,
                                         bytes_of(qmesh::ion_terrain_endpoint_json(kTerrainBase)));
    if (layer_json_status != 200) {
        overlume::testing::ion_fixture_serve(n, kTerrainBase + "layer.json", layer_json_status, {});
        return;
    }
    overlume::testing::ion_fixture_serve(n, kTerrainBase + "layer.json", 200,
                                         bytes_of(qmesh::layer_json({{0, 0, 0}, {0, 1, 0}})));
    for (const uint32_t x : {0u, 1u}) {
        overlume::testing::ion_fixture_serve(
            n, kTerrainBase + "0/" + std::to_string(x) + "/0.terrain", 200,
            qmesh::encode_quantized_mesh(0, x, 0, qmesh::QuantizedMeshOptions{}));
    }
}

// Asset 2: `endpoint_json`, Task 2's tilemapresource.xml at the TMS base and beside it, and one
// flat PNG for every other URL under the base.
void serve_imagery(IonFixtureNetwork* n, const std::string& endpoint_json) {
    overlume::testing::ion_fixture_serve(n, endpoint_url(2), 200, bytes_of(endpoint_json));
    const std::vector<uint8_t> xml = bytes_of(qmesh::tilemapresource_xml(512));
    overlume::testing::ion_fixture_serve(n, kImageryBase, 200, xml);
    overlume::testing::ion_fixture_serve(n, kImageryBase + "tilemapresource.xml", 200, xml);
    overlume::testing::ion_fixture_serve_prefix(
        n, kImageryBase, 200, overlume::testing::imagery::flat_png(512, 40, 160, 220));
}

bool is_imagery_tile_url(const std::string& url) {
    const std::string key = url.substr(0, url.find('?'));
    return key.rfind(kImageryBase, 0) == 0 && key != kImageryBase &&
           key != kImageryBase + "tilemapresource.xml";
}

size_t count_requested(IonFixtureNetwork* n, const std::function<bool(const std::string&)>& match) {
    const std::vector<std::string> urls = overlume::testing::ion_fixture_requested_urls(n);
    return static_cast<size_t>(std::count_if(urls.begin(), urls.end(), match));
}

size_t count_of(const std::string& text, const std::string& needle) {
    size_t count = 0;
    for (size_t pos = text.find(needle); pos != std::string::npos;
         pos = text.find(needle, pos + 1)) {
        ++count;
    }
    return count;
}

overlume::VisualRenderer* make_renderer() {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    return overlume::create_renderer(cfg);
}

void set_ego(overlume::VisualRenderer* r) {
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);
}

// Renders kStdPose frames 5 ms apart until `done` holds; false if it never did.
bool pump(overlume::VisualRenderer* r, std::vector<uint8_t>& buf, int max_frames,
          const std::function<bool()>& done) {
    for (int i = 0; i < max_frames; ++i) {
        if (!overlume::render_frame(r, kStdPose, {buf.data(), 320, 240})) return false;
        if (done()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return done();
}

std::string credits(overlume::VisualRenderer* r) {
    std::vector<char> buf(overlume::environment_source_credits(r, nullptr, 0) + 1);
    overlume::environment_source_credits(r, buf.data(), static_cast<uint32_t>(buf.size()));
    return buf.data();
}

}

namespace itf = ion_terrain_fixture;

TEST(EnvironmentStreamTerrain, IonTerrainAndImageryLoadOverTheFixtureNetwork) {
    auto* r = itf::make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* n = overlume::testing::make_ion_fixture_network();
    itf::serve_content(n, kTilesFixtureDir);
    itf::serve_terrain(n);
    itf::serve_imagery(n, itf::qmesh::ion_imagery_endpoint_json(itf::kImageryBase));
    ASSERT_TRUE(overlume::testing::install_fixture_ion_source(
        r, "ion://96188?terrain=1&imagery=2&cache=off", kFixtureAnchor, n,
        "ion-fixture-token-load"));
    itf::set_ego(r);
    EXPECT_TRUE(overlume::testing::environment_stream_has_terrain_tileset(r));

    std::vector<uint8_t> buf(itf::kFrameBytes);
    const auto terrainWithImagery = [&] {
        return overlume::testing::environment_terrain_tiles_loaded(r) > 0 &&
               overlume::testing::environment_stream_overlay_probe(r).rasters_attached > 0;
    };
    ASSERT_TRUE(itf::pump(r, buf, 600, terrainWithImagery))
        << "terrain tiles in the scene: " << overlume::testing::environment_terrain_tiles_loaded(r)
        << ", rasters attached: "
        << overlume::testing::environment_stream_overlay_probe(r).rasters_attached;
    EXPECT_GT(overlume::testing::environment_stream_overlay_probe(r).textures_alive, 0u);
    const auto buildings = [&] { return overlume::testing::environment_loaded_chunk_count(r) > 0; };
    EXPECT_TRUE(itf::pump(r, buf, 300, buildings))
        << "the content tileset must still stream its buildings";
    const auto easternTile = [](const std::string& url) {
        return url.rfind(itf::kTerrainBase + "0/1/0.terrain", 0) == 0;
    };
    EXPECT_GT(itf::count_requested(n, easternTile), 0u)
        << "the level-0 tile under the ego, the eastern hemisphere's";
    EXPECT_GT(itf::count_requested(n, itf::is_imagery_tile_url), 0u);

    overlume::set_environment_visible(r, false);
    EXPECT_EQ(overlume::testing::environment_terrain_tiles_loaded(r), 0u)
        << "hiding the environment must take the terrain out of the scene too";
    overlume::set_environment_visible(r, true);
    EXPECT_GT(overlume::testing::environment_terrain_tiles_loaded(r), 0u);
    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::STREAMING);
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamTerrain, TheTerrainSampleProvidesTheGroundWithoutFollowTerrain) {
    auto* r = itf::make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* n = overlume::testing::make_ion_fixture_network();
    itf::serve_content(n, kTilesFixtureDir);
    itf::serve_terrain(n);
    ASSERT_TRUE(overlume::testing::install_fixture_ion_source(
        r, "ion://96188?terrain=1&cache=off", kFixtureAnchor, n, "ion-fixture-token-ground"));
    itf::set_ego(r);

    std::vector<uint8_t> buf(itf::kFrameBytes);
    bool hit = false;
    for (int i = 0; i < 600 && !hit; ++i) {
        overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        hit = overlume::testing::environment_stream_provides_ground(r);
    }
    ASSERT_TRUE(hit) << "the sample under the ego must hit the terrain tileset: this content "
                        "fixture is buildings only (GroundPlaneKeptWhenTilesetIsBuildingsOnly)";
    overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});
    EXPECT_FALSE(overlume::testing::renderer_ground_plane_in_scene(r))
        << "the terrain must take the clay plane's place";
    EXPECT_EQ(overlume::testing::environment_terrain_offset_z(r), 0.0)
        << "without follow_terrain the sample must not move the environment";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamTerrain, TheFollowTerrainLiftMovesTheTerrainTiles) {
    auto* r = itf::make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* n = overlume::testing::make_ion_fixture_network();
    itf::serve_content(n, kTilesFixtureDir);
    itf::serve_terrain(n);
    ASSERT_TRUE(overlume::testing::install_fixture_ion_source(
        r, "ion://96188?terrain=1&cache=off&follow_terrain=on&max_tilt_deg=0&ground_bias=0",
        kFixtureAnchor, n, "ion-fixture-token-lift"));
    itf::set_ego(r);

    // One flat tile per hemisphere: its chords run tens of kilometres below the ellipsoid under
    // the ego, so the sample lands far below the anchor and the offset settles at its 30 m clamp.
    std::vector<uint8_t> buf(itf::kFrameBytes);
    const auto lifted = [&] {
        return overlume::testing::environment_terrain_tiles_loaded(r) > 0 &&
               std::abs(overlume::testing::environment_terrain_offset_z(r)) > 1.0;
    };
    ASSERT_TRUE(itf::pump(r, buf, 600, lifted))
        << "offset " << overlume::testing::environment_terrain_offset_z(r);
    for (int i = 0; i < 10; ++i) overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});
    EXPECT_NEAR(overlume::testing::environment_terrain_tile_lift_z(r),
                overlume::testing::environment_terrain_offset_z(r), 1.0)
        << "the terrain tiles must ride the buildings' lift (1 m: float32 transforms whose "
           "translation is the anchor's ECEF distance, about 6.4e6 m)";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamTerrain, TheTerrainResourcesFollowAThemeSwitch) {
    auto* r = itf::make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* n = overlume::testing::make_ion_fixture_network();
    itf::serve_content(n, kTilesFixtureDir);
    itf::serve_terrain(n);
    ASSERT_TRUE(overlume::testing::install_fixture_ion_source(
        r, "ion://96188?terrain=1&cache=off", kFixtureAnchor, n, "ion-fixture-token-theme"));
    itf::set_ego(r);

    std::vector<uint8_t> buf(itf::kFrameBytes);
    const auto terrainInScene = [&] {
        return overlume::testing::environment_terrain_tiles_loaded(r) > 0;
    };
    ASSERT_TRUE(itf::pump(r, buf, 600, terrainInScene)) << "no terrain tile reached the scene";
    float dark[3] = {-1.0f, -1.0f, -1.0f};
    ASSERT_TRUE(overlume::testing::ion_fixture_terrain_color(n, dark));
    EXPECT_GE(dark[0], 0.0f) << "update() must push dark_adas's terrain_color to the terrain tiles";

    // A transition that ended 9.9 s before the scene's sim time 0: the next frame is light_clay.
    ASSERT_TRUE(overlume::set_theme(r, "light_clay", -10.0, 0.1));
    for (int i = 0; i < 3; ++i) overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});
    float light[3] = {-1.0f, -1.0f, -1.0f};
    ASSERT_TRUE(overlume::testing::ion_fixture_terrain_color(n, light));
    EXPECT_FALSE(dark[0] == light[0] && dark[1] == light[1] && dark[2] == light[2])
        << "live terrain tiles must take light_clay's terrain_color";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamTerrain, CreditsAreThePlainTextOfTheImageryAttributions) {
    auto* r = itf::make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    char untouched[16];
    std::memset(untouched, 'x', sizeof untouched);
    EXPECT_EQ(overlume::environment_source_credits(r, untouched, sizeof untouched), 0u);
    EXPECT_STREQ(untouched, "") << "no source is open";

    auto* n = overlume::testing::make_ion_fixture_network();
    itf::serve_content(n, kTilesFixtureDir);
    itf::serve_terrain(n);
    itf::serve_imagery(n, itf::kAttributedImageryEndpoint);
    ASSERT_TRUE(overlume::testing::install_fixture_ion_source(
        r, "ion://96188?terrain=1&imagery=2&cache=off", kFixtureAnchor, n,
        "ion-fixture-token-credits"));
    itf::set_ego(r);
    std::vector<uint8_t> buf(itf::kFrameBytes);
    // tiles loaded: the view groups now reference the tilesets' own credits, which the filter must
    // keep out
    const auto credited = [&] {
        return overlume::environment_source_credits(r, nullptr, 0) > 0 &&
               overlume::testing::environment_loaded_chunk_count(r) > 0 &&
               overlume::testing::environment_terrain_tiles_loaded(r) > 0;
    };
    ASSERT_TRUE(itf::pump(r, buf, 600, credited))
        << "the imagery credits or the tiles never appeared";

    EXPECT_EQ(itf::credits(r), "Fixture Imagery & Co.")
        << "tags stripped, &amp; decoded, and the content and terrain tilesets' own credits "
           "(\"Fixture Buildings\", the terrain endpoint's) left out";
    char shortBuf[8];
    EXPECT_EQ(overlume::environment_source_credits(r, shortBuf, sizeof shortBuf), 21u)
        << "the length needed, as snprintf returns it";
    EXPECT_STREQ(shortBuf, "Fixture");

    ASSERT_TRUE(overlume::testing::environment_stream_force_fall_back(r));
    EXPECT_EQ(overlume::environment_source_credits(r, nullptr, 0), 0u)
        << "the credits go with the streamed source";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamTerrain, ATerrainLayerJsonAnswering401OpensWithoutTerrainAndDoesNotFallBack) {
    auto* r = itf::make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* n = overlume::testing::make_ion_fixture_network();
    itf::serve_content(n, kTilesFixtureDir);
    itf::serve_terrain(n, 401);
    const size_t linesBefore =
        itf::count_of(overlume::testing::captured_cesium_log_text(), itf::kTerrainDidNotOpen401);
    ASSERT_TRUE(overlume::testing::install_fixture_ion_source(
        r, "ion://96188?terrain=1&cache=off", kFixtureAnchor, n, "ion-fixture-token-401"));
    itf::set_ego(r);

    std::vector<uint8_t> buf(itf::kFrameBytes);
    const auto buildingsWithoutTerrain = [&] {
        return !overlume::testing::environment_stream_has_terrain_tileset(r) &&
               overlume::testing::environment_loaded_chunk_count(r) > 0;
    };
    ASSERT_TRUE(itf::pump(r, buf, 600, buildingsWithoutTerrain))
        << "the source must drop the terrain whose root never came and keep its buildings";
    for (int i = 0; i < 60; ++i) overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});

    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::STREAMING)
        << "a refused terrain is not network loss";
    EXPECT_FALSE(overlume::testing::environment_stream_has_terrain_tileset(r));
    EXPECT_EQ(overlume::testing::environment_terrain_tiles_loaded(r), 0u);
    EXPECT_GT(overlume::testing::environment_loaded_chunk_count(r), 0u);
    EXPECT_EQ(
        itf::count_of(overlume::testing::captured_cesium_log_text(), itf::kTerrainDidNotOpen401),
        linesBefore + 1)
        << "the source must say so, once";
    const auto terrainTile = [](const std::string& url) {
        return url.find(".terrain") != std::string::npos;
    };
    EXPECT_EQ(itf::count_requested(n, terrainTile), 0u) << "no terrain tile without a layer.json";
    overlume::destroy_renderer(r);
}

// Review Focus 4. The first rasters attach unheld; then every new imagery tile request is held
// (the upsampled terrain children keep asking for finer imagery: the TMS fixture lists levels
// 0-18) and the source is switched with one in flight. Once the old source is gone nothing
// dispatches its main-thread queue, so the late raster runs only its worker half
// (prepareRasterInLoadThread) and makes no texture. The teardown must free every texture the old
// terrain made, and the late raster must attach nowhere.
TEST(EnvironmentStreamTerrain, SwitchingAwayWithARasterRequestInFlightLeavesNoTexture) {
    auto* r = itf::make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* n = overlume::testing::make_ion_fixture_network();
    itf::serve_content(n, kTilesFixtureDir);
    itf::serve_terrain(n);
    itf::serve_imagery(n, itf::qmesh::ion_imagery_endpoint_json(itf::kImageryBase));
    ASSERT_TRUE(overlume::testing::install_fixture_ion_source(
        r, "ion://96188?terrain=1&imagery=2&cache=off", kFixtureAnchor, n,
        "ion-fixture-token-switch"));
    itf::set_ego(r);

    std::vector<uint8_t> buf(itf::kFrameBytes);
    const auto textured = [&] {
        return overlume::testing::ion_fixture_terrain_probe(n).textures_alive > 0;
    };
    ASSERT_TRUE(itf::pump(r, buf, 600, textured)) << "the first rasters must attach unheld";

    overlume::testing::ion_fixture_hold_prefix(n, itf::kImageryBase);
    const size_t imageryRequests = itf::count_requested(n, itf::is_imagery_tile_url);
    const auto rasterInFlight = [&] {
        return itf::count_requested(n, itf::is_imagery_tile_url) > imageryRequests;
    };
    ASSERT_TRUE(itf::pump(r, buf, 600, rasterInFlight))
        << "no imagery tile was requested after the first rasters attached";
    const overlume::testing::OverlayProbe atSwitch =
        overlume::testing::ion_fixture_terrain_probe(n);
    ASSERT_GT(atSwitch.textures_alive, 0u);

    ASSERT_TRUE(overlume::set_environment_source(r, kTestTownDir.c_str(), kFixtureAnchor));
    overlume::testing::ion_fixture_release_held(n);
    const auto baked = [&] { return overlume::testing::environment_loaded_chunk_count(r) > 0; };
    EXPECT_TRUE(itf::pump(r, buf, 120, baked)) << "the baked source must load after the switch";
    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::BAKED);
    const overlume::testing::OverlayProbe after = overlume::testing::ion_fixture_terrain_probe(n);
    EXPECT_EQ(after.textures_alive, 0u)
        << "the teardown must free the old terrain's textures, and the late raster make none";
    EXPECT_GE(after.textures_freed, atSwitch.textures_alive + atSwitch.textures_freed);
    EXPECT_EQ(after.rasters_attached, atSwitch.rasters_attached)
        << "the late raster must not attach to a torn-down tile";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamTerrain, NetworkLossFallsBackAndTearsDownBothTilesets) {
    auto* r = itf::make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* n = overlume::testing::make_ion_fixture_network();
    itf::serve_content_endpoint(n);
    overlume::testing::ion_fixture_serve(
        n, itf::kContentBase + "tileset.json", 200,
        itf::file_bytes(kTilesFixtureFallbackDir + "/tileset.json"));
    // Its 16 tiles are byte-identical copies of tile_root.b3dm (the fixture's PROVENANCE.md), so
    // one prefix serves them all; held, they answer only when the network is already gone.
    overlume::testing::ion_fixture_serve_prefix(
        n, itf::kContentBase + "tile_", 200,
        itf::file_bytes(kTilesFixtureFallbackDir + "/tile_root.b3dm"));
    overlume::testing::ion_fixture_hold_prefix(n, itf::kContentBase + "tile_");
    itf::serve_terrain(n);
    itf::serve_imagery(n, itf::qmesh::ion_imagery_endpoint_json(itf::kImageryBase));
    ASSERT_TRUE(overlume::testing::install_fixture_ion_source(
        r, "ion://96188?terrain=1&imagery=2&cache=off", kFixtureAnchor, n,
        "ion-fixture-token-loss"));
    itf::set_ego(r);

    std::vector<uint8_t> buf(itf::kFrameBytes);
    const auto terrainWithImagery = [&] {
        return overlume::testing::environment_terrain_tiles_loaded(r) > 0 &&
               overlume::testing::environment_stream_overlay_probe(r).rasters_attached > 0;
    };
    ASSERT_TRUE(itf::pump(r, buf, 600, terrainWithImagery))
        << "terrain and imagery must load while the building tiles wait";
    ASSERT_GT(overlume::testing::environment_stream_overlay_probe(r).textures_alive, 0u);

    overlume::testing::ion_fixture_fail_all_transport(n);
    overlume::testing::ion_fixture_release_held(n);
    const auto fellBack = [&] {
        return overlume::environment_source_state(r) ==
               overlume::EnvironmentSourceState::STREAMING_FALLBACK;
    };
    ASSERT_TRUE(itf::pump(r, buf, 200, fellBack))
        << "eight consecutive transport errors must fall back";

    EXPECT_FALSE(overlume::testing::environment_stream_has_terrain_tileset(r));
    EXPECT_EQ(overlume::testing::environment_terrain_tiles_loaded(r), 0u);
    const overlume::testing::OverlayProbe probe = overlume::testing::ion_fixture_terrain_probe(n);
    EXPECT_EQ(probe.textures_alive, 0u) << "the fallback must free every raster texture";
    EXPECT_GT(probe.textures_freed, 0u);
    EXPECT_EQ(overlume::environment_source_credits(r, nullptr, 0), 0u);
    overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});
    EXPECT_TRUE(overlume::testing::renderer_ground_plane_in_scene(r))
        << "the clay plane must come back with the terrain gone";
    overlume::destroy_renderer(r);
}
