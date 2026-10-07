// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "environment_test_hooks.hpp"
#include "golden.hpp"
#include "test_paths.hpp"

#include "stb_image_write.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
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
                           std::vector<uint8_t>& buf, int max_frames = 200) {
    uint64_t count = 0;
    for (int i = 0; i < max_frames; ++i) {
        if (!overlume::render_frame(r, pose, {buf.data(), 320, 240})) return count;
        count = overlume::testing::environment_loaded_chunk_count(r);
        if (count > 0) return count;
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
