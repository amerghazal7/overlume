// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "environment_test_hooks.hpp"
#include "imagery_fixture.hpp"
#include "mapbox_dem_fixture.hpp"
#include "mapbox_mvt_fixture.hpp"
#include "mapbox_test_hooks.hpp"
#include "mapbox_tile_frame.hpp"
#include "test_paths.hpp"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace dem = overlume::testing::dem;
namespace imagery = overlume::testing::imagery;
namespace mvt = overlume::testing::mvt;
using overlume::EnvironmentSourceState;
using overlume::mapbox::TileId;
using overlume::testing::MapboxFixtureNetwork;

constexpr const char* kToken = "pk.test-token-not-real";
constexpr size_t kFrameBytes = 320u * 240u * 3u;
const overlume::CameraPose kPose{{0, -8, 3}, {0, 0, 0.5}, 60};
// The loaders' retry schedule on a frozen clock: attempt 2 no sooner than 2 s after attempt 1,
// attempt 3 no sooner than 4 s after attempt 2.
constexpr double kAttempt2AtS = 2.0;
constexpr double kAttempt3AtS = 6.0;
constexpr double kPi = 3.14159265358979323846;

// Tile math runs on use, not during static initialisation: it may lean on statics of other
// translation units. The test site's z14 tile, 10707/7006.
TileId op_tile() { return overlume::mapbox::tile_containing(55.2744, 25.1972, 14); }

overlume::GeoAnchor anchor_at(const TileId& t, double u, double v) {
    const overlume::mapbox::LonLatDeg p = overlume::mapbox::tile_point_to_lonlat(t, u, v, 1.0);
    return overlume::GeoAnchor{p.lat, p.lon, 0.0, 0.0};
}

// Ground metres per DEM pixel (512 per z14 tile) at tile t's centre, spherical Web Mercator.
double metres_per_dem_px(const TileId& t) {
    const double lat = overlume::mapbox::tile_point_to_lonlat(t, 0.5, 0.5, 1.0).lat;
    return 2.0 * kPi * 6378137.0 * std::cos(lat * kPi / 180.0) / 16384.0 / 512.0;
}

std::string dem_url_of(const TileId& t) { return overlume::testing::mapbox_dem_url(t.x, t.y); }

void serve_dem(MapboxFixtureNetwork* n, const TileId& t,
               const std::function<double(double, double)>& height_m) {
    overlume::testing::mapbox_fixture_serve(n, dem_url_of(t), 200, dem::encode_dem_png(height_m));
}

// One 20 m building in square 10 of tile t (z16 units 1000-1400), gzip as Mapbox sends it.
void serve_block(MapboxFixtureNetwork* n, const TileId& t) {
    mvt::Layer layer;
    layer.name = "building";
    layer.features = {
        mvt::building(1, {{{1000, 1000}, {1400, 1000}, {1400, 1400}, {1000, 1400}}}, 20.0)};
    const TileId square = overlume::mapbox::footprint_tiles_of(t)[10];
    overlume::testing::mapbox_fixture_serve(
        n, overlume::testing::mapbox_footprint_url(square.x, square.y), 200,
        mvt::gzip_stored(mvt::encode_tile({layer})));
}

overlume::VisualRenderer* make_renderer() {
    return overlume::create_renderer(overlume::RenderConfig{320, 240, 1, kThemeDir, "dark_adas"});
}

bool install(overlume::VisualRenderer* r, const std::string& uri, const overlume::GeoAnchor& anchor,
             MapboxFixtureNetwork* n) {
    return overlume::testing::install_fixture_mapbox_source(r, uri.c_str(), anchor, n, kToken);
}

// Map x points north and map y west at heading 0 (compute_ecef_to_map).
void set_ego(overlume::VisualRenderer* r, double x, double y) {
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = overlume::Vec3{x, y, 0.0};
    overlume::set_scene(r, s);
}

uint64_t loaded(overlume::VisualRenderer* r) {
    return overlume::testing::environment_loaded_chunk_count(r);
}

uint64_t terrain_loaded(overlume::VisualRenderer* r) {
    return overlume::testing::environment_terrain_tiles_loaded(r);
}

EnvironmentSourceState state(overlume::VisualRenderer* r) {
    return overlume::environment_source_state(r);
}

overlume::testing::MapboxSourceStats stats() {
    return overlume::testing::mapbox_last_source_stats();
}

// Renders until done() or max_frames; the sleep lets worker-thread builds land between frames.
template <typename Done>
bool pump(overlume::VisualRenderer* r, std::vector<uint8_t>& buf, int max_frames, Done done) {
    for (int i = 0; i < max_frames && !done(); ++i) {
        if (!overlume::render_frame(r, kPose, {buf.data(), 320, 240})) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return done();
}

void frames(overlume::VisualRenderer* r, std::vector<uint8_t>& buf, int count) {
    pump(r, buf, count, [] { return false; });
}

// Every requested URL, without its query string.
std::set<std::string> requested(MapboxFixtureNetwork* n) {
    std::set<std::string> out;
    for (const std::string& url : overlume::testing::mapbox_fixture_requested_urls(n)) {
        out.insert(url.substr(0, url.find('?')));
    }
    return out;
}

// How often `url` (without its query string) has been requested.
size_t times_requested(MapboxFixtureNetwork* n, const std::string& url) {
    size_t count = 0;
    for (const std::string& u : overlume::testing::mapbox_fixture_requested_urls(n)) {
        if (u.compare(0, u.find('?'), url) == 0) ++count;
    }
    return count;
}

size_t log_size() { return overlume::testing::captured_cesium_log_text().size(); }

std::string log_since(size_t from) {
    const std::string text = overlume::testing::captured_cesium_log_text();
    return text.size() > from ? text.substr(from) : std::string();
}

size_t count_of(const std::string& text, const std::string& needle) {
    size_t n = 0;
    for (size_t at = text.find(needle); at != std::string::npos; at = text.find(needle, at + 1)) {
        ++n;
    }
    return n;
}

bool ends_with(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() &&
           s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// Freezes the loaders' retry clock at 0 s for one test, which then moves time with set(). The
// steady clock comes back when the test ends, however it ends.
class FrozenRetryClock {
public:
    FrozenRetryClock() { set(0.0); }
    ~FrozenRetryClock() { overlume::testing::mapbox_set_test_clock(-1.0); }
    FrozenRetryClock(const FrozenRetryClock&) = delete;
    FrozenRetryClock& operator=(const FrozenRetryClock&) = delete;
    void set(double seconds) { overlume::testing::mapbox_set_test_clock(seconds); }
};

}

TEST(MapboxTerrain, TerrainTileBuildsFromItsDem) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_dem(n, t, dem::hill_height_m);
    ASSERT_TRUE(install(r, "mapbox://terrain", anchor_at(t, 0.5, 0.5), n));
    EXPECT_TRUE(overlume::testing::environment_stream_has_terrain_tileset(r));
    EXPECT_FALSE(overlume::testing::environment_stream_has_disk_cache(r))
        << "no Mapbox byte reaches the disk (the hook reads the content tileset, whose counting "
           "accessor the terrain tileset's wraps)";
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return terrain_loaded(r) >= 1; }))
        << "the terrain tile never reached the scene";
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.terrain_tiles_built, 1u) << "500 m around the tile centre is one z14 tile";
    EXPECT_EQ(s.tiles_built, 0u) << "no buildings layer, no buildings";
    EXPECT_EQ(s.dem_requests, 1u);
    EXPECT_EQ(requested(n), std::set<std::string>{dem_url_of(t)})
        << "the terrain tile asks for its DEM and nothing else";
    EXPECT_EQ(overlume::testing::mapbox_fixture_requested_urls(n).size(), 1u) << "once";
    EXPECT_EQ(state(r), EnvironmentSourceState::STREAMING);
    overlume::destroy_renderer(r);
}

TEST(MapboxTerrain, OneDemFeedsBothLoaders) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_dem(n, t, dem::hill_height_m);
    serve_block(n, t);
    ASSERT_TRUE(install(r, "mapbox://buildings,terrain", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1 && terrain_loaded(r) >= 1; }));
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.tiles_built, 1u);
    EXPECT_EQ(s.terrain_tiles_built, 1u);
    EXPECT_EQ(s.dem_requests, 1u);
    EXPECT_EQ(times_requested(n, dem_url_of(t)), 1u)
        << "the anchor's reference, the buildings and the terrain share one request";
    overlume::destroy_renderer(r);
}

TEST(MapboxTerrain, MissingDemLeavesNoTerrainTile) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    FrozenRetryClock clock;
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    overlume::testing::mapbox_fixture_serve(n, dem_url_of(t), 404, {});
    const size_t logFrom = log_size();
    ASSERT_TRUE(install(r, "mapbox://terrain", anchor_at(t, 0.5, 0.5), n));
    EXPECT_TRUE(overlume::testing::environment_stream_has_terrain_tileset(r));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 100, [&] { return times_requested(n, dem_url_of(t)) == 1; }));
    frames(r, buf, 20);
    clock.set(kAttempt2AtS);
    frames(r, buf, 20);
    clock.set(kAttempt3AtS);
    frames(r, buf, 20);
    EXPECT_EQ(stats().terrain_tiles_built, 0u);
    EXPECT_EQ(terrain_loaded(r), 0u) << "no terrain there: the sky shows through";
    EXPECT_EQ(times_requested(n, dem_url_of(t)), 1u)
        << "a 404 is no failure for now: the tile settles empty and nothing asks again";
    EXPECT_EQ(stats().dem_requests, 1u);
    EXPECT_EQ(state(r), EnvironmentSourceState::STREAMING) << "one 404 is not network loss";
    EXPECT_EQ(count_of(log_since(logFrom), "HTTP 404"), 0u) << "404 is an empty tile, not an error";
    overlume::destroy_renderer(r);
}

TEST(MapboxTerrain, FailingDemIsAskedAgainOnTheNextAttempt) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    FrozenRetryClock clock;
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    overlume::testing::mapbox_fixture_serve(n, dem_url_of(t), 503, {});
    const size_t logFrom = log_size();
    ASSERT_TRUE(install(r, "mapbox://terrain", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 100, [&] { return times_requested(n, dem_url_of(t)) == 1; }));
    frames(r, buf, 20);
    EXPECT_EQ(times_requested(n, dem_url_of(t)), 1u)
        << "the anchor's reference and the tile share the attempt's one request";
    serve_dem(n, t, dem::hill_height_m);
    // The cache shares a failure for 1 s of steady time and the retry clock is frozen: wait it out.
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    clock.set(kAttempt2AtS);
    ASSERT_TRUE(pump(r, buf, 400, [] { return stats().terrain_tiles_built == 1; }))
        << "attempt 2 must ask again and build from the 200";
    EXPECT_EQ(times_requested(n, dem_url_of(t)), 2u);
    EXPECT_EQ(stats().dem_requests, 2u);
    EXPECT_EQ(count_of(log_since(logFrom), "mapbox: terrain-DEM failing (HTTP 503)"), 1u);
    overlume::destroy_renderer(r);
}

TEST(MapboxTerrain, SatelliteDrapesXyzTilesOnTheTerrain) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_dem(n, t, dem::hill_height_m);
    // From z0 to the overlay's z18 cap, every XYZ tile over t and the ring of tiles around it
    // answers one flat 512 px image: what t's terrain may ask for when the rows are Mapbox's (row 0
    // at the north edge), the ring because cesium may map a neighbour row a rounding error away
    // (QuadtreeRasterOverlayTileProvider.cpp:256-287). `inside` keeps the z15-z18 tiles over t.
    std::set<std::string> served;
    std::set<std::string> inside;
    for (uint32_t z = 0; z <= 18; ++z) {
        const int64_t k = z <= 14 ? 1 : int64_t{1} << (z - 14);
        const int64_t x0 = z <= 14 ? t.x >> (14 - z) : t.x * k;
        const int64_t y0 = z <= 14 ? t.y >> (14 - z) : t.y * k;
        const int64_t side = int64_t{1} << z;
        for (int64_t y = y0 - 1; y <= y0 + k; ++y) {
            for (int64_t x = x0 - 1; x <= x0 + k; ++x) {
                if (x < 0 || y < 0 || x >= side || y >= side) continue;
                const std::string url = overlume::testing::mapbox_satellite_url(
                    z, static_cast<uint32_t>(x), static_cast<uint32_t>(y));
                served.insert(url);
                if (z >= 15 && x >= x0 && x < x0 + k && y >= y0 && y < y0 + k) inside.insert(url);
            }
        }
    }
    const std::vector<uint8_t> png = imagery::flat_png(512, 40, 120, 60);
    for (const std::string& url : served) overlume::testing::mapbox_fixture_serve(n, url, 200, png);
    ASSERT_TRUE(install(r, "mapbox://terrain?imagery=satellite", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 600, [&] {
        return overlume::testing::environment_stream_overlay_probe(r).rasters_attached >= 1;
    })) << "no satellite raster attached to the terrain";
    size_t overT = 0;
    for (const std::string& url : overlume::testing::mapbox_fixture_requested_urls(n)) {
        const std::string bare = url.substr(0, url.find('?'));
        if (bare.find("mapbox.satellite") == std::string::npos) continue;
        EXPECT_EQ(served.count(bare), 1u)
            << "not an XYZ tile over the terrain tile or its ring: " << bare;
        overT += inside.count(bare);
        EXPECT_TRUE(ends_with(url, std::string("?access_token=") + kToken)) << bare;
    }
    EXPECT_GT(overT, 0u) << "no z15-z18 imagery over the terrain tile itself";
    char credits[256] = {};
    overlume::environment_source_credits(r, credits, static_cast<uint32_t>(sizeof credits));
    EXPECT_NE(std::string(credits).find("\xC2\xA9 Maxar"), std::string::npos)
        << "the overlay's credit, for the attribution line: " << credits;
    overlume::destroy_renderer(r);
}

TEST(MapboxTerrain, FailedSatelliteTileLogsThroughTheRedactingLogger) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_dem(n, t, dem::hill_height_m);
    // Every imagery tile answers 204, which cesium reads as a failed image and replaces by its
    // parent, level by level, dropping the child's error; the z0 tile answers 401, the one failure
    // cesium keeps and logs. The 204s are successes to the network-loss count, so the source keeps
    // streaming.
    const std::string z0 = overlume::testing::mapbox_satellite_url(0, 0, 0);
    overlume::testing::mapbox_fixture_serve(n, z0, 401, {});
    const std::string line = "Received response code 401 for image ";
    const size_t logFrom = log_size();
    ASSERT_TRUE(install(r, "mapbox://terrain?imagery=satellite", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    const auto logged = [&] {
        return count_of(log_since(logFrom), line) >= 1 && stats().terrain_tiles_built >= 1;
    };
    ASSERT_TRUE(pump(r, buf, 600, logged))
        << "cesium's line for the failed tile never reached the source's log: "
        << log_since(logFrom);
    frames(r, buf, 20);
    const std::string log = log_since(logFrom);
    EXPECT_GE(times_requested(n, z0), 1u);
    EXPECT_EQ(log.find(kToken), std::string::npos) << "the token never reaches the log";
    EXPECT_EQ(stats().terrain_tiles_built, 1u) << "a failed raster leaves the terrain tile";
    EXPECT_EQ(state(r), EnvironmentSourceState::STREAMING);
    overlume::destroy_renderer(r);
}

TEST(MapboxTerrain, TerrainLayerReplacesTheGroundPlane) {
    for (const bool terrain : {true, false}) {
        auto* r = make_renderer();
        if (!r) GTEST_SKIP() << "no GPU/EGL";
        const TileId t = op_tile();
        auto* n = overlume::testing::make_mapbox_fixture_network(204);
        serve_dem(n, t, dem::ramp_height_m);  // 0..20 m west to east
        serve_block(n, t);
        ASSERT_TRUE(install(r, terrain ? "mapbox://buildings,terrain" : "mapbox://buildings",
                            anchor_at(t, 0.5, 0.5), n));
        // The ego stands 300 m east of the anchor (map y points west), up the ramp.
        constexpr double kEgoEastM = 300.0;
        set_ego(r, 0.0, -kEgoEastM);
        std::vector<uint8_t> buf(kFrameBytes);
        ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
        if (terrain) {
            EXPECT_TRUE(pump(r, buf, 400, [&] {
                return overlume::testing::environment_stream_provides_ground(r);
            })) << "the terrain owns the ground once the sample under the ego hits it";
            frames(r, buf, 2);
            EXPECT_FALSE(overlume::testing::renderer_ground_plane_in_scene(r))
                << "the clay ground plane is hidden";
            // Following the terrain moves the environment down by the ramp's rise under the ego
            // and the 0.3 m ground bias, so the ego stands on the terrain.
            const double rise = dem::ramp_height_m(256.0 + kEgoEastM / metres_per_dem_px(t), 0.0) -
                                dem::ramp_height_m(256.0, 0.0);
            const double expected = -(rise + 0.3);
            const auto onTerrain = [&] {
                return std::abs(overlume::testing::environment_terrain_offset_z(r) - expected) <
                       0.5;
            };
            EXPECT_TRUE(pump(r, buf, 200, onTerrain))
                << "the source follows the terrain: offset "
                << overlume::testing::environment_terrain_offset_z(r) << ", expected " << expected;
        } else {
            frames(r, buf, 20);
            EXPECT_FALSE(overlume::testing::environment_stream_provides_ground(r))
                << "buildings alone never replace the ground plane";
            EXPECT_TRUE(overlume::testing::renderer_ground_plane_in_scene(r));
        }
        overlume::destroy_renderer(r);
    }
}
