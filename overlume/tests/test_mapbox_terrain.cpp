// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "environment_test_hooks.hpp"
#include "golden.hpp"
#include "imagery_fixture.hpp"
#include "mapbox_dem_fixture.hpp"
#include "mapbox_mvt_fixture.hpp"
#include "mapbox_test_hooks.hpp"
#include "mapbox_tile_frame.hpp"
#include "test_paths.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <thread>
#include <utility>
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
// When attempts 4 and 5 would start if nothing capped them (6 + 2*4 and 14 + 2*8, from
// mapbox_tiling.cpp:61).
constexpr double kAttempt4AtS = 14.0;
constexpr double kAttempt5AtS = 30.0;
constexpr double kPi = 3.14159265358979323846;

// Tile math runs on use, not during static initialisation: it may lean on statics of other
// translation units. The operating area's z14 tile, 10712/7012.
TileId op_tile() { return overlume::mapbox::tile_containing(55.3910, 25.0803, 14); }

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
        << "no Mapbox byte reaches the disk (the hook reads both tilesets' accessors; the terrain "
           "tileset shares the content tileset's counting accessor)";
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
    EXPECT_EQ(stats().terrain_retries, 0u) << "a 404 is no retry";
    EXPECT_EQ(terrain_loaded(r), 0u) << "no terrain there: the sky shows through";
    EXPECT_EQ(times_requested(n, dem_url_of(t)), 1u)
        << "a 404 is no failure for now: the tile settles empty and nothing asks again";
    EXPECT_EQ(stats().dem_requests, 1u);
    EXPECT_EQ(state(r), EnvironmentSourceState::STREAMING) << "one 404 is not network loss";
    EXPECT_EQ(count_of(log_since(logFrom), "HTTP 404"), 0u) << "404 is an empty tile, not an error";
    // The anchor's own DEM is the missing one, so no tile has a zero to stand on: the source says
    // which tile, once.
    const std::string anchorLine = "mapbox: terrain-DEM has no height at the anchor (tile 14/" +
                                   std::to_string(t.x) + "/" + std::to_string(t.y) + ")";
    EXPECT_EQ(count_of(log_since(logFrom), anchorLine), 1u) << log_since(logFrom);
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
    EXPECT_EQ(stats().terrain_retries, 1u);
    overlume::destroy_renderer(r);
}

TEST(MapboxTerrain, FailingDemSettlesEmptyAfterTheThirdAttempt) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    FrozenRetryClock clock;
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    overlume::testing::mapbox_fixture_serve(n, dem_url_of(t), 503, {});
    ASSERT_TRUE(install(r, "mapbox://terrain", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 100, [&] { return times_requested(n, dem_url_of(t)) == 1; }));
    frames(r, buf, 20);
    // The cache shares a failure for 1 s of steady time and the retry clock is frozen: wait it
    // out before each step.
    const std::pair<double, size_t> asks[] = {{kAttempt2AtS, 2}, {kAttempt3AtS, 3}};
    for (const auto& [at, count] : asks) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1100));
        clock.set(at);
        ASSERT_TRUE(pump(r, buf, 400, [&] { return times_requested(n, dem_url_of(t)) == count; }))
            << "attempt " << count << " must ask again";
        frames(r, buf, 20);
    }
    for (const double at : {kAttempt4AtS, kAttempt5AtS}) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1100));
        clock.set(at);
        frames(r, buf, 40);
    }
    EXPECT_EQ(times_requested(n, dem_url_of(t)), 3u) << "the third attempt is the last";
    EXPECT_EQ(stats().dem_requests, 3u);
    EXPECT_EQ(stats().terrain_tiles_built, 0u);
    EXPECT_EQ(terrain_loaded(r), 0u);
    EXPECT_EQ(stats().terrain_retries, 2u) << "attempts 1 and 2 retry, attempt 3 settles empty";
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
    // (QuadtreeRasterOverlayTileProvider.cpp:256-287). `inside` keeps the z15-z18 tiles over t,
    // `inside17` the z17-z18 ones.
    std::set<std::string> served;
    std::set<std::string> inside;
    std::set<std::string> inside17;
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
                if (z >= 15 && x >= x0 && x < x0 + k && y >= y0 && y < y0 + k) {
                    inside.insert(url);
                    if (z >= 17) inside17.insert(url);
                }
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
    size_t over17 = 0;
    size_t satelliteRequests = 0;
    size_t aboveCap = 0;
    for (const std::string& url : overlume::testing::mapbox_fixture_requested_urls(n)) {
        const std::string bare = url.substr(0, url.find('?'));
        const size_t at = bare.find("mapbox.satellite/");
        if (at == std::string::npos) continue;
        ++satelliteRequests;
        if (std::stoi(bare.substr(at + std::strlen("mapbox.satellite/"))) > 18) ++aboveCap;
        EXPECT_EQ(served.count(bare), 1u)
            << "not an XYZ tile over the terrain tile or its ring: " << bare;
        overT += inside.count(bare);
        over17 += inside17.count(bare);
        EXPECT_TRUE(ends_with(url, std::string("?access_token=") + kToken)) << bare;
    }
    EXPECT_EQ(aboveCap, 0u) << "imagery past the z18 cap";
    // 53 requests measured at 512 px tiles; 225 once the tile size is dropped. Spec section 12
    // promises a few dozen per z14 tile.
    EXPECT_LE(satelliteRequests, 64u) << "satellite requests per z14 tile: the 512 px tile size "
                                         "(billing) has regressed";
    EXPECT_GT(overT, 0u) << "no z15-z18 imagery over the terrain tile itself";
    // Only the z14 leaves' geometric error 2 makes cesium refine to z16, whose imagery is z17; a
    // z14 leaf at error 0 stops at z16 imagery.
    EXPECT_GT(over17, 0u)
        << "no z17+ imagery over the terrain tile: the z14 leaves are not refined";
    char credits[256] = {};
    overlume::environment_source_credits(r, credits, static_cast<uint32_t>(sizeof credits));
    EXPECT_NE(std::string(credits).find("\xC2\xA9 Maxar"), std::string::npos)
        << "the overlay's credit, for the attribution line: " << credits;
    // The overlay's own limits, as cesium's tile provider holds them. The requests above cannot
    // see the z18 cap (the texture budget stops refinement at z17 first), and a missing 512 px
    // size only shows as 225 requests instead of 53: the billing guards are read directly.
    const overlume::testing::OverlayOptionsProbe options =
        overlume::testing::environment_stream_overlay_options_probe(r);
    ASSERT_TRUE(options.found) << "the satellite overlay's tile provider never came up";
    EXPECT_EQ(options.maximum_level, 18u) << "the imagery cap (billing)";
    EXPECT_EQ(options.tile_width, 512u);
    EXPECT_EQ(options.tile_height, 512u);
    EXPECT_EQ(options.credit, "\xC2\xA9 Maxar");
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
            // 0.1 m: the 0.3 m bias must show (without it the offset lands within 0.01 m of -rise).
            const auto onTerrain = [&] {
                return std::abs(overlume::testing::environment_terrain_offset_z(r) - expected) <
                       0.1;
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

TEST(MapboxTerrain, BrightnessReachesTheTerrainRendererResources) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_dem(n, t, dem::hill_height_m);
    ASSERT_TRUE(
        install(r, "mapbox://terrain?imagery=satellite&brightness=1.8", anchor_at(t, 0.5, 0.5), n));
    EXPECT_DOUBLE_EQ(overlume::testing::environment_stream_terrain_brightness(r), 1.8)
        << "the imagery texture's brightness comes from the terrain tileset's resources";
    overlume::destroy_renderer(r);
}

// A teardown before the source's first update() used to free the quadtree root in cesium
// (StreamingEnvironmentSource::teardown says how). The root is every mapbox:// source's, so
// buildings alone crashed as well.
TEST(MapboxTerrain, InstallThenDestroyWithoutPumpingDoesNotCrash) {
    for (const char* uri : {"mapbox://buildings,terrain", "mapbox://terrain",
                            "mapbox://buildings,terrain?imagery=satellite", "mapbox://buildings"}) {
        SCOPED_TRACE(uri);
        auto* r = make_renderer();
        if (!r) GTEST_SKIP() << "no GPU/EGL";
        const TileId t = op_tile();
        auto* n = overlume::testing::make_mapbox_fixture_network(204);
        serve_dem(n, t, dem::hill_height_m);
        serve_block(n, t);
        ASSERT_TRUE(install(r, uri, anchor_at(t, 0.5, 0.5), n));
        overlume::destroy_renderer(r);
    }
}

// The node's live switch: set_environment_source (and the install hook) tears the old source
// down, here before its first update(); the renderer then goes on with the new source.
TEST(MapboxTerrain, SwitchingAwayBeforeTheFirstUpdateDoesNotCrash) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_dem(n, t, dem::hill_height_m);
    ASSERT_TRUE(install(r, "mapbox://buildings,terrain", anchor_at(t, 0.5, 0.5), n));
    ASSERT_TRUE(install(r, "mapbox://terrain", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    EXPECT_TRUE(overlume::render_frame(r, kPose, {buf.data(), 320, 240}));
    EXPECT_EQ(state(r), EnvironmentSourceState::STREAMING);
    overlume::destroy_renderer(r);
}

namespace {

// Task 12's end-to-end helpers. A namespace of their own keeps them clear of the helpers Task 10
// wrote above; Tasks 13 and 14 reuse them.
namespace e2e {

namespace dem = overlume::testing::dem;
namespace imagery = overlume::testing::imagery;
namespace mvt = overlume::testing::mvt;
using overlume::mapbox::TileId;
using overlume::testing::MapboxFixtureNetwork;

constexpr const char* kToken = "pk.test-token-not-real";
constexpr double kPi = 3.14159265358979323846;
constexpr uint32_t kWidth = 320;
constexpr uint32_t kHeight = 240;
constexpr size_t kFrameBytes = size_t{kWidth} * kHeight * 3u;
// Where the ground under the ego is level, the follow-terrain transform lowers the environment by
// the ground bias (environment_stream.cpp:974-977).
constexpr double kGroundBiasM = 0.3;
// A Mapbox source selects tiles with its area view, not the camera, so any pose loads them.
const overlume::CameraPose kAnyPose{{0.0, -8.0, 3.0}, {0.0, 0.0, 0.5}, 60.0};
const std::string kTownDir =
    std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/environment_test_town_0";
const std::string kSatellitePrefix = "https://api.mapbox.com/v4/mapbox.satellite/";

struct Rgb {
    uint8_t r, g, b;
};
// The imagery fixture paints each quadrant of the z14 tile in one colour, far apart in hue from the
// others, from light_clay's blue sky and from the clay.
constexpr Rgb kNorthWest{220, 50, 40};   // red
constexpr Rgb kNorthEast{40, 180, 50};   // green
constexpr Rgb kSouthWest{230, 200, 30};  // yellow
constexpr Rgb kSouthEast{200, 40, 210};  // magenta
constexpr Rgb kAncestor{128, 128, 128};  // levels 0-14 cover all four quadrants

using Colour = std::array<double, 3>;

// The operating area's z14 tile, 14/10712/7012.
TileId op_tile() { return overlume::mapbox::tile_containing(55.3910, 25.0803, 14); }

// The anchor at (u, v) of tile t, in fractions of its edge, heading pi/2: map +x east, +y north.
overlume::GeoAnchor anchor_in(const TileId& t, double u, double v) {
    const overlume::mapbox::LonLatDeg p = overlume::mapbox::tile_point_to_lonlat(t, u, v, 1.0);
    return overlume::GeoAnchor{p.lat, p.lon, kPi / 2.0, 0.0};
}

// Map metres of DEM pixel coordinates (u east, v south, 512 per edge) of z14 tile t, z_m metres
// above the anchor's ground, through the stream's placement chain.
overlume::Vec3 px_to_map(const overlume::GeoAnchor& a, const TileId& t, double u, double v,
                         double z_m) {
    double xyz[3] = {0.0, 0.0, 0.0};
    EXPECT_TRUE(
        overlume::testing::mapbox_tile_point_to_map_probe(a, 14, t.x, t.y, u, v, 512.0, z_m, xyz));
    return overlume::Vec3{xyz[0], xyz[1], xyz[2]};
}

overlume::Vec3 lowered(const overlume::Vec3& p) {
    return overlume::Vec3{p.x, p.y, p.z - kGroundBiasM};
}

overlume::VisualRenderer* make_renderer(const char* theme) {
    return overlume::create_renderer(overlume::RenderConfig{kWidth, kHeight, 1, kThemeDir, theme});
}

bool install(overlume::VisualRenderer* r, const std::string& uri, const overlume::GeoAnchor& a,
             MapboxFixtureNetwork* n) {
    return overlume::testing::install_fixture_mapbox_source(r, uri.c_str(), a, n, kToken);
}

void set_ego(overlume::VisualRenderer* r, double x, double y) {
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = overlume::Vec3{x, y, 0.0};
    overlume::set_scene(r, s);
}

// Renders until done() or max_frames; the sleep lets worker-thread work land between frames.
template <typename Done>
bool pump(overlume::VisualRenderer* r, const overlume::CameraPose& pose,
          std::vector<uint8_t>& frame, int max_frames, Done done) {
    for (int i = 0; i < max_frames && !done(); ++i) {
        if (!overlume::render_frame(r, pose, {frame.data(), kWidth, kHeight})) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return done();
}

overlume::testing::MapboxSourceStats stats() {
    return overlume::testing::mapbox_last_source_stats();
}

overlume::testing::OverlayProbe overlay(overlume::VisualRenderer* r) {
    return overlume::testing::environment_stream_overlay_probe(r);
}

// Both Mapbox loaders built `tiles` z14 tiles and a satellite raster is attached.
bool streamed_with_imagery(overlume::VisualRenderer* r, uint64_t tiles) {
    const overlume::testing::MapboxSourceStats s = stats();
    return s.tiles_built >= tiles && s.terrain_tiles_built >= tiles &&
           overlay(r).rasters_attached >= 1;
}

std::string credits(overlume::VisualRenderer* r) {
    std::array<char, 512> text{};
    overlume::environment_source_credits(r, text.data(), static_cast<uint32_t>(text.size()));
    return std::string(text.data());
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

bool contains(const std::vector<uint8_t>& bytes, const std::string& needle) {
    return std::search(bytes.begin(), bytes.end(), needle.begin(), needle.end()) != bytes.end();
}

std::string bare(const std::string& url) { return url.substr(0, url.find('?')); }

size_t times_requested(MapboxFixtureNetwork* n, const std::string& url) {
    size_t count = 0;
    for (const std::string& u : overlume::testing::mapbox_fixture_requested_urls(n)) {
        if (bare(u) == url) ++count;
    }
    return count;
}

// Every satellite request, without its query string.
std::vector<std::string> requested_satellite(MapboxFixtureNetwork* n) {
    std::vector<std::string> out;
    for (const std::string& u : overlume::testing::mapbox_fixture_requested_urls(n)) {
        if (u.rfind(kSatellitePrefix, 0) == 0) out.push_back(bare(u));
    }
    return out;
}

void serve_dem(MapboxFixtureNetwork* n, const TileId& t,
               const std::function<double(double, double)>& height_m) {
    overlume::testing::mapbox_fixture_serve(n, overlume::testing::mapbox_dem_url(t.x, t.y), 200,
                                            dem::encode_dem_png(height_m));
}

// An MVT exterior ring: positive surveyor's area in tile units (y down).
std::vector<std::array<int32_t, 2>> box(int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
    return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}

// A box u0..u1 east, v0..v1 south, in DEM pixels of its z14 tile (128 per z16 square), as an MVT
// ring in the 4096 units of the z16 square it lies in; `square` indexes footprint_tiles_of
// (row * 4 + column).
struct PxBox {
    size_t square = 0;
    std::vector<std::array<int32_t, 2>> ring;
};

PxBox px_box(double u0, double v0, double u1, double v1) {
    const int col = static_cast<int>(u0 / 128.0);
    const int row = static_cast<int>(v0 / 128.0);
    const auto unit = [](double px, int index) {
        return static_cast<int32_t>(std::lround((px - 128.0 * index) * 32.0));
    };
    return {static_cast<size_t>(row * 4 + col),
            box(unit(u0, col), unit(v0, row), unit(u1, col), unit(v1, row))};
}

// Serves z16 square `index` of z14 tile t: a building layer of `features`, gzip as Mapbox sends it.
void serve_square(MapboxFixtureNetwork* n, const TileId& t, size_t index,
                  std::vector<mvt::Feature> features) {
    const TileId square = overlume::mapbox::footprint_tiles_of(t)[index];
    mvt::Layer layer;
    layer.name = "building";
    layer.features = std::move(features);
    overlume::testing::mapbox_fixture_serve(
        n, overlume::testing::mapbox_footprint_url(square.x, square.y), 200,
        mvt::gzip_stored(mvt::encode_tile({layer})));
}

struct ImageryTile {
    std::string url;  // no query string
    Rgb colour;
};

// The satellite tiles over z14 tile t at levels 0-18: its 15 ancestors (levels 0-14) and its 340
// descendants (levels 15-18), each descendant in the colour of the quadrant of t it lies in.
// ponytail: the whole pyramid (355 URLs, five PNGs) instead of predicting the levels cesium picks;
// cesium also falls back to a failed tile's parent (QuadtreeRasterOverlayTileProvider.cpp:120-135).
// A scene whose radius reaches a neighbouring z14 tile serves that tile's pyramid too.
std::vector<ImageryTile> imagery_tiles(const TileId& t) {
    std::vector<ImageryTile> out;
    for (uint32_t z = 0; z <= 14; ++z) {
        const uint32_t shift = 14 - z;
        out.push_back(
            {overlume::testing::mapbox_satellite_url(z, t.x >> shift, t.y >> shift), kAncestor});
    }
    for (uint32_t z = 15; z <= 18; ++z) {
        const uint32_t side = 1u << (z - 14);
        const uint32_t half = side / 2;
        for (uint32_t row = 0; row < side; ++row) {
            for (uint32_t col = 0; col < side; ++col) {
                const bool north = row < half;  // XYZ rows count from the north
                const bool west = col < half;
                const Rgb colour =
                    north ? (west ? kNorthWest : kNorthEast) : (west ? kSouthWest : kSouthEast);
                out.push_back(
                    {overlume::testing::mapbox_satellite_url(z, t.x * side + col, t.y * side + row),
                     colour});
            }
        }
    }
    return out;
}

// Serves imagery_tiles(t) as 512-px PNGs and returns their URLs.
std::set<std::string> serve_imagery(MapboxFixtureNetwork* n, const TileId& t) {
    std::map<uint32_t, std::vector<uint8_t>> pngs;  // one encoding per colour
    std::set<std::string> served;
    for (const ImageryTile& tile : imagery_tiles(t)) {
        const uint32_t key = (uint32_t{tile.colour.r} << 16) | (uint32_t{tile.colour.g} << 8) |
                             uint32_t{tile.colour.b};
        auto it = pngs.find(key);
        if (it == pngs.end()) {
            it = pngs.emplace(key,
                              imagery::flat_png(512, tile.colour.r, tile.colour.g, tile.colour.b))
                     .first;
        }
        overlume::testing::mapbox_fixture_serve(n, tile.url, 200, it->second);
        served.insert(tile.url);
    }
    return served;
}

// Every satellite URL cesium may ask for over z14 tile t: levels 0-18, the tiles over t and the
// ring of tiles around them (cesium maps a neighbouring row a rounding error away,
// QuadtreeRasterOverlayTileProvider.cpp:256-287; SatelliteDrapesXyzTilesOnTheTerrain measures it).
// A mirrored row lies far outside it.
std::set<std::string> imagery_urls_with_ring(const TileId& t) {
    std::set<std::string> out;
    for (uint32_t z = 0; z <= 18; ++z) {
        const int64_t k = z <= 14 ? 1 : int64_t{1} << (z - 14);
        const int64_t x0 = z <= 14 ? t.x >> (14 - z) : int64_t{t.x} * k;
        const int64_t y0 = z <= 14 ? t.y >> (14 - z) : int64_t{t.y} * k;
        const int64_t side = int64_t{1} << z;
        for (int64_t y = y0 - 1; y <= y0 + k; ++y) {
            for (int64_t x = x0 - 1; x <= x0 + k; ++x) {
                if (x < 0 || y < 0 || x >= side || y >= side) continue;
                out.insert(overlume::testing::mapbox_satellite_url(z, static_cast<uint32_t>(x),
                                                                   static_cast<uint32_t>(y)));
            }
        }
    }
    return out;
}

// The plateau scene on z14 tile t: 0 m west of DEM column 280, a ramp to 30 m at column 300 and a
// 30 m plateau east of it; one 15 m block about 22 m square on the plateau (DEM pixels 337.5-342.5
// east, 239.5-244.5 south: about 350 m east and 60 m north of the tile centre); quadrant imagery.
// Returns the imagery URLs served.
std::set<std::string> serve_plateau_scene(MapboxFixtureNetwork* n, const TileId& t) {
    serve_dem(n, t, [](double u, double) {
        return u < 280.0 ? 0.0 : (u < 300.0 ? (u - 280.0) * 1.5 : 30.0);
    });
    const PxBox block = px_box(337.5, 239.5, 342.5, 244.5);
    serve_square(n, t, block.square, {mvt::building(301, {block.ring}, 15.0)});
    return serve_imagery(n, t);
}

// The 3x3 mean colour around pixel (x, y) of a 320x240 RGB frame, row 0 at the top.
Colour colour_at_px(const std::vector<uint8_t>& frame, int x, int y) {
    const int cx = std::clamp(x, 1, static_cast<int>(kWidth) - 2);
    const int cy = std::clamp(y, 1, static_cast<int>(kHeight) - 2);
    Colour sum{0.0, 0.0, 0.0};
    for (int j = cy - 1; j <= cy + 1; ++j) {
        for (int i = cx - 1; i <= cx + 1; ++i) {
            const size_t at = (static_cast<size_t>(j) * kWidth + static_cast<size_t>(i)) * 3u;
            for (size_t c = 0; c < 3; ++c) sum[c] += frame[at + c];
        }
    }
    for (double& c : sum) c /= 9.0;
    return sum;
}

// The colour where map point p lands in `frame`, rendered through the renderer's current camera
// (the last render_frame's). False off-screen.
bool colour_at(overlume::VisualRenderer* r, const std::vector<uint8_t>& frame,
               const overlume::Vec3& p, Colour* out) {
    float u = 0.0f, v = 0.0f;
    if (!overlume::project_to_screen(r, p, &u, &v)) return false;
    *out = colour_at_px(frame, static_cast<int>(u * static_cast<float>(kWidth)),
                        static_cast<int>(v * static_cast<float>(kHeight)));
    return true;
}

// ponytail: a hue test, not colour science. Every channel the fixture colour sets 100 or more
// above another must stay 40 above it after lighting, fog and tone mapping. If a theme change
// narrows that margin, compare against a rendered reference instead, as
// RejectedImageryKeepsTerrainColourAndCountsTowardFallback does.
bool looks_like(const Colour& c, const Rgb& want) {
    const std::array<int, 3> w{want.r, want.g, want.b};
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            if (w[i] - w[j] >= 100 && c[i] - c[j] < 40.0) return false;
        }
    }
    return true;
}

bool near(const Colour& a, const Colour& b, double tolerance) {
    for (size_t c = 0; c < 3; ++c) {
        if (std::abs(a[c] - b[c]) > tolerance) return false;
    }
    return true;
}

std::string str(const Colour& c) {
    return "(" + std::to_string(std::lround(c[0])) + ", " + std::to_string(std::lround(c[1])) +
           ", " + std::to_string(std::lround(c[2])) + ")";
}

}

}

// Spec §8: one DEM per tile with two consumers. Around a z14 corner four tiles load; each DEM is
// fetched once though the terrain loader, the buildings loader and the anchor's z_ref read it.
TEST(MapboxTerrainEndToEnd, CornerTilesFetchEachDemOnceForBothLoaders) {
    namespace mvt = overlume::testing::mvt;
    auto* r = e2e::make_renderer("dark_adas");
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    // The ego stands just inside D's north-west corner, where A (north-west), B (north), C (west)
    // and D meet: 500 m reaches all four tiles.
    const e2e::TileId d = e2e::op_tile();
    struct Ground {
        e2e::TileId tile;
        double height_m;
        uint64_t id;
    };
    // Level DEMs: D, the anchor's tile, at 0 m, so z_ref = 0; the other three higher.
    const std::array<Ground, 4> ground{{{{14, d.x - 1, d.y - 1}, 18.0, 401},
                                        {{14, d.x, d.y - 1}, 6.0, 402},
                                        {{14, d.x - 1, d.y}, 12.0, 403},
                                        {d, 0.0, 404}}};
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    for (const Ground& g : ground) {
        const double h = g.height_m;
        e2e::serve_dem(n, g.tile, [h](double, double) { return h; });
        // One 10 m box per tile, in z16 square 5 (row 1, column 1).
        e2e::serve_square(n, g.tile, 5,
                          {mvt::building(g.id, {e2e::box(1000, 1000, 1400, 1400)}, 10.0)});
    }
    ASSERT_TRUE(e2e::install(r, "mapbox://buildings,terrain", e2e::anchor_in(d, 0.002, 0.002), n));
    e2e::set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> frame(e2e::kFrameBytes);
    // Built is not yet in the scene: the main thread prepares a tile and the stream admits it in
    // later frames, so the pump waits for both.
    const bool built = e2e::pump(r, e2e::kAnyPose, frame, 600, [&] {
        const overlume::testing::MapboxSourceStats s = e2e::stats();
        return s.tiles_built == 4 && s.terrain_tiles_built == 4 &&
               overlume::testing::environment_terrain_tiles_loaded(r) == 4;
    });
    ASSERT_TRUE(built) << "tiles_built " << e2e::stats().tiles_built << ", terrain_tiles_built "
                       << e2e::stats().terrain_tiles_built << ", terrain tiles in the scene "
                       << overlume::testing::environment_terrain_tiles_loaded(r);
    const overlume::testing::MapboxSourceStats s = e2e::stats();
    EXPECT_EQ(s.dem_requests, 4u)
        << "one DEM request per tile, though the terrain loader, the buildings loader and the "
           "anchor's z_ref all read them";
    for (const Ground& g : ground) {
        EXPECT_EQ(e2e::times_requested(n, overlume::testing::mapbox_dem_url(g.tile.x, g.tile.y)),
                  1u)
            << "DEM 14/" << g.tile.x << "/" << g.tile.y;
    }
    EXPECT_NEAR(s.max_tile_top_m, 28.0, 0.2)
        << "A's 10 m box stands on A's DEM, 18 m above the anchor's ground (z_ref is D's 0 m)";
    EXPECT_EQ(overlume::testing::environment_terrain_tiles_loaded(r), 4u);
    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::STREAMING);
    overlume::destroy_renderer(r);
}

// Spec §8: imagery requests present, the token only in query strings.
TEST(MapboxTerrainEndToEnd, ImageryRequestedWithTheTokenOnlyInQueryStrings) {
    auto* r = e2e::make_renderer("dark_adas");
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const e2e::TileId t = e2e::op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    const std::set<std::string> served = e2e::serve_plateau_scene(n, t);
    overlume::testing::capture_stream_glbs(true);
    const size_t logFrom = e2e::log_size();
    ASSERT_TRUE(e2e::install(r, "mapbox://buildings,terrain?imagery=satellite",
                             e2e::anchor_in(t, 0.5, 0.5), n));
    EXPECT_FALSE(overlume::testing::environment_stream_has_disk_cache(r))
        << "no cache database: no Mapbox byte reaches the disk";
    e2e::set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> frame(e2e::kFrameBytes);
    ASSERT_TRUE(
        e2e::pump(r, e2e::kAnyPose, frame, 600, [&] { return e2e::streamed_with_imagery(r, 1); }));
    const std::vector<std::vector<uint8_t>> glbs = overlume::testing::captured_stream_glbs();
    overlume::testing::capture_stream_glbs(false);

    for (const std::string& url : overlume::testing::mapbox_fixture_requested_urls(n)) {
        const std::string query = std::string("?access_token=") + e2e::kToken;
        EXPECT_TRUE(url.size() > query.size() &&
                    url.compare(url.size() - query.size(), query.size(), query) == 0)
            << e2e::bare(url);
    }
    const std::vector<std::string> satellite = e2e::requested_satellite(n);
    const std::set<std::string> allowed = e2e::imagery_urls_with_ring(t);
    EXPECT_FALSE(satellite.empty()) << "the overlay requested no imagery";
    size_t inPyramid = 0;
    for (const std::string& url : satellite) {
        inPyramid += served.count(url);
        EXPECT_EQ(allowed.count(url), 1u)
            << "outside the z14 tile's level 0-18 pyramid and its ring (a level over 18 or a "
               "mirrored row): "
            << url;
    }
    EXPECT_GT(inPyramid, 0u) << "no request landed in the pyramid itself";
    EXPECT_EQ(e2e::times_requested(n, overlume::testing::mapbox_dem_url(t.x, t.y)), 1u);
    ASSERT_FALSE(glbs.empty());
    for (const std::vector<uint8_t>& glb : glbs) {
        EXPECT_FALSE(e2e::contains(glb, e2e::kToken)) << "a request URL reached a model";
        EXPECT_FALSE(e2e::contains(glb, "access_token"));
    }
    const std::string log = e2e::log_since(logFrom);
    EXPECT_EQ(log.find(e2e::kToken), std::string::npos);
    const std::string credited = e2e::credits(r);
    EXPECT_NE(credited.find("\xC2\xA9 Maxar"), std::string::npos)
        << "credits: '" << credited << "'";
    overlume::destroy_renderer(r);
}

// Spec §8: a building on the hill with its base lifted, by stats and a pixel probe.
TEST(MapboxTerrainEndToEnd, BuildingOnThePlateauStandsOnTheDem) {
    auto* r = e2e::make_renderer("light_clay");
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const e2e::TileId t = e2e::op_tile();
    const overlume::GeoAnchor a = e2e::anchor_in(t, 0.5, 0.5);
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    e2e::serve_plateau_scene(n, t);
    ASSERT_TRUE(e2e::install(r, "mapbox://buildings,terrain?imagery=satellite", a, n));
    e2e::set_ego(r, 0.0, 0.0);
    // The block's west wall at mid-height: its base is the DEM at its centroid (30 m) less z_ref
    // (the 0 m under the anchor), so 30 + 15 / 2 m above the anchor's ground.
    const overlume::Vec3 wall = e2e::lowered(e2e::px_to_map(a, t, 337.5, 242.0, 37.5));
    // Open plateau north-west of the block, in the tile's north-east quadrant: green imagery.
    const overlume::Vec3 plateau = e2e::lowered(e2e::px_to_map(a, t, 330.0, 236.0, 30.0));
    // From 150 m east of the ego, 45 m up, over the ramp's top edge. A block left at z = 0 is
    // buried in the plateau, and the ray through the wall's spot lands on the far, fogged
    // plateau, which is strongly tinted.
    const overlume::CameraPose pose{{150.0, wall.y, 45.0}, {wall.x, wall.y, wall.z}, 60.0};
    std::vector<uint8_t> frame(e2e::kFrameBytes);
    ASSERT_TRUE(e2e::pump(r, pose, frame, 600, [&] {
        return e2e::streamed_with_imagery(r, 1) &&
               overlume::testing::environment_stream_provides_ground(r);
    }));
    e2e::pump(r, pose, frame, 30, [] { return false; });
    EXPECT_NEAR(e2e::stats().max_tile_top_m, 45.0, 0.3)
        << "the block's top must sit 30 m (the plateau) plus 15 m above the anchor's ground";

    ASSERT_TRUE(overlume::render_frame(r, pose, {frame.data(), e2e::kWidth, e2e::kHeight}));
    const e2e::Colour sky = e2e::colour_at_px(frame, 160, 2);
    e2e::Colour atWall{}, atPlateau{};
    ASSERT_TRUE(e2e::colour_at(r, frame, wall, &atWall));
    ASSERT_TRUE(e2e::colour_at(r, frame, plateau, &atPlateau));
    ASSERT_TRUE(e2e::looks_like(atPlateau, e2e::kNorthEast))
        << "the plateau must show the fixture's north-east green: " << e2e::str(atPlateau);
    const auto [lo, hi] = std::minmax_element(atWall.begin(), atWall.end());
    EXPECT_LT(*hi - *lo, 40.0)
        << "the block's west wall must read as near-neutral clay; an unlifted block shows the "
           "fogged plateau behind it "
        << e2e::str(atWall);
    EXPECT_FALSE(e2e::near(atWall, sky, 10.0))
        << "sky where the block's west wall belongs " << e2e::str(atWall);
    overlume::destroy_renderer(r);
}

// Spec §8: provides_ground after the ego's sample, the clay plane gone (pixel probe) and the
// fixture tile's colour at known points, one per quadrant.
TEST(MapboxTerrainEndToEnd, TerrainReplacesTheClayPlaneAndShowsEachQuadrant) {
    auto* r = e2e::make_renderer("light_clay");
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const e2e::TileId t = e2e::op_tile();
    const overlume::GeoAnchor a = e2e::anchor_in(t, 0.5, 0.5);
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    e2e::serve_dem(n, t, overlume::testing::dem::sea_height_m);
    e2e::serve_imagery(n, t);
    ASSERT_TRUE(e2e::install(r, "mapbox://terrain?imagery=satellite", a, n));
    e2e::set_ego(r, 0.0, 0.0);
    const overlume::CameraPose pose{{0.0, -45.0, 40.0}, {0.0, 15.0, 0.0}, 60.0};
    std::vector<uint8_t> frame(e2e::kFrameBytes);
    const bool drawn = e2e::pump(r, pose, frame, 600, [&] {
        return e2e::stats().terrain_tiles_built == 1 && e2e::overlay(r).rasters_attached >= 1 &&
               overlume::testing::environment_stream_provides_ground(r);
    });
    ASSERT_TRUE(drawn) << "provides_ground "
                       << overlume::testing::environment_stream_provides_ground(r);
    e2e::pump(r, pose, frame, 30, [] { return false; });
    EXPECT_FALSE(overlume::testing::renderer_ground_plane_in_scene(r))
        << "the terrain owns the ground: the clay plane must leave the scene";

    // Six DEM pixels (about 26 m) from the tile centre into the northern quadrants, three (13 m)
    // into the southern ones: all inside the clay plane's 120 m square, which would cover them.
    struct Probe {
        const char* name;
        double u, v;
        e2e::Rgb colour;
    };
    const std::array<Probe, 4> probes{{{"north-east", 262.0, 250.0, e2e::kNorthEast},
                                       {"north-west", 250.0, 250.0, e2e::kNorthWest},
                                       {"south-west", 250.0, 259.0, e2e::kSouthWest},
                                       {"south-east", 262.0, 259.0, e2e::kSouthEast}}};
    ASSERT_TRUE(overlume::render_frame(r, pose, {frame.data(), e2e::kWidth, e2e::kHeight}));
    for (const Probe& p : probes) {
        e2e::Colour c{};
        ASSERT_TRUE(e2e::colour_at(r, frame, e2e::lowered(e2e::px_to_map(a, t, p.u, p.v, 0.0)), &c))
            << p.name;
        EXPECT_TRUE(e2e::looks_like(c, p.colour))
            << p.name << " ground reads " << e2e::str(c)
            << ": the clay plane covers the terrain, or the imagery is mirrored or missing";
    }
    overlume::destroy_renderer(r);
}

// Spec §8: imagery 401 -> terrainColor and the counter. Held requests keep the terrain tile drawn
// without a raster, so the probe must match a run with imagery off. Released as 401, every
// satellite request fails; cesium walks each failed tile up to level 0
// (QuadtreeRasterOverlayTileProvider.cpp:120-135), so the run of failures passes eight and the
// source falls back.
TEST(MapboxTerrainEndToEnd, RejectedImageryKeepsTerrainColourAndCountsTowardFallback) {
    // Both runs read frame kProbeFrame (counted from create_renderer): Filament's temporal dither
    // makes a frame's bytes depend on its index.
    constexpr int kProbeFrame = 120;
    const e2e::TileId t = e2e::op_tile();
    const overlume::GeoAnchor a = e2e::anchor_in(t, 0.5, 0.5);
    const overlume::CameraPose pose{{0.0, -45.0, 40.0}, {0.0, 15.0, 0.0}, 60.0};
    const auto terrain_drawn = [](overlume::VisualRenderer* r) {
        return e2e::stats().terrain_tiles_built == 1 &&
               overlume::testing::environment_terrain_tiles_loaded(r) >= 1 &&
               overlume::testing::environment_stream_provides_ground(r);
    };
    // Renders up to frame kProbeFrame, reaching terrain_drawn first; the probe's colour then.
    const auto probe_at_frame = [&](overlume::VisualRenderer* r, e2e::Colour* out) {
        std::vector<uint8_t> frame(e2e::kFrameBytes);
        int frames = 0;
        for (; frames < kProbeFrame - 1 && !terrain_drawn(r); ++frames) {
            if (!overlume::render_frame(r, pose, {frame.data(), e2e::kWidth, e2e::kHeight})) {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (!terrain_drawn(r)) return false;
        for (; frames < kProbeFrame; ++frames) {
            if (!overlume::render_frame(r, pose, {frame.data(), e2e::kWidth, e2e::kHeight})) {
                return false;
            }
        }
        return e2e::colour_at(r, frame, e2e::lowered(e2e::px_to_map(a, t, 262.0, 250.0, 0.0)), out);
    };

    e2e::Colour reference{};
    {
        auto* r = e2e::make_renderer("light_clay");
        if (!r) GTEST_SKIP() << "no GPU/EGL";
        auto* n = overlume::testing::make_mapbox_fixture_network(204);
        e2e::serve_dem(n, t, overlume::testing::dem::sea_height_m);
        ASSERT_TRUE(e2e::install(r, "mapbox://terrain", a, n));
        e2e::set_ego(r, 0.0, 0.0);
        ASSERT_TRUE(probe_at_frame(r, &reference)) << "imagery off: the terrain never drew";
        overlume::destroy_renderer(r);
    }

    auto* r = e2e::make_renderer("light_clay");
    ASSERT_NE(r, nullptr);
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    e2e::serve_dem(n, t, overlume::testing::dem::sea_height_m);
    for (const e2e::ImageryTile& tile : e2e::imagery_tiles(t)) {
        overlume::testing::mapbox_fixture_serve(n, tile.url, 401, {});
        overlume::testing::mapbox_fixture_hold(n, tile.url);
    }
    const size_t logFrom = e2e::log_size();
    ASSERT_TRUE(
        e2e::install(r, "mapbox://terrain?imagery=satellite&fallback=" + e2e::kTownDir, a, n));
    e2e::set_ego(r, 0.0, 0.0);
    e2e::Colour held{};
    ASSERT_TRUE(probe_at_frame(r, &held)) << "imagery held: the terrain never drew";
    ASSERT_FALSE(e2e::requested_satellite(n).empty()) << "no imagery request is in flight";
    EXPECT_EQ(e2e::overlay(r).rasters_attached, 0u);
    EXPECT_TRUE(e2e::near(held, reference, 6.0))
        << "before its imagery the terrain must show terrainColor: " << e2e::str(held) << " vs "
        << e2e::str(reference) << " with imagery off";

    overlume::testing::mapbox_fixture_release_held(n);
    std::vector<uint8_t> frame(e2e::kFrameBytes);
    EXPECT_TRUE(e2e::pump(r, pose, frame, 120, [&] {
        return overlume::environment_source_state(r) ==
               overlume::EnvironmentSourceState::STREAMING_FALLBACK;
    })) << "rejected imagery requests must count toward the network-loss fallback";
    EXPECT_EQ(e2e::overlay(r).rasters_attached, 0u) << "a rejected raster must never attach";
    // Cesium logs a failed raster's URL only when the root tile fails too (spec §5 table), and
    // this run falls back first, so nothing is logged here; NetworkLossTearsDownTerrainAndImagery
    // proves the redaction.
    EXPECT_EQ(e2e::log_since(logFrom).find(e2e::kToken), std::string::npos);
    overlume::destroy_renderer(r);
}

// Spec §8: network loss -> fallback with both tilesets torn down. Everything streams; then the
// ego drives into the east neighbour, whose every request fails in transport.
TEST(MapboxTerrainEndToEnd, NetworkLossTearsDownTerrainAndImagery) {
    auto* r = e2e::make_renderer("dark_adas");
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const e2e::TileId t = e2e::op_tile();
    const e2e::TileId east{14, t.x + 1, t.y};
    const overlume::GeoAnchor a = e2e::anchor_in(t, 0.5, 0.5);
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    e2e::serve_plateau_scene(n, t);
    overlume::testing::mapbox_fixture_fail_transport(
        n, overlume::testing::mapbox_dem_url(east.x, east.y));
    overlume::testing::mapbox_fixture_fail_transport(
        n, overlume::testing::mapbox_landmark_url(east.x, east.y));
    for (const e2e::TileId& square : overlume::mapbox::footprint_tiles_of(east)) {
        overlume::testing::mapbox_fixture_fail_transport(
            n, overlume::testing::mapbox_footprint_url(square.x, square.y));
    }
    ASSERT_TRUE(e2e::install(
        r, "mapbox://buildings,terrain?imagery=satellite&fallback=" + e2e::kTownDir, a, n));
    e2e::set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> frame(e2e::kFrameBytes);
    ASSERT_TRUE(e2e::pump(r, e2e::kAnyPose, frame, 600, [&] {
        return e2e::streamed_with_imagery(r, 1) && e2e::overlay(r).textures_alive >= 1;
    }));
    ASSERT_TRUE(overlume::testing::environment_stream_has_terrain_tileset(r));
    ASSERT_FALSE(e2e::credits(r).empty());
    // The east tile's imagery is down too. An unmatched URL answers 204, a success that would
    // reset the consecutive-failure count between two frames while the tile's imagery loads
    // beside its failing DEM, so every satellite URL over the east tile fails in transport.
    for (const std::string& url : e2e::imagery_urls_with_ring(east)) {
        overlume::testing::mapbox_fixture_fail_transport(n, url);
    }

    // The east tile's centre: more than 1100 m from the old tile, so it alone is selected, and its
    // 18 requests (16 squares, the landmark tile, the DEM) fail in a row.
    const overlume::Vec3 eastCentre = e2e::px_to_map(a, east, 256.0, 256.0, 0.0);
    const size_t logFrom = e2e::log_size();
    e2e::set_ego(r, eastCentre.x, eastCentre.y);
    ASSERT_TRUE(e2e::pump(r, e2e::kAnyPose, frame, 300, [&] {
        return overlume::environment_source_state(r) ==
               overlume::EnvironmentSourceState::STREAMING_FALLBACK;
    })) << "transport errors on a new tile must reach the fallback";
    e2e::pump(r, e2e::kAnyPose, frame, 2, [] { return false; });
    const std::string log = e2e::log_since(logFrom);
    EXPECT_NE(log.find("access_token=<redacted>"), std::string::npos)
        << "precondition: the failed requests' URLs reach the log";
    EXPECT_EQ(log.find(e2e::kToken), std::string::npos) << "the token never reaches the log";
    EXPECT_FALSE(overlume::testing::environment_stream_has_terrain_tileset(r))
        << "the fallback must tear the terrain tileset down too";
    EXPECT_EQ(e2e::overlay(r).textures_alive, 0u) << "imagery textures outlived the teardown";
    EXPECT_TRUE(overlume::testing::renderer_ground_plane_in_scene(r)) << "the clay plane is back";
    EXPECT_EQ(e2e::credits(r), "") << "no overlay is open, so no credit (and no attribution)";
    overlume::destroy_renderer(r);
}

// Spec §8: the URI refusals (layer list, imagery). Each opens nothing, logs one line and requests
// nothing.
TEST(MapboxTerrainEndToEnd, RefusedLayerListsAndImageryOpenNothing) {
    auto* r = e2e::make_renderer("dark_adas");
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const overlume::GeoAnchor a = e2e::anchor_in(e2e::op_tile(), 0.5, 0.5);
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    struct Refusal {
        const char* uri;
        const char* reason;  // a part of the logged reason; "" when the contract pins none
    };
    const std::array<Refusal, 7> refusals{
        {{"mapbox://buildings,buildings", ""},
         {"mapbox://terrain,roads",
          "supports only the layers 'buildings' and 'terrain', not 'roads'"},
         {"mapbox://buildings,,terrain", ""},
         {"mapbox://", ""},
         {"mapbox://buildings?imagery=satellite", "imagery=satellite needs the 'terrain' layer"},
         {"mapbox://terrain?imagery=streets", ""},
         {"mapbox://terrain?cache=tiles", ""}}};
    for (const Refusal& f : refusals) {
        const size_t logFrom = e2e::log_size();
        EXPECT_FALSE(e2e::install(r, f.uri, a, n)) << f.uri;
        const std::string logged = e2e::log_since(logFrom);
        EXPECT_EQ(e2e::count_of(logged, "refused"), 1u) << f.uri << ": " << logged;
        EXPECT_NE(logged.find(f.reason), std::string::npos) << f.uri << ": " << logged;
    }
    e2e::set_ego(r, 0.0, 0.0);  // a source opened by mistake would now request its tiles
    std::vector<uint8_t> frame(e2e::kFrameBytes);
    e2e::pump(r, e2e::kAnyPose, frame, 3, [] { return false; });
    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::NONE);
    EXPECT_TRUE(overlume::testing::mapbox_fixture_requested_urls(n).empty());
    overlume::destroy_renderer(r);
}

// Review Focus 1: neighbouring DEM tiles 5 m apart in height along their shared edge. Seen from
// the low side 60 m away, the high tile's skirt must fill the step: no sky along the edge. Run
// twice: terrain alone, and with satellite imagery, where the edges near the ego are drawn by
// cesium's upsampled children, whose skirts come from clipping our skirt triangles.
TEST(MapboxTerrainEndToEnd, FiveMetreStepBetweenTilesShowsNoSkyAlongTheEdge) {
    for (const bool withImagery : {false, true}) {
        const std::string pass = withImagery ? " with imagery" : "";
        auto* r = e2e::make_renderer("light_clay");
        if (!r) GTEST_SKIP() << "no GPU/EGL";
        const e2e::TileId west = e2e::op_tile();
        const e2e::TileId east{14, west.x + 1, west.y};
        // The ego about 50 m west of the shared edge (the tile is about 2217 m wide there).
        const overlume::GeoAnchor a = e2e::anchor_in(west, 1.0 - 50.0 / 2217.0, 0.5);
        auto* n = overlume::testing::make_mapbox_fixture_network(204);
        e2e::serve_dem(n, west, overlume::testing::dem::sea_height_m);
        e2e::serve_dem(n, east, [](double, double) { return 5.0; });
        if (withImagery) {
            e2e::serve_imagery(n, west);
            e2e::serve_imagery(n, east);
        }
        ASSERT_TRUE(e2e::install(
            r, withImagery ? "mapbox://buildings,terrain?imagery=satellite" : "mapbox://terrain", a,
            n))
            << pass;
        e2e::set_ego(r, 0.0, 0.0);
        const overlume::Vec3 edge = e2e::px_to_map(a, west, 512.0, 256.0, 0.0);
        const overlume::CameraPose pose{{edge.x - 60.0, edge.y, 3.0}, {edge.x, edge.y, 2.0}, 60.0};
        std::vector<uint8_t> frame(e2e::kFrameBytes);
        ASSERT_TRUE(e2e::pump(r, pose, frame, 600, [&] {
            return e2e::stats().terrain_tiles_built == 2 &&
                   overlume::testing::environment_terrain_tiles_loaded(r) >= 2 &&
                   overlume::testing::environment_stream_provides_ground(r) &&
                   (!withImagery || e2e::overlay(r).rasters_attached >= 1);
        })) << pass;
        e2e::pump(r, pose, frame, 10, [] { return false; });
        ASSERT_TRUE(overlume::render_frame(r, pose, {frame.data(), e2e::kWidth, e2e::kHeight}));
        const e2e::Colour sky = e2e::colour_at_px(frame, 160, 2);
        // The sky reference is the top row (about 29 degrees up); the control is the low tile's
        // ground 30 m before the edge, which must not read as sky.
        e2e::Colour ground{};
        ASSERT_TRUE(e2e::colour_at(
            r, frame, e2e::lowered(e2e::px_to_map(a, west, 505.0, 256.0, 0.0)), &ground))
            << pass;
        ASSERT_FALSE(e2e::near(ground, sky, 10.0)) << "the low tile's ground reads as sky" << pass;
        // The probe row: 1.7 m above the low tile's ground, inside the 5 m step, every 1.2 DEM
        // pixels (about 5 m) along 60 m of the edge.
        for (int k = -6; k <= 6; ++k) {
            e2e::Colour c{};
            const overlume::Vec3 p = e2e::px_to_map(a, west, 512.0, 256.0 + 1.2 * k, 1.7);
            ASSERT_TRUE(e2e::colour_at(r, frame, p, &c)) << "probe " << k << " off-screen" << pass;
            EXPECT_FALSE(e2e::near(c, sky, 10.0)) << "sky through the step at probe " << k << " (y "
                                                  << p.y << " m): no skirt, or one "
                                                  << "too short or facing inwards" << pass;
        }
        overlume::destroy_renderer(r);
    }
}

namespace {

namespace terrain_golden {

// The golden's hill: 15 m high, a gaussian of 32 DEM pixels (about 138 m: four cells of the
// terrain mesh's 8-pixel grid, which follows it to within 0.28 m) centred 75 pixels (about 323 m)
// north of the tile centre, on 0 m ground. Under the anchor (the tile centre) it is 0.95 m, so
// its top stands 14.05 m above the anchor's ground.
double hill_m(double u, double v) {
    const double du = u - 256.0;
    const double dv = v - 181.0;
    return 15.0 * std::exp(-(du * du + dv * dv) / 2048.0);
}

// Filament's temporal dither changes every frame's noise with the frame's index since
// create_renderer, so the compared frame is this one however early the scene settles. Settling
// takes the frames to readiness (about 40 here) plus a second of stillness, at most 200 frames
// with the 5 ms sleep, twice over if a late raster stage restarts the window: 500 holds on a
// machine of any speed, and each frame after settling costs about 1.5 ms.
constexpr int kComparedFrame = 500;

// Present: the hill, red west and green east of the north-south line through the ego, and three 8 m
// blocks standing on the DEM: on the summit (just east of that line), on the east slope, and on
// level ground near the ego. Absent: the clay ground plane, and sky anywhere below the crest.
void synthetic_hill(const char* theme, const char* golden_png, const char* actual_png,
                    bool probe_colours) {
    namespace mvt = overlume::testing::mvt;
    auto* r = e2e::make_renderer(theme);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const e2e::TileId t = e2e::op_tile();
    const overlume::GeoAnchor a = e2e::anchor_in(t, 0.5, 0.5);
    // Unmatched URLs answer 404, so every request the scene needs is served below.
    auto* n = overlume::testing::make_mapbox_fixture_network();
    std::set<std::string> served = e2e::serve_imagery(n, t);
    // Cesium also asks for the ring of tiles one row off the pyramid (a rounding error away,
    // imagery_urls_with_ring); those answer 404 and are allowed, a mirrored row is not.
    const std::set<std::string> ring = e2e::imagery_urls_with_ring(t);
    served.insert(ring.begin(), ring.end());
    e2e::serve_dem(n, t, hill_m);
    served.insert(overlume::testing::mapbox_dem_url(t.x, t.y));
    const e2e::PxBox summit = e2e::px_box(256.5, 179.8, 259.0, 182.2);
    const e2e::PxBox slope = e2e::px_box(270.7, 189.0, 275.3, 191.0);
    const e2e::PxBox level = e2e::px_box(258.2, 250.1, 260.0, 251.9);
    ASSERT_EQ(slope.square, summit.square);  // all three blocks lie in z16 square 6
    ASSERT_EQ(level.square, summit.square);
    e2e::serve_square(
        n, t, summit.square,
        {mvt::building(501, {summit.ring}, 8.0), mvt::building(502, {slope.ring}, 8.0),
         mvt::building(503, {level.ring}, 8.0)});
    const std::array<e2e::TileId, 16> squares = overlume::mapbox::footprint_tiles_of(t);
    for (size_t i = 0; i < squares.size(); ++i) {
        const std::string url = overlume::testing::mapbox_footprint_url(squares[i].x, squares[i].y);
        if (i != summit.square) overlume::testing::mapbox_fixture_serve(n, url, 204, {});
        served.insert(url);
    }
    const std::string landmark = overlume::testing::mapbox_landmark_url(t.x, t.y);
    overlume::testing::mapbox_fixture_serve(n, landmark, 204, {});
    served.insert(landmark);
    // radius=150 keeps the upsampled terrain to the tiles around the ego.
    ASSERT_TRUE(e2e::install(r, "mapbox://buildings,terrain?imagery=satellite&radius=150", a, n));
    e2e::set_ego(r, 0.0, 0.0);

    const overlume::CameraPose pose{{0.0, -100.0, 35.0}, {0.0, 100.0, 7.0}, 20.0};
    std::vector<uint8_t> frame(e2e::kFrameBytes);
    // Settled: both loaders built the tile, the ego's ground sample landed, a raster is attached,
    // and neither the terrain tiles in the scene nor the attached rasters changed for 10 frames
    // and for a second of wall clock. Upsampling and imagery decoding run on worker threads, so a
    // pause between raster stages can outlast 10 fast frames.
    int frames = 0;
    int still = 0;
    auto stillSince = std::chrono::steady_clock::now();
    bool settled = false;
    uint64_t lastTiles = 0;
    uint32_t lastRasters = 0;
    for (; frames < 1000 && !settled; ++frames) {
        ASSERT_TRUE(overlume::render_frame(r, pose, {frame.data(), e2e::kWidth, e2e::kHeight}));
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        const overlume::testing::MapboxSourceStats s = e2e::stats();
        const uint64_t tiles = overlume::testing::environment_terrain_tiles_loaded(r);
        const uint32_t rasters = e2e::overlay(r).rasters_attached;
        const bool ready = s.tiles_built == 1 && s.terrain_tiles_built == 1 && rasters >= 1 &&
                           overlume::testing::environment_stream_provides_ground(r);
        if (ready && tiles == lastTiles && rasters == lastRasters) {
            ++still;
        } else {
            still = 0;
            stillSince = std::chrono::steady_clock::now();
        }
        settled =
            still >= 10 && std::chrono::steady_clock::now() - stillSince >= std::chrono::seconds(1);
        lastTiles = tiles;
        lastRasters = rasters;
    }
    ASSERT_TRUE(settled) << "the golden scene never settled";
    std::cout << "[terrain_golden] " << theme << " settled at frame " << frames << "\n";
    // ponytail: a machine that settles at frame 500 or later fails here instead of comparing a
    // frame of another dither phase; raise kComparedFrame (and re-promote) if the gate machine
    // ever does.
    ASSERT_LT(frames, kComparedFrame) << "settled at frame " << frames;
    for (; frames < kComparedFrame - 1; ++frames) {
        overlume::render_frame(r, pose, {frame.data(), e2e::kWidth, e2e::kHeight});
    }

    const overlume::testing::MapboxSourceStats stats = e2e::stats();
    EXPECT_EQ(stats.tiles_built, 1u);
    EXPECT_EQ(stats.terrain_tiles_built, 1u);
    EXPECT_EQ(stats.dem_requests, 1u) << "one DEM for both loaders";
    EXPECT_EQ(stats.extruded_features, 3u);
    EXPECT_EQ(stats.wall_quads, 12u);
    EXPECT_NEAR(stats.max_tile_top_m, 22.05, 0.5)
        << "the summit block's top: the DEM at its centroid (15.0 m) less z_ref (0.95 m) plus 8 m";
    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::STREAMING);
    for (const std::string& url : overlume::testing::mapbox_fixture_requested_urls(n)) {
        EXPECT_EQ(served.count(e2e::bare(url)), 1u) << "unexpected request " << e2e::bare(url);
    }

    const double ssim = overlume::testing::render_and_compare(r, pose, golden_png, actual_png);
    EXPECT_GT(ssim, overlume::testing::kSsimMin);

    // light_clay only (its fog is 40 times thinner than dark_adas's): the SSIM need not notice
    // mirrored imagery on a dim hill or a block buried in it, so three probes pin them.
    if (probe_colours) {
        ASSERT_TRUE(overlume::render_frame(r, pose, {frame.data(), e2e::kWidth, e2e::kHeight}));
        const e2e::Colour sky = e2e::colour_at_px(frame, 160, 2);
        e2e::Colour ne{}, nw{}, atSummit{};
        ASSERT_TRUE(
            e2e::colour_at(r, frame, e2e::lowered(e2e::px_to_map(a, t, 264.0, 215.0, 7.33)), &ne));
        ASSERT_TRUE(
            e2e::colour_at(r, frame, e2e::lowered(e2e::px_to_map(a, t, 248.0, 215.0, 7.33)), &nw));
        // The summit block's south wall, 4 m up from its base (14.05 m above the anchor's ground).
        ASSERT_TRUE(e2e::colour_at(
            r, frame, e2e::lowered(e2e::px_to_map(a, t, 257.75, 182.2, 18.05)), &atSummit));
        EXPECT_TRUE(e2e::looks_like(ne, e2e::kNorthEast))
            << "the hill's north-east flank must be green " << e2e::str(ne);
        EXPECT_TRUE(e2e::looks_like(nw, e2e::kNorthWest))
            << "the hill's north-west flank must be red " << e2e::str(nw);
        EXPECT_FALSE(e2e::looks_like(atSummit, e2e::kNorthEast))
            << "green where the summit block stands: it is buried in the hill "
            << e2e::str(atSummit);
        EXPECT_FALSE(e2e::near(atSummit, sky, 10.0))
            << "sky where the summit block stands " << e2e::str(atSummit);
    }
    overlume::destroy_renderer(r);
}

}

}

TEST(MapboxTerrainGolden, SyntheticHillDarkAdas) {
    terrain_golden::synthetic_hill(
        "dark_adas", OVERLUME_TEST_DATA_DIR "/tests/goldens/mapbox_terrain_synthetic_dark_adas.png",
        OVERLUME_TMP_DIR "/mapbox_terrain_synthetic_dark_adas_actual.png", false);
}

TEST(MapboxTerrainGolden, SyntheticHillLightClay) {
    terrain_golden::synthetic_hill(
        "light_clay",
        OVERLUME_TEST_DATA_DIR "/tests/goldens/mapbox_terrain_synthetic_light_clay.png",
        OVERLUME_TMP_DIR "/mapbox_terrain_synthetic_light_clay_actual.png", true);
}
