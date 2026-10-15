// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "environment_test_hooks.hpp"
#include "mapbox_glb_fixture.hpp"
#include "mapbox_mvt_fixture.hpp"
#include "mapbox_test_hooks.hpp"
#include "mapbox_tile_frame.hpp"
#include "test_paths.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace glb = overlume::testing::glb;
namespace mvt = overlume::testing::mvt;
using overlume::EnvironmentSourceState;
using overlume::mapbox::TileId;
using overlume::testing::MapboxFixtureNetwork;

constexpr const char* kToken = "pk.test-token-not-real";
// anchor_square_body: one box per z16 square; square 0's five extra features build nothing.
constexpr uint64_t kAnchorTileExtruded = 16;
// The loader's retry schedule on a frozen clock: attempt 1 at 0 s, attempt 2 no sooner than 2 s
// after it, attempt 3 no sooner than 4 s after attempt 2.
constexpr double kAttempt2AtS = 2.0;
constexpr double kAttempt3AtS = 6.0;
constexpr size_t kFrameBytes = 320u * 240u * 3u;
const overlume::CameraPose kPose{{0, -8, 3}, {0, 0, 0.5}, 60};
const std::string kTestTownDir =
    std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/environment_test_town_0";

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

// Tile math runs on use, not during static initialisation: it may lean on statics of other
// translation units.
TileId op_tile() { return overlume::mapbox::tile_containing(55.3910, 25.0803, 14); }

overlume::GeoAnchor anchor_at(const TileId& t, double u, double v) {
    const overlume::mapbox::LonLatDeg p = overlume::mapbox::tile_point_to_lonlat(t, u, v, 1.0);
    return overlume::GeoAnchor{p.lat, p.lon, 0.0, 0.0};
}

std::string url_of(const TileId& square) {
    return overlume::testing::mapbox_footprint_url(square.x, square.y);
}

// An exterior ring: positive surveyor's area in tile units (y down).
std::vector<std::array<int32_t, 2>> box(int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
    return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}

// One z16 square's body: gzip as Mapbox sends it, or raw as curl hands it over after inflating.
std::vector<uint8_t> building_tile(std::vector<mvt::Feature> features, bool gzip = true) {
    mvt::Layer layer;
    layer.name = "building";
    layer.features = std::move(features);
    const std::vector<uint8_t> raw = mvt::encode_tile({layer});
    return gzip ? mvt::gzip_stored(raw) : raw;
}

void serve_square(MapboxFixtureNetwork* n, const TileId& square,
                  std::vector<mvt::Feature> features) {
    overlume::testing::mapbox_fixture_serve(n, url_of(square), 200,
                                            building_tile(std::move(features)));
}

// Square i of a tile: one 54 m box, 10 + i m tall. Square 0 also carries an extrude=false outline,
// an underground part, a part with height <= min_height and a part with no height, which the
// filter must skip, and a part wholly in the tile buffer, which passes the filter but clips to
// nothing and so must not count. Even squares are gzip, odd ones raw: the loader must read both.
std::vector<uint8_t> anchor_square_body(size_t i) {
    std::vector<mvt::Feature> features{
        mvt::building(100 + i, {box(1000, 1000, 1400, 1400)}, 10.0 + static_cast<double>(i))};
    if (i == 0) {
        features.push_back(mvt::building(1, {box(2000, 2000, 2400, 2400)}, 30.0, 0.0, false));
        features.push_back(mvt::building(2, {box(2600, 2000, 3000, 2400)}, 30.0, 0.0, true, true));
        features.push_back(mvt::building(3, {box(2000, 2600, 2400, 3000)}, 5.0, 5.0));
        mvt::Feature unmeasured = mvt::building(4, {box(2600, 2600, 3000, 3000)}, 9.0);
        unmeasured.tags.erase(unmeasured.tags.begin());  // its `height` tag
        features.push_back(std::move(unmeasured));
        features.push_back(mvt::building(5, {box(-30, 100, -2, 200)}, 30.0));
    }
    return building_tile(std::move(features), i % 2 == 0);
}

void serve_anchor_tile(MapboxFixtureNetwork* n, const TileId& t) {
    const std::array<TileId, 16> squares = overlume::mapbox::footprint_tiles_of(t);
    for (size_t i = 0; i < squares.size(); ++i) {
        overlume::testing::mapbox_fixture_serve(n, url_of(squares[i]), 200, anchor_square_body(i));
    }
}

// Every requested URL, without its query string.
std::set<std::string> requested(MapboxFixtureNetwork* n) {
    std::set<std::string> out;
    for (const std::string& url : overlume::testing::mapbox_fixture_requested_urls(n)) {
        out.insert(url.substr(0, url.find('?')));
    }
    return out;
}

std::set<std::string> requested_mvt(MapboxFixtureNetwork* n) {
    std::set<std::string> out;
    for (const std::string& url : requested(n)) {
        if (url.find("mapbox-streets-v8") != std::string::npos) out.insert(url);
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

std::set<std::string> squares_of(const std::vector<TileId>& tiles) {
    std::set<std::string> out;
    for (const TileId& t : tiles) {
        for (const TileId& square : overlume::mapbox::footprint_tiles_of(t)) {
            out.insert(url_of(square));
        }
    }
    return out;
}

overlume::VisualRenderer* make_renderer() {
    return overlume::create_renderer(overlume::RenderConfig{320, 240, 1, kThemeDir, "dark_adas"});
}

bool install(overlume::VisualRenderer* r, const std::string& uri, const overlume::GeoAnchor& anchor,
             MapboxFixtureNetwork* n, const char* token = kToken) {
    return overlume::testing::install_fixture_mapbox_source(r, uri.c_str(), anchor, n, token);
}

void set_ego(overlume::VisualRenderer* r, double x, double y) {
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = overlume::Vec3{x, y, 0.0};
    overlume::set_scene(r, s);
}

uint64_t loaded(overlume::VisualRenderer* r) {
    return overlume::testing::environment_loaded_chunk_count(r);
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

// Renders `count` frames, letting worker-thread work land between them.
void frames(overlume::VisualRenderer* r, std::vector<uint8_t>& buf, int count) {
    pump(r, buf, count, [] { return false; });
}

// For work that finishes on a worker thread with no frame to drive it.
template <typename Done>
bool wait_until(Done done) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!done() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return done();
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

// The first baseColorFactor in a GLB's JSON chunk, read as numbers so the writer's float format
// does not matter.
std::optional<std::array<double, 4>> base_color_factor(const std::vector<uint8_t>& glb) {
    const std::string text(glb.begin(), glb.end());
    const std::string key = "\"baseColorFactor\":[";
    const size_t at = text.find(key);
    if (at == std::string::npos) return std::nullopt;
    std::array<double, 4> rgba{};
    const char* p = text.c_str() + at + key.size();
    for (double& c : rgba) {
        char* end = nullptr;
        c = std::strtod(p, &end);
        if (end == p) return std::nullopt;
        p = end + 1;  // past ',' or ']'
    }
    return rgba;
}

bool ends_with(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() &&
           s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// Freezes the loader's retry clock at 0 s for one test, which then moves time with set() instead
// of sleeping through a wait. The steady clock comes back when the test ends, however it ends.
class FrozenRetryClock {
public:
    FrozenRetryClock() { set(0.0); }
    ~FrozenRetryClock() { overlume::testing::mapbox_set_test_clock(-1.0); }
    FrozenRetryClock(const FrozenRetryClock&) = delete;
    FrozenRetryClock& operator=(const FrozenRetryClock&) = delete;
    void set(double seconds) { overlume::testing::mapbox_set_test_clock(seconds); }
};

// Square 0 answers 503 (or fails in transport) until the tile has been sent back once, then 200.
void expect_retry_then_load(bool transport) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    FrozenRetryClock clock;
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_anchor_tile(n, t);
    const std::string square0 = url_of(overlume::mapbox::footprint_tiles_of(t)[0]);
    if (transport) {
        overlume::testing::mapbox_fixture_fail_transport(n, square0);
    } else {
        overlume::testing::mapbox_fixture_serve(n, square0, 503, {});
    }
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 100, [] { return stats().retries >= 1; }))
        << "a transient answer must send the tile back for another attempt";
    overlume::testing::mapbox_fixture_serve(n, square0, 200, anchor_square_body(0));
    clock.set(kAttempt2AtS - 0.1);
    frames(r, buf, 20);
    EXPECT_EQ(times_requested(n, square0), 1u) << "no request while the retry waits";
    EXPECT_EQ(loaded(r), 0u);
    EXPECT_EQ(stats().retries, 1u) << "waiting is not another attempt";
    clock.set(kAttempt2AtS);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    EXPECT_EQ(times_requested(n, square0), 2u);
    EXPECT_EQ(stats().extruded_features, kAnchorTileExtruded)
        << "the retry must build square 0 from its 200";
    EXPECT_EQ(stats().retries, 1u) << "the second attempt found every square answered";
    EXPECT_EQ(state(r), EnvironmentSourceState::STREAMING);
    overlume::destroy_renderer(r);
}

}

namespace {

// Downtown Dubai's z14 tile, 10707/7006, where Mapbox serves landmark models.
TileId downtown_tile() { return overlume::mapbox::tile_containing(55.2744, 25.1972, 14); }

// A landmark node at (tx, ty) z14 units, with Mapbox's node scale for its own latitude.
glb::LandmarkSpec landmark_at(const TileId& t, uint64_t id, double tx, double ty) {
    glb::LandmarkSpec spec;
    spec.id = id;
    spec.tx = tx;
    spec.ty = ty;
    spec.lat_deg = overlume::mapbox::tile_point_to_lonlat(t, tx, ty, 8192.0).lat;
    return spec;
}

// Square 5 (row 1, column 1) holds its box at z16 units 1000-1400; the box centre is z14 unit
// 2048 + 1200 / 2 on both axes. Square 10 (row 2, column 2): 4096 + 1200 / 2.
constexpr double kOverSquare5 = 2648.0;
constexpr double kOverSquare10 = 4696.0;
// anchor_square_body's 16 boxes plus the neighbour serve_downtown adds to square 5.
constexpr uint64_t kDowntownFeatures = kAnchorTileExtruded + 1;

// A GLB's JSON chunk: the first chunk, after the 12-byte file header and its own 8-byte header.
std::string glb_json(const std::vector<uint8_t>& glb) {
    if (glb.size() < 20) return {};
    const size_t length = size_t{glb[12]} | (size_t{glb[13]} << 8) | (size_t{glb[14]} << 16) |
                          (size_t{glb[15]} << 24);
    if (glb.size() - 20 < length) return {};
    return std::string(glb.begin() + 20, glb.begin() + 20 + static_cast<std::ptrdiff_t>(length));
}

// The tile's squares as anchor_square_body, square 5 with a 27 m box added 81 m east of the
// landmark spot (its centroid outside the 12 m footprint), and `glb_bytes` as its landmark tile.
void serve_downtown(MapboxFixtureNetwork* n, const TileId& t,
                    const std::vector<uint8_t>& glb_bytes) {
    serve_anchor_tile(n, t);
    serve_square(n, overlume::mapbox::footprint_tiles_of(t)[5],
                 {mvt::building(105, {box(1000, 1000, 1400, 1400)}, 15.0),
                  mvt::building(205, {box(1700, 1100, 1900, 1300)}, 12.0)});
    overlume::testing::mapbox_fixture_serve(n, overlume::testing::mapbox_landmark_url(t.x, t.y),
                                            200, glb_bytes);
}

// An unusable landmark tile costs its landmarks only: every extrusion shows, no half-decoded mesh
// reaches the GLB, and one log line names the tile.
void expect_extrusions_only(const TileId& t, const std::vector<uint8_t>& glb_bytes) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_downtown(n, t, glb_bytes);
    overlume::testing::capture_stream_glbs(true);
    const size_t logFrom = log_size();
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    const std::vector<std::vector<uint8_t>> glbs = overlume::testing::captured_stream_glbs();
    overlume::testing::capture_stream_glbs(false);
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.landmark_nodes, 0u);
    EXPECT_EQ(s.conflated_features, 0u) << "no footprints come from an unusable tile";
    EXPECT_EQ(s.extruded_features, kDowntownFeatures);
    ASSERT_EQ(glbs.size(), 1u);
    EXPECT_FALSE(contains(glbs[0], "EXT_meshopt_compression")) << "a half-decoded mesh got through";
    const std::string tile = "14/" + std::to_string(t.x) + "/" + std::to_string(t.y);
    const std::string log = log_since(logFrom);
    EXPECT_EQ(count_of(log, "landmarks of tile " + tile + " skipped"), 1u);
    EXPECT_EQ(log.find("api.mapbox.com"), std::string::npos);
    overlume::destroy_renderer(r);
}

}

TEST(MapboxBuildings, UnsetTokenRefusesAndNamesTheVariable) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ScopedUnsetEnv noToken("MAPBOX_TOKEN");
    const overlume::GeoAnchor anchor = anchor_at(op_tile(), 0.5, 0.5);
    const size_t logFrom = log_size();
    EXPECT_FALSE(overlume::set_environment_source(r, "mapbox://buildings", anchor));
    EXPECT_EQ(state(r), EnvironmentSourceState::NONE);
    EXPECT_NE(log_since(logFrom).find("MAPBOX_TOKEN"), std::string::npos)
        << "the refusal must name the variable to set";

    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    EXPECT_FALSE(install(r, "mapbox://buildings", anchor, n, " \t\n"))
        << "a token of only whitespace is no token";
    set_ego(r, 0.0, 0.0);  // a source opened by mistake would now request its tile
    std::vector<uint8_t> buf(kFrameBytes);
    frames(r, buf, 3);
    EXPECT_TRUE(overlume::testing::mapbox_fixture_requested_urls(n).empty());
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, RefusedUrisOpenNothing) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ScopedUnsetEnv noToken("MAPBOX_TOKEN");  // a URI accepted by mistake still reaches no network
    const overlume::GeoAnchor anchor = anchor_at(op_tile(), 0.5, 0.5);
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    for (const char* uri : {"mapbox://roads", "mapbox://buildings?cache=tiles",
                            "mapbox://buildings?radius=-5", "mapbox://buildings?brightness=0"}) {
        const size_t logFrom = log_size();
        EXPECT_FALSE(install(r, uri, anchor, n)) << uri;
        const std::string logged = log_since(logFrom);
        EXPECT_NE(logged.find("refused"), std::string::npos) << uri;
        // Said once: the parser's reason must not repeat the loader's "refused:" prefix.
        EXPECT_EQ(logged.find("refused"), logged.rfind("refused")) << logged;
        EXPECT_FALSE(overlume::set_environment_source(r, uri, anchor)) << uri;
    }
    set_ego(r, 0.0, 0.0);  // a source opened by mistake would now request its tile
    std::vector<uint8_t> buf(kFrameBytes);
    frames(r, buf, 3);
    EXPECT_EQ(state(r), EnvironmentSourceState::NONE);
    EXPECT_TRUE(overlume::testing::mapbox_fixture_requested_urls(n).empty());
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, AnchorTileLoadsWithStandardFilterApplied) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_anchor_tile(n, t);
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n));
    EXPECT_FALSE(overlume::testing::environment_stream_has_disk_cache(r))
        << "a mapbox:// source opens no cache database, so no Mapbox byte reaches the disk";
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    EXPECT_EQ(state(r), EnvironmentSourceState::STREAMING);
    EXPECT_EQ(loaded(r), 1u);
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.tiles_built, 1u);
    EXPECT_EQ(s.extruded_features, kAnchorTileExtruded)
        << "16 boxes extrude; the extrude=false outline, the underground part, the part with "
           "height <= min_height and the part with no height must not, and the part in the tile "
           "buffer builds nothing, so it must not count";
    EXPECT_EQ(s.retries, 0u);
    EXPECT_GT(s.max_build_ms, 0.0);
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, RequestsOnlyTheTileInsideTheRadius) {
    // The -meshopt landmark variant, vertex codec v0: GL JS's -meshopt-v2-lod tiles use codec v1,
    // which the linked meshoptimizer cannot decode. Checked before the GPU skip, so it always runs.
    EXPECT_EQ(overlume::testing::mapbox_landmark_url(1, 2),
              "https://api.mapbox.com/3dtiles/v1/mapbox.mapbox-3dbuildings-v1-meshopt/14/1/2.glb");
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_anchor_tile(n, t);
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    std::set<std::string> expected = squares_of({t});
    expected.insert(overlume::testing::mapbox_landmark_url(t.x, t.y));
    EXPECT_EQ(requested(n), expected)
        << "mid-tile with the 500 m default radius, only the ego's z14 tile may be requested";
    EXPECT_EQ(overlume::testing::mapbox_fixture_requested_urls(n).size(), expected.size())
        << "each URL exactly once";
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, RadiusKeyWidensTheRequestedTiles) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);  // neighbours answer empty
    serve_anchor_tile(n, t);
    ASSERT_TRUE(install(r, "mapbox://buildings?radius=1300", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    for (int i = 0; i < 5; ++i) overlume::render_frame(r, kPose, {buf.data(), 320, 240});
    const TileId north{14, t.x, t.y - 1}, south{14, t.x, t.y + 1};
    const TileId west{14, t.x - 1, t.y}, east{14, t.x + 1, t.y};
    EXPECT_EQ(requested_mvt(n), squares_of({t, north, south, west, east}))
        << "1300 m from the tile centre reaches the four edge neighbours (about 1108 m away) but "
           "no corner neighbour (about 1566 m)";
    EXPECT_EQ(stats().tiles_built, 1u) << "neighbours answering 204 with empty MVTs add nothing";
    EXPECT_EQ(loaded(r), 1u);
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, TokenRidesOnlyInRequestUrls) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_anchor_tile(n, t);
    overlume::testing::capture_stream_glbs(true);
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    const std::vector<std::vector<uint8_t>> glbs = overlume::testing::captured_stream_glbs();
    overlume::testing::capture_stream_glbs(false);

    const std::vector<std::string> urls = overlume::testing::mapbox_fixture_requested_urls(n);
    ASSERT_FALSE(urls.empty());
    for (const std::string& url : urls) {
        EXPECT_TRUE(ends_with(url, std::string("?access_token=") + kToken))
            << url.substr(0, url.find('?'));
    }
    ASSERT_FALSE(glbs.empty());
    for (const std::vector<uint8_t>& glb : glbs) {
        EXPECT_FALSE(contains(glb, kToken)) << "a request URL reached the model";
        EXPECT_FALSE(contains(glb, "access_token"));
    }
    // The log check lives in RejectedTokenFallsBackAndLogsOnce and the ion transport-error test.
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, TokenWhitespaceIsTrimmed) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_anchor_tile(n, t);
    ASSERT_TRUE(
        install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n, " pk.test-token-not-real\n"));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    const std::vector<std::string> urls = overlume::testing::mapbox_fixture_requested_urls(n);
    ASSERT_FALSE(urls.empty());
    for (const std::string& url : urls) {
        EXPECT_TRUE(ends_with(url, "?access_token=pk.test-token-not-real"))
            << url.substr(0, url.find('?'));
        EXPECT_EQ(url.find_first_of(" \t\r\n"), std::string::npos);
    }
    // The log check lives in RejectedTokenFallsBackAndLogsOnce and the ion transport-error test.
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, RejectedTokenFallsBackAndLogsOnce) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(401);
    const size_t logFrom = log_size();
    ASSERT_TRUE(
        install(r, "mapbox://buildings?fallback=" + kTestTownDir, anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    EXPECT_TRUE(pump(r, buf, 30, [&] {
        return state(r) == EnvironmentSourceState::STREAMING_FALLBACK;
    })) << "a run of rejected requests counts as network loss";
    EXPECT_TRUE(pump(r, buf, 30, [&] { return loaded(r) > 0; })) << "the baked fallback shows";
    ASSERT_TRUE(wait_until([] { return stats().max_build_ms > 0.0; }))
        << "the rejected tile's build never finished";
    const std::string log = log_since(logFrom);
    EXPECT_EQ(count_of(log, "Mapbox rejected MAPBOX_TOKEN (HTTP 401) for streets-v8 footprints"),
              1u)
        << "16 rejected squares, one line";
    EXPECT_EQ(count_of(log, "Mapbox rejected MAPBOX_TOKEN (HTTP 401) for 3D landmarks"), 1u)
        << "one line per endpoint";
    EXPECT_EQ(log.find("api.mapbox.com"), std::string::npos) << "log lines name tiles, not URLs";
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, ServerErrorRetriesThenLoads) { expect_retry_then_load(false); }

TEST(MapboxBuildings, TransportErrorRetriesThenLoads) { expect_retry_then_load(true); }

TEST(MapboxBuildings, PersistentServerErrorGivesUpAfterThreeAttempts) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    FrozenRetryClock clock;
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_anchor_tile(n, t);
    const std::string square0 = url_of(overlume::mapbox::footprint_tiles_of(t)[0]);
    overlume::testing::mapbox_fixture_serve(n, square0, 503, {});
    const size_t logFrom = log_size();
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 100, [] { return stats().retries >= 1; }));
    clock.set(kAttempt2AtS);
    ASSERT_TRUE(pump(r, buf, 100, [] { return stats().retries >= 2; }))
        << "attempt 2 starts once the first wait is over";
    clock.set(kAttempt3AtS - 0.1);
    frames(r, buf, 20);
    EXPECT_EQ(times_requested(n, square0), 2u) << "the second wait is twice the first";
    clock.set(kAttempt3AtS);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }))
        << "after three attempts the tile shows what loaded";
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.retries, 2u) << "attempts 1 and 2 retry; attempt 3 builds";
    EXPECT_EQ(times_requested(n, square0), 3u);
    EXPECT_EQ(s.extruded_features, kAnchorTileExtruded - 1) << "square 0 never answered";
    EXPECT_EQ(count_of(log_since(logFrom),
                       "streets-v8 footprints still failing (HTTP 503) after 3 attempts"),
              1u);
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, TransportLossEverywhereFallsBack) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    for (const TileId& square : overlume::mapbox::footprint_tiles_of(t)) {
        overlume::testing::mapbox_fixture_fail_transport(n, url_of(square));
    }
    overlume::testing::mapbox_fixture_fail_transport(
        n, overlume::testing::mapbox_landmark_url(t.x, t.y));
    ASSERT_TRUE(
        install(r, "mapbox://buildings?fallback=" + kTestTownDir, anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    EXPECT_TRUE(pump(r, buf, 30, [&] {
        return state(r) == EnvironmentSourceState::STREAMING_FALLBACK;
    })) << "requests that fail in transport (no HTTP answer at all) must count as network loss";
    EXPECT_TRUE(pump(r, buf, 30, [&] { return loaded(r) > 0; }));
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, GarbageBehindOkSkipsOnlyThoseSquares) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_anchor_tile(n, t);
    const std::array<TileId, 16> squares = overlume::mapbox::footprint_tiles_of(t);
    const std::string html = "<!DOCTYPE html><html><body>Service Unavailable</body></html>";
    const std::vector<uint8_t> htmlBody(html.begin(), html.end());
    std::vector<uint8_t> truncated = anchor_square_body(2);  // an even square: gzip
    truncated.resize(truncated.size() / 2);
    overlume::testing::mapbox_fixture_serve(n, url_of(squares[0]), 200, htmlBody);
    overlume::testing::mapbox_fixture_serve(n, url_of(squares[1]), 404, {});
    overlume::testing::mapbox_fixture_serve(n, url_of(squares[2]), 200, truncated);
    overlume::testing::mapbox_fixture_serve(n, url_of(squares[3]), 400, {});
    overlume::testing::mapbox_fixture_serve(n, url_of(squares[4]), 200, htmlBody);
    const size_t logFrom = log_size();
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }))
        << "garbage in some squares must not fail the tile";
    EXPECT_EQ(state(r), EnvironmentSourceState::STREAMING);
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.extruded_features, kAnchorTileExtruded - 5) << "squares 0-4 add nothing";
    EXPECT_EQ(s.retries, 0u) << "garbage and 4xx are not transient";
    const std::string log = log_since(logFrom);
    EXPECT_EQ(count_of(log, "is not valid gzip"), 1u);
    EXPECT_EQ(count_of(log, "is not a vector tile"), 1u) << "two HTML squares, one line";
    EXPECT_EQ(count_of(log, "answered HTTP 400"), 1u);
    EXPECT_EQ(count_of(log, "HTTP 404"), 0u) << "404 is an empty square, not an error";
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, CornerLoadsFourTilesWithoutDoubledSeamWalls) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    // The ego stands on the corner where A (north-west), B (north), C (west) and D meet.
    const TileId d = op_tile();
    const TileId a{14, d.x - 1, d.y - 1}, b{14, d.x, d.y - 1}, c{14, d.x - 1, d.y};
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    using overlume::mapbox::footprint_tiles_of;
    serve_square(n, footprint_tiles_of(a)[15],
                 {mvt::building(1, {box(3600, 3600, 3900, 3900)}, 20.0)});
    serve_square(n, footprint_tiles_of(b)[12],
                 {mvt::building(2, {box(200, 3600, 500, 3900)}, 20.0)});
    // One 80 m building across the C|D seam, cut as Mapbox cuts it: each copy runs 33 units (the
    // tile buffer) into the neighbouring square.
    serve_square(n, footprint_tiles_of(c)[3],
                 {mvt::building(3, {box(3800, 400, 4129, 700)}, 20.0)});
    serve_square(n, footprint_tiles_of(d)[0], {mvt::building(3, {box(-33, 400, 300, 700)}, 20.0)});
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(d, 0.0, 0.0), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) == 4; }))
        << "500 m around a z14 corner reaches all four tiles; loaded " << loaded(r);
    EXPECT_EQ(requested_mvt(n), squares_of({a, b, c, d}));
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.extruded_features, 4u);
    EXPECT_EQ(s.wall_quads, 4u + 4u + 6u)
        << "A and B have 4 walls each. The seam building's halves have 3 each: its 4 walls, with "
           "the north and south ones cut once at the seam, and none along the seam (doubled seam "
           "walls make 8)";

    set_ego(r, -600.0, -600.0);  // heading 0: map +x is north, +y west, so 600 m south and east
    EXPECT_TRUE(pump(r, buf, 100, [&] { return loaded(r) == 1; }))
        << "600 m into D, the other three tiles must leave the scene; loaded " << loaded(r);
    EXPECT_EQ(requested_mvt(n), squares_of({a, b, c, d})) << "nothing new is requested";
    overlume::destroy_renderer(r);
}

// A smoke test; sanitizers are not in the gate. Cesium keeps the content manager, which owns the
// loader, alive until a load's main-thread continuation runs (TilesetContentManager.cpp:1255,
// :1295). Here that continuation is never dispatched, so the loader outlives the stale build and
// the loader's shared_ptr captures are defence in depth. The test pins that a switch with a tile
// load in flight neither blocks nor crashes, that the new source works, and that the stale build
// completes on the old source's worker.
TEST(MapboxBuildings, SwitchingAwayMidBuildIsSafe) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    const overlume::GeoAnchor anchor = anchor_at(t, 0.5, 0.5);
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_anchor_tile(n, t);
    const std::string held = url_of(overlume::mapbox::footprint_tiles_of(t)[0]);
    overlume::testing::mapbox_fixture_hold(n, held);
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor, n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 30, [&] { return requested(n).count(held) == 1; }));
    ASSERT_EQ(stats().tiles_built, 0u) << "the tile must still wait on the held square";

    // Tear the Mapbox source down while its tile load is in flight, then let the load finish.
    ASSERT_TRUE(overlume::set_environment_source(r, kTestTownDir.c_str(), anchor));
    overlume::testing::mapbox_fixture_release_held(n);
    EXPECT_TRUE(wait_until([] { return stats().tiles_built == 1; }))
        << "the stale build must run to completion on the old source's worker";
    EXPECT_TRUE(pump(r, buf, 60, [&] { return loaded(r) > 0; }));
    EXPECT_EQ(state(r), EnvironmentSourceState::BAKED);
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, ClayReplacesAndOriginalKeepsTheExtrusionMaterial) {
    for (const bool original : {false, true}) {
        auto* r = make_renderer();
        if (!r) GTEST_SKIP() << "no GPU/EGL";
        const TileId t = op_tile();
        auto* n = overlume::testing::make_mapbox_fixture_network(204);
        serve_anchor_tile(n, t);
        overlume::testing::capture_stream_glbs(true);
        ASSERT_TRUE(
            install(r, original ? "mapbox://buildings?materials=original" : "mapbox://buildings",
                    anchor_at(t, 0.5, 0.5), n));
        set_ego(r, 0.0, 0.0);
        std::vector<uint8_t> buf(kFrameBytes);
        ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
        const std::vector<std::vector<uint8_t>> glbs = overlume::testing::captured_stream_glbs();
        overlume::testing::capture_stream_glbs(false);
        EXPECT_EQ(overlume::testing::environment_stream_first_primitive_is_clay(r), !original)
            << (original ? "materials=original must keep the model's own material"
                         : "clay must replace it with the theme's building material");
        ASSERT_EQ(glbs.size(), 1u);
        const std::optional<std::array<double, 4>> color = base_color_factor(glbs[0]);
        ASSERT_TRUE(color.has_value()) << "the extrusion material has a base colour";
        EXPECT_NEAR((*color)[0], 0.912, 1e-9) << "Mapbox Standard's building colour, linear";
        EXPECT_NEAR((*color)[1], 0.869, 1e-9);
        EXPECT_NEAR((*color)[2], 0.787, 1e-9);
        EXPECT_EQ((*color)[3], 1.0);
        overlume::destroy_renderer(r);
    }
}

TEST(MapboxBuildings, BrightnessReachesTheStreamingSource) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_anchor_tile(n, t);
    ASSERT_TRUE(install(r, "mapbox://buildings?materials=original&brightness=1.8",
                        anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    EXPECT_DOUBLE_EQ(overlume::testing::environment_stream_brightness(r), 1.8);
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, LandmarkReplacesTheExtrusionBeneathIt) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = downtown_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_downtown(n, t,
                   glb::build_landmark_glb({landmark_at(t, 7, kOverSquare5, kOverSquare5)},
                                           glb::GlbOptions{}));
    overlume::testing::capture_stream_glbs(true);
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    const std::vector<std::vector<uint8_t>> glbs = overlume::testing::captured_stream_glbs();
    overlume::testing::capture_stream_glbs(false);
    EXPECT_EQ(loaded(r), 1u) << "landmark and extrusions share the tile's one model";
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.landmark_nodes, 1u);
    EXPECT_EQ(s.conflated_features, 1u) << "square 5's box stands under the landmark footprint";
    EXPECT_EQ(s.extruded_features, kDowntownFeatures - 1) << "its neighbour 81 m east stays";
    EXPECT_DOUBLE_EQ(s.max_tile_top_m, 100.0)
        << "the region reaches the landmark's 100 m roof, not the 25 m extrusions";
    ASSERT_EQ(glbs.size(), 1u);
    const std::string json = glb_json(glbs[0]);
    EXPECT_EQ(count_of(json, "\"mesh\":"), 2u)
        << "one landmark node and one extrusion node (only nodes have a \"mesh\" key)";
    EXPECT_NE(json.find("\"nodes\":[0,1]"), std::string::npos) << "both in the tile's one scene";
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, ClayDropsLandmarkImagesOriginalKeepsOcclusion) {
    for (const bool original : {false, true}) {
        auto* r = make_renderer();
        if (!r) GTEST_SKIP() << "no GPU/EGL";
        const TileId t = downtown_tile();
        auto* n = overlume::testing::make_mapbox_fixture_network(204);
        serve_downtown(n, t,
                       glb::build_landmark_glb({landmark_at(t, 7, kOverSquare5, kOverSquare5)},
                                               glb::GlbOptions{}));
        overlume::testing::capture_stream_glbs(true);
        ASSERT_TRUE(
            install(r, original ? "mapbox://buildings?materials=original" : "mapbox://buildings",
                    anchor_at(t, 0.5, 0.5), n));
        set_ego(r, 0.0, 0.0);
        std::vector<uint8_t> buf(kFrameBytes);
        ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
        const std::vector<std::vector<uint8_t>> glbs = overlume::testing::captured_stream_glbs();
        overlume::testing::capture_stream_glbs(false);
        EXPECT_EQ(stats().landmark_nodes, 1u);
        ASSERT_EQ(glbs.size(), 1u);
        EXPECT_EQ(contains(glbs[0], "\"images\""), original)
            << (original ? "original keeps the landmark's JPEG" : "clay drops every image");
        EXPECT_EQ(contains(glbs[0], "\"occlusionTexture\""), original);
        overlume::destroy_renderer(r);
    }
}

TEST(MapboxBuildings, CorruptMeshoptKeepsExtrusionsAndLogsTheTileOnce) {
    const TileId t = downtown_tile();
    glb::GlbOptions opts;
    opts.corrupt_meshopt = true;
    expect_extrusions_only(
        t, glb::build_landmark_glb({landmark_at(t, 7, kOverSquare5, kOverSquare5)}, opts));
}

TEST(MapboxBuildings, BadGlbHeaderKeepsExtrusionsAndLogsTheTileOnce) {
    const TileId t = downtown_tile();
    std::vector<uint8_t> bytes =
        glb::build_landmark_glb({landmark_at(t, 7, kOverSquare5, kOverSquare5)}, glb::GlbOptions{});
    bytes[0] = 'x';  // the "glTF" magic, broken
    expect_extrusions_only(t, bytes);
}

TEST(MapboxBuildings, UnexpectedNodeMatrixSkipsOnlyThatLandmark) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = downtown_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    glb::GlbOptions opts;
    opts.bad_matrix_on_first = true;
    // Landmark 6, whose building node the fixture rotates, stands over square 10's box; landmark 7
    // over square 5's. Conflation uses every footprint the reader returns. In Mapbox's layout a
    // footprint node carries its building's matrix, so a matrix that skips a building skips its
    // footprint too and the extrusion beneath stays. The fixture rotates only the building node:
    // landmark 6's footprint survives, square 10's box goes, and nothing stands at that spot.
    serve_downtown(n, t,
                   glb::build_landmark_glb({landmark_at(t, 6, kOverSquare10, kOverSquare10),
                                            landmark_at(t, 7, kOverSquare5, kOverSquare5)},
                                           opts));
    const size_t logFrom = log_size();
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.landmark_nodes, 1u) << "landmark 7 is kept";
    EXPECT_EQ(s.conflated_features, 2u)
        << "square 5's box under landmark 7, square 10's under landmark 6's surviving footprint";
    EXPECT_EQ(s.extruded_features, kDowntownFeatures - 2);
    EXPECT_EQ(count_of(log_since(logFrom), "with an unexpected matrix skipped"), 1u);
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, LandmarksAloneStillBuildTheTile) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = downtown_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    overlume::testing::mapbox_fixture_serve(
        n, overlume::testing::mapbox_landmark_url(t.x, t.y), 200,
        glb::build_landmark_glb({landmark_at(t, 7, kOverSquare5, kOverSquare5)},
                                glb::GlbOptions{}));
    overlume::testing::capture_stream_glbs(true);
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    const std::vector<std::vector<uint8_t>> glbs = overlume::testing::captured_stream_glbs();
    overlume::testing::capture_stream_glbs(false);
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.tiles_built, 1u);
    EXPECT_EQ(s.landmark_nodes, 1u);
    EXPECT_EQ(s.extruded_features, 0u);
    EXPECT_EQ(s.conflated_features, 0u);
    ASSERT_EQ(glbs.size(), 1u);
    EXPECT_EQ(count_of(glb_json(glbs[0]), "\"mesh\":"), 1u) << "the landmark node alone";
    overlume::destroy_renderer(r);
}
