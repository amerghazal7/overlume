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

// Declarations only (no *_IMPLEMENTATION macro) -- golden.cpp already
// defines STB_IMAGE_WRITE_IMPLEMENTATION and is linked into every test
// binary as an extra source (CMakeLists.txt's `_golden_cpp`), so this just
// gets us stbi_write_png's prototype for the capture tests below, same
// one-implementation-many-includers shape stb itself is designed for.
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
#include <string>
#include <thread>
#include <vector>

namespace {

const std::string kTestTownDir =
    std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/environment_test_town_0";
const std::string kIonFixtureDir =
    std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/environment_ion_fixture_0";
// VM-063 (Task 4): a dedicated, synthetic 16-real-tile fixture for the
// network-loss e2e -- see its own PROVENANCE.md for why
// environment_ion_fixture_0's 3 tiles cannot produce
// kNetworkLossConsecutiveFailures (8) distinct failing requests (a
// succeeded/cached tile can't be forced to fail again inside one short
// test process; a failed tile isn't auto-retried without a fresh
// unload/redesire cycle) and why 16 real (duplicated) tiles fixes that.
const std::string kIonFixtureFallbackDir =
    std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/environment_ion_fixture_fallback_0";

constexpr mpviz::Vec3 kChunk0Center{-128.0, -128.0, 0.0};

// PROVENANCE.md: the anchor lies inside tile_root.b3dm's own region, so the
// map-frame block center IS the map origin, by WgsToMap's own definition.
constexpr mpviz::Vec3 kFixtureBlockCenterMap{0.0, 0.0, 0.0};
constexpr mpviz::GeoAnchor kFixtureAnchor{25.0803, 55.3910, 0.0};

mpviz::CameraPose kStdPose{{0, -8, 3}, {0, 0, 0.5}, 60};

// VM-064 gate round 1 finding: two tests below unsetenv("CESIUM_ION_TOKEN")
// to force the no-token path -- previously for the WHOLE PROCESS, so
// GooglePresetLiveRenderMsDeltaVsOsmClay's opt-in skip condition
// (getenv(...) == nullptr) could never see a caller-provided token in a
// full-binary run if gtest happened to order either of these first (default
// declaration order does). RAII save/restore removes the ordering
// dependency: each test's own env edit undoes itself on scope exit,
// success or failure.
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
        if (had_value_) ::setenv(name_, saved_.c_str(), 1);
        else ::unsetenv(name_);
    }
    ScopedUnsetEnv(const ScopedUnsetEnv&) = delete;
    ScopedUnsetEnv& operator=(const ScopedUnsetEnv&) = delete;

private:
    const char* name_;
    bool had_value_ = false;
    std::string saved_;
};

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
    ScopedUnsetEnv no_token("CESIUM_ION_TOKEN");  // this test binary's env only, restored on scope exit
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

// ── set_environment_visible() (VM-096): the streaming backend must
//    honor the same hide/show contract as BakedEnvironmentSource -- the
//    GUI's environment_enabled toggle has to work with EVERY preset
//    (baked/osm/clipped/google), not just the baked default. ────────────
//
// Asserted via environment_scene_membership_count() (environment_test_hooks.hpp),
// not a rendered-pixel diff: this fixture's 3 tiles cover only a small
// corner of FixtureBlock_DarkAdas's own {149,352,90}/60deg framing, so a
// whole-frame pixel/SSIM comparison can't tell "hidden" apart from ordinary
// frame-to-frame Cesium LOD-refinement noise with any real margin (measured
// directly while developing this test: a same-pose before/after diff was
// the SAME order of magnitude whether the source was actually toggled or
// not). The scene-membership hook instead asserts the exact invariant
// set_visible() is supposed to maintain -- deterministic, camera-framing
// independent, and it also proves loaded_count() (still-loaded tiles) is
// untouched by hiding, same as BakedEnvironmentSource's own tests.
TEST(EnvironmentStreamGolden, SetVisibleFalseHidesFixtureTilesWithoutTearingDown) {
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
    ASSERT_GT(pump_until_loaded(r, kStdPose, buf), 0u);
    for (int i = 0; i < 30; ++i) mpviz::render_frame(r, kStdPose, {buf.data(), 320, 240});
    const uint64_t loadedBefore = mpviz::testing::environment_loaded_chunk_count(r);
    ASSERT_GT(loadedBefore, 0u);
    ASSERT_EQ(mpviz::testing::environment_scene_membership_count(r), loadedBefore)
        << "every loaded tile should start out actually in the scene";

    ASSERT_TRUE(mpviz::set_environment_visible(r, false));
    ASSERT_TRUE(mpviz::render_frame(r, kStdPose, {buf.data(), 320, 240}));
    // Not torn down: still "loaded" (a re-show must not need a re-fetch),
    // but zero of it is actually in the Filament scene right now.
    EXPECT_EQ(mpviz::testing::environment_loaded_chunk_count(r), loadedBefore);
    EXPECT_EQ(mpviz::testing::environment_scene_membership_count(r), 0u)
        << "streamed tiles are still in the Filament scene after "
           "set_environment_visible(r, false)";

    ASSERT_TRUE(mpviz::set_environment_visible(r, true));
    ASSERT_TRUE(mpviz::render_frame(r, kStdPose, {buf.data(), 320, 240}));
    EXPECT_EQ(mpviz::testing::environment_loaded_chunk_count(r), loadedBefore);
    EXPECT_EQ(mpviz::testing::environment_scene_membership_count(r), loadedBefore)
        << "re-showing did not put every already-loaded tile back into the scene";
    mpviz::destroy_renderer(r);
}

namespace {
// Same per-channel tolerance test_map_elements.cpp's own PixelDiffers() uses
// for "real content change" vs "rendering noise" -- measured directly while
// writing TileLoadedWhileHiddenDoesNotPopIntoView below: two independent
// FEngine instances rendering the SAME scene/camera/theme are NOT
// byte-identical (a +-1-level cross-instance rendering difference, seen
// uniformly across an otherwise-flat sky region), so an exact byte compare
// over a whole frame that's mostly flat sky (this fixture's tiles cover
// only a small corner of this pose, same as
// EnvironmentStreamGolden.SetVisibleFalseHidesFixtureTilesWithoutTearingDown's
// own comment notes) reads as a large false "difference". 8 filters that
// noise out while still catching a real popped-in tile (a large, structured
// color change, not a +-1 flicker).
size_t count_differing_bytes_with_tolerance(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b,
                                             int tolerance) {
    size_t diff = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::abs(static_cast<int>(a[i]) - static_cast<int>(b[i])) > tolerance) ++diff;
    }
    return diff;
}
}  // namespace

// Finding #8: the streaming-side counterpart to
// Environment.ChunkLoadedWhileHiddenDoesNotPopIntoView (test_environment.cpp)
// -- hides BEFORE any tile has ever loaded, so a bug in
// StreamingEnvironmentSource::synthesize_view_and_pump()'s own
// `if (visible_ && inScene_.find(res) == inScene_.end())` add-while-hidden
// guard (environment_stream.cpp) would actually be caught.
// environment_scene_membership_count() (Finding #1,
// StreamingEnvironmentSource::scene_membership_count() in environment_stream.cpp)
// now does a genuine filament::Scene::hasEntity() read-back per tracked
// tile, not a re-derivation from visible_, so the EXPECT_EQ(...,0u) below is
// the load-bearing assertion for this test -- it fails if the guard is
// deleted and a hidden-load's entity actually lands in the Filament scene.
// The pixel comparison further down is a second, independent cross-check.
TEST(EnvironmentStreamGolden, TileLoadedWhileHiddenDoesNotPopIntoView) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(
        mpviz::testing::install_fixture_streaming_source(r, kIonFixtureDir.c_str(), kFixtureAnchor));
    ASSERT_TRUE(mpviz::set_environment_visible(r, false));

    mpviz::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    mpviz::set_scene(r, s);
    // Same framing as FixtureBlock_DarkAdas -- the fixture's real footprint
    // centroid, so a popped-in tile would occupy a real chunk of the frame.
    mpviz::CameraPose pose{{149, 352, 90}, {69, 472, 0}, 60.0};
    const size_t nBytes = 320u * 240u * 3u;
    std::vector<uint8_t> hiddenBuf(nBytes);
    ASSERT_GT(pump_until_loaded(r, pose, hiddenBuf), 0u) << "loading itself must still happen while hidden";
    for (int i = 0; i < 30; ++i) mpviz::render_frame(r, pose, {hiddenBuf.data(), 320, 240});
    ASSERT_TRUE(mpviz::render_frame(r, pose, {hiddenBuf.data(), 320, 240}));
    EXPECT_EQ(mpviz::testing::environment_scene_membership_count(r), 0u)
        << "a tile loaded while hidden was added to the Filament scene anyway";

    // Reference: a SECOND renderer with no environment source at all -- the
    // ground truth for "tiles not rendered" (same technique
    // test_environment.cpp's own sibling test uses).
    auto* rNoEnv = mpviz::create_renderer(cfg);
    ASSERT_NE(rNoEnv, nullptr);
    mpviz::set_scene(rNoEnv, s);
    std::vector<uint8_t> noEnvBuf(nBytes);
    ASSERT_TRUE(mpviz::render_frame(rNoEnv, pose, {noEnvBuf.data(), 320, 240}));
    mpviz::destroy_renderer(rNoEnv);

    // Measured on this fixture/framing: the tolerance-filtered noise floor
    // between a hidden-with-loaded-tiles renderer and one with no
    // environment source at all is ~0 bytes; the real pop-into-view signal
    // (this same comparison, hidden vs. re-shown on the SAME renderer, see
    // diffShown below) is ~326 bytes. 100 sits well under that signal while
    // leaving headroom over the noise floor, so this bound still fails if
    // the add-while-hidden guard is deleted -- it is a coarse sanity check;
    // the real guard is the membership EXPECT_EQ above.
    constexpr int kTolerance = 8;
    const size_t diffFromNoEnv = count_differing_bytes_with_tolerance(hiddenBuf, noEnvBuf, kTolerance);
    EXPECT_LT(diffFromNoEnv, 100u)
        << "a tile loaded while hidden popped into view (" << diffFromNoEnv << "/" << nBytes
        << " bytes differ beyond rendering noise from a renderer with no environment source at all)";

    ASSERT_TRUE(mpviz::set_environment_visible(r, true));
    std::vector<uint8_t> shownBuf(nBytes);
    ASSERT_TRUE(mpviz::render_frame(r, pose, {shownBuf.data(), 320, 240}));
    const size_t diffShown = count_differing_bytes_with_tolerance(hiddenBuf, shownBuf, kTolerance);
    // A small floor, not the ~5% SetEnvironmentVisibleFalseHidesLoadedChunksWithoutTearingDown
    // uses for the BAKED town's own centroid framing: this fixture's tiles
    // cover only a small corner of THIS pose (this file's own comment on
    // SetVisibleFalseHidesFixtureTilesWithoutTearingDown), so a re-show only
    // flips a modest, tolerance-filtered ~326 bytes at 320x240 -- a real
    // signal (this same tolerance already proved hiddenBuf indistinguishable
    // from a renderer with no source above), just a small one.
    EXPECT_GT(diffShown, 50u) << "showing again produced no visible change (only " << diffShown << "/"
                               << nBytes << " bytes changed beyond rendering noise)";
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


// ── Finding #0 (security, blocking): CESIUM_ION_TOKEN redaction ─────────
// Drives the real ion-handshake error path (no network, no real token --
// see drive_ion_token_redaction_probe()'s own comment) with a bogus literal
// standing in for the token, and asserts the library's own log output never
// carries it, nor an un-redacted access_token=/Bearer credential shape.
TEST(TokenRedaction, NeverLeaksIntoLogText) {
    constexpr const char* kBogusToken = "bogus-token-for-redaction-test";
    ASSERT_TRUE(mpviz::testing::drive_ion_token_redaction_probe(kBogusToken, /*asset_id=*/123456,
                                                                  /*max_ticks=*/200))
        << "the probe never logged anything at all -- test infrastructure issue, not a pass";

    const std::string logText = mpviz::testing::captured_cesium_log_text();
    ASSERT_NE(logText.find("access_token="), std::string::npos)
        << "probe never logged the ion endpoint URL -- redaction path was never exercised";
    // Failure text prints a bounded window around the offending match, never
    // the whole captured buffer: the capture is process-wide and the opt-in
    // EnvSourceCapture tests open REAL ion sources through the same sink, so
    // dumping it all on a redaction regression could print a live credential
    // into test output.
    const auto leakPos = logText.find(kBogusToken);
    EXPECT_EQ(leakPos, std::string::npos)
        << "the bogus token leaked verbatim into this library's own log output near: "
        << (leakPos == std::string::npos ? std::string() : logText.substr(leakPos, 48));

    // access_token= must never be followed by anything but the literal
    // "<redacted>" -- catches a partial/off-by-one redaction, not just the
    // literal token string above.
    size_t pos = 0;
    while ((pos = logText.find("access_token=", pos)) != std::string::npos) {
        const std::string rest = logText.substr(pos + std::strlen("access_token="));
        EXPECT_EQ(rest.rfind("<redacted>", 0), 0u)
            << "access_token= was followed by something other than <redacted> ("
            << rest.size() << " chars follow; content withheld -- it may be a credential)";
        pos += std::strlen("access_token=");
    }
}

// ── Task 4 (VM-063): network-loss fallback e2e ──────────────────────────
namespace {

// Pumps until `stop_state` is observed or `max_ticks` pass, asserting every
// tick's render_frame() still succeeds throughout (spec §9, never a crash).
// Ego stays put at kFixtureBlockCenterMap the whole time -- with
// environment_ion_fixture_fallback_0's 16 same-region tiles, the killed
// accessor produces >= kNetworkLossConsecutiveFailures within the first
// couple of ticks without any ego motion (unlike the original 3-tile
// fixture, where a stationary ego issues no NEW tile requests once
// whatever's already resident is loaded).
void pump_until_state(mpviz::VisualRenderer* r, const mpviz::CameraPose& pose,
                       std::vector<uint8_t>& buf, mpviz::EnvironmentSourceState stop_state,
                       int max_ticks = 30) {
    for (int i = 0; i < max_ticks; ++i) {
        ASSERT_TRUE(mpviz::render_frame(r, pose, {buf.data(), 320, 240}));
        if (mpviz::environment_source_state(r) == stop_state) return;
    }
}

}  // namespace

// VM-063 gate round 1, Finding 1: this test's real name is
// NetworkDeadFromFirstRequestFallsBackToBakedChunksOnce, not
// "...FallsBack...Once" read as "kills a live stream" -- reproduced
// empirically (not assumed) that a genuinely mid-stream kill is NOT
// reachable with this fixture: patching this test to pump_until_loaded()
// first (real STREAMING with a non-empty inScene_), THEN kill, THEN pump
// (tried up to 2000 ticks) never transitions to STREAMING_FALLBACK, because
// pump_until_loaded()'s very first non-zero read already observes all 16
// tiles loaded (loaded_chunk_count == 16 at that point, not 1) -- with only
// 2 worker threads racing 16 near-instant local-file requests from the same
// anchor, "first tile visible on the main thread" and "all 16 succeeded"
// are, empirically, the same tick. There is no fixture-local timing window
// between "some tiles streamed in" and "all of them did" to kill into. So
// this test (and its e2e/negative sibling below) exercises Decision 11's
// OTHER real regime: the network is already dead before the first content
// request ever completes (the root tileset.json manifest is deliberately
// exempted from the kill switch, per environment_ion_fixture_fallback_0's
// own PROVENANCE.md, modeling "resolved once at startup, while healthy").
// **Gap, named rather than silently dropped:** fallback from a truly live
// STREAMING state with resident streamed assets -- and therefore the
// teardown discipline (tearing down live Filament assets, not an
// already-empty inScene_) under fallback -- has NO test coverage here. A
// file fixture cannot produce it (this task's own empirical trail above);
// closing the gap needs either a second, geographically-separate group of
// never-cached tiles that only become desired after real ego motion (so
// they're still unrequested when the kill lands), or an accessor with
// injectable per-request latency -- both a fixture/harness change beyond
// this round's scope, recorded here and in the Task 4 Step 1 plan ledger
// entry rather than implied-but-untested.
TEST(EnvironmentStream, NetworkDeadFromFirstRequestFallsBackToBakedChunksOnce) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // Fixture source with a kill switch, fallback pointed at Epic 4's
    // committed baked test town (the REAL fallback content path, not a
    // stub) -- kFixtureBlockCenterMap sits within kLoadRadiusM of the
    // town's own chunk_-1_-1 (center (-128,-128,0), radius_m ~181 < 300),
    // so the fallback loads real chunks with no ego motion needed.
    auto* killable = mpviz::testing::install_fixture_streaming_source_with_fallback(
        r, kIonFixtureFallbackDir.c_str(), kTestTownDir.c_str(), kFixtureAnchor);
    ASSERT_NE(killable, nullptr);

    mpviz::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    mpviz::set_scene(r, s);

    // Phase 1: healthy -- construction alone reports STREAMING ("network
    // healthy (or untested)", scene.h's own Interfaces comment) before a
    // single tick has run. Nothing has streamed in yet (inScene_ is empty)
    // -- see the test-level comment above for why this test cannot start
    // from a genuinely loaded STREAMING state.
    EXPECT_EQ(mpviz::environment_source_state(r), mpviz::EnvironmentSourceState::STREAMING);

    // Phase 2: kill the network before any tile has ever been requested;
    // every one of the fixture's 16 real tiles is then a first-ever,
    // never-cached request that fails once the killed accessor is hit --
    // real HTTP-shaped failures through the SAME CountingAssetAccessor ->
    // CachingAssetAccessor(SqliteCache) stack production traffic runs, not
    // a mocked counter.
    std::vector<uint8_t> buf(320u * 240u * 3u);
    mpviz::testing::kill_fixture_network(killable);
    pump_until_state(r, kStdPose, buf, mpviz::EnvironmentSourceState::STREAMING_FALLBACK);

    // Fallback declared after kNetworkLossConsecutiveFailures failed requests:
    EXPECT_EQ(mpviz::environment_source_state(r), mpviz::EnvironmentSourceState::STREAMING_FALLBACK);
    // AC: "baked chunks appear" -- the count now reports the BAKED town's
    // chunks. NOTE: this does NOT prove the streamed tiles were "torn down,
    // not orphaned" -- none were ever resident (no tile ever succeeded in
    // this test), so there was nothing to tear down. That teardown-under-
    // fallback claim is the named gap in the comment above, not covered
    // here.
    EXPECT_GT(mpviz::testing::environment_loaded_chunk_count(r), 0u);

    // The "Once" in this test's name, asserted rather than left to review:
    // Decision 11 makes the switch ONE-WAY for the process's life. Revive
    // the fixture "network" and pump well past the tick count the original
    // transition needed -- the source must NOT resume streaming.
    mpviz::testing::revive_fixture_network(killable);
    for (int i = 0; i < 10; ++i) mpviz::render_frame(r, kStdPose, {buf.data(), 320, 240});
    EXPECT_EQ(mpviz::environment_source_state(r), mpviz::EnvironmentSourceState::STREAMING_FALLBACK)
        << "a revived network must not un-do the fallback (Decision 11: one-way)";
    EXPECT_GT(mpviz::testing::environment_loaded_chunk_count(r), 0u)
        << "the baked chunks must stay resident after the network returns";
    mpviz::destroy_renderer(r);
}

// Negative path (Decision 11): no &fallback= given -> state still
// transitions to STREAMING_FALLBACK, loaded_chunk_count stays at 0, no
// crash, render_frame keeps returning true (spec §9 all the way down).
TEST(EnvironmentStream, NetworkLossWithNoFallbackDirStillTransitionsAndStaysEmpty) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    auto* killable = mpviz::testing::install_fixture_streaming_source_with_fallback(
        r, kIonFixtureFallbackDir.c_str(), /*fallback_baked_dir=*/nullptr, kFixtureAnchor);
    ASSERT_NE(killable, nullptr);

    mpviz::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    mpviz::set_scene(r, s);

    std::vector<uint8_t> buf(320u * 240u * 3u);
    mpviz::testing::kill_fixture_network(killable);
    pump_until_state(r, kStdPose, buf, mpviz::EnvironmentSourceState::STREAMING_FALLBACK);

    EXPECT_EQ(mpviz::environment_source_state(r), mpviz::EnvironmentSourceState::STREAMING_FALLBACK);
    EXPECT_EQ(mpviz::testing::environment_loaded_chunk_count(r), 0u);
    EXPECT_TRUE(mpviz::render_frame(r, kStdPose, {buf.data(), 320, 240}));
    mpviz::destroy_renderer(r);
}

// ── Task 5 (VM-064): original-materials mode ────────────────────────────
// Step 0: materials_original parses and is mirrored by the hook (via the
// fixture install hook's own materials_original param -- see that hook's
// header comment for why this is how the test exercises Decision 5's
// materials= key rather than a real ion:// URI: the fixture path never
// goes through parse_ion_spec() at all, so a dedicated bool param is the
// TDD seam; parse_ion_spec()'s own "unknown value -> WARN once, clay"
// branch is exercised by IonUriWithoutTokenIsNonFatalFalse's sibling below,
// which DOES go through the real string parser via the public ion://
// dispatch).
TEST(EnvironmentStream, MaterialsOriginalDefaultsFalse) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(
        mpviz::testing::install_fixture_streaming_source(r, kIonFixtureDir.c_str(), kFixtureAnchor));
    EXPECT_FALSE(mpviz::testing::environment_stream_materials_original(r));
    mpviz::destroy_renderer(r);
}

TEST(EnvironmentStream, MaterialsOriginalTrueIsMirroredByHook) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(mpviz::testing::install_fixture_streaming_source(
        r, kIonFixtureDir.c_str(), kFixtureAnchor, /*materials_original=*/true));
    EXPECT_TRUE(mpviz::testing::environment_stream_materials_original(r));
    mpviz::destroy_renderer(r);
}

// VM-064 gate round 1 finding: the two tests above both reach
// materials_original via install_fixture_streaming_source()'s own bool
// param, which bypasses parse_ion_spec() entirely -- neither one would
// notice a typo in the parser's key match (e.g. "material" instead of
// "materials"). This calls parse_ion_spec() directly (via the
// environment_stream_parse_materials_original() hook) on the literal
// ion:// query string, no renderer/GPU needed.
TEST(EnvironmentStream, ParseMaterialsOriginalRecognizesTheRealKeyAndValue) {
    EXPECT_TRUE(mpviz::testing::environment_stream_parse_materials_original("96188?materials=original"));
    EXPECT_FALSE(mpviz::testing::environment_stream_parse_materials_original("96188"));
    EXPECT_FALSE(mpviz::testing::environment_stream_parse_materials_original("96188?materials=clay"));
    EXPECT_FALSE(mpviz::testing::environment_stream_parse_materials_original("96188?materials=bogus"));
}

// The real string-parser path (Decision 5's parse_ion_spec()), through the
// public ion:// dispatch -- no token, so this only pins that an unknown/
// absent materials= key never crashes the parse (spec §9); the true-token
// case can't run in ctest (network discipline), so parse_ion_spec()'s
// "unknown value -> WARN once, clay" branch itself is covered by this same
// non-fatal-false shape: any materials= value (or none) on a URI with no
// token still returns false, never crashes.
TEST(EnvironmentStream, UnknownMaterialsValueOnIonUriIsNonFatalFalse) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ScopedUnsetEnv no_token("CESIUM_ION_TOKEN");
    mpviz::GeoAnchor a{25.0803, 55.3910, 0.0};
    EXPECT_FALSE(mpviz::set_environment_source(r, "ion://96188?materials=bogus", a));
    std::vector<uint8_t> buf(320u * 240u * 3u);
    EXPECT_TRUE(mpviz::render_frame(r, kStdPose, {buf.data(), 320, 240}));
    mpviz::destroy_renderer(r);
}

// ── Step 1: original mode skips the clay remap ──────────────────────────
TEST(EnvironmentStream, ClayModeRemapsFirstPrimitiveToBuildingMaterial) {
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
    ASSERT_GT(pump_until_loaded(r, kStdPose, buf), 0u);
    EXPECT_TRUE(mpviz::testing::environment_stream_first_primitive_is_clay(r));
    mpviz::destroy_renderer(r);
}

TEST(EnvironmentStream, OriginalModeSkipsClayRemapOnFirstPrimitive) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(mpviz::testing::install_fixture_streaming_source(
        r, kIonFixtureDir.c_str(), kFixtureAnchor, /*materials_original=*/true));
    mpviz::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    mpviz::set_scene(r, s);
    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GT(pump_until_loaded(r, kStdPose, buf), 0u);
    EXPECT_FALSE(mpviz::testing::environment_stream_first_primitive_is_clay(r));
    // Fresh-opaque convention unaffected: original mode still casts/receives
    // shadows via the SAME renderable-manager calls (unconditional in
    // prepareInMainThread) -- nothing fade-blended is introduced here.
    mpviz::destroy_renderer(r);
}

// ── Task 5 Step 4: live perf, google preset vs OSM-clay preset ──────────
// Opt-in and self-skipping BY DESIGN (Global Constraints' network
// discipline: "no ctest/gtest ever requires live network or the token") --
// this test SKIPs cleanly whenever CESIUM_ION_TOKEN is unset OR the
// separate MPVIZ_LIVE_ION_PERF opt-in is unset, so a normal ctest/
// ci_visual_mode.sh run (neither set) never depends on either. VM-064 gate
// round 1 finding: this used to be unrunnable even with both vars set in a
// full-binary run -- two earlier tests unsetenv("CESIUM_ION_TOKEN") for the
// whole process (now fixed via ScopedUnsetEnv's save/restore above), so a
// full run with both vars exported now reaches this test with the token
// still present. --gtest_filter stays the recommended way to run it in
// isolation (deliberate opt-in, not a workaround for that bug):
// (`CESIUM_ION_TOKEN=... MPVIZ_LIVE_ION_PERF=1
// ./test_environment_stream --gtest_filter='*GooglePresetLive*'`) to
// record the real dev-box number this task's results block wants.
TEST(EnvironmentStreamPerf, GooglePresetLiveRenderMsDeltaVsOsmClay) {
    if (std::getenv("CESIUM_ION_TOKEN") == nullptr || std::getenv("MPVIZ_LIVE_ION_PERF") == nullptr) {
        GTEST_SKIP() << "opt-in live-network perf check -- set CESIUM_ION_TOKEN and "
                        "MPVIZ_LIVE_ION_PERF=1 to run (never required by ctest)";
    }
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    mpviz::CameraPose pose{{0, -300, 300}, {0, 0, 0}, 60.0};
    std::vector<uint8_t> buf(320u * 240u * 3u);
    mpviz::GeoAnchor anchor{25.0803, 55.3910, 0.0};  // Epic 4 Decision 6's verified fix location

    // Bounded by wall-clock, not just frame count -- a live network call
    // that never resolves must not hang this opt-in run indefinitely.
    auto pump_live = [&](const char* source_uri, std::vector<uint8_t>& out_buf) -> uint64_t {
        auto* r = mpviz::create_renderer(cfg);
        if (!r) return 0;
        if (!mpviz::set_environment_source(r, source_uri, anchor)) {
            mpviz::destroy_renderer(r);
            return 0;
        }
        mpviz::SceneGraph s{};
        s.ego.valid = 1;
        s.ego.position = mpviz::Vec3{0, 0, 0};
        mpviz::set_scene(r, s);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
        uint64_t loaded = 0;
        while (std::chrono::steady_clock::now() < deadline) {
            if (!mpviz::render_frame(r, pose, {out_buf.data(), 320, 240})) break;
            loaded = mpviz::testing::environment_loaded_chunk_count(r);
            if (loaded > 0) break;
        }
        if (loaded == 0) {
            mpviz::destroy_renderer(r);
            return 0;
        }
        double worstMs = 0.0;
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < 60; ++i) {
            const auto f0 = std::chrono::steady_clock::now();
            mpviz::render_frame(r, pose, {out_buf.data(), 320, 240});
            worstMs = std::max(
                worstMs, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - f0)
                             .count());
        }
        const double meanMs =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / 60.0;
        std::cerr << "[EnvironmentStreamPerf/live] source='" << source_uri << "' loaded=" << loaded
                  << " mean_render_ms=" << meanMs << " worst_single_frame_ms=" << worstMs << "\n";
        // Manual visual-confidence artifact only (not a golden -- no
        // committed comparison target, ssim result discarded): lets a human
        // eyeball whether original-materials mode actually shows textured
        // content vs. a blank/untextured mesh. Never asserted on.
        mpviz::testing::render_and_compare(r, pose, "/nonexistent_no_golden.png",
                                            "/tmp/environment_stream_live_actual.png");
        mpviz::destroy_renderer(r);
        return loaded;
    };

    std::vector<uint8_t> clayBuf(320u * 240u * 3u);
    const uint64_t clayLoaded = pump_live("ion://96188", clayBuf);
    const uint64_t googleLoaded = pump_live("ion://2275207?materials=original&cache=off", buf);
    std::cerr << "[EnvironmentStreamPerf/live] clay(96188) loaded=" << clayLoaded
              << " google(2275207,original) loaded=" << googleLoaded << "\n";
    SUCCEED();
}

// ── VM-096 comparison package: opt-in env-source capture tests ──────────
// One human-eyeball render per GUI environment preset ("baked"/"osm"/
// "clipped"/"google", VM-096), all at the SAME geo anchor (kFixtureAnchor,
// already this file's own anchor constant above) and the SAME camera pose,
// so a side-by-side comparison actually compares the sources and nothing
// else. Gated on MPVIZ_CAPTURE_ENV_SOURCES=1, same opt-in shape as
// EnvironmentStreamPerf.GooglePresetLiveRenderMsDeltaVsOsmClay above --
// never required by ctest/ci_visual_mode.sh. osm/google additionally need
// CESIUM_ION_TOKEN (real live ion network); baked needs neither (the
// committed offline fixture town).
namespace {

// The BAKED fixture's own real footprint centroid -- test_environment.cpp's
// kBuildingsCentroid (that file's own comment: "measured directly from the
// fixture .glb," not re-derived here, just re-typed the same way
// test_environment_stream.cpp's EcefToMapAgreesWithCppPinWithinHalfMeter
// re-types its fixture's literals rather than re-deriving them). Chosen as
// the ONE shared pose target because it's the only map-frame point that is
// GUARANTEED non-empty for the baked source (it's the baked town's own
// buildings) while ALSO being real, renderable ground truth for the live
// ion sources: kFixtureAnchor sits in Dubai, and Cesium OSM
// Buildings/Google Photorealistic 3D Tiles both have real-world coverage
// there, so a point ~113 m from the anchor still has genuine tile content
// to load -- unlike an arbitrary far-off point that might land on empty
// ocean/desert for the live sources even though the baked fixture (which
// only exists at this one location) would trivially still show its town.
constexpr mpviz::Vec3 kCaptureBuildingsCentroid{-109.2, -17.1, 3.0};
// Same eye/target offset test_environment.cpp's
// SetEnvironmentVisibleFalseHidesLoadedChunksWithoutTearingDown already
// uses to frame that exact centroid (+50/-70/40 eye offset, its own
// comment: "frames the real footprint centroid so the buildings occupy a
// real chunk of the image, not a corner") -- reused verbatim, not
// re-derived, as the shared pose every source below is captured from.
const mpviz::CameraPose kCapturePose{
    {kCaptureBuildingsCentroid.x + 50, kCaptureBuildingsCentroid.y - 70, 40},
    {kCaptureBuildingsCentroid.x, kCaptureBuildingsCentroid.y, kCaptureBuildingsCentroid.z},
    60.0};

// Generous size (per the task): these PNGs are for a human to look at side
// by side, not for SSIM -- golden.hpp's render_and_compare() is fixed at
// 320x240 for CI speed and is deliberately NOT used here.
constexpr uint32_t kCaptureWidth = 960;
constexpr uint32_t kCaptureHeight = 720;

// MPVIZ_CAPTURE_OUT_DIR lets a caller redirect where the PNG lands; default
// /tmp -- these are opt-in manual captures, not committed test artifacts.
// The capture note (docs/visual_mode/env_source_captures.md) documents
// copying the result into docs/visual_mode/env_source_captures/ for the
// comparison package itself.
std::string capture_out_path(const char* name) {
    const char* dir = std::getenv("MPVIZ_CAPTURE_OUT_DIR");
    return std::string(dir && *dir ? dir : "/tmp") + "/env_source_" + name + ".png";
}

// Resolution/pose-independent, no-golden-needed content check: a blank or
// sky-only frame is nearly one flat color, so both (1) the luminance
// std-deviation across the whole frame and (2) the fraction of pixels
// differing from a corner-pixel background sample by more than a small
// tolerance read near zero. Real building geometry -- edges, shadow,
// distinct materials -- pushes both well above zero. Reported per image
// (std::cerr below), never asserted on here: the capture's job is to
// produce and HONESTLY report a result, not to gate on one (the task's own
// "an empty render is reported as such, never presented as a successful
// capture" rule is satisfied by the EXPECT_GT(loaded, 0u) checks in the
// TESTs below, which is the real load-succeeded signal; this is
// corroborating pixel evidence for the report, not a second gate).
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

// Pumps render_frame() until environment_loaded_chunk_count(r) is nonzero
// (bounded by `initial_deadline_sec` wall-clock -- a live network call that
// never resolves must not hang this opt-in run indefinitely, same
// convention as GooglePresetLiveRenderMsDeltaVsOsmClay's own pump_live()
// above), THEN keeps rendering for `settle_seconds` MORE of REAL wall-clock
// time (paced with a short sleep between ticks, not a tight loop) before
// the capture frame.
//
// The settle phase's pacing is load-bearing, not cosmetic: at this
// resolution render_frame() itself is sub-millisecond once nothing new is
// happening (measured), so a tight tick-count loop (this function's first
// draft) burns through hundreds of ticks in under a second of REAL time --
// nowhere near enough for cesium's background thread pool to complete even
// one more HTTP round trip. loaded_chunk_count() > 0 after the FIRST tile
// (the tileset's coarse root, typically) is exactly that non-representative
// case: reproduced directly against the real network, that first-tile
// snapshot rendered as a totally flat, empty frame (no visible geometry at
// all) -- the same thing EnvironmentStreamPerf/live's own "manual visual-
// confidence artifact only" comment already flags as a known gap. Real
// per-building tile detail only appears after several more rounds of
// progressive LOD refinement, each gated on an actual network fetch
// completing -- which needs elapsed wall-clock time between ticks, not more
// ticks. The local, offline fixture path (FixtureBlock_DarkAdas et al.)
// never hit this because disk reads have no meaningful latency to wait out.
uint64_t pump_and_settle(mpviz::VisualRenderer* r, const mpviz::CameraPose& pose,
                          std::vector<uint8_t>& buf, int initial_deadline_sec, int settle_seconds) {
    const auto phase1Deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(initial_deadline_sec);
    uint64_t loaded = 0;
    while (std::chrono::steady_clock::now() < phase1Deadline) {
        if (!mpviz::render_frame(r, pose, {buf.data(), kCaptureWidth, kCaptureHeight})) return loaded;
        loaded = mpviz::testing::environment_loaded_chunk_count(r);
        if (loaded > 0) break;
    }
    if (loaded == 0) return 0;
    const auto settleDeadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(settle_seconds);
    while (std::chrono::steady_clock::now() < settleDeadline) {
        mpviz::render_frame(r, pose, {buf.data(), kCaptureWidth, kCaptureHeight});
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return mpviz::testing::environment_loaded_chunk_count(r);
}

// Renders ONE frame of `r`'s current scene from kCapturePose at the
// generous kCaptureWidth x kCaptureHeight, writes it to
// capture_out_path(name) via the same stb writer golden.cpp links into
// this binary, and logs the content-verification numbers to stderr.
void capture_and_report(mpviz::VisualRenderer* r, const char* name) {
    std::vector<uint8_t> buf(static_cast<size_t>(kCaptureWidth) * kCaptureHeight * 3);
    ASSERT_TRUE(mpviz::render_frame(r, kCapturePose, {buf.data(), kCaptureWidth, kCaptureHeight}));
    const std::string outPath = capture_out_path(name);
    // Gate round 1 finding: MPVIZ_CAPTURE_OUT_DIR is never told to exist by
    // the doc's own re-run command -- create it so the documented command
    // works as written, same as any other output-dir convention in this repo.
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(outPath).parent_path(), ec);
    // Gate round 1 finding: a failed write must never be accompanied by
    // plausible-looking stats -- check the return value BEFORE printing
    // anything, so a silent no-op (e.g. nonexistent dir) can't masquerade as
    // [ OK ] with genuine-looking numbers computed from the in-memory buffer.
    const int wrote = stbi_write_png(outPath.c_str(), static_cast<int>(kCaptureWidth),
                                      static_cast<int>(kCaptureHeight), 3, buf.data(),
                                      static_cast<int>(kCaptureWidth) * 3);
    ASSERT_NE(wrote, 0) << "failed to write capture PNG to " << outPath
                         << " (does MPVIZ_CAPTURE_OUT_DIR exist?)";
    const ContentStats stats = analyze_capture(buf, kCaptureWidth, kCaptureHeight);
    std::cerr << "[EnvSourceCapture] " << name << " -> " << outPath
              << " luminance_stddev=" << stats.luminance_stddev
              << " non_background_fraction=" << stats.non_background_fraction << "\n";
}

}  // namespace

TEST(EnvSourceCapture, Baked) {
    if (std::getenv("MPVIZ_CAPTURE_ENV_SOURCES") == nullptr) {
        GTEST_SKIP() << "opt-in visual comparison capture -- set MPVIZ_CAPTURE_ENV_SOURCES=1 to run "
                        "(never required by ctest)";
    }
    mpviz::RenderConfig cfg{kCaptureWidth, kCaptureHeight, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(mpviz::set_environment_source(r, kTestTownDir.c_str(), kFixtureAnchor));

    mpviz::SceneGraph s{};
    s.ego.valid = 1;
    // Gate round 1 finding: must match Osm/Google's ego position ({0,0,0}), not
    // kChunk0Center -- an ego-anchored ground element is the largest/brightest
    // feature in these dark-theme captures, and differing ego across presets
    // moved it to a different corner of the frame, manufacturing a large
    // apparent "difference" that had nothing to do with the environment
    // source. Verified the fixture chunks still load fine (loaded>0 below,
    // town renders unchanged) at {0,0,0}.
    s.ego.position = mpviz::Vec3{0, 0, 0};
    mpviz::set_scene(r, s);
    std::vector<uint8_t> warm(static_cast<size_t>(kCaptureWidth) * kCaptureHeight * 3);
    ASSERT_GT(pump_and_settle(r, kCapturePose, warm, /*initial_deadline_sec=*/5, /*settle_seconds=*/1), 0u)
        << "baked fixture chunk never loaded at ego {0,0,0}";
    capture_and_report(r, "baked");
    mpviz::destroy_renderer(r);
}

TEST(EnvSourceCapture, Osm) {
    if (std::getenv("MPVIZ_CAPTURE_ENV_SOURCES") == nullptr ||
        std::getenv("CESIUM_ION_TOKEN") == nullptr) {
        GTEST_SKIP() << "opt-in LIVE-network visual comparison capture -- set "
                        "MPVIZ_CAPTURE_ENV_SOURCES=1 and CESIUM_ION_TOKEN to run "
                        "(never required by ctest)";
    }
    mpviz::RenderConfig cfg{kCaptureWidth, kCaptureHeight, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(mpviz::set_environment_source(r, "ion://96188", kFixtureAnchor));

    mpviz::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = mpviz::Vec3{0, 0, 0};
    mpviz::set_scene(r, s);
    std::vector<uint8_t> warm(static_cast<size_t>(kCaptureWidth) * kCaptureHeight * 3);
    const uint64_t loaded =
        pump_and_settle(r, kCapturePose, warm, /*initial_deadline_sec=*/30, /*settle_seconds=*/60);
    EXPECT_GT(loaded, 0u) << "osm preset (ion://96188) loaded no tiles at kFixtureAnchor within 30s "
                             "-- would ship an empty/near-empty capture, reporting rather than faking "
                             "it";
    capture_and_report(r, "osm");
    mpviz::destroy_renderer(r);
}

TEST(EnvSourceCapture, Google) {
    if (std::getenv("MPVIZ_CAPTURE_ENV_SOURCES") == nullptr ||
        std::getenv("CESIUM_ION_TOKEN") == nullptr) {
        GTEST_SKIP() << "opt-in LIVE-network visual comparison capture -- set "
                        "MPVIZ_CAPTURE_ENV_SOURCES=1 and CESIUM_ION_TOKEN to run "
                        "(never required by ctest)";
    }
    mpviz::RenderConfig cfg{kCaptureWidth, kCaptureHeight, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(
        mpviz::set_environment_source(r, "ion://2275207?materials=original&cache=off", kFixtureAnchor));

    mpviz::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = mpviz::Vec3{0, 0, 0};
    mpviz::set_scene(r, s);
    std::vector<uint8_t> warm(static_cast<size_t>(kCaptureWidth) * kCaptureHeight * 3);
    const uint64_t loaded =
        pump_and_settle(r, kCapturePose, warm, /*initial_deadline_sec=*/30, /*settle_seconds=*/60);
    EXPECT_GT(loaded, 0u) << "google preset (ion://2275207) loaded no tiles at kFixtureAnchor within "
                             "30s -- would ship an empty/near-empty capture, reporting rather than "
                             "faking it";
    capture_and_report(r, "google");
    mpviz::destroy_renderer(r);
}

// "clipped" (VM-096's 4th GUI preset) resolves to the NODE's own
// environment_own_asset_uri parameter (resolved server-side in
// tools/vcam_ws_bridge.py's set_environment_source branch), not a
// fixed public ion asset id -- and that parameter defaults empty
// (declared at visualization_node.cpp:687) and is NOT configured on this box. There is
// no real ion asset id to render here: fabricating one would silently ship
// a picture of the WRONG preset (some other asset entirely), which is
// worse than no picture. Always skips, even with the capture opt-in set,
// with that exact reason -- the case is named and reported as
// not-renderable, never silently dropped from the comparison package.
TEST(EnvSourceCapture, Clipped) {
    if (std::getenv("MPVIZ_CAPTURE_ENV_SOURCES") == nullptr) {
        GTEST_SKIP() << "opt-in visual comparison capture -- set MPVIZ_CAPTURE_ENV_SOURCES=1 to run "
                        "(never required by ctest)";
    }
    GTEST_SKIP() << "'clipped' preset resolves to the node's environment_own_asset_uri, which is NOT "
                    "configured on this box (empty default) -- no real ion asset id exists to render, "
                    "so this case is reported as not-renderable rather than faked with a made-up id";
}

// Fixture provenance (Step 0): tests/fixtures/environment_ion_fixture_0/ --
// see that directory's own PROVENANCE.md (real ion OSM Buildings tiles,
// fetched once with the live token, tileset.json hand-pruned to a
// self-contained 3-tile subtree with relative local uris; token appears
// nowhere in the fixture, verified by grep before commit). Task 4's own
// environment_ion_fixture_fallback_0/PROVENANCE.md documents the
// duplicated-tile fixture used by the network-loss e2e above.
