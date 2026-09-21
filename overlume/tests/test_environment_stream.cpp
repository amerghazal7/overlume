// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// test_environment_stream.cpp — StreamingEnvironmentSource behind the VM-052
// seam (VM-062). Same "no Filament/cesium type" boundary as every other
// tests/*.cpp: this is a plain C++17 TU with no cesium include dirs at all
// (CMakeLists.txt's per-test `-I src` grants environment_test_hooks.hpp,
// nothing cesium-flavored) -- Decision 3's quarantine holds even here.
#include "overlume/api.h"
#include "overlume/scene.h"

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
    std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/environment_test_town_0";
// VM-097: synthesized locally by overlume/scripts/make_tile_fixture.py --
// no Cesium ion/OSM content, see that directory's own PROVENANCE.md.
const std::string kTilesFixtureDir =
    std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/environment_tiles_fixture_0";
// VM-063 (Task 4): a dedicated, synthetic 16-tile fixture for the
// network-loss e2e -- see its own PROVENANCE.md for why
// environment_tiles_fixture_0's 3 tiles cannot produce
// kNetworkLossConsecutiveFailures (8) distinct failing requests (a
// succeeded/cached tile can't be forced to fail again inside one short
// test process; a failed tile isn't auto-retried without a fresh
// unload/redesire cycle) and why 16 (duplicated) tiles fixes that.
const std::string kTilesFixtureFallbackDir =
    std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/environment_tiles_fixture_fallback_0";
// 2026-09-21 terrain following ("option 2"), gate round 1 finding 2:
// synthesized via `make_tile_fixture.py --encoding ecef --anchor-lat
// 25.08001258 --anchor-lon 55.38847719 --ground-height 3.0 --no-fallback`
// -- adds tile_ground.b3dm (a flat quad at ellipsoid height 3.0 m around
// its own anchor) to the usual tile_root/tile_a/tile_b set. Same anchor
// convention as NodeMatrixEncodingRendersIdenticallyToEcefEncoding's own
// kSessionAnchor above (the real-robot session's own lat/lon/heading), not
// the file's usual kFixtureAnchor -- see this dir's own PROVENANCE.md.
const std::string kSessionGroundFixtureDir =
    std::string(OVERLUME_TEST_DATA_DIR) +
    "/tests/fixtures/environment_tiles_fixture_session_ground_0";
const overlume::GeoAnchor kSessionGroundAnchor{25.08001258, 55.38847719, 90.1178 * M_PI / 180.0};

constexpr overlume::Vec3 kChunk0Center{-128.0, -128.0, 0.0};

// PROVENANCE.md: the anchor lies inside tile_root.b3dm's own region, so the
// map-frame block center IS the map origin, by WgsToMap's own definition.
constexpr overlume::Vec3 kFixtureBlockCenterMap{0.0, 0.0, 0.0};
constexpr overlume::GeoAnchor kFixtureAnchor{25.0803, 55.3910, 0.0};

overlume::CameraPose kStdPose{{0, -8, 3}, {0, 0, 0.5}, 60};

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

// Pumps render_frame() until either the streaming source has something
// loaded or `max_frames` ticks pass (cesium's own async load pipeline needs
// several ticks: request -> load thread -> main thread prepare -> next
// tick's tilesToRenderThisFrame).
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

}  // namespace

// ── Step 1: dispatch + factory skeleton ─────────────────────────────────
TEST(EnvironmentStream, IonUriWithoutTokenIsNonFatalFalse) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ScopedUnsetEnv no_token(
        "CESIUM_ION_TOKEN");  // this test binary's env only, restored on scope exit
    overlume::GeoAnchor a{25.0803, 55.3910, 0.0};
    EXPECT_FALSE(overlume::set_environment_source(r, "ion://96188", a));
    std::vector<uint8_t> buf(320u * 240u * 3u);
    EXPECT_TRUE(overlume::render_frame(r, kStdPose, {buf.data(), 320, 240}));
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, NonIonUriStillOpensBakedSource) {
    // Regression pin for Decision 5's "byte-identical baked path": the
    // existing fixture town still opens through the SAME entry point.
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

// ── Step 2: the real cesium plumbing, fixture-served ────────────────────
TEST(EnvironmentStream, FixtureTilesLoadRenderAsClayAndCount) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(r, kTilesFixtureDir.c_str(),
                                                                    kFixtureAnchor));

    std::vector<uint8_t> buf(320u * 240u * 3u);
    // Moves ego to `pos` and pumps render_frame(), bounded, until
    // environment_loaded_chunk_count() stops climbing (stable for 10
    // consecutive ticks), returning the settled count.
    // pump_until_loaded()'s own first-nonzero return is racy (it fires as
    // soon as ANY tile lands, not once every reachable tile has) -- this is
    // what actually pins a stable, reproducible number.
    //
    // 2026-09-21 two-frustum tile selection: the render camera is now ALSO a
    // selection frustum (see kStreamViewHeightM's comment,
    // environment_stream.hpp). This test's own job is to pin exact per-
    // position tile counts driven by the SYNTHETIC (ego-following) view
    // alone -- TileSelectionUsesRenderCameraAsSecondFrustum above is the
    // dedicated test for the camera-driven half. kStdPose's fixed,
    // origin-anchored eye/target would otherwise let the camera frustum
    // pin extra, position-independent tiles resident through every
    // settle_count() call, corrupting the counts pinned here -- so instead
    // of trying to co-locate it with `pos` (introducing its OWN new
    // geometric coverage the fixture's tile layout isn't shaped for), the
    // camera here points straight up and far above `pos` (all fixture tile
    // heights sit within roughly -25..40 m, this file's tileset.json), well
    // outside its own frustum -- present (camera_view_state() returns a
    // real ViewState, exercising the code path) but geometrically
    // contributing nothing, isolating this test back to the synthetic
    // view's own counts.
    auto settle_count = [&](overlume::Vec3 pos) -> uint64_t {
        overlume::SceneGraph s{};
        s.ego.valid = 1;
        s.ego.position = pos;
        overlume::set_scene(r, s);
        const overlume::CameraPose pose{
            // 1 mm lateral offset: a target exactly along render_frame()'s
            // fixed up vector (+Z) would be a degenerate lookAt().
            {pos.x, pos.y, pos.z + 5000.0},
            {pos.x + 0.001, pos.y, pos.z + 5001.0},
            60};
        uint64_t count = 0;
        for (int tick = 0, stableTicks = 0; tick < 500 && stableTicks < 10; ++tick) {
            overlume::render_frame(r, pose, {buf.data(), 320, 240});
            const uint64_t next = overlume::testing::environment_loaded_chunk_count(r);
            // Only start counting stability once at least one tile has
            // landed: the initial all-zero window must not read as settled
            // on a slow box. The 500-tick outer bound is the real timeout.
            if (next == count && count > 0) {
                ++stableTicks;
            } else {
                count = next;
                stableTicks = 0;
            }
        }
        return count;
    };

    // kFixtureBlockCenterMap (the anchor itself) sits INSIDE tile_root's own
    // region, but a verified ~1.4 km outside tile_a's nearest edge and
    // ~5.2 km outside tile_b's -- both far past the synthesized streaming
    // view's own ~230 m footprint (kStreamViewHeightM=300, matching
    // kStreamViewFovRad=1.3 in environment_stream.hpp), so only tile_root is
    // EVER reachable from here (empirically confirmed: pumping this ego
    // position never yields more than 1, no matter how long). Gate round 1
    // finding 2 originally wanted a single EXPECT_EQ(3u) at this one ego
    // position; that isn't achievable -- tile_a/tile_b's regions (copied
    // verbatim from the pre-existing real fixture's own tileset.json, same
    // distances from the anchor as before this fixture was synthesized) put
    // them outside the streaming radius by design, not by regression. Pin
    // what is actually true here, then prove tile_a and tile_b each load too
    // by moving ego to sit inside their OWN regions -- this is what actually
    // gives consolidate_buffers() (tile_a's multi-buffer merge) and
    // ensure_flat_normals() (tile_b's missing-NORMAL patch) real,
    // full-streaming-pipeline coverage; test_gltf_normals.cpp's own
    // GltfNormals.StreamedTileWithNormalAlreadyIsUntouched /
    // AddsNormalToStreamedTileMissingIt tests only exercise
    // ensure_flat_normals() at the raw-byte level, never through the
    // Tileset traversal + consolidate_buffers() this test drives.
    EXPECT_EQ(settle_count(kFixtureBlockCenterMap), 1u)
        << "tile_root (single-buffer+NORMAL, +_BATCHID) must load from the anchor's own "
           "position";

    // tile_a's / tile_b's own region-center map positions, via the SAME
    // anchor-relative transform install_fixture_streaming_source() itself
    // uses internally (ecef_to_map_probe wraps compute_ecef_to_map()).
    // ponytail: the lat/lon literals are retyped from
    // make_tile_fixture.py's own TILE_REGIONS midpoints, not re-derived here
    // -- same convention as EcefToMapAgreesWithCppPinWithinHalfMeter's own
    // probes above.
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

// 2026-09-21 two-frustum tile selection (coarse-LOD finding, docs/status.md
// item 4 addendum): synthesize_view_and_pump() must add the real render
// camera's own ViewState as a SECOND selection frustum, once the camera has
// actually been positioned -- see environment_stream.hpp's kStreamViewHeightM
// comment for the full "why" (Google Photorealistic ground rendering as a
// coarse, terrain-follower-lifted mesh with only the synthetic top-down
// frustum in play).
TEST(EnvironmentStream, TileSelectionUsesRenderCameraAsSecondFrustum) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(r, kTilesFixtureDir.c_str(),
                                                                    kFixtureAnchor));
    // environmentSource->update() (which reaches synthesize_view_and_pump())
    // only runs when the scene's ego is valid (renderer.cpp) -- same
    // precondition every other streaming test here sets via set_scene().
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);

    std::vector<uint8_t> buf(320u * 240u * 3u);
    // render_frame() calls environmentSource->update() (which reaches
    // synthesize_view_and_pump()) BEFORE this same frame's own
    // camera->lookAt()/setProjection() (renderer.cpp) -- the documented
    // one-frame lag -- so the FIRST frame's pump still sees an unpositioned
    // camera (1 frustum: synthetic only) and only positions the camera for
    // the frame that follows.
    ASSERT_TRUE(overlume::render_frame(r, kStdPose, {buf.data(), 320, 240}));
    // Fix-round finding (2026-09-21): an earlier version of the "camera not
    // positioned yet" guard was dead code (it also required a zero forward
    // vector, which Filament's camera -- unit-length by construction -- can
    // never report), so this frame's pump silently fed the untouched
    // identity camera to updateViewGroup() as a real selection frustum.
    // Pin the actual FIRST-frame count so a regression of the guard (not
    // just the steady-state count below) fails this test.
    EXPECT_EQ(overlume::testing::environment_stream_last_view_frustum_count(r), 1);
    // Second frame: synthesize_view_and_pump() now runs against a camera
    // positioned by the PRIOR frame's lookAt()/setProjection() -- the
    // steady-state case this test pins.
    ASSERT_TRUE(overlume::render_frame(r, kStdPose, {buf.data(), 320, 240}));

    EXPECT_EQ(overlume::testing::environment_stream_last_view_frustum_count(r), 2);

    overlume::destroy_renderer(r);
}

// ── Node-matrix regression: real Google 3D Tiles glbs vs. this generator's
//    own identity-node convention ──────────────────────────────────────────
// BUG (diagnosed 2026-09-21 against a real Google tile, ion asset 2275207,
// over the real-robot session_2026-09-01_13-57-00 geo anchor):
// strip_attributes_and_correct_heights() (environment_stream.cpp) rewrites
// every POSITION using ONLY the glTF ROOT transform (modelToEcef passed in
// from prepareInLoadThread()) -- it never looks at the per-primitive NODE
// transform gltfio itself applies at render time (ecefToMap_ * glb->transform
// composed with each node's own matrix). That's invisible against every
// OTHER committed fixture in this file, because make_tile_fixture.py's usual
// "ecef" encoding authors absolute-ECEF-scale positions under an IDENTITY
// node ({"mesh":0}, no matrix) -- exactly the one case where "root transform
// only" happens to be the whole transform. A real Google glb is not shaped
// like that: it carries node-LOCAL float32 positions under a real
// {"mesh":0,"matrix":[...]} node (see make_tile_fixture.py's own
// --encoding node-matrix comment for the exact matrix), so the root-only
// rewrite sends its vertices to the wrong ECEF position entirely --
// symptom: Google tiles render as giant tilted slabs across the sky.
//
// The two fixtures below encode the exact SAME synthetic geometry (same
// seed, same anchor -- the real session's own lat 25.08001258 /
// lon 55.38847719 / heading 90.1178 deg, not the file's usual
// kFixtureAnchor), one identity-node ("ecef"), one node-matrix -- a correct
// pipeline renders them identically (the node-matrix fixture's node matrix
// exactly reconstructs the ecef fixture's own authored positions, see
// build_tile_glb()'s comment), so this test renders both and requires a high
// SSIM between them. On current HEAD this MUST fail: frame B (node-matrix)
// is the one strip_attributes_and_correct_heights() mis-corrects.
TEST(EnvironmentStream, NodeMatrixEncodingRendersIdenticallyToEcefEncoding) {
    const std::string kSessionEcefDir = std::string(OVERLUME_TEST_DATA_DIR) +
                                        "/tests/fixtures/environment_tiles_fixture_session_ecef_0";
    const std::string kSessionNodeMatrixDir =
        std::string(OVERLUME_TEST_DATA_DIR) +
        "/tests/fixtures/environment_tiles_fixture_session_nodematrix_0";
    // Real-robot session_2026-09-01_13-57-00's own geo anchor -- the session
    // fixtures' TILE_REGIONS were shifted to sit around this exact anchor
    // (make_tile_fixture.py --anchor-lat/--anchor-lon), so kFixtureBlockCenterMap
    // (the anchor itself, map-frame origin by construction -- same reasoning
    // as kFixtureAnchor's own PROVENANCE.md note) and FixtureTilesLoadRenderAsClayAndCount
    // / EnvironmentStreamGolden.FixtureBlock_DarkAdas's own pose both carry
    // over unchanged: the geometry is anchor-relative, so shifting the
    // anchor by the same delta as the regions leaves every map-frame
    // position (ego, camera, tile footprint) exactly where it was.
    const overlume::GeoAnchor kSessionAnchor{25.08001258, 55.38847719, 90.1178 * M_PI / 180.0};
    // Framed on tile_root's own region-center map position -- NOT a
    // hardcoded literal like EnvironmentStreamGolden.FixtureBlock_DarkAdas's
    // own pose (measured from a heading=0 anchor): this fixture's heading is
    // 90.1178 deg, which rotates the ENU->map basis, so a heading=0-derived
    // camera offset would no longer point at the buildings at all. Computed
    // via the SAME ecef_to_map_probe() hook / literal-midpoint convention
    // the tile_a/tile_b probes above use (region center lat/lon retyped from
    // make_tile_fixture.py's own shifted TILE_REGIONS, not re-derived here),
    // then offset by the SAME relative vector FixtureBlock_DarkAdas's own
    // pose uses relative to ITS target ((149,352,90) - (69,472,0)) -- a
    // fixed map-frame offset, so it stays a sane framing distance regardless
    // of which way heading rotated the target.
    double targetX = 0, targetY = 0, targetZ = 0;
    ASSERT_TRUE(overlume::testing::ecef_to_map_probe(
        kSessionAnchor.origin_lat_deg, kSessionAnchor.origin_lon_deg, kSessionAnchor.heading_rad,
        0.0, 25.080637330000002, 55.38380269000002, 0.0, &targetX, &targetY, &targetZ));
    overlume::CameraPose pose{
        {targetX + 80, targetY - 120, targetZ + 90}, {targetX, targetY, targetZ}, 60.0};

    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};

    // Frame A: identity-node ("ecef") encoding -- this generator's usual
    // convention, unaffected by the bug above.
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
    // Manual visual-confidence artifact only -- the returned SSIM is
    // discarded (there is no golden at this path by construction); this
    // call's only job is writing frame A out so frame B can be compared
    // against it below, same "compare-against-a-just-captured-frame"
    // idiom EnvironmentStreamPerf.GooglePresetLiveRenderMsDeltaVsOsmClay's
    // own pump_live() uses.
    overlume::testing::render_and_compare(rA, pose, "/nonexistent_no_golden.png",
                                          "/tmp/environment_stream_session_ecef_actual.png");
    overlume::destroy_renderer(rA);

    // Frame B: the SAME synthetic geometry, node-matrix encoded (Google's
    // own convention) -- fresh renderer to swap sources, same pattern
    // DiskCacheServesTilesWithNetworkDead / FixtureBlock_DarkAdas use.
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
        rB, pose, "/tmp/environment_stream_session_ecef_actual.png",
        "/tmp/environment_stream_session_nodematrix_actual.png");
    EXPECT_GE(ssim, 0.97)
        << "node-matrix-encoded tiles (Google's own convention) must render identically to "
           "the SAME geometry's identity-node encoding -- strip_attributes_and_correct_heights() "
           "ignoring the primitive's own node transform sends node-matrix vertices to the wrong "
           "ECEF position (root-transform-only correction), got ssim="
        << ssim;
    overlume::destroy_renderer(rB);
}

// ── Step 3: geo placement cross-pin (Epic 4's committed pin fixture) ─────
// Same truth table as test_geo_anchor.cpp's own cross-language pin, applied
// to the STREAMING transform chain (ecef_to_map * wgs_to_ecef) instead of
// WgsToMap directly -- three implementations, one truth table.
TEST(EnvironmentStream, EcefToMapAgreesWithCppPinWithinHalfMeter) {
    const std::string path =
        std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/geo_anchor_cpp_pin_0.json";
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
        ASSERT_TRUE(overlume::testing::ecef_to_map_probe(25.0803, 55.391, p.heading_rad, 0.0, p.lat,
                                                         p.lon, 0.0, &x, &y, &z));
        const double err = std::sqrt((x - p.map_x) * (x - p.map_x) + (y - p.map_y) * (y - p.map_y));
        EXPECT_LT(err, 0.5) << "lat=" << p.lat << " lon=" << p.lon << " heading=" << p.heading_rad
                            << " got=(" << x << "," << y << "," << z << ") want=(" << p.map_x << ","
                            << p.map_y << "," << p.map_z << ") err=" << err;
        // z is asserted against the tangent-plane sag model itself,
        // -d^2/(2R) (VM-062 gate round 2 minor): a genuine up-axis
        // inversion or z-scale regression in compute_ecef_to_map must fail
        // HERE -- the horizontal bar above cannot see it.
        const double want_z = -(p.map_x * p.map_x + p.map_y * p.map_y) / (2.0 * 6371000.0);
        EXPECT_NEAR(z, want_z, 0.05)
            << "lat=" << p.lat << " lon=" << p.lon << " z=" << z << " want_z(sag)=" << want_z;
    }
}

// ── Open Follow-up 4: streamed-tile ellipsoid-height correction (pure math,
//    docs/status.md item 4) ────────────────────────────────────────────────
// Points ON the WGS84 ellipsoid at horizontal offsets 0/1/2.6/10 km due
// north of the SAME fixture anchor (25.0803 N, 55.3910 E) every other
// cross-pin test in this file uses. Proves both halves in one test: the
// UNcorrected rigid transform (ecef_height_correction_probe's
// out_z_uncorrected, identical formula to ecef_to_map_probe's own z, which
// EcefToMapAgreesWithCppPinWithinHalfMeter above already pins at its own
// probes) sags by the documented tangent-plane amount
// -d^2/(2R) (0.08 m / 0.54 m / 7.85 m at 1 / 2.6 / 10 km -- Open Follow-up
// 4's own numbers), AND the corrected z (correct_ecef_point_height(),
// environment_stream.hpp/.cpp -- the SAME function
// strip_attributes_and_correct_heights() calls per real vertex, gate round 1
// major finding: this probe must not carry an independently-typed copy of
// that formula) lands within 5 mm of the map plane at every offset,
// including 0 (the anchor itself).
//
// This is a PURE-MATH test: it proves the z-correction formula itself
// (ellipsoid-height-of-the-point minus the anchor's) is right, on doubles,
// for a point that is already exactly on the ellipsoid. It does NOT touch a
// glTF accessor, a float32 store, or the mesh/primitive traversal
// strip_attributes_and_correct_heights() does around this formula -- gate
// round 1 measured that those mechanics, applied to the REAL committed
// fixture geometry (which is authored as absolute-ECEF float32, ULP
// 0.25-0.5 m at Earth-radius magnitude), quantize this correction away
// entirely within about 1.5 km of the anchor and leave a 0.04-0.11 m
// residual at the fixture's actual tile ranges (3.8/6.6 km) -- see
// docs/status.md item 4 and docs/runbooks/cesium.md 5b for the measured
// numbers. An ingestion-level check that reads those float32-quantized
// results back through a loaded tile was scoped but not added here --
// FilamentAsset::getBoundingBox() reads the glTF accessor's DECLARED
// min/max (computed once at fixture-authoring time), which
// strip_attributes_and_correct_heights() does not update when it rewrites
// the accessor's data, so it would not reflect this correction at all
// without also teaching that function to refresh accessor min/max (real
// production scope, not a test-only addition); the alternative -- invoking
// Cesium3DTilesContent::B3dmToGltfConverter directly on the fixture bytes,
// bypassing the full Tileset pipeline -- needs a real AssetFetcher/
// AsyncSystem wired up outside the existing streaming-source machinery.
// Both are real follow-up work, not a small addition to this test.
TEST(EnvironmentStream, EllipsoidHeightCorrectionRemovesTangentPlaneSag) {
    constexpr double kAnchorLat = 25.0803, kAnchorLon = 55.391;
    // Same mean-sphere radius compute_ecef_to_map()'s own east/north rescale
    // targets (VM-062 gate round 1 Finding 1) and the sag model above uses --
    // duplicated here rather than shared across the C++17/C++20 test
    // boundary, same convention as every other literal-retyped constant in
    // this file.
    constexpr double kSphereRadiusM = 6371000.0;
    const std::vector<double> offsets_m = {0.0, 1000.0, 2600.0, 10000.0};
    for (const double offset_m : offsets_m) {
        // Due-north offset: dlat = offset / R (radians), same small-angle
        // relation geo_anchor.cpp's own WgsToMap north formula inverts.
        const double lat = kAnchorLat + (offset_m / kSphereRadiusM) * (180.0 / M_PI);

        double zUncorrected = 0.0, zCorrected = 0.0;
        ASSERT_TRUE(overlume::testing::ecef_height_correction_probe(
            kAnchorLat, kAnchorLon, /*heading_rad=*/0.0, /*origin_height_m=*/0.0, lat, kAnchorLon,
            /*alt_m=*/0.0, &zUncorrected, &zCorrected));

        const double wantSag = -(offset_m * offset_m) / (2.0 * kSphereRadiusM);
        EXPECT_NEAR(zUncorrected, wantSag, 0.05)
            << "offset=" << offset_m << "m: uncorrected z=" << zUncorrected << " should sag ~"
            << wantSag << "m below the tangent plane -- the bug this "
            << "follow-up fixes";
        EXPECT_LT(std::abs(zCorrected), 0.005)
            << "offset=" << offset_m << "m: corrected z=" << zCorrected
            << " should sit on the map plane within 5mm";
        // Gate round 1 major finding, minimum fix: assert the APPLIED DELTA
        // too, not just that corrected lands near 0 -- this at least fails
        // if the correction's sign or magnitude is wrong even at an offset
        // where both halves independently look small.
        EXPECT_NEAR(zCorrected - zUncorrected, -wantSag, 0.05)
            << "offset=" << offset_m << "m: correction should remove exactly the sag amount";
    }
}

// ── 2026-09-21 finding: anchor height is no longer hard-coded 0.0 ────────
// (docs/status.md item 4). On the real robot the anchor sits ~1.7 m above
// the WGS84 ellipsoid (Fixposition NavSatFix altitude); with
// origin_height_m set to a nonzero anchor height, a point at that SAME
// altitude must now correct to z=0 (the map plane), not to its old,
// simulation-only z=0-at-ellipsoid meaning.
TEST(EnvironmentStream, AnchorHeightShiftsCorrectedZ) {
    constexpr double kAnchorLat = 25.0803, kAnchorLon = 55.391;
    constexpr double kOriginHeightM = 25.0;

    double zUncorrected = 0.0, zCorrected = 0.0;
    ASSERT_TRUE(overlume::testing::ecef_height_correction_probe(
        kAnchorLat, kAnchorLon, /*heading_rad=*/0.0, kOriginHeightM, kAnchorLat, kAnchorLon,
        /*alt_m=*/25.0, &zUncorrected, &zCorrected));
    // Covers compute_ecef_to_map()'s half of the fix (review minor, 2026-09-21):
    // the rigid transform's ENU origin must sit at origin_height_m, not on the
    // ellipsoid -- pinned back at 0.0 this reads ~25 m.
    EXPECT_NEAR(zUncorrected, 0.0, 1e-3)
        << "the ENU origin must sit at origin_height_m, not on the ellipsoid; got " << zUncorrected;
    EXPECT_NEAR(zCorrected, 0.0, 1e-3)
        << "a point at the anchor's own altitude (25.0 m) must correct to z=0 when "
           "origin_height_m=25.0, got "
        << zCorrected;

    ASSERT_TRUE(overlume::testing::ecef_height_correction_probe(
        kAnchorLat, kAnchorLon, /*heading_rad=*/0.0, kOriginHeightM, kAnchorLat, kAnchorLon,
        /*alt_m=*/30.0, &zUncorrected, &zCorrected));
    EXPECT_NEAR(zCorrected, 5.0, 1e-3)
        << "a point 5 m above the anchor's own altitude must correct to z=5.0 when "
           "origin_height_m=25.0, got "
        << zCorrected;
}

// ── Step 4: disk cache proves itself (offline second-run reload) ────────
TEST(EnvironmentStream, DiskCacheServesTilesWithNetworkDead) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // Run 1: healthy fixture accessor, warms the on-disk sqlite cache
    // (Decision 10) in the per-process temp dir test_cache_dir() names
    // (under std::filesystem::temp_directory_path(), NOT the committed
    // fixture tree -- gate round 1 finding 4).
    auto* handle1 = overlume::testing::install_fixture_streaming_source_with_fallback(
        r, kTilesFixtureDir.c_str(), /*fallback_baked_dir=*/nullptr, kFixtureAnchor);
    ASSERT_NE(handle1, nullptr);
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);
    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GT(pump_until_loaded(r, kStdPose, buf), 0u);
    // The sqlite cache write for each response happens on a background
    // thread pool, independent of (and not gated by) content ingestion --
    // pump extra ticks so those writes settle before tearing this source
    // down, or the SECOND source below could race an incomplete cache.
    for (int i = 0; i < 30; ++i) overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});

    // Run 2: shares run 1's cache because test_cache_dir() is static
    // within the process (same per-process temp path, Decision 10), but the
    // "network" killed from tick 0 -- any tile that still loads came from
    // the sqlite cache, not a live fetch.
    auto* handle2 = overlume::testing::install_fixture_streaming_source_with_fallback(
        r, kTilesFixtureDir.c_str(), /*fallback_baked_dir=*/nullptr, kFixtureAnchor);
    ASSERT_NE(handle2, nullptr);
    overlume::testing::kill_fixture_network(handle2);
    overlume::set_scene(r, s);  // re-publish ego (install_* only swaps r->environmentSource)
    const uint64_t loadedOffline = pump_until_loaded(r, kStdPose, buf);
    EXPECT_GT(loadedOffline, 0u);
    overlume::destroy_renderer(r);
}

// ── Step 5: golden + theming (P3 golden scoping, one theme) ─────────────
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

    // Framed on the fixture tiles' own real footprint centroid -- measured
    // directly from the placed FilamentAsset's world-space bounding box
    // (~(69, 472, -3)), NOT the anchor point or the tile's nominal region
    // center (same "frame on the real footprint" lesson
    // test_environment.cpp's own kBuildingsCentroid comment documents).
    overlume::CameraPose pose{{149, 352, 90}, {69, 472, 0}, 60.0};
    std::vector<uint8_t> warm(320u * 240u * 3u);
    ASSERT_GT(pump_until_loaded(r, pose, warm), 0u);
    // Keep pumping so the fixture's other two tiles (not just whichever
    // loads first) have a chance to finish loading too before the golden
    // comparison frame -- more of the real content on screen.
    for (int i = 0; i < 60; ++i) overlume::render_frame(r, pose, {warm.data(), 320, 240});

    double ssim = overlume::testing::render_and_compare(
        r, pose, OVERLUME_TEST_DATA_DIR "/tests/goldens/environment_stream_dark_adas.png",
        "/tmp/environment_stream_dark_adas_actual.png");
    EXPECT_GT(ssim, 0.98);
    overlume::destroy_renderer(r);
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
    // Not torn down: still "loaded" (a re-show must not need a re-fetch),
    // but zero of it is actually in the Filament scene right now.
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
size_t count_differing_bytes_with_tolerance(const std::vector<uint8_t>& a,
                                            const std::vector<uint8_t>& b, int tolerance) {
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
    // Same framing as FixtureBlock_DarkAdas -- the fixture's real footprint
    // centroid, so a popped-in tile would occupy a real chunk of the frame.
    overlume::CameraPose pose{{149, 352, 90}, {69, 472, 0}, 60.0};
    const size_t nBytes = 320u * 240u * 3u;
    std::vector<uint8_t> hiddenBuf(nBytes);
    ASSERT_GT(pump_until_loaded(r, pose, hiddenBuf), 0u)
        << "loading itself must still happen while hidden";
    for (int i = 0; i < 30; ++i) overlume::render_frame(r, pose, {hiddenBuf.data(), 320, 240});
    ASSERT_TRUE(overlume::render_frame(r, pose, {hiddenBuf.data(), 320, 240}));
    EXPECT_EQ(overlume::testing::environment_scene_membership_count(r), 0u)
        << "a tile loaded while hidden was added to the Filament scene anyway";

    // Reference: a SECOND renderer with no environment source at all -- the
    // ground truth for "tiles not rendered" (same technique
    // test_environment.cpp's own sibling test uses).
    auto* rNoEnv = overlume::create_renderer(cfg);
    ASSERT_NE(rNoEnv, nullptr);
    overlume::set_scene(rNoEnv, s);
    std::vector<uint8_t> noEnvBuf(nBytes);
    ASSERT_TRUE(overlume::render_frame(rNoEnv, pose, {noEnvBuf.data(), 320, 240}));
    overlume::destroy_renderer(rNoEnv);

    // Measured on this fixture/framing: the tolerance-filtered noise floor
    // between a hidden-with-loaded-tiles renderer and one with no
    // environment source at all is ~0 bytes; the real pop-into-view signal
    // (this same comparison, hidden vs. re-shown on the SAME renderer, see
    // diffShown below) is ~326 bytes. 100 sits well under that signal while
    // leaving headroom over the noise floor, so this bound still fails if
    // the add-while-hidden guard is deleted -- it is a coarse sanity check;
    // the real guard is the membership EXPECT_EQ above.
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
    // A small floor, not the ~5% SetEnvironmentVisibleFalseHidesLoadedChunksWithoutTearingDown
    // uses for the BAKED town's own centroid framing: this fixture's tiles
    // cover only a small corner of THIS pose (this file's own comment on
    // SetVisibleFalseHidesFixtureTilesWithoutTearingDown), so a re-show only
    // flips a modest, tolerance-filtered ~326 bytes at 320x240 -- a real
    // signal (this same tolerance already proved hiddenBuf indistinguishable
    // from a renderer with no source above), just a small one.
    EXPECT_GT(diffShown, 50u) << "showing again produced no visible change (only " << diffShown
                              << "/" << nBytes << " bytes changed beyond rendering noise)";
    overlume::destroy_renderer(r);
}

// ── Step 6: perf check (dev-box proxy, same honesty class as
//    EnvironmentPerf.RenderMsDeltaWithTestTownLoaded) ─────────────────────
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
    ASSERT_GT(pump_until_loaded(r, pose, buf), 0u);  // warm: tiles already loaded before timing

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

// ── Finding #0 (security, blocking): CESIUM_ION_TOKEN redaction ─────────
// Drives the real ion-handshake error path (no network, no real token --
// see drive_ion_token_redaction_probe()'s own comment) with a bogus literal
// standing in for the token, and asserts the library's own log output never
// carries it, nor an un-redacted access_token=/Bearer credential shape.
TEST(TokenRedaction, NeverLeaksIntoLogText) {
    constexpr const char* kBogusToken = "bogus-token-for-redaction-test";
    ASSERT_TRUE(overlume::testing::drive_ion_token_redaction_probe(kBogusToken, /*asset_id=*/123456,
                                                                   /*max_ticks=*/200))
        << "the probe never logged anything at all -- test infrastructure issue, not a pass";

    const std::string logText = overlume::testing::captured_cesium_log_text();
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
            << "access_token= was followed by something other than <redacted> (" << rest.size()
            << " chars follow; content withheld -- it may be a credential)";
        pos += std::strlen("access_token=");
    }
}

// ── Task 4 (VM-063): network-loss fallback e2e ──────────────────────────
namespace {

// Pumps until `stop_state` is observed or `max_ticks` pass, asserting every
// tick's render_frame() still succeeds throughout (spec §9, never a crash).
// Ego stays put at kFixtureBlockCenterMap the whole time -- with
// environment_tiles_fixture_fallback_0's 16 same-region tiles, the killed
// accessor produces >= kNetworkLossConsecutiveFailures within the first
// couple of ticks without any ego motion (unlike the original 3-tile
// fixture, where a stationary ego issues no NEW tile requests once
// whatever's already resident is loaded).
void pump_until_state(overlume::VisualRenderer* r, const overlume::CameraPose& pose,
                      std::vector<uint8_t>& buf, overlume::EnvironmentSourceState stop_state,
                      int max_ticks = 30) {
    for (int i = 0; i < max_ticks; ++i) {
        ASSERT_TRUE(overlume::render_frame(r, pose, {buf.data(), 320, 240}));
        if (overlume::environment_source_state(r) == stop_state) return;
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
// exempted from the kill switch, per environment_tiles_fixture_fallback_0's
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
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // Fixture source with a kill switch, fallback pointed at Epic 4's
    // committed baked test town (the REAL fallback content path, not a
    // stub) -- kFixtureBlockCenterMap sits within kLoadRadiusM of the
    // town's own chunk_-1_-1 (center (-128,-128,0), radius_m ~181 < 300),
    // so the fallback loads real chunks with no ego motion needed.
    auto* killable = overlume::testing::install_fixture_streaming_source_with_fallback(
        r, kTilesFixtureFallbackDir.c_str(), kTestTownDir.c_str(), kFixtureAnchor);
    ASSERT_NE(killable, nullptr);

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);

    // Phase 1: healthy -- construction alone reports STREAMING ("network
    // healthy (or untested)", scene.h's own Interfaces comment) before a
    // single tick has run. Nothing has streamed in yet (inScene_ is empty)
    // -- see the test-level comment above for why this test cannot start
    // from a genuinely loaded STREAMING state.
    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::STREAMING);

    // Phase 2: kill the network before any tile has ever been requested;
    // every one of the fixture's 16 real tiles is then a first-ever,
    // never-cached request that fails once the killed accessor is hit --
    // real HTTP-shaped failures through the SAME CountingAssetAccessor ->
    // CachingAssetAccessor(SqliteCache) stack production traffic runs, not
    // a mocked counter.
    std::vector<uint8_t> buf(320u * 240u * 3u);
    overlume::testing::kill_fixture_network(killable);
    pump_until_state(r, kStdPose, buf, overlume::EnvironmentSourceState::STREAMING_FALLBACK);

    // Fallback declared after kNetworkLossConsecutiveFailures failed requests:
    EXPECT_EQ(overlume::environment_source_state(r),
              overlume::EnvironmentSourceState::STREAMING_FALLBACK);
    // AC: "baked chunks appear" -- the count now reports the BAKED town's
    // chunks. NOTE: this does NOT prove the streamed tiles were "torn down,
    // not orphaned" -- none were ever resident (no tile ever succeeded in
    // this test), so there was nothing to tear down. That teardown-under-
    // fallback claim is the named gap in the comment above, not covered
    // here.
    EXPECT_GT(overlume::testing::environment_loaded_chunk_count(r), 0u);

    // The "Once" in this test's name, asserted rather than left to review:
    // Decision 11 makes the switch ONE-WAY for the process's life. Revive
    // the fixture "network" and pump well past the tick count the original
    // transition needed -- the source must NOT resume streaming.
    overlume::testing::revive_fixture_network(killable);
    for (int i = 0; i < 10; ++i) overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});
    EXPECT_EQ(overlume::environment_source_state(r),
              overlume::EnvironmentSourceState::STREAMING_FALLBACK)
        << "a revived network must not un-do the fallback (Decision 11: one-way)";
    EXPECT_GT(overlume::testing::environment_loaded_chunk_count(r), 0u)
        << "the baked chunks must stay resident after the network returns";
    overlume::destroy_renderer(r);
}

// Negative path (Decision 11): no &fallback= given -> state still
// transitions to STREAMING_FALLBACK, loaded_chunk_count stays at 0, no
// crash, render_frame keeps returning true (spec §9 all the way down).
TEST(EnvironmentStream, NetworkLossWithNoFallbackDirStillTransitionsAndStaysEmpty) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    auto* killable = overlume::testing::install_fixture_streaming_source_with_fallback(
        r, kTilesFixtureFallbackDir.c_str(), /*fallback_baked_dir=*/nullptr, kFixtureAnchor);
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
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(
        r, kTilesFixtureDir.c_str(), kFixtureAnchor, /*materials_original=*/true));
    EXPECT_TRUE(overlume::testing::environment_stream_materials_original(r));
    overlume::destroy_renderer(r);
}

// VM-064 gate round 1 finding: the two tests above both reach
// materials_original via install_fixture_streaming_source()'s own bool
// param, which bypasses parse_ion_spec() entirely -- neither one would
// notice a typo in the parser's key match (e.g. "material" instead of
// "materials"). This calls parse_ion_spec() directly (via the
// environment_stream_parse_materials_original() hook) on the literal
// ion:// query string, no renderer/GPU needed.
TEST(EnvironmentStream, ParseMaterialsOriginalRecognizesTheRealKeyAndValue) {
    EXPECT_TRUE(
        overlume::testing::environment_stream_parse_materials_original("96188?materials=original"));
    EXPECT_FALSE(overlume::testing::environment_stream_parse_materials_original("96188"));
    EXPECT_FALSE(
        overlume::testing::environment_stream_parse_materials_original("96188?materials=clay"));
    EXPECT_FALSE(
        overlume::testing::environment_stream_parse_materials_original("96188?materials=bogus"));
}

// The real string-parser path (Decision 5's parse_ion_spec()), through the
// public ion:// dispatch -- no token, so this only pins that an unknown/
// absent materials= key never crashes the parse (spec §9); the true-token
// case can't run in ctest (network discipline), so parse_ion_spec()'s
// "unknown value -> WARN once, clay" branch itself is covered by this same
// non-fatal-false shape: any materials= value (or none) on a URI with no
// token still returns false, never crashes.
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

// ── 2026-09-21 terrain following ("option 2") ────────────────────────────
// Design item 1: unlike materials= (bad value degrades to clay), an
// unrecognized follow_terrain= value fails parse_ion_spec() entirely -- see
// environment_stream_parse_follow_terrain()'s own comment for why the hook
// reports parse success separately from the returned bool.
TEST(EnvironmentStream, ParseIonSpecFollowTerrain) {
    bool ok = false;
    EXPECT_TRUE(
        overlume::testing::environment_stream_parse_follow_terrain("96188?follow_terrain=on", &ok));
    EXPECT_TRUE(ok);
    EXPECT_FALSE(overlume::testing::environment_stream_parse_follow_terrain("96188", &ok));
    EXPECT_TRUE(ok);  // absent key -- parses fine, just defaults false
    overlume::testing::environment_stream_parse_follow_terrain("96188?follow_terrain=maybe", &ok);
    EXPECT_FALSE(ok);  // malformed value -- the WHOLE spec fails to parse
}

// 2026-09-21 multi-point plane fit, design item 5: max_tilt_deg= is the same
// shape as ground_bias= (non-numeric fails the whole parse) PLUS a
// negativity check ground_bias= doesn't have.
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

// Design item 7a: the real least-squares fit, pinned directly.
TEST(EnvironmentStream, TerrainPlaneFitProbe) {
    // Exact recovery of a synthetic line: slope 0.02, intercept 1.5, every
    // point exactly on it -> residual RMS ~0.
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
    // Noisy set (symmetric +-0.1 m jitter about the same line) -- the fit
    // still recovers slope/intercept closely, but RMS is now > 0 (the whole
    // point of reporting it: a caller can tell a clean fit from a noisy
    // one).
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
    // Design item 2's degrade ladder: 1-2 hits -> slope 0, intercept = mean.
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

// Design item 7b: the transform cancels a fitted grade -- points at
// s = -20, 0, +20 all land within 0.02 m of -ground_bias, unlike the OLD
// offset-only path (max_tilt_rad=0), which leaves +-0.4 m of error at
// s = +-20 for the SAME 2% grade (design item 7d exercises that
// old-path-equivalent directly too).
TEST(EnvironmentStream, TerrainTransformProbeCancelsGradeOnFittedLine) {
    constexpr double kSlope = 0.02;     // 2% grade
    constexpr double kIntercept = 3.0;  // fitted height at the pivot, ellipsoid metres
    constexpr double kAnchorHeightM = 0.0;
    constexpr double kGroundBiasM = 0.3;
    constexpr double kMaxTiltRad = 2.0 * M_PI / 180.0;  // kTerrainMaxTiltRad's own default
    constexpr double kHeadingRad = 0.0;
    for (const double s : {-20.0, 0.0, 20.0}) {
        const double z = overlume::testing::terrain_transform_probe(
            kSlope, kIntercept, kAnchorHeightM, kGroundBiasM, kMaxTiltRad, kHeadingRad,
            /*pivot_x=*/0.0, /*pivot_y=*/0.0, s);
        EXPECT_NEAR(z, -kGroundBiasM, 0.02)
            << "s=" << s << ": the fitted tilt must cancel the grade, landing near -ground_bias";
    }
}

// Design item 7c: a 20% slope's atan() (~11.3 deg) exceeds the 2 deg
// default clamp -- the applied tilt clamps to EXACTLY kTerrainMaxTiltRad,
// and the resulting residual error at the fit's own span edge is asserted
// (not pretended away as flat): with theta clamped below the exact-cancel
// angle, the s-dependent term s*(slope*cos(theta) - sin(theta)) is nonzero.
TEST(EnvironmentStream, TerrainTransformProbeClampsSteepGrade) {
    constexpr double kSlope = 0.20;  // 20% grade -- atan(0.20) ~= 0.1974 rad, well past the clamp
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
        kSlope, kIntercept, kAnchorHeightM, kGroundBiasM, kMaxTiltRad, kHeadingRad,
        /*pivot_x=*/0.0, /*pivot_y=*/0.0, kS);
    // z = -ground_bias*cos(theta) + s*(slope*cos(theta) - sin(theta)) at the clamped theta.
    // Tolerance 1e-5, not 1e-9 (Opus gate fix round): terrain_transform_probe()
    // now multiplies the point through the REAL terrain_root_matrix(), a
    // filament::math::mat4f (float32) -- so this pins the shipped code
    // path's own float32 rounding (~1e-7 relative at this magnitude), not a
    // bit-exact double reimplementation of it.
    EXPECT_NEAR(z, -kGroundBiasM * std::cos(clampedTheta) + expectedResidual, 1e-5);
    // The clamp leaves a REAL, nonzero residual at the span edge -- this is
    // the honest cost of the clamp, not a flat result.
    EXPECT_GT(std::abs(expectedResidual), 0.01);
}

// Design item 7d: max_tilt_deg=0 (max_tilt_rad=0.0) reproduces the pre-fit
// offset-only behaviour EXACTLY -- theta is forced to 0 regardless of
// slope, so every along-track point drifts by exactly slope*s away from
// -ground_bias (the SAME error the old single-scalar offset always had).
TEST(EnvironmentStream, TerrainTransformProbeZeroMaxTiltMatchesOldOffsetOnlyBehaviour) {
    constexpr double kSlope = 0.02;
    constexpr double kIntercept = 3.0;
    constexpr double kAnchorHeightM = 0.0;
    constexpr double kGroundBiasM = 0.3;
    constexpr double kHeadingRad = 0.0;
    // Tolerance 1e-5, not 1e-9 (Opus gate fix round): terrain_transform_probe()
    // now multiplies the point through the REAL terrain_root_matrix(), a
    // filament::math::mat4f (float32) -- so this pins the shipped code
    // path's own float32 rounding (~1e-7 relative at this magnitude), not a
    // bit-exact double reimplementation of it.
    for (const double s : {-20.0, 0.0, 20.0}) {
        const double z = overlume::testing::terrain_transform_probe(
            kSlope, kIntercept, kAnchorHeightM, kGroundBiasM, /*max_tilt_rad=*/0.0, kHeadingRad,
            /*pivot_x=*/0.0, /*pivot_y=*/0.0, s);
        EXPECT_NEAR(z, -kGroundBiasM + kSlope * s, 1e-5)
            << "max_tilt_deg=0 must reproduce the OLD offset-only error exactly";
    }
    // At s=+-20 that error is exactly 0.02*20 = 0.4 m -- the OLD path's own
    // documented failure this whole fix exists to close.
    const double zAt20 = overlume::testing::terrain_transform_probe(
        kSlope, kIntercept, kAnchorHeightM, kGroundBiasM, 0.0, kHeadingRad, 0.0, 0.0, 20.0);
    EXPECT_NEAR(zAt20 - (-kGroundBiasM), 0.4, 1e-5);
}

// Design item 3's smoothing/clamp math, pinned directly (no tileset/renderer
// needed -- terrain_ground_offset_probe() exercises the exact same
// terrain_target_offset_z()/terrain_smooth_toward() functions
// FollowTerrainShiftsEnvironmentToGroundUnderEgo below drives end to end
// through a real tileset).
TEST(EnvironmentStream, TerrainGroundOffsetSmoothingMath) {
    // Snap on the first sample: sampled height 3.0 m, anchor at 0.0 m ->
    // target offset -3.0 m, applied immediately regardless of dt.
    EXPECT_NEAR(overlume::testing::terrain_ground_offset_probe(
                    /*sampled_height_m=*/3.0, /*anchor_height_m=*/0.0, /*ground_bias_m=*/0.0,
                    /*current_offset_m=*/0.0, /*delta_seconds=*/0.0f, /*snap=*/true),
                -3.0, 1e-9);
    // Ground bias (2026-09-21 z-fight fix): the follower parks the ground
    // ground_bias_m BELOW the map plane, so the same 3.0 m sample with a
    // 0.3 m bias targets -3.3, not -3.0.
    EXPECT_NEAR(overlume::testing::terrain_ground_offset_probe(3.0, 0.0, 0.3, 0.0, 0.0f, true),
                -3.3, 1e-9);
    // Clamp: a 500 m sampled height (way outside the ±30 m ceiling) still
    // clamps to -30.0, not -500.0.
    EXPECT_NEAR(overlume::testing::terrain_ground_offset_probe(500.0, 0.0, /*ground_bias_m=*/0.0,
                                                               0.0, 0.0f, true),
                -30.0, 1e-9);
    // Smoothing (not snapping) moves the offset PART of the way to the
    // target over one 0.5 s tick (the time constant) -- strictly between
    // the start and the target, never past it in one step.
    const double stepped = overlume::testing::terrain_ground_offset_probe(
        3.0, 0.0, /*ground_bias_m=*/0.0, /*current_offset_m=*/0.0,
        /*delta_seconds=*/0.5f, /*snap=*/false);
    EXPECT_GT(stepped, -3.0);
    EXPECT_LT(stepped, 0.0);
}

// Design item 6: the real end-to-end path -- installs the ground-height
// fixture (tile_ground.b3dm sits at ellipsoid height 3.0 m; the anchor's
// own origin_height_m is 0.0), places the ego at the fixture's own block
// center (0,0,0), and pumps render_frame() until the smoothed offset
// converges on -3.0 m (design item 3: target = -(sampled_height -
// anchor_height) = -(3.0 - 0.0)). Bounded at 600 frames with a small sleep
// per tick -- sampleHeightMostDetailed() is a real async cesium request
// (needs the ground tile to have loaded first), same "bounded pump + sleep"
// shape pump_and_settle() above uses for its own async-latency tests.
//
// Gate round 1 finding 1: offset convergence alone does not prove the
// shift actually reached Filament (groundOffsetZ_ is bookkeeping this
// class keeps regardless of whether the apply call ran) -- so this also
// reads back the REAL world-space transform of the first tracked tile
// BEFORE and AFTER convergence and requires the same ~-3.0 m delta.
// Deleting tm.setParent(assetRoot, terrainRoot) (environment_stream.cpp,
// prepareInMainThread) or renderResources_->set_ground_offset_z()
// (update_ground_offset()) each make this delta stay ~0 -- see
// first_tracked_tile_world_z()'s own comment (environment_stream.hpp).
TEST(EnvironmentStream, FollowTerrainShiftsEnvironmentToGroundUnderEgo) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(
        r, kSessionGroundFixtureDir.c_str(), kSessionGroundAnchor, /*materials_original=*/false,
        /*follow_terrain=*/true));

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;  // (0,0,0) -- the anchor itself
    overlume::set_scene(r, s);

    // 2026-09-21 two-frustum tile selection: this test's own job is the
    // synthetic (ego-following) view's terrain-follow convergence + a
    // STABLE single tracked tile to read the before/after world-Z delta off
    // of (inScene_.begin()->first -- see first_tracked_tile_world_z()'s own
    // comment) -- kStdPose sits near this fixture's own anchor too, so as a
    // SECOND selection frustum it would load additional tiles over the
    // course of this test and can shift which tile "first" resolves to
    // between the two readings, confounding the delta with two different
    // tiles' own placement instead of one tile's real shift. Same
    // "point the camera away, exercise the code path, contribute nothing
    // geometrically" isolation FixtureTilesLoadRenderAsClayAndCount's own
    // settle_count() uses above.
    // 1 mm lateral offset keeps the straight-up lookAt() non-degenerate.
    const overlume::CameraPose awayPose{{0, 0, 5000}, {0.001, 0, 5001}, 60};

    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GT(pump_until_loaded(r, awayPose, buf), 0u)
        << "session_ground fixture must load from its own anchor's block center";
    const double worldZBefore = overlume::testing::environment_terrain_first_tile_world_z(r);
    ASSERT_FALSE(std::isnan(worldZBefore))
        << "a tracked tile must expose a real Filament transform once loaded";

    // Tolerance 0.1 m, not the design's own 0.05: cesium's height sampler
    // raycasts against the tile's own glTF POSITION accessor, encoded
    // float32 at full ECEF magnitude (~6.4e6 m, make_tile_fixture.py's
    // f32_bytes()) -- ~0.05-0.08 m of float32 quantization noise on the
    // sampled height is inherent to that encoding (every streamed tile in
    // every fixture this project ships uses it, not something this test
    // can special-case away) and reproduces deterministically (fixed seed,
    // fixed anchor) rather than flaking.
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

// Design item 3's last sentence: "When follow_terrain is off, nothing is
// sampled and the offset stays 0" -- same fixture as the test above (design
// item 6), follow_terrain=false, same pump bound, proving it's a true
// no-op end to end, not just in the isolated math above.
TEST(EnvironmentStream, FollowTerrainOffStaysAtZero) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(
        r, kSessionGroundFixtureDir.c_str(), kSessionGroundAnchor, /*materials_original=*/false,
        /*follow_terrain=*/false));
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

// ── 2026-09-21 live finding: clay ground plane vs streamed roadside detail
// ──────────────────────────────────────────────────────────────────────
// A tileset only earns the right to take the renderer's own 120 m clay
// ground plane out of the scene by EVIDENCE (a successful terrain height
// sample under the ego), never by preset name -- see
// EnvironmentSource::provides_ground()'s own comment (environment.hpp).
// Same session_ground fixture/anchor/pump-until-converged shape as
// FollowTerrainShiftsEnvironmentToGroundUnderEgo above (a real
// sampleHeightMostDetailed() round trip, bounded at 600 frames + a small
// per-tick sleep, pointed at an away pose so the loaded tile set stays
// stable across ticks).
TEST(EnvironmentStream, GroundPlaneHiddenWhenTilesetSuppliesGround) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(
        r, kSessionGroundFixtureDir.c_str(), kSessionGroundAnchor, /*materials_original=*/false,
        /*follow_terrain=*/true));

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);

    // 1 mm lateral offset keeps the straight-up lookAt() non-degenerate --
    // same away-pose isolation FollowTerrainShiftsEnvironmentToGroundUnderEgo
    // uses above.
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

// The OSM-Buildings analogue: kTilesFixtureDir has NO ground quad (buildings
// only), so even with follow_terrain=true, sampleHeightMostDetailed() never
// hits anything under the ego -- the clay plane must stay.
TEST(EnvironmentStream, GroundPlaneKeptWhenTilesetIsBuildingsOnly) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(
        r, kTilesFixtureDir.c_str(), kFixtureAnchor, /*materials_original=*/false,
        /*follow_terrain=*/true));
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);
    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GT(pump_until_loaded(r, kStdPose, buf), 0u);
    // Same bound as the test above -- buildings-only tiles never sample
    // ground, so this just proves repeated tries don't flip it. 200 ticks at
    // 10 ms is ~2 s, a full kTerrainSampleIntervalS window of resampling --
    // a longer bound only burns wall clock (review minor, 2026-09-21).
    for (int i = 0; i < 200; ++i) {
        overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_FALSE(overlume::testing::environment_stream_provides_ground(r));
    EXPECT_TRUE(overlume::testing::renderer_ground_plane_in_scene(r));
    overlume::destroy_renderer(r);
}

// Opus gate fix round (2026-09-21): provides_ground() must go false once
// fallen back, even though ground_hit is a one-way latch -- fall_back()
// tears the tileset down but never resets terrainState_, so without a
// !fallenBack_ guard the clay plane would stay hidden forever after a
// network-loss fallback and the robot would render over a void.
//
// Get a real ground hit first (same session_ground fixture + away-pose
// pump as GroundPlaneHiddenWhenTilesetSuppliesGround, so the latch is
// PROVEN true, not assumed), then drive a REAL fall_back() via
// environment_stream_force_fall_back() rather than racing
// kNetworkLossConsecutiveFailures through kill_fixture_network(): empirically
// (verified live while writing this test, and consistent with
// NetworkDeadFromFirstRequestFallsBackToBakedChunksOnce's own "Gap" comment
// below), once a tile has loaded once, every subsequent
// sampleHeightMostDetailed() call is served from the warmed on-disk sqlite
// cache and never touches the accessor again -- so a fixture that has
// already proven ground_hit true cannot be raced into STREAMING_FALLBACK
// through the real counting path inside one short test process; up to 1500
// ticks / 15 real seconds of continued resampling after killing the fixture
// "network" were observed to never trip it. force_fall_back_for_testing()
// calls the SAME fall_back() a real trip would, so this still exercises the
// real teardown/fallenBack_/fallbackSource_ path, just reached
// deterministically. Fails if the `!fallenBack_` term is removed from
// provides_ground().
TEST(EnvironmentStream, GroundPlaneReturnsAfterNetworkLossFallback) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* killable = overlume::testing::install_fixture_streaming_source_with_fallback(
        r, kSessionGroundFixtureDir.c_str(), kTestTownDir.c_str(), kSessionGroundAnchor,
        /*follow_terrain=*/true);
    ASSERT_NE(killable, nullptr);

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);

    // Away pose, same isolation reason as GroundPlaneHiddenWhenTilesetSuppliesGround.
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
    // One more tick: render_frame's reconcile (renderer.cpp) re-checks
    // provides_ground() and puts r->ground back into the scene when it goes
    // false -- the reconcile itself is not re-run just by calling fall_back().
    overlume::render_frame(r, awayPose, {buf.data(), 320, 240});

    EXPECT_TRUE(overlume::testing::renderer_ground_plane_in_scene(r))
        << "the clay plane must return once the streamed source has fallen back -- the ground "
           "hit latch must not survive Decision 11's network-loss fallback";
    overlume::destroy_renderer(r);
}

// Design item 4c: replaces_ground= parses the same way follow_terrain=
// does -- "off" -> false, absent -> true (today's default), any other
// value fails the WHOLE parse.
TEST(EnvironmentStream, ParseIonSpecReplacesGround) {
    bool ok = false;
    EXPECT_FALSE(overlume::testing::environment_stream_parse_replaces_ground(
        "96188?replaces_ground=off", &ok));
    EXPECT_TRUE(ok);
    EXPECT_TRUE(overlume::testing::environment_stream_parse_replaces_ground("96188", &ok));
    EXPECT_TRUE(ok);  // absent key -- parses fine, defaults true
    overlume::testing::environment_stream_parse_replaces_ground("96188?replaces_ground=maybe", &ok);
    EXPECT_FALSE(ok);  // malformed value -- the WHOLE spec fails to parse
}

// ── Step 1: original mode skips the clay remap ──────────────────────────
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
    ASSERT_TRUE(overlume::testing::install_fixture_streaming_source(
        r, kTilesFixtureDir.c_str(), kFixtureAnchor, /*materials_original=*/true));
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);
    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GT(pump_until_loaded(r, kStdPose, buf), 0u);
    EXPECT_FALSE(overlume::testing::environment_stream_first_primitive_is_clay(r));
    // Fresh-opaque convention unaffected: original mode still casts/receives
    // shadows via the SAME renderable-manager calls (unconditional in
    // prepareInMainThread) -- nothing fade-blended is introduced here.
    overlume::destroy_renderer(r);
}

// ── Task 5 Step 4: live perf, google preset vs OSM-clay preset ──────────
// Opt-in and self-skipping BY DESIGN (Global Constraints' network
// discipline: "no ctest/gtest ever requires live network or the token") --
// this test SKIPs cleanly whenever CESIUM_ION_TOKEN is unset OR the
// separate OVERLUME_LIVE_ION_PERF opt-in is unset, so a normal ctest/
// ci_visual_mode.sh run (neither set) never depends on either. VM-064 gate
// round 1 finding: this used to be unrunnable even with both vars set in a
// full-binary run -- two earlier tests unsetenv("CESIUM_ION_TOKEN") for the
// whole process (now fixed via ScopedUnsetEnv's save/restore above), so a
// full run with both vars exported now reaches this test with the token
// still present. --gtest_filter stays the recommended way to run it in
// isolation (deliberate opt-in, not a workaround for that bug):
// (`CESIUM_ION_TOKEN=... OVERLUME_LIVE_ION_PERF=1
// ./test_environment_stream --gtest_filter='*GooglePresetLive*'`) to
// record the real dev-box number this task's results block wants.
TEST(EnvironmentStreamPerf, GooglePresetLiveRenderMsDeltaVsOsmClay) {
    if (std::getenv("CESIUM_ION_TOKEN") == nullptr ||
        std::getenv("OVERLUME_LIVE_ION_PERF") == nullptr) {
        GTEST_SKIP() << "opt-in live-network perf check -- set CESIUM_ION_TOKEN and "
                        "OVERLUME_LIVE_ION_PERF=1 to run (never required by ctest)";
    }
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -300, 300}, {0, 0, 0}, 60.0};
    std::vector<uint8_t> buf(320u * 240u * 3u);
    overlume::GeoAnchor anchor{25.0803, 55.3910, 0.0};  // Epic 4 Decision 6's verified fix location

    // Bounded by wall-clock, not just frame count -- a live network call
    // that never resolves must not hang this opt-in run indefinitely.
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
        // Manual visual-confidence artifact only (not a golden -- no
        // committed comparison target, ssim result discarded): lets a human
        // eyeball whether original-materials mode actually shows textured
        // content vs. a blank/untextured mesh. Never asserted on.
        overlume::testing::render_and_compare(r, pose, "/nonexistent_no_golden.png",
                                              "/tmp/environment_stream_live_actual.png");
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

// ── VM-096 comparison package: opt-in env-source capture tests ──────────
// One human-eyeball render per GUI environment preset ("baked"/"osm"/
// "clipped"/"google", VM-096), all at the SAME geo anchor (kFixtureAnchor,
// already this file's own anchor constant above) and the SAME camera pose,
// so a side-by-side comparison actually compares the sources and nothing
// else. Gated on OVERLUME_CAPTURE_ENV_SOURCES=1, same opt-in shape as
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
constexpr overlume::Vec3 kCaptureBuildingsCentroid{-109.2, -17.1, 3.0};
// Same eye/target offset test_environment.cpp's
// SetEnvironmentVisibleFalseHidesLoadedChunksWithoutTearingDown already
// uses to frame that exact centroid (+50/-70/40 eye offset, its own
// comment: "frames the real footprint centroid so the buildings occupy a
// real chunk of the image, not a corner") -- reused verbatim, not
// re-derived, as the shared pose every source below is captured from.
const overlume::CameraPose kCapturePose{
    {kCaptureBuildingsCentroid.x + 50, kCaptureBuildingsCentroid.y - 70, 40},
    {kCaptureBuildingsCentroid.x, kCaptureBuildingsCentroid.y, kCaptureBuildingsCentroid.z},
    60.0};

// Generous size (per the task): these PNGs are for a human to look at side
// by side, not for SSIM -- golden.hpp's render_and_compare() is fixed at
// 320x240 for CI speed and is deliberately NOT used here.
constexpr uint32_t kCaptureWidth = 960;
constexpr uint32_t kCaptureHeight = 720;

// OVERLUME_CAPTURE_OUT_DIR lets a caller redirect where the PNG lands; default
// /tmp -- these are opt-in manual captures, not committed test artifacts.
// The capture note (docs/runbooks/env_source_captures.md) documents
// copying the result into docs/runbooks/env_source_captures/ for the
// comparison package itself.
std::string capture_out_path(const char* name) {
    const char* dir = std::getenv("OVERLUME_CAPTURE_OUT_DIR");
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

// Renders ONE frame of `r`'s current scene from kCapturePose at the
// generous kCaptureWidth x kCaptureHeight, writes it to
// capture_out_path(name) via the same stb writer golden.cpp links into
// this binary, and logs the content-verification numbers to stderr.
void capture_and_report(overlume::VisualRenderer* r, const char* name) {
    std::vector<uint8_t> buf(static_cast<size_t>(kCaptureWidth) * kCaptureHeight * 3);
    ASSERT_TRUE(
        overlume::render_frame(r, kCapturePose, {buf.data(), kCaptureWidth, kCaptureHeight}));
    const std::string outPath = capture_out_path(name);
    // Gate round 1 finding: OVERLUME_CAPTURE_OUT_DIR is never told to exist by
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
                        << " (does OVERLUME_CAPTURE_OUT_DIR exist?)";
    const ContentStats stats = analyze_capture(buf, kCaptureWidth, kCaptureHeight);
    std::cerr << "[EnvSourceCapture] " << name << " -> " << outPath
              << " luminance_stddev=" << stats.luminance_stddev
              << " non_background_fraction=" << stats.non_background_fraction << "\n";
}

}  // namespace

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
    // Gate round 1 finding: must match Osm/Google's ego position ({0,0,0}), not
    // kChunk0Center -- an ego-anchored ground element is the largest/brightest
    // feature in these dark-theme captures, and differing ego across presets
    // moved it to a different corner of the frame, manufacturing a large
    // apparent "difference" that had nothing to do with the environment
    // source. Verified the fixture chunks still load fine (loaded>0 below,
    // town renders unchanged) at {0,0,0}.
    s.ego.position = overlume::Vec3{0, 0, 0};
    overlume::set_scene(r, s);
    std::vector<uint8_t> warm(static_cast<size_t>(kCaptureWidth) * kCaptureHeight * 3);
    ASSERT_GT(
        pump_and_settle(r, kCapturePose, warm, /*initial_deadline_sec=*/5, /*settle_seconds=*/1),
        0u)
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
    const uint64_t loaded =
        pump_and_settle(r, kCapturePose, warm, /*initial_deadline_sec=*/30, /*settle_seconds=*/60);
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
    const uint64_t loaded =
        pump_and_settle(r, kCapturePose, warm, /*initial_deadline_sec=*/30, /*settle_seconds=*/60);
    EXPECT_GT(loaded, 0u)
        << "google preset (ion://2275207) loaded no tiles at kFixtureAnchor within "
           "30s -- would ship an empty/near-empty capture, reporting rather than "
           "faking it";
    capture_and_report(r, "google");
    overlume::destroy_renderer(r);
}

// "clipped" (VM-096's 4th GUI preset) resolves to the NODE's own
// environment_own_asset_uri parameter (resolved server-side in
// tools/vcam_ws_bridge.py's set_environment_source branch), not a
// fixed public ion asset id -- and that parameter defaults empty
// (declared at overlume_node.cpp:687) and is NOT configured on this box. There is
// no real ion asset id to render here: fabricating one would silently ship
// a picture of the WRONG preset (some other asset entirely), which is
// worse than no picture. Always skips, even with the capture opt-in set,
// with that exact reason -- the case is named and reported as
// not-renderable, never silently dropped from the comparison package.
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

// Fixture provenance (VM-097): tests/fixtures/environment_tiles_fixture_0/ --
// see that directory's own PROVENANCE.md. Synthesized locally by
// overlume/scripts/make_tile_fixture.py (fixed seed, deterministic); no
// Cesium ion/OSM Buildings content, no token, no network. Task 4's own
// environment_tiles_fixture_fallback_0/PROVENANCE.md documents the
// duplicated-tile fixture used by the network-loss e2e above.
