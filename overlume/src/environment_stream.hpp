// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// environment_stream.hpp — library-internal AND C++20-only (Decision 3,
// docs/plans/2026-08-18-visual-mode-epic6.md). Included by
// environment_stream.cpp and NOTHING else: the static_assert below makes
// that enforced, not just documented, and no other TU in this project has
// the cesium include dirs needed to even find these headers (they exist
// only on the overlume_stream OBJECT library, CMakeLists.txt).
#pragma once

static_assert(__cplusplus >= 202002L,
              "environment_stream.hpp is C++20-only -- include it from "
              "environment_stream.cpp, nowhere else");

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <glm/mat4x4.hpp>

#include <Cesium3DTilesSelection/IPrepareRendererResources.h>
#include <Cesium3DTilesSelection/Tile.h>
#include <Cesium3DTilesSelection/Tileset.h>
#include <Cesium3DTilesSelection/TilesetExternals.h>
#include <Cesium3DTilesSelection/TilesetViewGroup.h>
#include <CesiumAsync/AsyncSystem.h>
#include <CesiumAsync/IAssetAccessor.h>
#include <CesiumAsync/IAssetResponse.h>
#include <CesiumAsync/ITaskProcessor.h>
#include <CesiumGeospatial/LocalHorizontalCoordinateSystem.h>

#include <utils/Entity.h>

#include "environment.hpp"
#include "overlume/scene.h"

namespace overlume {

// ── Named constants (Decision 9, dev-box proxies -- same honesty class as
//    environment.hpp's kLoadRadiusM) ─────────────────────────────────────
inline constexpr double kStreamViewHeightM = 300.0;  // synthetic-camera height above ego
inline constexpr int kStreamViewportPx = 256;
inline constexpr double kStreamViewFovRad = 1.3;  // vertical AND horizontal (square viewport)
inline constexpr double kStreamMaxSseErr = 48.0;
// 2026-09-21 finding (docs/status.md item 4 addendum): the synthetic nadir
// view above stayed the ONLY selection frustum through VM-097, and at its
// own geometry -- 256 px viewport, kStreamViewHeightM=300 m range,
// kStreamViewFovRad=1.3 rad half-angle-ish fov -- screen-space error for a
// tile of geometric error `ge` works out to `ge * 256 / (2 * 300 *
// tan(0.65)) ~= 0.56 * ge`, so tiles only refine past kStreamMaxSseErr=48
// once their OWN geometric error drops below ~85 m: Google Photorealistic
// ground renders as a blurry, coarse mesh sitting metres above the real
// surface, and the terrain follower then samples the MOST detailed height
// under the ego (Tileset::sampleHeightMostDetailed) and lifts that coarse
// mesh clean through the road.
//
// Fix: synthesize_view_and_pump() now selects tiles with the real render
// camera's OWN ViewState as a SECOND frustum (camera_view_state(), just
// above that function) alongside this synthetic one -- standard Cesium
// usage (a tileset can be driven by more than one simultaneous view). Tiles
// the real camera can actually see refine to real detail through that
// frustum; the synthetic nadir view keeps doing its original job -- a wide
// top-down coverage frustum so everything past the camera's view stays
// loaded at coarse LOD instead of unloading and popping back in. Because
// render_frame() calls environmentSource->update() BEFORE this frame's
// camera->lookAt()/setProjection() (renderer.cpp), the camera frustum used
// here is always ONE FRAME STALE (the previous frame's pose) -- acceptable
// lag, noted here and at the call site.
//
// kStreamMaxSseErr stays 48 for BOTH frustums rather than getting its own
// tighter camera-frustum threshold: the camera frustum's own geometry
// already does the refining. At a typical ~20 m range from a ~720 px-tall
// render viewport with a ~50 deg vertical fov, screen-space error for
// geometric error `ge` is roughly `ge * 720 / (2 * 20 * tan(25 deg)) ~= 39
// * ge` -- nearly two orders of magnitude steeper than the synthetic view's own 0.56
// factor above -- so the SAME 48 threshold already refines tiles near the
// camera down to ~1-2 m geometric error without touching the constant.
// ponytail: two fixed frustums (synthetic coverage + real camera); a
// LOD-budget-aware N-frustum scheme is the upgrade if a future render mode
// wants more than one live camera at once.
inline constexpr double kStreamHeightOffsetM = 0.0;  // Decision 8: absorbs any
                                                     // measured float/sink, none measured yet.
inline constexpr uint64_t kDefaultMaxCacheItems = 4096;
// Decision 11 (VM-063): consecutive completed-with-error requests, zero
// interleaved successes, before network loss is declared and the source
// falls back to its baked dir. ponytail: consecutive-failure counter; a
// time-windowed health score is the upgrade if flapping links need
// hysteresis.
inline constexpr int kNetworkLossConsecutiveFailures = 8;
// Bounded wait for cesium's async tile-destruction completion at teardown
// (Decision, Task 3 Step 2) -- each iteration pumps
// asyncSystem.dispatchMainThreadTasks() once. Hitting the bound means
// cesium's async work is wedged; we leak its in-flight tiles rather than
// hang destroy_renderer(), and the leak is counted + logged.
// ponytail: bounded join with a leak-and-log fallback, not a hang.
inline constexpr int kTeardownPumpBound = 2000;

// ── Terrain following (2026-09-21 maintainer decision, "option 2"): the map
//    frame is flat (ego z always 0) but real terrain is not -- these tune
//    how often/how far the ego must move before re-sampling Google's own
//    terrain height under it, and how the streamed environment's root
//    entity chases that sample. ──────────────────────────────────────────
inline constexpr double kTerrainSampleMoveThresholdM =
    5.0;  // re-sample once ego moves this far (map x/y)
inline constexpr double kTerrainSampleIntervalS = 2.0;  // or once this much time has passed
inline constexpr double kTerrainOffsetClampM = 30.0;    // clamp |ground offset| to this many metres
inline constexpr double kTerrainSmoothTimeConstantS = 0.5;  // first-order smoothing time constant
// 2026-09-21 maintainer decision (multi-point plane fit): the fitted along-
// track grade is clamped to this tilt before it's applied to the terrain
// root -- a linear fit extrapolates forever, so far outside the fitted span
// (well past +-20 m) an unclamped tilt would rotate the environment by an
// ever-growing angle for one noisy/steep sample. The clamp bounds both how
// wrong the far field can get AND how far streamed buildings visibly lean
// (a tilt is a whole-scene rotation, not just a ground correction). Default
// 2 degrees; `max_tilt_deg=0` on the ion:// URI (IonSpec) disables tilt
// entirely (offset-only, byte-identical to the pre-fit single-scalar
// behaviour) -- the escape hatch.
inline constexpr double kTerrainMaxTiltRad = 2.0 * M_PI / 180.0;

// Defined in environment_stream.cpp, just above StreamingEnvironmentSource's
// own terrain-sampling methods -- only forward-declared here so it can be
// named by a std::shared_ptr<TerrainFollowState> member below (incomplete
// type is fine for a shared_ptr member; the .cpp TU has the full definition
// wherever it constructs/destroys one, including this class's own
// out-of-line, defaulted destructor).
struct TerrainFollowState;

// ── A minimal fixed-size thread pool ITaskProcessor (Decision 2's "~20
//    lines"; a std::thread pool, nothing fancier -- ponytail: no work
//    stealing/priority, upgrade if tile-load latency ever needs it) ──────
class SimpleTaskProcessor final : public CesiumAsync::ITaskProcessor {
public:
    explicit SimpleTaskProcessor(int threadCount = 2) {
        for (int i = 0; i < threadCount; ++i) {
            workers_.emplace_back([this] { workerLoop(); });
        }
    }
    ~SimpleTaskProcessor() override {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        cv_.notify_all();
        for (auto& t : workers_) {
            if (t.joinable()) t.join();
        }
    }
    void startTask(std::function<void()> f) override {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push_back(std::move(f));
        }
        cv_.notify_one();
    }

private:
    void workerLoop() {
        for (;;) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
                if (stop_ && queue_.empty()) return;
                task = std::move(queue_.front());
                queue_.pop_front();
            }
            task();
        }
    }
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::function<void()>> queue_;
    std::vector<std::thread> workers_;
    bool stop_ = false;
};

// ── CountingAssetAccessor (Decision 11 / Task 3 Step 2): decorator that
//    counts consecutive completed-with-error requests. The COUNT is used
//    by Task 4's fallback trigger; this task only builds the counter and
//    exposes it (kNetworkLossConsecutiveFailures lives here since the
//    counting is this task's job, the ACTING on it is Task 4's). ────────
class CountingAssetAccessor final : public CesiumAsync::IAssetAccessor {
public:
    explicit CountingAssetAccessor(std::shared_ptr<CesiumAsync::IAssetAccessor> inner)
        : inner_(std::move(inner)) {}

    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> get(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& url,
        const std::vector<THeader>& headers) override {
        return inner_->get(asyncSystem, url, headers)
            .thenImmediately([this](std::shared_ptr<CesiumAsync::IAssetRequest>&& req) {
                note(req.get());
                return req;
            });
    }
    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> request(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& verb,
        const std::string& url, const std::vector<THeader>& headers,
        const std::span<const std::byte>& payload) override {
        return inner_->request(asyncSystem, verb, url, headers, payload)
            .thenImmediately([this](std::shared_ptr<CesiumAsync::IAssetRequest>&& req) {
                note(req.get());
                return req;
            });
    }
    void tick() noexcept override { inner_->tick(); }

    // Consecutive completed-with-error requests, zero interleaved
    // successes -- Decision 11's kNetworkLossConsecutiveFailures counter.
    int consecutive_failures() const { return consecutiveFailures_.load(); }

private:
    void note(const CesiumAsync::IAssetRequest* req) {
        const CesiumAsync::IAssetResponse* resp = req ? req->response() : nullptr;
        const bool ok = resp != nullptr && resp->statusCode() >= 200 && resp->statusCode() < 300;
        if (ok) {
            consecutiveFailures_.store(0);
        } else {
            consecutiveFailures_.fetch_add(1);
        }
    }
    std::shared_ptr<CesiumAsync::IAssetAccessor> inner_;
    std::atomic<int> consecutiveFailures_{0};
};

// ── Test-only fixture accessors (Decision 13; environment_test_hooks.hpp's
//    opaque FixtureStreamHandle wraps one of these). Serves
//    tests/fixtures/environment_tiles_fixture_0/ from disk -- no network, no
//    token. KillableFixtureAccessor additionally honors a kill switch
//    (Task 3 Step 4's cache-offline proof; Task 4's network-loss e2e). ───
class FileFixtureAssetAccessor : public CesiumAsync::IAssetAccessor {
public:
    // `killed` is owned by the caller (the FixtureStreamHandle) so
    // kill_fixture_network() can flip it after construction; nullptr means
    // "never killable" (plain install_fixture_streaming_source()).
    explicit FileFixtureAssetAccessor(std::shared_ptr<std::atomic<bool>> killed = nullptr)
        : killed_(std::move(killed)) {}

    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> get(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& url,
        const std::vector<THeader>& /*headers*/) override {
        return asyncSystem.createResolvedFuture(makeRequest("GET", url));
    }
    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> request(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& verb,
        const std::string& url, const std::vector<THeader>& /*headers*/,
        const std::span<const std::byte>& /*payload*/) override {
        return asyncSystem.createResolvedFuture(makeRequest(verb, url));
    }
    void tick() noexcept override {}

private:
    std::shared_ptr<CesiumAsync::IAssetRequest> makeRequest(const std::string& verb,
                                                            const std::string& url);
    std::shared_ptr<std::atomic<bool>> killed_;
};

// ── IPrepareRendererResources (Decision 7): re-serializes each tile's
//    CesiumGltf::Model to GLB once per tile on the load thread, then feeds
//    it through the EXACT SAME createAsset -> loadResources ->
//    releaseSourceData -> buildingMaterial remap sequence
//    BakedEnvironmentSource::update() uses (environment.cpp:63-84), on the
//    main thread. Raster-overlay hooks are no-ops: OSM Buildings/clay
//    streaming uses no raster overlays. ──────────────────────────────────
class StreamRendererResources final : public Cesium3DTilesSelection::IPrepareRendererResources {
public:
    // `anchorHeightM` (2026-09-21 finding, docs/status.md item 4): the
    // anchor's own WGS84 ellipsoid height (GeoAnchor::origin_height_m),
    // threaded through to prepareInLoadThread's per-vertex height
    // correction (strip_attributes_and_correct_heights()) -- 0.0 keeps the
    // previous (simulation-only-correct) behaviour.
    // `materialsOriginal` (VM-064, Task 5): false (default) is today's clay
    // remap; true skips it in prepareInMainThread() -- gltfio's own loaded
    // ubershader materials stay bound (Google Photorealistic 3D Tiles).
    explicit StreamRendererResources(const glm::dmat4& ecefToMap, double anchorHeightM,
                                     bool materialsOriginal = false)
        : ecefToMap_(ecefToMap),
          anchorHeightM_(anchorHeightM),
          materialsOriginal_(materialsOriginal) {}

    // `r` is only valid for the duration of the StreamingEnvironmentSource
    // call that supplied it (update()/teardown() -- the seam's own
    // per-call contract, environment.hpp) -- set fresh at the top of each
    // such call, never stored beyond it by the caller.
    void set_renderer(VisualRenderer* r) { r_ = r; }

    CesiumAsync::Future<Cesium3DTilesSelection::TileLoadResultAndRenderResources>
    prepareInLoadThread(const CesiumAsync::AsyncSystem& asyncSystem,
                        Cesium3DTilesSelection::TileLoadResult&& tileLoadResult,
                        const glm::dmat4& transform, const std::any& rendererOptions) override;
    void* prepareInMainThread(Cesium3DTilesSelection::Tile& tile, void* pLoadThreadResult) override;
    void free(Cesium3DTilesSelection::Tile& tile, void* pLoadThreadResult,
              void* pMainThreadResult) noexcept override;

    // Raster overlays: unused by OSM Buildings/clay streaming (Decision 7)
    // -- trivial no-ops, never invoked in practice.
    void* prepareRasterInLoadThread(CesiumImage::ImageAsset&, const std::any&) override {
        return nullptr;
    }
    void* prepareRasterInMainThread(CesiumRasterOverlays::RasterOverlayTile&, void*) override {
        return nullptr;
    }
    void freeRaster(const CesiumRasterOverlays::RasterOverlayTile&, void*,
                    void*) noexcept override {}
    void attachRasterInMainThread(const Cesium3DTilesSelection::Tile&, int32_t,
                                  const CesiumRasterOverlays::RasterOverlayTile&, void*,
                                  const glm::dvec2&, const glm::dvec2&) override {}
    void detachRasterInMainThread(const Cesium3DTilesSelection::Tile&, int32_t,
                                  const CesiumRasterOverlays::RasterOverlayTile&,
                                  void*) noexcept override {}

    // Drains the deferred-free queue at the top of update() (Filament is
    // single-threaded; free() can arrive from any thread per its own
    // contract, so the actual destroyAsset happens here instead). Returns
    // the FilamentAsset pointers actually destroyed by this call (empty if
    // r_ is unset, in which case nothing was destroyed) -- VM-062 gate
    // round 1, Finding 3: the caller needs this list to evict the same
    // pointers from StreamingEnvironmentSource::inScene_ before its next
    // reconcile pass dereferences one of them post-free.
    std::vector<filament::gltfio::FilamentAsset*> drain_pending_frees();
    void note_torn_down() { tornDown_.store(true); }

    // 2026-09-21 terrain-following (option 2): every streamed asset root is
    // parented under this ONE entity (prepareInMainThread(), after the
    // per-asset placement transform is set) so a single transform here
    // moves/tilts the whole streamed environment. Created lazily -- on the
    // first prepareInMainThread() call that has an `r_` to create it with --
    // rather than at StreamRendererResources construction time, since the
    // Filament engine isn't available yet then. Identity (no-op) until
    // set_terrain_transform() is first called, so a follow_terrain=off
    // source is visually unaffected. Destroyed by destroy_terrain_root(),
    // called from StreamingEnvironmentSource::teardown() after every
    // streamed asset (its children) has already been torn down -- same
    // ordering discipline as the rest of that function.
    //
    // 2026-09-21 multi-point plane fit: renamed from set_ground_offset_z(),
    // which only ever built translation({0,0,z}) -- a single global height
    // shift. The fitted grade now also needs a TILT about the across-track
    // axis, pivoted at the ego position the fit was sampled at (not the
    // live ego position -- a moving pivot would sway the whole world with
    // every ego jitter). `pivot_x`/`pivot_y` are that sample-time position
    // (map frame); `heading_rad` is the ego heading the fit was taken
    // against, which defines the across-track (lateral) axis
    // `u = {-sin(heading), cos(heading), 0}` the tilt rotates about. Builds
    // `translation(P) * rotation(tilt_rad, u) * translation(-P) *
    // translation({0,0,z})` with `P = {pivot_x, pivot_y, 0}` -- `tilt_rad ==
    // 0` (max_tilt_deg=0, or a fit with slope 0) reduces this to exactly the
    // old translation-only matrix.
    void set_terrain_transform(double z, double tilt_rad, double pivot_x, double pivot_y,
                               double heading_rad);
    void destroy_terrain_root();

private:
    void ensure_terrain_root();

    VisualRenderer* r_ = nullptr;
    glm::dmat4 ecefToMap_;
    double anchorHeightM_ = 0.0;      // 2026-09-21 finding: anchor's own WGS84 ellipsoid height
    bool materialsOriginal_ = false;  // VM-064: gates the clay remap, see prepareInMainThread()
    std::mutex freeMutex_;
    std::vector<filament::gltfio::FilamentAsset*> pendingFrees_;
    std::atomic<bool> tornDown_{false};
    utils::Entity terrainRoot_;  // invalid (default) until ensure_terrain_root()
};

// Holds the GLB bytes produced in the load thread + parse errors, handed
// from prepareInLoadThread's future to prepareInMainThread as the
// `pLoadThreadResult` void*.
struct LoadThreadGlb {
    std::vector<std::byte> glbBytes;
    // The transform passed to prepareInLoadThread -- NOT the same as
    // tile.getTransform() read later in prepareInMainThread. Confirmed
    // empirically: tile.getTransform() is only this tileset's own
    // tileset.json-declared structural transform (identity here, since
    // our fixture sets none); the content-specific RTC_CENTER a b3dm's
    // OWN feature table carries (only knowable after parsing that
    // content) is folded into THIS parameter instead, computed by the
    // content loader once the content is available. Using
    // tile.getTransform() alone placed geometry at the ECEF origin
    // (Earth's center) offset by nothing -- millions of meters from the
    // anchor -- instead of at the tile's real position.
    glm::dmat4 transform{1.0};
    bool ok = false;
};

// Computes the rigid ECEF -> map-frame transform (Decision 8): ENU at the
// anchor (east/north/up) inverted to ecefToLocal, then rotated so
// ENU-east/north align with the map frame's own axes at `heading_rad`
// (the SAME convention geo_anchor.cpp's WgsToMap implements node-side --
// this is that same linear map, expressed as a 4x4 instead of two scalar
// formulas, so it can premultiply a tile's own ECEF transform once per
// asset root instead of per-vertex). The ENU origin's own height is
// `anchor.origin_height_m` (2026-09-21 finding, docs/status.md item 4) --
// previously hard-coded 0.0, which is only correct in simulation.
glm::dmat4 compute_ecef_to_map(const GeoAnchor& anchor);

// Open Follow-up 4 (docs/status.md item 4): the ONE implementation of the
// ellipsoid-height correction formula -- given a point already in ECEF,
// keeps x/y from the rigid `ecefToMap` transform and replaces z with
// z_map = (ellipsoid height at this point) - anchor_height_m, i.e.
// `carto.height - anchor_height_m`. 2026-09-21 finding (docs/status.md item
// 4): `anchor_height_m` was hard-coded 0.0 here on the (only
// simulation-true) assumption that the anchor sits exactly on the
// ellipsoid; on the real robot the anchor's own WGS84 ellipsoid height is
// ~1.7 m (Fixposition NavSatFix altitude), so Google's streamed terrain
// rendered ~1.7 m above the road until this was made a real parameter.
// Callers pass the SAME GeoAnchor::origin_height_m compute_ecef_to_map()
// used to build `ecef_to_map` -- both `strip_attributes_and_correct_heights()`
// (environment_stream.cpp, per real glTF vertex at tile load) and the
// `ecef_height_correction_probe()` test hook call this SAME function --
// gate round 1 minor finding: the two must not be independently-typed
// formulas that can silently drift apart. This function only ever sees an
// already-ECEF point, so it is unaffected by the 2026-09-21 Google
// node-matrix finding (glTF node-local positions under a per-primitive node
// transform, see strip_attributes_and_correct_heights()'s own comment) --
// getting the point INTO ECEF correctly is the caller's job, not this one's.
glm::dvec3 correct_ecef_point_height(const glm::dvec3& ecef_pos, const glm::dmat4& ecef_to_map,
                                     double anchor_height_m);

class StreamingEnvironmentSource : public EnvironmentSource {
public:
    // externals carry the composed accessor stack (fixture or
    // CesiumCurl -> CountingAssetAccessor -> CachingAssetAccessor(SqliteCache),
    // built by the factory) + task processor + AsyncSystem -- the cache db
    // path lives in that stack, not here. `asset_id > 0` + non-empty
    // `ion_access_token` selects the Tileset's ion constructor (the real
    // path, Decision 15.6); `asset_id == 0` selects the URL constructor
    // against `root_tileset_uri` (the fixture path, test-only).
    // `counting_accessor` is the SAME object build_externals() wrapped into
    // `externals.pAssetAccessor`'s CachingAssetAccessor -- kept as its own
    // shared_ptr here (not re-derived from `externals`, which only exposes
    // the composed IAssetAccessor base) so update() can read its failure
    // count (Decision 11 / VM-063 Task 4). May be null (the counting
    // accessor is always built by build_externals() in practice, but a
    // null-safe check costs nothing and means "network loss never
    // detected" rather than a crash for any future caller that omits it).
    // `materials_original` (VM-064, Task 5 Decision 14): false (default,
    // every pre-VM-064 call site) is today's clay remap; true is the
    // original-materials mode Google Photorealistic 3D Tiles needs --
    // parsed from the ion:// URI's `materials=` key (production path) or
    // passed directly by the fixture install hook (test path).
    // `follow_terrain` (2026-09-21, "option 2"): false (default, every
    // pre-existing call site) keeps the map frame flat, byte-identical to
    // before -- true samples Google's own terrain height under the ego
    // (Tileset::sampleHeightMostDetailed) and shifts the whole streamed
    // environment to meet it. Parsed from the ion:// URI's
    // `follow_terrain=` key (production path) or passed directly by the
    // fixture install hook (test path), same shape as `materials_original`.
    StreamingEnvironmentSource(Cesium3DTilesSelection::TilesetExternals externals, int64_t asset_id,
                               std::string ion_access_token, std::string root_tileset_uri,
                               std::string fallback_baked_dir, GeoAnchor anchor,
                               std::shared_ptr<CountingAssetAccessor> counting_accessor,
                               bool materials_original = false, bool follow_terrain = false,
                               // Defaults MATCH IonSpec's own (ground_bias_m 0.3,
                               // replaces_ground true) so a caller that omits them
                               // gets the same behaviour as the equivalent URI
                               // (review minor, 2026-09-21); both real call sites
                               // pass them explicitly regardless.
                               double ground_bias_m = 0.3, bool replaces_ground = true,
                               // `max_tilt_deg` (2026-09-21 multi-point plane fit): degrees the
                               // fitted grade's tilt is clamped to; default MATCHES IonSpec's own
                               // (2.0), same "caller that omits it gets the URI-equivalent
                               // behaviour" reasoning as the two params above. 0 disables tilt
                               // (offset-only, the pre-fit behaviour) -- see kTerrainMaxTiltRad.
                               double max_tilt_deg = 2.0);
    ~StreamingEnvironmentSource() override;

    void update(VisualRenderer& r, Vec3 ego_map_pos) override;
    void teardown(VisualRenderer& r) override;
    void set_visible(VisualRenderer& r, bool visible) override;
    size_t loaded_count() const override;
    size_t scene_membership_count(VisualRenderer& r) const override;
    EnvironmentSourceState state() const override;
    // 2026-09-21 live finding (see EnvironmentSource::provides_ground()):
    // true once EVIDENCE (a successful sampleHeightMostDetailed() hit under
    // the ego) proves this streamed tileset actually has ground geometry
    // there -- latched by TerrainFollowState::ground_hit for the life of
    // this source, gated on replacesGround_ so `replaces_ground=off` (or a
    // node param) forces the clay plane to stay regardless of evidence.
    // Defined out-of-line (environment_stream.cpp, after TerrainFollowState
    // itself) -- TerrainFollowState is only forward-declared here, so an
    // inline body at this point in the class would need its incomplete type.
    bool provides_ground() const override;

    // Recorded, not asserted (same class as budget_probe.md numbers) --
    // how many times teardown()'s bounded wait (kTeardownPumpBound) gave
    // up before cesium's async destruction event fired. 0 in every run
    // this task observed.
    int leaked_on_teardown_bound() const { return leakedOnTeardownBound_; }

    // VM-064 test-hook mirrors (environment_test_hooks.hpp's
    // environment_stream_materials_original() /
    // environment_stream_first_primitive_is_clay()) -- not a Filament
    // read-back, same "opaque hook" convention as every other test-only
    // accessor in this file.
    bool materials_original() const { return materialsOriginal_; }
    bool first_primitive_is_building_material(VisualRenderer& r) const;

    // 2026-09-21 terrain following: current smoothed ground offset (metres,
    // map-frame +Z), same value last handed to
    // StreamRendererResources::set_ground_offset_z() (or 0.0 if
    // follow_terrain is off, or on but no sample has landed yet).
    double ground_offset_z() const { return groundOffsetZ_; }

    // Gate round 1 finding 1: ground_offset_z() above reads groundOffsetZ_,
    // a plain double this class keeps updating on its own regardless of
    // whether renderResources_->set_ground_offset_z() or
    // tm.setParent(assetRoot, terrainRoot) actually ran -- so a check built
    // only on ground_offset_z() cannot fail when either of those two lines
    // is deleted. This instead reads back the REAL Filament transform: the
    // WORLD-space (post parent-composition) z translation of the first
    // currently-tracked streamed tile's own root entity. Deleting the
    // setParent call leaves the asset unparented (world == its own local
    // placement transform, terrain offset never composes in); deleting
    // set_ground_offset_z() leaves the terrain root's own transform at
    // identity forever (world == local too, just via the parent instead).
    // Either revert makes this value stop moving with groundOffsetZ_. NaN
    // if `r` has no tracked tile yet (nothing loaded) or its root has no
    // transform component.
    double first_tracked_tile_world_z(VisualRenderer& r) const;

    // 2026-09-21 two-frustum tile selection: how many ViewState frustums the
    // MOST RECENT synthesize_view_and_pump() call passed to
    // tileset_->updateViewGroup() -- 1 (synthetic-only) before the render
    // camera has ever been positioned, 2 (synthetic + camera) once it has.
    // 0 if update() has never run yet.
    size_t last_view_frustum_count() const { return lastViewFrustumCount_; }

    // Test-only (2026-09-21 Opus gate fix round, provides_ground() vs
    // fallenBack_): directly invokes the SAME fall_back() a real
    // kNetworkLossConsecutiveFailures trip calls -- reaches a genuine
    // post-fallback state (teardown run, fallenBack_ set, fallbackSource_
    // opened if one was configured) deterministically, without racing
    // consecutive-failure counting against a file fixture. Empirically (see
    // NetworkDeadFromFirstRequestFallsBackToBakedChunksOnce's own "Gap"
    // comment, environment_stream.cpp's testing section), a tile that has
    // loaded once is thereafter served from the warmed on-disk sqlite cache
    // and never touches a "killed" accessor again -- so a fixture that has
    // already proven ground_hit true can never be driven into
    // STREAMING_FALLBACK through the real counting path inside one short
    // test process. Calling the real fall_back() directly is the only way
    // to exercise "ground_hit was latched true, THEN the source fell back"
    // deterministically; it is not a reimplementation of fall_back()'s own
    // logic, so it cannot mask a break in that logic itself.
    void force_fall_back_for_testing(VisualRenderer& r) { fall_back(r); }

private:
    void synthesize_view_and_pump(VisualRenderer& r, Vec3 ego_map_pos);
    // Issues a new sampleHeightMostDetailed() request when none is already
    // in flight and the ego has moved/enough time has passed since the last
    // one (kTerrainSampleMoveThresholdM / kTerrainSampleIntervalS) -- a
    // no-op unless followTerrain_ is set. `heading_rad` is the ego heading
    // (r.scene_buffer.active().ego.heading_rad, read by the caller) the
    // batch's 5 along-track offsets are sampled against; stored on
    // `terrainState_` as the pivot heading so update_terrain_transform()
    // rotates about the SAME axis the fit was taken against, even if the
    // ego has since turned. The continuation captures `terrainState_` BY
    // VALUE, never `this`: `this` can be destroyed with a sample in flight
    // (teardown()/~StreamingEnvironmentSource() do not wait for it), so the
    // continuation must only touch the shared state, not the source.
    void maybe_trigger_terrain_sample(Vec3 ego_map_pos, double heading_rad);
    // Recomputes the target offset+tilt from the fit currently stored on
    // `terrainState_` (unchanged since the last call unless a sample just
    // landed) and applies first-order smoothing (kTerrainSmoothTimeConstantS)
    // of both toward that target, snapping instead of sliding on the very
    // first fit; calls renderResources_->set_terrain_transform() only when
    // EITHER value actually moved by more than 1e-3 (metres for the offset,
    // radians for the tilt) since the last call. A no-op unless
    // followTerrain_ is set and at least one fit has landed.
    void update_terrain_transform(float deltaSeconds);
    // Decision 11 (VM-063): tears down every streamed tile + the tileset
    // itself (via teardown(), the same discipline destroy_renderer() uses),
    // then opens `fallbackBakedDir_` (empty -> no fallback source, tiles
    // just stop appearing -- the no-fallback-configured path) and switches
    // fallenBack_ so every subsequent update()/teardown()/loaded_count()
    // call delegates to it. One-way: no automatic recovery to streaming
    // even if the link returns (restart recovers). ponytail: one-way
    // fallback; auto-resume when the link returns is the upgrade.
    void fall_back(VisualRenderer& r);

    CesiumAsync::AsyncSystem asyncSystem_;
    std::shared_ptr<StreamRendererResources> renderResources_;
    std::unique_ptr<Cesium3DTilesSelection::Tileset> tileset_;
    Cesium3DTilesSelection::TilesetViewGroup* viewGroup_ = nullptr;  // owned by tileset_
    GeoAnchor anchor_;
    glm::dmat4 ecefToMap_;
    glm::dmat4 mapToEcef_;
    std::string fallbackBakedDir_;    // stored, acted on by Task 4
    bool materialsOriginal_ = false;  // VM-064: threaded into renderResources_ at construction
    bool tornDown_ = false;
    int leakedOnTeardownBound_ = 0;
    std::optional<std::chrono::steady_clock::time_point> lastUpdate_;
    // FilamentAsset* -> tracked (Cesium currently wants this tile rendered).
    // VM-096 (set_environment_visible()): the invariant is now
    // "visible_ <=> every tracked entry is actually added to r.scene" --
    // while hidden, entries are still tracked/reconciled every tick (so
    // synthesize_view_and_pump's eviction logic keeps working unchanged),
    // they just never get an addEntities call. See set_visible()'s own
    // comment.
    std::unordered_map<const void*, bool> inScene_;
    bool visible_ = true;
    // 2026-09-21 two-frustum tile selection: mirrors last_view_frustum_count().
    size_t lastViewFrustumCount_ = 0;

    // ── 2026-09-21 terrain following ("option 2") ────────────────────────
    bool followTerrain_ = false;
    double groundBiasM_ = 0.0;  // IonSpec::ground_bias_m; 0 on the fixture path
    // IonSpec::replaces_ground (default true): gates provides_ground() above
    // regardless of evidence -- `replaces_ground=off` / node param forces
    // the renderer's clay ground plane to stay even if this tileset would
    // otherwise prove it has ground under the ego.
    bool replacesGround_ = true;
    std::shared_ptr<TerrainFollowState> terrainState_;
    double groundOffsetZ_ = 0.0;             // current smoothed offset (map-frame +Z, metres)
    double lastAppliedGroundOffsetZ_ = 0.0;  // value last pushed to renderResources_
    // 2026-09-21 multi-point plane fit: current smoothed tilt (radians,
    // about the across-track axis) and the value last pushed to
    // renderResources_ -- same shape as the two offset members immediately
    // above.
    double terrainTiltRad_ = 0.0;
    double lastAppliedTiltRad_ = 0.0;
    // max_tilt_deg (URI key) / StreamingEnvironmentSource ctor param,
    // stored pre-converted to radians -- kTerrainMaxTiltRad's own default.
    double maxTiltRad_ = kTerrainMaxTiltRad;
    bool groundOffsetSnapped_ =
        false;  // true once the first fit has snapped groundOffsetZ_/terrainTiltRad_

    // ── VM-063 (Task 4): fallback state ──────────────────────────────────
    std::shared_ptr<CountingAssetAccessor> countingAccessor_;
    bool fallenBack_ = false;
    std::unique_ptr<EnvironmentSource>
        fallbackSource_;  // null iff no &fallback= dir, or it failed to open
};

}  // namespace overlume
