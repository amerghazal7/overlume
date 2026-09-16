// environment_stream.hpp — library-internal AND C++20-only (Decision 3,
// docs/superpowers/plans/2026-08-18-visual-mode-epic6.md). Included by
// environment_stream.cpp and NOTHING else: the static_assert below makes
// that enforced, not just documented, and no other TU in this project has
// the cesium include dirs needed to even find these headers (they exist
// only on the visual_renderer_stream OBJECT library, CMakeLists.txt).
#pragma once

static_assert(__cplusplus >= 202002L,
              "environment_stream.hpp is C++20-only -- include it from "
              "environment_stream.cpp, nowhere else");

#include <atomic>
#include <chrono>
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

#include "environment.hpp"
#include "visual_renderer/scene.h"

namespace mpviz {

// ── Named constants (Decision 9, dev-box proxies -- same honesty class as
//    environment.hpp's kLoadRadiusM) ─────────────────────────────────────
inline constexpr double kStreamViewHeightM = 300.0;   // synthetic-camera height above ego
inline constexpr int kStreamViewportPx = 256;
inline constexpr double kStreamViewFovRad = 1.3;      // vertical AND horizontal (square viewport)
inline constexpr double kStreamMaxSseErr = 48.0;
// ponytail: nadir synthetic view sized to kLoadRadiusM; driving selection
// from the real render camera is the upgrade if building pop-in bothers
// anyone.
inline constexpr double kStreamHeightOffsetM = 0.0;   // Decision 8: absorbs any
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
//    tests/fixtures/environment_ion_fixture_0/ from disk -- no network, no
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
    // `materialsOriginal` (VM-064, Task 5): false (default) is today's clay
    // remap; true skips it in prepareInMainThread() -- gltfio's own loaded
    // ubershader materials stay bound (Google Photorealistic 3D Tiles).
    explicit StreamRendererResources(const glm::dmat4& ecefToMap, bool materialsOriginal = false)
        : ecefToMap_(ecefToMap), materialsOriginal_(materialsOriginal) {}

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
    void freeRaster(const CesiumRasterOverlays::RasterOverlayTile&, void*, void*) noexcept override {}
    void attachRasterInMainThread(const Cesium3DTilesSelection::Tile&, int32_t,
                                   const CesiumRasterOverlays::RasterOverlayTile&, void*, const glm::dvec2&,
                                   const glm::dvec2&) override {}
    void detachRasterInMainThread(const Cesium3DTilesSelection::Tile&, int32_t,
                                   const CesiumRasterOverlays::RasterOverlayTile&, void*) noexcept override {}

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

private:
    VisualRenderer* r_ = nullptr;
    glm::dmat4 ecefToMap_;
    bool materialsOriginal_ = false;  // VM-064: gates the clay remap, see prepareInMainThread()
    std::mutex freeMutex_;
    std::vector<filament::gltfio::FilamentAsset*> pendingFrees_;
    std::atomic<bool> tornDown_{false};
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
// asset root instead of per-vertex).
glm::dmat4 compute_ecef_to_map(const GeoAnchor& anchor);

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
    StreamingEnvironmentSource(Cesium3DTilesSelection::TilesetExternals externals,
                               int64_t asset_id, std::string ion_access_token,
                               std::string root_tileset_uri, std::string fallback_baked_dir,
                               GeoAnchor anchor,
                               std::shared_ptr<CountingAssetAccessor> counting_accessor,
                               bool materials_original = false);
    ~StreamingEnvironmentSource() override;

    void update(VisualRenderer& r, Vec3 ego_map_pos) override;
    void teardown(VisualRenderer& r) override;
    void set_visible(VisualRenderer& r, bool visible) override;
    size_t loaded_count() const override;
    size_t scene_membership_count() const override;
    EnvironmentSourceState state() const override;

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

private:
    void synthesize_view_and_pump(VisualRenderer& r, Vec3 ego_map_pos);
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
    std::string fallbackBakedDir_;  // stored, acted on by Task 4
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

    // ── VM-063 (Task 4): fallback state ──────────────────────────────────
    std::shared_ptr<CountingAssetAccessor> countingAccessor_;
    bool fallenBack_ = false;
    std::unique_ptr<EnvironmentSource> fallbackSource_;  // null iff no &fallback= dir, or it failed to open
};

}  // namespace mpviz
