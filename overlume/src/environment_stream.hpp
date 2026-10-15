// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

static_assert(__cplusplus >= 202002L,
              "environment_stream.hpp is C++20-only -- include it only from overlume_stream "
              "sources (environment_stream.cpp, mapbox_*.cpp)");

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <glm/mat4x4.hpp>

#include <Cesium3DTilesSelection/IPrepareRendererResources.h>
#include <Cesium3DTilesSelection/Tile.h>
#include <Cesium3DTilesSelection/Tileset.h>
#include <Cesium3DTilesSelection/TilesetContentLoader.h>
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

namespace spdlog {
class logger;
}

namespace filament {
class MaterialInstance;
class Texture;
}

namespace overlume {

inline constexpr double kStreamViewHeightM = 300.0;
inline constexpr int kStreamViewportPx = 256;
inline constexpr double kStreamViewFovRad = 1.3;
inline constexpr double kStreamMaxSseErr = 48.0;
inline constexpr double kStreamHeightOffsetM = 0.0;
inline constexpr uint64_t kDefaultMaxCacheItems = 4096;
inline constexpr int kNetworkLossConsecutiveFailures = 8;
inline constexpr int kTeardownPumpBound = 2000;
inline constexpr double kDefaultTileRadiusM = 700.0;
inline constexpr double kTileRadiusUnloadRatio = 4.0 / 3.0;

inline constexpr double kTerrainSampleMoveThresholdM = 5.0;
inline constexpr double kTerrainSampleIntervalS = 2.0;
inline constexpr double kTerrainOffsetClampM = 30.0;
inline constexpr double kTerrainSmoothTimeConstantS = 0.5;
inline constexpr double kTerrainMaxTiltRad = 2.0 * M_PI / 180.0;

struct TerrainFollowState;

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

class CountingAssetAccessor final : public CesiumAsync::IAssetAccessor {
public:
    explicit CountingAssetAccessor(std::shared_ptr<CesiumAsync::IAssetAccessor> inner)
        : inner_(std::move(inner)) {}

    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> get(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& url,
        const std::vector<THeader>& headers) override {
        return counted(inner_->get(asyncSystem, url, headers));
    }
    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> request(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& verb,
        const std::string& url, const std::vector<THeader>& headers,
        const std::span<const std::byte>& payload) override {
        return counted(inner_->request(asyncSystem, verb, url, headers, payload));
    }
    void tick() noexcept override { inner_->tick(); }

    int consecutive_failures() const { return consecutiveFailures_.load(); }

private:
    // A transport error (curl: DNS failure, refused connection, timeout) rejects the future with
    // no response at all. Count it like a failed reply, then rethrow so the caller still sees it.
    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> counted(
        CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>>&& reply) {
        return std::move(reply)
            .thenImmediately([this](std::shared_ptr<CesiumAsync::IAssetRequest>&& req) {
                note(req.get());
                return req;
            })
            .catchImmediately(
                [this](std::exception&& e) -> std::shared_ptr<CesiumAsync::IAssetRequest> {
                    note(nullptr);
                    throw std::runtime_error(e.what());
                });
    }
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

class FileFixtureAssetAccessor : public CesiumAsync::IAssetAccessor {
public:
    // transport_down rejects every request but tileset.json with no response, as curl's accessor
    // does on a DNS failure, refused connection or timeout.
    explicit FileFixtureAssetAccessor(std::shared_ptr<std::atomic<bool>> killed = nullptr,
                                      std::shared_ptr<std::atomic<bool>> transport_down = nullptr)
        : killed_(std::move(killed)), transportDown_(std::move(transport_down)) {}

    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> get(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& url,
        const std::vector<THeader>&) override {
        return respond(asyncSystem, "GET", url);
    }
    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> request(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& verb,
        const std::string& url, const std::vector<THeader>&,
        const std::span<const std::byte>&) override {
        return respond(asyncSystem, verb, url);
    }
    void tick() noexcept override {}

private:
    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> respond(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& verb,
        const std::string& url);
    std::shared_ptr<CesiumAsync::IAssetRequest> makeRequest(const std::string& verb,
                                                            const std::string& url);
    std::shared_ptr<std::atomic<bool>> killed_;
    std::shared_ptr<std::atomic<bool>> transportDown_;
};

// Test accessor beside FileFixtureAssetAccessor, reusing its response classes: canned answers keyed
// by URL without its query string, or a rejected future (a transport error); every requested URL
// is recorded in full. Thread-safe.
class MapFixtureAssetAccessor final : public CesiumAsync::IAssetAccessor {
public:
    explicit MapFixtureAssetAccessor(uint16_t unmatched_status = 404)
        : unmatchedStatus_(unmatched_status) {}

    void serve(const std::string& url_without_query, uint16_t status, std::vector<std::byte> body);
    void fail_transport(const std::string& url_without_query);
    std::vector<std::string> requested_urls() const;
    // A held URL's requests stay pending until release_held() answers them, so a test can switch
    // sources while a tile load is in flight.
    void hold(const std::string& url_without_query);
    void release_held();
    // Every URL starting with `prefix` that no exact rule names gets this answer; the longest
    // matching prefix wins, and a later rule for the same prefix replaces an earlier one.
    void serve_prefix(const std::string& prefix, uint16_t status, std::vector<std::byte> body);
    // Requests for a URL starting with `prefix` that no exact rule names stay pending, as hold()'s.
    void hold_prefix(const std::string& prefix);
    // From now on every request, and every held request release_held() answers, is a transport
    // error: the network is gone.
    void fail_all_transport();

    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> get(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& url,
        const std::vector<THeader>&) override {
        return respond(asyncSystem, "GET", url);
    }
    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> request(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& verb,
        const std::string& url, const std::vector<THeader>&,
        const std::span<const std::byte>&) override {
        return respond(asyncSystem, verb, url);
    }
    void tick() noexcept override {}

private:
    using RequestPromise = CesiumAsync::Promise<std::shared_ptr<CesiumAsync::IAssetRequest>>;
    struct Canned {
        uint16_t status = 0;
        std::vector<std::byte> body;
        bool transport_error = false;
    };
    struct Held {
        RequestPromise promise;
        std::string verb;
        std::string url;
    };

    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> respond(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& verb,
        const std::string& url);
    void answer(const RequestPromise& promise, const std::string& verb,
                const std::string& url) const;

    uint16_t unmatchedStatus_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, Canned> canned_;
    std::vector<std::string> held_;
    std::vector<Held> pending_;
    std::vector<std::string> requested_;
    std::vector<std::pair<std::string, Canned>> prefixes_;
    std::vector<std::string> heldPrefixes_;
    bool allTransportDown_ = false;
};

class StreamRendererResources final : public Cesium3DTilesSelection::IPrepareRendererResources {
public:
    // Content tiles get the clay building material (or keep their own with materials=original).
    // Terrain tiles get a terrain.mat instance each and take raster overlays through the five
    // overlay callbacks; Content resources never see a raster.
    enum class Kind : uint8_t { Content, Terrain };
    struct OverlayCounters {
        uint32_t rasters_attached = 0, rasters_detached = 0, textures_alive = 0, textures_freed = 0;
    };

    explicit StreamRendererResources(const glm::dmat4& ecefToMap, double anchorHeightM,
                                     bool materialsOriginal = false, double brightness = 1.0,
                                     Kind kind = Kind::Content)
        : ecefToMap_(ecefToMap),
          anchorHeightM_(anchorHeightM),
          materialsOriginal_(materialsOriginal),
          brightness_(brightness),
          kind_(kind) {}

    void set_renderer(VisualRenderer* r) { r_ = r; }
    double brightness() const { return brightness_; }

    CesiumAsync::Future<Cesium3DTilesSelection::TileLoadResultAndRenderResources>
    prepareInLoadThread(const CesiumAsync::AsyncSystem& asyncSystem,
                        Cesium3DTilesSelection::TileLoadResult&& tileLoadResult,
                        const glm::dmat4& transform, const std::any& rendererOptions) override;
    void* prepareInMainThread(Cesium3DTilesSelection::Tile& tile, void* pLoadThreadResult) override;
    void free(Cesium3DTilesSelection::Tile& tile, void* pLoadThreadResult,
              void* pMainThreadResult) noexcept override;

    // Load thread: the decoded image as RGBA8 (RasterLoadThreadResult). Main thread: one
    // mipmapped sRGB texture per raster tile. Attach: the tile's instance samples it through
    // cesium's translation and scale. Detach: back to terrainColor. Free: the texture joins the
    // deferred frees that drain_pending_frees destroys.
    void* prepareRasterInLoadThread(CesiumImage::ImageAsset& image,
                                    const std::any& rendererOptions) override;
    void* prepareRasterInMainThread(CesiumRasterOverlays::RasterOverlayTile& rasterTile,
                                    void* pLoadThreadResult) override;
    void freeRaster(const CesiumRasterOverlays::RasterOverlayTile& rasterTile,
                    void* pLoadThreadResult, void* pMainThreadResult) noexcept override;
    void attachRasterInMainThread(const Cesium3DTilesSelection::Tile& tile,
                                  int32_t overlayTextureCoordinateID,
                                  const CesiumRasterOverlays::RasterOverlayTile& rasterTile,
                                  void* pMainThreadRendererResources, const glm::dvec2& translation,
                                  const glm::dvec2& scale) override;
    void detachRasterInMainThread(const Cesium3DTilesSelection::Tile& tile,
                                  int32_t overlayTextureCoordinateID,
                                  const CesiumRasterOverlays::RasterOverlayTile& rasterTile,
                                  void* pMainThreadRendererResources) noexcept override;

    std::vector<filament::gltfio::FilamentAsset*> drain_pending_frees();
    // Main thread: pushes the active theme's environment.terrain_color to every live terrain tile
    // when it changed (new tiles copy it from terrain.mat's default instance).
    void sync_terrain_color();
    OverlayCounters overlay_counters() const { return counters_; }
    // The terrain_color sync_terrain_color last pushed (-1 each before the first push).
    std::array<float, 3> terrain_color_for_testing() const { return terrainColor_; }
    void note_torn_down() { tornDown_.store(true); }

    void set_terrain_transform(double z, double tilt_rad, double pivot_x, double pivot_y,
                               double heading_rad);
    void destroy_terrain_root();

private:
    void ensure_terrain_root();
    filament::MaterialInstance* terrain_instance(const Cesium3DTilesSelection::Tile& tile) const;

    VisualRenderer* r_ = nullptr;
    glm::dmat4 ecefToMap_;
    double anchorHeightM_ = 0.0;
    bool materialsOriginal_ = false;
    double brightness_ = 1.0;
    Kind kind_ = Kind::Content;
    std::mutex freeMutex_;
    std::vector<filament::gltfio::FilamentAsset*> pendingFrees_;
    std::vector<filament::Texture*> pendingTextureFrees_;  // under freeMutex_
    std::atomic<bool> tornDown_{false};
    utils::Entity terrainRoot_;
    // Main thread only: each terrain tile's instance, keyed by its FilamentAsset.
    std::unordered_map<const void*, filament::MaterialInstance*> terrainInstances_;
    std::array<float, 3> terrainColor_{-1.0f, -1.0f, -1.0f};  // last pushed by sync_terrain_color
    OverlayCounters counters_;                                // main thread only
};

struct LoadThreadGlb {
    std::vector<std::byte> glbBytes;
    glm::dmat4 transform{1.0};
    bool ok = false;
};

// One raster overlay tile's pixels, RGBA8, row 0 at the north edge, load thread to main thread.
struct RasterLoadThreadResult {
    uint32_t width = 0;
    uint32_t height = 0;
    std::vector<uint8_t> rgba;
};

glm::dmat4 compute_ecef_to_map(const GeoAnchor& anchor);

glm::dvec3 correct_ecef_point_height(const glm::dvec3& ecef_pos, const glm::dmat4& ecef_to_map,
                                     double anchor_height_m);

std::shared_ptr<spdlog::logger> make_redacting_logger();

// Wraps `base` in a CountingAssetAccessor, behind the SQLite cache unless cache_dir is "off".
Cesium3DTilesSelection::TilesetExternals build_externals(
    std::shared_ptr<CesiumAsync::IAssetAccessor> base, const CesiumAsync::AsyncSystem& asyncSystem,
    const std::string& cache_dir, uint64_t max_cache_items,
    std::shared_ptr<CountingAssetAccessor>* out_counting = nullptr);

struct CustomTileset {
    std::unique_ptr<Cesium3DTilesSelection::TilesetContentLoader> loader;
    std::unique_ptr<Cesium3DTilesSelection::Tile> root;
    // Select with cesium's area view, a sphere of the source's radius around the ego, as the only
    // view (geometric-error threshold 1.0); also turns preloadSiblings off. (Fog culling needs no
    // switch: cesium 0.64 never fog-culls an area view, TilesetSelection.cpp:471.)
    bool area_selection = false;
};

class StreamingEnvironmentSource : public EnvironmentSource {
public:
    StreamingEnvironmentSource(Cesium3DTilesSelection::TilesetExternals externals, int64_t asset_id,
                               std::string ion_access_token, std::string root_tileset_uri,
                               std::string fallback_baked_dir, GeoAnchor anchor,
                               std::shared_ptr<CountingAssetAccessor> counting_accessor,
                               bool materials_original = false, bool follow_terrain = false,
                               double ground_bias_m = 0.3, bool replaces_ground = true,
                               double max_tilt_deg = 2.0, double brightness = 1.0,
                               std::optional<double> radius_override_m = std::nullopt,
                               CustomTileset custom_tileset = {},
                               StreamRendererResources::Kind resources_kind = {});
    ~StreamingEnvironmentSource() override;

    void update(VisualRenderer& r, Vec3 ego_map_pos) override;
    void teardown(VisualRenderer& r) override;
    void set_visible(VisualRenderer& r, bool visible) override;
    size_t loaded_count() const override;
    size_t scene_membership_count(VisualRenderer& r) const override;
    EnvironmentSourceState state() const override;
    bool provides_ground() const override;

    int leaked_on_teardown_bound() const { return leakedOnTeardownBound_; }

    bool materials_original() const { return materialsOriginal_; }
    double brightness() const { return brightness_; }
    // True while the tileset reads through the SQLite cache, i.e. a cache database is open.
    bool has_disk_cache() const;
    bool first_primitive_is_building_material(VisualRenderer& r) const;

    double ground_offset_z() const { return groundOffsetZ_; }

    double first_tracked_tile_world_z(VisualRenderer& r) const;

    size_t last_view_frustum_count() const { return lastViewFrustumCount_; }

    void force_fall_back_for_testing(VisualRenderer& r) { fall_back(r); }

    // Task 11: a second tileset. Called once before the first update(); overlays are already added
    // to `terrain->getOverlays()` by the caller. provides_ground(), height sampling and the
    // terrain transform target the terrain tileset when present.
    void attach_terrain(std::unique_ptr<Cesium3DTilesSelection::Tileset> terrain,
                        std::shared_ptr<StreamRendererResources> terrain_resources);
    bool has_terrain() const;
    // Task 11: the overlay credits of both tilesets, plain text, "; " separated, HTML stripped.
    std::string credits() const;

    // The terrain tileset's renderer resources, null without one. Kept after teardown.
    std::shared_ptr<StreamRendererResources> terrain_resources() const { return terrainResources_; }
    // Terrain tiles in the Filament scene.
    size_t terrain_scene_count(VisualRenderer& r) const;
    // World z minus local z of a terrain tile's root: the follow-terrain lift (NaN without one).
    double terrain_tile_lift_z(VisualRenderer& r) const;

    // The terrain resources' counters once attach_terrain has run, else these (Task 5's fixture
    // opens its terrain as the content tileset).
    StreamRendererResources::OverlayCounters overlay_counters() const {
        return (terrainResources_ ? terrainResources_ : renderResources_)->overlay_counters();
    }
    // Test hooks add and remove raster overlays here: the terrain tileset's once attach_terrain has
    // run (where real sources keep their overlay), else the content tileset's (Task 5's fixture).
    Cesium3DTilesSelection::RasterOverlayCollection& overlays_for_testing() {
        return (terrainTileset_ ? *terrainTileset_ : *tileset_).getOverlays();
    }

private:
    void synthesize_view_and_pump(VisualRenderer& r, Vec3 ego_map_pos);
    void maybe_trigger_terrain_sample(Vec3 ego_map_pos, double heading_rad);
    void update_terrain_transform(float deltaSeconds);
    void fall_back(VisualRenderer& r);
    // Adds the tiles of `result` within the radius to the scene, removes those that left it.
    void sync_scene(VisualRenderer& r, const Cesium3DTilesSelection::ViewUpdateResult& result,
                    Vec3 ego_map_pos, double radius_m,
                    std::unordered_map<const void*, bool>& in_scene);

    CesiumAsync::AsyncSystem asyncSystem_;
    std::shared_ptr<StreamRendererResources> renderResources_;
    std::unique_ptr<Cesium3DTilesSelection::Tileset> tileset_;
    Cesium3DTilesSelection::TilesetViewGroup* viewGroup_ = nullptr;
    GeoAnchor anchor_;
    glm::dmat4 ecefToMap_;
    glm::dmat4 mapToEcef_;
    std::string fallbackBakedDir_;
    bool materialsOriginal_ = false;
    bool tornDown_ = false;
    int leakedOnTeardownBound_ = 0;
    std::optional<std::chrono::steady_clock::time_point> lastUpdate_;
    std::unordered_map<const void*, bool> inScene_;
    bool visible_ = true;
    size_t lastViewFrustumCount_ = 0;

    bool followTerrain_ = false;
    double groundBiasM_ = 0.0;
    bool replacesGround_ = true;
    std::shared_ptr<TerrainFollowState> terrainState_;
    double groundOffsetZ_ = 0.0;
    double lastAppliedGroundOffsetZ_ = 0.0;
    double terrainTiltRad_ = 0.0;
    double lastAppliedTiltRad_ = 0.0;
    double maxTiltRad_ = kTerrainMaxTiltRad;
    double brightness_ = 1.0;
    bool groundOffsetSnapped_ = false;
    std::optional<double> tileRadiusOverrideM_;
    bool areaSelection_ = false;

    std::shared_ptr<CountingAssetAccessor> countingAccessor_;
    bool fallenBack_ = false;
    std::unique_ptr<EnvironmentSource> fallbackSource_;

    std::shared_ptr<StreamRendererResources> terrainResources_;
    std::unique_ptr<Cesium3DTilesSelection::Tileset> terrainTileset_;
    std::unordered_map<const void*, bool> terrainInScene_;
};

}
