// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

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

inline constexpr double kStreamViewHeightM = 300.0;
inline constexpr int kStreamViewportPx = 256;
inline constexpr double kStreamViewFovRad = 1.3;
inline constexpr double kStreamMaxSseErr = 48.0;
inline constexpr double kStreamHeightOffsetM = 0.0;
inline constexpr uint64_t kDefaultMaxCacheItems = 4096;
inline constexpr int kNetworkLossConsecutiveFailures = 8;
inline constexpr int kTeardownPumpBound = 2000;

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

class FileFixtureAssetAccessor : public CesiumAsync::IAssetAccessor {
public:
    explicit FileFixtureAssetAccessor(std::shared_ptr<std::atomic<bool>> killed = nullptr)
        : killed_(std::move(killed)) {}

    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> get(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& url,
        const std::vector<THeader>&) override {
        return asyncSystem.createResolvedFuture(makeRequest("GET", url));
    }
    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> request(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& verb,
        const std::string& url, const std::vector<THeader>&,
        const std::span<const std::byte>&) override {
        return asyncSystem.createResolvedFuture(makeRequest(verb, url));
    }
    void tick() noexcept override {}

private:
    std::shared_ptr<CesiumAsync::IAssetRequest> makeRequest(const std::string& verb,
                                                            const std::string& url);
    std::shared_ptr<std::atomic<bool>> killed_;
};

class StreamRendererResources final : public Cesium3DTilesSelection::IPrepareRendererResources {
public:
    explicit StreamRendererResources(const glm::dmat4& ecefToMap, double anchorHeightM,
                                     bool materialsOriginal = false, double brightness = 1.0)
        : ecefToMap_(ecefToMap),
          anchorHeightM_(anchorHeightM),
          materialsOriginal_(materialsOriginal),
          brightness_(brightness) {}

    void set_renderer(VisualRenderer* r) { r_ = r; }

    CesiumAsync::Future<Cesium3DTilesSelection::TileLoadResultAndRenderResources>
    prepareInLoadThread(const CesiumAsync::AsyncSystem& asyncSystem,
                        Cesium3DTilesSelection::TileLoadResult&& tileLoadResult,
                        const glm::dmat4& transform, const std::any& rendererOptions) override;
    void* prepareInMainThread(Cesium3DTilesSelection::Tile& tile, void* pLoadThreadResult) override;
    void free(Cesium3DTilesSelection::Tile& tile, void* pLoadThreadResult,
              void* pMainThreadResult) noexcept override;

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

    std::vector<filament::gltfio::FilamentAsset*> drain_pending_frees();
    void note_torn_down() { tornDown_.store(true); }

    void set_terrain_transform(double z, double tilt_rad, double pivot_x, double pivot_y,
                               double heading_rad);
    void destroy_terrain_root();

private:
    void ensure_terrain_root();

    VisualRenderer* r_ = nullptr;
    glm::dmat4 ecefToMap_;
    double anchorHeightM_ = 0.0;
    bool materialsOriginal_ = false;
    double brightness_ = 1.0;
    std::mutex freeMutex_;
    std::vector<filament::gltfio::FilamentAsset*> pendingFrees_;
    std::atomic<bool> tornDown_{false};
    utils::Entity terrainRoot_;
};

struct LoadThreadGlb {
    std::vector<std::byte> glbBytes;
    glm::dmat4 transform{1.0};
    bool ok = false;
};

glm::dmat4 compute_ecef_to_map(const GeoAnchor& anchor);

glm::dvec3 correct_ecef_point_height(const glm::dvec3& ecef_pos, const glm::dmat4& ecef_to_map,
                                     double anchor_height_m);

class StreamingEnvironmentSource : public EnvironmentSource {
public:
    StreamingEnvironmentSource(Cesium3DTilesSelection::TilesetExternals externals, int64_t asset_id,
                               std::string ion_access_token, std::string root_tileset_uri,
                               std::string fallback_baked_dir, GeoAnchor anchor,
                               std::shared_ptr<CountingAssetAccessor> counting_accessor,
                               bool materials_original = false, bool follow_terrain = false,
                               double ground_bias_m = 0.3, bool replaces_ground = true,
                               double max_tilt_deg = 2.0, double brightness = 1.0);
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
    bool first_primitive_is_building_material(VisualRenderer& r) const;

    double ground_offset_z() const { return groundOffsetZ_; }

    double first_tracked_tile_world_z(VisualRenderer& r) const;

    size_t last_view_frustum_count() const { return lastViewFrustumCount_; }

    void force_fall_back_for_testing(VisualRenderer& r) { fall_back(r); }

private:
    void synthesize_view_and_pump(VisualRenderer& r, Vec3 ego_map_pos);
    void maybe_trigger_terrain_sample(Vec3 ego_map_pos, double heading_rad);
    void update_terrain_transform(float deltaSeconds);
    void fall_back(VisualRenderer& r);

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

    std::shared_ptr<CountingAssetAccessor> countingAccessor_;
    bool fallenBack_ = false;
    std::unique_ptr<EnvironmentSource> fallbackSource_;
};

}  // namespace overlume
