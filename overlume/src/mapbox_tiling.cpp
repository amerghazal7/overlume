// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_tiling.hpp"

#include "mapbox_test_hooks.hpp"

#include <CesiumAsync/IAssetRequest.h>
#include <CesiumAsync/IAssetResponse.h>
#include <CesiumGeometry/QuadtreeTileID.h>
#include <CesiumGeospatial/BoundingRegion.h>
#include <CesiumGeospatial/GlobeRectangle.h>

#include <glm/gtc/type_ptr.hpp>

#include <spdlog/spdlog.h>

#include <chrono>
#include <exception>
#include <span>
#include <variant>

namespace overlume::mapbox {

namespace {

using Cesium3DTilesSelection::Tile;
using Cesium3DTilesSelection::TileEmptyContent;
using Cesium3DTilesSelection::TileLoadResult;
using Cesium3DTilesSelection::TileLoadResultState;

// Where mapbox_set_test_clock froze clock_s(); negative: the steady clock.
std::atomic<double> g_testClockS{-1.0};

uint64_t attempt_key(const TileId& t14) { return (uint64_t{t14.x} << 32) | t14.y; }

}

bool SourceLog::first_time(const std::string& key) {
    std::lock_guard<std::mutex> lock(mutex_);
    return logged_.insert(key).second;
}

bool AttemptBook::waiting(const TileId& t14, double now_s) const {
    const auto it = attempts_.find(attempt_key(t14));
    if (it == attempts_.end()) return false;
    const Attempt& a = it->second;
    return a.settled && !a.settled->load() && a.count > 0 && now_s < a.not_before_s;
}

Attempt& AttemptBook::begin(const TileId& t14, double now_s) {
    Attempt& a = attempts_[attempt_key(t14)];
    if (!a.settled || a.settled->load()) {
        a = Attempt{0, 0.0, std::make_shared<std::atomic<bool>>(false)};
    }
    const uint32_t attempt = ++a.count;
    // ponytail: the wait runs from this attempt's start, so the tile's attempt state stays
    // main-thread only; an attempt slower than its wait retries at once, and a tile that
    // leaves the radius mid-wait resumes its old count when it is back (fewer attempts,
    // never more).
    a.not_before_s = now_s + kRetryWaitS * static_cast<double>(1u << (attempt - 1));
    return a;
}

double clock_s() {
    const double frozen = g_testClockS.load();
    if (frozen >= 0.0) return frozen;
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

bool transient_status(uint16_t status) { return status == 0 || status == 429 || status >= 500; }

Outcome classify(const Fetched& f) {
    if (f.transport_error) return Outcome::Transient;
    if (f.status == 200) return Outcome::Body;
    if (f.status == 204 || f.status == 404) return Outcome::Empty;
    if (f.status == 401 || f.status == 403) return Outcome::Auth;
    // Status 0 comes only with transport_error: curl and the fixture network reject instead.
    if (transient_status(f.status)) return Outcome::Transient;
    return Outcome::Other;
}

CesiumAsync::Future<Fetched> fetch(const CesiumAsync::AsyncSystem& asyncSystem,
                                   const std::shared_ptr<CesiumAsync::IAssetAccessor>& accessor,
                                   const std::string& url) {
    return accessor->get(asyncSystem, url, {})
        .thenImmediately([](std::shared_ptr<CesiumAsync::IAssetRequest>&& request) {
            Fetched f;
            const CesiumAsync::IAssetResponse* response = request ? request->response() : nullptr;
            if (response == nullptr) {
                f.transport_error = true;
                return f;
            }
            f.status = response->statusCode();
            if (f.status == 200) {
                const std::span<const std::byte> data = response->data();
                f.body.assign(data.begin(), data.end());
            }
            return f;
        })
        .catchImmediately([](std::exception&&) {
            Fetched f;
            f.transport_error = true;
            return f;
        });
}

void note_status(SourceLog& seen, spdlog::logger& log, const char* endpoint, const Fetched& f,
                 const TileId& t14) {
    const Outcome outcome = classify(f);
    if (outcome == Outcome::Body || outcome == Outcome::Empty) return;
    if (!seen.first_time(std::string(endpoint) + " " + std::to_string(f.status))) return;
    if (outcome == Outcome::Auth) {
        log.error("Mapbox rejected MAPBOX_TOKEN (HTTP {}) for {}; those parts of tiles stay empty",
                  f.status, endpoint);
    } else if (outcome == Outcome::Transient) {
        const std::string what =
            f.transport_error ? std::string("transport error") : "HTTP " + std::to_string(f.status);
        log.warn(
            "mapbox: {} still failing ({}) after {} attempts at tile 14/{}/{}; showing what "
            "loaded",
            endpoint, what, kMaxAttempts, t14.x, t14.y);
    } else {
        log.warn("mapbox: {} answered HTTP {} at tile 14/{}/{}; that part stays empty", endpoint,
                 f.status, t14.x, t14.y);
    }
}

TileLoadResult success(Cesium3DTilesSelection::TileContentKind content,
                       std::optional<Cesium3DTilesSelection::BoundingVolume> bounds,
                       const std::shared_ptr<CesiumAsync::IAssetAccessor>& accessor,
                       const CesiumGeospatial::Ellipsoid& ellipsoid) {
    TileLoadResult result{};
    result.contentKind = std::move(content);
    result.glTFUpAxis = CesiumGeometry::Axis::Z;
    result.updatedBoundingVolume = std::move(bounds);
    result.pAssetAccessor = accessor;
    result.state = TileLoadResultState::Success;
    result.ellipsoid = ellipsoid;
    return result;
}

Cesium3DTilesSelection::BoundingVolume tile_region(const TileId& t, double min_height_m,
                                                   double max_height_m,
                                                   const CesiumGeospatial::Ellipsoid& ellipsoid) {
    const LonLatRect b = tile_bounds(t);
    return CesiumGeospatial::BoundingRegion(
        CesiumGeospatial::GlobeRectangle::fromDegrees(b.west, b.south, b.east, b.north),
        min_height_m, max_height_m, ellipsoid);
}

std::unique_ptr<Tile> quadtree_root(Cesium3DTilesSelection::TilesetContentLoader& loader,
                                    double origin_height_m) {
    auto root =
        std::make_unique<Tile>(&loader, Cesium3DTilesSelection::TileID(), TileEmptyContent{});
    root->setUnconditionallyRefine();
    root->setBoundingVolume(tile_region(TileId{0, 0, 0}, origin_height_m - kRegionBelowM,
                                        origin_height_m + kRegionAboveM));
    std::vector<Tile> children;
    Tile& world =
        children.emplace_back(&loader, CesiumGeometry::QuadtreeTileID(0, 0, 0), TileEmptyContent{});
    world.setUnconditionallyRefine();
    world.setBoundingVolume(root->getBoundingVolume());
    root->createChildTiles(std::move(children));
    return root;
}

std::vector<Tile> quadtree_children(const Tile& parent, const LoaderConfig& cfg,
                                    const CesiumGeospatial::Ellipsoid& ellipsoid) {
    const auto* id = std::get_if<CesiumGeometry::QuadtreeTileID>(&parent.getTileID());
    if (id == nullptr || id->level >= kLandmarkZoom) return {};
    // Every tile of a Mapbox tileset carries its loader (quadtree_root and the lines below).
    Cesium3DTilesSelection::TilesetContentLoader* loader = parent.getLoader();
    std::vector<Tile> children;
    children.reserve(4);
    for (uint32_t row = 0; row < 2; ++row) {
        for (uint32_t col = 0; col < 2; ++col) {
            const TileId t{id->level + 1, 2 * id->x + col, 2 * id->y + row};
            const CesiumGeometry::QuadtreeTileID childId(t.z, t.x, t.y);
            const Cesium3DTilesSelection::BoundingVolume region =
                tile_region(t, cfg.origin_height_m - kRegionBelowM,
                            cfg.origin_height_m + kRegionAboveM, ellipsoid);
            if (t.z < kLandmarkZoom) {
                Tile& child = children.emplace_back(loader, childId, TileEmptyContent{});
                child.setUnconditionallyRefine();
                child.setBoundingVolume(region);
                continue;
            }
            // The transform is absolute: the z14 tile's east-north-up frame, in double.
            Tile& child = children.emplace_back(loader, childId);
            child.setTransform(
                glm::make_mat4(make_tile_frame(t, cfg.origin_height_m).frame_to_ecef.data()));
            child.setGeometricError(0.0);
            child.setBoundingVolume(region);
        }
    }
    return children;
}

}

namespace overlume::testing {

void mapbox_set_test_clock(double seconds) { overlume::mapbox::g_testClockS.store(seconds); }

}
