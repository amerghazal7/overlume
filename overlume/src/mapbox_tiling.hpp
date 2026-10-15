// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

static_assert(__cplusplus >= 202002L, "mapbox_tiling.hpp is C++20-only (overlume_stream)");

// What the Mapbox loaders share: the z14 quadtree and its bounding regions, requests that never
// reject, the status rules, the retry schedule and the per-source log dedup.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <Cesium3DTilesSelection/BoundingVolume.h>
#include <Cesium3DTilesSelection/Tile.h>
#include <Cesium3DTilesSelection/TileLoadResult.h>
#include <Cesium3DTilesSelection/TilesetContentLoader.h>
#include <CesiumAsync/AsyncSystem.h>
#include <CesiumAsync/Future.h>
#include <CesiumAsync/IAssetAccessor.h>
#include <CesiumGeospatial/Ellipsoid.h>

#include "mapbox_tile_frame.hpp"

namespace spdlog {
class logger;
}

namespace overlume::mapbox {

inline constexpr uint32_t kMaxAttempts = 3;
// The wait before attempt 2, from attempt 1's start; doubled before attempt 3.
inline constexpr double kRetryWaitS = 2.0;
// Tile bounds span the anchor height -10 m to +1000 m until a built tile reports its real top.
inline constexpr double kRegionBelowM = 10.0;
inline constexpr double kRegionAboveM = 1000.0;

// Fixed when the source opens. Async work holds this and the loader's state by shared_ptr, never
// the loader itself, so switching the source away mid-build stays safe.
struct LoaderConfig {
    std::string token;  // goes into request URLs and nowhere else
    bool materials_original = false;
    double origin_height_m = 0.0;
};

// At most one log line per endpoint and status, or failure kind, per source.
struct SourceLog {
    bool first_time(const std::string& key);

private:
    std::mutex mutex_;
    std::set<std::string> logged_;
};

struct Attempt {
    uint32_t count = 0;
    double not_before_s = 0.0;  // clock_s() before which the next attempt may not start
    std::shared_ptr<std::atomic<bool>> settled;  // set by the build that stops retrying
};

// Attempt bookkeeping keyed by z14 tile. Main thread only: cesium calls loadTileContent there.
class AttemptBook {
public:
    // True while the tile's retry wait runs: its last attempt has not settled and now_s is before
    // that attempt's not_before_s. Cesium asks again for a RetryLater tile on its very next
    // selection; until the wait is over the answer is RetryLater again, with no request, no
    // attempt and no retries count.
    bool waiting(const TileId& t14, double now_s) const;
    // Starts the tile's next attempt: a fresh entry when there is none or the last build settled,
    // then count + 1 and not_before_s = now_s + kRetryWaitS * 2^(count - 1).
    Attempt& begin(const TileId& t14, double now_s);

private:
    // ponytail: one entry per z14 tile ever loaded; prune settled entries if drives ever span
    // thousands of tiles.
    std::unordered_map<uint64_t, Attempt> attempts_;
};

// The retry wait's clock in seconds: the steady clock, or where a test froze it
// (mapbox_set_test_clock).
double clock_s();

struct Fetched {
    uint16_t status = 0;
    std::vector<std::byte> body;  // kept for 200 only
    bool transport_error = false;
};

enum class Outcome { Body, Empty, Auth, Transient, Other };

Outcome classify(const Fetched& f);

// A status worth another attempt: 429, 5xx, or 0 (no HTTP answer: a transport error).
bool transient_status(uint16_t status);

// Never rejects: AsyncSystem::all would reject too, and cesium fails a tile whose load rejects
// for good. The transport error's text names the URL, token included, so it is dropped here.
CesiumAsync::Future<Fetched> fetch(const CesiumAsync::AsyncSystem& asyncSystem,
                                   const std::shared_ptr<CesiumAsync::IAssetAccessor>& accessor,
                                   const std::string& url);

// One line per endpoint and status per source; empty answers (204, 404) are normal and silent.
// Log lines name tiles, never URLs.
void note_status(SourceLog& seen, spdlog::logger& log, const char* endpoint, const Fetched& f,
                 const TileId& t14);

// A Success result for content in the tile's Z-up frame. pCompletedRequest stays null: cesium
// would copy that request's URL, token included, into the model.
Cesium3DTilesSelection::TileLoadResult success(
    Cesium3DTilesSelection::TileContentKind content,
    std::optional<Cesium3DTilesSelection::BoundingVolume> bounds,
    const std::shared_ptr<CesiumAsync::IAssetAccessor>& accessor,
    const CesiumGeospatial::Ellipsoid& ellipsoid);

// Tile t's bounding region from min_height_m to max_height_m above the ellipsoid.
Cesium3DTilesSelection::BoundingVolume tile_region(
    const TileId& t, double min_height_m, double max_height_m,
    const CesiumGeospatial::Ellipsoid& ellipsoid = CesiumGeospatial::Ellipsoid::WGS84);

// Web Mercator XYZ as a cesium quadtree: levels 0-13 are empty and always refined, and each
// level-14 tile carries the content. Ids carry Mapbox's XYZ y (north-down); only Mapbox loaders
// read them. The root and its single z0 child cover the Mercator world (tile_bounds of 0/0/0 is
// WebMercatorProjection::MAXIMUM_GLOBE_RECTANGLE), as LayerJsonTerrainLoader sets up EPSG:3857.
std::unique_ptr<Cesium3DTilesSelection::Tile> quadtree_root(
    Cesium3DTilesSelection::TilesetContentLoader& loader, double origin_height_m);

// The four children of a level 0-13 tile, on the parent's loader, row-major from the north-west;
// none for a z14 tile or a tile whose id is not a QuadtreeTileID. Each spans the anchor height
// -kRegionBelowM to +kRegionAboveM; a z14 child carries its tile frame as an absolute transform
// and geometric error 0.
std::vector<Cesium3DTilesSelection::Tile> quadtree_children(
    const Cesium3DTilesSelection::Tile& parent, const LoaderConfig& cfg,
    const CesiumGeospatial::Ellipsoid& ellipsoid = CesiumGeospatial::Ellipsoid::WGS84);

}
