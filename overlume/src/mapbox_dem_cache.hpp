// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

static_assert(__cplusplus >= 202002L, "mapbox_dem_cache.hpp is C++20-only (overlume_stream)");

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include <CesiumAsync/AsyncSystem.h>
#include <CesiumAsync/IAssetAccessor.h>
#include <CesiumAsync/SharedFuture.h>

#include "mapbox_dem.hpp"
#include "mapbox_tile_frame.hpp"
#include "mapbox_tiling.hpp"

namespace spdlog {
class logger;
}

namespace overlume::mapbox {

// Mapbox terrain-DEM: a 512 px terrain-RGB PNG per z14 tile. No query string.
inline constexpr char kDemUrlTemplate[] =
    "https://api.mapbox.com/v4/mapbox.mapbox-terrain-dem-v1/{z}/{x}/{y}@2x.pngraw";
inline constexpr size_t kDemCacheTiles = 32;  // about 1 MB of heights each

// `url_template` with `{z}`, `{x}` and `{y}` (XYZ: row 0 at the north edge) taken from t.
std::string fill_tile_template(std::string url_template, const TileId& t);

// Where a longitude and latitude fall in z14 tile t14's DEM: pixel coordinates u east, v south,
// 0..kDemSize across the tile (outside that range for a point outside the tile), as dem_sample
// takes them.
std::array<double, 2> dem_pixel_of(const LonLatDeg& p, const TileId& t14);

// The terrain-DEM tiles of one Mapbox source, shared by its buildings and terrain loaders.
class DemCache {
public:
    DemCache(std::shared_ptr<CesiumAsync::IAssetAccessor> accessor, CesiumAsync::AsyncSystem async,
             std::string dem_url_template_with_query,
             std::shared_ptr<spdlog::logger> redacting_logger);
    // One request per z14 tile, shared by every caller; LRU of 32 decoded tiles. The future
    // resolves to nullptr on 204/404 (empty) and on any failure (one log line per status, like the
    // loader). A failure for now (429, 5xx, transport) is shared for kRetryWaitS / 2 of steady
    // time, which covers every caller of one load attempt; a caller after that, a later attempt,
    // asks again.
    CesiumAsync::SharedFuture<std::shared_ptr<const DemTile>> get(const TileId& t14);
    // The DEM height at the anchor, memoised: the source's first request. While it has no value
    // (the anchor's DEM missing), each call derives it again through get().
    CesiumAsync::SharedFuture<std::optional<double>> reference_z(const LonLatDeg& anchor);
    // True when the tile's cached request failed for now; read once its future has resolved.
    bool failing(const TileId& t14) const;
    uint64_t requests() const;  // for stats

private:
    struct Entry {
        CesiumAsync::SharedFuture<std::shared_ptr<const DemTile>> dem;
        // Steady-clock seconds at which the request failed for now; negative otherwise.
        std::shared_ptr<std::atomic<double>> failed_at_s;
        std::list<uint64_t>::iterator recent;  // its place in recent_
    };

    std::shared_ptr<CesiumAsync::IAssetAccessor> accessor_;
    CesiumAsync::AsyncSystem async_;
    std::string urlTemplate_;  // carries the token: never logged
    std::shared_ptr<spdlog::logger> log_;
    std::shared_ptr<SourceLog> seen_ = std::make_shared<SourceLog>();
    mutable std::mutex mutex_;
    std::unordered_map<uint64_t, Entry> entries_;
    std::list<uint64_t> recent_;  // keys, most recently used first
    std::optional<CesiumAsync::SharedFuture<std::optional<double>>> reference_;
    std::atomic<uint64_t> requests_{0};
};

// A z14 tile's DEM and the anchor's height on its own DEM. With both, a point lifted by
// lift(u, v) stands on the terrain, which passes through z = 0 at the anchor.
struct Rebase {
    std::shared_ptr<const DemTile> dem;
    std::optional<double> z_ref_m;
    // One of them is missing because its request failed for now: worth another attempt.
    bool retry = false;
    bool active() const { return dem != nullptr && z_ref_m.has_value(); }
    // Metres to add at DEM pixel (u, v); active() only.
    double lift(double u, double v) const { return dem_sample(*dem, u, v) - *z_ref_m; }
};

// Main thread: the tile's DEM and the anchor's height, one request when the anchor lies in t14 and
// one per tile for both Mapbox loaders.
CesiumAsync::SharedFuture<Rebase> rebase_of(const std::shared_ptr<DemCache>& dem, const TileId& t14,
                                            const LonLatDeg& anchor);

}
