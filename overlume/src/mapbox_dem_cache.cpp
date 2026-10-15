// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_dem_cache.hpp"

#include "mapbox_test_hooks.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <utility>

namespace overlume::mapbox {

namespace {

constexpr char kDemEndpoint[] = "terrain-DEM";

uint64_t key_of(const TileId& t14) { return (uint64_t{t14.x} << 32) | t14.y; }

double steady_s() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// Worker thread: one DEM answer. Empty answers (204, 404) are normal and silent; any other answer
// without heights logs once per status, or per decode error, per source, naming the tile, never
// the URL. A failure for now also records when it happened.
std::shared_ptr<const DemTile> read_dem(const Fetched& f, spdlog::logger& log, SourceLog& seen,
                                        std::atomic<double>& failed_at_s, const TileId& t14) {
    const Outcome outcome = classify(f);
    if (outcome == Outcome::Body) {
        auto dem = std::make_shared<DemTile>(
            decode_dem_png(reinterpret_cast<const uint8_t*>(f.body.data()), f.body.size()));
        if (dem->ok) return dem;
        if (seen.first_time(std::string(kDemEndpoint) + " decode " + dem->error)) {
            log.warn("mapbox:// terrain tile 14/{}/{} skipped: {}", t14.x, t14.y, dem->error);
        }
        return nullptr;
    }
    if (outcome != Outcome::Transient) {
        note_status(seen, log, kDemEndpoint, f, t14);  // 401/403 and other statuses, once each
        return nullptr;
    }
    failed_at_s.store(steady_s());
    if (seen.first_time(std::string(kDemEndpoint) + " " + std::to_string(f.status))) {
        const std::string what =
            f.transport_error ? std::string("transport error") : "HTTP " + std::to_string(f.status);
        log.warn("mapbox: {} failing ({}) at tile 14/{}/{}; asked again on the tile's next attempt",
                 kDemEndpoint, what, t14.x, t14.y);
    }
    return nullptr;
}

}

std::string fill_tile_template(std::string url_template, const TileId& t) {
    const auto put = [&url_template](const std::string& key, uint32_t value) {
        const size_t at = url_template.find(key);
        if (at != std::string::npos) url_template.replace(at, key.size(), std::to_string(value));
    };
    put("{z}", t.z);
    put("{x}", t.x);
    put("{y}", t.y);
    return url_template;
}

std::array<double, 2> dem_pixel_of(const LonLatDeg& p, const TileId& t14) {
    // tile_containing's spherical Web Mercator, without the floor.
    constexpr double kMaxLatDeg = 85.0511287798;
    const double lat = std::clamp(p.lat, -kMaxLatDeg, kMaxLatDeg) * std::numbers::pi / 180.0;
    const double n = std::ldexp(1.0, static_cast<int>(t14.z));
    const double fx = (p.lon + 180.0) / 360.0 * n - t14.x;
    const double fy = (1.0 - std::asinh(std::tan(lat)) / std::numbers::pi) / 2.0 * n - t14.y;
    return {fx * kDemSize, fy * kDemSize};
}

DemCache::DemCache(std::shared_ptr<CesiumAsync::IAssetAccessor> accessor,
                   CesiumAsync::AsyncSystem async, std::string dem_url_template_with_query,
                   std::shared_ptr<spdlog::logger> redacting_logger)
    : accessor_(std::move(accessor)),
      async_(std::move(async)),
      urlTemplate_(std::move(dem_url_template_with_query)),
      log_(std::move(redacting_logger)) {}

CesiumAsync::SharedFuture<std::shared_ptr<const DemTile>> DemCache::get(const TileId& t14) {
    const uint64_t key = key_of(t14);
    std::lock_guard<std::mutex> lock(mutex_);
    if (const auto it = entries_.find(key); it != entries_.end()) {
        const double failedAt = it->second.failed_at_s->load();
        if (failedAt < 0.0 || steady_s() - failedAt < kRetryWaitS / 2.0) {
            recent_.splice(recent_.begin(), recent_, it->second.recent);
            return it->second.dem;
        }
        recent_.erase(it->second.recent);  // failed for now, a while ago: ask again
        entries_.erase(it);
    }
    requests_.fetch_add(1);
    auto failedAt = std::make_shared<std::atomic<double>>(-1.0);
    // fetch never rejects: the cesium callers would fail their tile for good.
    CesiumAsync::SharedFuture<std::shared_ptr<const DemTile>> dem =
        fetch(async_, accessor_, fill_tile_template(urlTemplate_, t14))
            .thenInWorkerThread([log = log_, seen = seen_, failedAt, t14](Fetched&& f) {
                return read_dem(f, *log, *seen, *failedAt, t14);
            })
            .share();
    recent_.push_front(key);
    entries_.emplace(key, Entry{dem, std::move(failedAt), recent_.begin()});
    if (entries_.size() > kDemCacheTiles) {
        entries_.erase(recent_.back());
        recent_.pop_back();
    }
    return dem;
}

CesiumAsync::SharedFuture<std::optional<double>> DemCache::reference_z(const LonLatDeg& anchor) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (reference_ && (!reference_->isReady() || reference_->wait().has_value())) {
            return *reference_;
        }
    }
    const TileId t = tile_containing(anchor.lon, anchor.lat, kLandmarkZoom);
    const std::array<double, 2> px = dem_pixel_of(anchor, t);
    CesiumAsync::SharedFuture<std::optional<double>> z =
        get(t)
            .thenImmediately(
                [px](const std::shared_ptr<const DemTile>& dem) -> std::optional<double> {
                    if (!dem) return std::nullopt;
                    return dem_sample(*dem, px[0], px[1]);
                })
            .share();
    std::lock_guard<std::mutex> lock(mutex_);
    reference_ = z;
    return z;
}

bool DemCache::failing(const TileId& t14) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = entries_.find(key_of(t14));
    return it != entries_.end() && it->second.failed_at_s->load() >= 0.0;
}

uint64_t DemCache::requests() const { return requests_.load(); }

CesiumAsync::SharedFuture<Rebase> rebase_of(const std::shared_ptr<DemCache>& dem, const TileId& t14,
                                            const LonLatDeg& anchor) {
    const TileId at = tile_containing(anchor.lon, anchor.lat, kLandmarkZoom);
    // The weak reference keeps the cache from owning its own futures' continuations.
    // ponytail: failing() reads the tile's entry when the rebase resolves; an entry evicted or
    // asked again in between reads as not failing, and that attempt settles at z = 0 as before
    // the retry existed.
    return dem->get(t14)
        .thenImmediately([ref = dem->reference_z(anchor), cache = std::weak_ptr<DemCache>(dem), t14,
                          at](const std::shared_ptr<const DemTile>& tile) mutable {
            return ref.thenImmediately([tile, cache, t14, at](const std::optional<double>& z) {
                Rebase r{tile, z};
                if (const std::shared_ptr<DemCache> c = cache.lock()) {
                    r.retry = (!tile && c->failing(t14)) || (!z && c->failing(at));
                }
                return r;
            });
        })
        .share();
}

}

namespace overlume::testing {

std::string mapbox_dem_url(uint32_t x14, uint32_t y14) {
    return overlume::mapbox::fill_tile_template(overlume::mapbox::kDemUrlTemplate,
                                                {overlume::mapbox::kLandmarkZoom, x14, y14});
}

}
