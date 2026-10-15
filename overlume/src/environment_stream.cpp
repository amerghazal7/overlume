// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "environment_stream.hpp"

#include "environment_test_hooks.hpp"
#include "gltf_normals.hpp"
#include "mapbox_test_hooks.hpp"
#include "renderer_internal.hpp"

#include <Cesium3DTilesContent/registerAllTileContentTypes.h>
#include <Cesium3DTilesSelection/BoundingVolume.h>
#include <Cesium3DTilesSelection/RasterMappedTo3DTile.h>
#include <CesiumAsync/CachingAssetAccessor.h>
#include <CesiumAsync/SqliteCache.h>
#include <CesiumCurl/CurlAssetAccessor.h>
#include <CesiumGeometry/BoundingSphere.h>
#include <CesiumGeometry/QuadtreeTileID.h>
#include <CesiumGeometry/Transforms.h>
#include <CesiumGeospatial/BoundingRegion.h>
#include <CesiumGeospatial/Cartographic.h>
#include <CesiumGeospatial/Ellipsoid.h>
#include <CesiumGeospatial/GlobeRectangle.h>
#include <CesiumGeospatial/GlobeTransforms.h>
#include <CesiumGeospatial/Projection.h>
#include <CesiumGeospatial/WebMercatorProjection.h>
#include <CesiumGltf/AccessorWriter.h>
#include <CesiumGltf/Model.h>
#include <CesiumGltfWriter/GltfWriter.h>
#include <CesiumImage/ImageAsset.h>
#include <CesiumRasterOverlays/IonRasterOverlay.h>
#include <CesiumRasterOverlays/CreateRasterOverlayTileProviderParameters.h>
#include <CesiumRasterOverlays/QuadtreeRasterOverlayTileProvider.h>
#include <CesiumRasterOverlays/RasterOverlay.h>
#include <CesiumRasterOverlays/RasterOverlayTile.h>
#include <CesiumRasterOverlays/RasterOverlayTileProvider.h>
#include <CesiumRasterOverlays/UrlTemplateRasterOverlay.h>
#include <CesiumUtility/CreditReferencer.h>
#include <CesiumUtility/CreditSystem.h>

#include <mutex>

#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/Texture.h>
#include <filament/TextureSampler.h>
#include <filament/TransformManager.h>

#include <backend/PixelBufferDescriptor.h>

#include <gltfio/FilamentInstance.h>

#include <utils/EntityManager.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <optional>
#include <unordered_set>

#ifdef _WIN32
#include <process.h>
#else
#include <unistd.h>
#endif

namespace overlume {

namespace {

struct IonSpec {
    int64_t asset_id = 0;
    std::string cache_dir;
    std::string fallback_dir;
    uint64_t max_cache_items = kDefaultMaxCacheItems;
    bool materials_original = false;
    bool follow_terrain = false;
    double ground_bias_m = 0.3;
    bool replaces_ground = true;
    double max_tilt_deg = 2.0;
    double brightness = 1.0;
    std::optional<double> radius_m;
    std::optional<int64_t> terrain_asset;  // terrain=<id>|off (default off)
    std::optional<int64_t> imagery_asset;  // imagery=<id>|off (default off); needs terrain_asset
};

// A refusal names what failed in `error` (when given), as the mapbox:// refusals do: the key and
// the value, or the asset id.
std::optional<IonSpec> parse_ion_spec(const std::string& spec, std::string* error = nullptr) {
    const auto refuse = [error](const std::string& why) {
        if (error) *error = why;
        return std::optional<IonSpec>();
    };
    const size_t q = spec.find('?');
    const std::string idPart = spec.substr(0, q);
    const auto badId = [&] {
        return refuse("the asset id '" + idPart + "' is not a positive integer");
    };
    if (idPart.empty()) return badId();
    IonSpec out;
    try {
        size_t consumed = 0;
        out.asset_id = std::stoll(idPart, &consumed);
        if (consumed != idPart.size() || out.asset_id <= 0) return badId();
    } catch (const std::exception&) {
        return badId();
    }
    if (q == std::string::npos) return out;
    std::string rest = spec.substr(q + 1);
    size_t pos = 0;
    while (pos < rest.size()) {
        const size_t amp = rest.find('&', pos);
        const std::string kv = rest.substr(pos, amp == std::string::npos ? amp : amp - pos);
        const size_t eq = kv.find('=');
        if (eq != std::string::npos) {
            const std::string key = kv.substr(0, eq);
            const std::string val = kv.substr(eq + 1);
            const auto bad = [&](const char* want) {
                return refuse(key + "='" + val + "' is not " + want);
            };
            if (key == "cache") {
                out.cache_dir = val;
            } else if (key == "fallback") {
                out.fallback_dir = val;
            } else if (key == "max_cache_items") {
                try {
                    out.max_cache_items = std::stoull(val);
                } catch (const std::exception&) {
                }
            } else if (key == "materials") {
                if (val == "original") {
                    out.materials_original = true;
                } else if (val == "clay") {
                    out.materials_original = false;
                } else {
                    static std::once_flag unknownMaterialsWarnOnce;
                    std::call_once(unknownMaterialsWarnOnce, [&val] {
                        spdlog::warn(
                            "environment_stream: unknown materials='{}' -- treating as clay", val);
                    });
                    out.materials_original = false;
                }
            } else if (key == "ground_bias") {
                try {
                    size_t used = 0;
                    out.ground_bias_m = std::stod(val, &used);
                    if (used != val.size() || !std::isfinite(out.ground_bias_m))
                        return bad("a finite number");
                } catch (const std::exception&) {
                    return bad("a finite number");
                }
            } else if (key == "follow_terrain") {
                if (val == "on" || val == "true" || val == "1") {
                    out.follow_terrain = true;
                } else if (val == "off" || val == "false" || val == "0") {
                    out.follow_terrain = false;
                } else {
                    return bad("on or off");
                }
            } else if (key == "max_tilt_deg") {
                try {
                    size_t used = 0;
                    out.max_tilt_deg = std::stod(val, &used);
                    if (used != val.size() || !std::isfinite(out.max_tilt_deg) ||
                        out.max_tilt_deg < 0.0)
                        return bad("a finite number >= 0");
                } catch (const std::exception&) {
                    return bad("a finite number >= 0");
                }
            } else if (key == "brightness") {
                try {
                    size_t used = 0;
                    out.brightness = std::stod(val, &used);
                    if (used != val.size() || !std::isfinite(out.brightness) ||
                        out.brightness <= 0.0)
                        return bad("a finite number > 0");
                } catch (const std::exception&) {
                    return bad("a finite number > 0");
                }
            } else if (key == "replaces_ground") {
                if (val == "on" || val == "true" || val == "1") {
                    out.replaces_ground = true;
                } else if (val == "off" || val == "false" || val == "0") {
                    out.replaces_ground = false;
                } else {
                    return bad("on or off");
                }
            } else if (key == "radius") {
                try {
                    size_t used = 0;
                    const double radius = std::stod(val, &used);
                    if (used != val.size() || !std::isfinite(radius) || radius <= 0.0)
                        return bad("a finite number > 0");
                    out.radius_m = radius;
                } catch (const std::exception&) {
                    return bad("a finite number > 0");
                }
            } else if (key == "terrain" || key == "imagery") {
                std::optional<int64_t>& asset =
                    key == "terrain" ? out.terrain_asset : out.imagery_asset;
                if (val == "off") {
                    asset.reset();
                } else {
                    // An ion asset id: digits only (std::stoll alone takes "+1" and " 1"), > 0.
                    if (val.empty() || val.find_first_not_of("0123456789") != std::string::npos)
                        return bad("a positive asset id or 'off'");
                    try {
                        asset = std::stoll(val);
                    } catch (const std::exception&) {
                        return bad("a positive asset id or 'off'");  // out of range
                    }
                    if (*asset <= 0) return bad("a positive asset id or 'off'");
                }
            }
        }
        if (amp == std::string::npos) break;
        pos = amp + 1;
    }
    // The imagery drapes on the terrain tileset: without one it has nothing to drape on.
    if (out.imagery_asset && !out.terrain_asset)
        return refuse("imagery=" + std::to_string(*out.imagery_asset) +
                      " needs terrain=<asset id>");
    return out;
}

std::string default_cache_dir() {
    const char* xdg = std::getenv("XDG_CACHE_HOME");
#ifdef _WIN32
    const char* home = std::getenv("USERPROFILE");
#else
    const char* home = std::getenv("HOME");
#endif
    const std::string base = (xdg && xdg[0]) ? xdg : (std::string(home ? home : ".") + "/.cache");
    return base + "/overlume-tile-cache";
}

filament::math::mat4f to_filament_mat4(const glm::dmat4& m) {
    filament::math::mat4f out;
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            out[c][r] = static_cast<float>(m[c][r]);
        }
    }
    return out;
}

bool consolidate_buffers(CesiumGltf::Model& model) {
    if (model.buffers.empty()) return false;
    if (model.buffers.size() == 1) return true;

    std::vector<size_t> newOffset(model.buffers.size(), 0);
    size_t total = 0;
    for (size_t i = 0; i < model.buffers.size(); ++i) {
        newOffset[i] = total;
        total += model.buffers[i].cesium.data.size();
        if (const size_t rem = total % 4; rem != 0) total += (4 - rem);
    }
    std::vector<std::byte> merged;
    merged.reserve(total);
    for (const CesiumGltf::Buffer& buf : model.buffers) {
        merged.insert(merged.end(), buf.cesium.data.begin(), buf.cesium.data.end());
        if (const size_t rem = merged.size() % 4; rem != 0)
            merged.resize(merged.size() + (4 - rem));
    }
    for (CesiumGltf::BufferView& bv : model.bufferViews) {
        if (bv.buffer < 0 || static_cast<size_t>(bv.buffer) >= newOffset.size()) continue;
        bv.byteOffset += static_cast<int64_t>(newOffset[static_cast<size_t>(bv.buffer)]);
        bv.buffer = 0;
    }
    model.buffers.resize(1);
    model.buffers[0].byteLength = static_cast<int64_t>(merged.size());
    model.buffers[0].cesium.data = std::move(merged);
    model.buffers[0].uri.reset();
    return true;
}

// Terrain tiles: cesium writes overlay n's texture coordinates as _CESIUMOVERLAY_<n>, which the
// strip below drops (gltfio refuses non-core attributes). TEXCOORD_0 keeps overlay 0's: gltfio
// loads it as UV0 when the tile's own material samples no texture (AssetLoader.cpp, the UNUSED
// slot), and terrain.mat reads UV0.
void copy_overlay_uvs_to_texcoord0(CesiumGltf::Model& model) {
    for (CesiumGltf::Mesh& mesh : model.meshes) {
        for (CesiumGltf::MeshPrimitive& prim : mesh.primitives) {
            const auto it = prim.attributes.find("_CESIUMOVERLAY_0");
            if (it == prim.attributes.end()) continue;
            const int32_t accessor = it->second;
            prim.attributes["TEXCOORD_0"] = accessor;
        }
    }
}

void strip_attributes_and_correct_heights(CesiumGltf::Model& model, const glm::dmat4& modelToEcef,
                                          const glm::dmat4& ecefToMap, double anchorHeightM) {
    for (CesiumGltf::Mesh& mesh : model.meshes) {
        for (CesiumGltf::MeshPrimitive& prim : mesh.primitives) {
            for (auto it = prim.attributes.begin(); it != prim.attributes.end();) {
                if (!it->first.empty() && it->first[0] == '_') {
                    it = prim.attributes.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }

    std::unordered_set<int32_t> correctedPositionAccessors;
    model.forEachPrimitiveInScene(
        -1, [&](CesiumGltf::Model& m, CesiumGltf::Node&, CesiumGltf::Mesh&,
                CesiumGltf::MeshPrimitive& prim, const glm::dmat4& nodeTransform) {
            const auto posIt = prim.attributes.find("POSITION");
            if (posIt == prim.attributes.end()) return;
            if (!correctedPositionAccessors.insert(posIt->second).second) return;
            const glm::dmat4 localToEcef = modelToEcef * nodeTransform;
            const glm::dmat4 mapToLocal = glm::inverse(ecefToMap * localToEcef);
            CesiumGltf::AccessorWriter<glm::vec3> pos(m, posIt->second);
            if (pos.status() != CesiumGltf::AccessorViewStatus::Valid) {
                static std::once_flag quantizedWarnOnce;
                std::call_once(quantizedWarnOnce, [&] {
                    make_redacting_logger()->warn(
                        "strip_attributes_and_correct_heights: POSITION accessor {} is not a "
                        "valid float32 VEC3 (status {}); height correction skipped, tile may "
                        "sag (further occurrences not logged)",
                        posIt->second, static_cast<int>(pos.status()));
                });
                return;
            }
            for (int64_t i = 0; i < pos.size(); ++i) {
                glm::vec3& p = pos[i];
                const glm::dvec4 ecefPos = localToEcef * glm::dvec4(glm::dvec3(p), 1.0);
                const glm::dvec3 newMapPos =
                    correct_ecef_point_height(glm::dvec3(ecefPos), ecefToMap, anchorHeightM);
                const glm::dvec4 newLocalPos = mapToLocal * glm::dvec4(newMapPos, 1.0);
                p = glm::vec3(newLocalPos);
            }
        });
}

std::string redact_credentials(std::string text) {
    auto redact_after = [&text](const std::string& marker) {
        size_t pos = 0;
        while ((pos = text.find(marker, pos)) != std::string::npos) {
            const size_t valueStart = pos + marker.size();
            size_t valueEnd = valueStart;
            while (valueEnd < text.size() && text[valueEnd] != '&' &&
                   std::isspace(static_cast<unsigned char>(text[valueEnd])) == 0) {
                ++valueEnd;
            }
            static constexpr char kRedacted[] = "<redacted>";
            text.replace(valueStart, valueEnd - valueStart, kRedacted);
            pos = valueStart + (sizeof(kRedacted) - 1);
        }
    };
    redact_after("access_token=");
    redact_after("Bearer ");
    return text;
}

class RedactingSink final : public spdlog::sinks::base_sink<std::mutex> {
public:
    explicit RedactingSink(std::shared_ptr<spdlog::sinks::sink> inner) : inner_(std::move(inner)) {}

    std::string captured_text_for_test() {
        std::lock_guard<std::mutex> lock(mutex_);
        return captured_;
    }

protected:
    void sink_it_(const spdlog::details::log_msg& msg) override {
        std::string redacted =
            redact_credentials(std::string(msg.payload.data(), msg.payload.size()));
        if (captured_.size() < kMaxCapturedBytes) {
            captured_ += redacted;
            captured_ += '\n';
        }
        spdlog::details::log_msg redactedMsg(
            msg.time, msg.source, msg.logger_name, msg.level,
            spdlog::string_view_t(redacted.data(), redacted.size()));
        inner_->log(redactedMsg);
    }
    void flush_() override { inner_->flush(); }

private:
    static constexpr size_t kMaxCapturedBytes = 64 * 1024;
    std::shared_ptr<spdlog::sinks::sink> inner_;
    std::string captured_;
};

std::shared_ptr<RedactingSink> g_redactingSinkForTest;

}

std::shared_ptr<spdlog::logger> make_redacting_logger() {
    static const std::shared_ptr<spdlog::logger> logger = [] {
        auto inner = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        auto redacting = std::make_shared<RedactingSink>(inner);
        g_redactingSinkForTest = redacting;
        auto l = std::make_shared<spdlog::logger>("overlume.cesium", redacting);
        spdlog::register_logger(l);
        return l;
    }();
    return logger;
}

namespace {

class TokenBypassAssetAccessor final : public CesiumAsync::IAssetAccessor {
public:
    TokenBypassAssetAccessor(std::shared_ptr<CesiumAsync::IAssetAccessor> cached,
                             std::shared_ptr<CesiumAsync::IAssetAccessor> direct)
        : cached_(std::move(cached)), direct_(std::move(direct)) {}

    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> get(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& url,
        const std::vector<THeader>& headers) override {
        return pick(url)->get(asyncSystem, url, headers);
    }
    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> request(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& verb,
        const std::string& url, const std::vector<THeader>& headers,
        const std::span<const std::byte>& payload) override {
        return pick(url)->request(asyncSystem, verb, url, headers, payload);
    }
    void tick() noexcept override { cached_->tick(); }

private:
    CesiumAsync::IAssetAccessor* pick(const std::string& url) const {
        return (url.find("access_token=") != std::string::npos) ? direct_.get() : cached_.get();
    }
    std::shared_ptr<CesiumAsync::IAssetAccessor> cached_;
    std::shared_ptr<CesiumAsync::IAssetAccessor> direct_;
};

}

Cesium3DTilesSelection::TilesetExternals build_externals(
    std::shared_ptr<CesiumAsync::IAssetAccessor> base, const CesiumAsync::AsyncSystem& asyncSystem,
    const std::string& cache_dir, uint64_t max_cache_items,
    std::shared_ptr<CountingAssetAccessor>* out_counting) {
    auto counting = std::make_shared<CountingAssetAccessor>(std::move(base));
    if (out_counting != nullptr) *out_counting = counting;
    auto logger = make_redacting_logger();

    // Every source gets a credit system: overlays create their credits only through one, and a
    // layer.json terrain tileset dereferences it unchecked (TilesetContentManager.cpp:2244-2250).
    auto credits = std::make_shared<CesiumUtility::CreditSystem>();

    if (cache_dir == "off") {
        Cesium3DTilesSelection::TilesetExternals externals{nullptr, nullptr, asyncSystem};
        externals.pAssetAccessor = counting;
        externals.pLogger = logger;
        externals.pCreditSystem = credits;
        return externals;
    }

    std::error_code ec;
    std::filesystem::create_directories(cache_dir, ec);
    auto cacheDb = std::make_shared<CesiumAsync::SqliteCache>(
        logger, cache_dir + "/cesium-tiles.sqlite", max_cache_items);
    auto caching = std::make_shared<CesiumAsync::CachingAssetAccessor>(logger, counting, cacheDb);
    auto tokenSafe = std::make_shared<TokenBypassAssetAccessor>(caching, counting);

    Cesium3DTilesSelection::TilesetExternals externals{nullptr, nullptr, asyncSystem};
    externals.pAssetAccessor = tokenSafe;
    externals.pLogger = logger;
    externals.pCreditSystem = credits;
    return externals;
}

glm::dmat4 compute_ecef_to_map(const GeoAnchor& anchor) {
    const CesiumGeospatial::LocalHorizontalCoordinateSystem enu(
        CesiumGeospatial::Cartographic::fromDegrees(anchor.origin_lon_deg, anchor.origin_lat_deg,
                                                    anchor.origin_height_m));
    const glm::dmat4 ecefToEnu = enu.getEcefToLocalTransformation();

    constexpr double kWgs84A = 6378137.0;
    constexpr double kWgs84F = 1.0 / 298.257223563;
    constexpr double kWgs84E2 = kWgs84F * (2.0 - kWgs84F);
    constexpr double kMapSphereRadiusM = 6371000.0;
    const double lat0_rad = anchor.origin_lat_deg * (M_PI / 180.0);
    const double sin2Lat0 = std::sin(lat0_rad) * std::sin(lat0_rad);
    const double denom = 1.0 - kWgs84E2 * sin2Lat0;
    const double N = kWgs84A / std::sqrt(denom);
    const double M = kWgs84A * (1.0 - kWgs84E2) / (denom * std::sqrt(denom));
    glm::dmat4 sphereScale(1.0);
    sphereScale[0][0] = kMapSphereRadiusM / N;
    sphereScale[1][1] = kMapSphereRadiusM / M;

    const double s = std::sin(anchor.heading_rad), c = std::cos(anchor.heading_rad);
    glm::dmat4 rot(1.0);
    rot[0][0] = s;
    rot[1][0] = c;
    rot[0][1] = -c;
    rot[1][1] = s;
    return rot * sphereScale * ecefToEnu;
}

glm::dvec3 correct_ecef_point_height(const glm::dvec3& ecef_pos, const glm::dmat4& ecef_to_map,
                                     double anchor_height_m) {
    const glm::dvec4 oldMapPos = ecef_to_map * glm::dvec4(ecef_pos, 1.0);
    const auto carto = CesiumGeospatial::Ellipsoid::WGS84.cartesianToCartographic(ecef_pos);
    const double zMap = carto.has_value() ? carto->height - anchor_height_m : oldMapPos.z;
    return glm::dvec3(oldMapPos.x, oldMapPos.y, zMap);
}

namespace {

class FixtureAssetResponse final : public CesiumAsync::IAssetResponse {
public:
    FixtureAssetResponse(uint16_t status, std::vector<std::byte> data)
        : status_(status), data_(std::move(data)) {
        if (status_ == 200) headers_["Cache-Control"] = "max-age=3600";
    }
    uint16_t statusCode() const override { return status_; }
    std::string contentType() const override { return "application/octet-stream"; }
    const CesiumAsync::HttpHeaders& headers() const override { return headers_; }
    std::span<const std::byte> data() const override {
        return std::span<const std::byte>(data_.data(), data_.size());
    }

private:
    uint16_t status_;
    std::vector<std::byte> data_;
    CesiumAsync::HttpHeaders headers_;
};

class FixtureAssetRequest final : public CesiumAsync::IAssetRequest {
public:
    FixtureAssetRequest(std::string method, std::string url,
                        std::unique_ptr<FixtureAssetResponse> resp)
        : method_(std::move(method)), url_(std::move(url)), resp_(std::move(resp)) {}
    const std::string& method() const override { return method_; }
    const std::string& url() const override { return url_; }
    const CesiumAsync::HttpHeaders& headers() const override { return headers_; }
    const CesiumAsync::IAssetResponse* response() const override { return resp_.get(); }

private:
    std::string method_, url_;
    std::unique_ptr<FixtureAssetResponse> resp_;
    CesiumAsync::HttpHeaders headers_;
};

constexpr char kFileScheme[] = "file://";

// Both fixture switches spare the root manifest: behind the cache its request runs on the cache
// thread, possibly after a test has already flipped a switch.
bool is_root_manifest(const std::string& url) {
    return url.size() >= 12 && url.compare(url.size() - 12, 12, "tileset.json") == 0;
}

}

CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> FileFixtureAssetAccessor::respond(
    const CesiumAsync::AsyncSystem& asyncSystem, const std::string& verb, const std::string& url) {
    if (transportDown_ && transportDown_->load() && !is_root_manifest(url)) {
        const auto promise =
            asyncSystem.createPromise<std::shared_ptr<CesiumAsync::IAssetRequest>>();
        // curl's text carries the request URL, token included; this one carries a bogus token.
        promise.reject(std::runtime_error("fixture transport error: " + url +
                                          "?access_token=fixture-transport-token"));
        return promise.getFuture();
    }
    return asyncSystem.createResolvedFuture(makeRequest(verb, url));
}

std::shared_ptr<CesiumAsync::IAssetRequest> FileFixtureAssetAccessor::makeRequest(
    const std::string& verb, const std::string& url) {
    if (killed_ && killed_->load() && !is_root_manifest(url)) {
        return std::make_shared<FixtureAssetRequest>(
            verb, url, std::make_unique<FixtureAssetResponse>(503, std::vector<std::byte>{}));
    }
    std::string path = url;
    if (path.rfind(kFileScheme, 0) == 0) path = path.substr(sizeof(kFileScheme) - 1);
#ifdef _WIN32
    // Cesium resolves tile URIs against the tileset's to file:///D:/dir/x.b3dm: drop the slash
    // before the drive.
    if (path.size() > 2 && path[0] == '/' && path[2] == ':') path.erase(0, 1);
#endif
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        return std::make_shared<FixtureAssetRequest>(
            verb, url, std::make_unique<FixtureAssetResponse>(404, std::vector<std::byte>{}));
    }
    const std::streamsize size = file.tellg();
    std::vector<std::byte> bytes(static_cast<size_t>(size));
    file.seekg(0);
    file.read(reinterpret_cast<char*>(bytes.data()), size);
    return std::make_shared<FixtureAssetRequest>(
        verb, url, std::make_unique<FixtureAssetResponse>(200, std::move(bytes)));
}

void MapFixtureAssetAccessor::serve(const std::string& url_without_query, uint16_t status,
                                    std::vector<std::byte> body) {
    std::lock_guard<std::mutex> lock(mutex_);
    canned_[url_without_query] = Canned{status, std::move(body), false};
}

void MapFixtureAssetAccessor::fail_transport(const std::string& url_without_query) {
    std::lock_guard<std::mutex> lock(mutex_);
    canned_[url_without_query] = Canned{0, {}, true};
}

std::vector<std::string> MapFixtureAssetAccessor::requested_urls() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return requested_;
}

void MapFixtureAssetAccessor::hold(const std::string& url_without_query) {
    std::lock_guard<std::mutex> lock(mutex_);
    held_.push_back(url_without_query);
}

void MapFixtureAssetAccessor::serve_prefix(const std::string& prefix, uint16_t status,
                                           std::vector<std::byte> body) {
    std::lock_guard<std::mutex> lock(mutex_);
    prefixes_.emplace_back(prefix, Canned{status, std::move(body), false});
}

void MapFixtureAssetAccessor::hold_prefix(const std::string& prefix) {
    std::lock_guard<std::mutex> lock(mutex_);
    heldPrefixes_.push_back(prefix);
}

void MapFixtureAssetAccessor::fail_all_transport() {
    std::lock_guard<std::mutex> lock(mutex_);
    allTransportDown_ = true;
}

void MapFixtureAssetAccessor::release_held() {
    std::vector<Held> pending;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        held_.clear();
        heldPrefixes_.clear();
        pending.swap(pending_);
    }
    // Outside the lock: continuations run inline on this thread when a promise resolves.
    for (const Held& h : pending) answer(h.promise, h.verb, h.url);
}

CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> MapFixtureAssetAccessor::respond(
    const CesiumAsync::AsyncSystem& asyncSystem, const std::string& verb, const std::string& url) {
    const std::string key = url.substr(0, url.find('?'));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        requested_.push_back(url);
        const auto startsKey = [&key](const std::string& p) { return key.rfind(p, 0) == 0; };
        if (std::find(held_.begin(), held_.end(), key) != held_.end() ||
            (canned_.find(key) == canned_.end() &&
             std::any_of(heldPrefixes_.begin(), heldPrefixes_.end(), startsKey))) {
            pending_.push_back(
                Held{asyncSystem.createPromise<std::shared_ptr<CesiumAsync::IAssetRequest>>(), verb,
                     url});
            return pending_.back().promise.getFuture();
        }
    }
    return asyncSystem.createFuture<std::shared_ptr<CesiumAsync::IAssetRequest>>(
        [&](const RequestPromise& promise) { answer(promise, verb, url); });
}

void MapFixtureAssetAccessor::answer(const RequestPromise& promise, const std::string& verb,
                                     const std::string& url) const {
    Canned canned{unmatchedStatus_, {}, false};
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::string key = url.substr(0, url.find('?'));
        if (allTransportDown_) {
            canned.transport_error = true;
        } else if (const auto it = canned_.find(key); it != canned_.end()) {
            canned = it->second;
        } else {
            size_t longest = 0;
            for (const auto& [prefix, rule] : prefixes_) {
                if (key.rfind(prefix, 0) == 0 && prefix.size() >= longest) {
                    canned = rule;
                    longest = prefix.size();
                }
            }
        }
    }
    if (canned.transport_error) {
        // Like curl's, the message carries the URL: only a redacting logger may print it.
        promise.reject(std::runtime_error("fixture transport error for " + url));
        return;
    }
    std::shared_ptr<CesiumAsync::IAssetRequest> request = std::make_shared<FixtureAssetRequest>(
        verb, url, std::make_unique<FixtureAssetResponse>(canned.status, std::move(canned.body)));
    promise.resolve(std::move(request));
}

struct TerrainFollowState {
    std::atomic<bool> in_flight{false};
    std::optional<double> fit_slope;
    std::optional<double> fit_intercept;
    double fit_rms_m = 0.0;
    int fit_hit_count = 0;
    double fit_pivot_x = 0.0;
    double fit_pivot_y = 0.0;
    double fit_pivot_heading_rad = 0.0;
    double sampled_x = 0.0;
    double sampled_y = 0.0;
    std::chrono::steady_clock::time_point sampled_at{};
    bool ever_sampled = false;
    double anchor_height_m = 0.0;
    double ground_bias_m = 0.0;
    double max_tilt_rad = 0.0;
    std::atomic<bool> ground_hit{false};
};

namespace {
void terrain_plane_fit(const double* s, const double* h, int n, double* out_slope,
                       double* out_intercept, double* out_rms) {
    double a = 0.0, c = 0.0;
    if (n >= 3) {
        double sumS = 0.0, sumH = 0.0, sumSS = 0.0, sumSH = 0.0;
        for (int i = 0; i < n; ++i) {
            sumS += s[i];
            sumH += h[i];
            sumSS += s[i] * s[i];
            sumSH += s[i] * h[i];
        }
        const double denom = static_cast<double>(n) * sumSS - sumS * sumS;
        if (std::abs(denom) > 1e-6) {
            a = (static_cast<double>(n) * sumSH - sumS * sumH) / denom;
            c = (sumH - a * sumS) / static_cast<double>(n);
        } else {
            a = 0.0;
            c = sumH / static_cast<double>(n);
        }
    } else if (n > 0) {
        double sumH = 0.0;
        for (int i = 0; i < n; ++i) sumH += h[i];
        c = sumH / static_cast<double>(n);
    }
    double sumSq = 0.0;
    for (int i = 0; i < n; ++i) {
        const double residual = a * s[i] + c - h[i];
        sumSq += residual * residual;
    }
    if (out_slope) *out_slope = a;
    if (out_intercept) *out_intercept = c;
    if (out_rms) *out_rms = n > 0 ? std::sqrt(sumSq / static_cast<double>(n)) : 0.0;
}

double terrain_tilt_target(double slope, double max_tilt_rad) {
    return std::clamp(std::atan(slope), -max_tilt_rad, max_tilt_rad);
}

filament::math::mat4f terrain_root_matrix(double z, double tilt_rad, double pivot_x, double pivot_y,
                                          double heading_rad) {
    using filament::math::float3;
    using filament::math::mat4f;
    const float3 pivot{static_cast<float>(pivot_x), static_cast<float>(pivot_y), 0.0f};
    const float3 lateralAxis{static_cast<float>(-std::sin(heading_rad)),
                             static_cast<float>(std::cos(heading_rad)), 0.0f};
    return mat4f::translation(pivot) * mat4f::rotation(static_cast<float>(tilt_rad), lateralAxis) *
           mat4f::translation(-pivot) *
           mat4f::translation(float3{0.0f, 0.0f, static_cast<float>(z)});
}
}

bool StreamingEnvironmentSource::provides_ground() const {
    return !fallenBack_ && replacesGround_ && terrainState_ && terrainState_->ground_hit.load();
}

void StreamingEnvironmentSource::attach_terrain(
    std::unique_ptr<Cesium3DTilesSelection::Tileset> terrain,
    std::shared_ptr<StreamRendererResources> terrain_resources) {
    terrainTileset_ = std::move(terrain);
    terrainResources_ = std::move(terrain_resources);
}

bool StreamingEnvironmentSource::has_terrain() const {
    // A terrain tileset whose root never came (layer.json refused, asset missing) has no root once
    // its root-available event is ready: the source then runs on its content tileset alone.
    return terrainTileset_ != nullptr && !(terrainTileset_->getRootTileAvailableEvent().isReady() &&
                                           terrainTileset_->getRootTile() == nullptr);
}

namespace {

// The plain text of an attribution's HTML: tags dropped, a few entities decoded, whitespace runs
// collapsed. ponytail: a tag scanner, not an HTML parser (a '>' inside an attribute value ends
// the tag early); use a real parser if an attribution ever shows markup on the line.
std::string credit_plain_text(const std::string& html) {
    static constexpr std::pair<const char*, const char*> kEntities[] = {
        {"&amp;", "&"}, {"&lt;", "<"},   {"&gt;", ">"},         {"&quot;", "\""},
        {"&#39;", "'"}, {"&nbsp;", " "}, {"&copy;", "\xC2\xA9"}};
    std::string out;
    bool inTag = false;
    for (size_t i = 0; i < html.size(); ++i) {
        const char c = html[i];
        if (inTag || c == '<') {
            inTag = c != '>';
            continue;
        }
        if (std::isspace(static_cast<unsigned char>(c)) != 0) {
            if (!out.empty() && out.back() != ' ') out += ' ';
            continue;
        }
        const auto entity = std::find_if(
            std::begin(kEntities), std::end(kEntities),
            [&](const auto& e) { return html.compare(i, std::strlen(e.first), e.first) == 0; });
        if (entity != std::end(kEntities)) {
            out += entity->second;
            i += std::strlen(entity->first) - 1;
            continue;
        }
        out += c;
    }
    while (!out.empty() && out.back() == ' ') out.pop_back();
    return out;
}

}

std::string StreamingEnvironmentSource::credits() const {
    if (fallenBack_ || tornDown_) return {};
    std::vector<std::string> texts;
    for (const Cesium3DTilesSelection::Tileset* tileset : {tileset_.get(), terrainTileset_.get()}) {
        if (tileset == nullptr || !tileset->getExternals().pCreditSystem) continue;
        CesiumUtility::CreditSystem& creditSystem = *tileset->getExternals().pCreditSystem;
        // CreditReferencer cannot list its credits: reference the overlays' credits, then keep the
        // system's referenced credits that this referencer holds.
        CesiumUtility::CreditReferencer overlayCredits(tileset->getExternals().pCreditSystem);
        for (const auto& activated : tileset->getOverlays().getActivatedOverlays()) {
            if (activated->getTileProvider() != nullptr) {
                activated->getTileProvider()->addCredits(overlayCredits);
            }
        }
        // Bing's data-provider credits (Microsoft, Earthstar, Maxar) belong to individual raster
        // tiles, not to the provider: add those of the rasters mapped onto the tileset's tiles.
        // The snapshot filter below keeps the ones the view group referenced this frame.
        for (const Cesium3DTilesSelection::Tile& tile : tileset->loadedTiles()) {
            for (const Cesium3DTilesSelection::RasterMappedTo3DTile& mapped :
                 tile.getMappedRasterTiles()) {
                if (const CesiumRasterOverlays::RasterOverlayTile* raster = mapped.getReadyTile()) {
                    for (const CesiumUtility::Credit& credit : raster->getCredits()) {
                        overlayCredits.addCreditReference(credit);
                    }
                }
            }
        }
        for (const CesiumUtility::Credit& credit :
             creditSystem.getSnapshot(CesiumUtility::CreditFilteringMode::None).currentCredits) {
            if (!overlayCredits.isCreditReferenced(credit)) continue;
            std::string text = credit_plain_text(creditSystem.getHtml(credit));
            if (!text.empty() && std::find(texts.begin(), texts.end(), text) == texts.end()) {
                texts.push_back(std::move(text));
            }
        }
    }
    std::string out;
    for (const std::string& text : texts) out += (out.empty() ? "" : "; ") + text;
    return out;
}

namespace {
size_t count_in_scene(VisualRenderer& r, const std::unordered_map<const void*, bool>& tracked) {
    size_t count = 0;
    for (const auto& [res, _] : tracked) {
        auto* asset = static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(res));
        if (asset->getEntityCount() > 0 && r.scene->hasEntity(asset->getEntities()[0])) {
            ++count;
        }
    }
    return count;
}
}

size_t StreamingEnvironmentSource::terrain_scene_count(VisualRenderer& r) const {
    return fallenBack_ ? 0 : count_in_scene(r, terrainInScene_);
}

double StreamingEnvironmentSource::terrain_tile_lift_z(VisualRenderer& r) const {
    if (terrainInScene_.empty() || r.engine == nullptr) return std::nan("");
    auto* asset = static_cast<filament::gltfio::FilamentAsset*>(
        const_cast<void*>(terrainInScene_.begin()->first));
    filament::TransformManager& tm = r.engine->getTransformManager();
    const auto inst = tm.getInstance(asset->getRoot());
    if (!inst.isValid()) return std::nan("");
    return static_cast<double>(tm.getWorldTransform(inst)[3].z - tm.getTransform(inst)[3].z);
}

void StreamRendererResources::ensure_terrain_root() {
    if (terrainRoot_ || r_ == nullptr) return;
    terrainRoot_ = utils::EntityManager::get().create();
    r_->engine->getTransformManager().create(terrainRoot_);
}

void StreamRendererResources::set_terrain_transform(double z, double tilt_rad, double pivot_x,
                                                    double pivot_y, double heading_rad) {
    ensure_terrain_root();
    if (!terrainRoot_) return;
    filament::TransformManager& tm = r_->engine->getTransformManager();
    const auto inst = tm.getInstance(terrainRoot_);
    if (!inst.isValid()) return;
    tm.setTransform(inst, terrain_root_matrix(z, tilt_rad, pivot_x, pivot_y, heading_rad));
}

void StreamRendererResources::destroy_terrain_root() {
    if (!terrainRoot_ || r_ == nullptr) return;
    r_->engine->destroy(terrainRoot_);
    utils::EntityManager::get().destroy(terrainRoot_);
    terrainRoot_ = {};
}

namespace {

// Test-only (capture_stream_glbs): the GLBs prepareInLoadThread writes while the flag is on.
std::atomic<bool> g_captureStreamGlbs{false};
std::mutex g_capturedGlbsMutex;
std::vector<std::vector<uint8_t>> g_capturedGlbs;

}

CesiumAsync::Future<Cesium3DTilesSelection::TileLoadResultAndRenderResources>
StreamRendererResources::prepareInLoadThread(
    const CesiumAsync::AsyncSystem& asyncSystem,
    Cesium3DTilesSelection::TileLoadResult&& tileLoadResult, const glm::dmat4& transform,
    const std::any&) {
    auto* pGlb = new LoadThreadGlb();
    pGlb->transform = transform * CesiumGeometry::Transforms::getUpAxisTransform(
                                      tileLoadResult.glTFUpAxis, CesiumGeometry::Axis::Z);
    if (auto* keptModel = std::get_if<CesiumGltf::Model>(&tileLoadResult.contentKind)) {
        // cesium keeps this model for height queries and for upsampling a terrain tile's children
        // (which needs its _CESIUMOVERLAY_ attributes and its own positions), so a terrain tile
        // is rewritten on a copy; a content tile's model is rewritten in place, as ever.
        CesiumGltf::Model terrainCopy;
        CesiumGltf::Model* model = keptModel;
        if (kind_ == Kind::Terrain) {
            terrainCopy = *keptModel;
            model = &terrainCopy;
        }
        if (consolidate_buffers(*model)) {
            if (kind_ == Kind::Terrain) copy_overlay_uvs_to_texcoord0(*model);
            strip_attributes_and_correct_heights(*model, pGlb->transform, ecefToMap_,
                                                 anchorHeightM_);
            const auto& bufData = model->buffers[0].cesium.data;
            CesiumGltfWriter::GltfWriter writer;
            const CesiumGltfWriter::GltfWriterResult res =
                writer.writeGlb(*model, std::span<const std::byte>(bufData.data(), bufData.size()));
            if (res.errors.empty()) {
                std::vector<uint8_t> bytes(
                    reinterpret_cast<const uint8_t*>(res.gltfBytes.data()),
                    reinterpret_cast<const uint8_t*>(res.gltfBytes.data() + res.gltfBytes.size()));
                bytes = ensure_flat_normals(std::move(bytes));
                if (g_captureStreamGlbs.load()) {
                    std::lock_guard<std::mutex> lock(g_capturedGlbsMutex);
                    g_capturedGlbs.push_back(bytes);
                }
                pGlb->glbBytes.assign(
                    reinterpret_cast<const std::byte*>(bytes.data()),
                    reinterpret_cast<const std::byte*>(bytes.data() + bytes.size()));
                pGlb->ok = true;
            }
        }
    }
    Cesium3DTilesSelection::TileLoadResultAndRenderResources out{std::move(tileLoadResult), pGlb};
    return asyncSystem.createResolvedFuture(std::move(out));
}

void* StreamRendererResources::prepareInMainThread(Cesium3DTilesSelection::Tile&,
                                                   void* pLoadThreadResult) {
    std::unique_ptr<LoadThreadGlb> glb(static_cast<LoadThreadGlb*>(pLoadThreadResult));
    if (tornDown_.load() || !glb || !glb->ok || r_ == nullptr) return nullptr;
    if (!ensure_gltf_loader(*r_)) return nullptr;

    filament::gltfio::FilamentAsset* asset =
        r_->sharedAssetLoader->createAsset(reinterpret_cast<const uint8_t*>(glb->glbBytes.data()),
                                           static_cast<uint32_t>(glb->glbBytes.size()));
    if (asset == nullptr) return nullptr;
    if (!r_->sharedResourceLoader->loadResources(asset)) {
        r_->sharedAssetLoader->destroyAsset(asset);
        return nullptr;
    }
    asset->releaseSourceData();

    // Terrain: one instance per tile, so each tile can carry its own raster. Content: the shared
    // clay instance unless materials=original.
    filament::MaterialInstance* swap = materialsOriginal_ ? nullptr : r_->buildingMaterial;
    if (kind_ == Kind::Terrain) {
        swap = r_->terrainMaterial->createInstance();
        swap->setParameter("brightness", static_cast<float>(brightness_));
        terrainInstances_[asset] = swap;
    }
    filament::RenderableManager& rm = r_->engine->getRenderableManager();
    const utils::Entity* renderables = asset->getRenderableEntities();
    const size_t renderableCount = asset->getRenderableEntityCount();
    for (size_t i = 0; i < renderableCount; ++i) {
        const auto inst = rm.getInstance(renderables[i]);
        if (!inst.isValid()) continue;
        const size_t primCount = rm.getPrimitiveCount(inst);
        if (swap != nullptr) {
            for (size_t p = 0; p < primCount; ++p) rm.setMaterialInstanceAt(inst, p, swap);
        }
        rm.setCastShadows(inst, true);
        rm.setReceiveShadows(inst, true);
    }

    // Content only: a terrain tile's own glTF materials are unbound, terrain.mat replaced them.
    if (kind_ == Kind::Content && materialsOriginal_ && brightness_ != 1.0) {
        if (filament::gltfio::FilamentInstance* finst = asset->getInstance(); finst != nullptr) {
            filament::MaterialInstance* const* insts = finst->getMaterialInstances();
            const size_t instCount = finst->getMaterialInstanceCount();
            for (size_t i = 0; i < instCount; ++i) {
                filament::MaterialInstance* mi = insts[i];
                if (mi == nullptr || mi->getMaterial() == nullptr ||
                    !mi->getMaterial()->hasParameter("baseColorFactor")) {
                    continue;
                }
                filament::math::float4 base =
                    mi->getParameter<filament::math::float4>("baseColorFactor");
                const float gain = static_cast<float>(brightness_);
                base.r *= gain;
                base.g *= gain;
                base.b *= gain;
                mi->setParameter("baseColorFactor", base);
            }
        }
    }

    filament::TransformManager& tm = r_->engine->getTransformManager();
    const auto tinst = tm.getInstance(asset->getRoot());
    if (tinst.isValid()) {
        tm.setTransform(tinst, to_filament_mat4(ecefToMap_ * glb->transform));
        ensure_terrain_root();
        const auto terrainInst = tm.getInstance(terrainRoot_);
        if (terrainInst.isValid()) tm.setParent(tinst, terrainInst);
    }
    return asset;
}

void StreamRendererResources::free(Cesium3DTilesSelection::Tile&, void* pLoadThreadResult,
                                   void* pMainThreadResult) noexcept {
    delete static_cast<LoadThreadGlb*>(pLoadThreadResult);
    if (pMainThreadResult == nullptr) return;
    auto* asset = static_cast<filament::gltfio::FilamentAsset*>(pMainThreadResult);
    if (tornDown_.load()) {
        return;
    }
    std::lock_guard<std::mutex> lock(freeMutex_);
    pendingFrees_.push_back(asset);
}

namespace {
// Trilinear, clamped: raster tiles meet at tile edges, so no wrap.
const filament::TextureSampler kImagerySampler(
    filament::TextureSampler::MinFilter::LINEAR_MIPMAP_LINEAR,
    filament::TextureSampler::MagFilter::LINEAR, filament::TextureSampler::WrapMode::CLAMP_TO_EDGE);
}

void* StreamRendererResources::prepareRasterInLoadThread(CesiumImage::ImageAsset& image,
                                                         const std::any&) {
    // ponytail: 8-bit RGB or RGBA, what cesium decodes PNG and JPEG to; any other layout attaches
    // nothing. KTX2 or 16-bit imagery needs a conversion here.
    const size_t pixels = static_cast<size_t>(std::max(image.width, 0)) *
                          static_cast<size_t>(std::max(image.height, 0));
    const size_t channels = static_cast<size_t>(image.channels);
    if (pixels == 0 || image.bytesPerChannel != 1 || (channels != 3 && channels != 4) ||
        image.compressedPixelFormat != CesiumImage::GpuCompressedPixelFormat::NONE ||
        image.pixelData.size() < pixels * channels) {
        return nullptr;
    }
    auto* out = new RasterLoadThreadResult{static_cast<uint32_t>(image.width),
                                           static_cast<uint32_t>(image.height),
                                           std::vector<uint8_t>(pixels * 4, 255)};
    const auto* src = reinterpret_cast<const uint8_t*>(image.pixelData.data());
    for (size_t i = 0; i < pixels; ++i) std::memcpy(&out->rgba[i * 4], &src[i * channels], 3);
    return out;
}

void* StreamRendererResources::prepareRasterInMainThread(CesiumRasterOverlays::RasterOverlayTile&,
                                                         void* pLoadThreadResult) {
    std::unique_ptr<RasterLoadThreadResult> pixels(
        static_cast<RasterLoadThreadResult*>(pLoadThreadResult));
    if (!pixels || tornDown_.load() || r_ == nullptr) return nullptr;
    filament::Engine& engine = *r_->engine;
    filament::Texture* texture = filament::Texture::Builder()
                                     .width(pixels->width)
                                     .height(pixels->height)
                                     .levels(0xff)  // clamped to the full chain
                                     .format(filament::Texture::InternalFormat::SRGB8_A8)
                                     .sampler(filament::Texture::Sampler::SAMPLER_2D)
                                     .build(engine);
    auto* heap = new std::vector<uint8_t>(std::move(pixels->rgba));
    texture->setImage(
        engine, 0,
        filament::backend::PixelBufferDescriptor(
            heap->data(), heap->size(),
            filament::backend::PixelBufferDescriptor::PixelDataFormat::RGBA,
            filament::backend::PixelBufferDescriptor::PixelDataType::UBYTE,
            [](void*, size_t, void* user) { delete static_cast<std::vector<uint8_t>*>(user); },
            heap));
    texture->generateMipmaps(engine);
    ++counters_.textures_alive;
    return texture;
}

void StreamRendererResources::freeRaster(const CesiumRasterOverlays::RasterOverlayTile&,
                                         void* pLoadThreadResult,
                                         void* pMainThreadResult) noexcept {
    delete static_cast<RasterLoadThreadResult*>(pLoadThreadResult);
    if (pMainThreadResult == nullptr || tornDown_.load()) return;
    std::lock_guard<std::mutex> lock(freeMutex_);
    pendingTextureFrees_.push_back(static_cast<filament::Texture*>(pMainThreadResult));
}

filament::MaterialInstance* StreamRendererResources::terrain_instance(
    const Cesium3DTilesSelection::Tile& tile) const {
    const Cesium3DTilesSelection::TileRenderContent* content = tile.getContent().getRenderContent();
    if (content == nullptr) return nullptr;
    const auto it = terrainInstances_.find(content->getRenderResources());
    return it != terrainInstances_.end() ? it->second : nullptr;
}

void StreamRendererResources::attachRasterInMainThread(
    const Cesium3DTilesSelection::Tile& tile, int32_t overlayTextureCoordinateID,
    const CesiumRasterOverlays::RasterOverlayTile&, void* pMainThreadRendererResources,
    const glm::dvec2& translation, const glm::dvec2& scale) {
    filament::MaterialInstance* mi = terrain_instance(tile);
    auto* texture = static_cast<filament::Texture*>(pMainThreadRendererResources);
    // ponytail: one overlay per terrain tileset, read from TEXCOORD_0; a second overlay's rasters
    // are ignored. A second sampler and UV set are the upgrade.
    if (mi == nullptr || texture == nullptr || overlayTextureCoordinateID != 0) return;
    mi->setParameter("imagery", texture, kImagerySampler);
    mi->setParameter(
        "uvTransform",
        filament::math::float4{static_cast<float>(translation.x), static_cast<float>(translation.y),
                               static_cast<float>(scale.x), static_cast<float>(scale.y)});
    mi->setParameter("hasImagery", int32_t{1});
    ++counters_.rasters_attached;
}

void StreamRendererResources::detachRasterInMainThread(
    const Cesium3DTilesSelection::Tile& tile, int32_t overlayTextureCoordinateID,
    const CesiumRasterOverlays::RasterOverlayTile&, void* pMainThreadRendererResources) noexcept {
    filament::MaterialInstance* mi = terrain_instance(tile);
    if (mi == nullptr || pMainThreadRendererResources == nullptr ||
        overlayTextureCoordinateID != 0 || r_ == nullptr) {
        return;
    }
    // The placeholder goes back in, so no instance still holds a texture freeRaster may destroy.
    // Filament keeps a mipmapped texture in the instance's mutable-texture map, and binding the
    // 1-level placeholder over it does not erase that entry (MaterialInstance::commit would then
    // panic on the destroyed texture), so the nullptr bind erases it first.
    mi->setParameter("imagery", static_cast<const filament::Texture*>(nullptr), kImagerySampler);
    mi->setParameter("imagery", r_->terrainPlaceholderTexture, kImagerySampler);
    mi->setParameter("hasImagery", int32_t{0});
    ++counters_.rasters_detached;
}

void StreamRendererResources::sync_terrain_color() {
    if (r_ == nullptr) return;
    const detail::Float3& c = r_->active_theme.environment.terrain_color;
    if (terrainColor_ == std::array<float, 3>{c.r, c.g, c.b}) return;
    terrainColor_ = {c.r, c.g, c.b};
    for (const auto& [asset, mi] : terrainInstances_) {
        mi->setParameter("terrainColor", filament::math::float3{c.r, c.g, c.b});
    }
}

std::vector<filament::gltfio::FilamentAsset*> StreamRendererResources::drain_pending_frees() {
    std::vector<filament::gltfio::FilamentAsset*> toFree;
    std::vector<filament::Texture*> textures;
    {
        std::lock_guard<std::mutex> lock(freeMutex_);
        toFree.swap(pendingFrees_);
        textures.swap(pendingTextureFrees_);
    }
    if (r_ == nullptr) return {};
    for (filament::gltfio::FilamentAsset* asset : toFree) {
        r_->scene->removeEntities(asset->getEntities(), asset->getEntityCount());
        auto terrain = terrainInstances_.extract(asset);
        r_->sharedAssetLoader->destroyAsset(asset);
        // After the asset: its renderables held the instance.
        if (!terrain.empty()) r_->engine->destroy(terrain.mapped());
    }
    for (filament::Texture* texture : textures) {
        r_->engine->destroy(texture);
        --counters_.textures_alive;
        ++counters_.textures_freed;
    }
    return toFree;
}

StreamingEnvironmentSource::StreamingEnvironmentSource(
    Cesium3DTilesSelection::TilesetExternals externals, int64_t asset_id,
    std::string ion_access_token, std::string root_tileset_uri, std::string fallback_baked_dir,
    GeoAnchor anchor, std::shared_ptr<CountingAssetAccessor> counting_accessor,
    bool materials_original, bool follow_terrain, double ground_bias_m, bool replaces_ground,
    double max_tilt_deg, double brightness, std::optional<double> radius_override_m,
    CustomTileset custom_tileset, StreamRendererResources::Kind resources_kind)
    : asyncSystem_(externals.asyncSystem),
      anchor_(anchor),
      ecefToMap_(compute_ecef_to_map(anchor)),
      mapToEcef_(glm::inverse(ecefToMap_)),
      fallbackBakedDir_(std::move(fallback_baked_dir)),
      materialsOriginal_(materials_original),
      followTerrain_(follow_terrain),
      groundBiasM_(ground_bias_m),
      replacesGround_(replaces_ground),
      maxTiltRad_(max_tilt_deg * M_PI / 180.0),
      brightness_(brightness),
      tileRadiusOverrideM_(radius_override_m),
      areaSelection_(custom_tileset.area_selection),
      countingAccessor_(std::move(counting_accessor)) {
    static std::once_flag registerContentTypesOnce;
    std::call_once(registerContentTypesOnce,
                   [] { Cesium3DTilesContent::registerAllTileContentTypes(); });

    renderResources_ = std::make_shared<StreamRendererResources>(
        ecefToMap_, anchor_.origin_height_m, materialsOriginal_, brightness_, resources_kind);
    externals.pPrepareRendererResources = renderResources_;

    Cesium3DTilesSelection::TilesetOptions options;
    options.maximumScreenSpaceError = kStreamMaxSseErr;
    if (areaSelection_) {
        // Otherwise every culled sibling still loads, at preload priority.
        options.preloadSiblings = false;
    }
    if (custom_tileset.loader) {
        tileset_ = std::make_unique<Cesium3DTilesSelection::Tileset>(
            externals, std::move(custom_tileset.loader), std::move(custom_tileset.root), options);
    } else if (asset_id > 0 && !ion_access_token.empty()) {
        tileset_ = std::make_unique<Cesium3DTilesSelection::Tileset>(externals, asset_id,
                                                                     ion_access_token, options);
    } else {
        tileset_ =
            std::make_unique<Cesium3DTilesSelection::Tileset>(externals, root_tileset_uri, options);
    }
    viewGroup_ = &tileset_->getDefaultViewGroup();
}

StreamingEnvironmentSource::~StreamingEnvironmentSource() = default;

void StreamingEnvironmentSource::update(VisualRenderer& r, Vec3 ego_map_pos) {
    if (fallenBack_) {
        if (fallbackSource_) fallbackSource_->update(r, ego_map_pos);
        return;
    }
    if (countingAccessor_ &&
        countingAccessor_->consecutive_failures() >= kNetworkLossConsecutiveFailures) {
        fall_back(r);
        if (fallbackSource_) fallbackSource_->update(r, ego_map_pos);
        return;
    }
    renderResources_->set_renderer(&r);
    renderResources_->sync_terrain_color();
    for (filament::gltfio::FilamentAsset* freed : renderResources_->drain_pending_frees()) {
        inScene_.erase(freed);
    }
    if (terrainResources_) {
        terrainResources_->set_renderer(&r);
        terrainResources_->sync_terrain_color();
        for (filament::gltfio::FilamentAsset* freed : terrainResources_->drain_pending_frees()) {
            terrainInScene_.erase(freed);
        }
    }
    synthesize_view_and_pump(r, ego_map_pos);
}

void StreamingEnvironmentSource::fall_back(VisualRenderer& r) {
    teardown(r);
    fallenBack_ = true;
    if (!fallbackBakedDir_.empty()) {
        fallbackSource_ = open_baked_environment_source(fallbackBakedDir_, anchor_);
        if (fallbackSource_) fallbackSource_->set_visible(r, visible_);
    }
}

void StreamingEnvironmentSource::maybe_trigger_terrain_sample(Vec3 ego_map_pos,
                                                              double heading_rad) {
    if (!terrainState_) terrainState_ = std::make_shared<TerrainFollowState>();
    terrainState_->anchor_height_m = anchor_.origin_height_m;
    terrainState_->ground_bias_m = groundBiasM_;
    terrainState_->max_tilt_rad = maxTiltRad_;
    if (terrainState_->in_flight.load()) return;

    const double dx = ego_map_pos.x - terrainState_->sampled_x;
    const double dy = ego_map_pos.y - terrainState_->sampled_y;
    const double movedM = std::sqrt(dx * dx + dy * dy);
    const double elapsedS =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - terrainState_->sampled_at)
            .count();
    if (terrainState_->ever_sampled && movedM < kTerrainSampleMoveThresholdM &&
        elapsedS < kTerrainSampleIntervalS) {
        return;
    }

    static constexpr double kAlongTrackOffsetsM[5] = {-20.0, -10.0, 0.0, 10.0, 20.0};
    const double fwdX = std::cos(heading_rad), fwdY = std::sin(heading_rad);
    std::vector<double> validS;
    std::vector<CesiumGeospatial::Cartographic> cartos;
    validS.reserve(5);
    cartos.reserve(5);
    for (const double s : kAlongTrackOffsetsM) {
        const glm::dvec4 sampleEcef4 =
            mapToEcef_ * glm::dvec4(ego_map_pos.x + s * fwdX, ego_map_pos.y + s * fwdY, 0.0, 1.0);
        const std::optional<CesiumGeospatial::Cartographic> carto =
            CesiumGeospatial::Ellipsoid::WGS84.cartesianToCartographic(glm::dvec3(sampleEcef4));
        if (!carto.has_value()) continue;
        validS.push_back(s);
        cartos.push_back(*carto);
    }
    if (cartos.empty()) return;

    terrainState_->sampled_x = ego_map_pos.x;
    terrainState_->sampled_y = ego_map_pos.y;
    terrainState_->sampled_at = std::chrono::steady_clock::now();
    terrainState_->ever_sampled = true;
    terrainState_->in_flight.store(true);

    std::shared_ptr<TerrainFollowState> state = terrainState_;
    const double pivotX = ego_map_pos.x;
    const double pivotY = ego_map_pos.y;
    const double pivotHeadingRad = heading_rad;
    // With a terrain tileset, the ground under the ego is the terrain's.
    Cesium3DTilesSelection::Tileset& sampled = has_terrain() ? *terrainTileset_ : *tileset_;
    sampled.sampleHeightMostDetailed(cartos)
        .thenInMainThread([state, validS, pivotX, pivotY,
                           pivotHeadingRad](Cesium3DTilesSelection::SampleHeightResult&& res) {
            std::vector<double> hitS, hitH;
            const size_t n = std::min(res.sampleSuccess.size(), res.positions.size());
            for (size_t i = 0; i < n && i < validS.size(); ++i) {
                if (res.sampleSuccess[i]) {
                    hitS.push_back(validS[i]);
                    hitH.push_back(res.positions[i].height);
                }
            }
            if (!hitS.empty()) {
                double slope = 0.0, intercept = 0.0, rms = 0.0;
                terrain_plane_fit(hitS.data(), hitH.data(), static_cast<int>(hitS.size()), &slope,
                                  &intercept, &rms);
                state->fit_slope = slope;
                state->fit_intercept = intercept;
                state->fit_rms_m = rms;
                state->fit_hit_count = static_cast<int>(hitS.size());
                state->fit_pivot_x = pivotX;
                state->fit_pivot_y = pivotY;
                state->fit_pivot_heading_rad = pivotHeadingRad;
                state->ground_hit.store(true);
                const double clampedThetaRad = terrain_tilt_target(slope, state->max_tilt_rad);
                const double offsetTarget =
                    std::clamp(-(intercept - state->anchor_height_m) - state->ground_bias_m,
                               -kTerrainOffsetClampM, kTerrainOffsetClampM);
                make_redacting_logger()->info(
                    "terrain fit under ego: {}/5 hits, grade {:.2f}%, residual RMS {:.3f} m -> "
                    "tilt {:.2f} deg, ground offset {:.2f} m",
                    hitS.size(), slope * 100.0, rms, clampedThetaRad * 180.0 / M_PI, offsetTarget);
            } else {
                make_redacting_logger()->warn(
                    "terrain sample under ego: no geometry hit (offset/tilt unchanged)");
            }
            state->in_flight.store(false);
        })
        .catchInMainThread([state](std::exception&&) { state->in_flight.store(false); });
}

namespace {
double terrain_target_offset_z(double sampled_height_m, double anchor_height_m,
                               double ground_bias_m) {
    return std::clamp(-(sampled_height_m - anchor_height_m) - ground_bias_m, -kTerrainOffsetClampM,
                      kTerrainOffsetClampM);
}
double terrain_smooth_toward(double current, double target, float deltaSeconds) {
    if (deltaSeconds <= 0.0f) return current;
    const double alpha =
        1.0 - std::exp(-static_cast<double>(deltaSeconds) / kTerrainSmoothTimeConstantS);
    return current + (target - current) * alpha;
}
}

void StreamingEnvironmentSource::update_terrain_transform(float deltaSeconds) {
    if (!terrainState_ || !terrainState_->fit_intercept.has_value()) return;

    const double slope = terrainState_->fit_slope.value_or(0.0);
    const double intercept = *terrainState_->fit_intercept;
    const double zTarget =
        terrain_target_offset_z(intercept, anchor_.origin_height_m, groundBiasM_);
    const double thetaTarget = terrain_tilt_target(slope, maxTiltRad_);

    if (!groundOffsetSnapped_) {
        groundOffsetZ_ = zTarget;
        terrainTiltRad_ = thetaTarget;
        groundOffsetSnapped_ = true;
    } else {
        groundOffsetZ_ = terrain_smooth_toward(groundOffsetZ_, zTarget, deltaSeconds);
        terrainTiltRad_ = terrain_smooth_toward(terrainTiltRad_, thetaTarget, deltaSeconds);
    }

    if (std::abs(groundOffsetZ_ - lastAppliedGroundOffsetZ_) > 1e-3 ||
        std::abs(terrainTiltRad_ - lastAppliedTiltRad_) > 1e-3) {
        for (StreamRendererResources* res : {renderResources_.get(), terrainResources_.get()}) {
            if (res == nullptr) continue;
            res->set_terrain_transform(groundOffsetZ_, terrainTiltRad_, terrainState_->fit_pivot_x,
                                       terrainState_->fit_pivot_y,
                                       terrainState_->fit_pivot_heading_rad);
        }
        lastAppliedGroundOffsetZ_ = groundOffsetZ_;
        lastAppliedTiltRad_ = terrainTiltRad_;
    }
}

double StreamingEnvironmentSource::first_tracked_tile_world_z(VisualRenderer& r) const {
    if (inScene_.empty() || r.engine == nullptr) return std::nan("");
    auto* asset =
        static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(inScene_.begin()->first));
    if (asset == nullptr) return std::nan("");
    filament::TransformManager& tm = r.engine->getTransformManager();
    const auto inst = tm.getInstance(asset->getRoot());
    if (!inst.isValid()) return std::nan("");
    return static_cast<double>(tm.getWorldTransform(inst)[3].z);
}

namespace {
std::optional<Cesium3DTilesSelection::ViewState> camera_view_state(const VisualRenderer& r,
                                                                   const glm::dmat4& mapToEcef) {
    if (r.camera == nullptr || r.width == 0 || r.height == 0) return std::nullopt;

    const filament::math::double3 pos = r.camera->getPosition();
    const filament::math::float3 fwd = r.camera->getForwardVector();
    const filament::math::float3 up = r.camera->getUpVector();
    if (pos.x == 0.0 && pos.y == 0.0 && pos.z == 0.0 && fwd.x == 0.0f && fwd.y == 0.0f &&
        fwd.z == -1.0f && up.x == 0.0f && up.y == 1.0f && up.z == 0.0f) {
        return std::nullopt;
    }

    const glm::dvec4 posEcef4 = mapToEcef * glm::dvec4(pos.x, pos.y, pos.z, 1.0);
    const glm::dvec3 dirEcef =
        glm::normalize(glm::dvec3(mapToEcef * glm::dvec4(fwd.x, fwd.y, fwd.z, 0.0)));
    const glm::dvec3 upEcef =
        glm::normalize(glm::dvec3(mapToEcef * glm::dvec4(up.x, up.y, up.z, 0.0)));

    const double aspect = static_cast<double>(r.width) / static_cast<double>(r.height);
    const double vfovRad =
        static_cast<double>(r.camera->getFieldOfViewInDegrees(filament::Camera::Fov::VERTICAL)) *
        (M_PI / 180.0);
    const double hfovRad = 2.0 * std::atan(std::tan(vfovRad * 0.5) * aspect);

    return Cesium3DTilesSelection::ViewState(
        glm::dvec3(posEcef4), dirEcef, upEcef,
        glm::dvec2(static_cast<double>(r.width), static_cast<double>(r.height)), hfovRad, vfovRad);
}
}

void StreamingEnvironmentSource::synthesize_view_and_pump(VisualRenderer& r, Vec3 ego_map_pos) {
    using Cesium3DTilesSelection::ViewState;
    using Cesium3DTilesSelection::ViewUpdateResult;

    const glm::dvec4 eyeEcef4 = mapToEcef_ * glm::dvec4(ego_map_pos.x, ego_map_pos.y,
                                                        ego_map_pos.z + kStreamViewHeightM, 1.0);
    const glm::dvec3 eyeEcef(eyeEcef4);
    const glm::dvec3 downEcef =
        glm::normalize(glm::dvec3(mapToEcef_ * glm::dvec4(0.0, 0.0, -1.0, 0.0)));
    const glm::dvec3 northEcef =
        glm::normalize(glm::dvec3(mapToEcef_ * glm::dvec4(0.0, 1.0, 0.0, 0.0)));

    const ViewState view(eyeEcef, downEcef, northEcef,
                         glm::dvec2(kStreamViewportPx, kStreamViewportPx), kStreamViewFovRad,
                         kStreamViewFovRad);

    const auto now = std::chrono::steady_clock::now();
    float deltaSeconds = 0.0f;
    if (lastUpdate_.has_value()) {
        deltaSeconds = std::chrono::duration<float>(now - *lastUpdate_).count();
    }
    lastUpdate_ = now;

    // A terrain tileset that replaces the ground is sampled even without follow_terrain: the sample
    // under the ego is what lets it take the clay plane's place.
    if (followTerrain_ || (has_terrain() && replacesGround_)) {
        maybe_trigger_terrain_sample(ego_map_pos, r.scene_buffer.active().ego.heading_rad);
    }
    if (followTerrain_) update_terrain_transform(deltaSeconds);

    const double radiusM = tileRadiusOverrideM_.value_or(
        static_cast<double>(r.active_theme.environment.tile_radius_m));
    std::vector<ViewState> views;
    if (areaSelection_) {
        // The only view: next to a screen-space view, cesium would select by screen-space error.
        const glm::dvec3 egoEcef(mapToEcef_ *
                                 glm::dvec4(ego_map_pos.x, ego_map_pos.y, ego_map_pos.z, 1.0));
        views.emplace_back(Cesium3DTilesSelection::BoundingVolume(
                               CesiumGeometry::BoundingSphere(egoEcef, radiusM)),
                           1.0);
    } else {
        const std::optional<ViewState> cameraView = camera_view_state(r, mapToEcef_);
        views = cameraView.has_value() ? std::vector<ViewState>{view, *cameraView}
                                       : std::vector<ViewState>{view};
    }
    lastViewFrustumCount_ = views.size();
    const ViewUpdateResult& result = tileset_->updateViewGroup(*viewGroup_, views, deltaSeconds);
    tileset_->loadTiles();
    // The terrain tileset selects from the same views; one dispatch serves both (one AsyncSystem).
    const ViewUpdateResult* terrainResult = nullptr;
    if (terrainTileset_) {
        terrainResult = &terrainTileset_->updateViewGroup(terrainTileset_->getDefaultViewGroup(),
                                                          views, deltaSeconds);
        terrainTileset_->loadTiles();
    }
    asyncSystem_.dispatchMainThreadTasks();

    sync_scene(r, result, ego_map_pos, radiusM, inScene_);
    if (terrainResult != nullptr) {
        sync_scene(r, *terrainResult, ego_map_pos, radiusM, terrainInScene_);
    }
}

void StreamingEnvironmentSource::sync_scene(VisualRenderer& r,
                                            const Cesium3DTilesSelection::ViewUpdateResult& result,
                                            Vec3 ego_map_pos, double radius_m,
                                            std::unordered_map<const void*, bool>& in_scene) {
    std::unordered_map<const void*, bool> stillPresent;
    for (const auto& tilePtr : result.tilesToRenderThisFrame) {
        const Cesium3DTilesSelection::Tile* tile = tilePtr.get();
        if (tile == nullptr || !tile->isRenderContent()) continue;
        // Until Done, the render resources are still the worker's LoadThreadGlb, not an asset.
        if (tile->getState() != Cesium3DTilesSelection::TileLoadState::Done) continue;
        const auto* renderContent = tile->getContent().getRenderContent();
        if (renderContent == nullptr) continue;
        void* res = renderContent->getRenderResources();
        if (res == nullptr) continue;
        const CesiumGeometry::OrientedBoundingBox obb =
            Cesium3DTilesSelection::getOrientedBoundingBoxFromBoundingVolume(
                tile->getBoundingVolume(), CesiumGeospatial::Ellipsoid::WGS84);
        const glm::dvec4 centerMap4 = ecefToMap_ * glm::dvec4(obb.getCenter(), 1.0);
        const glm::dmat3& halfAxes = obb.getHalfAxes();
        const double extentM =
            glm::length(halfAxes[0]) + glm::length(halfAxes[1]) + glm::length(halfAxes[2]);
        const double dx = centerMap4.x - ego_map_pos.x;
        const double dy = centerMap4.y - ego_map_pos.y;
        const bool wasInScene = in_scene.find(res) != in_scene.end();
        const double limitM = wasInScene ? radius_m * kTileRadiusUnloadRatio : radius_m;
        if (std::hypot(dx, dy) - extentM > limitM) continue;
        auto* asset = static_cast<filament::gltfio::FilamentAsset*>(res);
        stillPresent[res] = true;
        if (visible_ && in_scene.find(res) == in_scene.end()) {
            r.scene->addEntities(asset->getEntities(), asset->getEntityCount());
        }
    }
    for (auto it = in_scene.begin(); it != in_scene.end();) {
        if (stillPresent.find(it->first) == stillPresent.end()) {
            auto* asset =
                static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(it->first));
            if (visible_) {
                r.scene->removeEntities(asset->getEntities(), asset->getEntityCount());
            }
            it = in_scene.erase(it);
        } else {
            ++it;
        }
    }
    in_scene = std::move(stillPresent);
}

void StreamingEnvironmentSource::teardown(VisualRenderer& r) {
    if (fallbackSource_) {
        fallbackSource_->teardown(r);
        fallbackSource_.reset();
    }
    if (tornDown_) return;
    // The terrain goes first. Destroying a tileset unloads every tile, and cesium detaches a tile's
    // raster overlay tiles before it frees the tile (TilesetContentManager.cpp:1416-1420), so the
    // overlays are detached before either tileset's tiles are freed.
    std::vector<StreamRendererResources*> resources{renderResources_.get()};
    if (terrainResources_) resources.insert(resources.begin(), terrainResources_.get());
    for (StreamRendererResources* res : resources) res->set_renderer(&r);

    for (std::unordered_map<const void*, bool>* tracked : {&terrainInScene_, &inScene_}) {
        for (auto& [res, _] : *tracked) {
            auto* asset = static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(res));
            if (visible_) {
                r.scene->removeEntities(asset->getEntities(), asset->getEntityCount());
            }
        }
        tracked->clear();
    }
    for (StreamRendererResources* res : resources) res->drain_pending_frees();

    std::vector<CesiumAsync::SharedFuture<void>> destructionComplete;
    if (terrainTileset_) {
        destructionComplete.push_back(terrainTileset_->getAsyncDestructionCompleteEvent());
        terrainTileset_.reset();
    }
    destructionComplete.push_back(tileset_->getAsyncDestructionCompleteEvent());
    tileset_.reset();
    for (StreamRendererResources* res : resources) res->drain_pending_frees();

    const auto allDestroyed = [&destructionComplete] {
        return std::all_of(destructionComplete.begin(), destructionComplete.end(),
                           [](const CesiumAsync::SharedFuture<void>& f) { return f.isReady(); });
    };
    int pumps = 0;
    for (; pumps < kTeardownPumpBound && !allDestroyed(); ++pumps) {
        asyncSystem_.dispatchMainThreadTasks();
    }
    if (pumps >= kTeardownPumpBound && !allDestroyed()) {
        ++leakedOnTeardownBound_;
    }
    // Drains the cancelled continuation a height sample in flight at teardown leaves queued: left
    // to ~AsyncSystem it segfaults in QueuedScheduler::schedule (the ion fixture-network terrain
    // test and the MapboxBuildings failing-DEM tests crash without this line).
    asyncSystem_.dispatchMainThreadTasks();

    for (StreamRendererResources* res : resources) {
        res->drain_pending_frees();
        res->destroy_terrain_root();
        res->note_torn_down();
    }
    tornDown_ = true;
}

void StreamingEnvironmentSource::set_visible(VisualRenderer& r, bool visible) {
    if (fallenBack_) {
        visible_ = visible;
        if (fallbackSource_) fallbackSource_->set_visible(r, visible);
        return;
    }
    if (visible == visible_) return;
    visible_ = visible;
    for (std::unordered_map<const void*, bool>* tracked : {&inScene_, &terrainInScene_}) {
        for (auto& [res, _] : *tracked) {
            auto* asset = static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(res));
            if (visible_) {
                r.scene->addEntities(asset->getEntities(), asset->getEntityCount());
            } else {
                r.scene->removeEntities(asset->getEntities(), asset->getEntityCount());
            }
        }
    }
}

size_t StreamingEnvironmentSource::loaded_count() const {
    if (fallenBack_) return fallbackSource_ ? fallbackSource_->loaded_count() : 0;
    return inScene_.size();
}

size_t StreamingEnvironmentSource::scene_membership_count(VisualRenderer& r) const {
    if (fallenBack_) return fallbackSource_ ? fallbackSource_->scene_membership_count(r) : 0;
    return count_in_scene(r, inScene_);
}

EnvironmentSourceState StreamingEnvironmentSource::state() const {
    return fallenBack_ ? EnvironmentSourceState::STREAMING_FALLBACK
                       : EnvironmentSourceState::STREAMING;
}

bool StreamingEnvironmentSource::has_disk_cache() const {
    // build_externals hands the tileset TokenBypassAssetAccessor exactly when it opened a cache.
    return tileset_ != nullptr && dynamic_cast<const TokenBypassAssetAccessor*>(
                                      tileset_->getExternals().pAssetAccessor.get()) != nullptr;
}

bool StreamingEnvironmentSource::first_primitive_is_building_material(VisualRenderer& r) const {
    if (inScene_.empty()) return false;
    auto* asset =
        static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(inScene_.begin()->first));
    const size_t renderableCount = asset->getRenderableEntityCount();
    if (renderableCount == 0) return false;
    const utils::Entity* renderables = asset->getRenderableEntities();
    filament::RenderableManager& rm = r.engine->getRenderableManager();
    const auto inst = rm.getInstance(renderables[0]);
    if (!inst.isValid() || rm.getPrimitiveCount(inst) == 0) return false;
    return rm.getMaterialInstanceAt(inst, 0) == r.buildingMaterial;
}

}

namespace overlume {

namespace {

// Serves open_streaming_environment_source (curl, CESIUM_ION_TOKEN) and install_fixture_ion_source
// (a fixture network, a given token). A null `base` means curl. The spec is parsed, and a refusal
// logged, before the token is looked at: Task 4's ParseIonSpecRefusalLogsOneLine runs without one.
std::unique_ptr<StreamingEnvironmentSource> open_ion_source(
    const std::string& ion_spec, GeoAnchor anchor,
    std::shared_ptr<CesiumAsync::IAssetAccessor> base, const std::string& token) {
    std::string why;
    const std::optional<IonSpec> spec = parse_ion_spec(ion_spec, &why);
    if (!spec) {
        // One line (spec section 5). The spec holds no token: the caller passes it.
        make_redacting_logger()->error("ion://{} refused: {}", ion_spec, why);
        return nullptr;
    }
    // A spec that parses opens nothing without a token, silently.
    if (token.empty()) return nullptr;

    const std::string cacheDir = spec->cache_dir.empty() ? default_cache_dir() : spec->cache_dir;

    if (!base) base = std::make_shared<CesiumCurl::CurlAssetAccessor>();
    CesiumAsync::AsyncSystem asyncSystem(std::make_shared<SimpleTaskProcessor>());
    std::shared_ptr<CountingAssetAccessor> counting;
    Cesium3DTilesSelection::TilesetExternals externals =
        build_externals(std::move(base), asyncSystem, cacheDir, spec->max_cache_items, &counting);

    auto source = std::make_unique<StreamingEnvironmentSource>(
        externals, spec->asset_id, token, std::string(), spec->fallback_dir, anchor,
        std::move(counting), spec->materials_original, spec->follow_terrain, spec->ground_bias_m,
        spec->replaces_ground, spec->max_tilt_deg, spec->brightness, spec->radius_m);
    if (!spec->terrain_asset) return source;

    // The terrain shares the content tileset's accessor (so its failures count toward network
    // loss), task processor, logger and credit system; only its renderer resources differ.
    auto terrainResources = std::make_shared<StreamRendererResources>(
        compute_ecef_to_map(anchor), anchor.origin_height_m, false, spec->brightness,
        StreamRendererResources::Kind::Terrain);
    Cesium3DTilesSelection::TilesetExternals terrainExternals = externals;
    terrainExternals.pPrepareRendererResources = terrainResources;
    Cesium3DTilesSelection::TilesetOptions options;
    options.maximumScreenSpaceError = kStreamMaxSseErr;
    options.loadErrorCallback = [](const Cesium3DTilesSelection::TilesetLoadFailureDetails& d) {
        make_redacting_logger()->warn(
            "environment source: terrain tileset did not open (HTTP {}) -- continuing without "
            "terrain",
            d.statusCode);
    };
    auto terrain = std::make_unique<Cesium3DTilesSelection::Tileset>(
        terrainExternals, *spec->terrain_asset, token, options);
    if (spec->imagery_asset) {
        const CesiumUtility::IntrusivePointer<const CesiumRasterOverlays::RasterOverlay> imagery =
            new CesiumRasterOverlays::IonRasterOverlay("ion-imagery", *spec->imagery_asset, token);
        terrain->getOverlays().add(imagery);
    }
    source->attach_terrain(std::move(terrain), std::move(terrainResources));
    return source;
}

}

std::unique_ptr<EnvironmentSource> open_streaming_environment_source(const std::string& ion_spec,
                                                                     GeoAnchor anchor) {
    const char* token = std::getenv("CESIUM_ION_TOKEN");
    return open_ion_source(ion_spec, anchor, nullptr, token != nullptr ? token : "");
}

uint32_t environment_source_credits(VisualRenderer* r, char* out, uint32_t cap) {
    auto* stream = r != nullptr
                       ? dynamic_cast<StreamingEnvironmentSource*>(r->environmentSource.get())
                       : nullptr;
    const std::string text = stream != nullptr ? stream->credits() : std::string();
    if (out != nullptr && cap > 0) {
        const size_t n = std::min<size_t>(text.size(), cap - 1);
        std::memcpy(out, text.data(), n);
        out[n] = '\0';
    }
    return static_cast<uint32_t>(text.size());
}

}

namespace overlume::testing {

struct FixtureStreamHandle {
    std::shared_ptr<std::atomic<bool>> killed;
    std::shared_ptr<std::atomic<bool>> transport_down;
};

namespace {

#ifdef _WIN32
const int kProcessId = _getpid();
#else
const int kProcessId = ::getpid();
#endif

std::string test_cache_dir() {
    static const std::string dir = [] {
        const std::string d = (std::filesystem::temp_directory_path() /
                               ("overlume-stream-test-cache-" + std::to_string(kProcessId)))
                                  .string();
        std::error_code ec;
        std::filesystem::remove_all(d, ec);
        return d;
    }();
    return dir;
}

std::unique_ptr<overlume::EnvironmentSource> make_fixture_source(
    const char* fixture_dir, const char* fallback_baked_dir, overlume::GeoAnchor anchor,
    std::shared_ptr<std::atomic<bool>> killed, bool materials_original, bool follow_terrain = false,
    std::optional<double> radius_override_m = std::nullopt,
    std::shared_ptr<std::atomic<bool>> transport_down = nullptr) {
    if (fixture_dir == nullptr) return nullptr;
    auto fileAccessor = std::make_shared<overlume::FileFixtureAssetAccessor>(
        std::move(killed), std::move(transport_down));
    CesiumAsync::AsyncSystem asyncSystem(std::make_shared<overlume::SimpleTaskProcessor>());
    const std::string cacheDir = test_cache_dir();
    std::shared_ptr<overlume::CountingAssetAccessor> counting;
    Cesium3DTilesSelection::TilesetExternals externals = overlume::build_externals(
        fileAccessor, asyncSystem, cacheDir, overlume::kDefaultMaxCacheItems, &counting);
    const std::string tilesetUri = std::string("file://") + fixture_dir + "/tileset.json";
    return std::make_unique<overlume::StreamingEnvironmentSource>(
        externals, 0, std::string(), tilesetUri,
        fallback_baked_dir ? std::string(fallback_baked_dir) : std::string(), anchor,
        std::move(counting), materials_original, follow_terrain, 0.0, true, 2.0, 1.0,
        radius_override_m);
}

}

bool install_fixture_streaming_source(overlume::VisualRenderer* r, const char* fixture_dir,
                                      overlume::GeoAnchor anchor, bool materials_original,
                                      bool follow_terrain,
                                      std::optional<double> radius_override_m) {
    if (r == nullptr) return false;
    if (r->environmentSource) r->environmentSource->teardown(*r);
    auto source = make_fixture_source(fixture_dir, nullptr, anchor, nullptr, materials_original,
                                      follow_terrain, radius_override_m);
    if (!source) {
        r->environmentSource.reset();
        return false;
    }
    r->environmentSource = std::move(source);
    return true;
}

FixtureStreamHandle* install_fixture_streaming_source_with_fallback(overlume::VisualRenderer* r,
                                                                    const char* fixture_dir,
                                                                    const char* fallback_baked_dir,
                                                                    overlume::GeoAnchor anchor,
                                                                    bool follow_terrain) {
    if (r == nullptr) return nullptr;
    if (r->environmentSource) r->environmentSource->teardown(*r);
    auto killed = std::make_shared<std::atomic<bool>>(false);
    auto transportDown = std::make_shared<std::atomic<bool>>(false);
    auto source = make_fixture_source(fixture_dir, fallback_baked_dir, anchor, killed, false,
                                      follow_terrain, std::nullopt, transportDown);
    if (!source) {
        r->environmentSource.reset();
        return nullptr;
    }
    r->environmentSource = std::move(source);
    return new FixtureStreamHandle{std::move(killed), std::move(transportDown)};
}

void kill_fixture_network(FixtureStreamHandle* handle) {
    if (handle && handle->killed) handle->killed->store(true);
}

void revive_fixture_network(FixtureStreamHandle* handle) {
    if (handle && handle->killed) handle->killed->store(false);
}

void break_fixture_network_transport(FixtureStreamHandle* handle) {
    if (handle && handle->transport_down) handle->transport_down->store(true);
}

namespace {

constexpr uint32_t kGridCellsPerSide = 4;
constexpr double kGridCellM = 1000.0;

struct GridRequestLog {
    std::mutex mutex;
    std::vector<std::pair<int, int>> leaves;
};

// One triangle in the tile frame, so every leaf becomes a real asset in the scene.
CesiumGltf::Model one_triangle_model() {
    static constexpr float kPositions[9] = {-5, -5, 0, 5, -5, 0, 0, 5, 10};
    static constexpr uint16_t kIndices[3] = {0, 1, 2};

    CesiumGltf::Model model;
    model.asset.version = "2.0";
    model.buffers.resize(1);
    model.bufferViews.resize(2);
    model.accessors.resize(2);
    model.meshes.resize(1);
    model.nodes.resize(1);
    model.scenes.resize(1);

    std::vector<std::byte>& bytes = model.buffers[0].cesium.data;
    bytes.resize(sizeof(kPositions) + sizeof(kIndices));
    std::memcpy(bytes.data(), kPositions, sizeof(kPositions));
    std::memcpy(bytes.data() + sizeof(kPositions), kIndices, sizeof(kIndices));
    model.buffers[0].byteLength = static_cast<int64_t>(bytes.size());

    model.bufferViews[0].buffer = 0;
    model.bufferViews[0].byteLength = sizeof(kPositions);
    model.bufferViews[1].buffer = 0;
    model.bufferViews[1].byteOffset = sizeof(kPositions);
    model.bufferViews[1].byteLength = sizeof(kIndices);

    CesiumGltf::Accessor& position = model.accessors[0];
    position.bufferView = 0;
    position.count = 3;
    position.componentType = CesiumGltf::Accessor::ComponentType::FLOAT;
    position.type = CesiumGltf::Accessor::Type::VEC3;
    position.min = {-5.0, -5.0, 0.0};
    position.max = {5.0, 5.0, 10.0};
    CesiumGltf::Accessor& index = model.accessors[1];
    index.bufferView = 1;
    index.count = 3;
    index.componentType = CesiumGltf::Accessor::ComponentType::UNSIGNED_SHORT;
    index.type = CesiumGltf::Accessor::Type::SCALAR;

    CesiumGltf::MeshPrimitive& primitive = model.meshes[0].primitives.emplace_back();
    primitive.attributes["POSITION"] = 0;
    primitive.indices = 1;
    model.nodes[0].mesh = 0;
    model.scenes[0].nodes.push_back(0);
    model.scene = 0;
    return model;
}

class GridTestLoader final : public Cesium3DTilesSelection::TilesetContentLoader {
public:
    explicit GridTestLoader(std::shared_ptr<GridRequestLog> log) : log_(std::move(log)) {}

    // Only leaves get here: the root holds empty content under a non-loadable ID.
    CesiumAsync::Future<Cesium3DTilesSelection::TileLoadResult> loadTileContent(
        const Cesium3DTilesSelection::TileLoadInput& input) override {
        const auto& id = std::get<CesiumGeometry::QuadtreeTileID>(input.tile.getTileID());
        {
            std::lock_guard<std::mutex> lock(log_->mutex);
            log_->leaves.emplace_back(static_cast<int>(id.x), static_cast<int>(id.y));
        }
        return input.asyncSystem.createResolvedFuture(Cesium3DTilesSelection::TileLoadResult{
            one_triangle_model(),
            CesiumGeometry::Axis::Z,
            std::nullopt,
            std::nullopt,
            std::nullopt,
            input.pAssetAccessor,
            nullptr,
            {},
            Cesium3DTilesSelection::TileLoadResultState::Success,
            input.ellipsoid});
    }

    Cesium3DTilesSelection::TileChildrenResult createTileChildren(
        const Cesium3DTilesSelection::Tile&, const CesiumGeospatial::Ellipsoid&) override {
        return {{}, Cesium3DTilesSelection::TileLoadResultState::Failed};
    }

private:
    std::shared_ptr<GridRequestLog> log_;
};

// The root covers the whole square and refines unconditionally into the 16 leaves. They are built
// here, as EllipsoidTilesetLoader builds its root's children, so the tree is complete before
// cesium's first selection. Leaf (c, r) is QuadtreeTileID(2, c, r): column c west to east, row r
// north to south, each with its own east-north-up frame at its centre.
std::unique_ptr<Cesium3DTilesSelection::Tile> make_grid_root(GridTestLoader* loader,
                                                             const overlume::GeoAnchor& anchor) {
    using CesiumGeospatial::Cartographic;
    using CesiumGeospatial::Ellipsoid;
    using CesiumGeospatial::GlobeTransforms;
    const glm::dmat4 anchorEnu = GlobeTransforms::eastNorthUpToFixedFrame(
        Ellipsoid::WGS84.cartographicToCartesian(Cartographic::fromDegrees(
            anchor.origin_lon_deg, anchor.origin_lat_deg, anchor.origin_height_m)));
    // The point east_m, north_m of the anchor on its tangent plane, as longitude and latitude.
    const auto lonLat = [&](double east_m, double north_m) {
        return *Ellipsoid::WGS84.cartesianToCartographic(
            glm::dvec3(anchorEnu * glm::dvec4(east_m, north_m, 0.0, 1.0)));
    };
    const auto region = [&](double west_m, double south_m, double east_m, double north_m) {
        const Cartographic sw = lonLat(west_m, south_m);
        const Cartographic ne = lonLat(east_m, north_m);
        return CesiumGeospatial::BoundingRegion(
            CesiumGeospatial::GlobeRectangle(sw.longitude, sw.latitude, ne.longitude, ne.latitude),
            anchor.origin_height_m - 10.0, anchor.origin_height_m + 100.0);
    };

    const double half = 0.5 * kGridCellsPerSide * kGridCellM;
    auto root = std::make_unique<Cesium3DTilesSelection::Tile>(
        loader, Cesium3DTilesSelection::TileID(), Cesium3DTilesSelection::TileEmptyContent{});
    root->setUnconditionallyRefine();
    root->setBoundingVolume(region(-half, -half, half, half));

    std::vector<Cesium3DTilesSelection::Tile> leaves;
    leaves.reserve(kGridCellsPerSide * kGridCellsPerSide);
    for (uint32_t c = 0; c < kGridCellsPerSide; ++c) {
        for (uint32_t r = 0; r < kGridCellsPerSide; ++r) {
            const double west = -half + c * kGridCellM;
            const double north = half - r * kGridCellM;
            Cesium3DTilesSelection::Tile& leaf =
                leaves.emplace_back(loader, CesiumGeometry::QuadtreeTileID(2, c, r));
            leaf.setBoundingVolume(region(west, north - kGridCellM, west + kGridCellM, north));
            const Cartographic centre = lonLat(west + 0.5 * kGridCellM, north - 0.5 * kGridCellM);
            leaf.setTransform(
                GlobeTransforms::eastNorthUpToFixedFrame(Ellipsoid::WGS84.cartographicToCartesian(
                    Cartographic(centre.longitude, centre.latitude, anchor.origin_height_m))));
        }
    }
    root->createChildTiles(std::move(leaves));
    return root;
}

}

struct FixtureGridHandle {
    std::shared_ptr<GridRequestLog> log;
};

FixtureGridHandle* install_fixture_grid_source(overlume::VisualRenderer* r,
                                               overlume::GeoAnchor anchor, double radius_m,
                                               bool area_selection) {
    if (r == nullptr) return nullptr;
    auto log = std::make_shared<GridRequestLog>();
    auto loader = std::make_unique<GridTestLoader>(log);
    std::unique_ptr<Cesium3DTilesSelection::Tile> root = make_grid_root(loader.get(), anchor);
    CesiumAsync::AsyncSystem asyncSystem(std::make_shared<overlume::SimpleTaskProcessor>());
    std::shared_ptr<overlume::CountingAssetAccessor> counting;
    Cesium3DTilesSelection::TilesetExternals externals =
        overlume::build_externals(std::make_shared<overlume::FileFixtureAssetAccessor>(),
                                  asyncSystem, "off", overlume::kDefaultMaxCacheItems, &counting);
    auto source = std::make_unique<overlume::StreamingEnvironmentSource>(
        externals, 0, std::string(), std::string(), std::string(), anchor, std::move(counting),
        false, false, 0.0, true, 2.0, 1.0, radius_m,
        overlume::CustomTileset{std::move(loader), std::move(root), area_selection});
    source->set_visible(*r, r->environmentVisible);
    if (r->environmentSource) r->environmentSource->teardown(*r);
    r->environmentSource = std::move(source);
    return new FixtureGridHandle{std::move(log)};
}

std::vector<std::pair<int, int>> fixture_grid_requested_leaves(FixtureGridHandle* handle) {
    if (handle == nullptr) return {};
    std::lock_guard<std::mutex> lock(handle->log->mutex);
    return handle->log->leaves;
}

namespace {
// Like UrlTemplateRasterOverlay on `<base>/{z}/{x}/{y}.png`, but every tile carries `credit_html`
// as a per-tile credit (LoadTileImageFromUrlOptions::credits), as Bing's coverage credits do.
class PerTileCreditOverlay final : public CesiumRasterOverlays::RasterOverlay {
public:
    PerTileCreditOverlay(std::string base, std::string credit_html, uint32_t maximum_level)
        : RasterOverlay("fixture-per-tile-credit"),
          base_(std::move(base)),
          creditHtml_(std::move(credit_html)),
          maximumLevel_(maximum_level) {}

    CesiumAsync::Future<CreateTileProviderResult> createTileProvider(
        const CesiumRasterOverlays::CreateRasterOverlayTileProviderParameters& parameters)
        const override {
        const CesiumGeospatial::Projection projection = CesiumGeospatial::WebMercatorProjection();
        const CesiumGeometry::Rectangle rectangle = CesiumGeospatial::projectRectangleSimple(
            projection, CesiumGeospatial::WebMercatorProjection::MAXIMUM_GLOBE_RECTANGLE);
        return parameters.externals.asyncSystem.createResolvedFuture<CreateTileProviderResult>(
            new Provider(this, parameters, projection, rectangle, base_, creditHtml_,
                         maximumLevel_));
    }

private:
    class Provider final : public CesiumRasterOverlays::QuadtreeRasterOverlayTileProvider {
    public:
        Provider(const CesiumUtility::IntrusivePointer<const RasterOverlay>& creator,
                 const CesiumRasterOverlays::CreateRasterOverlayTileProviderParameters& parameters,
                 const CesiumGeospatial::Projection& projection,
                 const CesiumGeometry::Rectangle& rectangle, std::string base,
                 const std::string& credit_html, uint32_t maximum_level)
            : QuadtreeRasterOverlayTileProvider(
                  creator, parameters, projection,
                  CesiumGeometry::QuadtreeTilingScheme(rectangle, 1, 1), rectangle, 0,
                  maximum_level, 256, 256),
              base_(std::move(base)),
              credit_(parameters.externals.pCreditSystem->createCredit(getCreditSource(),
                                                                       credit_html)) {}

    protected:
        CesiumAsync::Future<CesiumRasterOverlays::LoadedRasterOverlayImage> loadQuadtreeTileImage(
            const CesiumGeometry::QuadtreeTileID& id) const override {
            CesiumRasterOverlays::LoadTileImageFromUrlOptions options;
            options.rectangle = getTilingScheme().tileToRectangle(id);
            options.moreDetailAvailable = id.level < getMaximumLevel();
            options.credits.push_back(credit_);
            return loadTileImageFromUrl(base_ + "/" + std::to_string(id.level) + "/" +
                                            std::to_string(id.x) + "/" + std::to_string(id.y) +
                                            ".png",
                                        {}, std::move(options));
        }

    private:
        std::string base_;
        CesiumUtility::Credit credit_;
    };

    std::string base_;
    std::string creditHtml_;
    uint32_t maximumLevel_;
};
}

bool install_fixture_terrain_with_file_imagery(overlume::VisualRenderer* r, const char* fixture_dir,
                                               const char* imagery_dir, double brightness,
                                               uint32_t maximum_level, const char* tile_credit) {
    if (r == nullptr || fixture_dir == nullptr || imagery_dir == nullptr) return false;
    // The anchor every fixture test uses (kFixtureAnchor in test_environment_stream.cpp).
    constexpr overlume::GeoAnchor kAnchor{25.0803, 55.3910, 0.0, 0.0};
    CesiumAsync::AsyncSystem asyncSystem(std::make_shared<overlume::SimpleTaskProcessor>());
    std::shared_ptr<overlume::CountingAssetAccessor> counting;
    Cesium3DTilesSelection::TilesetExternals externals =
        overlume::build_externals(std::make_shared<overlume::FileFixtureAssetAccessor>(),
                                  asyncSystem, "off", overlume::kDefaultMaxCacheItems, &counting);
    auto source = std::make_unique<overlume::StreamingEnvironmentSource>(
        externals, 0, std::string(), std::string("file://") + fixture_dir + "/tileset.json",
        std::string(), kAnchor, std::move(counting), false, false, 0.0, true, 2.0, brightness,
        std::nullopt, overlume::CustomTileset{}, overlume::StreamRendererResources::Kind::Terrain);
    // At the default level 10 one tile holds the fixture's whole region, so a test writes a few
    // files. cesium fills {y} counting rows from the south (QuadtreeTilingScheme::tileToRectangle).
    CesiumRasterOverlays::UrlTemplateRasterOverlayOptions options;
    options.maximumLevel = maximum_level;
    if (tile_credit != nullptr) {
        source->overlays_for_testing().add(new PerTileCreditOverlay(
            std::string("file://") + imagery_dir, tile_credit, maximum_level));
    } else {
        source->overlays_for_testing().add(new CesiumRasterOverlays::UrlTemplateRasterOverlay(
            "fixture-imagery", std::string("file://") + imagery_dir + "/{z}/{x}/{y}.png", {},
            options));
    }
    source->set_visible(*r, r->environmentVisible);
    if (r->environmentSource) r->environmentSource->teardown(*r);
    r->environmentSource = std::move(source);
    return true;
}

bool environment_stream_remove_overlays(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource ||
        r->environmentSource->state() != overlume::EnvironmentSourceState::STREAMING) {
        return false;
    }
    auto* stream = dynamic_cast<overlume::StreamingEnvironmentSource*>(r->environmentSource.get());
    if (stream == nullptr) return false;
    auto& overlays = stream->overlays_for_testing();
    // A copy: remove() erases from the collection's own vector. cesium's remove() detaches the
    // overlay's mapped raster on every live tile before erasing it.
    const auto activated = overlays.getActivatedOverlays();
    for (const auto& o : activated) overlays.remove(o);
    return !activated.empty();
}

OverlayProbe environment_stream_overlay_probe(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return {};
    auto* stream = dynamic_cast<overlume::StreamingEnvironmentSource*>(r->environmentSource.get());
    if (stream == nullptr) return {};
    const overlume::StreamRendererResources::OverlayCounters c = stream->overlay_counters();
    return {c.rasters_attached, c.rasters_detached, c.textures_alive, c.textures_freed};
}

bool environment_stream_has_terrain_tileset(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return false;
    auto* stream = dynamic_cast<overlume::StreamingEnvironmentSource*>(r->environmentSource.get());
    return stream != nullptr && stream->has_terrain();
}

uint64_t environment_terrain_tiles_loaded(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return 0;
    auto* stream = dynamic_cast<overlume::StreamingEnvironmentSource*>(r->environmentSource.get());
    return stream != nullptr ? static_cast<uint64_t>(stream->terrain_scene_count(*r)) : 0;
}

double environment_terrain_tile_lift_z(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return std::nan("");
    auto* stream = dynamic_cast<overlume::StreamingEnvironmentSource*>(r->environmentSource.get());
    return stream != nullptr ? stream->terrain_tile_lift_z(*r) : std::nan("");
}

struct IonFixtureNetwork {
    std::shared_ptr<overlume::MapFixtureAssetAccessor> accessor;
    std::shared_ptr<overlume::StreamRendererResources> last_terrain;
};

namespace {
std::vector<std::byte> as_bytes(const std::vector<uint8_t>& body) {
    const auto* b = reinterpret_cast<const std::byte*>(body.data());
    return std::vector<std::byte>(b, b + body.size());
}
}

IonFixtureNetwork* make_ion_fixture_network() {
    return new IonFixtureNetwork{std::make_shared<overlume::MapFixtureAssetAccessor>(404), nullptr};
}

void ion_fixture_serve(IonFixtureNetwork* n, const std::string& url_without_query, uint16_t status,
                       std::vector<uint8_t> body) {
    n->accessor->serve(url_without_query, status, as_bytes(body));
}

void ion_fixture_serve_prefix(IonFixtureNetwork* n, const std::string& prefix, uint16_t status,
                              std::vector<uint8_t> body) {
    n->accessor->serve_prefix(prefix, status, as_bytes(body));
}

void ion_fixture_hold_prefix(IonFixtureNetwork* n, const std::string& prefix) {
    n->accessor->hold_prefix(prefix);
}

void ion_fixture_release_held(IonFixtureNetwork* n) { n->accessor->release_held(); }

void ion_fixture_fail_all_transport(IonFixtureNetwork* n) { n->accessor->fail_all_transport(); }

std::vector<std::string> ion_fixture_requested_urls(IonFixtureNetwork* n) {
    return n->accessor->requested_urls();
}

bool install_fixture_ion_source(overlume::VisualRenderer* r, const char* ion_uri,
                                overlume::GeoAnchor anchor, IonFixtureNetwork* n,
                                const char* token) {
    constexpr char kScheme[] = "ion://";
    if (r == nullptr || ion_uri == nullptr || n == nullptr) return false;
    const std::string uri(ion_uri);
    if (uri.rfind(kScheme, 0) != 0) return false;
    std::unique_ptr<overlume::StreamingEnvironmentSource> source = overlume::open_ion_source(
        uri.substr(sizeof(kScheme) - 1), anchor, n->accessor, token != nullptr ? token : "");
    if (!source) return false;
    n->last_terrain = source->terrain_resources();
    source->set_visible(*r, r->environmentVisible);
    if (r->environmentSource) r->environmentSource->teardown(*r);
    r->environmentSource = std::move(source);
    return true;
}

OverlayProbe ion_fixture_terrain_probe(IonFixtureNetwork* n) {
    if (n == nullptr || !n->last_terrain) return {};
    const overlume::StreamRendererResources::OverlayCounters c =
        n->last_terrain->overlay_counters();
    return {c.rasters_attached, c.rasters_detached, c.textures_alive, c.textures_freed};
}

bool ion_fixture_terrain_color(IonFixtureNetwork* n, float rgb[3]) {
    if (n == nullptr || !n->last_terrain) return false;
    const std::array<float, 3> c = n->last_terrain->terrain_color_for_testing();
    std::copy(c.begin(), c.end(), rgb);
    return true;
}

bool environment_stream_materials_original(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return false;
    auto* stream = dynamic_cast<overlume::StreamingEnvironmentSource*>(r->environmentSource.get());
    return stream != nullptr && stream->materials_original();
}

double environment_stream_brightness(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return std::nan("");
    auto* stream = dynamic_cast<overlume::StreamingEnvironmentSource*>(r->environmentSource.get());
    return stream != nullptr ? stream->brightness() : std::nan("");
}

double environment_stream_terrain_brightness(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return std::nan("");
    auto* stream = dynamic_cast<overlume::StreamingEnvironmentSource*>(r->environmentSource.get());
    if (stream == nullptr || !stream->terrain_resources()) return std::nan("");
    return stream->terrain_resources()->brightness();
}

bool environment_stream_has_disk_cache(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return false;
    auto* stream = dynamic_cast<overlume::StreamingEnvironmentSource*>(r->environmentSource.get());
    return stream != nullptr && stream->has_disk_cache();
}

double environment_terrain_offset_z(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return std::nan("");
    auto* stream = dynamic_cast<overlume::StreamingEnvironmentSource*>(r->environmentSource.get());
    if (stream == nullptr) return std::nan("");
    return stream->ground_offset_z();
}

double environment_terrain_first_tile_world_z(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return std::nan("");
    auto* stream = dynamic_cast<overlume::StreamingEnvironmentSource*>(r->environmentSource.get());
    if (stream == nullptr) return std::nan("");
    return stream->first_tracked_tile_world_z(*r);
}

int environment_stream_last_view_frustum_count(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return -1;
    auto* stream = dynamic_cast<overlume::StreamingEnvironmentSource*>(r->environmentSource.get());
    if (stream == nullptr) return -1;
    return static_cast<int>(stream->last_view_frustum_count());
}

bool environment_stream_force_fall_back(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return false;
    auto* stream = dynamic_cast<overlume::StreamingEnvironmentSource*>(r->environmentSource.get());
    if (stream == nullptr) return false;
    stream->force_fall_back_for_testing(*r);
    return true;
}

bool environment_stream_parse_materials_original(const char* ion_spec) {
    const std::optional<IonSpec> spec = parse_ion_spec(ion_spec ? ion_spec : "");
    return spec.has_value() && spec->materials_original;
}

bool environment_stream_parse_follow_terrain(const char* ion_spec, bool* out_parse_ok) {
    const std::optional<IonSpec> spec = parse_ion_spec(ion_spec ? ion_spec : "");
    if (out_parse_ok) *out_parse_ok = spec.has_value();
    return spec.has_value() && spec->follow_terrain;
}

bool environment_stream_parse_replaces_ground(const char* ion_spec, bool* out_parse_ok) {
    const std::optional<IonSpec> spec = parse_ion_spec(ion_spec ? ion_spec : "");
    if (out_parse_ok) *out_parse_ok = spec.has_value();
    return spec.has_value() && spec->replaces_ground;
}

double environment_stream_parse_max_tilt_deg(const char* ion_spec, bool* out_parse_ok) {
    const std::optional<IonSpec> spec = parse_ion_spec(ion_spec ? ion_spec : "");
    if (out_parse_ok) *out_parse_ok = spec.has_value();
    return spec.has_value() ? spec->max_tilt_deg : 2.0;
}

double environment_stream_parse_brightness(const char* ion_spec, bool* out_parse_ok) {
    const std::optional<IonSpec> spec = parse_ion_spec(ion_spec ? ion_spec : "");
    if (out_parse_ok) *out_parse_ok = spec.has_value();
    return spec.has_value() ? spec->brightness : 1.0;
}

bool environment_stream_parse_has_radius(const char* ion_spec) {
    const std::optional<IonSpec> spec = parse_ion_spec(ion_spec ? ion_spec : "");
    return spec.has_value() && spec->radius_m.has_value();
}

double environment_stream_parse_radius(const char* ion_spec, bool* out_parse_ok) {
    const std::optional<IonSpec> spec = parse_ion_spec(ion_spec ? ion_spec : "");
    if (out_parse_ok) *out_parse_ok = spec.has_value();
    return spec.has_value() ? spec->radius_m.value_or(kDefaultTileRadiusM) : kDefaultTileRadiusM;
}

bool environment_stream_parse_terrain_assets(const char* ion_spec, int64_t* terrain,
                                             int64_t* imagery, bool* ok) {
    const std::optional<IonSpec> spec = parse_ion_spec(ion_spec ? ion_spec : "");
    if (ok) *ok = spec.has_value();
    if (terrain) *terrain = spec.has_value() ? spec->terrain_asset.value_or(0) : 0;
    if (imagery) *imagery = spec.has_value() ? spec->imagery_asset.value_or(0) : 0;
    return spec.has_value() && spec->terrain_asset.has_value();
}

double terrain_ground_offset_probe(double sampled_height_m, double anchor_height_m,
                                   double ground_bias_m, double current_offset_m,
                                   float delta_seconds, bool snap) {
    const double target =
        overlume::terrain_target_offset_z(sampled_height_m, anchor_height_m, ground_bias_m);
    return snap ? target : overlume::terrain_smooth_toward(current_offset_m, target, delta_seconds);
}

void terrain_plane_fit_probe(const double* s, const double* h, int n, double* out_slope,
                             double* out_intercept, double* out_rms) {
    overlume::terrain_plane_fit(s, h, n, out_slope, out_intercept, out_rms);
}

double terrain_transform_probe(double slope, double intercept, double anchor_height_m,
                               double ground_bias_m, double max_tilt_rad, double heading_rad,
                               double pivot_x, double pivot_y, double s) {
    const double zTarget =
        overlume::terrain_target_offset_z(intercept, anchor_height_m, ground_bias_m);
    const double theta = overlume::terrain_tilt_target(slope, max_tilt_rad);
    const double fwdX = std::cos(heading_rad), fwdY = std::sin(heading_rad);
    const double rawZ = slope * s + intercept - anchor_height_m;

    const filament::math::mat4f m =
        overlume::terrain_root_matrix(zTarget, theta, pivot_x, pivot_y, heading_rad);
    const filament::math::float4 p{static_cast<float>(pivot_x + s * fwdX),
                                   static_cast<float>(pivot_y + s * fwdY), static_cast<float>(rawZ),
                                   1.0f};
    const filament::math::float4 result = m * p;
    return static_cast<double>(result.z);
}

bool environment_stream_first_primitive_is_clay(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return false;
    auto* stream = dynamic_cast<overlume::StreamingEnvironmentSource*>(r->environmentSource.get());
    return stream != nullptr && stream->first_primitive_is_building_material(*r);
}

bool ecef_to_map_probe(double origin_lat_deg, double origin_lon_deg, double heading_rad,
                       double origin_height_m, double lat_deg, double lon_deg, double alt_m,
                       double* out_x, double* out_y, double* out_z) {
    const overlume::GeoAnchor anchor{origin_lat_deg, origin_lon_deg, heading_rad, origin_height_m};
    const glm::dmat4 ecefToMap = overlume::compute_ecef_to_map(anchor);
    const CesiumGeospatial::Cartographic carto =
        CesiumGeospatial::Cartographic::fromDegrees(lon_deg, lat_deg, alt_m);
    const glm::dvec3 ecef = CesiumGeospatial::Ellipsoid::WGS84.cartographicToCartesian(carto);
    const glm::dvec4 mapPt = ecefToMap * glm::dvec4(ecef, 1.0);
    if (out_x) *out_x = mapPt.x;
    if (out_y) *out_y = mapPt.y;
    if (out_z) *out_z = mapPt.z;
    return true;
}

bool ecef_height_correction_probe(double origin_lat_deg, double origin_lon_deg, double heading_rad,
                                  double origin_height_m, double lat_deg, double lon_deg,
                                  double alt_m, double* out_z_uncorrected,
                                  double* out_z_corrected) {
    const overlume::GeoAnchor anchor{origin_lat_deg, origin_lon_deg, heading_rad, origin_height_m};
    const glm::dmat4 ecefToMap = overlume::compute_ecef_to_map(anchor);
    const CesiumGeospatial::Cartographic carto =
        CesiumGeospatial::Cartographic::fromDegrees(lon_deg, lat_deg, alt_m);
    const glm::dvec3 ecef = CesiumGeospatial::Ellipsoid::WGS84.cartographicToCartesian(carto);
    const glm::dvec4 oldMapPt = ecefToMap * glm::dvec4(ecef, 1.0);
    if (out_z_uncorrected) *out_z_uncorrected = oldMapPt.z;
    if (out_z_corrected) {
        *out_z_corrected = overlume::correct_ecef_point_height(ecef, ecefToMap, origin_height_m).z;
    }
    return true;
}

void capture_stream_glbs(bool on) {
    std::lock_guard<std::mutex> lock(g_capturedGlbsMutex);
    if (on) g_capturedGlbs.clear();
    g_captureStreamGlbs.store(on);
}

std::vector<std::vector<uint8_t>> captured_stream_glbs() {
    std::lock_guard<std::mutex> lock(g_capturedGlbsMutex);
    return g_capturedGlbs;
}

std::string captured_cesium_log_text() {
    return g_redactingSinkForTest ? g_redactingSinkForTest->captured_text_for_test()
                                  : std::string();
}

bool drive_ion_token_redaction_probe(const char* bogus_token, int64_t asset_id, int max_ticks) {
    auto fileAccessor = std::make_shared<overlume::FileFixtureAssetAccessor>();
    CesiumAsync::AsyncSystem asyncSystem(std::make_shared<overlume::SimpleTaskProcessor>());
    std::shared_ptr<overlume::CountingAssetAccessor> counting;
    Cesium3DTilesSelection::TilesetExternals externals = overlume::build_externals(
        fileAccessor, asyncSystem, "off", overlume::kDefaultMaxCacheItems, &counting);
    Cesium3DTilesSelection::TilesetOptions options;
    auto tileset = std::make_unique<Cesium3DTilesSelection::Tileset>(
        externals, asset_id, std::string(bogus_token), options);
    for (int i = 0; i < max_ticks; ++i) {
        asyncSystem.dispatchMainThreadTasks();
        if (captured_cesium_log_text().find("access_token=") != std::string::npos) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return captured_cesium_log_text().find("access_token=") != std::string::npos;
}

}
