// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "environment_stream.hpp"

#include "environment_test_hooks.hpp"
#include "gltf_normals.hpp"
#include "renderer_internal.hpp"

#include <Cesium3DTilesContent/registerAllTileContentTypes.h>
#include <CesiumAsync/CachingAssetAccessor.h>
#include <CesiumAsync/SqliteCache.h>
#include <CesiumCurl/CurlAssetAccessor.h>
#include <CesiumGeometry/Transforms.h>
#include <CesiumGeospatial/Cartographic.h>
#include <CesiumGeospatial/Ellipsoid.h>
#include <CesiumGltf/AccessorWriter.h>
#include <CesiumGltf/Model.h>
#include <CesiumGltfWriter/GltfWriter.h>

#include <mutex>

#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/TransformManager.h>

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

#include <unistd.h>

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
};

std::optional<IonSpec> parse_ion_spec(const std::string& spec) {
    const size_t q = spec.find('?');
    const std::string idPart = spec.substr(0, q);
    if (idPart.empty()) return std::nullopt;
    IonSpec out;
    try {
        size_t consumed = 0;
        out.asset_id = std::stoll(idPart, &consumed);
        if (consumed != idPart.size() || out.asset_id <= 0) return std::nullopt;
    } catch (const std::exception&) {
        return std::nullopt;
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
                        return std::nullopt;
                } catch (const std::exception&) {
                    return std::nullopt;
                }
            } else if (key == "follow_terrain") {
                if (val == "on" || val == "true" || val == "1") {
                    out.follow_terrain = true;
                } else if (val == "off" || val == "false" || val == "0") {
                    out.follow_terrain = false;
                } else {
                    return std::nullopt;
                }
            } else if (key == "max_tilt_deg") {
                try {
                    size_t used = 0;
                    out.max_tilt_deg = std::stod(val, &used);
                    if (used != val.size() || !std::isfinite(out.max_tilt_deg) ||
                        out.max_tilt_deg < 0.0)
                        return std::nullopt;
                } catch (const std::exception&) {
                    return std::nullopt;
                }
            } else if (key == "brightness") {
                try {
                    size_t used = 0;
                    out.brightness = std::stod(val, &used);
                    if (used != val.size() || !std::isfinite(out.brightness) ||
                        out.brightness <= 0.0)
                        return std::nullopt;
                } catch (const std::exception&) {
                    return std::nullopt;
                }
            } else if (key == "replaces_ground") {
                if (val == "on" || val == "true" || val == "1") {
                    out.replaces_ground = true;
                } else if (val == "off" || val == "false" || val == "0") {
                    out.replaces_ground = false;
                } else {
                    return std::nullopt;
                }
            }
        }
        if (amp == std::string::npos) break;
        pos = amp + 1;
    }
    return out;
}

std::string default_cache_dir() {
    const char* xdg = std::getenv("XDG_CACHE_HOME");
    const char* home = std::getenv("HOME");
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

std::shared_ptr<spdlog::logger> make_redacting_logger();

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

Cesium3DTilesSelection::TilesetExternals build_externals(
    std::shared_ptr<CesiumAsync::IAssetAccessor> base, const CesiumAsync::AsyncSystem& asyncSystem,
    const std::string& cache_dir, uint64_t max_cache_items,
    std::shared_ptr<CountingAssetAccessor>* out_counting = nullptr) {
    auto counting = std::make_shared<CountingAssetAccessor>(std::move(base));
    if (out_counting != nullptr) *out_counting = counting;
    auto logger = make_redacting_logger();

    if (cache_dir == "off") {
        Cesium3DTilesSelection::TilesetExternals externals{nullptr, nullptr, asyncSystem};
        externals.pAssetAccessor = counting;
        externals.pLogger = logger;
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
    return externals;
}

}  // namespace

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

}  // namespace

std::shared_ptr<CesiumAsync::IAssetRequest> FileFixtureAssetAccessor::makeRequest(
    const std::string& verb, const std::string& url) {
    const bool isRootManifest =
        url.size() >= 12 && url.compare(url.size() - 12, 12, "tileset.json") == 0;
    if (killed_ && killed_->load() && !isRootManifest) {
        return std::make_shared<FixtureAssetRequest>(
            verb, url, std::make_unique<FixtureAssetResponse>(503, std::vector<std::byte>{}));
    }
    std::string path = url;
    if (path.rfind(kFileScheme, 0) == 0) path = path.substr(sizeof(kFileScheme) - 1);
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
}  // namespace

bool StreamingEnvironmentSource::provides_ground() const {
    return !fallenBack_ && replacesGround_ && terrainState_ && terrainState_->ground_hit.load();
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

CesiumAsync::Future<Cesium3DTilesSelection::TileLoadResultAndRenderResources>
StreamRendererResources::prepareInLoadThread(
    const CesiumAsync::AsyncSystem& asyncSystem,
    Cesium3DTilesSelection::TileLoadResult&& tileLoadResult, const glm::dmat4& transform,
    const std::any&) {
    auto* pGlb = new LoadThreadGlb();
    pGlb->transform = transform * CesiumGeometry::Transforms::getUpAxisTransform(
                                      tileLoadResult.glTFUpAxis, CesiumGeometry::Axis::Z);
    if (auto* model = std::get_if<CesiumGltf::Model>(&tileLoadResult.contentKind)) {
        if (consolidate_buffers(*model)) {
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

    filament::RenderableManager& rm = r_->engine->getRenderableManager();
    const utils::Entity* renderables = asset->getRenderableEntities();
    const size_t renderableCount = asset->getRenderableEntityCount();
    for (size_t i = 0; i < renderableCount; ++i) {
        const auto inst = rm.getInstance(renderables[i]);
        if (!inst.isValid()) continue;
        const size_t primCount = rm.getPrimitiveCount(inst);
        if (!materialsOriginal_) {
            for (size_t p = 0; p < primCount; ++p) {
                rm.setMaterialInstanceAt(inst, p, r_->buildingMaterial);
            }
        }
        rm.setCastShadows(inst, true);
        rm.setReceiveShadows(inst, true);
    }

    if (materialsOriginal_ && brightness_ != 1.0) {
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

std::vector<filament::gltfio::FilamentAsset*> StreamRendererResources::drain_pending_frees() {
    std::vector<filament::gltfio::FilamentAsset*> toFree;
    {
        std::lock_guard<std::mutex> lock(freeMutex_);
        toFree.swap(pendingFrees_);
    }
    if (r_ == nullptr) return {};
    for (filament::gltfio::FilamentAsset* asset : toFree) {
        r_->scene->removeEntities(asset->getEntities(), asset->getEntityCount());
        r_->sharedAssetLoader->destroyAsset(asset);
    }
    return toFree;
}

StreamingEnvironmentSource::StreamingEnvironmentSource(
    Cesium3DTilesSelection::TilesetExternals externals, int64_t asset_id,
    std::string ion_access_token, std::string root_tileset_uri, std::string fallback_baked_dir,
    GeoAnchor anchor, std::shared_ptr<CountingAssetAccessor> counting_accessor,
    bool materials_original, bool follow_terrain, double ground_bias_m, bool replaces_ground,
    double max_tilt_deg, double brightness)
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
      countingAccessor_(std::move(counting_accessor)) {
    static std::once_flag registerContentTypesOnce;
    std::call_once(registerContentTypesOnce,
                   [] { Cesium3DTilesContent::registerAllTileContentTypes(); });

    renderResources_ = std::make_shared<StreamRendererResources>(
        ecefToMap_, anchor_.origin_height_m, materialsOriginal_, brightness_);
    externals.pPrepareRendererResources = renderResources_;

    Cesium3DTilesSelection::TilesetOptions options;
    options.maximumScreenSpaceError = kStreamMaxSseErr;
    if (asset_id > 0 && !ion_access_token.empty()) {
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
    for (filament::gltfio::FilamentAsset* freed : renderResources_->drain_pending_frees()) {
        inScene_.erase(freed);
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
    tileset_->sampleHeightMostDetailed(cartos)
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
}  // namespace

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
        renderResources_->set_terrain_transform(
            groundOffsetZ_, terrainTiltRad_, terrainState_->fit_pivot_x, terrainState_->fit_pivot_y,
            terrainState_->fit_pivot_heading_rad);
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
}  // namespace

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

    if (followTerrain_) {
        maybe_trigger_terrain_sample(ego_map_pos, r.scene_buffer.active().ego.heading_rad);
        update_terrain_transform(deltaSeconds);
    }

    const std::optional<ViewState> cameraView = camera_view_state(r, mapToEcef_);
    const std::vector<ViewState> views = cameraView.has_value()
                                             ? std::vector<ViewState>{view, *cameraView}
                                             : std::vector<ViewState>{view};
    lastViewFrustumCount_ = views.size();
    const ViewUpdateResult& result = tileset_->updateViewGroup(*viewGroup_, views, deltaSeconds);
    tileset_->loadTiles();
    asyncSystem_.dispatchMainThreadTasks();

    std::unordered_map<const void*, bool> stillPresent;
    for (const auto& tilePtr : result.tilesToRenderThisFrame) {
        const Cesium3DTilesSelection::Tile* tile = tilePtr.get();
        if (tile == nullptr || !tile->isRenderContent()) continue;
        const auto* renderContent = tile->getContent().getRenderContent();
        if (renderContent == nullptr) continue;
        void* res = renderContent->getRenderResources();
        if (res == nullptr) continue;
        auto* asset = static_cast<filament::gltfio::FilamentAsset*>(res);
        stillPresent[res] = true;
        if (visible_ && inScene_.find(res) == inScene_.end()) {
            r.scene->addEntities(asset->getEntities(), asset->getEntityCount());
        }
    }
    for (auto it = inScene_.begin(); it != inScene_.end();) {
        if (stillPresent.find(it->first) == stillPresent.end()) {
            auto* asset =
                static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(it->first));
            if (visible_) {
                r.scene->removeEntities(asset->getEntities(), asset->getEntityCount());
            }
            it = inScene_.erase(it);
        } else {
            ++it;
        }
    }
    inScene_ = std::move(stillPresent);
}

void StreamingEnvironmentSource::teardown(VisualRenderer& r) {
    if (fallbackSource_) {
        fallbackSource_->teardown(r);
        fallbackSource_.reset();
    }
    if (tornDown_) return;
    renderResources_->set_renderer(&r);

    for (auto& [res, _] : inScene_) {
        auto* asset = static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(res));
        if (visible_) {
            r.scene->removeEntities(asset->getEntities(), asset->getEntityCount());
        }
    }
    inScene_.clear();
    renderResources_->drain_pending_frees();

    CesiumAsync::SharedFuture<void> destructionComplete =
        tileset_->getAsyncDestructionCompleteEvent();
    tileset_.reset();
    renderResources_->drain_pending_frees();

    int pumps = 0;
    for (; pumps < kTeardownPumpBound && !destructionComplete.isReady(); ++pumps) {
        asyncSystem_.dispatchMainThreadTasks();
    }
    if (pumps >= kTeardownPumpBound && !destructionComplete.isReady()) {
        ++leakedOnTeardownBound_;
    }

    renderResources_->drain_pending_frees();
    renderResources_->destroy_terrain_root();
    renderResources_->note_torn_down();
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
    for (auto& [res, _] : inScene_) {
        auto* asset = static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(res));
        if (visible_) {
            r.scene->addEntities(asset->getEntities(), asset->getEntityCount());
        } else {
            r.scene->removeEntities(asset->getEntities(), asset->getEntityCount());
        }
    }
}

size_t StreamingEnvironmentSource::loaded_count() const {
    if (fallenBack_) return fallbackSource_ ? fallbackSource_->loaded_count() : 0;
    return inScene_.size();
}

size_t StreamingEnvironmentSource::scene_membership_count(VisualRenderer& r) const {
    if (fallenBack_) return fallbackSource_ ? fallbackSource_->scene_membership_count(r) : 0;
    size_t count = 0;
    for (const auto& [res, _] : inScene_) {
        auto* asset = static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(res));
        if (asset->getEntityCount() > 0 && r.scene->hasEntity(asset->getEntities()[0])) {
            ++count;
        }
    }
    return count;
}

EnvironmentSourceState StreamingEnvironmentSource::state() const {
    return fallenBack_ ? EnvironmentSourceState::STREAMING_FALLBACK
                       : EnvironmentSourceState::STREAMING;
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

}  // namespace overlume

namespace overlume {

std::unique_ptr<EnvironmentSource> open_streaming_environment_source(const std::string& ion_spec,
                                                                     GeoAnchor anchor) {
    const std::optional<IonSpec> spec = parse_ion_spec(ion_spec);
    if (!spec) return nullptr;

    const char* token = std::getenv("CESIUM_ION_TOKEN");
    if (token == nullptr || token[0] == '\0') return nullptr;

    const std::string cacheDir = spec->cache_dir.empty() ? default_cache_dir() : spec->cache_dir;

    auto curl = std::make_shared<CesiumCurl::CurlAssetAccessor>();
    CesiumAsync::AsyncSystem asyncSystem(std::make_shared<SimpleTaskProcessor>());
    std::shared_ptr<CountingAssetAccessor> counting;
    Cesium3DTilesSelection::TilesetExternals externals =
        build_externals(curl, asyncSystem, cacheDir, spec->max_cache_items, &counting);

    return std::make_unique<StreamingEnvironmentSource>(
        externals, spec->asset_id, std::string(token), std::string(), spec->fallback_dir, anchor,
        std::move(counting), spec->materials_original, spec->follow_terrain, spec->ground_bias_m,
        spec->replaces_ground, spec->max_tilt_deg, spec->brightness);
}

}  // namespace overlume

namespace overlume::testing {

struct FixtureStreamHandle {
    std::shared_ptr<std::atomic<bool>> killed;
};

namespace {

std::string test_cache_dir() {
    static const std::string dir = [] {
        const std::string d = (std::filesystem::temp_directory_path() /
                               ("overlume-stream-test-cache-" + std::to_string(::getpid())))
                                  .string();
        std::error_code ec;
        std::filesystem::remove_all(d, ec);
        return d;
    }();
    return dir;
}

std::unique_ptr<overlume::EnvironmentSource> make_fixture_source(
    const char* fixture_dir, const char* fallback_baked_dir, overlume::GeoAnchor anchor,
    std::shared_ptr<std::atomic<bool>> killed, bool materials_original,
    bool follow_terrain = false) {
    if (fixture_dir == nullptr) return nullptr;
    auto fileAccessor = std::make_shared<overlume::FileFixtureAssetAccessor>(std::move(killed));
    CesiumAsync::AsyncSystem asyncSystem(std::make_shared<overlume::SimpleTaskProcessor>());
    const std::string cacheDir = test_cache_dir();
    std::shared_ptr<overlume::CountingAssetAccessor> counting;
    Cesium3DTilesSelection::TilesetExternals externals = overlume::build_externals(
        fileAccessor, asyncSystem, cacheDir, overlume::kDefaultMaxCacheItems, &counting);
    const std::string tilesetUri = std::string("file://") + fixture_dir + "/tileset.json";
    return std::make_unique<overlume::StreamingEnvironmentSource>(
        externals, 0, std::string(), tilesetUri,
        fallback_baked_dir ? std::string(fallback_baked_dir) : std::string(), anchor,
        std::move(counting), materials_original, follow_terrain, 0.0, true);
}

}  // namespace

bool install_fixture_streaming_source(overlume::VisualRenderer* r, const char* fixture_dir,
                                      overlume::GeoAnchor anchor, bool materials_original,
                                      bool follow_terrain) {
    if (r == nullptr) return false;
    if (r->environmentSource) r->environmentSource->teardown(*r);
    auto source = make_fixture_source(fixture_dir, nullptr, anchor, nullptr, materials_original,
                                      follow_terrain);
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
    auto source =
        make_fixture_source(fixture_dir, fallback_baked_dir, anchor, killed, false, follow_terrain);
    if (!source) {
        r->environmentSource.reset();
        return nullptr;
    }
    r->environmentSource = std::move(source);
    return new FixtureStreamHandle{std::move(killed)};
}

void kill_fixture_network(FixtureStreamHandle* handle) {
    if (handle && handle->killed) handle->killed->store(true);
}

void revive_fixture_network(FixtureStreamHandle* handle) {
    if (handle && handle->killed) handle->killed->store(false);
}

bool environment_stream_materials_original(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return false;
    auto* stream = dynamic_cast<overlume::StreamingEnvironmentSource*>(r->environmentSource.get());
    return stream != nullptr && stream->materials_original();
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

}  // namespace overlume::testing
