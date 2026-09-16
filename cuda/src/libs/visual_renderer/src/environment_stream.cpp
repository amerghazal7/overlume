// environment_stream.cpp — the ONE C++20 TU (Decision 3). Cesium-native
// types NEVER leak outside this file: environment.hpp's factory declaration
// and environment_test_hooks.hpp's test hooks are the only surfaces the
// rest of the library (or any test TU) sees. See environment_stream.hpp's
// header comment + the epic plan's Task 3 for the full design.
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
#include <CesiumGltf/Model.h>
#include <CesiumGltfWriter/GltfWriter.h>

#include <mutex>

#include <spdlog/spdlog.h>

#include <filament/RenderableManager.h>
#include <filament/TransformManager.h>

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <optional>

#include <unistd.h>  // getpid() -- test_cache_dir()'s per-process key (Linux-only build, fine here)

namespace mpviz {

namespace {

// ── Decision 5's ~30-line split-only parser: "<assetId>[?cache=<dir>]
//    [&fallback=<baked_dir>][&max_cache_items=<n>][&materials=original|clay]"
//    (the materials= key is VM-064/Task 5's one addition). No URL library --
//    "?"/"&"/"=" split only.
//    ponytail: split-only parser; percent-encoding if a real path ever
//    needs it. ─────────────────────────────────────────────────────────
struct IonSpec {
    int64_t asset_id = 0;
    std::string cache_dir;
    std::string fallback_dir;
    uint64_t max_cache_items = kDefaultMaxCacheItems;
    // VM-064 (Task 5): "original" keeps gltfio's own ubershader materials
    // (Google Photorealistic 3D Tiles); "clay"/absent (default) is today's
    // buildingMaterial remap, byte-identical to every pre-VM-064 URI.
    bool materials_original = false;
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
                    // malformed -- keep the default rather than fail the whole open.
                }
            } else if (key == "materials") {
                if (val == "original") {
                    out.materials_original = true;
                } else if (val == "clay") {
                    out.materials_original = false;
                } else {
                    // Spec §9 "malformed data degrades, never crashes": an
                    // unknown materials= value is treated as clay (today's
                    // behavior) rather than failing the whole open --
                    // WARNed once, process-wide (same std::call_once shape
                    // as registerAllTileContentTypes() below).
                    static std::once_flag unknownMaterialsWarnOnce;
                    std::call_once(unknownMaterialsWarnOnce, [&val] {
                        spdlog::warn("environment_stream: unknown materials='{}' -- treating as clay", val);
                    });
                    out.materials_original = false;
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
    return base + "/mpviz-tile-cache";
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

// Decision 7 / Decision 15.4's named unknown, RESOLVED at implementation:
// real OSM Buildings b3dm tiles are NOT single-buffer (verified against
// this fixture's 3 real tiles: 21/9/89 buffers each -- draco/meshopt
// compression splits per-primitive, one buffer per compressed
// bufferView). writeGlb's own single-buffer contract
// (CesiumGltfWriter/GltfWriter.h: "the first buffer object implicitly
// refers to the GLB binary chunk") means every real tile would be
// silently skipped without this pass. Concatenates every buffer into one
// contiguous chunk (4-byte aligned, matching glTF's own alignment
// convention), rebasing each bufferView's byteOffset by its original
// buffer's new position and repointing it at buffer 0. Returns false only
// if the model has zero buffers (nothing to consolidate -- not observed
// against real content).
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
        if (const size_t rem = merged.size() % 4; rem != 0) merged.resize(merged.size() + (4 - rem));
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

// gltfio's AssetLoader rejects (createAsset returns nullptr, logging
// "Unrecognized vertex semantic") any primitive carrying a non-standard
// vertex attribute -- confirmed against this fixture's real tiles: OSM
// Buildings b3dm content carries a per-vertex `_BATCHID` attribute (the
// legacy b3dm batch-table linkage), which is not one of glTF's core
// semantics gltfio recognizes. We re-materialize every primitive onto
// r.buildingMaterial regardless of feature/batch id (Decision 7's clay
// remap), so batch linkage is unused here -- stripped before writeGlb
// rather than worked around downstream. Strips every attribute whose name
// starts with `_` (glTF's own "application-specific attribute" prefix
// convention, so this also covers any `_FEATURE_ID_n` an EXT_mesh_features
// tile might carry, not just `_BATCHID`).
void strip_custom_vertex_attributes(CesiumGltf::Model& model) {
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
}

// Builds the composed accessor stack shared by both the fixture and the
// real ion path (Decision 10/11, Task 3 Step 2): base -> CountingAssetAccessor
// -> CachingAssetAccessor(SqliteCache(dbPath, maxItems)). The
// CountingAssetAccessor's failure count is Task 4's fallback trigger, not
// read by anything in Task 3 -- built into the stack now (Step 4's
// disk-cache proof explicitly wants it present) so Task 4 doesn't have to
// re-plumb this.
// `out_counting`, when non-null, receives the SAME CountingAssetAccessor
// wrapped into the returned externals' pAssetAccessor chain (Decision 11 /
// VM-063 Task 4: StreamingEnvironmentSource keeps its own shared_ptr to it
// so update() can read consecutive_failures() -- the externals struct only
// exposes the composed IAssetAccessor base, not this concrete type).
Cesium3DTilesSelection::TilesetExternals build_externals(
    std::shared_ptr<CesiumAsync::IAssetAccessor> base, const CesiumAsync::AsyncSystem& asyncSystem,
    const std::string& cache_dir, uint64_t max_cache_items,
    std::shared_ptr<CountingAssetAccessor>* out_counting = nullptr) {
    auto counting = std::make_shared<CountingAssetAccessor>(std::move(base));
    if (out_counting != nullptr) *out_counting = counting;
    auto logger = spdlog::default_logger();

    // VM-064 Decision 14 / Task 5 Step 2(b): "off" is a documented sentinel
    // (not a directory) -- Google's Map Tiles terms bound how long tile
    // responses may be cached, and the google preset's shipped default sets
    // `cache=off` until those cache-lifetime terms are re-verified for this
    // deployment (docs/visual_mode/cesium.md's Google section). No
    // SqliteCache/CachingAssetAccessor is constructed in this branch --
    // requests go straight through the counting decorator, nothing
    // persisted to disk. ponytail: literal "off"/dir-path dispatch, an enum
    // is the upgrade if a third cache mode is ever needed.
    if (cache_dir == "off") {
        Cesium3DTilesSelection::TilesetExternals externals{nullptr, nullptr, asyncSystem};
        externals.pAssetAccessor = counting;
        externals.pLogger = logger;
        return externals;
    }

    std::error_code ec;
    std::filesystem::create_directories(cache_dir, ec);  // best-effort; SqliteCache errors loudly if this fails for real
    auto cacheDb =
        std::make_shared<CesiumAsync::SqliteCache>(logger, cache_dir + "/cesium-tiles.sqlite", max_cache_items);
    auto caching = std::make_shared<CesiumAsync::CachingAssetAccessor>(logger, counting, cacheDb);

    // TilesetExternals has no default constructor (its `asyncSystem` member
    // doesn't) -- aggregate-init the one member that actually requires a
    // value at construction, then assign the rest (every other member has
    // its own default member initializer, TilesetExternals.h).
    Cesium3DTilesSelection::TilesetExternals externals{nullptr, nullptr, asyncSystem};
    externals.pAssetAccessor = caching;
    externals.pLogger = logger;
    return externals;
}

}  // namespace

// ── ECEF <-> map-frame (Decision 8) ──────────────────────────────────────
glm::dmat4 compute_ecef_to_map(const GeoAnchor& anchor) {
    const CesiumGeospatial::LocalHorizontalCoordinateSystem enu(
        CesiumGeospatial::Cartographic::fromDegrees(anchor.origin_lon_deg, anchor.origin_lat_deg, 0.0));
    const glm::dmat4 ecefToEnu = enu.getEcefToLocalTransformation();

    // VM-062 gate round 1, Finding 1: getEcefToLocalTransformation() above
    // returns ENU on the TRUE WGS84 ellipsoid, but geo_anchor.cpp's
    // WgsToMap/MapToWgs -- the map-frame model every other map-frame
    // consumer (baked chunks, GPS-derived ego) actually agrees with -- is a
    // fixed-radius SPHERE (kEarthRadiusM = 6371000, geo_anchor.cpp:14-20).
    // Left unreconciled, streamed tiles drift off everything else in the
    // map frame by a curvature-vs-sphere error that grows linearly with
    // distance from the anchor (measured ~7.5 m at a 2.6 km probe -- see
    // this task's Step 3 results block). Rescale the ellipsoidal
    // east/north axes by the ratio of the sphere model's radius to the
    // ellipsoid's own local radii of curvature at the anchor's latitude
    // (N = prime-vertical radius, M = meridian radius), so this transform
    // reproduces geo_anchor.cpp's own east/north formulas
    // (kEarthRadiusM * cos(lat0) * dlon, kEarthRadiusM * dlat) instead of
    // the ellipsoid's -- BEFORE applying the same heading rotation
    // geo_anchor.cpp:23-37 uses.
    constexpr double kWgs84A = 6378137.0;              // WGS84 semi-major axis (m)
    constexpr double kWgs84F = 1.0 / 298.257223563;    // WGS84 flattening
    constexpr double kWgs84E2 = kWgs84F * (2.0 - kWgs84F);  // first eccentricity^2
    // geo_anchor.cpp's own kEarthRadiusM, duplicated here rather than
    // shared across the node/library boundary -- this TU cannot include
    // geo_anchor.hpp (Decision 3's node/library quarantine).
    constexpr double kMapSphereRadiusM = 6371000.0;
    const double lat0_rad = anchor.origin_lat_deg * (M_PI / 180.0);
    const double sin2Lat0 = std::sin(lat0_rad) * std::sin(lat0_rad);
    const double denom = 1.0 - kWgs84E2 * sin2Lat0;
    const double N = kWgs84A / std::sqrt(denom);                        // prime-vertical radius
    const double M = kWgs84A * (1.0 - kWgs84E2) / (denom * std::sqrt(denom));  // meridian radius
    glm::dmat4 sphereScale(1.0);
    sphereScale[0][0] = kMapSphereRadiusM / N;  // east
    sphereScale[1][1] = kMapSphereRadiusM / M;  // north

    const double s = std::sin(anchor.heading_rad), c = std::cos(anchor.heading_rad);
    // Same rotation geo_anchor.cpp's WgsToMap applies to (east, north):
    // map.x = east*s + north*c; map.y = -east*c + north*s; map.z = up
    // (verbatim -- geo_anchor.cpp:23-37). Expressed as a 4x4 that leaves
    // z/w alone: mat[col][row] is the coefficient of input axis `col` in
    // output row `row` (glm's column-major convention).
    glm::dmat4 rot(1.0);
    rot[0][0] = s;
    rot[1][0] = c;
    rot[0][1] = -c;
    rot[1][1] = s;
    return rot * sphereScale * ecefToEnu;
}

// ── FileFixtureAssetAccessor (Decision 13; test-only) ────────────────────
namespace {

class FixtureAssetResponse final : public CesiumAsync::IAssetResponse {
public:
    FixtureAssetResponse(uint16_t status, std::vector<std::byte> data)
        : status_(status), data_(std::move(data)) {
        // Step 4's disk-cache proof needs these responses to actually get
        // stored: CachingAssetAccessor's shouldCacheRequest (Decision 10)
        // requires a Cache-Control max-age/Expires/ETag/Last-Modified
        // header on a 200 response before it persists anything -- with no
        // headers at all (this class's original shape), every fixture
        // response was silently treated as non-cacheable and NEVER
        // written to the sqlite db, so a second source pointed at the
        // same cache dir had nothing to read. A real ion response carries
        // its own Cache-Control; this is the fixture-only equivalent.
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
    FixtureAssetRequest(std::string method, std::string url, std::unique_ptr<FixtureAssetResponse> resp)
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

// "file://<abs path>" -> "<abs path>" (Decision 13: the fixture's own
// scheme, resolved by cesium's own URI-join logic against the fixture's
// tileset.json url exactly like a real http(s) url would be, but served
// from local disk here -- no network, no token).
constexpr char kFileScheme[] = "file://";

}  // namespace

std::shared_ptr<CesiumAsync::IAssetRequest> FileFixtureAssetAccessor::makeRequest(
    const std::string& verb, const std::string& url) {
    // VM-063 (Task 4): the root tileset.json manifest is exempt from the
    // kill switch -- it models the realistic shape of "network loss
    // mid-run" (Decision 11): a real deployment resolves the root manifest
    // ONCE at startup, while healthy, and every subsequent per-tile content
    // request is what actually observes a later network loss. Without this
    // exemption, killing from before the root ever resolves leaves cesium
    // with no known children to request at all (root fetch fails once,
    // permanently, with nothing to retry -- verified empirically, see
    // environment_ion_fixture_fallback_0/PROVENANCE.md), which can never
    // reach kNetworkLossConsecutiveFailures. Real ion tilesets don't
    // special-case this (the accessor decorator stack has no such
    // exemption) -- it exists only in this test-only fixture accessor.
    const bool isRootManifest = url.size() >= 12 && url.compare(url.size() - 12, 12, "tileset.json") == 0;
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
    return std::make_shared<FixtureAssetRequest>(verb, url,
                                                  std::make_unique<FixtureAssetResponse>(200, std::move(bytes)));
}

// ── StreamRendererResources (Decision 7) ─────────────────────────────────
CesiumAsync::Future<Cesium3DTilesSelection::TileLoadResultAndRenderResources>
StreamRendererResources::prepareInLoadThread(const CesiumAsync::AsyncSystem& asyncSystem,
                                              Cesium3DTilesSelection::TileLoadResult&& tileLoadResult,
                                              const glm::dmat4& transform, const std::any& /*rendererOptions*/) {
    auto* pGlb = new LoadThreadGlb();
    // Real OSM Buildings b3dm content is Y-up (glTF's own convention,
    // TileLoadResult::glTFUpAxis, default Y) with vertex positions already
    // resolved to absolute ECEF-scale numbers (no separate RTC_CENTER node
    // -- verified at implementation: the pre-existing FilamentAsset root
    // transform was plain identity, yet raw vertex magnitudes were ~6.37e6,
    // Earth-radius scale). ECEF itself is Z-up (Z = north-pole axis), so
    // Y-up content must be rotated back to Z-up BEFORE `ecefToMap_` (built
    // for genuine ECEF input, Decision 8) can place it correctly --
    // skipping this rotated every real tile ~6-7 million meters from the
    // anchor (confirmed empirically: same magnitude with or without
    // `transform`, since neither tile.getTransform() nor this parameter
    // carries the axis fix -- it's a fixed, content-format convention, not
    // a per-tile placement value). cesium-native ships the exact
    // conversion (Transforms::getUpAxisTransform) rather than a hand-rolled
    // one.
    pGlb->transform =
        transform * CesiumGeometry::Transforms::getUpAxisTransform(tileLoadResult.glTFUpAxis,
                                                                     CesiumGeometry::Axis::Z);
    if (auto* model = std::get_if<CesiumGltf::Model>(&tileLoadResult.contentKind)) {
        // Decision 7/15.4: real OSM Buildings tiles are multi-buffer
        // (verified at implementation) -- consolidate before writeGlb,
        // whose own single-buffer GLB-chunk contract requires exactly one.
        if (consolidate_buffers(*model)) {
            strip_custom_vertex_attributes(*model);
            const auto& bufData = model->buffers[0].cesium.data;
            CesiumGltfWriter::GltfWriter writer;
            const CesiumGltfWriter::GltfWriterResult res =
                writer.writeGlb(*model, std::span<const std::byte>(bufData.data(), bufData.size()));
            if (res.errors.empty()) {
                // Defense-in-depth, not a known-needed fix: every committed
                // environment_ion_fixture_*/*.b3dm already carries NORMAL,
                // so this is a no-op on real tiles today (see
                // gltf_normals.hpp) -- but it's the same load-time hook
                // environment.cpp uses for baked chunks, and costs nothing
                // on tiles that already have normals.
                std::vector<uint8_t> bytes(reinterpret_cast<const uint8_t*>(res.gltfBytes.data()),
                                            reinterpret_cast<const uint8_t*>(res.gltfBytes.data() +
                                                                              res.gltfBytes.size()));
                bytes = ensure_flat_normals(std::move(bytes));
                pGlb->glbBytes.resize(bytes.size());
                std::memcpy(pGlb->glbBytes.data(), bytes.data(), bytes.size());
                pGlb->ok = true;
            }
        }
    }
    Cesium3DTilesSelection::TileLoadResultAndRenderResources out{std::move(tileLoadResult), pGlb};
    return asyncSystem.createResolvedFuture(std::move(out));
}

void* StreamRendererResources::prepareInMainThread(Cesium3DTilesSelection::Tile& /*tile*/,
                                                    void* pLoadThreadResult) {
    std::unique_ptr<LoadThreadGlb> glb(static_cast<LoadThreadGlb*>(pLoadThreadResult));
    if (tornDown_.load() || !glb || !glb->ok || r_ == nullptr) return nullptr;
    if (!ensure_gltf_loader(*r_)) return nullptr;

    filament::gltfio::FilamentAsset* asset = r_->sharedAssetLoader->createAsset(
        reinterpret_cast<const uint8_t*>(glb->glbBytes.data()), static_cast<uint32_t>(glb->glbBytes.size()));
    if (asset == nullptr) return nullptr;
    if (!r_->sharedResourceLoader->loadResources(asset)) {
        r_->sharedAssetLoader->destroyAsset(asset);
        return nullptr;
    }
    asset->releaseSourceData();

    // Clay re-materialization: verbatim baked ingestion sequence
    // (environment.cpp:63-84, cited not paraphrased) -- streamed buildings
    // are the SAME rendered element as baked ones (element-config
    // directive), OPAQUE the entire time they're loaded (fresh-opaque
    // convention, Global Constraints).
    filament::RenderableManager& rm = r_->engine->getRenderableManager();
    const utils::Entity* renderables = asset->getRenderableEntities();
    const size_t renderableCount = asset->getRenderableEntityCount();
    for (size_t i = 0; i < renderableCount; ++i) {
        const auto inst = rm.getInstance(renderables[i]);
        if (!inst.isValid()) continue;
        const size_t primCount = rm.getPrimitiveCount(inst);
        // VM-064 (Task 5) Step 1: the one-line gate that IS the feature --
        // gltfio's loadResources() above already loaded this tile's own
        // ubershader materials/textures; the clay remap below was
        // DISCARDING them. Google Photorealistic 3D Tiles (materials=
        // original) keeps them; every other preset (materialsOriginal_
        // false, the default) remaps exactly as before -- byte-identical.
        if (!materialsOriginal_) {
            for (size_t p = 0; p < primCount; ++p) {
                rm.setMaterialInstanceAt(inst, p, r_->buildingMaterial);
            }
        }
        rm.setCastShadows(inst, true);
        rm.setReceiveShadows(inst, true);
    }

    // Geo placement (Decision 8): one root-entity transform, no per-vertex
    // math. `glb->transform` (captured in prepareInLoadThread, NOT
    // tile.getTransform() -- see LoadThreadGlb's own comment) is this
    // content's local-to-ECEF transform, RTC_CENTER included; ecefToMap_
    // was computed once at open().
    filament::TransformManager& tm = r_->engine->getTransformManager();
    const auto tinst = tm.getInstance(asset->getRoot());
    if (tinst.isValid()) {
        tm.setTransform(tinst, to_filament_mat4(ecefToMap_ * glb->transform));
    }
    return asset;
}

void StreamRendererResources::free(Cesium3DTilesSelection::Tile& /*tile*/, void* pLoadThreadResult,
                                    void* pMainThreadResult) noexcept {
    delete static_cast<LoadThreadGlb*>(pLoadThreadResult);
    if (pMainThreadResult == nullptr) return;
    auto* asset = static_cast<filament::gltfio::FilamentAsset*>(pMainThreadResult);
    if (tornDown_.load()) {
        // Arrived after teardown()'s own bounded wait gave up: r's
        // sharedAssetLoader may already be gone by the time anyone would
        // drain this. Deliberately leaked rather than risk a
        // use-after-free -- see teardown()'s own comment.
        // ponytail: bounded-wait leak path, same ceiling kTeardownPumpBound documents.
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
    if (r_ == nullptr) return {};  // nothing actually destroyed -- report none
    for (filament::gltfio::FilamentAsset* asset : toFree) {
        r_->scene->removeEntities(asset->getEntities(), asset->getEntityCount());
        r_->sharedAssetLoader->destroyAsset(asset);
    }
    return toFree;
}

// ── StreamingEnvironmentSource ────────────────────────────────────────────
StreamingEnvironmentSource::StreamingEnvironmentSource(
    Cesium3DTilesSelection::TilesetExternals externals, int64_t asset_id, std::string ion_access_token,
    std::string root_tileset_uri, std::string fallback_baked_dir, GeoAnchor anchor,
    std::shared_ptr<CountingAssetAccessor> counting_accessor, bool materials_original)
    : asyncSystem_(externals.asyncSystem),
      anchor_(anchor),
      ecefToMap_(compute_ecef_to_map(anchor)),
      mapToEcef_(glm::inverse(ecefToMap_)),
      fallbackBakedDir_(std::move(fallback_baked_dir)),
      countingAccessor_(std::move(counting_accessor)),
      materialsOriginal_(materials_original) {
    // GltfConverters' magic-byte dispatch table (b3dm/glTF/cmpt/i3dm/pnts)
    // is empty until this is called once, process-wide -- cesium-native
    // deliberately leaves it to the embedding application (not every
    // consumer wants every content type linked in). Without it, EVERY
    // tile's content -- b3dm included -- falls through to "must be an
    // external tileset or GeoJSON" and fails to parse as JSON (found by
    // running this task's own Step 2 test with tracing enabled: cesium's
    // own TilesetJsonLoader logged "Error when parsing JSON content" for
    // every real b3dm tile in the fixture).
    static std::once_flag registerContentTypesOnce;
    std::call_once(registerContentTypesOnce, [] { Cesium3DTilesContent::registerAllTileContentTypes(); });

    // Owned here (not passed in via `externals`) so update()/teardown() can
    // reach it directly -- the SAME object also becomes
    // externals.pPrepareRendererResources below, so cesium's Tileset holds
    // the other half of this shared_ptr's ownership.
    renderResources_ = std::make_shared<StreamRendererResources>(ecefToMap_, materialsOriginal_);
    externals.pPrepareRendererResources = renderResources_;

    Cesium3DTilesSelection::TilesetOptions options;
    options.maximumScreenSpaceError = kStreamMaxSseErr;
    if (asset_id > 0 && !ion_access_token.empty()) {
        tileset_ = std::make_unique<Cesium3DTilesSelection::Tileset>(externals, asset_id, ion_access_token, options);
    } else {
        tileset_ = std::make_unique<Cesium3DTilesSelection::Tileset>(externals, root_tileset_uri, options);
    }
    viewGroup_ = &tileset_->getDefaultViewGroup();
}

StreamingEnvironmentSource::~StreamingEnvironmentSource() = default;

void StreamingEnvironmentSource::update(VisualRenderer& r, Vec3 ego_map_pos) {
    // Decision 11 (VM-063): once fallen back, every subsequent update()
    // delegates to the baked source (or is a no-op if none was configured
    // -- Decision 11's "no &fallback= given" path). One-way: never
    // re-checked against countingAccessor_ again.
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
    // VM-062 gate round 1, Finding 3: evict every just-destroyed asset from
    // inScene_ BEFORE synthesize_view_and_pump()'s reconcile loop runs --
    // otherwise a free() that lands between two ticks leaves a dangling
    // pointer in inScene_ that the next tick's drain destroys and this
    // tick's reconcile loop (stillPresent/removeEntities below) then
    // dereferences again.
    for (filament::gltfio::FilamentAsset* freed : renderResources_->drain_pending_frees()) {
        inScene_.erase(freed);
    }
    synthesize_view_and_pump(r, ego_map_pos);
}

void StreamingEnvironmentSource::fall_back(VisualRenderer& r) {
    // teardown() with fallenBack_ still false at this point tears down ONLY
    // the streamed tiles + tileset (fallbackSource_ is not yet set, so its
    // own early-teardown branch is skipped) -- see teardown()'s own
    // comment for why this ordering matters.
    teardown(r);
    fallenBack_ = true;
    if (!fallbackBakedDir_.empty()) {
        fallbackSource_ = open_baked_environment_source(fallbackBakedDir_, anchor_);
        // A failed open (bad dir/missing index.yaml) is non-fatal, same as
        // every other open_baked_environment_source() caller: fallbackSource_
        // stays null, loaded_count() reports 0, state() still reports
        // STREAMING_FALLBACK (network loss WAS declared -- the state names
        // the transition, not whether the fallback dir itself was valid).
        //
        // VM-096: a freshly opened source defaults visible -- sync it
        // to this source's OWN current visible_ (whatever the node/GUI
        // last set via set_environment_visible()) so falling back while
        // hidden doesn't pop the fallback baked chunks into view.
        if (fallbackSource_) fallbackSource_->set_visible(r, visible_);
    }
    // No &fallback= configured (fallbackBakedDir_ empty): fallbackSource_
    // stays null -- tiles simply stop appearing, state still reported
    // (Decision 11's "no automatic recovery" + "no fallback dir" paths).
}

void StreamingEnvironmentSource::synthesize_view_and_pump(VisualRenderer& r, Vec3 ego_map_pos) {
    using Cesium3DTilesSelection::ViewState;
    using Cesium3DTilesSelection::ViewUpdateResult;

    const glm::dvec4 eyeEcef4 =
        mapToEcef_ * glm::dvec4(ego_map_pos.x, ego_map_pos.y, ego_map_pos.z + kStreamViewHeightM, 1.0);
    const glm::dvec3 eyeEcef(eyeEcef4);
    // Map frame +Z is "up" by this project's own convention (ego.cpp, bowl.cpp);
    // nadir direction is straight down that same axis, expressed in ECEF.
    const glm::dvec3 downEcef = glm::normalize(glm::dvec3(mapToEcef_ * glm::dvec4(0.0, 0.0, -1.0, 0.0)));
    const glm::dvec3 northEcef = glm::normalize(glm::dvec3(mapToEcef_ * glm::dvec4(0.0, 1.0, 0.0, 0.0)));

    const ViewState view(eyeEcef, downEcef, northEcef,
                          glm::dvec2(kStreamViewportPx, kStreamViewportPx), kStreamViewFovRad,
                          kStreamViewFovRad);

    const auto now = std::chrono::steady_clock::now();
    float deltaSeconds = 0.0f;
    if (lastUpdate_.has_value()) {
        deltaSeconds = std::chrono::duration<float>(now - *lastUpdate_).count();
    }
    lastUpdate_ = now;

    const ViewUpdateResult& result = tileset_->updateViewGroup(*viewGroup_, {view}, deltaSeconds);
    tileset_->loadTiles();  // NOT optional -- without this no tile ever loads (Decision 9).
    // Runs continuations queued onto the main thread (root-tile-available,
    // prepareInMainThread, free()...) -- TilesetExternals.h's own doc says
    // this is called automatically from the OLD updateView(), but NOT from
    // updateViewGroup()/loadTiles() (Decision 9's non-deprecated pair);
    // confirmed empirically (root tile never resolved without this call).
    asyncSystem_.dispatchMainThreadTasks();

    std::unordered_map<const void*, bool> stillPresent;
    for (const auto& tilePtr : result.tilesToRenderThisFrame) {
        const Cesium3DTilesSelection::Tile* tile = tilePtr.get();
        if (tile == nullptr || !tile->isRenderContent()) continue;
        const auto* renderContent = tile->getContent().getRenderContent();
        if (renderContent == nullptr) continue;
        void* res = renderContent->getRenderResources();
        if (res == nullptr) continue;  // prepareInMainThread hasn't produced it yet
        auto* asset = static_cast<filament::gltfio::FilamentAsset*>(res);
        stillPresent[res] = true;
        // visible_ invariant (VM-096): only add if currently visible --
        // a tile that finishes loading while hidden must not pop into
        // view. It's still tracked in stillPresent/inScene_ either way, so
        // loaded_count() is unaffected and a later set_visible(true) picks
        // it up without a re-fetch.
        if (visible_ && inScene_.find(res) == inScene_.end()) {
            r.scene->addEntities(asset->getEntities(), asset->getEntityCount());
        }
    }
    for (auto it = inScene_.begin(); it != inScene_.end();) {
        if (stillPresent.find(it->first) == stillPresent.end()) {
            auto* asset = static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(it->first));
            // Only remove from the scene if it was ever added there (see
            // the visible_ guard above) -- same invariant.
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
    // VM-063 (Task 4): a fallen-back source's OWN teardown discipline
    // (BakedEnvironmentSource::teardown, or a no-op if none was ever
    // opened) runs first, every time -- fall_back() calls teardown()
    // itself BEFORE fallbackSource_ is constructed, so that first call
    // takes the tornDown_-guarded branch below (tearing down the streamed
    // tiles + tileset exactly once); every LATER teardown() call (the
    // node's normal destroy_renderer() path, or set_environment_source()
    // re-entry) finds fallbackSource_ already set and tears IT down here,
    // then returns early via the tornDown_ guard below (already true).
    if (fallbackSource_) {
        fallbackSource_->teardown(r);
        fallbackSource_.reset();
    }
    if (tornDown_) return;
    renderResources_->set_renderer(&r);

    // (1) Remove/destroy every currently-in-scene asset (like
    // BakedEnvironmentSource::teardown) and flush any already-queued frees.
    for (auto& [res, _] : inScene_) {
        auto* asset = static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(res));
        // visible_ invariant, same as the reconcile loop above: nothing to
        // remove from the scene for entries that were never added while
        // hidden.
        if (visible_) {
            r.scene->removeEntities(asset->getEntities(), asset->getEntityCount());
        }
    }
    inScene_.clear();
    renderResources_->drain_pending_frees();

    // (2) Capture the async-destruction event BEFORE destroying the
    // tileset (upstream: ~Tileset() does not join in-flight async work --
    // "these tiles will be unloaded asynchronously some time after this
    // destructor returns", Tileset.h). Then destroy it (unloads
    // synchronously as much as it can) and pump main-thread tasks in a
    // bounded loop until the event fires.
    CesiumAsync::SharedFuture<void> destructionComplete = tileset_->getAsyncDestructionCompleteEvent();
    tileset_.reset();
    renderResources_->drain_pending_frees();  // whatever ~Tileset() freed synchronously

    int pumps = 0;
    for (; pumps < kTeardownPumpBound && !destructionComplete.isReady(); ++pumps) {
        asyncSystem_.dispatchMainThreadTasks();
    }
    if (pumps >= kTeardownPumpBound && !destructionComplete.isReady()) {
        // ponytail: bounded join, kTeardownPumpBound -- hitting it means
        // cesium's async work is wedged; we leak its in-flight tiles
        // rather than hang destroy_renderer(). Not observed in this
        // fixture-scale test suite.
        ++leakedOnTeardownBound_;
    }

    // (3) Only now: any free() arriving after this point (a genuinely
    // late async completion past the bound above) is unsafe to act on --
    // r.sharedAssetLoader/sharedResourceLoader die shortly after this
    // function returns (destroy_renderer()'s own contract). One final
    // drain of whatever's already queued, using `r` while it's still
    // valid, THEN the flag goes up.
    renderResources_->drain_pending_frees();
    renderResources_->note_torn_down();
    tornDown_ = true;
}

void StreamingEnvironmentSource::set_visible(VisualRenderer& r, bool visible) {
    // Once fallen back, this source's own inScene_ is permanently empty
    // (teardown() already ran, Decision 11) -- delegate to whichever baked
    // source is standing in, same as every other fallenBack_ member
    // function here. visible_ is still recorded (not merely delegated)
    // so a hypothetical future caller reading it directly sees the truth,
    // and so a null fallbackSource_ (no &fallback= configured) doesn't
    // silently drop the request.
    if (fallenBack_) {
        visible_ = visible;
        if (fallbackSource_) fallbackSource_->set_visible(r, visible);
        return;
    }
    if (visible == visible_) return;  // no-op: matches the current state already
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

size_t StreamingEnvironmentSource::scene_membership_count() const {
    if (fallenBack_) return fallbackSource_ ? fallbackSource_->scene_membership_count() : 0;
    // visible_ invariant (VM-096): every inScene_ entry is actually
    // added to r.scene iff visible_ -- see synthesize_view_and_pump()'s
    // reconcile loop and set_visible() above.
    return visible_ ? inScene_.size() : 0;
}

EnvironmentSourceState StreamingEnvironmentSource::state() const {
    return fallenBack_ ? EnvironmentSourceState::STREAMING_FALLBACK
                        : EnvironmentSourceState::STREAMING;
}

// VM-064 (Task 5) Step 1 test-hook mirror: the first currently-in-scene
// tile's first renderable's first primitive, compared against
// r.buildingMaterial -- true in every non-original-materials preset
// (today's clay remap), false in original-materials mode. false (not a
// crash) if nothing is loaded yet -- same null-safety shape as every other
// test hook in this file.
bool StreamingEnvironmentSource::first_primitive_is_building_material(VisualRenderer& r) const {
    if (inScene_.empty()) return false;
    auto* asset = static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(inScene_.begin()->first));
    const size_t renderableCount = asset->getRenderableEntityCount();
    if (renderableCount == 0) return false;
    const utils::Entity* renderables = asset->getRenderableEntities();
    filament::RenderableManager& rm = r.engine->getRenderableManager();
    const auto inst = rm.getInstance(renderables[0]);
    if (!inst.isValid() || rm.getPrimitiveCount(inst) == 0) return false;
    return rm.getMaterialInstanceAt(inst, 0) == r.buildingMaterial;
}

}  // namespace mpviz

// ── Public factory (production ion:// path, Decision 5/6/15.6) ───────────
namespace mpviz {

std::unique_ptr<EnvironmentSource> open_streaming_environment_source(const std::string& ion_spec,
                                                                      GeoAnchor anchor) {
    const std::optional<IonSpec> spec = parse_ion_spec(ion_spec);
    if (!spec) return nullptr;

    // Decision 6: read once at open time, by NAME only -- never a URI
    // component, never logged.
    const char* token = std::getenv("CESIUM_ION_TOKEN");
    if (token == nullptr || token[0] == '\0') return nullptr;  // non-fatal, caller WARNs

    const std::string cacheDir = spec->cache_dir.empty() ? default_cache_dir() : spec->cache_dir;

    // CesiumCurl -> CountingAssetAccessor -> CachingAssetAccessor(SqliteCache)
    // (Decision 10/11). The ion handshake itself is NOT here: the Tileset's
    // own ion constructor (below) performs it and builds the session-token
    // refresh accessor internally (Decision 15.6) -- nothing hand-rolled.
    auto curl = std::make_shared<CesiumCurl::CurlAssetAccessor>();
    CesiumAsync::AsyncSystem asyncSystem(std::make_shared<SimpleTaskProcessor>());
    std::shared_ptr<CountingAssetAccessor> counting;
    Cesium3DTilesSelection::TilesetExternals externals =
        build_externals(curl, asyncSystem, cacheDir, spec->max_cache_items, &counting);

    return std::make_unique<StreamingEnvironmentSource>(externals, spec->asset_id, std::string(token),
                                                          std::string(), spec->fallback_dir, anchor,
                                                          std::move(counting), spec->materials_original);
}

}  // namespace mpviz

// ── Test-only fixture hooks (Decision 13; environment_test_hooks.hpp) ────
// Every function here is DEFINED in this, the one C++20 TU -- the header
// they implement is C++17-safe and cesium-free (test TUs never see
// FileFixtureAssetAccessor/StreamingEnvironmentSource, only the opaque
// FixtureStreamHandle + bool/pointer-returning functions below).
namespace mpviz::testing {

struct FixtureStreamHandle {
    std::shared_ptr<std::atomic<bool>> killed;
};

namespace {

// VM-062 gate round 1, Finding 4: a per-PROCESS cache dir under the system
// temp path, NOT <fixture_dir>/.test_cache -- the old path wrote into the
// committed fixture source tree and silently shared cache state across
// separate ctest runs (a leftover cache from an earlier run made
// DiskCacheServesTilesWithNetworkDead pass regardless of whether THIS run's
// disk-cache write path actually worked). `static` gives every call in this
// process the SAME path -- Step 4's proof needs its two
// install_fixture_streaming_source_with_fallback() calls to share one
// cache within a run -- while a fresh process (a new ctest invocation) gets
// a fresh, genuinely cold directory.
std::string test_cache_dir() {
    // remove_all on first use: a reused pid must not inherit an earlier
    // run's warm cache (that would let DiskCacheServesTilesWithNetworkDead
    // pass with a broken cache-write path -- the exact spurious-pass mode
    // gate round 1 finding 4 closed). One-time per process, like the path.
    static const std::string dir = [] {
        const std::string d = (std::filesystem::temp_directory_path() /
                               ("mpviz-stream-test-cache-" + std::to_string(::getpid())))
                                  .string();
        std::error_code ec;
        std::filesystem::remove_all(d, ec);
        return d;
    }();
    return dir;
}

// Shared by both fixture install hooks: composes the SAME accessor stack
// the production path uses (Decision 10/11's CachingAssetAccessor(SqliteCache)
// over a CountingAssetAccessor) but rooted at the fixture's own
// FileFixtureAssetAccessor instead of CesiumCurl -- Step 4's disk-cache
// proof depends on this being the real stack, not a bare fixture accessor.
// `killed` non-null makes the accessor honor kill_fixture_network().
std::unique_ptr<mpviz::EnvironmentSource> make_fixture_source(
    const char* fixture_dir, const char* fallback_baked_dir, mpviz::GeoAnchor anchor,
    std::shared_ptr<std::atomic<bool>> killed, bool materials_original) {
    if (fixture_dir == nullptr) return nullptr;
    auto fileAccessor = std::make_shared<mpviz::FileFixtureAssetAccessor>(std::move(killed));
    CesiumAsync::AsyncSystem asyncSystem(std::make_shared<mpviz::SimpleTaskProcessor>());
    const std::string cacheDir = test_cache_dir();
    std::shared_ptr<mpviz::CountingAssetAccessor> counting;
    Cesium3DTilesSelection::TilesetExternals externals = mpviz::build_externals(
        fileAccessor, asyncSystem, cacheDir, mpviz::kDefaultMaxCacheItems, &counting);
    const std::string tilesetUri = std::string("file://") + fixture_dir + "/tileset.json";
    return std::make_unique<mpviz::StreamingEnvironmentSource>(
        externals, /*asset_id=*/0, /*ion_access_token=*/std::string(),
        tilesetUri, fallback_baked_dir ? std::string(fallback_baked_dir) : std::string(), anchor,
        std::move(counting), materials_original);
}

}  // namespace

bool install_fixture_streaming_source(mpviz::VisualRenderer* r, const char* fixture_dir,
                                       mpviz::GeoAnchor anchor, bool materials_original) {
    if (r == nullptr) return false;
    // Teardown-THEN-construct (not build-then-swap, unlike
    // set_environment_source()'s general baked/streaming dispatch): two
    // sqlite-backed accessor stacks pointed at the same on-disk cache file
    // (Step 4's own disk-cache proof does exactly this, by design) cannot
    // both be open at once -- SQLite's single-writer file lock rejects the
    // second connection with "database is locked" if the first is still
    // live. These test-only hooks aren't bound by set_environment_source's
    // re-entrant-swap contract, so they tear down first.
    if (r->environmentSource) r->environmentSource->teardown(*r);
    auto source = make_fixture_source(fixture_dir, /*fallback_baked_dir=*/nullptr, anchor,
                                       /*killed=*/nullptr, materials_original);
    if (!source) {
        r->environmentSource.reset();
        return false;
    }
    r->environmentSource = std::move(source);
    return true;
}

FixtureStreamHandle* install_fixture_streaming_source_with_fallback(mpviz::VisualRenderer* r,
                                                                     const char* fixture_dir,
                                                                     const char* fallback_baked_dir,
                                                                     mpviz::GeoAnchor anchor) {
    if (r == nullptr) return nullptr;
    if (r->environmentSource) r->environmentSource->teardown(*r);  // see install_fixture_streaming_source's comment
    auto killed = std::make_shared<std::atomic<bool>>(false);
    auto source = make_fixture_source(fixture_dir, fallback_baked_dir, anchor, killed,
                                       /*materials_original=*/false);
    if (!source) {
        r->environmentSource.reset();
        return nullptr;
    }
    r->environmentSource = std::move(source);
    // ponytail: test-only handle, intentionally leaked -- this process is a
    // short-lived gtest binary, and the handle is a single shared_ptr<atomic<bool>>.
    return new FixtureStreamHandle{std::move(killed)};
}

void kill_fixture_network(FixtureStreamHandle* handle) {
    if (handle && handle->killed) handle->killed->store(true);
}

void revive_fixture_network(FixtureStreamHandle* handle) {
    if (handle && handle->killed) handle->killed->store(false);
}

// VM-064 (Task 5) Step 0: mirrors the installed streaming source's own
// materials_original flag. false on null r, no installed source, or a
// non-streaming source (BakedEnvironmentSource) -- dynamic_cast is safe and
// cheap here (this whole namespace lives in the one C++20 TU that has
// StreamingEnvironmentSource's complete definition, Decision 3).
bool environment_stream_materials_original(mpviz::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return false;
    auto* stream = dynamic_cast<mpviz::StreamingEnvironmentSource*>(r->environmentSource.get());
    return stream != nullptr && stream->materials_original();
}

// VM-064 gate round 1 finding: exercises the REAL parser (parse_ion_spec(),
// anonymous namespace above), unlike the hook above which only reads back an
// already-installed source's flag -- the two Step 0 tests that reach
// materials_original both go through install_fixture_streaming_source(),
// which never calls parse_ion_spec() at all.
bool environment_stream_parse_materials_original(const char* ion_spec) {
    const std::optional<IonSpec> spec = parse_ion_spec(ion_spec ? ion_spec : "");
    return spec.has_value() && spec->materials_original;
}

// VM-064 Step 1: see StreamingEnvironmentSource::first_primitive_is_building_material()'s
// own comment. false on the same null/non-streaming conditions as the hook
// above, or if nothing has loaded yet.
bool environment_stream_first_primitive_is_clay(mpviz::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return false;
    auto* stream = dynamic_cast<mpviz::StreamingEnvironmentSource*>(r->environmentSource.get());
    return stream != nullptr && stream->first_primitive_is_building_material(*r);
}

bool ecef_to_map_probe(double origin_lat_deg, double origin_lon_deg, double heading_rad,
                        double lat_deg, double lon_deg, double alt_m, double* out_x, double* out_y,
                        double* out_z) {
    const mpviz::GeoAnchor anchor{origin_lat_deg, origin_lon_deg, heading_rad};
    const glm::dmat4 ecefToMap = mpviz::compute_ecef_to_map(anchor);
    const CesiumGeospatial::Cartographic carto =
        CesiumGeospatial::Cartographic::fromDegrees(lon_deg, lat_deg, alt_m);
    const glm::dvec3 ecef = CesiumGeospatial::Ellipsoid::WGS84.cartographicToCartesian(carto);
    const glm::dvec4 mapPt = ecefToMap * glm::dvec4(ecef, 1.0);
    if (out_x) *out_x = mapPt.x;
    if (out_y) *out_y = mapPt.y;
    if (out_z) *out_z = mapPt.z;
    return true;
}

}  // namespace mpviz::testing
