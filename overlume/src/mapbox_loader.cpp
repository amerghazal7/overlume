// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_loader.hpp"

#include "environment_stream.hpp"
#include "mapbox_extrude.hpp"
#include "mapbox_landmarks.hpp"
#include "mapbox_mvt.hpp"
#include "mapbox_test_hooks.hpp"
#include "mapbox_tile_frame.hpp"
#include "renderer_internal.hpp"

#include <Cesium3DTilesSelection/BoundingVolume.h>
#include <Cesium3DTilesSelection/Tile.h>
#include <Cesium3DTilesSelection/TileLoadResult.h>
#include <Cesium3DTilesSelection/TilesetContentLoader.h>
#include <CesiumAsync/IAssetRequest.h>
#include <CesiumAsync/IAssetResponse.h>
#include <CesiumCurl/CurlAssetAccessor.h>
#include <CesiumGeometry/QuadtreeTileID.h>
#include <CesiumGeospatial/BoundingRegion.h>
#include <CesiumGeospatial/Ellipsoid.h>
#include <CesiumGeospatial/GlobeRectangle.h>
#include <CesiumGltf/Model.h>
#include <CesiumUtility/Gzip.h>

#include <glm/gtc/type_ptr.hpp>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <limits>
#include <mutex>
#include <set>
#include <span>
#include <unordered_map>
#include <variant>

namespace overlume::mapbox {

namespace {

// Fully consumed, finite and > 0, as parse_ion_spec reads `radius` and `brightness`.
bool parse_positive(const std::string& val, double& out) {
    try {
        size_t used = 0;
        const double v = std::stod(val, &used);
        if (used != val.size() || !std::isfinite(v) || v <= 0.0) return false;
        out = v;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

}

MapboxSpecResult parse_mapbox_spec(const std::string& spec) {
    const size_t q = spec.find('?');
    const std::string layers = spec.substr(0, q);
    if (layers != "buildings") {
        return {std::nullopt,
                "mapbox:// supports only the layer list 'buildings', not '" + layers + "'", ""};
    }
    MapboxSpec out;
    std::string warning;
    const std::string rest = q == std::string::npos ? std::string() : spec.substr(q + 1);
    size_t pos = 0;
    while (pos < rest.size()) {
        const size_t amp = rest.find('&', pos);
        const std::string kv = rest.substr(pos, amp == std::string::npos ? amp : amp - pos);
        const size_t eq = kv.find('=');
        if (eq != std::string::npos) {
            const std::string key = kv.substr(0, eq);
            const std::string val = kv.substr(eq + 1);
            // Every other key, the ion-only ones included, is ignored.
            if (key == "materials") {
                out.materials_original = val == "original";
                if (val != "original" && val != "clay")
                    warning = "mapbox:// materials='" + val + "' is unknown, treated as clay";
            } else if (key == "radius" || key == "brightness") {
                if (!parse_positive(val, key == "radius" ? out.radius_m : out.brightness))
                    return {std::nullopt,
                            "mapbox:// " + key + "='" + val + "' is not a finite number > 0", ""};
            } else if (key == "fallback") {
                // Empty means no fallback, as when absent. It also keeps the node's composer from
                // appending its own, since that skips any URI already naming `fallback=`.
                out.fallback_dir = val;
            } else if (key == "cache" && val != "off") {
                return {std::nullopt,
                        "mapbox:// cache='" + val +
                            "' is not 'off': Mapbox tiles are never written to disk",
                        ""};
            }
        }
        if (amp == std::string::npos) break;
        pos = amp + 1;
    }
    return {out, "", warning};
}

}

namespace overlume::mapbox {

namespace {

using Cesium3DTilesSelection::Tile;
using Cesium3DTilesSelection::TileChildrenResult;
using Cesium3DTilesSelection::TileEmptyContent;
using Cesium3DTilesSelection::TileLoadInput;
using Cesium3DTilesSelection::TileLoadResult;
using Cesium3DTilesSelection::TileLoadResultState;

constexpr uint32_t kMaxAttempts = 3;
// The wait before attempt 2, from attempt 1's start; doubled before attempt 3.
constexpr double kRetryWaitS = 2.0;
// Tile bounds span the anchor height -10 m to +1000 m until a built tile reports its real top.
constexpr double kRegionBelowM = 10.0;
constexpr double kRegionAboveM = 1000.0;
constexpr char kStreets[] = "streets-v8 footprints";
constexpr char kLandmarks[] = "3D landmarks";
constexpr size_t kLandmarkIndex = 16;  // after the 16 squares in a tile's requests

std::string footprint_url(uint32_t x16, uint32_t y16) {
    return "https://api.mapbox.com/v4/mapbox.mapbox-streets-v8/16/" + std::to_string(x16) + "/" +
           std::to_string(y16) + ".vector.pbf";
}

std::string landmark_url(uint32_t x14, uint32_t y14) {
    return "https://api.mapbox.com/3dtiles/v1/mapbox.mapbox-3dbuildings-v1-meshopt/14/" +
           std::to_string(x14) + "/" + std::to_string(y14) + ".glb";
}

// A shell export can leave spaces or a newline around the token; a request URL must not carry them.
std::string trim_ascii_whitespace(const std::string& s) {
    constexpr char kWhitespace[] = " \t\n\v\f\r";
    const size_t first = s.find_first_not_of(kWhitespace);
    if (first == std::string::npos) return {};
    return s.substr(first, s.find_last_not_of(kWhitespace) - first + 1);
}

// Fixed when the source opens. Async work holds this and LoaderState by shared_ptr, never the
// loader itself, so switching the source away mid-build stays safe.
struct LoaderConfig {
    std::string token;  // goes into request URLs and nowhere else
    bool materials_original = false;
    double origin_height_m = 0.0;
};

struct LoaderState {
    std::atomic<uint64_t> tiles_built{0};
    std::atomic<uint64_t> landmark_nodes{0};
    std::atomic<uint64_t> extruded_features{0};
    std::atomic<uint64_t> conflated_features{0};
    std::atomic<uint64_t> retries{0};
    std::atomic<uint64_t> wall_quads{0};
    std::atomic<double> max_build_ms{0.0};
    std::atomic<double> max_tile_top_m{0.0};
    std::mutex logged_mutex;
    std::set<std::string> logged;

    // At most one log line per endpoint and status, or failure kind, per source.
    bool first_time(const std::string& key) {
        std::lock_guard<std::mutex> lock(logged_mutex);
        return logged.insert(key).second;
    }

    void note_build_ms(double ms) {
        double seen = max_build_ms.load();
        while (ms > seen && !max_build_ms.compare_exchange_weak(seen, ms)) {
        }
    }

    void note_top_m(double top) {
        double seen = max_tile_top_m.load();
        while (top > seen && !max_tile_top_m.compare_exchange_weak(seen, top)) {
        }
    }
};

std::mutex g_lastStateMutex;
std::shared_ptr<LoaderState> g_lastState;  // the latest source's, for mapbox_last_source_stats()

// The retry wait's clock in seconds: the steady clock, or where a test froze it
// (mapbox_set_test_clock).
std::atomic<double> g_testClockS{-1.0};

double clock_s() {
    const double frozen = g_testClockS.load();
    if (frozen >= 0.0) return frozen;
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

struct Fetched {
    uint16_t status = 0;
    std::vector<std::byte> body;  // kept for 200 only
    bool transport_error = false;
};

enum class Outcome { Body, Empty, Auth, Transient, Other };

Outcome classify(const Fetched& f) {
    if (f.transport_error) return Outcome::Transient;
    if (f.status == 200) return Outcome::Body;
    if (f.status == 204 || f.status == 404) return Outcome::Empty;
    if (f.status == 401 || f.status == 403) return Outcome::Auth;
    if (f.status == 429 || f.status >= 500) return Outcome::Transient;
    return Outcome::Other;
}

// Never rejects: AsyncSystem::all would reject too, and cesium fails a tile whose load rejects
// for good. The transport error's text names the URL, token included, so it is dropped here.
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

Cesium3DTilesSelection::BoundingVolume region_of(const TileId& t, double origin_height_m,
                                                 double top_m,
                                                 const CesiumGeospatial::Ellipsoid& ellipsoid) {
    const LonLatRect b = tile_bounds(t);
    return CesiumGeospatial::BoundingRegion(
        CesiumGeospatial::GlobeRectangle::fromDegrees(b.west, b.south, b.east, b.north),
        origin_height_m - kRegionBelowM, origin_height_m + top_m, ellipsoid);
}

// A Success result for content in the tile's Z-up frame. pCompletedRequest stays null: cesium
// would copy that request's URL, token included, into the model.
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

std::vector<TilePolygon> to_tile_polygons(const std::vector<MvtPolygon>& polygons) {
    const auto ring = [](const std::vector<MvtPoint>& points) {
        TileRing out;
        out.reserve(points.size());
        for (const MvtPoint& p : points) {
            out.push_back({static_cast<double>(p.x), static_cast<double>(p.y)});
        }
        return out;
    };
    std::vector<TilePolygon> out;
    out.reserve(polygons.size());
    for (const MvtPolygon& polygon : polygons) {
        TilePolygon& t = out.emplace_back();
        t.outer = ring(polygon.outer);
        for (const std::vector<MvtPoint>& hole : polygon.holes) t.holes.push_back(ring(hole));
    }
    return out;
}

// Appends the extrusions as one mesh (positions, normals and uint32 indices in a new buffer) with
// Mapbox Standard's building colour, under an identity node in the model's scene.
void append_extrusions(CesiumGltf::Model& model, const TriangleMesh& mesh) {
    using CesiumGltf::Accessor;
    using CesiumGltf::BufferView;
    const size_t vertexBytes = mesh.positions.size() * sizeof(float);
    const size_t indexBytes = mesh.indices.size() * sizeof(uint32_t);
    CesiumGltf::Buffer& buffer = model.buffers.emplace_back();
    buffer.cesium.data.resize(2 * vertexBytes + indexBytes);
    std::memcpy(buffer.cesium.data.data(), mesh.positions.data(), vertexBytes);
    std::memcpy(buffer.cesium.data.data() + vertexBytes, mesh.normals.data(), vertexBytes);
    std::memcpy(buffer.cesium.data.data() + 2 * vertexBytes, mesh.indices.data(), indexBytes);
    buffer.byteLength = static_cast<int64_t>(buffer.cesium.data.size());
    const int32_t bufferIndex = static_cast<int32_t>(model.buffers.size() - 1);

    const auto add_accessor = [&](size_t offset, size_t length, int32_t target,
                                  int32_t componentType, const std::string& type, size_t count) {
        BufferView& view = model.bufferViews.emplace_back();
        view.buffer = bufferIndex;
        view.byteOffset = static_cast<int64_t>(offset);
        view.byteLength = static_cast<int64_t>(length);
        view.target = target;
        Accessor& accessor = model.accessors.emplace_back();
        accessor.bufferView = static_cast<int32_t>(model.bufferViews.size() - 1);
        accessor.componentType = componentType;
        accessor.type = type;
        accessor.count = static_cast<int64_t>(count);
        return static_cast<int32_t>(model.accessors.size() - 1);
    };
    const size_t vertexCount = mesh.positions.size() / 3;
    const int32_t position =
        add_accessor(0, vertexBytes, BufferView::Target::ARRAY_BUFFER,
                     Accessor::ComponentType::FLOAT, Accessor::Type::VEC3, vertexCount);
    const int32_t normal =
        add_accessor(vertexBytes, vertexBytes, BufferView::Target::ARRAY_BUFFER,
                     Accessor::ComponentType::FLOAT, Accessor::Type::VEC3, vertexCount);
    const int32_t indices = add_accessor(
        2 * vertexBytes, indexBytes, BufferView::Target::ELEMENT_ARRAY_BUFFER,
        Accessor::ComponentType::UNSIGNED_INT, Accessor::Type::SCALAR, mesh.indices.size());

    // glTF requires POSITION bounds.
    std::vector<double> lo(3, std::numeric_limits<double>::max());
    std::vector<double> hi(3, std::numeric_limits<double>::lowest());
    for (size_t i = 0; i < mesh.positions.size(); ++i) {
        lo[i % 3] = std::min(lo[i % 3], static_cast<double>(mesh.positions[i]));
        hi[i % 3] = std::max(hi[i % 3], static_cast<double>(mesh.positions[i]));
    }
    model.accessors[static_cast<size_t>(position)].min = lo;
    model.accessors[static_cast<size_t>(position)].max = hi;

    // Clay mode replaces this material at upload; materials=original shows it.
    CesiumGltf::MaterialPBRMetallicRoughness& pbr =
        model.materials.emplace_back().pbrMetallicRoughness.emplace();
    pbr.baseColorFactor = {0.912, 0.869, 0.787, 1.0};  // Standard's hsl(40, 43%, 93%), linear
    pbr.metallicFactor = 0.0;
    pbr.roughnessFactor = 1.0;

    CesiumGltf::MeshPrimitive& primitive = model.meshes.emplace_back().primitives.emplace_back();
    primitive.attributes["POSITION"] = position;
    primitive.attributes["NORMAL"] = normal;
    primitive.indices = indices;
    primitive.material = static_cast<int32_t>(model.materials.size() - 1);
    primitive.mode = CesiumGltf::MeshPrimitive::Mode::TRIANGLES;

    model.nodes.emplace_back().mesh = static_cast<int32_t>(model.meshes.size() - 1);
    if (model.scenes.empty()) model.scenes.emplace_back();
    if (model.scene < 0) model.scene = 0;
    model.scenes[static_cast<size_t>(model.scene)].nodes.push_back(
        static_cast<int32_t>(model.nodes.size() - 1));
    if (model.asset.version.empty()) model.asset.version = "2.0";
}

// One line per endpoint and status per source; empty answers (204, 404) are normal and silent.
// Log lines name tiles, never URLs.
void note_status(LoaderState& st, spdlog::logger& log, const char* endpoint, const Fetched& f,
                 const TileId& t14) {
    const Outcome outcome = classify(f);
    if (outcome == Outcome::Body || outcome == Outcome::Empty) return;
    if (!st.first_time(std::string(endpoint) + " " + std::to_string(f.status))) return;
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

// The building layer of one z16 square; ok = false when the answer holds none (an empty answer, a
// refusal or garbage, each kind logged once per source).
MvtBuildingLayer read_square(LoaderState& st, spdlog::logger& log, const Fetched& f,
                             const TileId& square, const TileId& t14) {
    if (classify(f) != Outcome::Body) {
        note_status(st, log, kStreets, f, t14);
        return {};
    }
    // curl inflates a gzip Content-Encoding itself; a body that still is gzip is inflated here.
    std::span<const std::byte> bytes(f.body);
    std::vector<std::byte> inflated;
    if (CesiumUtility::isGzip(bytes)) {
        if (!CesiumUtility::gunzip(bytes, inflated)) {
            if (st.first_time("mvt-gzip")) {
                log.warn("mapbox: square 16/{}/{} is not valid gzip; skipped", square.x, square.y);
            }
            return {};
        }
        bytes = std::span<const std::byte>(inflated);
    }
    MvtBuildingLayer layer =
        decode_mvt_building_layer(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size());
    if (!layer.ok && st.first_time("mvt-decode")) {
        log.warn("mapbox: square 16/{}/{} is not a vector tile; skipped", square.x, square.y);
    }
    return layer;
}

// Highest POSITION z over a landmark model's meshes: local metres above the ground.
double landmark_top_m(const CesiumGltf::Model& model) {
    double top = 0.0;
    for (const CesiumGltf::Mesh& mesh : model.meshes) {
        for (const CesiumGltf::MeshPrimitive& primitive : mesh.primitives) {
            const auto it = primitive.attributes.find("POSITION");
            if (it == primitive.attributes.end()) continue;
            const CesiumGltf::Accessor* position =
                CesiumGltf::Model::getSafe(&model.accessors, it->second);
            if (position != nullptr && position->max.size() == 3) {
                top = std::max(top, position->max[2]);
            }
        }
    }
    return top;
}

// Worker thread: one z14 tile from its answers, in request order.
TileLoadResult build_tile(const LoaderConfig& cfg, LoaderState& st, const TileId& t14,
                          uint32_t attempt, std::atomic<bool>& settled,
                          const std::vector<Fetched>& fetched,
                          const std::shared_ptr<CesiumAsync::IAssetAccessor>& accessor,
                          spdlog::logger& log, const CesiumGeospatial::Ellipsoid& ellipsoid) {
    const auto started = std::chrono::steady_clock::now();
    const bool transient = std::any_of(fetched.begin(), fetched.end(), [](const Fetched& f) {
        return classify(f) == Outcome::Transient;
    });
    if (transient && attempt < kMaxAttempts) {
        ++st.retries;
        return TileLoadResult::createRetryLaterResult(accessor, nullptr);
    }
    settled = true;  // the next load of this tile starts counting attempts afresh

    const TileFrame frame = make_tile_frame(t14, cfg.origin_height_m);
    const std::array<TileId, 16> squares = footprint_tiles_of(t14);
    // Landmarks first: their footprints decide which extrusions to drop. An unreadable GLB costs
    // the landmarks of this tile, never its extrusions.
    LandmarkTile landmarks;
    const Fetched& glb = fetched[kLandmarkIndex];
    if (classify(glb) == Outcome::Body) {
        landmarks = read_landmark_tile(glb.body, frame, cfg.materials_original);
        if (!landmarks.ok && st.first_time("glb")) {
            log.warn("mapbox: landmarks of tile 14/{}/{} skipped: {}", t14.x, t14.y,
                     landmarks.error);
        }
        if (landmarks.ok && landmarks.nodes_skipped > 0 && st.first_time("glb-node")) {
            log.warn("mapbox: tile 14/{}/{}: {} landmark node(s) with an unexpected matrix skipped",
                     t14.x, t14.y, landmarks.nodes_skipped);
        }
    } else {
        note_status(st, log, kLandmarks, glb, t14);
    }
    TriangleMesh mesh;
    uint64_t extruded = 0;
    uint64_t conflated = 0;
    uint64_t walls = 0;
    double top = landmarks.ok ? landmark_top_m(landmarks.model) : 0.0;
    for (size_t i = 0; i < squares.size(); ++i) {
        const TileId& square = squares[i];
        const MvtBuildingLayer layer = read_square(st, log, fetched[i], square, t14);
        if (!layer.ok) continue;
        const double extent = static_cast<double>(layer.extent);
        const ProjectFn project = [&frame, &square, extent](double u, double v, double z) {
            return project_to_frame(frame, square, u, v, extent, z);
        };
        for (const MvtBuilding& b : layer.buildings) {
            const double base = b.min_height.value_or(0.0);
            if (!passes_standard_3d_filter(b) || !b.height || !std::isfinite(*b.height) ||
                !std::isfinite(base) || *b.height <= base) {
                continue;
            }
            const std::vector<TilePolygon> polygons = to_tile_polygons(b.polygons);
            // A landmark model replaces the extrusion whose unclipped footprint centroid it
            // covers, as GL JS does. read_landmark_tile returns an empty tile on every failure,
            // which is what the tests pin; this ok check and the merge's are belt and braces.
            if (landmarks.ok) {
                if (const std::optional<std::array<double, 2>> c = footprint_centroid(polygons)) {
                    const std::array<double, 3> p = project((*c)[0], (*c)[1], 0.0);
                    if (inside_any_footprint(landmarks.footprint_triangles, p[0], p[1])) {
                        ++conflated;
                        continue;
                    }
                }
            }
            // Clipping to the square's own extent drops the tile buffer, so the copies of a
            // building in neighbouring squares meet without doubled walls.
            const ExtrudeStats es =
                extrude_polygons(polygons, extent, base, *b.height, project, mesh);
            if (es.polygons_built == 0) continue;
            ++extruded;
            walls += es.wall_quads;
            top = std::max(top, *b.height);
        }
    }
    st.extruded_features += extruded;
    st.conflated_features += conflated;
    st.wall_quads += walls;
    st.note_build_ms(
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
            .count());

    const bool haveLandmarks = landmarks.ok && landmarks.buildings_kept > 0;
    if (mesh.indices.empty() && !haveLandmarks) {
        return success(TileEmptyContent{}, std::nullopt, accessor, ellipsoid);
    }
    // The landmark model already holds its nodes in scene 0; the extrusions join it there.
    CesiumGltf::Model model = haveLandmarks ? std::move(landmarks.model) : CesiumGltf::Model();
    if (!mesh.indices.empty()) append_extrusions(model, mesh);
    if (haveLandmarks) st.landmark_nodes += landmarks.buildings_kept;
    ++st.tiles_built;
    st.note_top_m(top);
    return success(std::move(model), region_of(t14, cfg.origin_height_m, top, ellipsoid), accessor,
                   ellipsoid);
}

// Web Mercator XYZ as a cesium quadtree: levels 0-13 are empty and always refined, and each
// level-14 tile carries the buildings. Ids carry Mapbox's XYZ y (north-down); only this loader
// reads them.
class MapboxBuildingsLoader final : public Cesium3DTilesSelection::TilesetContentLoader {
public:
    MapboxBuildingsLoader(std::shared_ptr<const LoaderConfig> config,
                          std::shared_ptr<LoaderState> state)
        : config_(std::move(config)), state_(std::move(state)) {}

    CesiumAsync::Future<TileLoadResult> loadTileContent(const TileLoadInput& input) override {
        const auto* id = std::get_if<CesiumGeometry::QuadtreeTileID>(&input.tile.getTileID());
        if (id == nullptr || id->level != kLandmarkZoom) {
            // Levels 0-13 hold nothing; cesium may unload an empty tile and ask for it again.
            return input.asyncSystem.createResolvedFuture(
                success(TileEmptyContent{}, std::nullopt, input.pAssetAccessor, input.ellipsoid));
        }
        const TileId t14{kLandmarkZoom, id->x, id->y};
        Attempt& a = attempts_[(uint64_t{id->x} << 32) | id->y];
        if (!a.settled || a.settled->load()) {
            a = Attempt{0, 0.0, std::make_shared<std::atomic<bool>>(false)};
        }
        // Cesium asks again for a RetryLater tile on its very next selection. Until the wait is
        // over the answer is RetryLater again: no request, no attempt, no retries count.
        const double now = clock_s();
        if (a.count > 0 && now < a.not_before_s) {
            return input.asyncSystem.createResolvedFuture(
                TileLoadResult::createRetryLaterResult(input.pAssetAccessor, nullptr));
        }
        const uint32_t attempt = ++a.count;
        // ponytail: the wait runs from this attempt's start, so the tile's attempt state stays
        // main-thread only; an attempt slower than its wait retries at once, and a tile that
        // leaves the radius mid-wait resumes its old count when it is back (fewer attempts,
        // never more).
        a.not_before_s = now + kRetryWaitS * static_cast<double>(1u << (attempt - 1));

        // TileLoadInput is a stack object of references: the continuation copies what it needs.
        const std::string query = "?access_token=" + config_->token;
        std::vector<CesiumAsync::Future<Fetched>> requests;
        for (const TileId& square : footprint_tiles_of(t14)) {
            requests.push_back(fetch(input.asyncSystem, input.pAssetAccessor,
                                     footprint_url(square.x, square.y) + query));
        }
        requests.push_back(fetch(input.asyncSystem, input.pAssetAccessor,
                                 landmark_url(t14.x, t14.y) + query));  // kLandmarkIndex
        return input.asyncSystem.all(std::move(requests))
            .thenInWorkerThread([config = config_, state = state_, t14, attempt,
                                 settled = a.settled, accessor = input.pAssetAccessor,
                                 logger = input.pLogger,
                                 ellipsoid = input.ellipsoid](std::vector<Fetched>&& fetched) {
                return build_tile(*config, *state, t14, attempt, *settled, fetched, accessor,
                                  *logger, ellipsoid);
            });
    }

    TileChildrenResult createTileChildren(const Tile& tile,
                                          const CesiumGeospatial::Ellipsoid& ellipsoid) override {
        const auto* id = std::get_if<CesiumGeometry::QuadtreeTileID>(&tile.getTileID());
        if (id == nullptr || id->level >= kLandmarkZoom) return {{}, TileLoadResultState::Success};
        std::vector<Tile> children;
        children.reserve(4);
        for (uint32_t row = 0; row < 2; ++row) {
            for (uint32_t col = 0; col < 2; ++col) {
                const TileId t{id->level + 1, 2 * id->x + col, 2 * id->y + row};
                const CesiumGeometry::QuadtreeTileID childId(t.z, t.x, t.y);
                const Cesium3DTilesSelection::BoundingVolume region =
                    region_of(t, config_->origin_height_m, kRegionAboveM, ellipsoid);
                if (t.z < kLandmarkZoom) {
                    Tile& child = children.emplace_back(this, childId, TileEmptyContent{});
                    child.setUnconditionallyRefine();
                    child.setBoundingVolume(region);
                    continue;
                }
                // The transform is absolute: the z14 tile's east-north-up frame, in double.
                Tile& child = children.emplace_back(this, childId);
                child.setTransform(glm::make_mat4(
                    make_tile_frame(t, config_->origin_height_m).frame_to_ecef.data()));
                child.setGeometricError(0.0);
                child.setBoundingVolume(region);
            }
        }
        return {std::move(children), TileLoadResultState::Success};
    }

private:
    struct Attempt {
        uint32_t count = 0;
        double not_before_s = 0.0;  // clock_s() before which the next attempt may not start
        std::shared_ptr<std::atomic<bool>> settled;  // set by the build that stops retrying
    };

    std::shared_ptr<const LoaderConfig> config_;
    std::shared_ptr<LoaderState> state_;
    // Main thread only: cesium calls loadTileContent there. ponytail: one entry per z14 tile ever
    // loaded; prune settled entries if drives ever span thousands of tiles.
    std::unordered_map<uint64_t, Attempt> attempts_;
};

// The root and its single z0 child cover the Mercator world (tile_bounds of 0/0/0 is
// WebMercatorProjection::MAXIMUM_GLOBE_RECTANGLE), as LayerJsonTerrainLoader sets up EPSG:3857.
std::unique_ptr<Tile> make_root(MapboxBuildingsLoader& loader, double origin_height_m) {
    auto root =
        std::make_unique<Tile>(&loader, Cesium3DTilesSelection::TileID(), TileEmptyContent{});
    root->setUnconditionallyRefine();
    root->setBoundingVolume(region_of(TileId{0, 0, 0}, origin_height_m, kRegionAboveM,
                                      CesiumGeospatial::Ellipsoid::WGS84));
    std::vector<Tile> children;
    Tile& world =
        children.emplace_back(&loader, CesiumGeometry::QuadtreeTileID(0, 0, 0), TileEmptyContent{});
    world.setUnconditionallyRefine();
    world.setBoundingVolume(root->getBoundingVolume());
    root->createChildTiles(std::move(children));
    return root;
}

// Serves open_mapbox_environment_source (curl, MAPBOX_TOKEN) and install_fixture_mapbox_source
// (a fixture network, a given token) alike. A null `base` means curl.
std::unique_ptr<EnvironmentSource> open_mapbox_source(
    const std::string& mapbox_spec, GeoAnchor anchor,
    std::shared_ptr<CesiumAsync::IAssetAccessor> base, const std::string& raw_token) {
    const std::shared_ptr<spdlog::logger> log = make_redacting_logger();
    const MapboxSpecResult parsed = parse_mapbox_spec(mapbox_spec);
    if (!parsed.spec) {
        log->error("mapbox://{} refused: {}", mapbox_spec, parsed.error);
        return nullptr;
    }
    if (!parsed.warning.empty()) log->warn("mapbox://{}: {}", mapbox_spec, parsed.warning);
    std::string token = trim_ascii_whitespace(raw_token);
    if (token.empty()) {
        log->error("mapbox:// needs MAPBOX_TOKEN in the environment; the source was not opened");
        return nullptr;
    }
    const MapboxSpec& spec = *parsed.spec;
    if (!base) base = std::make_shared<CesiumCurl::CurlAssetAccessor>();
    // ponytail: two worker threads, the default the ion source uses too. Curl blocks one per
    // request in flight, so a tile's requests go two at a time; pass a threadCount here if
    // MapboxLive shows a first tile well over a second.
    CesiumAsync::AsyncSystem asyncSystem(std::make_shared<SimpleTaskProcessor>());
    std::shared_ptr<CountingAssetAccessor> counting;
    // "off": no cache database at all, so no Mapbox byte reaches the disk.
    Cesium3DTilesSelection::TilesetExternals externals =
        build_externals(std::move(base), asyncSystem, "off", kDefaultMaxCacheItems, &counting);

    auto state = std::make_shared<LoaderState>();
    {
        std::lock_guard<std::mutex> lock(g_lastStateMutex);
        g_lastState = state;
    }
    auto loader = std::make_unique<MapboxBuildingsLoader>(
        std::make_shared<const LoaderConfig>(
            LoaderConfig{std::move(token), spec.materials_original, anchor.origin_height_m}),
        std::move(state));
    std::unique_ptr<Tile> root = make_root(*loader, anchor.origin_height_m);
    // Buildings only: never follows terrain, never replaces the ground plane.
    return std::make_unique<StreamingEnvironmentSource>(
        externals, 0, std::string(), std::string(), spec.fallback_dir, anchor, std::move(counting),
        spec.materials_original, false, 0.3, false, 2.0, spec.brightness, spec.radius_m,
        CustomTileset{std::move(loader), std::move(root), true});
}

}

}

namespace overlume {

std::unique_ptr<EnvironmentSource> open_mapbox_environment_source(const std::string& mapbox_spec,
                                                                  GeoAnchor anchor) {
    const char* token = std::getenv("MAPBOX_TOKEN");
    return mapbox::open_mapbox_source(mapbox_spec, anchor, nullptr, token != nullptr ? token : "");
}

}

namespace overlume::testing {

struct MapboxFixtureNetwork {
    std::shared_ptr<overlume::MapFixtureAssetAccessor> accessor;
};

MapboxFixtureNetwork* make_mapbox_fixture_network(uint16_t unmatched_status) {
    return new MapboxFixtureNetwork{
        std::make_shared<overlume::MapFixtureAssetAccessor>(unmatched_status)};
}

void mapbox_fixture_serve(MapboxFixtureNetwork* n, const std::string& url_without_query,
                          uint16_t status, std::vector<uint8_t> body) {
    const auto* bytes = reinterpret_cast<const std::byte*>(body.data());
    n->accessor->serve(url_without_query, status,
                       std::vector<std::byte>(bytes, bytes + body.size()));
}

void mapbox_fixture_fail_transport(MapboxFixtureNetwork* n, const std::string& url_without_query) {
    n->accessor->fail_transport(url_without_query);
}

void mapbox_fixture_hold(MapboxFixtureNetwork* n, const std::string& url_without_query) {
    n->accessor->hold(url_without_query);
}

void mapbox_fixture_release_held(MapboxFixtureNetwork* n) { n->accessor->release_held(); }

std::vector<std::string> mapbox_fixture_requested_urls(MapboxFixtureNetwork* n) {
    return n->accessor->requested_urls();
}

std::string mapbox_landmark_url(uint32_t x14, uint32_t y14) {
    return overlume::mapbox::landmark_url(x14, y14);
}

std::string mapbox_footprint_url(uint32_t x16, uint32_t y16) {
    return overlume::mapbox::footprint_url(x16, y16);
}

bool install_fixture_mapbox_source(overlume::VisualRenderer* r, const char* mapbox_uri,
                                   overlume::GeoAnchor anchor, MapboxFixtureNetwork* n,
                                   const char* token) {
    constexpr char kScheme[] = "mapbox://";
    if (r == nullptr || mapbox_uri == nullptr || n == nullptr) return false;
    const std::string uri(mapbox_uri);
    if (uri.rfind(kScheme, 0) != 0) return false;
    std::unique_ptr<overlume::EnvironmentSource> source = overlume::mapbox::open_mapbox_source(
        uri.substr(sizeof(kScheme) - 1), anchor, n->accessor, token != nullptr ? token : "");
    if (!source) return false;
    source->set_visible(*r, r->environmentVisible);
    if (r->environmentSource) r->environmentSource->teardown(*r);
    r->environmentSource = std::move(source);
    return true;
}

MapboxSourceStats mapbox_last_source_stats() {
    std::shared_ptr<overlume::mapbox::LoaderState> state;
    {
        std::lock_guard<std::mutex> lock(overlume::mapbox::g_lastStateMutex);
        state = overlume::mapbox::g_lastState;
    }
    MapboxSourceStats out;
    if (!state) return out;
    out.tiles_built = state->tiles_built.load();
    out.landmark_nodes = state->landmark_nodes.load();
    out.extruded_features = state->extruded_features.load();
    out.conflated_features = state->conflated_features.load();
    out.retries = state->retries.load();
    out.max_build_ms = state->max_build_ms.load();
    out.wall_quads = state->wall_quads.load();
    out.max_tile_top_m = state->max_tile_top_m.load();
    return out;
}

void mapbox_set_test_clock(double seconds) { overlume::mapbox::g_testClockS.store(seconds); }

}
