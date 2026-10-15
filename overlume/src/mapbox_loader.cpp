// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_loader.hpp"

#include "environment_stream.hpp"
#include "mapbox_dem_cache.hpp"
#include "mapbox_extrude.hpp"
#include "mapbox_landmarks.hpp"
#include "mapbox_mvt.hpp"
#include "mapbox_test_hooks.hpp"
#include "mapbox_tile_frame.hpp"
#include "mapbox_tiling.hpp"
#include "renderer_internal.hpp"

#include <Cesium3DTilesSelection/BoundingVolume.h>
#include <Cesium3DTilesSelection/Tile.h>
#include <Cesium3DTilesSelection/TileLoadResult.h>
#include <Cesium3DTilesSelection/TilesetContentLoader.h>
#include <CesiumCurl/CurlAssetAccessor.h>
#include <CesiumGeometry/QuadtreeTileID.h>
#include <CesiumGeospatial/Cartographic.h>
#include <CesiumGeospatial/Ellipsoid.h>
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
#include <numbers>
#include <optional>
#include <span>
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
    MapboxSpec out;
    out.buildings = false;
    size_t from = 0;
    while (true) {
        const size_t comma = layers.find(',', from);
        const std::string name =
            layers.substr(from, comma == std::string::npos ? comma : comma - from);
        bool* layer = nullptr;
        if (name == "buildings") layer = &out.buildings;
        if (name == "terrain") layer = &out.terrain;
        // Empty, unknown and repeated names share one text: the second mention is the one refused.
        if (layer == nullptr || *layer) {
            return {
                std::nullopt,
                "mapbox:// supports only the layers 'buildings' and 'terrain', not '" + name + "'",
                ""};
        }
        *layer = true;
        if (comma == std::string::npos) break;
        from = comma + 1;
    }
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
            } else if (key == "imagery") {
                if (val != "satellite" && val != "off")
                    return {std::nullopt,
                            "mapbox:// imagery='" + val + "' is not 'satellite' or 'off'", ""};
                out.imagery_satellite = val == "satellite";
            }
        }
        if (amp == std::string::npos) break;
        pos = amp + 1;
    }
    if (out.imagery_satellite && !out.terrain)
        return {std::nullopt, "mapbox:// imagery=satellite needs the 'terrain' layer", ""};
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

// first_time (SourceLog): at most one log line per endpoint and status, or failure kind.
struct LoaderState : SourceLog {
    std::atomic<uint64_t> tiles_built{0};
    std::atomic<uint64_t> landmark_nodes{0};
    std::atomic<uint64_t> extruded_features{0};
    std::atomic<uint64_t> conflated_features{0};
    std::atomic<uint64_t> retries{0};
    std::atomic<uint64_t> wall_quads{0};
    std::atomic<double> max_build_ms{0.0};
    std::atomic<double> max_tile_top_m{0.0};
    std::atomic<double> min_tile_floor_m{0.0};
    // The source's DEM cache when the layer list carries `terrain`, for dem_requests; the loaders
    // own it.
    std::weak_ptr<const DemCache> dem;

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

    void note_floor_m(double floor) {
        double seen = min_tile_floor_m.load();
        while (floor < seen && !min_tile_floor_m.compare_exchange_weak(seen, floor)) {
        }
    }
};

std::mutex g_lastStateMutex;
std::shared_ptr<LoaderState> g_lastState;  // the latest source's, for mapbox_last_source_stats()

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

// (u, v) in units of z16 square `square` (`extent` per edge) → DEM pixels of its z14 tile t14.
std::array<double, 2> square_to_dem_px(const TileId& square, const TileId& t14, double u, double v,
                                       double extent) {
    constexpr double kPxPerSquare = kDemSize / 4.0;
    return {(static_cast<double>(square.x - 4 * t14.x) + u / extent) * kPxPerSquare,
            (static_cast<double>(square.y - 4 * t14.y) + v / extent) * kPxPerSquare};
}

// The DEM pixel under tile-frame point (x, y, z) of frame f's z14 tile: frame → ECEF → longitude
// and latitude, project_to_frame inverted.
std::array<double, 2> frame_to_dem_px(const TileFrame& f, double x, double y, double z) {
    const glm::dvec4 ecef = glm::make_mat4(f.frame_to_ecef.data()) * glm::dvec4(x, y, z, 1.0);
    const std::optional<CesiumGeospatial::Cartographic> c =
        CesiumGeospatial::Ellipsoid::WGS84.cartesianToCartographic(glm::dvec3(ecef));
    if (!c) return {kDemSize / 2.0, kDemSize / 2.0};
    constexpr double kDegPerRad = 180.0 / std::numbers::pi;
    return dem_pixel_of(LonLatDeg{c->longitude * kDegPerRad, c->latitude * kDegPerRad}, f.tile);
}

// Worker thread: one z14 tile from its answers, in request order.
TileLoadResult build_tile(const LoaderConfig& cfg, LoaderState& st, const TileId& t14,
                          uint32_t attempt, std::atomic<bool>& settled,
                          const std::vector<Fetched>& fetched, const Rebase& rebase,
                          const std::shared_ptr<CesiumAsync::IAssetAccessor>& accessor,
                          spdlog::logger& log, const CesiumGeospatial::Ellipsoid& ellipsoid) {
    const auto started = std::chrono::steady_clock::now();
    const bool transient = std::any_of(fetched.begin(), fetched.end(), [](const Fetched& f) {
        return classify(f) == Outcome::Transient;
    });
    // On the terrain layer a DEM failing for now sends the tile back too: built at z = 0, its
    // buildings would stay buried or floating once the DEM answers.
    if ((transient || rebase.retry) && attempt < kMaxAttempts) {
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
    // On the terrain layer each landmark node stands on the DEM at its anchor, the node's origin:
    // its matrix's third column is the local up per metre (landmark_node_to_frame).
    double low = 0.0;           // the lowest lift, for the region's floor
    double landmarkLift = 0.0;  // the highest node lift, for the region's top
    if (landmarks.ok && rebase.active()) {
        bool first = true;
        for (CesiumGltf::Node& node : landmarks.model.nodes) {
            if (node.matrix.size() != 16) continue;
            const std::array<double, 2> px =
                frame_to_dem_px(frame, node.matrix[12], node.matrix[13], node.matrix[14]);
            const double lift = rebase.lift(px[0], px[1]);
            for (size_t i = 0; i < 3; ++i) node.matrix[12 + i] += lift * node.matrix[8 + i];
            landmarkLift = first ? lift : std::max(landmarkLift, lift);
            low = std::min(low, lift);
            first = false;
        }
    }
    TriangleMesh mesh;
    uint64_t extruded = 0;
    uint64_t conflated = 0;
    uint64_t walls = 0;
    double top = landmarks.ok ? landmark_top_m(landmarks.model) + landmarkLift : 0.0;
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
            const std::optional<std::array<double, 2>> c = footprint_centroid(polygons);
            // A landmark model replaces the extrusion whose unclipped footprint centroid it
            // covers, as GL JS does. read_landmark_tile returns an empty tile on every failure,
            // which is what the tests pin; this ok check and the merge's are belt and braces.
            if (landmarks.ok && c) {
                const std::array<double, 3> p = project((*c)[0], (*c)[1], 0.0);
                if (inside_any_footprint(landmarks.footprint_triangles, p[0], p[1])) {
                    ++conflated;
                    continue;
                }
            }
            // On the terrain layer the footprint stands on the DEM at its unclipped centroid,
            // flat-bottomed (spec section 12). ponytail: one base per square's copy, so a building
            // cut by a z16 square edge on a slope steps at the cut; key the lift by feature id
            // across the tile's squares if that shows.
            double lift = 0.0;
            if (rebase.active() && c) {
                const std::array<double, 2> px =
                    square_to_dem_px(square, t14, (*c)[0], (*c)[1], extent);
                lift = rebase.lift(px[0], px[1]);
            }
            // Clipping to the square's own extent drops the tile buffer, so the copies of a
            // building in neighbouring squares meet without doubled walls.
            const ExtrudeStats es =
                extrude_polygons(polygons, extent, base + lift, *b.height + lift, project, mesh);
            if (es.polygons_built == 0) continue;
            ++extruded;
            walls += es.wall_quads;
            top = std::max(top, *b.height + lift);
            low = std::min(low, lift);
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
    // From the lowest lifted base (kRegionBelowM under it, as before) to the highest top.
    const double floor = low - kRegionBelowM;
    st.note_top_m(top);
    st.note_floor_m(floor);
    return success(
        std::move(model),
        tile_region(t14, cfg.origin_height_m + floor, cfg.origin_height_m + top, ellipsoid),
        accessor, ellipsoid);
}

// The buildings on mapbox_tiling's z14 quadtree: each level-14 tile carries one tile's
// footprints and landmarks.
class MapboxBuildingsLoader final : public Cesium3DTilesSelection::TilesetContentLoader {
public:
    // `dem` is the source's DemCache when the layer list carries `terrain`, else null.
    MapboxBuildingsLoader(std::shared_ptr<const LoaderConfig> config,
                          std::shared_ptr<LoaderState> state, std::shared_ptr<DemCache> dem,
                          LonLatDeg anchor)
        : config_(std::move(config)),
          state_(std::move(state)),
          dem_(std::move(dem)),
          anchor_(anchor) {}

    CesiumAsync::Future<TileLoadResult> loadTileContent(const TileLoadInput& input) override {
        const auto* id = std::get_if<CesiumGeometry::QuadtreeTileID>(&input.tile.getTileID());
        if (id == nullptr || id->level != kLandmarkZoom) {
            // Levels 0-13 hold nothing; cesium may unload an empty tile and ask for it again.
            return input.asyncSystem.createResolvedFuture(
                success(TileEmptyContent{}, std::nullopt, input.pAssetAccessor, input.ellipsoid));
        }
        const TileId t14{kLandmarkZoom, id->x, id->y};
        const double now = clock_s();
        if (attempts_.waiting(t14, now)) {
            return input.asyncSystem.createResolvedFuture(
                TileLoadResult::createRetryLaterResult(input.pAssetAccessor, nullptr));
        }
        const Attempt& a = attempts_.begin(t14, now);
        const uint32_t attempt = a.count;

        // TileLoadInput is a stack object of references: the continuation copies what it needs.
        const std::string query = "?access_token=" + config_->token;
        std::vector<CesiumAsync::Future<Fetched>> requests;
        for (const TileId& square : footprint_tiles_of(t14)) {
            requests.push_back(fetch(input.asyncSystem, input.pAssetAccessor,
                                     footprint_url(square.x, square.y) + query));
        }
        requests.push_back(fetch(input.asyncSystem, input.pAssetAccessor,
                                 landmark_url(t14.x, t14.y) + query));  // kLandmarkIndex
        // With the terrain layer the build also waits for the tile's DEM and the anchor's
        // height; without it, for nothing.
        CesiumAsync::SharedFuture<Rebase> rebase =
            dem_ ? rebase_of(dem_, t14, anchor_)
                 : input.asyncSystem.createResolvedFuture(Rebase{}).share();
        CesiumAsync::SharedFuture<std::vector<Fetched>> answers =
            input.asyncSystem.all(std::move(requests)).share();
        return rebase.thenImmediately([answers, config = config_, state = state_, t14, attempt,
                                       settled = a.settled, accessor = input.pAssetAccessor,
                                       logger = input.pLogger,
                                       ellipsoid = input.ellipsoid](const Rebase& r) mutable {
            return answers.thenInWorkerThread([r, config, state, t14, attempt, settled, accessor,
                                               logger,
                                               ellipsoid](const std::vector<Fetched>& fetched) {
                return build_tile(*config, *state, t14, attempt, *settled, fetched, r, accessor,
                                  *logger, ellipsoid);
            });
        });
    }

    TileChildrenResult createTileChildren(const Tile& tile,
                                          const CesiumGeospatial::Ellipsoid& ellipsoid) override {
        return {quadtree_children(tile, *config_, ellipsoid), TileLoadResultState::Success};
    }

private:
    std::shared_ptr<const LoaderConfig> config_;
    std::shared_ptr<LoaderState> state_;
    // Null without the terrain layer.
    std::shared_ptr<DemCache> dem_;
    // The map origin, whose DEM height is z = 0.
    LonLatDeg anchor_;
    AttemptBook attempts_;  // main thread only: cesium calls loadTileContent there
};

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
    // The terrain layer: one DEM per z14 tile, shared by the source's loaders and never on disk
    // (the accessor is the cache-off counting one).
    const std::string query = "?access_token=" + token;
    std::shared_ptr<DemCache> dem;
    if (spec.terrain) {
        dem = std::make_shared<DemCache>(externals.pAssetAccessor, asyncSystem,
                                         std::string(kDemUrlTemplate) + query, log);
        state->dem = dem;
    }
    auto loader = std::make_unique<MapboxBuildingsLoader>(
        std::make_shared<const LoaderConfig>(
            LoaderConfig{std::move(token), spec.materials_original, anchor.origin_height_m}),
        std::move(state), dem, LonLatDeg{anchor.origin_lon_deg, anchor.origin_lat_deg});
    std::unique_ptr<Tile> root = quadtree_root(*loader, anchor.origin_height_m);
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
    out.min_tile_floor_m = state->min_tile_floor_m.load();
    if (const std::shared_ptr<const overlume::mapbox::DemCache> dem = state->dem.lock()) {
        out.dem_requests = dem->requests();
    }
    return out;
}

std::vector<uint64_t> mapbox_dem_cache_requests(MapboxFixtureNetwork* n,
                                                const std::vector<std::array<uint32_t, 2>>& tiles) {
    using namespace overlume::mapbox;
    DemCache cache(n->accessor, CesiumAsync::AsyncSystem(std::make_shared<SimpleTaskProcessor>()),
                   std::string(kDemUrlTemplate) + "?access_token=t", make_redacting_logger());
    std::vector<uint64_t> out;
    for (const std::array<uint32_t, 2>& t : tiles) {
        cache.get({kLandmarkZoom, t[0], t[1]}).wait();
        out.push_back(cache.requests());
    }
    return out;
}

}
