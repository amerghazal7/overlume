// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// The Mapbox terrain loader (docs/design/2026-10-15-imagery-terrain-design.md, section 5): the
// z14 terrain-DEM tiles within the radius, meshed with skirts in each tile's east-north-up frame.
// Raster overlays drape on them, and cesium upsamples below z14 for finer imagery.

#include "environment_stream.hpp"
#include "mapbox_dem.hpp"
#include "mapbox_dem_cache.hpp"
#include "mapbox_tile_frame.hpp"
#include "mapbox_tiling.hpp"

#include <Cesium3DTilesSelection/Tile.h>
#include <Cesium3DTilesSelection/TileLoadResult.h>
#include <Cesium3DTilesSelection/TilesetContentLoader.h>
#include <CesiumGeometry/QuadtreeTileID.h>
#include <CesiumGeospatial/Ellipsoid.h>
#include <CesiumGltf/Model.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace overlume::mapbox {

namespace {

using Cesium3DTilesSelection::Tile;
using Cesium3DTilesSelection::TileChildrenResult;
using Cesium3DTilesSelection::TileEmptyContent;
using Cesium3DTilesSelection::TileLoadInput;
using Cesium3DTilesSelection::TileLoadResult;
using Cesium3DTilesSelection::TileLoadResultState;

// The z14 leaves' geometric error. The area view refines a tile whose error is 1 or more
// (TilesetSelection.cpp:335) and cesium halves it for each level of upsampled children
// (TilesetContentManager.cpp:280): z14 (2) and z15 (1) refine, z16 (0.5) is the leaf near the ego.
// The imagery on those z16 tiles is z17, not the overlay's z18 cap (maximumLevel): cesium lowers a
// tile's imagery level until its texture fits maximumTextureSize, 2048 px by default
// (QuadtreeRasterOverlayTileProvider.cpp:312-325). SatelliteDrapesXyzTilesOnTheTerrain asserts
// z17 requests, none above z18 and at most 64 requests in all; at the defaults the texture budget
// stops at z17 before the cap binds, so the requests cannot see the cap (a limit of 8192 px makes
// the same tiles ask for z18) and the test reads the cap, the 512 px tile size and the credit from
// the overlay's tile provider (environment_stream_overlay_options_probe). A z14 leaf left at
// error 0 would not refine, and its imagery would top out at z16.
constexpr double kTerrainGeometricErrorM = 2.0;

static_assert(sizeof(std::array<float, 3>) == 3 * sizeof(float) &&
              sizeof(std::array<float, 2>) == 2 * sizeof(float));

// One buffer (positions, normals, UVs, uint32 indices), one TRIANGLES primitive, one material
// (the terrain renderer resources swap it for terrain.mat), one node in scene 0.
CesiumGltf::Model write_dem_gltf(const DemMesh& mesh) {
    using CesiumGltf::Accessor;
    using CesiumGltf::BufferView;
    CesiumGltf::Model model;
    const size_t vertexCount = mesh.positions.size();
    const size_t vec3Bytes = vertexCount * sizeof(std::array<float, 3>);
    const size_t uvBytes = vertexCount * sizeof(std::array<float, 2>);
    const size_t indexBytes = mesh.indices.size() * sizeof(uint32_t);
    CesiumGltf::Buffer& buffer = model.buffers.emplace_back();
    buffer.cesium.data.resize(2 * vec3Bytes + uvBytes + indexBytes);
    std::byte* out = buffer.cesium.data.data();
    std::memcpy(out, mesh.positions.data(), vec3Bytes);
    std::memcpy(out + vec3Bytes, mesh.normals.data(), vec3Bytes);
    std::memcpy(out + 2 * vec3Bytes, mesh.uvs.data(), uvBytes);
    std::memcpy(out + 2 * vec3Bytes + uvBytes, mesh.indices.data(), indexBytes);
    buffer.byteLength = static_cast<int64_t>(buffer.cesium.data.size());

    const auto add_accessor = [&model](size_t offset, size_t length, int32_t target,
                                       int32_t componentType, const std::string& type,
                                       size_t count) {
        BufferView& view = model.bufferViews.emplace_back();
        view.buffer = 0;
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
    const int32_t position =
        add_accessor(0, vec3Bytes, BufferView::Target::ARRAY_BUFFER, Accessor::ComponentType::FLOAT,
                     Accessor::Type::VEC3, vertexCount);
    const int32_t normal =
        add_accessor(vec3Bytes, vec3Bytes, BufferView::Target::ARRAY_BUFFER,
                     Accessor::ComponentType::FLOAT, Accessor::Type::VEC3, vertexCount);
    const int32_t uv =
        add_accessor(2 * vec3Bytes, uvBytes, BufferView::Target::ARRAY_BUFFER,
                     Accessor::ComponentType::FLOAT, Accessor::Type::VEC2, vertexCount);
    const int32_t indices = add_accessor(
        2 * vec3Bytes + uvBytes, indexBytes, BufferView::Target::ELEMENT_ARRAY_BUFFER,
        Accessor::ComponentType::UNSIGNED_INT, Accessor::Type::SCALAR, mesh.indices.size());

    // glTF requires POSITION bounds.
    std::vector<double> lo(3, std::numeric_limits<double>::max());
    std::vector<double> hi(3, std::numeric_limits<double>::lowest());
    for (const std::array<float, 3>& p : mesh.positions) {
        for (size_t k = 0; k < 3; ++k) {
            lo[k] = std::min(lo[k], static_cast<double>(p[k]));
            hi[k] = std::max(hi[k], static_cast<double>(p[k]));
        }
    }
    model.accessors[static_cast<size_t>(position)].min = lo;
    model.accessors[static_cast<size_t>(position)].max = hi;

    CesiumGltf::MaterialPBRMetallicRoughness& pbr =
        model.materials.emplace_back().pbrMetallicRoughness.emplace();
    pbr.metallicFactor = 0.0;
    pbr.roughnessFactor = 1.0;

    CesiumGltf::MeshPrimitive& primitive = model.meshes.emplace_back().primitives.emplace_back();
    primitive.attributes["POSITION"] = position;
    primitive.attributes["NORMAL"] = normal;
    primitive.attributes["TEXCOORD_0"] = uv;
    primitive.indices = indices;
    primitive.material = 0;
    primitive.mode = CesiumGltf::MeshPrimitive::Mode::TRIANGLES;

    model.nodes.emplace_back().mesh = 0;
    model.scenes.emplace_back().nodes.push_back(0);
    model.scene = 0;
    model.asset.version = "2.0";
    return model;
}

// Worker thread: one z14 tile from its DEM and the anchor's height on it (rebase_of).
TileLoadResult build_terrain_tile(const TileId& t14, uint32_t attempt, std::atomic<bool>& settled,
                                  const Rebase& rebase, double origin_height_m,
                                  std::atomic<uint64_t>& tiles_built,
                                  std::atomic<uint64_t>& retries,
                                  const std::shared_ptr<CesiumAsync::IAssetAccessor>& accessor,
                                  const CesiumGeospatial::Ellipsoid& ellipsoid) {
    if (!rebase.active()) {
        // A DEM failing for now goes back for another attempt; an empty or unreadable one, or the
        // last attempt, leaves no terrain there.
        if (rebase.retry && attempt < kMaxAttempts) {
            ++retries;
            return TileLoadResult::createRetryLaterResult(accessor, nullptr);
        }
        settled = true;
        return success(TileEmptyContent{}, std::nullopt, accessor, ellipsoid);
    }
    settled = true;  // the next load of this tile starts counting attempts afresh
    const DemMesh mesh =
        mesh_dem(*rebase.dem, make_tile_frame(t14, origin_height_m), t14, *rebase.z_ref_m);
    double low = std::numeric_limits<double>::max();
    double high = std::numeric_limits<double>::lowest();
    for (const std::array<float, 3>& p : mesh.positions) {
        low = std::min(low, static_cast<double>(p[2]));
        high = std::max(high, static_cast<double>(p[2]));
    }
    ++tiles_built;
    return success(write_dem_gltf(mesh),
                   tile_region(t14, origin_height_m + low, origin_height_m + high, ellipsoid),
                   accessor, ellipsoid);
}

// The terrain on mapbox_tiling's z14 quadtree: each level-14 tile carries one tile's DEM mesh.
class MapboxTerrainLoader final : public Cesium3DTilesSelection::TilesetContentLoader {
public:
    MapboxTerrainLoader(std::shared_ptr<DemCache> dem, const GeoAnchor& anchor,
                        std::shared_ptr<std::atomic<uint64_t>> tiles_built,
                        std::shared_ptr<std::atomic<uint64_t>> retries)
        : dem_(std::move(dem)),
          anchor_{anchor.origin_lon_deg, anchor.origin_lat_deg},
          tilesBuilt_(std::move(tiles_built)),
          terrainRetries_(std::move(retries)),
          config_{std::string(), false, anchor.origin_height_m} {}

    CesiumAsync::Future<TileLoadResult> loadTileContent(const TileLoadInput& input) override {
        const auto* id = std::get_if<CesiumGeometry::QuadtreeTileID>(&input.tile.getTileID());
        if (id == nullptr || id->level != kLandmarkZoom) {
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
        return rebase_of(dem_, t14, anchor_)
            .thenInWorkerThread([t14, attempt, settled = a.settled,
                                 origin = config_.origin_height_m, built = tilesBuilt_,
                                 retries = terrainRetries_, accessor = input.pAssetAccessor,
                                 ellipsoid = input.ellipsoid](const Rebase& rebase) {
                return build_terrain_tile(t14, attempt, *settled, rebase, origin, *built, *retries,
                                          accessor, ellipsoid);
            });
    }

    TileChildrenResult createTileChildren(const Tile& tile,
                                          const CesiumGeospatial::Ellipsoid& ellipsoid) override {
        std::vector<Tile> children = quadtree_children(tile, config_, ellipsoid);
        for (Tile& child : children) {
            const auto* id = std::get_if<CesiumGeometry::QuadtreeTileID>(&child.getTileID());
            if (id != nullptr && id->level == kLandmarkZoom) {
                child.setGeometricError(kTerrainGeometricErrorM);
            }
        }
        return {std::move(children), TileLoadResultState::Success};
    }

private:
    std::shared_ptr<DemCache> dem_;
    LonLatDeg anchor_;
    std::shared_ptr<std::atomic<uint64_t>> tilesBuilt_;
    std::shared_ptr<std::atomic<uint64_t>> terrainRetries_;
    LoaderConfig config_;   // the anchor height for quadtree_children; no token
    AttemptBook attempts_;  // main thread only: cesium calls loadTileContent there
};

}

CustomTileset make_mapbox_terrain_tileset(std::shared_ptr<DemCache> dem, const GeoAnchor& anchor,
                                          std::shared_ptr<std::atomic<uint64_t>> tiles_built,
                                          std::shared_ptr<std::atomic<uint64_t>> retries) {
    auto loader = std::make_unique<MapboxTerrainLoader>(std::move(dem), anchor,
                                                        std::move(tiles_built), std::move(retries));
    std::unique_ptr<Tile> root = quadtree_root(*loader, anchor.origin_height_m);
    return CustomTileset{std::move(loader), std::move(root), true};
}

}
