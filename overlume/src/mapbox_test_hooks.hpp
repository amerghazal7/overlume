// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

// inside_any_footprint for C++17 tests (the real one is declared in C++20 mapbox_landmarks.hpp).
bool mapbox_inside_any_footprint_probe(const std::vector<std::array<double, 6>>& triangles,
                                       double x, double y);
struct MapboxLandmarkProbe {
    bool ok = false;
    std::string error;
    uint32_t buildings_kept = 0;
    uint32_t nodes_skipped = 0;
    uint32_t scenes = 0, nodes = 0, meshes = 0, accessors = 0, buffer_views = 0, buffers = 0;
    uint32_t images = 0, textures = 0, samplers = 0, materials = 0;
    uint32_t footprint_triangles = 0;
    uint64_t buffer_bytes = 0;            // summed size of every buffer's data
    bool meshopt_extension_left = false;  // EXT_meshopt_compression anywhere after reading
    std::vector<std::array<double, 16>> node_matrices;  // kept nodes, column-major, in order
};
MapboxLandmarkProbe probe_mapbox_landmark_tile(const std::vector<uint8_t>& glb, uint32_t x14,
                                               uint32_t y14, double origin_height_m,
                                               bool keep_materials);

// A point (u, v) in tile (z, x, y) units (`extent` per edge) at z_m metres above the
// anchor's ground, through the Mapbox placement chain (tile frame of its z14 tile, ECEF, the map
// frame of `anchor`, the stream's height correction) → map metres.
bool mapbox_tile_point_to_map_probe(overlume::GeoAnchor anchor, uint32_t z, uint32_t x, uint32_t y,
                                    double u, double v, double extent, double z_m,
                                    double out_map_xyz[3]);

struct MapboxFixtureNetwork;  // wraps a MapFixtureAssetAccessor; never freed
MapboxFixtureNetwork* make_mapbox_fixture_network(uint16_t unmatched_status = 404);
void mapbox_fixture_serve(MapboxFixtureNetwork* n, const std::string& url_without_query,
                          uint16_t status, std::vector<uint8_t> body);
void mapbox_fixture_fail_transport(MapboxFixtureNetwork* n, const std::string& url_without_query);
// Requests for a held URL stay pending until mapbox_fixture_release_held answers them as served.
void mapbox_fixture_hold(MapboxFixtureNetwork* n, const std::string& url_without_query);
void mapbox_fixture_release_held(MapboxFixtureNetwork* n);
std::vector<std::string> mapbox_fixture_requested_urls(MapboxFixtureNetwork* n);
std::string mapbox_landmark_url(uint32_t x14, uint32_t y14);   // no query string
std::string mapbox_footprint_url(uint32_t x16, uint32_t y16);  // no query string
std::string mapbox_dem_url(uint32_t x14, uint32_t y14);        // no query string
// Mapbox Satellite's XYZ tile (row 0 at the north edge), no query string.
std::string mapbox_satellite_url(uint32_t z, uint32_t x, uint32_t y);
// Opens a mapbox:// URI exactly as set_environment_source does (parse, source, set_visible,
// teardown of the old source, swap), but over `n` and with `token` in place of MAPBOX_TOKEN.
bool install_fixture_mapbox_source(overlume::VisualRenderer* r, const char* mapbox_uri,
                                   overlume::GeoAnchor anchor, MapboxFixtureNetwork* n,
                                   const char* token);
// Counting rules: a tile counts once, when it reaches Success with content. A feature counts once
// per z16 square it is decoded from (a building clipped into two squares counts twice), and in
// exactly one of extruded_features (it produced geometry) or conflated_features (dropped under a
// landmark footprint); filtered-out features (extrude=false, underground, no height) count in
// neither.
struct MapboxSourceStats {
    uint64_t tiles_built = 0;         // z14 tiles that reached Success with content
    uint64_t landmark_nodes = 0;      // building nodes kept, summed over built tiles
    uint64_t extruded_features = 0;   // features extruded, summed
    uint64_t conflated_features = 0;  // features dropped under a landmark footprint
    uint64_t retries = 0;             // attempts that ended in RetryLater (waits not counted)
    double max_build_ms = 0.0;        // slowest tile build, worker time
    uint64_t wall_quads = 0;          // wall quads of the extruded features, summed
    // Highest region top over built tiles, metres above the anchor ground.
    double max_tile_top_m = 0.0;
    // Lowest region floor over built tiles, metres above the anchor ground (negative: below).
    double min_tile_floor_m = 0.0;
    // DEM tiles the source's DemCache requested (terrain layer); 0 once the source is torn down.
    uint64_t dem_requests = 0;
    // Terrain z14 tiles that reached Success with content.
    uint64_t terrain_tiles_built = 0;
    // Terrain z14 attempts that ended in RetryLater (waits not counted).
    uint64_t terrain_retries = 0;
};
// The counters of the most recently opened Mapbox source in this process.
MapboxSourceStats mapbox_last_source_stats();
// A fresh DemCache over n: get() each z14 (x, y) in order, waiting for each; requests() after each
// get.
std::vector<uint64_t> mapbox_dem_cache_requests(MapboxFixtureNetwork* n,
                                                const std::vector<std::array<uint32_t, 2>>& tiles);
// Test-only: while on, every GLB the stream writes in prepareInLoadThread is kept; turning it on
// drops what was kept before.
void capture_stream_glbs(bool on);
std::vector<std::vector<uint8_t>> captured_stream_glbs();
// Test-only: freezes the clock the loader's retry wait reads at `seconds`; a negative value gives
// the steady clock back.
void mapbox_set_test_clock(double seconds);

}
