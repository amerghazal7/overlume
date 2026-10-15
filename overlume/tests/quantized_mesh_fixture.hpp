// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

// Cesium terrain synthesized in the test process, so no ion bytes enter the tree:
// quantized-mesh-1.0 tiles (https://github.com/CesiumGS/quantized-mesh: an 88-byte header, the
// vertex count, zigzag-delta u/v/height, high-water-mark 16-bit indices, edge indices; no
// extensions), the layer.json that lists them, the Cesium ion endpoint bodies and a TMS
// tilemapresource.xml. Header-only C++17. Little-endian hosts only, as every Overlume target is.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace overlume::testing::qmesh {

struct QuantizedMeshOptions {
    uint32_t grid = 8;  // cells per side, 1..255 (so the vertices fit 16-bit indices)
    double min_height_m = 0.0;
    double max_height_m = 0.0;
};

// What the fixture endpoints hand out. The token is a fixed fake, never a credential.
inline constexpr const char* kFixtureAssetToken = "overlume-fixture-asset-token";
inline constexpr const char* kTerrainAttributionHtml = "<span>Overlume fixture terrain</span>";
inline constexpr const char* kImageryAttributionHtml = "<b>Overlume fixture imagery</b>";
inline constexpr uint32_t kTmsMaxLevel = 18;  // tilemapresource_xml lists levels 0..18

namespace detail {

template <typename T>
void put(std::vector<uint8_t>& out, T value) {
    uint8_t bytes[sizeof(T)];
    std::memcpy(bytes, &value, sizeof(T));
    out.insert(out.end(), bytes, bytes + sizeof(T));
}

inline uint16_t zigzag(int32_t d) {
    return static_cast<uint16_t>((static_cast<uint32_t>(d) << 1) ^ static_cast<uint32_t>(d >> 31));
}

// WGS84 longitude, latitude (degrees) and ellipsoidal height (m) to ECEF metres.
inline std::array<double, 3> ecef(double lon_deg, double lat_deg, double height_m) {
    constexpr double kA = 6378137.0, kE2 = 6.69437999014e-3;
    constexpr double kRad = 3.14159265358979323846 / 180.0;
    const double lon = lon_deg * kRad, lat = lat_deg * kRad;
    const double n = kA / std::sqrt(1.0 - kE2 * std::sin(lat) * std::sin(lat));
    return {(n + height_m) * std::cos(lat) * std::cos(lon),
            (n + height_m) * std::cos(lat) * std::sin(lon),
            (n * (1.0 - kE2) + height_m) * std::sin(lat)};
}

}

// One quantized-mesh tile (header, zigzag-delta u/v/height, 16-bit indices, edge indices) covering
// the tile at x, y, level of the geographic (EPSG:4326) TMS quadtree (2 x 1 root tiles, y = 0 at
// the south), flat at height (min == max) or a ramp between them rising west to east. Vertices are
// a (grid + 1)^2 lattice; empty when grid is 0 or above 255.
inline std::vector<uint8_t> encode_quantized_mesh(uint32_t level, uint32_t x, uint32_t y,
                                                  const QuantizedMeshOptions& o) {
    if (o.grid == 0 || o.grid > 255) return {};
    const uint32_t n = o.grid + 1;  // lattice vertices per side
    const double size_deg = 180.0 / std::ldexp(1.0, static_cast<int>(level));
    const double west = -180.0 + x * size_deg, south = -90.0 + y * size_deg;
    const double span_m = o.max_height_m - o.min_height_m;
    const auto quantize = [&o](uint32_t k) {
        return static_cast<uint16_t>((k * 32767u + o.grid / 2) / o.grid);
    };
    // Two triangles per cell, counter-clockwise seen from above; lattice vertex (i east, j north)
    // is j * n + i.
    std::vector<uint32_t> triangles;
    for (uint32_t j = 0; j < o.grid; ++j) {
        for (uint32_t i = 0; i < o.grid; ++i) {
            const uint32_t sw = j * n + i, se = sw + 1, nw = sw + n, ne = nw + 1;
            triangles.insert(triangles.end(), {sw, se, ne, sw, ne, nw});
        }
    }
    // The high-water-mark index code needs vertices numbered in order of first use.
    std::vector<uint32_t> id_of(size_t{n} * n, std::numeric_limits<uint32_t>::max()), lattice_of;
    for (const uint32_t g : triangles) {
        if (id_of[g] == std::numeric_limits<uint32_t>::max()) {
            id_of[g] = static_cast<uint32_t>(lattice_of.size());
            lattice_of.push_back(g);
        }
    }
    const std::array<double, 3> centre = detail::ecef(west + size_deg / 2.0, south + size_deg / 2.0,
                                                      (o.min_height_m + o.max_height_m) / 2.0);
    std::vector<uint16_t> us, vs, hs;
    double radius = 0.0;
    for (const uint32_t g : lattice_of) {
        const uint32_t i = g % n, j = g / n;
        us.push_back(quantize(i));
        vs.push_back(quantize(j));
        hs.push_back(span_m > 0.0 ? quantize(i) : uint16_t{0});
        const std::array<double, 3> p =
            detail::ecef(west + size_deg * i / o.grid, south + size_deg * j / o.grid,
                         o.min_height_m + span_m * i / o.grid);
        radius = std::max(radius, std::hypot(p[0] - centre[0], p[1] - centre[1], p[2] - centre[2]));
    }

    std::vector<uint8_t> out;
    for (const double c : centre) detail::put(out, c);  // CenterX, Y, Z
    detail::put(out, static_cast<float>(o.min_height_m));
    detail::put(out, static_cast<float>(o.max_height_m));
    for (const double c : centre) detail::put(out, c);  // BoundingSphereCenter
    detail::put(out, radius);                           // BoundingSphereRadius
    // ponytail: the horizon occlusion point is the ellipsoid-scaled centre, not CesiumJS's horizon
    // point; cesium-native reads it and never uses it. Compute the real point if a consumer culls.
    detail::put(out, centre[0] / 6378137.0);
    detail::put(out, centre[1] / 6378137.0);
    detail::put(out, centre[2] / 6356752.314245179);
    detail::put(out, static_cast<uint32_t>(lattice_of.size()));  // vertexCount
    for (const std::vector<uint16_t>* values : {&us, &vs, &hs}) {
        int32_t previous = 0;
        for (const uint16_t v : *values) {
            detail::put(out, detail::zigzag(int32_t{v} - previous));
            previous = v;
        }
    }
    detail::put(out, static_cast<uint32_t>(triangles.size() / 3));  // triangleCount
    uint32_t highest = 0;
    for (const uint32_t g : triangles) {
        detail::put(out, static_cast<uint16_t>(highest - id_of[g]));
        if (id_of[g] == highest) ++highest;
    }
    // Edge indices: west (i = 0), south (j = 0), east (i = grid), north (j = grid).
    for (int edge = 0; edge < 4; ++edge) {
        detail::put(out, n);
        for (uint32_t k = 0; k < n; ++k) {
            const uint32_t i = edge == 0 ? 0 : edge == 2 ? o.grid : k;
            const uint32_t j = edge == 1 ? 0 : edge == 3 ? o.grid : k;
            detail::put(out, static_cast<uint16_t>(id_of[j * n + i]));
        }
    }
    return out;
}

// A layer.json for a geographic tiling scheme with `available` for exactly the tiles listed,
// each (level, x, y). Tiles resolve against its URL as "{z}/{x}/{y}.terrain?v=1.0.0".
inline std::string layer_json(const std::vector<std::array<uint32_t, 3>>& available_level_x_y) {
    uint32_t max_level = 0;
    for (const std::array<uint32_t, 3>& t : available_level_x_y) {
        max_level = std::max(max_level, t[0]);
    }
    std::string available;
    for (uint32_t level = 0; level <= max_level; ++level) {
        std::string ranges;
        for (const std::array<uint32_t, 3>& t : available_level_x_y) {
            if (t[0] != level) continue;
            const std::string x = std::to_string(t[1]), y = std::to_string(t[2]);
            ranges += std::string(ranges.empty() ? "" : ",") + "{\"startX\":" + x +
                      ",\"startY\":" + y + ",\"endX\":" + x + ",\"endY\":" + y + "}";
        }
        available += std::string(level == 0 ? "" : ",") + "[" + ranges + "]";
    }
    return "{\"tilejson\":\"2.1.0\",\"format\":\"quantized-mesh-1.0\",\"version\":\"1.0.0\","
           "\"scheme\":\"tms\",\"projection\":\"EPSG:4326\","
           "\"tiles\":[\"{z}/{x}/{y}.terrain?v={version}\"],\"maxzoom\":" +
           std::to_string(max_level) + ",\"available\":[" + available + "]}";
}

// Cesium ion endpoint bodies the fixture network serves at
// https://api.cesium.com/v1/assets/{id}/endpoint. URLs carry no quote or backslash and end with
// '/': cesium resolves "layer.json" against the terrain URL, and adds a missing '/' to a TMS URL
// only when its path has 4 or more characters (a base like ".../2" would lose its last segment).
inline std::string ion_terrain_endpoint_json(const std::string& layer_json_base_url) {
    return std::string("{\"type\":\"TERRAIN\",\"url\":\"") + layer_json_base_url +
           "\",\"accessToken\":\"" + kFixtureAssetToken + "\",\"attributions\":[{\"html\":\"" +
           kTerrainAttributionHtml + "\",\"collapsible\":false}]}";
}

// Type IMAGERY with no externalType: cesium opens the URL as a Tile Map Service.
inline std::string ion_imagery_endpoint_json(const std::string& tms_base_url) {
    return std::string("{\"type\":\"IMAGERY\",\"url\":\"") + tms_base_url +
           "\",\"accessToken\":\"" + kFixtureAssetToken + "\",\"attributions\":[{\"html\":\"" +
           kImageryAttributionHtml + "\",\"collapsible\":false}]}";
}

// For the TMS imagery: a geodetic (EPSG:4326) tile map with levels 0..kTmsMaxLevel, each at
// href "<level>", so tiles are <tms_base_url><level>/<x>/<y>.png (y = 0 at the south).
inline std::string tilemapresource_xml(uint32_t tile_size) {
    const std::string s = std::to_string(tile_size);
    std::string xml =
        "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
        "<TileMap version=\"1.0.0\" tilemapservice=\"http://tms.osgeo.org/1.0.0\">\n"
        "  <SRS>EPSG:4326</SRS>\n"
        "  <BoundingBox minx=\"-180.0\" miny=\"-90.0\" maxx=\"180.0\" maxy=\"90.0\"/>\n"
        "  <Origin x=\"-180.0\" y=\"-90.0\"/>\n"
        "  <TileFormat width=\"" +
        s + "\" height=\"" + s + "\" mime-type=\"image/png\" extension=\"png\"/>\n" +
        "  <TileSets profile=\"geodetic\">\n";
    for (uint32_t level = 0; level <= kTmsMaxLevel; ++level) {
        const std::string l = std::to_string(level);
        xml += "    <TileSet href=\"" + l + "\" order=\"" + l + "\"/>\n";
    }
    return xml + "  </TileSets>\n</TileMap>\n";
}

}
