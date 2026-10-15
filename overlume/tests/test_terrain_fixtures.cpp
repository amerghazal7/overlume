// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// The terrain fixtures, pinned structurally. No library symbol is used: the cesium-OFF build
// (hosted CI) compiles and links every tests/test_*.cpp outside test_mapbox_* and
// test_environment_stream, so DEM PNGs are read back with stb_image (golden.cpp's) and the
// published terrain-RGB formula; decode_dem_png has its own known-pixel tests (test_mapbox_dem).

#include "imagery_fixture.hpp"
#include "mapbox_dem_fixture.hpp"
#include "quantized_mesh_fixture.hpp"

#include "stb_image.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace dem = overlume::testing::dem;
namespace imagery = overlume::testing::imagery;
namespace qmesh = overlume::testing::qmesh;

struct Rgb {
    int width = 0, height = 0;
    std::vector<uint8_t> pixels;  // RGB
};

Rgb read_png(const std::vector<uint8_t>& bytes) {
    Rgb out;
    int channels = 0;
    stbi_uc* p = stbi_load_from_memory(bytes.data(), static_cast<int>(bytes.size()), &out.width,
                                       &out.height, &channels, 3);
    if (p != nullptr) {
        out.pixels.assign(p, p + size_t(out.width) * size_t(out.height) * 3);
        stbi_image_free(p);
    }
    return out;
}

// A strict JSON (RFC 8259) syntax check: true when `s` is exactly one JSON value.
struct JsonCheck {
    const std::string& s;
    size_t i = 0;

    void ws() {
        while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n')) ++i;
    }
    bool lit(const char* word) {
        const size_t n = std::strlen(word);
        if (s.compare(i, n, word) != 0) return false;
        i += n;
        return true;
    }
    bool digits() {
        const size_t start = i;
        while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i])) != 0) ++i;
        return i > start;
    }
    bool number() {
        if (i < s.size() && s[i] == '-') ++i;
        if (!digits()) return false;
        if (i < s.size() && s[i] == '.') {
            ++i;
            if (!digits()) return false;
        }
        if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
            ++i;
            if (i < s.size() && (s[i] == '+' || s[i] == '-')) ++i;
            return digits();
        }
        return true;
    }
    bool string() {
        if (i >= s.size() || s[i] != '"') return false;
        for (++i; i < s.size(); ++i) {
            const unsigned char c = static_cast<unsigned char>(s[i]);
            if (c == '"') {
                ++i;
                return true;
            }
            if (c < 0x20) return false;
            if (c == '\\') {
                if (++i >= s.size() || std::strchr("\"\\/bfnrtu", s[i]) == nullptr) return false;
                if (s[i] == 'u') {
                    for (int k = 0; k < 4; ++k) {
                        if (++i >= s.size() ||
                            std::isxdigit(static_cast<unsigned char>(s[i])) == 0) {
                            return false;
                        }
                    }
                }
            }
        }
        return false;
    }
    bool members(char close, bool object) {
        ++i;
        ws();
        if (i < s.size() && s[i] == close) {
            ++i;
            return true;
        }
        for (;;) {
            if (object) {
                ws();
                if (!string()) return false;
                ws();
                if (i >= s.size() || s[i++] != ':') return false;
            }
            if (!value()) return false;
            if (i < s.size() && s[i] == ',') {
                ++i;
                continue;
            }
            if (i < s.size() && s[i] == close) {
                ++i;
                return true;
            }
            return false;
        }
    }
    bool value() {
        ws();
        if (i >= s.size()) return false;
        bool ok = false;
        switch (s[i]) {
            case '{':
                ok = members('}', true);
                break;
            case '[':
                ok = members(']', false);
                break;
            case '"':
                ok = string();
                break;
            case 't':
                ok = lit("true");
                break;
            case 'f':
                ok = lit("false");
                break;
            case 'n':
                ok = lit("null");
                break;
            default:
                ok = number();
        }
        ws();
        return ok;
    }
};

bool parses_as_json(const std::string& s) {
    JsonCheck c{s};
    return c.value() && c.i == s.size();
}

bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

// A parsed quantized-mesh tile: the fields the format defines, decoded here, not by cesium.
struct Qmesh {
    double center[3], bs_center[3], bs_radius;
    float min_h, max_h;
    uint32_t vertex_count = 0, triangle_count = 0;
    std::vector<uint16_t> u, v, h;               // after zigzag-delta decoding
    std::vector<uint32_t> indices;               // after high-water-mark decoding
    std::array<std::vector<uint16_t>, 4> edges;  // west, south, east, north
    size_t bytes_read = 0;
};

Qmesh parse(const std::vector<uint8_t>& b) {
    Qmesh q{};
    size_t at = 0;
    const auto read = [&b, &at](auto& value) {
        std::memcpy(&value, b.data() + at, sizeof value);
        at += sizeof value;
    };
    for (double& c : q.center) read(c);
    read(q.min_h);
    read(q.max_h);
    for (double& c : q.bs_center) read(c);
    read(q.bs_radius);
    at += 3 * sizeof(double);  // the horizon occlusion point
    EXPECT_EQ(at, 88u) << "the header is 88 bytes";
    read(q.vertex_count);
    for (std::vector<uint16_t>* values : {&q.u, &q.v, &q.h}) {
        int32_t running = 0;
        for (uint32_t k = 0; k < q.vertex_count; ++k) {
            uint16_t zz = 0;
            read(zz);
            running += (zz >> 1) ^ -(zz & 1);
            values->push_back(static_cast<uint16_t>(running));
        }
    }
    read(q.triangle_count);
    uint32_t highest = 0;
    for (uint32_t k = 0; k < q.triangle_count * 3; ++k) {
        uint16_t code = 0;
        read(code);
        q.indices.push_back(highest - code);
        if (code == 0) ++highest;
    }
    for (std::vector<uint16_t>& edge : q.edges) {
        uint32_t count = 0;
        read(count);
        edge.resize(count);
        for (uint16_t& index : edge) read(index);
    }
    q.bytes_read = at;
    return q;
}

}

TEST(TerrainFixtures, DemPngEncodesTheNamedScenes) {
    const std::pair<const char*, std::function<double(double, double)>> scenes[] = {
        {"ramp", dem::ramp_height_m},
        {"hill", dem::hill_height_m},
        {"shelf", dem::shelf_height_m},
        {"sea", dem::sea_height_m}};
    for (const auto& [name, height] : scenes) {
        const Rgb img = read_png(dem::encode_dem_png(height));
        ASSERT_EQ(img.width, 512) << name;
        ASSERT_EQ(img.height, 512) << name;
        double worst = 0.0;
        for (int py = 0; py < 512; ++py) {
            for (int px = 0; px < 512; ++px) {
                const uint8_t* p = &img.pixels[(size_t(py) * 512 + size_t(px)) * 3];
                const double decoded = -10000.0 + (p[0] * 65536.0 + p[1] * 256.0 + p[2]) * 0.1;
                worst = std::max(worst, std::abs(decoded - height(px + 0.5, py + 0.5)));
            }
        }
        EXPECT_LE(worst, 0.05 + 1e-6) << name << ": rounded to 0.1 m, so within 0.05 m";
    }
    EXPECT_DOUBLE_EQ(dem::ramp_height_m(0.0, 300.0), 0.0);
    EXPECT_DOUBLE_EQ(dem::ramp_height_m(512.0, 10.0), 20.0);
    EXPECT_DOUBLE_EQ(dem::hill_height_m(256.0, 256.0), 40.0);
    EXPECT_NEAR(dem::hill_height_m(0.0, 0.0), 40.0 * std::exp(-0.4096), 1e-9) << "sigma 400 px";
    EXPECT_DOUBLE_EQ(dem::shelf_height_m(100.0, 255.5), 10.0);
    EXPECT_DOUBLE_EQ(dem::shelf_height_m(100.0, 256.5), 0.0);
    EXPECT_DOUBLE_EQ(dem::sea_height_m(17.0, 400.0), 0.0);
}

TEST(TerrainFixtures, FlatPngDecodesToItsColour) {
    for (const uint32_t size : {512u, 256u}) {
        const Rgb img = read_png(imagery::flat_png(size, 10, 200, 30));
        ASSERT_EQ(img.width, static_cast<int>(size));
        ASSERT_EQ(img.height, static_cast<int>(size));
        for (size_t k = 0; k < img.pixels.size(); k += 3) {
            ASSERT_EQ(img.pixels[k], 10) << "pixel " << k / 3;
            ASSERT_EQ(img.pixels[k + 1], 200) << "pixel " << k / 3;
            ASSERT_EQ(img.pixels[k + 2], 30) << "pixel " << k / 3;
        }
    }
}

TEST(TerrainFixtures, QuantizedMeshHeaderAndCounts) {
    // Level 0, x 1 spans longitude 0..180 and latitude -90..90: its centre is (lon 90, lat 0).
    const std::vector<uint8_t> b = qmesh::encode_quantized_mesh(0, 1, 0, {8, 0.0, 0.0});
    const Qmesh q = parse(b);
    EXPECT_NEAR(q.center[0], 0.0, 1e-6);
    EXPECT_NEAR(q.center[1], 6378137.0, 1e-6);
    EXPECT_NEAR(q.center[2], 0.0, 1e-6);
    for (int c = 0; c < 3; ++c) EXPECT_EQ(q.bs_center[c], q.center[c]);
    // The lattice holds the corner at (lon 0, lat 0), ECEF (a, 0, 0), a * sqrt(2) from the centre.
    EXPECT_GE(q.bs_radius, 6378137.0 * std::sqrt(2.0) - 1e-3);
    EXPECT_EQ(q.min_h, 0.0f);
    EXPECT_EQ(q.max_h, 0.0f);
    EXPECT_EQ(q.vertex_count, 81u);     // 9 x 9
    EXPECT_EQ(q.triangle_count, 128u);  // 8 x 8 cells, two each
    for (const std::vector<uint16_t>& edge : q.edges) EXPECT_EQ(edge.size(), 9u);
    // 88 + 4 + 3 * 81 * 2 + 4 + 128 * 3 * 2 + 4 * (4 + 9 * 2): nothing after the edges.
    EXPECT_EQ(b.size(), 1438u);
    EXPECT_EQ(q.bytes_read, b.size());

    const Qmesh ramp = parse(qmesh::encode_quantized_mesh(14, 17000, 6000, {4, 10.0, 50.0}));
    EXPECT_EQ(ramp.min_h, 10.0f);
    EXPECT_EQ(ramp.max_h, 50.0f);
    EXPECT_EQ(ramp.vertex_count, 25u);
    EXPECT_TRUE(qmesh::encode_quantized_mesh(14, 0, 0, {0, 0.0, 0.0}).empty());
    EXPECT_TRUE(qmesh::encode_quantized_mesh(14, 0, 0, {256, 0.0, 0.0}).empty());
    EXPECT_FALSE(qmesh::encode_quantized_mesh(14, 0, 0, {255, 0.0, 0.0}).empty());
}

TEST(TerrainFixtures, QuantizedMeshVerticesDecodeFromZigzagDeltas) {
    for (const bool sloped : {false, true}) {
        const Qmesh q =
            parse(qmesh::encode_quantized_mesh(15, 40000, 12000, {8, 10.0, sloped ? 50.0 : 10.0}));
        std::set<std::pair<uint16_t, uint16_t>> lattice;
        for (uint32_t k = 0; k < q.vertex_count; ++k) {
            ASSERT_LE(q.u[k], 32767) << "vertex " << k;
            ASSERT_LE(q.v[k], 32767) << "vertex " << k;
            lattice.insert({q.u[k], q.v[k]});
            // Flat: every height code 0. The ramp rises west to east, so height tracks u.
            EXPECT_EQ(q.h[k], sloped ? q.u[k] : 0) << "vertex " << k;
        }
        ASSERT_EQ(lattice.size(), 81u) << "each lattice point once";
        std::set<uint16_t> steps;
        for (const auto& [u, v] : lattice) {
            steps.insert(u);
            steps.insert(v);
        }
        EXPECT_EQ(steps,
                  (std::set<uint16_t>{0, 4096, 8192, 12288, 16384, 20479, 24575, 28671, 32767}));
    }
}

TEST(TerrainFixtures, QuantizedMeshIndicesDecodeWithTheHighWaterMark) {
    const Qmesh q = parse(qmesh::encode_quantized_mesh(15, 40000, 12000, {8, 0.0, 0.0}));
    ASSERT_EQ(q.indices.size(), 384u);
    EXPECT_EQ(q.indices[0], 0u) << "the first index is a new vertex";
    std::set<uint32_t> used;
    for (size_t t = 0; t < q.indices.size(); t += 3) {
        const uint32_t a = q.indices[t], b = q.indices[t + 1], c = q.indices[t + 2];
        ASSERT_LT(std::max({a, b, c}), q.vertex_count) << "triangle " << t / 3;
        used.insert({a, b, c});
        // Counter-clockwise seen from above: u east, v north.
        const double cross = (double(q.u[b]) - q.u[a]) * (double(q.v[c]) - q.v[a]) -
                             (double(q.v[b]) - q.v[a]) * (double(q.u[c]) - q.u[a]);
        EXPECT_GT(cross, 0.0) << "triangle " << t / 3;
    }
    EXPECT_EQ(used.size(), size_t{q.vertex_count}) << "every vertex in some triangle";
    const char* names[] = {"west", "south", "east", "north"};
    for (int e = 0; e < 4; ++e) {
        std::set<uint16_t> distinct(q.edges[e].begin(), q.edges[e].end());
        EXPECT_EQ(distinct.size(), 9u) << names[e];
        for (const uint16_t k : q.edges[e]) {
            ASSERT_LT(k, q.vertex_count) << names[e];
            const uint16_t along = (e % 2 == 0) ? q.u[k] : q.v[k];
            EXPECT_EQ(along, e < 2 ? 0 : 32767) << names[e] << " edge vertex " << k;
        }
    }
}

TEST(TerrainFixtures, LayerJsonParsesAndListsTheAvailableTiles) {
    EXPECT_FALSE(parses_as_json("{\"a\":1 \"b\":2}")) << "the checker sees a missing comma";
    EXPECT_FALSE(parses_as_json("[1,]")) << "and a trailing one";
    EXPECT_TRUE(parses_as_json("{\"a\":[1,-2.5e3,true,null,\"x\\\"y\"]}"));

    const std::string json = qmesh::layer_json({{0, 0, 0}, {0, 1, 0}, {2, 5, 3}});
    EXPECT_TRUE(parses_as_json(json)) << json;
    EXPECT_TRUE(contains(json,
                         "\"available\":[[{\"startX\":0,\"startY\":0,\"endX\":0,\"endY\":0},"
                         "{\"startX\":1,\"startY\":0,\"endX\":1,\"endY\":0}],[],"
                         "[{\"startX\":5,\"startY\":3,\"endX\":5,\"endY\":3}]]"))
        << json;
    EXPECT_TRUE(contains(json, "\"format\":\"quantized-mesh-1.0\"")) << json;
    EXPECT_TRUE(contains(json, "\"projection\":\"EPSG:4326\"")) << json;
    EXPECT_TRUE(contains(json, "\"tiles\":[\"{z}/{x}/{y}.terrain?v={version}\"]")) << json;
    EXPECT_TRUE(contains(json, "\"maxzoom\":2")) << json;
    EXPECT_TRUE(parses_as_json(qmesh::layer_json({}))) << "no tiles: one empty level";
}

TEST(TerrainFixtures, IonEndpointBodiesParseWithTheirTypes) {
    const std::string terrain = qmesh::ion_terrain_endpoint_json("https://assets.fixture/1/");
    const std::string imagery = qmesh::ion_imagery_endpoint_json("https://assets.fixture/2/");
    for (const std::string* body : {&terrain, &imagery}) {
        EXPECT_TRUE(parses_as_json(*body)) << *body;
        EXPECT_TRUE(
            contains(*body, std::string("\"accessToken\":\"") + qmesh::kFixtureAssetToken + "\""))
            << *body;
        EXPECT_FALSE(contains(*body, "externalType")) << *body;
    }
    EXPECT_TRUE(contains(terrain, "\"type\":\"TERRAIN\",\"url\":\"https://assets.fixture/1/\""));
    EXPECT_TRUE(contains(terrain, std::string("\"html\":\"") + qmesh::kTerrainAttributionHtml));
    EXPECT_TRUE(contains(imagery, "\"type\":\"IMAGERY\",\"url\":\"https://assets.fixture/2/\""));
    EXPECT_TRUE(contains(imagery, std::string("\"html\":\"") + qmesh::kImageryAttributionHtml));
}

TEST(TerrainFixtures, TileMapResourceListsGeodeticLevels) {
    const std::string xml = qmesh::tilemapresource_xml(256);
    EXPECT_EQ(xml.rfind("<?xml version=\"1.0\"", 0), 0u);
    EXPECT_EQ(xml.substr(xml.size() - 11), "</TileMap>\n");
    EXPECT_TRUE(contains(xml, "<SRS>EPSG:4326</SRS>"));
    EXPECT_TRUE(contains(xml, "<TileSets profile=\"geodetic\">"));
    EXPECT_TRUE(contains(xml,
                         "<TileFormat width=\"256\" height=\"256\" mime-type=\"image/png\" "
                         "extension=\"png\"/>"));
    size_t tilesets = 0;
    for (size_t at = xml.find("<TileSet "); at != std::string::npos;
         at = xml.find("<TileSet ", at + 1)) {
        ++tilesets;
    }
    EXPECT_EQ(tilesets, size_t{qmesh::kTmsMaxLevel} + 1);
    for (uint32_t level = 0; level <= qmesh::kTmsMaxLevel; ++level) {
        const std::string l = std::to_string(level);
        EXPECT_TRUE(contains(xml, "<TileSet href=\"" + l + "\" order=\"" + l + "\"/>")) << level;
    }
    EXPECT_TRUE(contains(qmesh::tilemapresource_xml(512), "width=\"512\" height=\"512\""));
}
