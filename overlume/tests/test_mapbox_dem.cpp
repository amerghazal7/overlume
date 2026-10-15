// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "imagery_fixture.hpp"
#include "mapbox_dem.hpp"
#include "mapbox_tile_frame.hpp"

#include "stb_image_write.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace mb = overlume::mapbox;

constexpr uint32_t kSide = mb::kDemGridCells + 1;           // 65 grid vertices per side
constexpr uint32_t kGridVertices = kSide * kSide;           // 4225
constexpr uint32_t kSkirtVertices = 4 * mb::kDemGridCells;  // 256: each border vertex once
// The operating area's z14 tile (test_mapbox_tile_frame.cpp), in a frame 12 m up so no height
// hides behind a zero.
const mb::TileId kTile{14, 10712, 7012};
constexpr double kOriginHeightM = 12.0;

using overlume::testing::imagery::png;

mb::DemTile decode(const std::vector<uint8_t>& bytes) {
    return mb::decode_dem_png(bytes.data(), bytes.size());
}

// A decoded tile whose pixel (px, py) holds f(px, py), without a PNG in between.
mb::DemTile tile_of(const std::function<float(uint32_t px, uint32_t py)>& f) {
    mb::DemTile t;
    t.ok = true;
    t.height_m.resize(size_t{mb::kDemSize} * mb::kDemSize);
    for (uint32_t py = 0; py < mb::kDemSize; ++py) {
        for (uint32_t px = 0; px < mb::kDemSize; ++px) {
            t.height_m[size_t{py} * mb::kDemSize + px] = f(px, py);
        }
    }
    return t;
}

// 0 m at the west edge, 20 m at the east edge.
mb::DemTile ramp() {
    return tile_of([](uint32_t px, uint32_t) { return 20.0f * (px + 0.5f) / mb::kDemSize; });
}

// 40 m at the tile centre, falling off over 100 px.
mb::DemTile hill() {
    return tile_of([](uint32_t px, uint32_t py) {
        const double du = px + 0.5 - 256.0, dv = py + 0.5 - 256.0;
        return static_cast<float>(40.0 * std::exp(-(du * du + dv * dv) / (2.0 * 100.0 * 100.0)));
    });
}

std::array<double, 3> sub(const std::array<float, 3>& a, const std::array<float, 3>& b) {
    return {double{a[0]} - b[0], double{a[1]} - b[1], double{a[2]} - b[2]};
}

std::array<double, 3> cross(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

double norm(const std::array<double, 3>& a) {
    return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

// A tile-frame point in ECEF, through the frame's column-major frame_to_ecef.
std::array<double, 3> to_ecef(const mb::TileFrame& f, const std::array<float, 3>& p) {
    std::array<double, 3> out{};
    for (size_t r = 0; r < 3; ++r) {
        out[r] = f.frame_to_ecef[r] * p[0] + f.frame_to_ecef[4 + r] * p[1] +
                 f.frame_to_ecef[8 + r] * p[2] + f.frame_to_ecef[12 + r];
    }
    return out;
}

bool on_border(uint32_t g) {
    const uint32_t i = g % kSide, j = g / kSide;
    return i == 0 || j == 0 || i == kSide - 1 || j == kSide - 1;
}

}

TEST(MapboxDem, KnownPixelsDecodeToHeights) {
    for (const int channels : {3, 4}) {
        std::vector<uint8_t> pixels(size_t{mb::kDemSize} * mb::kDemSize * channels, 0);
        const auto set = [&](uint32_t px, uint32_t py, uint8_t r, uint8_t g, uint8_t b) {
            uint8_t* p = &pixels[(size_t{py} * mb::kDemSize + px) * channels];
            p[0] = r;
            p[1] = g;
            p[2] = b;
            if (channels == 4) p[3] = 7;  // alpha plays no part
        };
        set(511, 0, 2, 34, 224);       // 140000 -> 4000 m
        set(0, 511, 1, 134, 160);      // 100000 -> sea level
        set(511, 511, 255, 255, 255);  // 16777215 -> the code's ceiling
        set(100, 200, 1, 135, 6);      // 100102 -> 10.2 m
        const mb::DemTile t = decode(png(512, 512, channels, pixels));
        ASSERT_TRUE(t.ok) << t.error;
        EXPECT_TRUE(t.error.empty());
        ASSERT_EQ(t.height_m.size(), size_t{512} * 512);
        EXPECT_FLOAT_EQ(t.at(0, 0), -10000.0f) << "R = G = B = 0 is -10000 m";
        EXPECT_FLOAT_EQ(t.at(511, 0), 4000.0f) << "row 0 is the north edge, px 511 the east";
        EXPECT_FLOAT_EQ(t.at(0, 511), 0.0f);
        EXPECT_FLOAT_EQ(t.at(511, 511), 1667721.5f);
        EXPECT_NEAR(t.at(100, 200), 10.2f, 1e-3f);
        EXPECT_FLOAT_EQ(t.at(101, 200), -10000.0f);
    }
}

TEST(MapboxDem, MalformedTilesAreRejectedWithReasons) {
    const std::vector<uint8_t> rgb(size_t{512} * 512 * 3, 100);
    const std::vector<uint8_t> valid = png(512, 512, 3, rgb);
    std::vector<uint8_t> deep = valid;
    deep[24] = 16;  // IHDR bit depth (8 signature + 8 chunk header + 8 width/height): 16-bit RGB
    std::vector<uint8_t> jpeg;
    stbi_write_jpg_to_func(
        [](void* context, void* data, int size) {
            auto* bytes = static_cast<std::vector<uint8_t>*>(context);
            bytes->insert(bytes->end(), static_cast<uint8_t*>(data),
                          static_cast<uint8_t*>(data) + size);
        },
        &jpeg, 512, 512, 3, rgb.data(), 90);
    const std::string text = "<html>404 Not Found</html>";
    struct Case {
        const char* name;
        std::vector<uint8_t> bytes;
        const char* error;
    };
    const Case cases[] = {
        {"empty", {}, "DEM tile is empty"},
        {"text", {text.begin(), text.end()}, "DEM tile does not decode as PNG"},
        {"jpeg", jpeg, "DEM tile does not decode as PNG"},
        {"truncated",
         {valid.begin(), valid.begin() + static_cast<std::ptrdiff_t>(valid.size() / 2)},
         "DEM tile does not decode as PNG"},
        {"256 px", png(256, 256, 3, std::vector<uint8_t>(size_t{256} * 256 * 3, 100)),
         "DEM tile is not 512 x 512 pixels"},
        {"512 x 256", png(512, 256, 3, std::vector<uint8_t>(size_t{512} * 256 * 3, 100)),
         "DEM tile is not 512 x 512 pixels"},
        {"grey", png(512, 512, 1, std::vector<uint8_t>(size_t{512} * 512, 100)),
         "DEM tile has fewer than 3 channels"},
        {"grey + alpha", png(512, 512, 2, std::vector<uint8_t>(size_t{512} * 512 * 2, 100)),
         "DEM tile has fewer than 3 channels"},
        {"16-bit", deep, "DEM tile is not 8-bit"},
    };
    for (const Case& c : cases) {
        const mb::DemTile t = decode(c.bytes);
        EXPECT_FALSE(t.ok) << c.name;
        EXPECT_EQ(t.error, c.error) << c.name;
        EXPECT_TRUE(t.height_m.empty()) << c.name;
    }
    EXPECT_EQ(mb::decode_dem_png(nullptr, 10).error, "DEM tile is empty");
    EXPECT_TRUE(decode(valid).ok) << "the unmutated tile decodes";
}

TEST(MapboxDem, SamplingIsBilinearBetweenPixelCentres) {
    mb::DemTile cell = tile_of([](uint32_t, uint32_t) { return 0.0f; });
    cell.height_m[20 * 512 + 11] = 10.0f;  // (11, 20)
    cell.height_m[21 * 512 + 10] = 20.0f;  // (10, 21)
    cell.height_m[21 * 512 + 11] = 30.0f;  // (11, 21)
    EXPECT_DOUBLE_EQ(mb::dem_sample(cell, 10.5, 20.5), 0.0) << "a pixel centre is that pixel";
    EXPECT_DOUBLE_EQ(mb::dem_sample(cell, 11.5, 21.5), 30.0);
    EXPECT_DOUBLE_EQ(mb::dem_sample(cell, 10.75, 20.5), 2.5) << "a quarter of the way east";
    EXPECT_DOUBLE_EQ(mb::dem_sample(cell, 11.0, 21.0), 15.0) << "the mean of the four";
    EXPECT_DOUBLE_EQ(mb::dem_sample(cell, 10.5, 21.0), 10.0);

    const mb::DemTile plane =
        tile_of([](uint32_t px, uint32_t py) { return static_cast<float>(px + 1000.0 * py); });
    EXPECT_NEAR(mb::dem_sample(plane, 100.25, 300.75), 99.75 + 1000.0 * 300.25, 1e-6);
    EXPECT_NEAR(mb::dem_sample(plane, 0.0, 0.0), 0.0, 1e-9) << "clamped to the corner pixel";
    EXPECT_NEAR(mb::dem_sample(plane, 512.0, 512.0), 511.0 + 511000.0, 1e-9);
    EXPECT_NEAR(mb::dem_sample(plane, -40.0, 600.0), 511000.0, 1e-9);
    EXPECT_NEAR(mb::dem_sample(plane, 512.0, 256.0), 511.0 + 1000.0 * 255.5, 1e-6);
    EXPECT_EQ(mb::dem_sample(mb::DemTile{}, 256.0, 256.0), 0.0) << "an undecoded tile reads 0";
}

TEST(MapboxDem, MeshHasTheGridThenTheSkirtRing) {
    const mb::DemMesh m =
        mb::mesh_dem(hill(), mb::make_tile_frame(kTile, kOriginHeightM), kTile, 0.0);
    EXPECT_EQ(m.grid_vertex_count, kGridVertices);
    EXPECT_EQ(m.positions.size(), size_t{kGridVertices + kSkirtVertices});  // 4481
    EXPECT_EQ(m.normals.size(), m.positions.size());
    EXPECT_EQ(m.uvs.size(), m.positions.size());
    // 64 x 64 cells of two triangles, then two triangles per border segment.
    EXPECT_EQ(m.indices.size(), size_t{64 * 64 * 6 + kSkirtVertices * 6});  // 26112
    EXPECT_LT(*std::max_element(m.indices.begin(), m.indices.end()), m.positions.size());
}

TEST(MapboxDem, GridVerticesFollowTheDemThroughTheTileFrame) {
    const mb::TileFrame f = mb::make_tile_frame(kTile, kOriginHeightM);
    const mb::DemTile dem = hill();
    constexpr double kZRef = 15.0;
    const mb::DemMesh m = mb::mesh_dem(dem, f, kTile, kZRef);
    for (const auto [i, j] :
         {std::array<uint32_t, 2>{0, 0}, {64, 0}, {0, 64}, {64, 64}, {32, 32}, {1, 0}, {17, 45}}) {
        const double u = 8.0 * i, v = 8.0 * j;
        const std::array<double, 3> want =
            mb::project_to_frame(f, kTile, u, v, mb::kDemSize, mb::dem_sample(dem, u, v) - kZRef);
        const std::array<float, 3>& got = m.positions[j * kSide + i];
        for (size_t c = 0; c < 3; ++c) {
            EXPECT_NEAR(got[c], want[c], 2e-3) << "vertex (" << i << ", " << j << ") axis " << c;
        }
    }
    // The tile centre is the frame origin: z there is the height minus z_ref.
    const mb::DemTile flat = tile_of([](uint32_t, uint32_t) { return 100.0f; });
    const mb::DemMesh at_ref = mb::mesh_dem(flat, f, kTile, 100.0);
    const mb::DemMesh at_zero = mb::mesh_dem(flat, f, kTile, 0.0);
    const uint32_t centre = 32 * kSide + 32;
    for (size_t c = 0; c < 3; ++c) EXPECT_NEAR(at_ref.positions[centre][c], 0.0, 1e-3);
    EXPECT_NEAR(at_zero.positions[centre][2], 100.0, 1e-3);
    EXPECT_NEAR(at_zero.positions[centre][0], 0.0, 1e-3);
}

TEST(MapboxDem, EdgeVerticesSitOnTheTileBoundary) {
    const mb::DemTile sea = tile_of([](uint32_t, uint32_t) { return 0.0f; });
    const mb::TileId east{14, kTile.x + 1, kTile.y}, south{14, kTile.x, kTile.y + 1};
    const mb::TileFrame f = mb::make_tile_frame(kTile, kOriginHeightM);
    const mb::TileFrame fe = mb::make_tile_frame(east, kOriginHeightM);
    const mb::TileFrame fs = mb::make_tile_frame(south, kOriginHeightM);
    const mb::DemMesh m = mb::mesh_dem(sea, f, kTile, 0.0);
    const mb::DemMesh me = mb::mesh_dem(sea, fe, east, 0.0);
    const mb::DemMesh ms = mb::mesh_dem(sea, fs, south, 0.0);
    double worst = 0.0;
    for (uint32_t k = 0; k < kSide; ++k) {
        const std::array<double, 3> a = to_ecef(f, m.positions[k * kSide + 64]);   // our east edge
        const std::array<double, 3> b = to_ecef(fe, me.positions[k * kSide + 0]);  // its west edge
        const std::array<double, 3> c = to_ecef(f, m.positions[64 * kSide + k]);   // our south edge
        const std::array<double, 3> d = to_ecef(fs, ms.positions[0 * kSide + k]);  // its north edge
        worst = std::max({worst, norm({a[0] - b[0], a[1] - b[1], a[2] - b[2]}),
                          norm({c[0] - d[0], c[1] - d[1], c[2] - d[2]})});
    }
    EXPECT_LT(worst, 0.01) << "neighbouring tiles share their edge vertices (metres apart)";
}

TEST(MapboxDem, SkirtHangsSkirtMetresBelowEveryEdgeVertex) {
    const mb::TileFrame f = mb::make_tile_frame(kTile, kOriginHeightM);
    // The default argument is pinned to 20 m (Review Focus 1); 5 m proves the argument is used.
    const std::pair<mb::DemMesh, double> cases[] = {
        {mb::mesh_dem(hill(), f, kTile, 0.0), 20.0},
        {mb::mesh_dem(hill(), f, kTile, 0.0, 5.0), 5.0}};
    for (const auto& [m, skirt] : cases) {
        std::vector<int> hits(kGridVertices, 0);
        for (uint32_t s = kGridVertices; s < m.positions.size(); ++s) {
            uint32_t nearest = 0;
            double best = 1e300;
            for (uint32_t g = 0; g < kGridVertices; ++g) {
                const double d = norm(sub(m.positions[s], m.positions[g]));
                if (d < best) {
                    best = d;
                    nearest = g;
                }
            }
            ++hits[nearest];
            EXPECT_TRUE(on_border(nearest)) << "skirt vertex " << s;
            EXPECT_NEAR(best, skirt, 0.01) << "skirt vertex " << s;
            EXPECT_NEAR(m.positions[nearest][2] - m.positions[s][2], skirt, 0.01)
                << "skirt vertex " << s << " hangs below its edge vertex";
            EXPECT_EQ(m.normals[s], m.normals[nearest]) << "skirt vertex " << s;
            EXPECT_EQ(m.uvs[s], m.uvs[nearest]) << "skirt vertex " << s;
        }
        for (uint32_t g = 0; g < kGridVertices; ++g) {
            EXPECT_EQ(hits[g], on_border(g) ? 1 : 0) << "grid vertex " << g << " skirt " << skirt;
        }
    }
}

TEST(MapboxDem, NormalsAreUnitAndFollowTheSlope) {
    const mb::TileFrame f = mb::make_tile_frame(kTile, kOriginHeightM);
    for (const mb::DemTile& dem : {hill(), ramp()}) {
        const mb::DemMesh m = mb::mesh_dem(dem, f, kTile, 0.0);
        for (size_t k = 0; k < m.normals.size(); ++k) {
            const std::array<float, 3>& n = m.normals[k];
            ASSERT_NEAR(norm({n[0], n[1], n[2]}), 1.0, 1e-5) << "normal " << k;
            ASSERT_GT(n[2], 0.99) << "normal " << k << " points up";
        }
    }
    // The ramp rises 20 m over the tile's width: at the centre the normal leans west by that slope.
    const mb::DemMesh m = mb::mesh_dem(ramp(), f, kTile, 0.0);
    const std::array<double, 3> w = mb::project_to_frame(f, kTile, 0.0, 256.0, mb::kDemSize, 0.0);
    const std::array<double, 3> e = mb::project_to_frame(f, kTile, 512.0, 256.0, mb::kDemSize, 0.0);
    const double slope = 20.0 / (e[0] - w[0]);
    const std::array<float, 3>& n = m.normals[32 * kSide + 32];
    EXPECT_NEAR(n[0], -slope / std::sqrt(1.0 + slope * slope), 2e-5);
    EXPECT_NEAR(n[1], 0.0, 2e-5);
    // On the hill's east flank the ground falls away eastward, so the normal leans east.
    EXPECT_GT(mb::mesh_dem(hill(), f, kTile, 0.0).normals[32 * kSide + 40][0], 0.01);
}

TEST(MapboxDem, TrianglesFaceUpAndTheSkirtFacesOutward) {
    const mb::DemMesh m =
        mb::mesh_dem(hill(), mb::make_tile_frame(kTile, kOriginHeightM), kTile, 0.0);
    const size_t grid_indices = size_t{64} * 64 * 6;
    for (size_t t = 0; t < m.indices.size(); t += 3) {
        const std::array<float, 3>& a = m.positions[m.indices[t]];
        const std::array<float, 3>& b = m.positions[m.indices[t + 1]];
        const std::array<float, 3>& c = m.positions[m.indices[t + 2]];
        const std::array<double, 3> n = cross(sub(b, a), sub(c, a));
        if (t < grid_indices) {
            ASSERT_GT(n[2], 0.0) << "grid triangle " << t / 3 << " is counter-clockwise from above";
        } else {
            // The frame origin is the tile centre, so outward is away from (0, 0).
            const double cx = (a[0] + b[0] + c[0]) / 3.0, cy = (a[1] + b[1] + c[1]) / 3.0;
            ASSERT_GT(n[0] * cx + n[1] * cy, 0.0) << "skirt triangle " << t / 3 << " faces out";
        }
    }
}

TEST(MapboxDem, UvsSpanTheTileEastAndSouth) {
    const mb::DemMesh m =
        mb::mesh_dem(hill(), mb::make_tile_frame(kTile, kOriginHeightM), kTile, 0.0);
    for (const std::array<float, 2>& uv : m.uvs) {
        ASSERT_GE(uv[0], 0.0f);
        ASSERT_LE(uv[0], 1.0f);
        ASSERT_GE(uv[1], 0.0f);
        ASSERT_LE(uv[1], 1.0f);
    }
    EXPECT_EQ(m.uvs[0], (std::array<float, 2>{0.0f, 0.0f})) << "north-west corner";
    EXPECT_EQ(m.uvs[64], (std::array<float, 2>{1.0f, 0.0f})) << "north-east corner";
    EXPECT_EQ(m.uvs[64 * kSide], (std::array<float, 2>{0.0f, 1.0f})) << "south-west corner";
    EXPECT_EQ(m.uvs[17 * kSide + 9], (std::array<float, 2>{9.0f / 64, 17.0f / 64}));
}

TEST(MapboxDem, AnUndecodedTileMeshesToNothing) {
    // The guard has two halves: an empty DemTile{} fails the height count, and this tile, with
    // every height present but ok = false, fails only the flag.
    mb::DemTile flagged_off = tile_of([](uint32_t, uint32_t) { return 5.0f; });
    flagged_off.ok = false;
    for (const mb::DemTile& dem : {mb::DemTile{}, flagged_off}) {
        const mb::DemMesh m = mb::mesh_dem(dem, mb::make_tile_frame(kTile, 0.0), kTile, 0.0);
        EXPECT_EQ(m.grid_vertex_count, 0u) << dem.height_m.size() << " heights";
        EXPECT_TRUE(m.positions.empty());
        EXPECT_TRUE(m.normals.empty());
        EXPECT_TRUE(m.uvs.empty());
        EXPECT_TRUE(m.indices.empty());
    }
}
