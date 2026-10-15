// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_extrude.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

using overlume::mapbox::ExtrudeStats;
using overlume::mapbox::TilePolygon;
using overlume::mapbox::TileRing;
using overlume::mapbox::TriangleMesh;
using overlume::mapbox::extrude_polygons;
using overlume::mapbox::footprint_centroid;

namespace {

constexpr double kExtent = 4096.0;
constexpr double kMetresPerUnit = 0.25;
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// A linear stand-in for the tile-frame chain. Like the real one it mirrors v: tile y runs south,
// output y north.
std::array<double, 3> project(double u, double v, double z_m) {
    return {u * kMetresPerUnit, -v * kMetresPerUnit, z_m};
}

ExtrudeStats extrude(const std::vector<TilePolygon>& polygons, double min_height_m, double height_m,
                     TriangleMesh& out) {
    return extrude_polygons(polygons, kExtent, min_height_m, height_m, project, out);
}

TilePolygon polygon(TileRing outer, std::vector<TileRing> holes = {}) {
    return {std::move(outer), std::move(holes)};
}

// The MVT exterior winding: positive area in tile units, so clockwise once mirrored into output
// space.
TileRing box(double u0, double v0, double u1, double v1) {
    return {{u0, v0}, {u1, v0}, {u1, v1}, {u0, v1}};
}

TileRing reversed(TileRing ring) {
    std::reverse(ring.begin(), ring.end());
    return ring;
}

struct V3 {
    double x = 0.0, y = 0.0, z = 0.0;
};

V3 operator-(const V3& a, const V3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }

double dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

V3 cross(const V3& a, const V3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

V3 position(const TriangleMesh& m, size_t i) {
    return {m.positions[3 * i], m.positions[3 * i + 1], m.positions[3 * i + 2]};
}

V3 normal(const TriangleMesh& m, size_t i) {
    return {m.normals[3 * i], m.normals[3 * i + 1], m.normals[3 * i + 2]};
}

struct Triangle {
    std::array<uint32_t, 3> v{};
    std::array<V3, 3> p{};
    V3 n;  // the first vertex's normal

    bool wall() const { return std::abs(n.z) < 0.5; }
    V3 centre() const {
        return {(p[0].x + p[1].x + p[2].x) / 3.0, (p[0].y + p[1].y + p[2].y) / 3.0,
                (p[0].z + p[1].z + p[2].z) / 3.0};
    }
    // Points to the side the triangle is counter-clockwise from.
    V3 winding() const { return cross(p[1] - p[0], p[2] - p[0]); }
    double area_from_above() const { return 0.5 * std::abs(winding().z); }
};

std::vector<Triangle> triangles(const TriangleMesh& m) {
    std::vector<Triangle> out;
    for (size_t k = 0; k + 2 < m.indices.size(); k += 3) {
        Triangle t;
        for (size_t c = 0; c < 3; ++c) {
            t.v[c] = m.indices[k + c];
            t.p[c] = position(m, t.v[c]);
        }
        t.n = normal(m, t.v[0]);
        out.push_back(t);
    }
    return out;
}

double roof_area(const TriangleMesh& m) {
    double area = 0.0;
    for (const Triangle& t : triangles(m)) {
        if (!t.wall()) area += t.area_from_above();
    }
    return area;
}

// Indices in range, unit normals shared by each face's three vertices, and every triangle
// counter-clockwise seen from the side its normal points to.
void expect_well_formed(const TriangleMesh& m) {
    ASSERT_EQ(m.positions.size(), m.normals.size());
    ASSERT_EQ(m.positions.size() % 3, 0u);
    ASSERT_EQ(m.indices.size() % 3, 0u);
    const size_t vertices = m.positions.size() / 3;
    for (uint32_t i : m.indices) ASSERT_LT(i, vertices);
    for (size_t i = 0; i < vertices; ++i) {
        const V3 n = normal(m, i);
        EXPECT_NEAR(std::sqrt(dot(n, n)), 1.0, 1e-6) << "vertex " << i;
    }
    for (const Triangle& t : triangles(m)) {
        for (size_t c = 1; c < 3; ++c) {
            const V3 n = normal(m, t.v[c]);
            EXPECT_TRUE(n.x == t.n.x && n.y == t.n.y && n.z == t.n.z)
                << "a face with mixed normals";
        }
        EXPECT_GT(dot(t.winding(), t.n), 0.0) << "a face clockwise seen from outside";
    }
}

}

TEST(MapboxExtrude, BoxGivesFourWallQuadsAndTwoRoofTriangles) {
    TriangleMesh m;
    const ExtrudeStats s = extrude({polygon(box(1000, 1000, 1100, 1100))}, 0.0, 10.0, m);
    EXPECT_EQ(s.polygons_in, 1u);
    EXPECT_EQ(s.polygons_built, 1u);
    EXPECT_EQ(s.wall_quads, 4u);
    EXPECT_EQ(s.roof_triangles, 2u);
    // Walls: 4 vertices and 6 indices per quad. Roof: the 4 corners and 2 triangles.
    EXPECT_EQ(m.positions.size(), (16u + 4u) * 3u);
    EXPECT_EQ(m.normals.size(), (16u + 4u) * 3u);
    EXPECT_EQ(m.indices.size(), 24u + 6u);
    expect_well_formed(m);
}

TEST(MapboxExtrude, WallsRunFromMinHeightToHeightAndThereIsNoFloor) {
    // A building standing on the ground, then a part floating from 20 m to 30 m.
    for (const auto& [min_m, height_m] : {std::pair{0.0, 10.0}, std::pair{20.0, 30.0}}) {
        TriangleMesh m;
        extrude({polygon(box(1000, 1000, 1100, 1100))}, min_m, height_m, m);
        size_t at_min = 0, at_top = 0;
        for (size_t i = 0; i < m.positions.size() / 3; ++i) {
            const V3 p = position(m, i);
            const V3 n = normal(m, i);
            if (std::abs(n.z) < 0.5) {
                EXPECT_TRUE(p.z == min_m || p.z == height_m) << "a wall vertex at " << p.z;
                if (p.z == min_m) ++at_min;
                if (p.z == height_m) ++at_top;
            } else {
                EXPECT_EQ(n.z, 1.0) << "a face that is neither wall nor roof";
                EXPECT_EQ(p.z, height_m);
            }
        }
        EXPECT_EQ(at_min, 8u);  // two per wall quad
        EXPECT_EQ(at_top, 8u);
    }
}

TEST(MapboxExtrude, EitherInputWindingGivesOutwardWallsAndAnUpwardRoof) {
    // box() comes out clockwise in output space and its reverse counter-clockwise: without
    // re-orientation, one of the two would have its walls facing in.
    const V3 centre{1050 * kMetresPerUnit, -1050 * kMetresPerUnit, 5.0};
    for (const TileRing& ring :
         {box(1000, 1000, 1100, 1100), reversed(box(1000, 1000, 1100, 1100))}) {
        TriangleMesh m;
        extrude({polygon(ring)}, 0.0, 10.0, m);
        expect_well_formed(m);
        size_t walls = 0, roofs = 0;
        for (const Triangle& t : triangles(m)) {
            if (t.wall()) {
                ++walls;
                EXPECT_EQ(t.n.z, 0.0);
                EXPECT_GT(dot(t.n, t.centre() - centre), 0.0) << "a wall facing in";
            } else {
                ++roofs;
                EXPECT_EQ(t.n.x, 0.0);
                EXPECT_EQ(t.n.y, 0.0);
                EXPECT_EQ(t.n.z, 1.0);
                EXPECT_GT(t.winding().z, 0.0) << "a roof triangle clockwise from above";
            }
        }
        EXPECT_EQ(walls, 8u);
        EXPECT_EQ(roofs, 2u);
    }
}

TEST(MapboxExtrude, CourtyardWallsFaceAwayFromTheSolidAndTheRoofLeavesTheHoleOpen) {
    // The hole in the MVT winding (against the exterior's), then reversed.
    const TileRing hole = reversed(box(1025, 1025, 1075, 1075));
    const V3 centre{1050 * kMetresPerUnit, -1050 * kMetresPerUnit, 5.0};
    for (const TileRing& h : {hole, reversed(hole)}) {
        TriangleMesh m;
        const ExtrudeStats s = extrude({polygon(box(1000, 1000, 1100, 1100), {h})}, 0.0, 10.0, m);
        EXPECT_EQ(s.wall_quads, 8u);
        expect_well_formed(m);
        size_t outer = 0, courtyard = 0;
        for (const Triangle& t : triangles(m)) {
            if (!t.wall()) continue;
            // The wall plane's signed distance from the centre, along the wall's normal.
            const double along = dot(t.n, t.centre() - centre);
            if (std::abs(std::abs(along) - 12.5) < 1e-3) {
                ++outer;
                EXPECT_GT(along, 0.0) << "an outer wall facing in";
            } else {
                ++courtyard;
                EXPECT_NEAR(along, -6.25, 1e-3) << "a courtyard wall facing the solid";
            }
        }
        EXPECT_EQ(outer, 8u);
        EXPECT_EQ(courtyard, 8u);
        // (100^2 - 50^2) units^2 at 0.25 m per unit.
        EXPECT_NEAR(roof_area(m), 468.75, 468.75e-6);
    }
}

TEST(MapboxExtrude, PolygonOutsideTheTileGivesNothing) {
    TriangleMesh m;
    const ExtrudeStats s = extrude(
        {polygon(box(-300, 1000, -200, 1100)), polygon(box(5000, 5000, 5100, 5100))}, 0.0, 10.0, m);
    EXPECT_EQ(s.polygons_in, 2u);
    EXPECT_EQ(s.polygons_built, 0u);
    EXPECT_EQ(s.wall_quads, 0u);
    EXPECT_EQ(s.roof_triangles, 0u);
    EXPECT_TRUE(m.positions.empty());
    EXPECT_TRUE(m.normals.empty());
    EXPECT_TRUE(m.indices.empty());
}

TEST(MapboxExtrude, NoWallOnTheClipEdgeSoNeighbouringTilesMeetWithoutSeamWalls) {
    // One 79 x 100-unit building seen from this tile and from its west neighbour, where u is
    // larger by the extent. u0 = -60 puts an unsnapped clip crossing 7e-15 off u == 0.
    TriangleMesh here, west;
    const ExtrudeStats a = extrude({polygon(box(-60, 1000, 19, 1100))}, 0.0, 10.0, here);
    const ExtrudeStats b =
        extrude({polygon(box(kExtent - 60, 1000, kExtent + 19, 1100))}, 0.0, 10.0, west);
    EXPECT_EQ(a.wall_quads, 3u);  // east, north and south
    EXPECT_EQ(b.wall_quads, 3u);  // west, north and south
    EXPECT_EQ(here.positions.size(), (12u + 4u) * 3u);
    EXPECT_EQ(here.indices.size(), 18u + 6u);
    expect_well_formed(here);
    expect_well_formed(west);
    for (const Triangle& t : triangles(here)) {
        if (t.wall()) {
            EXPECT_FALSE(t.p[0].x == 0.0 && t.p[1].x == 0.0 && t.p[2].x == 0.0);
        }
    }
    const double seam_x = kExtent * kMetresPerUnit;
    for (const Triangle& t : triangles(west)) {
        if (t.wall()) {
            EXPECT_FALSE(t.p[0].x == seam_x && t.p[1].x == seam_x && t.p[2].x == seam_x);
        }
    }
    // The two halves cover the footprint once: 19 and 60 units of its 79-unit width.
    EXPECT_NEAR(roof_area(here), 118.75, 118.75e-6);
    EXPECT_NEAR(roof_area(west), 375.0, 375e-6);
}

TEST(MapboxExtrude, CornerCrossingsGetWallsOnlyOnTheirInTileEdges) {
    // Over the north-west corner (u == 0, v == 0) and the south-east one (u == E, v == E).
    const double e = kExtent * kMetresPerUnit;
    const std::array<std::array<double, 2>, 2> seams = {{{0.0, 0.0}, {e, -e}}};  // output x, y
    const std::array<TileRing, 2> rings = {
        box(-60, -60, 19, 19), box(kExtent - 19, kExtent - 19, kExtent + 60, kExtent + 60)};
    for (size_t c = 0; c < 2; ++c) {
        TriangleMesh m;
        const ExtrudeStats s = extrude({polygon(rings[c])}, 0.0, 10.0, m);
        EXPECT_EQ(s.wall_quads, 2u);
        EXPECT_EQ(s.roof_triangles, 2u);
        expect_well_formed(m);
        for (const Triangle& t : triangles(m)) {
            if (!t.wall()) continue;
            for (size_t a = 0; a < 2; ++a) {
                const auto on = [&](const V3& p) { return (a == 0 ? p.x : p.y) == seams[c][a]; };
                EXPECT_FALSE(on(t.p[0]) && on(t.p[1]) && on(t.p[2])) << "a wall on a clip line";
            }
        }
    }
    // A cut across the corner: its ends lie on two different clip lines, so it keeps its wall.
    TriangleMesh m;
    const ExtrudeStats cut = extrude({polygon({{-10, 30}, {30, -10}, {-10, -10}})}, 0.0, 10.0, m);
    EXPECT_EQ(cut.wall_quads, 1u);
    EXPECT_EQ(cut.roof_triangles, 1u);
    expect_well_formed(m);
}

TEST(MapboxExtrude, HoleCrossingTheClipEdgeIsCutWithItsExterior) {
    TriangleMesh m;
    const ExtrudeStats s = extrude(
        {polygon(box(-40, 1010, 60, 1090), {reversed(box(-20, 1030, 20, 1070))})}, 0.0, 10.0, m);
    EXPECT_EQ(s.polygons_built, 1u);
    EXPECT_EQ(s.wall_quads, 6u);  // 3 outer and 3 courtyard walls, none on u == 0
    expect_well_formed(m);
    // (60 x 80 - 20 x 40) units^2 at 0.25 m per unit.
    EXPECT_NEAR(roof_area(m), 250.0, 250e-6);
}

TEST(MapboxExtrude, DegenerateRingsAndRepeatedVerticesAddNoFaces) {
    const TileRing line = {{1040, 1050}, {1060, 1050}, {1050, 1050}};
    TriangleMesh m;
    const ExtrudeStats s = extrude({polygon(line), polygon({{1000, 1000}, {1100, 1100}}),
                                    polygon(box(1000, 1000, 1100, 1100), {line})},
                                   0.0, 10.0, m);
    EXPECT_EQ(s.polygons_in, 3u);
    EXPECT_EQ(s.polygons_built, 1u);  // the collinear and the two-vertex exteriors are dropped,
    EXPECT_EQ(s.wall_quads, 4u);      // and so is the collinear hole
    EXPECT_EQ(s.roof_triangles, 2u);

    // A repeated vertex is a zero-length edge: no wall and no NaN normal.
    TriangleMesh r;
    const ExtrudeStats d =
        extrude({polygon({{1000, 1000}, {1100, 1000}, {1100, 1000}, {1100, 1100}, {1000, 1100}})},
                0.0, 10.0, r);
    EXPECT_EQ(d.wall_quads, 4u);
    EXPECT_EQ(d.roof_triangles, 2u);
    expect_well_formed(r);
}

TEST(MapboxExtrude, SelfTouchingAndSelfCrossingRingsGiveAWellFormedMesh) {
    // Two squares meeting at one corner, walked as one ring that visits the corner twice.
    const TileRing pinched = {{1000, 1000}, {1050, 1000}, {1050, 1050}, {1100, 1050},
                              {1100, 1100}, {1050, 1100}, {1050, 1050}, {1000, 1050}};
    // A twisted corner: the first and third edges cross at (1010, 1010). Earcut 2.2.4 cuts it in
    // its self-intersection pass, which emits a clockwise triangle that the roof must flip.
    const TileRing twisted = {{1020, 1000}, {1000, 1020}, {1020, 1020},
                              {1000, 1000}, {1060, 1080}, {1000, 1080}};
    for (const TileRing& ring : {pinched, twisted}) {
        TriangleMesh m;
        const ExtrudeStats s = extrude({polygon(ring)}, 0.0, 10.0, m);
        EXPECT_EQ(s.polygons_built, 1u);
        EXPECT_GT(s.roof_triangles, 0u);
        expect_well_formed(m);
    }
}

TEST(MapboxExtrude, NothingUnlessHeightsAreFiniteAndHeightIsAboveMinHeight) {
    const std::vector<std::pair<double, double>> cases = {{10.0, 10.0},  {12.0, 10.0}, {0.0, kInf},
                                                          {-kInf, 10.0}, {kNaN, 10.0}, {0.0, kNaN}};
    for (const auto& [min_m, height_m] : cases) {
        TriangleMesh m;
        const ExtrudeStats s = extrude({polygon(box(1000, 1000, 1100, 1100))}, min_m, height_m, m);
        EXPECT_EQ(s.polygons_in, 0u) << min_m << " " << height_m;
        EXPECT_EQ(s.polygons_built, 0u);
        EXPECT_EQ(s.wall_quads, 0u);
        EXPECT_EQ(s.roof_triangles, 0u);
        EXPECT_TRUE(m.positions.empty() && m.normals.empty() && m.indices.empty());
    }
}

TEST(MapboxExtrude, NonFiniteCoordinateSkipsItsPolygon) {
    TileRing bad_outer = box(1000, 1000, 1100, 1100);
    bad_outer[2][0] = kNaN;
    TileRing bad_hole = reversed(box(1225, 1025, 1275, 1075));
    bad_hole[1][1] = kInf;
    TriangleMesh m;
    const ExtrudeStats s =
        extrude({polygon(bad_outer), polygon(box(1200, 1000, 1300, 1100), {bad_hole}),
                 polygon(box(1400, 1000, 1500, 1100))},
                0.0, 10.0, m);
    EXPECT_EQ(s.polygons_in, 3u);
    EXPECT_EQ(s.polygons_built, 1u);
    EXPECT_EQ(s.wall_quads, 4u);
    EXPECT_EQ(s.roof_triangles, 2u);
    expect_well_formed(m);
    for (float f : m.positions) EXPECT_TRUE(std::isfinite(f));
}

TEST(MapboxExtrude, AppendsToTheMeshItIsGiven) {
    TriangleMesh m;
    extrude({polygon(box(1000, 1000, 1100, 1100))}, 0.0, 10.0, m);
    const TriangleMesh first = m;
    extrude({polygon(box(2000, 1000, 2100, 1100))}, 0.0, 10.0, m);
    ASSERT_EQ(m.positions.size(), 2 * first.positions.size());
    ASSERT_EQ(m.indices.size(), 2 * first.indices.size());
    EXPECT_TRUE(std::equal(first.positions.begin(), first.positions.end(), m.positions.begin()));
    EXPECT_TRUE(std::equal(first.indices.begin(), first.indices.end(), m.indices.begin()));
    // The same box 1000 units east: the same topology, offset past the first box's vertices.
    const uint32_t offset = static_cast<uint32_t>(first.positions.size() / 3);
    for (size_t k = 0; k < first.indices.size(); ++k) {
        EXPECT_EQ(m.indices[first.indices.size() + k], first.indices[k] + offset);
    }
    expect_well_formed(m);
}

TEST(MapboxExtrude, StatsCountPolygonsWallQuadsAndRoofTriangles) {
    TriangleMesh m;
    const ExtrudeStats s =
        extrude({polygon(box(1000, 1000, 1100, 1100)), polygon(box(-300, 1000, -200, 1100)),
                 polygon(box(-60, 2000, 19, 2100)),
                 polygon(box(3000, 3000, 3100, 3100), {reversed(box(3025, 3025, 3075, 3075))})},
                0.0, 10.0, m);
    EXPECT_EQ(s.polygons_in, 4u);
    EXPECT_EQ(s.polygons_built, 3u);  // not the one outside the tile
    EXPECT_EQ(s.wall_quads, 4u + 3u + 8u);
    EXPECT_EQ(s.roof_triangles, 2u + 2u + 8u);
    EXPECT_EQ(m.indices.size(), 3u * (2u * s.wall_quads + s.roof_triangles));
    // Roof vertices: the box's 4, the cut box's 4, the courtyard's 4 + 4.
    EXPECT_EQ(m.positions.size(), 3u * (4u * s.wall_quads + 4u + 4u + 8u));
}

TEST(MapboxExtrude, FootprintCentroidWeighsUnclippedExteriorsByArea) {
    const auto expect_at = [](const std::vector<TilePolygon>& polygons, double u, double v) {
        const auto c = footprint_centroid(polygons);
        ASSERT_TRUE(c.has_value());
        EXPECT_NEAR((*c)[0], u, 1e-9);
        EXPECT_NEAR((*c)[1], v, 1e-9);
    };
    expect_at({polygon(box(1000, 1000, 1100, 1100))}, 1050.0, 1050.0);
    // An L: a 20 x 10 bar and a 10 x 10 foot, in either winding.
    const TileRing ell = {{0, 0}, {20, 0}, {20, 10}, {10, 10}, {10, 20}, {0, 20}};
    expect_at({polygon(ell)}, 25.0 / 3.0, 25.0 / 3.0);
    expect_at({polygon(reversed(ell))}, 25.0 / 3.0, 25.0 / 3.0);
    // Two parts of opposite winding weighing 100 and 400; the hole does not count.
    expect_at({polygon(box(0, 0, 10, 10), {reversed(box(2, 2, 8, 8))}),
               polygon(reversed(box(20, 0, 40, 20)))},
              25.0, 9.0);
    // Not clipped to the tile.
    expect_at({polygon(box(-50, 1000, 50, 1100))}, 0.0, 1050.0);
    // A part with a non-finite coordinate is left out.
    TileRing bad = box(1200, 1000, 1300, 1100);
    bad[0][1] = kNaN;
    expect_at({polygon(box(1000, 1000, 1100, 1100)), polygon(bad)}, 1050.0, 1050.0);
}

TEST(MapboxExtrude, FootprintCentroidOfNoAreaIsNullopt) {
    EXPECT_FALSE(footprint_centroid({}).has_value());
    EXPECT_FALSE(footprint_centroid({polygon({})}).has_value());
    EXPECT_FALSE(footprint_centroid({polygon({{0, 0}, {10, 0}, {20, 0}})}).has_value());
    EXPECT_FALSE(footprint_centroid({polygon({{0, 0}, {10, 10}})}).has_value());
}
