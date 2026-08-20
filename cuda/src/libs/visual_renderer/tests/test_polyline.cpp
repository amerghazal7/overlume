// test_polyline.cpp — Epic 2 Task 2 (VM-024) Step 4/5: the shared
// polyline/polygon extruder. Pure geometry, no GPU, no GTEST_SKIP, and no
// Filament type anywhere in it or in polyline.hpp — see that header's own
// comment.
#include "polyline.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

using mpviz::Vec3;
using mpviz::detail::extrude_polyline;
using mpviz::detail::extrude_polyline_indices;
using mpviz::detail::polyline_chunks;
using mpviz::detail::triangulate_convex_polygon;

TEST(Polyline, StraightSegmentExtrudesToRectangleOfGivenWidth) {
    const Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    auto v = extrude_polyline(pts, 2, /*half_width=*/0.25f, /*z_lift=*/0.02f);
    ASSERT_EQ(v.size(), 4u);  // 2 points -> 2 pairs
    EXPECT_NEAR(v[0].y, -0.25, 1e-5);
    EXPECT_NEAR(v[1].y, 0.25, 1e-5);
    EXPECT_NEAR(v[0].z, 0.02, 1e-5);  // lifted off the ground: no z-fighting
    EXPECT_NEAR(v[2].y, -0.25, 1e-5);
    EXPECT_NEAR(v[3].y, 0.25, 1e-5);
    EXPECT_NEAR(v[0].x, 0.0, 1e-5);
    EXPECT_NEAR(v[2].x, 10.0, 1e-5);
}

TEST(Polyline, CornerMitresWithoutSelfIntersection) {
    // 90-degree corner: (0,0)->(10,0)->(10,10). The inner (concave) rail
    // vertex at the corner must stay on the concave side of both segments'
    // centerlines, not cross over to the convex side (the classic bowtie).
    const Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}, {10, 10, 0}};
    auto v = extrude_polyline(pts, 3, /*half_width=*/1.0f, /*z_lift=*/0.0f);
    ASSERT_EQ(v.size(), 6u);
    // Corner pair is v[2]/v[3] (point index 1). One of them is the inner
    // rail (concave side, smaller |x-10| turned toward the turn) — miter
    // scale for an exact 90 degree turn is 1/cos(45deg) ~= 1.41421, so the
    // inner corner vertex sits ~1.41 units from the corner point along the
    // bisector, and critically must NOT have crossed past the corner point
    // onto the other segment's far side (a self-intersection would put the
    // inner vertex's x on the wrong side of x=10 or its y on the wrong side
    // of y=0).
    const double innerX = std::min(v[2].x, v[3].x);
    const double innerY = std::min(v[2].y, v[3].y);
    EXPECT_LT(innerX, 10.0);  // inner rail stays left of the vertical leg
    EXPECT_LT(innerY, 0.0) << "or the mitre bowtied past the corner";
    // Distance from the shared corner point (10,0) is bounded by the
    // clamp (half_width * 4), not by an unbounded miter spike.
    for (const auto& p : {v[2], v[3]}) {
        const double d = std::hypot(p.x - 10.0, p.y - 0.0);
        EXPECT_LT(d, 1.0 * 4.0 + 1e-3);
    }
}

TEST(Polyline, DegenerateInputsAreDropped) {
    EXPECT_TRUE(extrude_polyline(nullptr, 0, 0.25f, 0.0f).empty());
    const Vec3 one[] = {{0, 0, 0}};
    EXPECT_TRUE(extrude_polyline(one, 1, 0.25f, 0.0f).empty());
    const Vec3 dup[] = {{1, 1, 0}, {1, 1, 0}};  // zero-length segment
    EXPECT_TRUE(extrude_polyline(dup, 2, 0.25f, 0.0f).empty());
}

TEST(Polyline, NanPointIsDroppedNotPropagated) {
    // A NaN in the middle of an otherwise-valid polyline truncates it (only
    // the prefix before the NaN survives) rather than propagating a NaN
    // vertex into a vertex buffer (spec §9).
    const double kNan = std::numeric_limits<double>::quiet_NaN();
    const Vec3 pts[] = {{0, 0, 0}, {5, 0, 0}, {kNan, 0, 0}, {10, 0, 0}};
    auto v = extrude_polyline(pts, 4, 0.25f, 0.0f);
    ASSERT_EQ(v.size(), 4u);  // only the 2-point prefix survives -> 2 pairs
    for (const auto& p : v) {
        EXPECT_TRUE(std::isfinite(p.x));
        EXPECT_TRUE(std::isfinite(p.y));
        EXPECT_TRUE(std::isfinite(p.z));
    }
    EXPECT_NEAR(v[2].x, 5.0, 1e-5);  // second surviving point, not the 10.0 tail
}

TEST(Polyline, ExtrudeIndicesCoverEverySegmentTwice) {
    EXPECT_TRUE(extrude_polyline_indices(0).empty());
    EXPECT_TRUE(extrude_polyline_indices(1).empty());
    auto idx = extrude_polyline_indices(4);  // 3 segments -> 6 triangles
    ASSERT_EQ(idx.size(), 18u);
    uint16_t maxIndex = 0;
    for (uint16_t i : idx) maxIndex = std::max(maxIndex, i);
    EXPECT_EQ(maxIndex, 7u);  // 4 points -> 8 vertices, highest index 7
}

TEST(Polyline, TriangulateConvexPolygonFansFromFirstVertex) {
    EXPECT_TRUE(triangulate_convex_polygon(nullptr, 0, 0.0f).empty());
    const Vec3 two[] = {{0, 0, 0}, {1, 0, 0}};
    EXPECT_TRUE(triangulate_convex_polygon(two, 2, 0.0f).empty());

    const Vec3 square[] = {{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}};
    auto tris = triangulate_convex_polygon(square, 4, /*z_lift=*/0.5f);
    ASSERT_EQ(tris.size(), 6u);  // (4-2) triangles * 3 verts
    for (const auto& p : tris) EXPECT_NEAR(p.z, 0.5, 1e-6);
    // Every triangle fans from vertex 0.
    EXPECT_NEAR(tris[0].x, 0.0, 1e-6);
    EXPECT_NEAR(tris[0].y, 0.0, 1e-6);
    EXPECT_NEAR(tris[3].x, 0.0, 1e-6);
    EXPECT_NEAR(tris[3].y, 0.0, 1e-6);
}

TEST(Polyline, ChunksCoverEveryPointOverlappingByOne) {
    EXPECT_TRUE(polyline_chunks(0).empty());
    EXPECT_TRUE(polyline_chunks(1).empty());

    auto small = polyline_chunks(5);
    ASSERT_EQ(small.size(), 1u);
    EXPECT_EQ(small[0].first, 0u);
    EXPECT_EQ(small[0].second, 5u);

    // 32001 points -> 2 chunks, overlapping by one, covering every point.
    auto big = polyline_chunks(32001);
    ASSERT_EQ(big.size(), 2u);
    EXPECT_EQ(big[0].first, 0u);
    EXPECT_EQ(big[0].second, 32000u);
    EXPECT_EQ(big[1].first, 31999u);   // overlaps chunk 0's last point
    EXPECT_EQ(big[1].second, 32001u);  // covers through the final index
}
