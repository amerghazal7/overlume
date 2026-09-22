// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "polyline.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

using overlume::Vec3;
using overlume::detail::extrude_polyline;
using overlume::detail::extrude_polyline_indices;
using overlume::detail::polyline_chunks;
using overlume::detail::triangulate_convex_polygon;

TEST(Polyline, StraightSegmentExtrudesToRectangleOfGivenWidth) {
    const Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    auto v = extrude_polyline(pts, 2, 0.25f, 0.02f);
    ASSERT_EQ(v.size(), 4u);
    EXPECT_NEAR(v[0].y, -0.25, 1e-5);
    EXPECT_NEAR(v[1].y, 0.25, 1e-5);
    EXPECT_NEAR(v[0].z, 0.02, 1e-5);
    EXPECT_NEAR(v[2].y, -0.25, 1e-5);
    EXPECT_NEAR(v[3].y, 0.25, 1e-5);
    EXPECT_NEAR(v[0].x, 0.0, 1e-5);
    EXPECT_NEAR(v[2].x, 10.0, 1e-5);
}

TEST(Polyline, CornerMitresWithoutSelfIntersection) {
    const Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}, {10, 10, 0}};
    auto v = extrude_polyline(pts, 3, 1.0f, 0.0f);
    ASSERT_EQ(v.size(), 6u);
    const double innerX = std::min(v[2].x, v[3].x);
    const double innerY = std::min(v[2].y, v[3].y);
    EXPECT_LT(innerX, 10.0);
    EXPECT_LT(innerY, 0.0) << "or the mitre bowtied past the corner";
    for (const auto& p : {v[2], v[3]}) {
        const double d = std::hypot(p.x - 10.0, p.y - 0.0);
        EXPECT_LT(d, 1.0 * 4.0 + 1e-3);
    }
}

TEST(Polyline, DegenerateInputsAreDropped) {
    EXPECT_TRUE(extrude_polyline(nullptr, 0, 0.25f, 0.0f).empty());
    const Vec3 one[] = {{0, 0, 0}};
    EXPECT_TRUE(extrude_polyline(one, 1, 0.25f, 0.0f).empty());
    const Vec3 dup[] = {{1, 1, 0}, {1, 1, 0}};
    EXPECT_TRUE(extrude_polyline(dup, 2, 0.25f, 0.0f).empty());
}

TEST(Polyline, NanPointIsDroppedNotPropagated) {
    const double kNan = std::numeric_limits<double>::quiet_NaN();
    const Vec3 pts[] = {{0, 0, 0}, {5, 0, 0}, {kNan, 0, 0}, {10, 0, 0}};
    auto v = extrude_polyline(pts, 4, 0.25f, 0.0f);
    ASSERT_EQ(v.size(), 4u);
    for (const auto& p : v) {
        EXPECT_TRUE(std::isfinite(p.x));
        EXPECT_TRUE(std::isfinite(p.y));
        EXPECT_TRUE(std::isfinite(p.z));
    }
    EXPECT_NEAR(v[2].x, 5.0, 1e-5);
}

TEST(Polyline, ExtrudeIndicesCoverEverySegmentTwice) {
    EXPECT_TRUE(extrude_polyline_indices(0).empty());
    EXPECT_TRUE(extrude_polyline_indices(1).empty());
    auto idx = extrude_polyline_indices(4);
    ASSERT_EQ(idx.size(), 18u);
    uint16_t maxIndex = 0;
    for (uint16_t i : idx) maxIndex = std::max(maxIndex, i);
    EXPECT_EQ(maxIndex, 7u);
}

TEST(Polyline, TriangulateConvexPolygonFansFromFirstVertex) {
    EXPECT_TRUE(triangulate_convex_polygon(nullptr, 0, 0.0f).empty());
    const Vec3 two[] = {{0, 0, 0}, {1, 0, 0}};
    EXPECT_TRUE(triangulate_convex_polygon(two, 2, 0.0f).empty());

    const Vec3 square[] = {{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}};
    auto tris = triangulate_convex_polygon(square, 4, 0.5f);
    ASSERT_EQ(tris.size(), 6u);
    for (const auto& p : tris) EXPECT_NEAR(p.z, 0.5, 1e-6);
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

    auto big = polyline_chunks(32001);
    ASSERT_EQ(big.size(), 2u);
    EXPECT_EQ(big[0].first, 0u);
    EXPECT_EQ(big[0].second, 32000u);
    EXPECT_EQ(big[1].first, 31999u);
    EXPECT_EQ(big[1].second, 32001u);
}
