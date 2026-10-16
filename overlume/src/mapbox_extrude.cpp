// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_extrude.hpp"

#include <mapbox/earcut.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

namespace overlume::mapbox {

namespace {

using Point2 = std::array<double, 2>;
using Point3 = std::array<double, 3>;

// Below this area (tile units^2) a clipped ring is a sliver with no surface to build.
constexpr double kMinRingArea = 1e-9;

// Shoelace area over the first two coordinates: positive when counter-clockwise with y up.
template <typename Point>
double signed_area(const std::vector<Point>& ring) {
    double sum = 0.0;
    for (size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        sum += ring[j][0] * ring[i][1] - ring[i][0] * ring[j][1];
    }
    return 0.5 * sum;
}

bool all_finite(const TileRing& ring) {
    return std::all_of(ring.begin(), ring.end(),
                       [](const Point2& p) { return std::isfinite(p[0]) && std::isfinite(p[1]); });
}

// One Sutherland-Hodgman pass: keeps the side of the line p[axis] == bound at or below it
// (keep_below) or at or above it. Crossings are put exactly on the line, so an edge along the clip
// square is found by exact comparison.
TileRing clip_to_line(const TileRing& ring, size_t axis, double bound, bool keep_below) {
    const auto inside = [&](const Point2& p) {
        return keep_below ? p[axis] <= bound : p[axis] >= bound;
    };
    TileRing out;
    out.reserve(ring.size() + 2);
    for (size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        const Point2& prev = ring[j];
        const Point2& cur = ring[i];
        if (inside(cur) != inside(prev)) {
            const double t = (bound - prev[axis]) / (cur[axis] - prev[axis]);
            Point2 hit{};
            hit[axis] = bound;
            hit[1 - axis] = prev[1 - axis] + t * (cur[1 - axis] - prev[1 - axis]);
            out.push_back(hit);
        }
        if (inside(cur)) out.push_back(cur);
    }
    return out;
}

// The ring clipped to [0, e]^2, or empty when no area is left (which drops rings of fewer than
// three vertices too).
TileRing clip_ring(const TileRing& ring, double e) {
    TileRing r = clip_to_line(ring, 0, 0.0, false);
    r = clip_to_line(r, 0, e, true);
    r = clip_to_line(r, 1, 0.0, false);
    r = clip_to_line(r, 1, e, true);
    if (std::abs(signed_area(r)) < kMinRingArea) r.clear();
    return r;
}

// Both ends on one side of the clip square: taken as a cut through a building, not one of its
// walls. Known limit: a real wall lying exactly on that line is skipped too (the neighbour tile
// drops its zero-area copy); telling the two apart needs a per-vertex "made by the clip" flag.
bool on_one_clip_line(const Point2& a, const Point2& b, double e) {
    return (a[0] == b[0] && (a[0] == 0.0 || a[0] == e)) ||
           (a[1] == b[1] && (a[1] == 0.0 || a[1] == e));
}

Point3 sub(const Point3& a, const Point3& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }

Point3 cross(const Point3& a, const Point3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

// A clipped ring with its vertices projected at the base and at the top.
struct Ring {
    TileRing uv;
    std::vector<Point3> base;
    std::vector<Point3> top;
};

// Projects the ring, then winds it so the solid lies left of every edge seen from above
// (exteriors counter-clockwise, holes clockwise). Decided in output space: the projection mirrors
// tile y, which runs south.
Ring project_ring(TileRing uv, double min_height_m, double height_m, const ProjectFn& project,
                  bool exterior) {
    Ring r;
    r.base.reserve(uv.size());
    r.top.reserve(uv.size());
    for (const Point2& p : uv) {
        r.base.push_back(project(p[0], p[1], min_height_m));
        r.top.push_back(project(p[0], p[1], height_m));
    }
    r.uv = std::move(uv);
    if ((signed_area(r.base) > 0.0) != exterior) {
        std::reverse(r.uv.begin(), r.uv.end());
        std::reverse(r.base.begin(), r.base.end());
        std::reverse(r.top.begin(), r.top.end());
    }
    return r;
}

uint32_t vertex_count(const TriangleMesh& m) {
    return static_cast<uint32_t>(m.positions.size() / 3);
}

void push_vertex(TriangleMesh& m, const Point3& p, const Point3& n) {
    for (size_t c = 0; c < 3; ++c) {
        m.positions.push_back(static_cast<float>(p[c]));
        m.normals.push_back(static_cast<float>(n[c]));
    }
}

uint32_t add_walls(const Ring& r, double e, TriangleMesh& out) {
    uint32_t quads = 0;
    const size_t n = r.uv.size();
    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        if (on_one_clip_line(r.uv[i], r.uv[j], e)) continue;
        const Point3& b0 = r.base[i];
        const Point3& b1 = r.base[j];
        const Point3& t1 = r.top[j];
        const Point3& t0 = r.top[i];
        // The quad's area vector (half the cross product of its diagonals) points away from the
        // solid on the edge's left.
        Point3 normal = cross(sub(t1, b0), sub(t0, b1));
        const double length =
            std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
        if (!(length > 0.0)) continue;  // a repeated vertex: no face
        for (double& c : normal) c /= length;
        const uint32_t k = vertex_count(out);
        for (const Point3& p : {b0, b1, t1, t0}) push_vertex(out, p, normal);
        out.indices.insert(out.indices.end(), {k, k + 1, k + 2, k, k + 2, k + 3});
        ++quads;
    }
    return quads;
}

uint32_t add_roof(const std::vector<Ring>& rings, TriangleMesh& out) {
    std::vector<std::vector<Point2>> polygon;  // the top rings seen from above
    std::vector<Point3> vertices;
    for (const Ring& r : rings) {
        std::vector<Point2>& flat = polygon.emplace_back();
        for (const Point3& p : r.top) {
            flat.push_back({p[0], p[1]});
            vertices.push_back(p);
        }
    }
    // Spelled ::mapbox because inside overlume::mapbox a bare `mapbox` names this namespace.
    const std::vector<uint32_t> tris = ::mapbox::earcut<uint32_t>(polygon);
    if (tris.empty()) return 0;
    const uint32_t k = vertex_count(out);
    for (const Point3& p : vertices) push_vertex(out, p, {0.0, 0.0, 1.0});
    for (size_t t = 0; t + 2 < tris.size(); t += 3) {
        const uint32_t a = tris[t];
        uint32_t b = tris[t + 1];
        uint32_t c = tris[t + 2];
        const Point3 ab = sub(vertices[b], vertices[a]);
        const Point3 ac = sub(vertices[c], vertices[a]);
        // Earcut's self-intersection pass can emit a clockwise triangle.
        if (ab[0] * ac[1] - ab[1] * ac[0] < 0.0) std::swap(b, c);
        out.indices.insert(out.indices.end(), {k + a, k + b, k + c});
    }
    return static_cast<uint32_t>(tris.size() / 3);
}

}

ExtrudeStats extrude_polygons(const std::vector<TilePolygon>& polygons, double clip_extent,
                              double min_height_m, double height_m, const ProjectFn& project,
                              TriangleMesh& out) {
    ExtrudeStats stats;
    // Also refuses NaN and infinite heights: values from a tile must not reach the mesh.
    if (!(std::isfinite(min_height_m) && std::isfinite(height_m) && height_m > min_height_m)) {
        return stats;
    }
    for (const TilePolygon& polygon : polygons) {
        ++stats.polygons_in;
        if (!all_finite(polygon.outer) ||
            !std::all_of(polygon.holes.begin(), polygon.holes.end(), all_finite)) {
            continue;
        }
        TileRing outer = clip_ring(polygon.outer, clip_extent);
        if (outer.empty()) continue;
        std::vector<Ring> rings;
        rings.push_back(project_ring(std::move(outer), min_height_m, height_m, project, true));
        for (const TileRing& hole : polygon.holes) {
            TileRing clipped = clip_ring(hole, clip_extent);
            if (clipped.empty()) continue;
            rings.push_back(
                project_ring(std::move(clipped), min_height_m, height_m, project, false));
        }
        ++stats.polygons_built;
        for (const Ring& r : rings) stats.wall_quads += add_walls(r, clip_extent, out);
        stats.roof_triangles += add_roof(rings, out);
    }
    return stats;
}

std::optional<std::array<double, 2>> footprint_centroid(const std::vector<TilePolygon>& polygons) {
    double total = 0.0, sum_u = 0.0, sum_v = 0.0;
    for (const TilePolygon& polygon : polygons) {
        const TileRing& r = polygon.outer;
        if (r.size() < 3 || !all_finite(r)) continue;
        // Moments about the first vertex keep the products small.
        const double u0 = r[0][0], v0 = r[0][1];
        double area2 = 0.0, mu = 0.0, mv = 0.0;  // 2 * area and 6 * area * centroid
        for (size_t i = 0, j = r.size() - 1; i < r.size(); j = i++) {
            const double uj = r[j][0] - u0, vj = r[j][1] - v0;
            const double ui = r[i][0] - u0, vi = r[i][1] - v0;
            const double c = uj * vi - ui * vj;
            area2 += c;
            mu += (uj + ui) * c;
            mv += (vj + vi) * c;
        }
        if (area2 == 0.0) continue;
        // Weighted by |area|, so exteriors of either winding count alike.
        const double w = std::abs(area2);
        sum_u += w * (u0 + mu / (3.0 * area2));
        sum_v += w * (v0 + mv / (3.0 * area2));
        total += w;
    }
    if (!(total > 0.0)) return std::nullopt;
    return std::array<double, 2>{sum_u / total, sum_v / total};
}

}
