// polyline.cpp — see polyline.hpp. Pure geometry, no Filament, no GPU.
#include "polyline.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace overlume::detail {

namespace {

bool is_finite(const Vec3& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

bool nearly_equal(const Vec3& a, const Vec3& b) {
    constexpr double kEps = 1e-9;
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return (dx * dx + dy * dy + dz * dz) < (kEps * kEps);
}

struct Vec2f {
    float x = 0.0f, y = 0.0f;
};

Vec2f normalize(Vec2f v) {
    const float len = std::sqrt(v.x * v.x + v.y * v.y);
    if (len < 1e-9f) return {0.0f, 0.0f};
    return {v.x / len, v.y / len};
}

// Rotates `d` by +90 degrees about +Z (in the XY plane) — "left" of travel.
Vec2f perp(Vec2f d) { return {-d.y, d.x}; }

// Cleans `pts` per polyline.hpp's documented rules: truncate at the first
// NaN, drop zero-length-segment duplicates. Returns the surviving points.
std::vector<Vec3> clean_polyline(const Vec3* pts, uint32_t n) {
    std::vector<Vec3> out;
    if (pts == nullptr) return out;
    out.reserve(n);
    for (uint32_t i = 0; i < n; ++i) {
        if (!is_finite(pts[i])) break;  // truncate, don't propagate
        if (!out.empty() && nearly_equal(out.back(), pts[i])) continue;  // zero-length segment
        out.push_back(pts[i]);
    }
    return out;
}

}  // namespace

std::vector<Vec3> extrude_polyline(const Vec3* pts, uint32_t n, float half_width, float z_lift) {
    const std::vector<Vec3> clean = clean_polyline(pts, n);
    const size_t m = clean.size();
    if (m < 2) return {};

    // A near-reversal corner's raw mitre length (half_width / cos(half the
    // turn angle)) blows up as the turn approaches 180 degrees; clamping it
    // turns an unbounded spike into a bounded pinch instead — still visibly
    // "a sharp corner", never a vertex flung far off the ribbon.
    constexpr float kMaxMiterRatio = 4.0f;

    std::vector<Vec2f> dirs(m - 1);
    for (size_t i = 0; i + 1 < m; ++i) {
        dirs[i] = normalize(Vec2f{static_cast<float>(clean[i + 1].x - clean[i].x),
                                   static_cast<float>(clean[i + 1].y - clean[i].y)});
    }

    std::vector<Vec3> out(m * 2);
    for (size_t i = 0; i < m; ++i) {
        Vec2f normal;
        if (i == 0) {
            normal = perp(dirs[0]);
        } else if (i == m - 1) {
            normal = perp(dirs[m - 2]);
        } else {
            const Vec2f pPrev = perp(dirs[i - 1]);
            const Vec2f pNext = perp(dirs[i]);
            Vec2f avg = normalize(Vec2f{pPrev.x + pNext.x, pPrev.y + pNext.y});
            if (avg.x == 0.0f && avg.y == 0.0f) {
                // Exact 180-degree reversal: no well-defined miter direction —
                // fall back to the incoming segment's own normal (a clean
                // bevel at that one point, not a NaN).
                avg = pPrev;
            }
            const float cosHalf = avg.x * pPrev.x + avg.y * pPrev.y;  // both unit-length
            float scale = (cosHalf > 1e-3f) ? (1.0f / cosHalf) : kMaxMiterRatio;
            if (scale > kMaxMiterRatio) scale = kMaxMiterRatio;
            normal = {avg.x * scale, avg.y * scale};
        }
        const float px = static_cast<float>(clean[i].x);
        const float py = static_cast<float>(clean[i].y);
        const float pz = static_cast<float>(clean[i].z) + z_lift;
        out[2 * i] = Vec3{px - normal.x * half_width, py - normal.y * half_width, pz};
        out[2 * i + 1] = Vec3{px + normal.x * half_width, py + normal.y * half_width, pz};
    }
    return out;
}

std::vector<uint16_t> extrude_polyline_indices(uint32_t point_count) {
    if (point_count < 2) return {};
    std::vector<uint16_t> indices;
    indices.reserve(static_cast<size_t>(point_count - 1) * 6);
    for (uint32_t i = 0; i + 1 < point_count; ++i) {
        const auto l0 = static_cast<uint16_t>(2 * i);
        const auto r0 = static_cast<uint16_t>(2 * i + 1);
        const auto l1 = static_cast<uint16_t>(2 * i + 2);
        const auto r1 = static_cast<uint16_t>(2 * i + 3);
        indices.push_back(l0);
        indices.push_back(r0);
        indices.push_back(r1);
        indices.push_back(l0);
        indices.push_back(r1);
        indices.push_back(l1);
    }
    return indices;
}

std::vector<Vec3> triangulate_convex_polygon(const Vec3* pts, uint32_t n, float z_lift) {
    if (pts == nullptr || n < 3) return {};
    for (uint32_t i = 0; i < n; ++i) {
        if (!is_finite(pts[i])) return {};  // malformed polygon: no safe partial reading
    }
    std::vector<Vec3> out;
    out.reserve(static_cast<size_t>(n - 2) * 3);
    auto lift = [z_lift](const Vec3& p) { return Vec3{p.x, p.y, p.z + z_lift}; };
    const Vec3 p0 = lift(pts[0]);
    for (uint32_t i = 1; i + 1 < n; ++i) {
        out.push_back(p0);
        out.push_back(lift(pts[i]));
        out.push_back(lift(pts[i + 1]));
    }
    return out;
}

std::pair<double, double> closest_arc_station(const Vec3* pts, uint32_t n, const Vec3& ego) {
    if (n < 2) return {0.0, std::numeric_limits<double>::infinity()};
    double cum = 0.0;
    double bestStation = 0.0;
    double bestDist = std::numeric_limits<double>::infinity();
    for (uint32_t i = 0; i + 1 < n; ++i) {
        const Vec3& a = pts[i];
        const Vec3& b = pts[i + 1];
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double segLen = std::sqrt(dx * dx + dy * dy);
        double t = 0.0;
        if (segLen > 0.0) {
            t = ((ego.x - a.x) * dx + (ego.y - a.y) * dy) / (segLen * segLen);
            t = std::clamp(t, 0.0, 1.0);
        }
        const double px = a.x + dx * t, py = a.y + dy * t;
        const double dist = std::hypot(ego.x - px, ego.y - py);
        if (dist < bestDist) {
            bestDist = dist;
            bestStation = cum + t * segLen;
        }
        cum += segLen;
    }
    return {bestStation, bestDist};
}

PolylineClip compute_polyline_clip(const Vec3* pts, uint32_t n, const Vec3& ego_position) {
    PolylineClip clip;
    const auto [station, dist] = closest_arc_station(pts, n, ego_position);
    if (dist >= kPolylineEgoClipLateralM) return clip;  // proximity gate: too far, render whole
    clip.active = true;
    // ceil, not lround: round-to-nearest would land the cut up to 0.25m
    // behind the closest-approach station about half the time, rendering
    // part of the polyline behind the ego. ceil keeps the cut at-or-ahead,
    // equally deterministic (parked-ego zero-rebuild property unchanged).
    clip.quantized_units = static_cast<int64_t>(std::ceil(station / kPolylineClipQuantizeM));
    clip.station_m = static_cast<double>(clip.quantized_units) * kPolylineClipQuantizeM;
    return clip;
}

std::vector<double> clean_polyline_stations(const Vec3* pts, uint32_t n) {
    std::vector<double> out;
    if (pts == nullptr) return out;
    out.reserve(n);
    Vec3 prev{};
    bool havePrev = false;
    double cum = 0.0;
    for (uint32_t i = 0; i < n; ++i) {
        if (!is_finite(pts[i])) break;  // truncate, don't propagate -- mirrors clean_polyline()
        if (havePrev && nearly_equal(prev, pts[i])) continue;  // zero-length segment, adds 0 arc
        if (havePrev) {
            const double dx = pts[i].x - prev.x, dy = pts[i].y - prev.y;
            cum += std::sqrt(dx * dx + dy * dy);
        }
        out.push_back(cum);
        prev = pts[i];
        havePrev = true;
    }
    return out;
}

void collapse_clipped_positions(std::vector<Vec3>& positions, const std::vector<double>& stations,
                                 bool clip_active, double clip_station_m) {
    const size_t m = stations.size();
    if (!clip_active || m == 0) return;
    if (positions.size() != 2 * m) return;  // caller contract violated -- no-op, not a crash

    // Last point index still behind the cut (station < clip_station_m); -1
    // (via the m sentinel below) means the whole strip is already ahead.
    size_t behind = m;  // m == "none behind" sentinel
    for (size_t i = 0; i < m; ++i) {
        if (stations[i] < clip_station_m) behind = i; else break;
    }
    if (behind == m) return;  // nothing behind the cut in this strip

    Vec3 cutL, cutR;
    if (behind + 1 >= m) {
        // The cut itself lies beyond this strip (only possible across a
        // >kMaxPointsPerMesh chunk boundary) -- collapse the whole strip to
        // its own last pair, a zero-area sliver; the chunk that actually
        // contains the cut draws the real edge.
        cutL = positions[2 * (m - 1)];
        cutR = positions[2 * (m - 1) + 1];
        behind = m - 1;
    } else {
        const double span = stations[behind + 1] - stations[behind];
        const double t = span > 1e-9 ? (clip_station_m - stations[behind]) / span : 0.0;
        const auto lerp = [t](const Vec3& a, const Vec3& b) {
            return Vec3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
        };
        cutL = lerp(positions[2 * behind], positions[2 * (behind + 1)]);
        cutR = lerp(positions[2 * behind + 1], positions[2 * (behind + 1) + 1]);
    }
    for (size_t i = 0; i <= behind; ++i) {
        positions[2 * i] = cutL;
        positions[2 * i + 1] = cutR;
    }
}

std::vector<std::pair<uint32_t, uint32_t>> polyline_chunks(uint32_t n) {
    std::vector<std::pair<uint32_t, uint32_t>> chunks;
    if (n < 2) return chunks;
    uint32_t start = 0;
    while (start + 1 < n) {
        uint32_t end = start + kMaxPointsPerMesh;
        if (end > n) end = n;
        chunks.emplace_back(start, end);
        if (end >= n) break;
        start = end - 1;  // overlap by one point so strips join with no gap
    }
    return chunks;
}

}  // namespace overlume::detail
