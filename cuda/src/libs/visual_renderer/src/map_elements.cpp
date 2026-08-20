// map_elements.cpp — Epic 2 Task 2 (VM-024): HD-map lane centerlines and
// crosswalk polygons on the lit clay pipeline, lane styling driven entirely
// by theme tokens (no per-theme branch here — same discipline as
// renderer.cpp's push_theme_to_scene()).
#include "map_elements.hpp"
#include "polyline.hpp"
#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

#include <filament/RenderableManager.h>

#include <math/vec3.h>

#include <cmath>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

namespace mpviz {

namespace {

using filament::math::float3;

float3 to_f3(const Vec3& v) {
    return float3{static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

uint64_t hash_combine(uint64_t seed, uint64_t v) {
    return seed ^ (v + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2));
}

uint64_t hash_vec3(const Vec3& v) {
    uint64_t h = std::hash<double>{}(v.x);
    h = hash_combine(h, std::hash<double>{}(v.y));
    h = hash_combine(h, std::hash<double>{}(v.z));
    return h;
}

// Content signature for one mesh chunk -- NOT keyed by array position: the
// source topic (/hd_map_local_elements) is a rolling ~50m window
// republished at 18 Hz, so an element's index in the array is not stable
// from one set_scene() call to the next, but its own point data is (a lane
// segment that hasn't left the window yet still has the same points). Two
// chunks of the SAME polyline (see polyline_chunks()) get distinct
// signatures because each chunk's own first/last point differs.
uint64_t chunk_signature(bool is_polygon, const Vec3* pts, uint32_t n) {
    uint64_t h = is_polygon ? 0x1ULL : 0x0ULL;
    h = hash_combine(h, static_cast<uint64_t>(n));
    if (n > 0) {
        h = hash_combine(h, hash_vec3(pts[0]));
        h = hash_combine(h, hash_vec3(pts[n - 1]));
    }
    return h;
}

std::vector<Vertex> to_verts(const std::vector<Vec3>& positions) {
    std::vector<Vertex> verts(positions.size());
    std::vector<float3> normals(positions.size(), float3{0.0f, 0.0f, 1.0f});
    for (size_t i = 0; i < positions.size(); ++i) verts[i].position = to_f3(positions[i]);
    fill_tangent_frames(verts, normals);
    return verts;
}

// Lazy crosswalk hatch (spec §7's "crosswalk reads as a crosswalk", not a
// new material -- see map_elements.hpp/the plan's own note: "the lazy hatch
// is geometry, not a new material"). This IS the crosswalk's rendering for
// the common 4-point quad every recorded crosswalk marker actually is
// (Task 2 Step 6's fixture note: crosswalk_/crosswalk_stopline_ markers are
// LINE_STRIP/polygon quads on the wire) -- deliberately NOT a full-polygon
// solid fill with stripes drawn on top of it (that would be invisible: the
// stripes and the fill share the one lane-paint MaterialInstance, so an
// on-top stripe over an identically-colored fill changes zero pixels). The
// visual differentiation IS the geometry: painted bars with ground visible
// in the gaps between them. Returns empty for n != 4 -- the caller falls
// back to triangulate_convex_polygon()'s plain fan fill for any other
// polygon shape.
// ponytail: bilinear-interpolated stripes between the quad's two SHORT
// edges, not a general convex-polygon clip -- guaranteed to stay inside a
// convex quad (the only shape this epic's recorded data produces) with far
// less code than Sutherland-Hodgman; upgrade if a skewed/non-quad crosswalk
// ever shows a stripe spilling outside its polygon in a golden.
std::vector<Vec3> build_crosswalk_hatch(const Vec3* pts, uint32_t n, float z_lift) {
    std::vector<Vec3> tris;
    if (n != 4) return tris;
    auto edge_len = [&](int a, int b) {
        const double dx = pts[a].x - pts[b].x;
        const double dy = pts[a].y - pts[b].y;
        return std::sqrt(dx * dx + dy * dy);
    };
    const double lenA = edge_len(0, 1) + edge_len(2, 3);
    const double lenB = edge_len(1, 2) + edge_len(3, 0);
    // rail0/rail1: the pair of SHORT edges -- lerping along them sweeps
    // stripes across the polygon's long axis.
    Vec3 r0a, r0b, r1a, r1b;
    if (lenA >= lenB) {
        r0a = pts[1];
        r0b = pts[2];
        r1a = pts[0];
        r1b = pts[3];
    } else {
        r0a = pts[0];
        r0b = pts[1];
        r1a = pts[3];
        r1b = pts[2];
    }
    constexpr int kStripes = 5;
    auto lerp = [](const Vec3& a, const Vec3& b, double t) {
        return Vec3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
    };
    auto lift = [&](Vec3 v) {
        v.z += z_lift;
        return v;
    };
    for (int i = 0; i < kStripes; ++i) {
        const double t0 = (2.0 * i) / (2.0 * kStripes);
        const double t1 = (2.0 * i + 1.0) / (2.0 * kStripes);
        const Vec3 a0 = lift(lerp(r0a, r0b, t0));
        const Vec3 a1 = lift(lerp(r0a, r0b, t1));
        const Vec3 b0 = lift(lerp(r1a, r1b, t0));
        const Vec3 b1 = lift(lerp(r1a, r1b, t1));
        tris.push_back(a0);
        tris.push_back(a1);
        tris.push_back(b1);
        tris.push_back(a0);
        tris.push_back(b1);
        tris.push_back(b0);
    }
    return tris;
}

// Half-width 0.05 -> 0.10 m FULL stripe width, matching what the real
// publishers ask for via Marker::scale.x (measured on the wire 2026-08-20:
// boundaries 0.10, centerlines 0.15, stoplines 0.20 -- rviz honors scale.x,
// and at our previous 0.20 m full width two adjacent-lane boundary lines a
// few decimetres apart merged into "offset" double bands that rviz showed
// as crisp separate lines). One constant is a stopgap: MapElement is frozen
// without a width field; per-marker width crosses the boundary with
// VM-036's kind/width_m at the Epic 3 freeze lift.
constexpr float kLaneHalfWidthM = 0.05f;  // lane-paint stripe half-width
constexpr float kLaneZLiftM = 0.02f;      // matches the ego-box/grid z-lift convention

}  // namespace

// Epic 2 Task 2 (VM-024) STATED DEVIATION from spec §5 -- the HD-map
// category pops, it does not fade: `MapElement` is frozen with
// {points, point_count, is_polygon} and no `last_update_sec`, so
// SceneBuffer::staleness_alpha has nothing to evaluate for map geometry and
// this function cannot ramp it. What actually ships: past the adapter row's
// timeout_sec, HdMapAdapter (node-side) stops filling map_elements and
// every lane/crosswalk vanishes from the NEXT set_scene() in one frame, on
// a real dropout that is a visible pop, not the opacity ramp spec §5 asks
// for. See the plan's "Staleness: where the timeouts actually live" section
// for the two unblock paths -- neither is this epic's to take.
void update_map_elements(VisualRenderer& r, const SceneGraph& s) {
    std::unordered_map<uint64_t, Mesh> next;
    next.reserve(r.mapElementMeshes.size());

    auto adopt_or_build = [&](uint64_t key, auto build_fn) {
        auto it = r.mapElementMeshes.find(key);
        if (it != r.mapElementMeshes.end()) {
            next.emplace(key, std::move(it->second));
            r.mapElementMeshes.erase(it);
            return;
        }
        std::vector<Vec3> positions = build_fn();
        if (positions.empty()) return;
        std::vector<Vertex> verts = to_verts(positions);
        // Flat sequential indexing lives under the uint16 index ceiling.
        // Today's builders stay well below it (lanes chunked, crosswalks
        // tiny); this guard turns a would-be infinite loop (a uint16_t
        // counter WRAPS at 65536 and never reaches a larger verts.size() --
        // see ribbon.cpp's header, review 2026-08-20) into a loud drop.
        // ponytail: flat + guard; convert to ribbon.cpp's true-indexed
        // pattern if a real >10k-point map element ever shows up.
        if (verts.size() > 65535) {
            std::fprintf(stderr,
                         "[visual_renderer] map element mesh (%zu verts) exceeds the uint16 "
                         "index ceiling; element dropped\n",
                         verts.size());
            return;
        }
        std::vector<uint16_t> indices(verts.size());
        for (size_t i = 0; i < verts.size(); ++i) indices[i] = static_cast<uint16_t>(i);
        Mesh mesh;
        add_mesh(r, mesh, std::move(verts), std::move(indices),
                 filament::RenderableManager::PrimitiveType::TRIANGLES, r.laneMaterial,
                 /*cast_shadows=*/false, /*receive_shadows=*/true);
        next.emplace(key, std::move(mesh));
    };

    for (uint32_t i = 0; i < s.map_element_count; ++i) {
        const MapElement& e = s.map_elements[i];
        if (e.points == nullptr || e.point_count < 2) continue;  // malformed guard

        if (e.is_polygon) {
            const uint64_t key = chunk_signature(true, e.points, e.point_count);
            adopt_or_build(key, [&]() {
                std::vector<Vec3> hatch = build_crosswalk_hatch(e.points, e.point_count, kLaneZLiftM);
                if (!hatch.empty()) return hatch;
                // Not a 4-point quad -- no safe hatch reading; fall back to
                // a plain solid fan fill so the polygon still renders as
                // SOMETHING rather than nothing.
                return detail::triangulate_convex_polygon(e.points, e.point_count, kLaneZLiftM);
            });
        } else {
            for (auto [a, b] : detail::polyline_chunks(e.point_count)) {
                const uint32_t n = b - a;
                const uint64_t key = chunk_signature(false, e.points + a, n);
                const Vec3* chunkPts = e.points + a;
                adopt_or_build(key, [&, chunkPts, n]() {
                    std::vector<Vec3> ribbon =
                        detail::extrude_polyline(chunkPts, n, kLaneHalfWidthM, kLaneZLiftM);
                    if (ribbon.empty()) return ribbon;
                    // Re-triangulate the strip's own index order into a flat
                    // list here (to_verts()/adopt_or_build() above build a
                    // plain triangle list with sequential indices, matching
                    // the polygon path) -- expand extrude_polyline_indices()
                    // against `ribbon` rather than emitting it as its own
                    // indexed mesh, so both branches share one small
                    // "build a Mesh from a flat triangle list" path.
                    const auto stripIdx =
                        detail::extrude_polyline_indices(static_cast<uint32_t>(ribbon.size() / 2));
                    std::vector<Vec3> flat(stripIdx.size());
                    for (size_t k = 0; k < stripIdx.size(); ++k) flat[k] = ribbon[stripIdx[k]];
                    return flat;
                });
            }
        }
    }

    for (auto& [key, mesh] : r.mapElementMeshes) {
        destroy_mesh(*r.engine, *r.scene, mesh);
    }
    r.mapElementMeshes = std::move(next);
}

}  // namespace mpviz
