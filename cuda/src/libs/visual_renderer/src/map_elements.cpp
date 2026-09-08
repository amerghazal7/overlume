// map_elements.cpp — Epic 2 Task 2 (VM-024): HD-map lane centerlines,
// boundaries, crosswalk polygons and (Epic 3 Task 1 / VM-036) road-surface
// fill on the lit clay pipeline. Per-kind styling driven entirely by theme
// tokens + a fixed dispatch table (material_for_kind/z_lift_for_kind) --
// no per-theme branch here, same discipline as renderer.cpp's
// push_theme_to_scene().
#include "map_elements.hpp"
#include "polyline.hpp"
#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

#include <filament/RenderableManager.h>

#include <math/vec3.h>
#include <math/vec4.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

namespace mpviz {

namespace {

using filament::math::float3;
using filament::math::float4;

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

}  // namespace

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
// polygon shape. Declared in polyline.hpp (mpviz::detail), not
// map_elements.hpp, per Epic 3 Task 1 (VM-036) Step 2: reachable from
// Filament-free tests the same way extrude_polyline/triangulate_convex_
// polygon already are.
// ponytail: bilinear-interpolated stripes between the quad's two LONG
// edges, not a general convex-polygon clip -- guaranteed to stay inside a
// convex quad (the only shape this epic's recorded data produces) with far
// less code than Sutherland-Hodgman; upgrade if a skewed/non-quad crosswalk
// ever shows a stripe spilling outside its polygon in a golden.
//
// User directive 2026-09-08: "crosswalk is rendered wrongly! it's
// horizontal lines instead of vertical!" -- root cause was picking the
// quad's SHORT-edge pair as the two rails, which sweeps each bar's LONG
// axis across the quad's own LONG axis (road width) while stacking bars
// along the SHORT axis (direction of travel): ladder rungs across the
// road, the wrong way round. FIX: rails = the LONG-edge pair instead, so
// each bar's long axis runs along the SHORT (travel) axis and bars stack
// across the LONG axis (crossing width) -- real zebra orientation. Stripe
// count is now pitch-derived (kCrosswalkStripePitchM) rather than a fixed
// 5 -- a fixed count would render absurdly fat bars once a wide crossing
// is oriented correctly.
constexpr double kCrosswalkStripePitchM = 1.2;  // target bar+gap pitch along the long axis
constexpr int kCrosswalkStripesMin = 3;
constexpr int kCrosswalkStripesMax = 24;

std::vector<Vec3> detail::build_crosswalk_hatch(const Vec3* pts, uint32_t n, float z_lift) {
    std::vector<Vec3> tris;
    if (n != 4) return tris;
    auto edge_len = [&](int a, int b) {
        const double dx = pts[a].x - pts[b].x;
        const double dy = pts[a].y - pts[b].y;
        return std::sqrt(dx * dx + dy * dy);
    };
    const double lenA = edge_len(0, 1) + edge_len(2, 3);
    const double lenB = edge_len(1, 2) + edge_len(3, 0);
    // rail0/rail1: the pair of LONG edges -- lerping along them sweeps
    // bars across the polygon's SHORT (travel) axis, stacking them along
    // the LONG (crossing-width) axis.
    Vec3 r0a, r0b, r1a, r1b;
    double longAxisLen;
    if (lenA >= lenB) {
        r0a = pts[0];
        r0b = pts[1];
        r1a = pts[3];
        r1b = pts[2];
        longAxisLen = lenA / 2.0;
    } else {
        r0a = pts[1];
        r0b = pts[2];
        r1a = pts[0];
        r1b = pts[3];
        longAxisLen = lenB / 2.0;
    }
    const int kStripes = std::clamp(
        static_cast<int>(std::lround(longAxisLen / kCrosswalkStripePitchM)), kCrosswalkStripesMin,
        kCrosswalkStripesMax);
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

namespace {

// Half-width 0.05 -> 0.10 m FULL stripe width, matching what the real
// publishers ask for via Marker::scale.x (measured on the wire 2026-08-20:
// boundaries 0.10, centerlines 0.15, stoplines 0.20 -- rviz honors scale.x,
// and at our previous 0.20 m full width two adjacent-lane boundary lines a
// few decimetres apart merged into "offset" double bands that rviz showed
// as crisp separate lines). One constant is a stopgap: per-marker width is
// a code constant, not a theme/wire field -- a deliberate YAGNI call (Epic
// 3 Task 1 / VM-036 decision #6's own "per-kind width/z-lift are code
// constants" note); promote the day a deployment actually asks for it.
constexpr float kLaneHalfWidthM = 0.05f;  // lane-paint stripe half-width
constexpr float kLaneZLiftM = 0.02f;      // matches the ego-box/grid z-lift convention
// Road-fill z-lift (Epic 3 Task 1 / VM-036, decision #1): between ground
// (0) and OGM's kGradientZLiftM (0.010, ground_grid.cpp) -- the road
// surface is a static base coat painted directly on the ground, and OGM (a
// live perception overlay) must sit above it so a dynamic occupancy
// reading is never hidden behind the static road tint.
constexpr float kRoadZLiftM = 0.005f;

bool IsBoundaryKind(MapKind kind) {
    return kind == MapKind::LEFT_BOUNDARY || kind == MapKind::RIGHT_BOUNDARY;
}

float z_lift_for_kind(MapKind kind) {
    return kind == MapKind::ROAD_SURFACE ? kRoadZLiftM : kLaneZLiftM;
}

// Per-kind MaterialInstance dispatch (decision #6). STOPLINE/JUNCTION/OTHER
// have no dedicated token (a deliberate YAGNI call, same decision) and fall
// back to the pre-existing laneMaterial/palette.lane_paint -- unchanged
// behaviour for those kinds. ROAD_EDGE got its own dedicated token (user
// directive 2026-09-08) -- solid yellow-family, see IsBoundaryKind() below
// for why it never dashes.
filament::MaterialInstance* material_for_kind(VisualRenderer& r, MapKind kind) {
    switch (kind) {
        case MapKind::CENTERLINE:
            return r.laneCenterlineMaterial;
        case MapKind::LEFT_BOUNDARY:
        case MapKind::RIGHT_BOUNDARY:
            return r.laneBoundaryMaterial;
        case MapKind::CROSSWALK:
            return r.crosswalkMaterial;
        case MapKind::ROAD_SURFACE:
            return r.roadMaterial;
        case MapKind::ROAD_EDGE:
            return r.roadEdgeMaterial;
        default:
            return r.laneMaterial;
    }
}

// Epic 3 Task 2 (VM-034): the tint currently pushed into `kind`'s opaque
// template (mirrors material_for_kind()'s own dispatch field-for-field —
// same rows, same fallback). Needed because the staleness fade seeds a
// FRESH clay_translucent.mat instance from this stored value (Filament's
// MaterialInstance has no getter, the same reason every *MaterialBaseColor
// field exists at all) rather than the opaque template's own baseColor.
detail::Float3 tint_for_kind(const VisualRenderer& r, MapKind kind) {
    switch (kind) {
        case MapKind::CENTERLINE:
            return r.laneCenterlineMaterialBaseColor;
        case MapKind::LEFT_BOUNDARY:
        case MapKind::RIGHT_BOUNDARY:
            return r.laneBoundaryMaterialBaseColor;
        case MapKind::CROSSWALK:
            return r.crosswalkMaterialBaseColor;
        case MapKind::ROAD_SURFACE:
            return r.roadMaterialBaseColor;
        case MapKind::ROAD_EDGE:
            return r.roadEdgeMaterialBaseColor;
        default:
            return r.laneMaterialBaseColor;
    }
}

// Dash geometry (Epic 3 Task 1 / VM-036, decision #3): moved renderer-side
// from the adapter (hd_map.cpp), same algorithm and constants, now gated on
// BOUNDARY kinds instead of centerlines -- the flip debt item 2 asks for.
// This is a SEPARATE, independent arc-length walker from the adapter's own
// ResampleByArcLength (hd_map.cpp's road-fill resample) -- two small
// functions, one per side of the ABI boundary, not duplication (map_
// elements.hpp's Files-list note).
constexpr double kDashLenM = 1.5;
constexpr double kGapLenM = 1.5;
constexpr double kMinDashLenM = 0.25;  // shorter trailing dash -> dropped

double dash_dist(const Vec3& a, const Vec3& b) {
    const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Shared arc-length walk: given `pts`/`n` and its own cumulative-length
// table `cum` (cum[0]==0, cum[n-1]==total arc length), returns the
// interpolated point at arc-length `s` (clamped to [0, total]). Extracted
// (user directive 2026-09-08) out of chop_into_dashes()'s own point_at
// lambda so build_centerline_dots() below reuses the exact same walk
// instead of a second copy -- unlike hd_map.cpp's ResampleByArcLength
// (map_elements.hpp's Files-list note explains why THAT one stays a
// separate copy across the ABI boundary), this is the same file, same
// toolchain, so reuse is the correct move, not "two ~15-line functions."
Vec3 point_at_arc_length(const Vec3* pts, uint32_t n, const std::vector<double>& cum, double s) {
    const double total_len = cum.back();
    s = std::clamp(s, 0.0, total_len);
    size_t i = static_cast<size_t>(std::lower_bound(cum.begin(), cum.end(), s) - cum.begin());
    if (i == 0) i = 1;
    if (i >= n) i = n - 1;
    const double seg_len = cum[i] - cum[i - 1];
    const double t = seg_len > 0.0 ? (s - cum[i - 1]) / seg_len : 0.0;
    const Vec3& a = pts[i - 1];
    const Vec3& b = pts[i];
    return Vec3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
}

// Same worked example as the pre-Epic3 adapter-side ChopIntoDashes (10 m
// polyline, 1.5/1.5 -> dashes at [0,1.5],[3,4.5],[6,7.5],[9,10]).
std::vector<std::vector<Vec3>> chop_into_dashes(const Vec3* pts, uint32_t n) {
    std::vector<std::vector<Vec3>> out;
    if (n < 2) return out;

    std::vector<double> cum(n, 0.0);
    for (uint32_t i = 1; i < n; ++i) cum[i] = cum[i - 1] + dash_dist(pts[i - 1], pts[i]);
    const double total_len = cum.back();
    if (total_len <= 0.0) return out;

    constexpr double kPeriod = kDashLenM + kGapLenM;
    for (double s0 = 0.0; s0 < total_len; s0 += kPeriod) {
        const double s1 = std::min(s0 + kDashLenM, total_len);
        if (s1 - s0 < kMinDashLenM) continue;
        std::vector<Vec3> dash;
        dash.push_back(point_at_arc_length(pts, n, cum, s0));
        for (uint32_t i = 0; i < n; ++i) {
            if (cum[i] > s0 && cum[i] < s1) dash.push_back(pts[i]);
        }
        dash.push_back(point_at_arc_length(pts, n, cum, s1));
        out.push_back(std::move(dash));
    }
    return out;
}

// Centerline dot guidance (user directive 2026-09-08, post Task 1 candidate
// review): "circles points along the line instead of a yellow filled line"
// -- flat filled discs (a triangle fan per dot, kCenterlineDotSegments
// wedges) spaced by arc length along the polyline, replacing the old solid
// ribbon strip for kind==CENTERLINE. Radius/spacing are named constants,
// same "code constant, not a theme/wire field" YAGNI call decision #6 made
// for per-kind width/z-lift -- promote to a theme field the day a
// deployment asks to tune it (ribbon.width_m's own history). ponytail:
// segment count fixed at 10 regardless of camera distance/quality preset --
// a per-quality LOD would shave triangles at long range, add if a profiling
// pass ever shows centerline dots costing real frame time.
constexpr float kCenterlineDotRadiusM = 0.15f;
constexpr float kCenterlineDotSpacingM = 2.0f;  // arc-length spacing, dot centre to dot centre
constexpr int kCenterlineDotSegments = 10;
constexpr float kTwoPi = 6.28318530717958647692f;

std::vector<Vec3> build_centerline_dots(const Vec3* pts, uint32_t n, float z_lift) {
    std::vector<Vec3> tris;
    if (n < 2) return tris;

    std::vector<double> cum(n, 0.0);
    for (uint32_t i = 1; i < n; ++i) cum[i] = cum[i - 1] + dash_dist(pts[i - 1], pts[i]);
    const double total_len = cum.back();
    if (total_len <= 0.0) return tris;

    // <= (not <): the LAST dot sits exactly at the polyline's own end
    // point, same "chunk boundaries are inclusive" convention chop_into_
    // dashes' own s1 clamp already follows.
    for (double s = 0.0; s <= total_len; s += kCenterlineDotSpacingM) {
        const Vec3 c = point_at_arc_length(pts, n, cum, s);
        const Vec3 centre{c.x, c.y, c.z + z_lift};
        for (int k = 0; k < kCenterlineDotSegments; ++k) {
            const float a0 = kTwoPi * static_cast<float>(k) / kCenterlineDotSegments;
            const float a1 = kTwoPi * static_cast<float>(k + 1) / kCenterlineDotSegments;
            tris.push_back(centre);
            tris.push_back(Vec3{c.x + kCenterlineDotRadiusM * std::cos(a0),
                                 c.y + kCenterlineDotRadiusM * std::sin(a0), centre.z});
            tris.push_back(Vec3{c.x + kCenterlineDotRadiusM * std::cos(a1),
                                 c.y + kCenterlineDotRadiusM * std::sin(a1), centre.z});
        }
    }
    return tris;
}

// Shared by both the dashed (boundary) and non-dashed polyline paths --
// extrude_polyline()'s own 2*n distinct vertices, re-triangulated into a
// flat sequentially-indexed list via extrude_polyline_indices() (see
// ribbon.cpp's header for why a TRUE indexed mesh matters at scale; this
// path stays under the uint16 ceiling by construction, polyline_chunks()
// already guarantees that upstream).
std::vector<Vec3> build_ribbon_flat(const Vec3* pts, uint32_t n, float half_width, float z_lift) {
    std::vector<Vec3> ribbon = detail::extrude_polyline(pts, n, half_width, z_lift);
    if (ribbon.empty()) return ribbon;
    const auto stripIdx = detail::extrude_polyline_indices(static_cast<uint32_t>(ribbon.size() / 2));
    std::vector<Vec3> flat(stripIdx.size());
    for (size_t k = 0; k < stripIdx.size(); ++k) flat[k] = ribbon[stripIdx[k]];
    return flat;
}

// Road-surface fill (Epic 3 Task 1 / VM-036, decision #5): zips the
// adapter's two-rail encoding (point_count == 2*kRoadFillSamples;
// points[0..n) left rail, points[n..2n) right rail, index-parallel by
// station) into a triangle strip. No polygon-clipping code, no resampling
// here -- the adapter already resampled both rails to the same fixed
// station count, so this is a plain zip. Malformed guard: fewer than 4
// points or an odd point_count (can't split evenly into two rails) yields
// an empty strip -- silently dropped, not malformed (spec §9).
std::vector<Vec3> build_road_strip(const Vec3* pts, uint32_t point_count, float z_lift) {
    std::vector<Vec3> tris;
    if (point_count < 4 || point_count % 2 != 0) return tris;
    const uint32_t n = point_count / 2;
    const Vec3* left = pts;
    const Vec3* right = pts + n;
    const auto lift = [&](Vec3 v) {
        v.z += z_lift;
        return v;
    };
    tris.reserve(static_cast<size_t>(n - 1) * 6);
    for (uint32_t i = 0; i + 1 < n; ++i) {
        const Vec3 l0 = lift(left[i]), l1 = lift(left[i + 1]);
        const Vec3 r0 = lift(right[i]), r1 = lift(right[i + 1]);
        tris.push_back(l0);
        tris.push_back(l1);
        tris.push_back(r1);
        tris.push_back(l0);
        tris.push_back(r1);
        tris.push_back(r0);
    }
    return tris;
}

// Staleness fade (Epic 3 Task 2 / VM-034) — the exact per-entity
// clay_translucent.mat MaterialInstance-swap mechanism objects.cpp's
// update_entity_staleness()/alert_polygons.cpp's rebind_slot_material()
// already established (see the Epic 2 plan's "…and the material that can
// actually do it"), specialized for a single-mesh slot the same way
// alert_polygons.cpp's rebind_slot_material() is: fresh (alpha>=1.0) stays
// on the shared OPAQUE per-kind template (material_for_kind()), no
// per-entity instance; fading swaps to a clay_translucent.mat instance
// seeded from tint_for_kind()'s stored tint, alpha set every call.
//
// `ego_valid=false` (Epic 2 gate finding, the ego-invalid map cosmetic —
// renderer.cpp:1458-1470's update_ground_grid_transform() snaps the
// ego-following ground/grid patch to the world origin whenever
// ego.valid==0, but update_map_elements() had no matching gate at all, so
// real map geometry kept rendering against an origin-snapped ground) drives
// alpha to 0 via this SAME fade path, multiplying staleness down to zero
// exactly like alert_polygons.cpp's kAlertSeverityAlpha*staleness — not a
// second mechanism, and not skip-and-freeze (which would leave the LAST
// valid frame's geometry at full opacity forever, the identical bug one
// frame later).
void apply_map_element_staleness(VisualRenderer& r, Mesh& mesh, MapKind kind,
                                  double last_update_sec, double sim_time_sec, bool ego_valid) {
    if (!mesh.entity) return;
    const auto staleness = static_cast<float>(detail::SceneBuffer::staleness_alpha(
        sim_time_sec, last_update_sec, kStaleFadeStartSec, kStaleFadeTimeoutSec));
    const float alpha = ego_valid ? staleness : 0.0f;

    filament::RenderableManager& rm = r.engine->getRenderableManager();
    const auto ri = rm.getInstance(mesh.entity);

    if (alpha >= 1.0f) {
        if (mesh.fadeInstance != nullptr) {
            if (ri.isValid()) rm.setMaterialInstanceAt(ri, 0, material_for_kind(r, kind));
            r.engine->destroy(mesh.fadeInstance);
            mesh.fadeInstance = nullptr;
        }
        mesh.fadeAlpha = 1.0f;
        return;
    }
    if (mesh.fadeInstance == nullptr) {
        mesh.fadeInstance = r.clayTranslucentMaterial->createInstance();
        mesh.fadeInstance->setCullingMode(filament::backend::CullingMode::NONE);
        if (ri.isValid()) rm.setMaterialInstanceAt(ri, 0, mesh.fadeInstance);
    }
    const detail::Float3 tint = tint_for_kind(r, kind);
    mesh.fadeInstance->setParameter("baseColor", float4{tint.r, tint.g, tint.b, alpha});
    mesh.fadeInstance->setParameter("roughness", r.active_theme.material.roughness);
    mesh.fadeInstance->setParameter("metallic", r.active_theme.material.metallic);
    mesh.fadeAlpha = alpha;
}

}  // namespace

// Epic 3 Task 2 (VM-034): the HD-map category now fades via the one shared
// staleness_alpha() path every other category uses (apply_map_element_
// staleness(), above) — closes Epic 2's stated deviation ("the HD-map
// category pops, it does not fade", epic2 plan lines 228-238). This
// function still renders whatever the last set_scene() call handed it; the
// fade is applied per-mesh, right after each is adopted or (re)built, below.
void update_map_elements(VisualRenderer& r, const SceneGraph& s) {
    std::unordered_map<uint64_t, Mesh> next;
    next.reserve(r.mapElementMeshes.size());

    // Epic 3 Task 2 (VM-034): `kind`/`last_update_sec` are the source
    // MapElement's own — every chunk/dash of one element shares them — and
    // drive the staleness-fade pass applied at the end, below, on BOTH the
    // adopt path and the freshly-built path (not just one), the same way
    // every other category's per-frame diff pass re-evaluates staleness on
    // every live entity regardless of whether ITS geometry changed this
    // frame. `s.ego.valid` is read directly (this lambda already captures
    // `s` by reference) — not threaded through as its own parameter.
    auto adopt_or_build = [&](uint64_t key, filament::MaterialInstance* material, MapKind kind,
                               double last_update_sec, auto build_fn) {
        auto it = r.mapElementMeshes.find(key);
        if (it != r.mapElementMeshes.end()) {
            next.emplace(key, std::move(it->second));
            r.mapElementMeshes.erase(it);
        } else {
            // Epic 3 Task 1 (VM-036) Step 7: this IS the cache-miss branch
            // Epic 2's untested "cached, no per-frame rebuild" AC needs a
            // counter for -- incremented here, not after build_fn() runs, so
            // an empty-result build (malformed geometry) still counts as an
            // attempted rebuild, not a silent no-op.
            ++r.mapElementRebuildCount;
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
                     filament::RenderableManager::PrimitiveType::TRIANGLES, material,
                     /*cast_shadows=*/false, /*receive_shadows=*/true);
            next.emplace(key, std::move(mesh));
        }
        apply_map_element_staleness(r, next.at(key), kind, last_update_sec, s.sim_time_sec,
                                     s.ego.valid != 0);
    };

    for (uint32_t i = 0; i < s.map_element_count; ++i) {
        const MapElement& e = s.map_elements[i];
        if (e.points == nullptr || e.point_count < 2) continue;  // malformed guard

        filament::MaterialInstance* const material = material_for_kind(r, e.kind);
        const float z_lift = z_lift_for_kind(e.kind);

        if (e.kind == MapKind::ROAD_SURFACE) {
            // Two-rail encoding (decision #5) -- never is_polygon, never
            // dashed, its own triangulation entirely.
            const uint64_t key = chunk_signature(false, e.points, e.point_count);
            adopt_or_build(key, material, e.kind, e.last_update_sec,
                           [&]() { return build_road_strip(e.points, e.point_count, z_lift); });
        } else if (e.is_polygon) {
            const uint64_t key = chunk_signature(true, e.points, e.point_count);
            adopt_or_build(key, material, e.kind, e.last_update_sec, [&]() {
                std::vector<Vec3> hatch;
                if (e.kind == MapKind::CROSSWALK) {
                    hatch = detail::build_crosswalk_hatch(e.points, e.point_count, z_lift);
                }
                if (!hatch.empty()) return hatch;
                // Not a hatched crosswalk quad -- no safe hatch reading;
                // fall back to a plain solid fan fill so the polygon still
                // renders as SOMETHING rather than nothing.
                return detail::triangulate_convex_polygon(e.points, e.point_count, z_lift);
            });
        } else if (IsBoundaryKind(e.kind)) {
            // Dashing (decision #3): moved here from the adapter, now
            // gated on LEFT_BOUNDARY/RIGHT_BOUNDARY instead of centerline
            // -- CENTERLINE falls through to its own dot-disc path below;
            // ROAD_EDGE falls through to the plain solid polyline path
            // further below (user directive 2026-09-08: the road's outer
            // edge is SOLID, never dashed -- it isn't a BOUNDARY kind).
            for (auto& dash : chop_into_dashes(e.points, e.point_count)) {
                for (auto [a, b] : detail::polyline_chunks(static_cast<uint32_t>(dash.size()))) {
                    const uint32_t chunkStart = a;  // structured bindings can't be captured directly
                    const uint32_t n = b - a;
                    const uint64_t key = chunk_signature(false, dash.data() + chunkStart, n);
                    adopt_or_build(key, material, e.kind, e.last_update_sec,
                                   [dash, chunkStart, n, z_lift]() {
                        return build_ribbon_flat(dash.data() + chunkStart, n, kLaneHalfWidthM, z_lift);
                    });
                }
            }
        } else if (e.kind == MapKind::CENTERLINE) {
            // Dot guidance (user directive 2026-09-08): circles along the
            // line instead of a filled ribbon strip -- chunked the same way
            // the plain polyline path below is (uint16 index-ceiling guard),
            // even though a dot-disc mesh is far smaller than a ribbon of
            // the same point count.
            for (auto [a, b] : detail::polyline_chunks(e.point_count)) {
                const uint32_t n = b - a;
                const Vec3* chunkPts = e.points + a;
                const uint64_t key = chunk_signature(false, chunkPts, n);
                adopt_or_build(key, material, e.kind, e.last_update_sec, [chunkPts, n, z_lift]() {
                    return build_centerline_dots(chunkPts, n, z_lift);
                });
            }
        } else {
            for (auto [a, b] : detail::polyline_chunks(e.point_count)) {
                const uint32_t n = b - a;
                const Vec3* chunkPts = e.points + a;
                const uint64_t key = chunk_signature(false, chunkPts, n);
                adopt_or_build(key, material, e.kind, e.last_update_sec, [chunkPts, n, z_lift]() {
                    return build_ribbon_flat(chunkPts, n, kLaneHalfWidthM, z_lift);
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
