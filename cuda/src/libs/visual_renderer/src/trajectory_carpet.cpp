// trajectory_carpet.cpp — output_trajectory_carpet (VM-077), rendered as a
// velocity-colored RIBBON extruded from adapter-derived centerline stations
// (redirected 2026-09-10 per user directive; see the plan doc's 2026-09-10
// section for the full flicker-investigation history, including the
// disproven H2 hypothesis and the confound-controlled re-measurement --
// not retold here).
//
// Half-width comes from the theme token ribbon.margin_velocity_m (clamped
// to kRibbonMinHalfWidthM, the same floor ribbon.cpp's three PathRole
// ribbons clamp against). Geometry reuses ribbon.cpp's own
// extrude_polyline()/extrude_polyline_indices()/polyline_chunks() machinery
// verbatim -- no second extruder. Per-vertex color stays: each extruded
// rail vertex takes its source station's color.
//
// Content signature mirrors ribbon_signature()'s shape (point count +
// first/last point + half-width + ego-clip state) with deliberately NO
// color term -- load-bearing: color-only drift (the producer's baked
// velocity gradient changing while stations don't move) causes zero
// rebuilds, an explicit accepted tradeoff, not an oversight. The displayed
// color freezes at the last-built values until the next position-changing
// rebuild -- no separate "recolor without rebuilding geometry" fast path
// exists anywhere in this library (ribbon.cpp/point_cloud.cpp don't have
// one either), and inventing one is new-scope machinery flagged back, not
// silently built.
//
// scene.h UNCHANGED: TrajectoryCarpet::points (PointCloudPoint, reused
// verbatim) already fit a centerline-plus-per-station-color encoding with
// zero struct changes; only the adapter's interpretation (raw wire vertex
// -> centerline station) and this file's geometry (flat triangle list ->
// extruded ribbon) changed.
//
// Opacity: OPAQUE while fresh, per the user directive -- staleness_alpha()
// alone drives the shared MaterialInstance's "alpha" uniform now; the old
// measured-producer-alpha parity (0.7) is superseded, not reused.
#include "trajectory_carpet.hpp"
#include "trajectory_carpet_test_hooks.hpp"
#include "polyline.hpp"
#include "renderer_internal.hpp"
#include "theme.hpp"
#include "visual_renderer/scene.h"

#include <filament/RenderableManager.h>

#include <math/vec3.h>

#include <utils/EntityManager.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <utility>
#include <vector>

namespace mpviz {

namespace {

using filament::math::float3;

// Same generous half-extent reasoning as point_cloud.cpp's
// kPointCloudBoundsM -- a velocity ribbon can run the length of a planning
// horizon, well past the 60m ground/lane geometry elsewhere. culling(false)
// below makes this a formality, same convention as every other category.
constexpr float kTrajectoryCarpetBoundsM = 200.0f;

// Z-lift for the velocity ribbon: BETWEEN LOCAL (ribbon.cpp's
// kRibbonZLiftByRoleM[LOCAL] == 0.046) and BEHAVIOR (kRibbonZLiftByRoleM
// [BEHAVIOR] == 0.05) -- "stacked on top of local ribbon" (user directive)
// while keeping the BEHAVIOR hero ribbon topmost, exactly the reasoning
// ribbon.cpp's own per-role stagger doc states for why GLOBAL/LOCAL/
// BEHAVIOR don't share a plane. Also strictly below alert_polygons.cpp's
// kAlertZLiftM (0.06, topmost) for the same reason the retired
// kTrajectoryCarpetZLiftM was: collision rings on the ego's own corridor
// must stay visible above every path/ribbon layer. Retires
// kTrajectoryCarpetZLiftM (0.055, chosen only to clear BEHAVIOR from
// "above everything" carpet-era thinking that no longer applies once this
// renders IN the ribbon stack rather than over it).
constexpr float kVelocityRibbonZLiftM = 0.052f;  // widened stagger 2026-09-10, between LOCAL 0.046 and BEHAVIOR 0.058

// One trajectory-carpet vertex: world-space position + packed rgba8, EXACTLY
// point_cloud.cpp's PointVertex layout (D1: PointCloudPoint reused verbatim
// as the wire vertex type, so the GPU-side vertex struct matches too -- a
// second, byte-identical struct only exists here because point_cloud.cpp's
// is file-local (anonymous namespace), not because the layout differs).
struct CarpetVertex {
    float3 position;
    uint32_t rgba;
};

filament::VertexBuffer* make_carpet_vertex_buffer(filament::Engine& engine,
                                                   std::vector<CarpetVertex> verts) {
    auto* heap = new std::vector<CarpetVertex>(std::move(verts));
    filament::VertexBuffer* vb =
        filament::VertexBuffer::Builder()
            .vertexCount(static_cast<uint32_t>(heap->size()))
            .bufferCount(1)
            .attribute(filament::VertexAttribute::POSITION, 0,
                       filament::VertexBuffer::AttributeType::FLOAT3,
                       offsetof(CarpetVertex, position), sizeof(CarpetVertex))
            .attribute(filament::VertexAttribute::COLOR, 0,
                       filament::VertexBuffer::AttributeType::UBYTE4,
                       offsetof(CarpetVertex, rgba), sizeof(CarpetVertex))
            .normalized(filament::VertexAttribute::COLOR)
            .build(engine);
    vb->setBufferAt(
        engine, 0,
        filament::VertexBuffer::BufferDescriptor(
            heap->data(), heap->size() * sizeof(CarpetVertex),
            [](void*, size_t, void* user) { delete static_cast<std::vector<CarpetVertex>*>(user); },
            heap));
    return vb;
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

// Content signature for the velocity ribbon: mirrors ribbon.cpp's
// ribbon_signature() shape EXACTLY (point count + first/last point +
// half-width + ego-clip state) with NO color term -- see this file's own
// header comment for why that omission is deliberate, not an
// oversight. half_width_m hashed as its bit pattern (exact reproducibility
// across calls with the same width matters, not numeric comparison), same
// convention ribbon_signature() uses.
uint64_t trajectory_carpet_signature(const Vec3* pts, uint32_t n, float half_width_m,
                                       bool clip_active, int64_t clip_units) {
    uint64_t h = hash_combine(0, static_cast<uint64_t>(n));
    if (n > 0) {
        h = hash_combine(h, hash_vec3(pts[0]));
        h = hash_combine(h, hash_vec3(pts[n - 1]));
    }
    uint32_t widthBits;
    std::memcpy(&widthBits, &half_width_m, sizeof(widthBits));
    h = hash_combine(h, static_cast<uint64_t>(widthBits));
    h = hash_combine(h, clip_active ? 1ULL : 0ULL);
    h = hash_combine(h, static_cast<uint64_t>(clip_units));
    return h;
}

// Effective half-width for the velocity ribbon: same clamp shape as
// ribbon.cpp's build_effective_half_width(), against the new
// ribbon.margin_velocity_m token instead of a PathRole's margin (this
// category has no role) -- ponytail: duplicated 2-line formula rather than
// generalizing build_effective_half_width() to take a raw margin, since
// ribbon.cpp's version is keyed by PathRole and the shapes don't quite
// match; promote to a shared helper if a third margin-driven half-width
// caller shows up. kRibbonMinHalfWidthM (renderer_internal.hpp) is the same
// shared floor ribbon.cpp's three roles clamp against.
float velocity_ribbon_half_width_m(const detail::Theme::Ribbon& cfg) {
    const float halfWidth = (cfg.lane_width_m - 2.0f * cfg.margin_velocity_m) * 0.5f;
    return std::max(halfWidth, kRibbonMinHalfWidthM);
}

// Unpacks PointCloudPoint::rgba per scene.h's documented convention and
// substitutes the theme's neutral token whenever alpha==0 -- IDENTICAL
// sentinel rule to point_cloud.cpp's own resolve_rgba() (ponytail: this is
// the second caller of the 3-line helper, per that file's own comment).
uint32_t resolve_rgba(const VisualRenderer& r, uint32_t packed) {
    const uint32_t a = (packed >> 24) & 0xFFu;
    if (a != 0) return packed;
    const detail::Float3& tint = r.active_theme.palette.object_tints.unknown;
    const auto to_byte = [](float c) {
        return static_cast<uint32_t>(std::clamp(c, 0.0f, 1.0f) * 255.0f + 0.5f);
    };
    return to_byte(tint.r) | (to_byte(tint.g) << 8) | (to_byte(tint.b) << 16) | (255u << 24);
}

// Cleans a centerline-station array per polyline.cpp's clean_polyline()
// rule (truncate at the first non-finite position; drop an exact-duplicate-
// position station), carrying each surviving station's own rgba alongside
// -- extrude_polyline() itself only knows Vec3, so this is the "make a
// colour-carrying station list I can extrude and still know which colour
// belongs to which output vertex pair" step. Mirrors polyline.cpp's private
// clean_polyline() exactly (same 1e-9 epsilon) so a chunk sliced from the
// result and handed to extrude_polyline() is already clean -- extrude_
// polyline's OWN internal cleaning becomes a no-op, so pair i of its output
// always corresponds to cleaned[i] (see build_slot_meshes() below).
std::vector<PointCloudPoint> clean_carpet_points(const PointCloudPoint* pts, uint32_t n) {
    std::vector<PointCloudPoint> out;
    if (pts == nullptr) return out;
    out.reserve(n);
    constexpr double kEps = 1e-9;
    for (uint32_t i = 0; i < n; ++i) {
        const Vec3& p = pts[i].position;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) break;
        if (!out.empty()) {
            const Vec3& prev = out.back().position;
            const double dx = p.x - prev.x, dy = p.y - prev.y, dz = p.z - prev.z;
            if ((dx * dx + dy * dy + dz * dz) < (kEps * kEps)) continue;  // zero-length segment
        }
        out.push_back(pts[i]);
    }
    return out;
}

// Ego-proximity forward clip, colour-carrying: walks the identical 2D arc
// length detail::clip_polyline_forward() walks, but keeps each surviving
// station's rgba alongside the (possibly interpolated) position -- the
// interpolated cut point takes the EARLIER station's colour (no colour
// interpolation; the GPU's own triangle rasterization already blends it
// toward the next station's colour across the following face, same as
// every other station-to-station transition). Ponytail: this duplicates
// clip_polyline_forward()'s ~15-line walk rather than adding a colour
// parameter to that shared, Filament-free, colour-agnostic helper -- promote
// to a shared "clip with payload" helper if a third colour-carrying caller
// ever needs one.
std::vector<PointCloudPoint> clip_carpet_forward(const PointCloudPoint* pts, uint32_t n, double s0) {
    std::vector<PointCloudPoint> out;
    if (n < 2) return out;
    double cum = 0.0;
    for (uint32_t i = 0; i + 1 < n; ++i) {
        const Vec3& a = pts[i].position;
        const Vec3& b = pts[i + 1].position;
        const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
        const double segLen = std::sqrt(dx * dx + dy * dy);
        const bool isLastSeg = (i + 2 == n);
        if (cum + segLen >= s0 || isLastSeg) {
            const double t = segLen > 0.0 ? std::clamp((s0 - cum) / segLen, 0.0, 1.0) : 0.0;
            PointCloudPoint cut{};
            cut.position = Vec3{a.x + dx * t, a.y + dy * t, a.z + dz * t};
            cut.rgba = pts[i].rgba;  // earlier station's colour, see comment above
            out.push_back(cut);
            for (uint32_t k = i + 1; k < n; ++k) out.push_back(pts[k]);
            return out;
        }
        cum += segLen;
    }
    out.push_back(pts[n - 1]);  // defensive fallback, same as clip_polyline_forward's own
    return out;
}

void destroy_slot_meshes(VisualRenderer& r, VisualRenderer::TrajectoryCarpetSlot& slot) {
    for (auto& m : slot.meshes) destroy_mesh(*r.engine, *r.scene, m);
    slot.meshes.clear();
    slot.firstMeshRgba.clear();
    slot.firstMeshZ.clear();
}

// Rebuilds `slot`'s geometry from `pts`/`n` (centerline stations: position +
// packed rgba per station) as an extruded ribbon strip at `halfWidthM`,
// chunked at polyline_chunks()'s LINE_STRIP-shaped overlap-by-one ceiling --
// exactly ribbon.cpp's build_slot_meshes() shape, extrude_polyline()/
// extrude_polyline_indices() reused verbatim, with per-vertex colour taken
// from the source station instead of a per-role material tint.
void build_slot_meshes(VisualRenderer& r, VisualRenderer::TrajectoryCarpetSlot& slot,
                        const PointCloudPoint* pts, uint32_t n, float halfWidthM) {
    destroy_slot_meshes(r, slot);
    slot.totalVertexCount = 0;
    slot.halfWidthM = halfWidthM;

    const std::vector<PointCloudPoint> cleaned = clean_carpet_points(pts, n);
    if (cleaned.size() < 2) return;  // nothing to extrude -- extrude_polyline's own floor

    std::vector<Vec3> positions(cleaned.size());
    for (size_t i = 0; i < cleaned.size(); ++i) positions[i] = cleaned[i].position;

    bool first_chunk = true;
    for (auto [a, b] : detail::polyline_chunks(static_cast<uint32_t>(cleaned.size()))) {
        const uint32_t chunkN = b - a;
        std::vector<Vec3> strip =
            detail::extrude_polyline(positions.data() + a, chunkN, halfWidthM, kVelocityRibbonZLiftM);
        if (strip.empty()) continue;
        std::vector<uint16_t> idx =
            detail::extrude_polyline_indices(static_cast<uint32_t>(strip.size() / 2));
        if (idx.empty()) continue;

        // `cleaned` is already clean, so a contiguous slice of it handed to
        // extrude_polyline() above triggers no further internal drops --
        // strip.size() == 2*chunkN exactly, and pair i (vertices 2i/2i+1)
        // corresponds to cleaned[a+i] one-to-one.
        std::vector<CarpetVertex> verts(strip.size());
        for (size_t i = 0; i < strip.size(); ++i) {
            const size_t stationIdx = a + i / 2;
            verts[i].position = float3{static_cast<float>(strip[i].x), static_cast<float>(strip[i].y),
                                        static_cast<float>(strip[i].z)};
            verts[i].rgba = resolve_rgba(r, cleaned[stationIdx].rgba);
        }
        if (first_chunk) {
            // Test-hook mirror only (trajectory_carpet_test_hooks.hpp) --
            // NOT a Filament read-back, same reasoning as RibbonSlot's
            // firstPointM/halfWidthM (renderer_internal.hpp).
            slot.firstMeshRgba.reserve(verts.size());
            slot.firstMeshZ.reserve(verts.size());
            for (const auto& v : verts) {
                slot.firstMeshRgba.push_back(v.rgba);
                slot.firstMeshZ.push_back(v.position.z);
            }
            first_chunk = false;
        }
        slot.totalVertexCount += static_cast<uint32_t>(verts.size());
        Mesh mesh;
        mesh.vertexCount = static_cast<uint32_t>(verts.size());
        mesh.vb = make_carpet_vertex_buffer(*r.engine, std::move(verts));
        mesh.ib = make_index_buffer(*r.engine, std::move(idx));
        mesh.entity = utils::EntityManager::get().create();
        filament::RenderableManager::Builder(1)
            .boundingBox({{0, 0, 0},
                          {kTrajectoryCarpetBoundsM, kTrajectoryCarpetBoundsM,
                           kTrajectoryCarpetBoundsM}})
            .geometry(0, filament::RenderableManager::PrimitiveType::TRIANGLES, mesh.vb, mesh.ib)
            .material(0, r.trajectoryCarpetMaterialInstance)
            .culling(false)
            .castShadows(false)
            .receiveShadows(false)
            .build(*r.engine, mesh.entity);
        r.scene->addEntity(mesh.entity);
        slot.meshes.push_back(std::move(mesh));
    }
}

}  // namespace

void update_trajectory_carpets(VisualRenderer& r, const SceneGraph& s) {
    while (r.trajectoryCarpetSlots.size() > s.trajectory_carpet_count) {
        destroy_slot_meshes(r, r.trajectoryCarpetSlots.back());
        r.trajectoryCarpetSlots.pop_back();
    }
    if (r.trajectoryCarpetSlots.size() < s.trajectory_carpet_count) {
        r.trajectoryCarpetSlots.resize(s.trajectory_carpet_count);
    }

    const float halfWidthM = velocity_ribbon_half_width_m(r.active_theme.ribbon);

    // ONE MaterialInstance, ONE alpha: same "freshest of several wins" N>1
    // policy as point_cloud.cpp (today's shipped profile carries exactly one
    // carpet row, so this never fires in practice).
    float alpha = s.trajectory_carpet_count > 0 ? 0.0f : 1.0f;
    for (uint32_t i = 0; i < s.trajectory_carpet_count; ++i) {
        const TrajectoryCarpet& tc = s.trajectory_carpets[i];
        VisualRenderer::TrajectoryCarpetSlot& slot = r.trajectoryCarpetSlots[i];

        // Ego-proximity clip: never render the part of the velocity ribbon
        // behind the ego -- identical mechanism/quantization/gate every
        // other ribbon uses (polyline.hpp's compute_polyline_clip(), shared
        // with ribbon.cpp), computed against the RAW (pre-clean) station
        // positions, same as ribbon.cpp does against ribbon.points.
        std::vector<Vec3> rawPositions(tc.point_count);
        for (uint32_t j = 0; j < tc.point_count; ++j) rawPositions[j] = tc.points[j].position;
        const detail::PolylineClip clip =
            s.ego.valid ? detail::compute_polyline_clip(rawPositions.data(), tc.point_count,
                                                          s.ego.position)
                        : detail::PolylineClip{};

        const uint64_t sig = trajectory_carpet_signature(rawPositions.data(), tc.point_count,
                                                            halfWidthM, clip.active,
                                                            clip.quantized_units);
        if (!slot.has_signature || slot.signature != sig) {
            std::vector<PointCloudPoint> clippedStorage;
            const PointCloudPoint* geomPts = tc.points;
            uint32_t geomN = tc.point_count;
            if (clip.active) {
                clippedStorage = clip_carpet_forward(tc.points, tc.point_count, clip.station_m);
                geomPts = clippedStorage.data();
                geomN = static_cast<uint32_t>(clippedStorage.size());
            }
            ++r.trajectoryCarpetRebuildCount;
            build_slot_meshes(r, slot, geomPts, geomN, halfWidthM);
            slot.signature = sig;
            slot.has_signature = true;
        }

        alpha = std::max(alpha, static_cast<float>(detail::SceneBuffer::staleness_alpha(
                                     s.sim_time_sec, tc.last_update_sec, kStaleFadeStartSec,
                                     kStaleFadeTimeoutSec)));
    }
    // OPAQUE while fresh (user directive, 2026-09-10) -- staleness_alpha()
    // alone drives this uniform now; the old kTrajectoryCarpetBaseAlpha
    // (0.7, the measured producer m.color.a) is retired, not multiplied in.
    r.trajectoryCarpetMaterialInstance->setParameter("alpha", alpha);
    r.trajectoryCarpetAlpha = alpha;
}

}  // namespace mpviz

// Filament-free test introspection hooks; see trajectory_carpet_test_hooks.hpp
// for why these live here.
namespace mpviz::testing {

size_t trajectory_carpet_mesh_count(mpviz::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->trajectoryCarpetSlots.size()) return 0;
    return r->trajectoryCarpetSlots[slot].meshes.size();
}

size_t trajectory_carpet_vertex_count(mpviz::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->trajectoryCarpetSlots.size()) return 0;
    return r->trajectoryCarpetSlots[slot].totalVertexCount;
}

float trajectory_carpet_material_alpha(mpviz::VisualRenderer* r) {
    if (r == nullptr) return 1.0f;
    return r->trajectoryCarpetAlpha;
}

uint32_t trajectory_carpet_vertex_rgba(mpviz::VisualRenderer* r, size_t slot, size_t vertex_idx) {
    if (r == nullptr || slot >= r->trajectoryCarpetSlots.size()) return 0;
    const auto& rgba = r->trajectoryCarpetSlots[slot].firstMeshRgba;
    if (vertex_idx >= rgba.size()) return 0;
    return rgba[vertex_idx];
}

float trajectory_carpet_vertex_z(mpviz::VisualRenderer* r, size_t slot, size_t vertex_idx) {
    if (r == nullptr || slot >= r->trajectoryCarpetSlots.size()) return 0.0f;
    const auto& z = r->trajectoryCarpetSlots[slot].firstMeshZ;
    if (vertex_idx >= z.size()) return 0.0f;
    return z[vertex_idx];
}

float trajectory_carpet_half_width_m(mpviz::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->trajectoryCarpetSlots.size()) return 0.0f;
    return r->trajectoryCarpetSlots[slot].halfWidthM;
}

uint64_t trajectory_carpet_rebuild_count(mpviz::VisualRenderer* r) {
    if (r == nullptr) return 0;
    return r->trajectoryCarpetRebuildCount;
}

}  // namespace mpviz::testing
