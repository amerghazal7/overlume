// ribbon.cpp — path ribbons in three roles (BEHAVIOR/GLOBAL/LOCAL), the
// behavior ribbon as the emissive bloom hero. Geometry reuses
// extrude_polyline/extrude_polyline_indices/polyline_chunks (polyline.hpp)
// — no second extruder.
#include "ribbon.hpp"
#include "ribbon_test_hooks.hpp"
#include "polyline.hpp"
#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

#include <filament/RenderableManager.h>

#include <math/vec3.h>
#include <math/vec4.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <utility>
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

// Content signature for one ribbon slot (not a chunk-within-the-ribbon —
// see renderer_internal.hpp's RibbonSlot comment): role + point count +
// first/last point + half-width + clip state. Role is included so a slot
// re-homed to a different role still rebuilds and re-binds to the new
// role's material, even if point data happened to be identical.
// half_width_m is included because a set_theme() that changes only a
// margin touches no PathRibbon point data at all — without the effective
// width in the signature, a mid-transition margin change would be
// silently ignored until some unrelated content change forced a rebuild.
// Hashed as its bit pattern (not the float value): exact reproducibility
// across calls with the same width matters here, not numeric comparison.
//
// clip_active/clip_units (ego-proximity ribbon clip) fold in so a slot
// rebuilds when the quantized clip station moves. clip_units is hashed as
// a plain integer (not the quantized station's float bit pattern) — it's
// already an exact multiple of kRibbonClipQuantizeM by construction, so
// two frames landing on the same station always hash identically; a
// parked ego (or one outside the proximity gate) recomputes the same pair
// every call, so this causes zero rebuilds (see update_ribbons()).
uint64_t ribbon_signature(PathRole role, const Vec3* pts, uint32_t n, float half_width_m,
                           bool clip_active, int64_t clip_units) {
    uint64_t h = hash_combine(0, static_cast<uint64_t>(role));
    h = hash_combine(h, static_cast<uint64_t>(n));
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

// z-lift for the extruded ribbon strip -- above both neighbours it can
// cross: the 0.02m lane paint and objects.cpp's 0.03m predicted-path
// ribbons (at 0.03 a behavior/local ribbon crossing a tracked object's
// predicted path was coplanar and z-fought it).
//
// Half-width is not a fixed constant here -- it comes from
// build_effective_half_width() below, derived from the active theme's
// ribbon.lane_width_m and the role's own margin at build time
// (build_slot_meshes() below). 0.24 remains Theme::Ribbon's own
// soft-default seed for the margin fields (theme.hpp/theme.cpp), so an
// older theme file renders identically.
// Per-role stagger, not one shared plane: BEHAVIOR and LOCAL routinely
// trace the same route (planner output vs velocity path), and at a shared
// z they z-fight into a patchy interleave. Order: GLOBAL lowest, LOCAL
// middle, BEHAVIOR (the hero) on top.
constexpr float kRibbonZLiftByRoleM[3] = {0.05f,   // BEHAVIOR (PathRole 0)
                                          0.040f,  // GLOBAL   (PathRole 1)
                                          0.045f}; // LOCAL    (PathRole 2)

// Fixed half-width floor: per-role effective half-width is clamped to
// never go narrower than this, however aggressively a theme authors its
// margins.
constexpr float kRibbonMinHalfWidthM = 0.12f;

// Per-role extruded half-width: a ribbon doesn't fully occupy
// theme.ribbon.lane_width_m -- each role's own margin
// (margin_behavior_m/margin_global_m/margin_local_m) eats into both sides,
// so a lower-z-lifted ribbon of a different color peeks out as a colored
// rim around whichever role sits above it in the z-stagger
// (kRibbonZLiftByRoleM). theme.cpp's parse() derives the margin defaults
// from the old ribbon.width_m field so a pre-existing theme YAML
// reproduces today's 0.24m strip exactly — see its own comment.
float role_margin_m(const detail::Theme::Ribbon& cfg, PathRole role) {
    switch (role) {
        case PathRole::BEHAVIOR:
            return cfg.margin_behavior_m;
        case PathRole::GLOBAL:
            return cfg.margin_global_m;
        case PathRole::LOCAL:
        default:
            return cfg.margin_local_m;
    }
}

float build_effective_half_width(const detail::Theme::Ribbon& cfg, PathRole role) {
    const float halfWidth = (cfg.lane_width_m - 2.0f * role_margin_m(cfg, role)) * 0.5f;
    return std::max(halfWidth, kRibbonMinHalfWidthM);
}

// Ego-proximity ribbon clip: never render the part of the ribbon behind
// the ego. Finds the ribbon polyline's closest-approach arc-station to
// `ego` (2D -- x/y only, map frame) by walking every segment once; returns
// {station, min lateral distance}. n<2 -> {0, +inf} (nothing to clip
// against -- the +inf distance also fails the proximity gate below, so an
// empty/degenerate ribbon is never clipped).
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

// Proximity gate: only clip a ribbon the ego is actually near -- a
// far-away GLOBAL route (the planned destination route, routinely
// kilometers of it loaded at once) must render whole, untouched. 5.0m is
// deliberately generous versus the ~0.1-0.5m half-widths ribbons actually
// draw at -- this is "is the ego riding this route at all", not a precise
// lateral-offset threshold.
constexpr float kRibbonEgoClipLateralM = 5.0f;
// Arc-length quantization: the clip station folds into ribbon_signature()
// only at this granularity, so a parked ego causes zero rebuilds and a
// moving one rebuilds a few times a second, not every frame for a
// sub-millimeter station drift.
constexpr float kRibbonClipQuantizeM = 0.5f;

struct RibbonClip {
    bool active = false;
    int64_t quantizedUnits = 0;  // station / kRibbonClipQuantizeM, rounded -- meaningful iff active
    double stationM = 0.0;       // quantizedUnits * kRibbonClipQuantizeM -- meaningful iff active
};

// Clip applies only when ego.valid -- an invalid ego has no real position
// to clip against (EgoState::valid's "0 = no TF yet" contract, scene.h).
RibbonClip compute_ribbon_clip(const PathRibbon& ribbon, const EgoState& ego) {
    RibbonClip clip;
    if (!ego.valid) return clip;
    const auto [station, dist] = closest_arc_station(ribbon.points, ribbon.point_count, ego.position);
    if (dist >= kRibbonEgoClipLateralM) return clip;  // proximity gate: too far, render whole
    clip.active = true;
    // ceil, not lround: round-to-nearest would land the cut up to 0.25m
    // behind the closest-approach station about half the time, rendering
    // part of the ribbon behind the ego. ceil keeps the cut at-or-ahead,
    // equally deterministic (parked-ego zero-rebuild property unchanged).
    clip.quantizedUnits = static_cast<int64_t>(std::ceil(station / kRibbonClipQuantizeM));
    clip.stationM = static_cast<double>(clip.quantizedUnits) * kRibbonClipQuantizeM;
    return clip;
}

// Truncates `pts`/`n` to the forward half starting at arc-length `s0`,
// with an interpolated point at the cut (not a snap to the nearest
// original vertex). Walks the same 2D arc length closest_arc_station()
// computed `s0` against, so the cut lands exactly where that search says
// it should. n<2 -> empty.
std::vector<Vec3> clip_ribbon_forward(const Vec3* pts, uint32_t n, double s0) {
    std::vector<Vec3> out;
    if (n < 2) return out;
    double cum = 0.0;
    for (uint32_t i = 0; i + 1 < n; ++i) {
        const Vec3& a = pts[i];
        const Vec3& b = pts[i + 1];
        const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
        const double segLen = std::sqrt(dx * dx + dy * dy);
        const bool isLastSeg = (i + 2 == n);
        if (cum + segLen >= s0 || isLastSeg) {
            const double t = segLen > 0.0 ? std::clamp((s0 - cum) / segLen, 0.0, 1.0) : 0.0;
            out.push_back(Vec3{a.x + dx * t, a.y + dy * t, a.z + dz * t});
            for (uint32_t k = i + 1; k < n; ++k) out.push_back(pts[k]);
            return out;
        }
        cum += segLen;
    }
    // Unreachable in practice (isLastSeg always fires by the final
    // segment); kept as a defensive fallback so this renders something
    // rather than silently dropping the ribbon.
    out.push_back(pts[n - 1]);
    return out;
}

// Destroys every mesh in `slot.meshes` (independent of role/fade state --
// called both on a full content-signature rebuild and on slot release) and
// clears the vector.
void destroy_slot_meshes(VisualRenderer& r, VisualRenderer::RibbonSlot& slot) {
    for (auto& m : slot.meshes) destroy_mesh(*r.engine, *r.scene, m);
    slot.meshes.clear();
}

// Rebuilds `slot`'s geometry from `ribbon`'s current point data, chunked at
// polyline_chunks()'s uint16-index-buffer ceiling -- each chunk becomes its
// own Mesh, all bound to `ribbon.role`'s opaque template instance (a fresh
// rebuild always starts opaque; the staleness pass below re-applies a fade
// if the ribbon is already stale the same frame it changed).
//
// A true indexed mesh, deliberately not the "flatten via
// extrude_polyline_indices() into a sequentially-indexed triangle list"
// shortcut map_elements.cpp/objects.cpp use for their much smaller
// geometry: flattening a chunk of kMaxPointsPerMesh (32000) points would
// produce ~192000 flat vertices, and the sequential uint16_t vertex index
// that pattern assigns silently wraps at 65536 and never reaches a
// verts.size() that large -- an infinite loop. Instead this keeps
// extrude_polyline()'s 2*n distinct vertices and hands
// extrude_polyline_indices()'s index list (values 0..2n-1, always < 65536
// for n <= kMaxPointsPerMesh) straight to the IndexBuffer -- no
// flattening, no per-corner duplication.
// pts/n is whatever update_ribbons() decided the slot's geometry should be
// built from -- the raw PathRibbon points, or an ego-clipped
// forward-truncated subset. `role` picks the material and the per-role
// margin.
void build_slot_meshes(VisualRenderer& r, VisualRenderer::RibbonSlot& slot, PathRole role,
                       const Vec3* pts, uint32_t n) {
    destroy_slot_meshes(r, slot);
    slot.totalVertexCount = 0;
    filament::MaterialInstance* mat = r.ribbonMaterial[static_cast<uint8_t>(role)];
    // Half-width from the active (possibly mid-transition-blended) theme at
    // build time -- r.active_theme is kept live every render_frame() call
    // by apply_current_theme(), which runs before update_ribbons() (this
    // function's only caller), so this always reads whatever
    // lane_width_m/margin a live set_theme() transition has blended to by
    // this frame, not a frozen create_renderer()-time snapshot. Mirrored
    // onto the slot itself (RibbonSlot::halfWidthM, see its own comment)
    // so tests can read back what was actually used without a Filament
    // AABB query (add_mesh() gives every mesh the same hard-coded declared
    // culling box, unrelated to the strip's real extent).
    const float halfWidthM = build_effective_half_width(r.active_theme.ribbon, role);
    slot.halfWidthM = halfWidthM;
    slot.firstPointM = n > 0 ? pts[0] : Vec3{};
    for (auto [a, b] : detail::polyline_chunks(n)) {
        const uint32_t chunkN = b - a;
        std::vector<Vec3> strip = detail::extrude_polyline(
            pts + a, chunkN, halfWidthM, kRibbonZLiftByRoleM[static_cast<uint8_t>(role)]);
        if (strip.empty()) continue;
        std::vector<uint16_t> idx =
            detail::extrude_polyline_indices(static_cast<uint32_t>(strip.size() / 2));
        if (idx.empty()) continue;

        std::vector<Vertex> verts(strip.size());
        for (size_t i = 0; i < strip.size(); ++i) verts[i].position = to_f3(strip[i]);
        fill_tangent_frames(verts, std::vector<float3>(strip.size(), float3{0.0f, 0.0f, 1.0f}));

        slot.totalVertexCount += static_cast<uint32_t>(verts.size());
        Mesh mesh;
        add_mesh(r, mesh, std::move(verts), std::move(idx),
                 filament::RenderableManager::PrimitiveType::TRIANGLES, mat,
                 /*cast_shadows=*/false, /*receive_shadows=*/true);
        slot.meshes.push_back(std::move(mesh));
    }
}

// Rebinds every primitive in `slot.meshes` to `mat` for the GLOBAL/LOCAL
// staleness swap (BEHAVIOR never rebinds, see below); mirrors
// objects.cpp's remap_to_material(), specialized for a ribbon slot's
// (possibly several, post-chunking) single-primitive meshes.
void rebind_slot_material(VisualRenderer& r, VisualRenderer::RibbonSlot& slot,
                         filament::MaterialInstance* mat) {
    filament::RenderableManager& rm = r.engine->getRenderableManager();
    for (auto& mesh : slot.meshes) {
        const auto ri = rm.getInstance(mesh.entity);
        if (ri.isValid()) rm.setMaterialInstanceAt(ri, 0, mat);
    }
}

}  // namespace

void update_ribbons(VisualRenderer& r, const SceneGraph& s) {
    // Release slots >= path_count.
    while (r.ribbonSlots.size() > s.path_count) {
        VisualRenderer::RibbonSlot& slot = r.ribbonSlots.back();
        destroy_slot_meshes(r, slot);
        if (slot.fadeInstance != nullptr) r.engine->destroy(slot.fadeInstance);
        r.ribbonSlots.pop_back();
    }
    if (r.ribbonSlots.size() < s.path_count) r.ribbonSlots.resize(s.path_count);

    for (uint32_t i = 0; i < s.path_count; ++i) {
        const PathRibbon& ribbon = s.paths[i];
        VisualRenderer::RibbonSlot& slot = r.ribbonSlots[i];

        // Ego-proximity clip, recomputed every frame from the ribbon's own
        // points and the current ego -- cheap (one segment walk), and this
        // per-frame recomputation is exactly what makes "parked ego -> zero
        // rebuilds" true: a stationary ego against unchanged ribbon points
        // always lands on the same quantized station, so the signature
        // below never changes even though this runs every call.
        const RibbonClip clip = compute_ribbon_clip(ribbon, s.ego);

        const float effectiveHalfWidthM = build_effective_half_width(r.active_theme.ribbon, ribbon.role);
        const uint64_t sig =
            ribbon_signature(ribbon.role, ribbon.points, ribbon.point_count,
                             effectiveHalfWidthM, clip.active, clip.quantizedUnits);
        if (!slot.has_signature || slot.signature != sig) {
            // Clipped geometry is materialized only on a rebuild -- the
            // clip station is already in the signature, so the rebuild
            // branch is the only consumer. Computing the clipped copy every
            // frame would allocate and discard the whole point list of a
            // long ridden GLOBAL route ~30x/s for nothing.
            std::vector<Vec3> clippedStorage;
            const Vec3* geomPts = ribbon.points;
            uint32_t geomN = ribbon.point_count;
            if (clip.active) {
                clippedStorage = clip_ribbon_forward(ribbon.points, ribbon.point_count, clip.stationM);
                geomPts = clippedStorage.data();
                geomN = static_cast<uint32_t>(clippedStorage.size());
            }
            // Content, role, or clip station changed -- full rebuild. Drop
            // any live fade instance (stale bookkeeping for the old
            // geometry) and let the staleness pass below re-decide from
            // scratch.
            if (slot.fadeInstance != nullptr) {
                r.engine->destroy(slot.fadeInstance);
                slot.fadeInstance = nullptr;
                slot.fadeAlpha = 1.0f;
            }
            ++r.ribbonRebuildCount;
            build_slot_meshes(r, slot, ribbon.role, geomPts, geomN);
            slot.role = ribbon.role;
            slot.signature = sig;
            slot.has_signature = true;
        }

        const auto alpha = static_cast<float>(detail::SceneBuffer::staleness_alpha(
            s.sim_time_sec, ribbon.last_update_sec, kStaleFadeStartSec, kStaleFadeTimeoutSec));
        const auto roleIdx = static_cast<uint8_t>(slot.role);

        if (slot.role == PathRole::BEHAVIOR) {
            // The hero ribbon: alpha is the staleness knob on its own
            // material (ribbon_emissive.mat's float4 baseColor + blending
            // fade) -- never an instance swap. Set every frame regardless
            // of value (cheap; at most one BEHAVIOR row ships today).
            // ponytail: a second live BEHAVIOR ribbon would share this one
            // instance and the last slot processed each frame would win —
            // not a shipped scenario; revisit if a second BEHAVIOR row
            // ever ships.
            const detail::Float3& tint = r.ribbonTint[roleIdx];
            r.ribbonMaterial[roleIdx]->setParameter("baseColor",
                                                    float4{tint.r, tint.g, tint.b, alpha});
            slot.fadeAlpha = alpha;
        } else {
            // GLOBAL/LOCAL: the same clay_translucent.mat swap mechanism as
            // objects.cpp's update_entity_staleness() -- never
            // MaterialInstance::duplicate() of the opaque template.
            if (alpha >= 1.0f) {
                if (slot.fadeInstance != nullptr) {
                    rebind_slot_material(r, slot, r.ribbonMaterial[roleIdx]);
                    r.engine->destroy(slot.fadeInstance);
                    slot.fadeInstance = nullptr;
                    slot.fadeAlpha = 1.0f;
                }
            } else {
                if (slot.fadeInstance == nullptr) {
                    slot.fadeInstance = r.clayTranslucentMaterial->createInstance();
                    slot.fadeInstance->setCullingMode(filament::backend::CullingMode::NONE);
                    rebind_slot_material(r, slot, slot.fadeInstance);
                }
                const detail::Float3& tint = r.ribbonTint[roleIdx];
                slot.fadeInstance->setParameter("baseColor", float4{tint.r, tint.g, tint.b, alpha});
                slot.fadeInstance->setParameter("roughness", r.active_theme.material.roughness);
                slot.fadeInstance->setParameter("metallic", r.active_theme.material.metallic);
                slot.fadeAlpha = alpha;
            }
        }
    }
}

}  // namespace mpviz

// Filament-free test introspection hooks; see ribbon_test_hooks.hpp for
// why these live here.
namespace mpviz::testing {

mpviz::detail::Float3 ribbon_role_base_color(mpviz::VisualRenderer* r, mpviz::PathRole role) {
    if (r == nullptr) return {};
    return r->ribbonTint[static_cast<uint8_t>(role)];
}

size_t ribbon_mesh_count(mpviz::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->ribbonSlots.size()) return 0;
    return r->ribbonSlots[slot].meshes.size();
}

size_t ribbon_vertex_count(mpviz::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->ribbonSlots.size()) return 0;
    return r->ribbonSlots[slot].totalVertexCount;
}

RibbonMaterialInfo ribbon_slot_material_info(mpviz::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->ribbonSlots.size()) return {};
    const mpviz::VisualRenderer::RibbonSlot& s = r->ribbonSlots[slot];
    RibbonMaterialInfo info;
    info.alpha = s.fadeAlpha;
    if (s.meshes.empty()) return info;
    filament::RenderableManager& rm = r->engine->getRenderableManager();
    const auto ri = rm.getInstance(s.meshes.front().entity);
    if (!ri.isValid()) return info;
    filament::MaterialInstance* bound = rm.getMaterialInstanceAt(ri, 0);
    info.bound_to_translucent = bound != nullptr && bound->getMaterial() == r->clayTranslucentMaterial;
    return info;
}

float ribbon_slot_half_width_m(mpviz::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->ribbonSlots.size()) return 0.0f;
    return r->ribbonSlots[slot].halfWidthM;
}

bool ribbon_slot_first_point(mpviz::VisualRenderer* r, size_t slot, mpviz::Vec3* out) {
    if (r == nullptr || slot >= r->ribbonSlots.size() || out == nullptr) return false;
    const auto& s = r->ribbonSlots[slot];
    if (s.meshes.empty()) return false;
    *out = s.firstPointM;
    return true;
}

uint64_t ribbon_rebuild_count(mpviz::VisualRenderer* r) {
    if (r == nullptr) return 0;
    return r->ribbonRebuildCount;
}

}  // namespace mpviz::testing
