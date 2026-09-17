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
// first/last point + half-width. Role is included so a slot re-homed to a
// different role still rebuilds and re-binds to the new role's material,
// even if point data happened to be identical. half_width_m is included
// because a set_theme() that changes only a margin touches no PathRibbon
// point data at all — without the effective width in the signature, a
// mid-transition margin change would be silently ignored until some
// unrelated content change forced a rebuild. Hashed as its bit pattern (not
// the float value): exact reproducibility across calls with the same width
// matters here, not numeric comparison.
//
// Clip state deliberately excluded; applied per-frame as a position
// collapse, see apply_ribbon_clip().
uint64_t ribbon_signature(PathRole role, const Vec3* pts, uint32_t n, float half_width_m) {
    uint64_t h = hash_combine(0, static_cast<uint64_t>(role));
    h = hash_combine(h, static_cast<uint64_t>(n));
    if (n > 0) {
        h = hash_combine(h, hash_vec3(pts[0]));
        h = hash_combine(h, hash_vec3(pts[n - 1]));
    }
    uint32_t widthBits;
    std::memcpy(&widthBits, &half_width_m, sizeof(widthBits));
    h = hash_combine(h, static_cast<uint64_t>(widthBits));
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
//
// trajectory_carpet.cpp's velocity ribbon (VM-077 carpet-as-ribbon redirect,
// 2026-09-10) is NOT a PathRole and so isn't in this array, but it stacks
// INTO this same z-order: its own kVelocityRibbonZLiftM sits at 0.052,
// strictly between LOCAL (0.046) and BEHAVIOR (0.058) -- "on top of local
// ribbon" (user directive) while the hero ribbon stays topmost of the whole
// stack. Full order, lowest to highest (Finding #14: figures corrected to
// match the live constants below/trajectory_carpet.cpp:85 -- the ordering
// was always right, only these numbers had drifted): GLOBAL 0.038 < LOCAL
// 0.046 < velocity 0.052 < BEHAVIOR 0.058 < alert_polygons.cpp's
// kAlertZLiftM 0.06. The 2026-09 map-element stagger adds a lower band
// underneath all of this, disjoint from it: ROAD_SURFACE 0.005, then
// OTHER..CROSSWALK across 0.016-0.023 (map_elements.cpp's own comment) --
// both bands share the one "lowest to highest, no collisions" ground stack,
// this array's own roles just start higher up it.
// Stagger widened 2026-09-10 (user: "Still flickering, increase the z a
// bit and let me judge") -- gaps 6-8mm, was 2.5-5mm; whole stack stays
// under alert_polygons' 0.06 so alerts remain topmost.
constexpr float kRibbonZLiftByRoleM[3] = {0.058f,  // BEHAVIOR (PathRole 0)
                                          0.038f,  // GLOBAL   (PathRole 1)
                                          0.046f}; // LOCAL    (PathRole 2)

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
// the ego. The arc-length walk/quantization machinery
// (closest_arc_station/compute_polyline_clip) is shared with
// trajectory_carpet.cpp's velocity ribbon via polyline.hpp -- promoted out
// of this file rather than duplicated, since it's the exact same "never
// render behind the ego" mechanism both need. RibbonClip stays a thin
// PathRibbon/EgoState-flavored wrapper here so update_ribbons() below
// (and its call site's ergonomics) is unchanged.
using RibbonClip = detail::PolylineClip;

// Clip applies only when ego.valid -- an invalid ego has no real position
// to clip against (EgoState::valid's "0 = no TF yet" contract, scene.h).
RibbonClip compute_ribbon_clip(const PathRibbon& ribbon, const EgoState& ego) {
    if (!ego.valid) return RibbonClip{};
    return detail::compute_polyline_clip(ribbon.points, ribbon.point_count, ego.position);
}

// Destroys every mesh in `slot.meshes` (independent of role/fade state --
// called both on a full content-signature rebuild and on slot release) and
// clears the vector.
void destroy_slot_meshes(VisualRenderer& r, VisualRenderer::RibbonSlot& slot) {
    for (auto& m : slot.meshes) destroy_mesh(*r.engine, *r.scene, m);
    slot.meshes.clear();
    slot.baseStripPositions.clear();
    slot.pointStations.clear();
    slot.has_applied_clip = false;
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
// pts/n is always the FULL, unclipped PathRibbon points -- the ego-clip is
// applied afterward, every frame, as a position collapse (apply_ribbon_clip
// below). `role` picks the material and the per-role margin.
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
    slot.firstPointM = n > 0 ? pts[0] : Vec3{};  // baseline; apply_ribbon_clip() overwrites if clipped
    // Global arc-length offset carried across chunks so `pointStations`
    // stays index-aligned with compute_polyline_clip()'s own station
    // measure even for a >kMaxPointsPerMesh ribbon (each chunk's own
    // cleaning starts fresh -- clean_polyline_stations() mirrors
    // extrude_polyline()'s internal per-chunk clean exactly, see its own
    // comment -- so this offset is what stitches those chunk-local walks
    // back into one global measure). A no-op (stays 0.0) for the
    // overwhelmingly common single-chunk case.
    double chunkStationOffset = 0.0;
    for (auto [a, b] : detail::polyline_chunks(n)) {
        const uint32_t chunkN = b - a;
        std::vector<Vec3> strip = detail::extrude_polyline(
            pts + a, chunkN, halfWidthM, kRibbonZLiftByRoleM[static_cast<uint8_t>(role)]);
        if (strip.empty()) continue;
        std::vector<uint16_t> idx =
            detail::extrude_polyline_indices(static_cast<uint32_t>(strip.size() / 2));
        if (idx.empty()) continue;

        std::vector<double> stations = detail::clean_polyline_stations(pts + a, chunkN);
        for (double& s : stations) s += chunkStationOffset;
        if (!stations.empty()) chunkStationOffset = stations.back();

        std::vector<Vertex> verts(strip.size());
        for (size_t i = 0; i < strip.size(); ++i) verts[i].position = to_f3(strip[i]);
        fill_tangent_frames(verts, std::vector<float3>(strip.size(), float3{0.0f, 0.0f, 1.0f}));

        slot.totalVertexCount += static_cast<uint32_t>(verts.size());
        Mesh mesh;
        add_mesh(r, mesh, std::move(verts), std::move(idx),
                 filament::RenderableManager::PrimitiveType::TRIANGLES, mat,
                 /*cast_shadows=*/false, /*receive_shadows=*/true);
        slot.meshes.push_back(std::move(mesh));
        slot.baseStripPositions.push_back(std::move(strip));
        slot.pointStations.push_back(std::move(stations));
    }
}

// Applies (or re-applies) the ego-proximity clip to every mesh in `slot` by
// collapsing degenerate vertices onto the interpolated cut and re-uploading
// via update_mesh_positions() -- see polyline.hpp's
// collapse_clipped_positions() for why this replaces the old
// destroy-then-rebuild clip path. Called every render_frame() (through
// update_ribbons() below); the GPU upload itself only happens when the
// clip state actually moved since the last call, or right after a fresh
// (always-unclipped) content rebuild, which forces one. A parked ego
// causes zero uploads. Also keeps RibbonSlot::firstPointM (the test-hook
// mirror) in sync with what's actually on screen.
void apply_ribbon_clip(VisualRenderer& r, VisualRenderer::RibbonSlot& slot, const RibbonClip& clip) {
    const bool changed = !slot.has_applied_clip || slot.appliedClipActive != clip.active ||
                          (clip.active && slot.appliedClipUnits != clip.quantized_units);
    if (!changed) return;
    for (size_t i = 0; i < slot.meshes.size(); ++i) {
        std::vector<Vec3> positions = slot.baseStripPositions[i];
        detail::collapse_clipped_positions(positions, slot.pointStations[i], clip.active,
                                            clip.station_m);
        if (i == 0 && !positions.empty()) slot.firstPointM = positions[0];
        std::vector<Vertex> verts(positions.size());
        for (size_t v = 0; v < positions.size(); ++v) verts[v].position = to_f3(positions[v]);
        fill_tangent_frames(verts, std::vector<float3>(positions.size(), float3{0.0f, 0.0f, 1.0f}));
        update_mesh_positions(*r.engine, slot.meshes[i], std::move(verts));
    }
    slot.has_applied_clip = true;
    slot.appliedClipActive = clip.active;
    slot.appliedClipUnits = clip.quantized_units;
}

// Rebinds every primitive in `slot.meshes` to `mat` for the staleness swap
// (all three roles since the 2026-09-10 opaque fix, see below); mirrors
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
        // points and the current ego -- cheap (one segment walk). No
        // longer feeds the content signature (see ribbon_signature()'s
        // comment); applied below via apply_ribbon_clip() as a position
        // collapse on whatever geometry the signature check just decided
        // (rebuilt or not).
        const RibbonClip clip = compute_ribbon_clip(ribbon, s.ego);

        const float effectiveHalfWidthM = build_effective_half_width(r.active_theme.ribbon, ribbon.role);
        const uint64_t sig =
            ribbon_signature(ribbon.role, ribbon.points, ribbon.point_count, effectiveHalfWidthM);
        if (!slot.has_signature || slot.signature != sig) {
            // Content, role, or width changed -- full rebuild (always the
            // FULL unclipped ribbon; apply_ribbon_clip() below handles the
            // clip). Drop any live fade instance (stale bookkeeping for the
            // old geometry) and let the staleness pass below re-decide
            // from scratch.
            if (slot.fadeInstance != nullptr) {
                r.engine->destroy(slot.fadeInstance);
                slot.fadeInstance = nullptr;
                slot.fadeAlpha = 1.0f;
            }
            ++r.ribbonRebuildCount;
            build_slot_meshes(r, slot, ribbon.role, ribbon.points, ribbon.point_count);
            slot.role = ribbon.role;
            slot.signature = sig;
            slot.has_signature = true;
        }

        apply_ribbon_clip(r, slot, clip);

        const auto alpha = static_cast<float>(detail::SceneBuffer::staleness_alpha(
            s.sim_time_sec, ribbon.last_update_sec, kStaleFadeStartSec, kStaleFadeTimeoutSec));
        const auto roleIdx = static_cast<uint8_t>(slot.role);

        // ALL roles (BEHAVIOR included, since the 2026-09-10 flicker
        // root-cause fix made ribbon_emissive.mat opaque -- see that file):
        // fresh renders on the role's own OPAQUE instance, staleness fades
        // via the clay_translucent.mat swap -- the same mechanism as
        // objects.cpp's update_entity_staleness(), never
        // MaterialInstance::duplicate() of the opaque template. BEHAVIOR's
        // emissive glow is not carried through the 1s death-fade
        // (clay_translucent has no emissive param) -- accepted; both
        // shipped themes author emissive.ribbon_strength 0.0 today.
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
