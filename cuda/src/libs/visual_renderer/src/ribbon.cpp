// ribbon.cpp — Epic 2 Task 5 (VM-023): path ribbons in three roles
// (BEHAVIOR/GLOBAL/LOCAL), the behavior ribbon as the emissive bloom hero.
// Geometry reuses Task 2's extrude_polyline/extrude_polyline_indices/
// polyline_chunks (polyline.hpp) — no second extruder.
#include "ribbon.hpp"
#include "ribbon_test_hooks.hpp"
#include "polyline.hpp"
#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

#include <filament/RenderableManager.h>

#include <math/vec3.h>
#include <math/vec4.h>

#include <cstdint>
#include <functional>
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

// Content signature for one ribbon SLOT (not a chunk-within-the-ribbon --
// see renderer_internal.hpp's RibbonSlot comment): role + point count +
// first/last point. Role is included on purpose -- a slot re-homed to a
// different role (Task 5 Step 4: "role changes re-home the material") must
// rebuild so its geometry gets re-bound to the new role's material, even if
// point data happened to be identical (never happens in practice, but the
// signature must not silently skip it).
uint64_t ribbon_signature(PathRole role, const Vec3* pts, uint32_t n) {
    uint64_t h = hash_combine(0, static_cast<uint64_t>(role));
    h = hash_combine(h, static_cast<uint64_t>(n));
    if (n > 0) {
        h = hash_combine(h, hash_vec3(pts[0]));
        h = hash_combine(h, hash_vec3(pts[n - 1]));
    }
    return h;
}

// Half-width/z-lift for the extruded ribbon strip -- a stopgap constant,
// same shape as map_elements.cpp's kLaneHalfWidthM: PathRibbon is frozen
// without a width field, so per-role/per-topic width crosses the boundary
// only at a future freeze lift. z-lift 0.04: above BOTH neighbours it can
// cross -- the 0.02m lane paint AND objects.cpp's 0.03m predicted-path
// ribbons (review 2026-08-20: at 0.03 a behavior/local ribbon crossing a
// tracked object's predicted path was coplanar and z-fought it).
constexpr float kRibbonHalfWidthM = 0.12f;
constexpr float kRibbonZLiftM = 0.04f;

// Destroys every mesh in `slot.meshes` (independent of role/fade state --
// called both on a full content-signature rebuild and on slot release) and
// clears the vector.
void destroy_slot_meshes(VisualRenderer& r, VisualRenderer::RibbonSlot& slot) {
    for (auto& m : slot.meshes) destroy_mesh(*r.engine, *r.scene, m);
    slot.meshes.clear();
}

// Rebuilds `slot`'s geometry from `ribbon`'s current point data, chunked at
// polyline_chunks()'s uint16-index-buffer ceiling (Task 5 Step 4 test:
// LongPathSplitsAcrossMeshesWithoutTruncation) -- each chunk becomes its own
// Mesh, all bound to `ribbon.role`'s OPAQUE template instance (a fresh
// rebuild always starts opaque; the staleness pass below re-applies a fade
// if the ribbon is ALREADY stale the same frame it changed).
//
// A TRUE indexed mesh, deliberately NOT the "flatten via
// extrude_polyline_indices() into a sequentially-indexed triangle list"
// shortcut map_elements.cpp/objects.cpp use for their much smaller marker/
// predicted-path geometry: flattening a chunk of kMaxPointsPerMesh (32000)
// points would produce 6*(32000-1) ~= 192000 FLAT vertices, and the
// sequential `uint16_t` vertex index that pattern assigns
// (`for (uint16_t i = 0; i < verts.size(); ++i)`) silently WRAPS at 65536 and
// never reaches a `verts.size()` that large -- an infinite loop, caught by
// this task's own LongPathSplitsAcrossMeshesWithoutTruncation test (found
// live: the flattened shape hung this exact test at 100% CPU). The real
// fix, and the one polyline_chunks()'s own header comment already assumes
// ("2 verts/pt, < 65535/2"): keep extrude_polyline()'s 2*n DISTINCT vertices
// and hand extrude_polyline_indices()'s index list (values 0..2n-1, always
// < 65536 for n <= kMaxPointsPerMesh) straight to the IndexBuffer -- no
// flattening, no per-corner duplication, and the actual GPU vertex count
// this task's tests assert against.
void build_slot_meshes(VisualRenderer& r, VisualRenderer::RibbonSlot& slot,
                       const PathRibbon& ribbon) {
    destroy_slot_meshes(r, slot);
    slot.totalVertexCount = 0;
    filament::MaterialInstance* mat = r.ribbonMaterial[static_cast<uint8_t>(ribbon.role)];
    for (auto [a, b] : detail::polyline_chunks(ribbon.point_count)) {
        const uint32_t n = b - a;
        std::vector<Vec3> strip =
            detail::extrude_polyline(ribbon.points + a, n, kRibbonHalfWidthM, kRibbonZLiftM);
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

// Rebinds every primitive in `slot.meshes` to `mat` -- the GLOBAL/LOCAL
// staleness swap needs this (BEHAVIOR never rebinds, see below); mirrors
// objects.cpp's remap_to_material()/bind_everywhere(), specialized for a
// ribbon slot's (possibly several, post-chunking) single-primitive meshes.
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
    // Release slots >= path_count (teardown walks the whole vector, same
    // rule destroy_renderer()'s final pass follows).
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

        const uint64_t sig = ribbon_signature(ribbon.role, ribbon.points, ribbon.point_count);
        if (!slot.has_signature || slot.signature != sig) {
            // Content (or role) changed -- full rebuild. A live translucent
            // fade instance is stale bookkeeping for the OLD geometry; drop
            // it and let the staleness pass below re-decide from scratch
            // against the ribbon's (now current) last_update_sec.
            if (slot.fadeInstance != nullptr) {
                r.engine->destroy(slot.fadeInstance);
                slot.fadeInstance = nullptr;
                slot.fadeAlpha = 1.0f;
            }
            build_slot_meshes(r, slot, ribbon);
            slot.role = ribbon.role;
            slot.signature = sig;
            slot.has_signature = true;
        }

        const auto alpha = static_cast<float>(detail::SceneBuffer::staleness_alpha(
            s.sim_time_sec, ribbon.last_update_sec, kStaleFadeStartSec, kStaleFadeTimeoutSec));
        const auto roleIdx = static_cast<uint8_t>(slot.role);

        if (slot.role == PathRole::BEHAVIOR) {
            // THE hero ribbon: alpha IS the staleness knob on its OWN
            // material (ribbon_emissive.mat's float4 baseColor + blending:
            // fade) -- never an instance swap. Set every frame regardless
            // of value (cheap; at most one BEHAVIOR row ships today).
            // ponytail: a SECOND live BEHAVIOR ribbon would share this one
            // instance and the last slot processed each frame would win --
            // not a shipped scenario (both profiles ship exactly one
            // behavior row) and not what Task 5's keying fix is for (that
            // fix is about GLOBAL/LOCAL's two rows); revisit if a second
            // BEHAVIOR row ever ships.
            const detail::Float3& tint = r.ribbonTint[roleIdx];
            r.ribbonMaterial[roleIdx]->setParameter("baseColor",
                                                    float4{tint.r, tint.g, tint.b, alpha});
            slot.fadeAlpha = alpha;
        } else {
            // GLOBAL/LOCAL: the exact clay_translucent.mat swap mechanism
            // as objects.cpp's update_entity_staleness() -- NEVER
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

// Epic 2 Task 5 (VM-023): Filament-free test introspection hooks (see
// ribbon_test_hooks.hpp's own comment for why these live here, mirroring
// map_elements.cpp/objects.cpp's own hook definitions).
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

}  // namespace mpviz::testing
