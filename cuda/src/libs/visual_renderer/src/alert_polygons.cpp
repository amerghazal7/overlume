// alert_polygons.cpp — Epic 2 Task 7 (VM-026): translucent collision alert
// polygons. Geometry reuses Task 2's triangulate_convex_polygon (polyline.
// hpp) — no second triangulator. Every alert polygon lives on
// clay_translucent.mat (Task 4) from the instant it exists — its
// severity's CONSTANT alpha (renderer_internal.hpp's kAlertSeverityAlpha),
// never a fade — and only staleness_alpha() MULTIPLIES that constant down
// further while stale, via the exact per-entity MaterialInstance-swap
// mechanism objects.cpp/ribbon.cpp already established (see the plan's
// "…and the material that can actually do it").
//
// FIXTURE GAP 4 (epic2 plan): the five collision-checker topics were
// silent in the recorded bag (a calm scenario, zero messages) — every
// fixture and this task's own golden are synthetic, unvalidated against a
// live publisher.
//
// Z-ORDER (spec §7's "alerts overlay everything" — documented once, here,
// since this is the topmost layer of the whole epic's z-stack): ground
// plane 0, OGM ground-grid shading 0.010 (gradient)/0.015 (dynamic,
// ground_grid.cpp), HD-map lane paint/crosswalks 0.02 (map_elements.cpp),
// object predicted-path ribbons 0.03 (objects.cpp), path ribbons 0.04
// (GLOBAL)-0.05 (BEHAVIOR, ribbon.cpp) — and alert polygons here, at 0.06,
// above all of them.
#include "alert_polygons.hpp"
#include "alert_polygons_test_hooks.hpp"
#include "polyline.hpp"
#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

#include <filament/RenderableManager.h>

#include <math/vec3.h>
#include <math/vec4.h>

#include <cstdint>
#include <cstdio>
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

// Content signature for one alert SLOT (ribbon.cpp's ribbon_signature()
// shape): severity + point count + first/last point. Severity is included
// on purpose, same reasoning as ribbon_signature()'s role — a slot
// re-homed to a different severity (the underlying topic's role changed
// via a profile-YAML edit, no rebuild there either) must rebuild so its
// geometry re-binds to the right severity's template, even on the (never
// observed in practice) chance the point data happened to be identical.
uint64_t alert_signature(uint8_t severity, const Vec3* pts, uint32_t n) {
    uint64_t h = hash_combine(0, static_cast<uint64_t>(severity));
    h = hash_combine(h, static_cast<uint64_t>(n));
    if (n > 0) {
        h = hash_combine(h, hash_vec3(pts[0]));
        h = hash_combine(h, hash_vec3(pts[n - 1]));
    }
    return h;
}

// z-lift for the fan-triangulated alert polygon — see this file's top
// header comment for the full z-stack. 0.06: above every other Epic 2
// overlay (ribbons top out at 0.05), so an alert polygon never z-fights
// with or hides under the geometry it's warning about.
constexpr float kAlertZLiftM = 0.06f;

// Rebuilds `slot`'s single mesh from `poly`'s current point data, bound to
// severity `severity`'s OPAQUE-ALPHA template instance (a fresh rebuild
// always starts on the shared template; the staleness pass in
// update_alert_polygons() below re-applies a per-slot fade if the polygon
// is ALREADY stale the same frame it changed — ribbon.cpp's
// build_slot_meshes() precedent). Destroys any previous mesh first
// (content OR severity changed).
//
// Flat sequential uint16_t indexing, NOT ribbon.cpp's true-indexed
// pattern: alert polygons are small (a handful of boundary points, never a
// 32000-point path), so map_elements.cpp's flat-list-plus-guard shape is
// the simplest correct thing here, not the second-extruder-scale problem
// ribbon.cpp's own header explains. The guard itself is copied verbatim
// (same wide `size_t`/`std::vector<Vertex>::size()` counter, checked BEFORE
// any uint16_t is ever assigned) so a producer that somehow published a
// huge polygon fails loudly instead of silently wrapping.
void build_slot_mesh(VisualRenderer& r, VisualRenderer::AlertSlot& slot, const AlertPolygon& poly,
                     uint8_t severity) {
    if (slot.mesh.vb != nullptr) destroy_mesh(*r.engine, *r.scene, slot.mesh);

    // The adapter stores rings CLOSED (first == last), so the fan emits one
    // zero-area trailing triangle per polygon -- accepted deliberately
    // (3 wasted verts on an already-tiny mesh) rather than special-casing
    // the closed-ring detection here (review 2026-08-20). Not a bug.
    std::vector<Vec3> tris =
        detail::triangulate_convex_polygon(poly.points, poly.point_count, kAlertZLiftM);
    if (tris.empty()) return;  // malformed/degenerate -- nothing to render this frame

    if (tris.size() > 65535) {
        std::fprintf(stderr,
                     "[visual_renderer] alert polygon mesh (%zu verts) exceeds the uint16 index "
                     "ceiling; alert dropped\n",
                     tris.size());
        return;
    }

    std::vector<Vertex> verts(tris.size());
    for (size_t i = 0; i < tris.size(); ++i) verts[i].position = to_f3(tris[i]);
    fill_tangent_frames(verts, std::vector<float3>(tris.size(), float3{0.0f, 0.0f, 1.0f}));
    std::vector<uint16_t> indices(verts.size());
    for (size_t i = 0; i < verts.size(); ++i) indices[i] = static_cast<uint16_t>(i);

    add_mesh(r, slot.mesh, std::move(verts), std::move(indices),
             filament::RenderableManager::PrimitiveType::TRIANGLES, r.alertMaterial[severity],
             /*cast_shadows=*/false, /*receive_shadows=*/true);
}

// Rebinds slot `slot`'s one renderable primitive to `mat` — the fresh<->
// stale swap needs this, mirroring objects.cpp's remap_to_material()/
// ribbon.cpp's rebind_slot_material(), specialized for a single-mesh slot.
void rebind_slot_material(VisualRenderer& r, VisualRenderer::AlertSlot& slot,
                         filament::MaterialInstance* mat) {
    if (!slot.mesh.entity) return;
    filament::RenderableManager& rm = r.engine->getRenderableManager();
    const auto ri = rm.getInstance(slot.mesh.entity);
    if (ri.isValid()) rm.setMaterialInstanceAt(ri, 0, mat);
}

}  // namespace

void update_alert_polygons(VisualRenderer& r, const SceneGraph& s) {
    // Release slots >= alert_count (teardown walks the whole vector, same
    // rule destroy_renderer()'s final pass follows).
    while (r.alertSlots.size() > s.alert_count) {
        VisualRenderer::AlertSlot& slot = r.alertSlots.back();
        if (slot.mesh.vb != nullptr) destroy_mesh(*r.engine, *r.scene, slot.mesh);
        if (slot.fadeInstance != nullptr) r.engine->destroy(slot.fadeInstance);
        r.alertSlots.pop_back();
    }
    if (r.alertSlots.size() < s.alert_count) r.alertSlots.resize(s.alert_count);

    for (uint32_t i = 0; i < s.alert_count; ++i) {
        const AlertPolygon& poly = s.alerts[i];
        VisualRenderer::AlertSlot& slot = r.alertSlots[i];

        // AlertPolygon::severity is a raw uint8_t, not the frozen 3-value
        // enum every OTHER category gets (scene.h has none for this one) —
        // ponytail: clamp defensively to the last (critical, the
        // fail-open-to-most-visible choice) rather than index
        // alertMaterial/alertTint out of bounds; node-side collision.cpp's
        // role table (Task 1's profile validator upstream of it) is the
        // real guarantee this never actually fires today.
        const uint8_t severity =
            poly.severity < VisualRenderer::kAlertSeverityCount
                ? poly.severity
                : static_cast<uint8_t>(VisualRenderer::kAlertSeverityCount - 1);

        const uint64_t sig = alert_signature(severity, poly.points, poly.point_count);
        if (!slot.has_signature || slot.signature != sig) {
            // Content (or severity) changed -- full rebuild. A live
            // translucent fade instance is stale bookkeeping for the OLD
            // geometry; drop it and let the staleness pass below re-decide
            // from scratch against the polygon's (now current)
            // last_update_sec — ribbon.cpp's identical rebuild-time reset.
            if (slot.fadeInstance != nullptr) {
                r.engine->destroy(slot.fadeInstance);
                slot.fadeInstance = nullptr;
            }
            build_slot_mesh(r, slot, poly, severity);
            slot.severity = severity;
            slot.signature = sig;
            slot.has_signature = true;
        }

        const auto staleness = static_cast<float>(detail::SceneBuffer::staleness_alpha(
            s.sim_time_sec, poly.last_update_sec, kStaleFadeStartSec, kStaleFadeTimeoutSec));
        const float alpha = kAlertSeverityAlpha[slot.severity] * staleness;

        if (staleness >= 1.0f) {
            if (slot.fadeInstance != nullptr) {
                rebind_slot_material(r, slot, r.alertMaterial[slot.severity]);
                r.engine->destroy(slot.fadeInstance);
                slot.fadeInstance = nullptr;
            }
            slot.fadeAlpha = kAlertSeverityAlpha[slot.severity];
        } else {
            // The exact clay_translucent.mat per-entity swap mechanism as
            // objects.cpp's update_entity_staleness()/ribbon.cpp's
            // GLOBAL/LOCAL branch -- NEVER MaterialInstance::duplicate()
            // of the (already translucent, here) template.
            if (slot.fadeInstance == nullptr) {
                slot.fadeInstance = r.clayTranslucentMaterial->createInstance();
                slot.fadeInstance->setCullingMode(filament::backend::CullingMode::NONE);
                rebind_slot_material(r, slot, slot.fadeInstance);
            }
            const detail::Float3& tint = r.alertTint[slot.severity];
            slot.fadeInstance->setParameter("baseColor", float4{tint.r, tint.g, tint.b, alpha});
            slot.fadeInstance->setParameter("roughness", r.active_theme.material.roughness);
            slot.fadeInstance->setParameter("metallic", r.active_theme.material.metallic);
            slot.fadeAlpha = alpha;
        }
    }
}

}  // namespace mpviz

// Epic 2 Task 7 (VM-026): Filament-free test introspection hooks (see
// alert_polygons_test_hooks.hpp's own comment for why these live here,
// mirroring ribbon.cpp/objects.cpp's own hook definitions).
namespace mpviz::testing {

mpviz::detail::Float3 alert_severity_base_color(mpviz::VisualRenderer* r, uint8_t severity) {
    if (r == nullptr || severity >= mpviz::VisualRenderer::kAlertSeverityCount) return {};
    return r->alertTint[severity];
}

size_t alert_slot_count(mpviz::VisualRenderer* r) {
    return r == nullptr ? 0 : r->alertSlots.size();
}

AlertMaterialInfo alert_slot_material_info(mpviz::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->alertSlots.size()) return {};
    const mpviz::VisualRenderer::AlertSlot& s = r->alertSlots[slot];
    AlertMaterialInfo info;
    info.alpha = s.fadeAlpha;
    if (!s.mesh.entity) return info;
    filament::RenderableManager& rm = r->engine->getRenderableManager();
    const auto ri = rm.getInstance(s.mesh.entity);
    if (!ri.isValid()) return info;
    filament::MaterialInstance* bound = rm.getMaterialInstanceAt(ri, 0);
    info.bound_to_severity_template = bound == r->alertMaterial[s.severity];
    return info;
}

}  // namespace mpviz::testing
