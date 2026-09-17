// alert_polygons.cpp — translucent collision alert polygons. Geometry
// reuses triangulate_convex_polygon (polyline.hpp) — no second
// triangulator. Every alert polygon lives on clay_translucent.mat from the
// instant it exists, at its severity's constant alpha
// (renderer_internal.hpp's kAlertSeverityAlpha); staleness_alpha() only
// multiplies that constant down further while stale, via the same
// per-entity MaterialInstance-swap objects.cpp/ribbon.cpp use.
//
// Z-order (alerts overlay everything): ground plane 0, ground-grid shading
// 0.010/0.015 (ground_grid.cpp), HD-map lane paint/crosswalks 0.02
// (map_elements.cpp), object predicted-path ribbons 0.03 (objects.cpp),
// path ribbons 0.038-0.058 incl. the velocity ribbon at 0.052
// (ribbon.cpp / trajectory_carpet.cpp), alert polygons here at 0.06 —
// topmost.
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

// Content signature for one alert slot: severity + point count + first/last
// point (same shape as ribbon.cpp's ribbon_signature()). Severity is
// included so a slot re-homed to a different severity still rebuilds and
// re-binds to the right template.
uint64_t alert_signature(uint8_t severity, const Vec3* pts, uint32_t n) {
    uint64_t h = hash_combine(0, static_cast<uint64_t>(severity));
    h = hash_combine(h, static_cast<uint64_t>(n));
    if (n > 0) {
        h = hash_combine(h, hash_vec3(pts[0]));
        h = hash_combine(h, hash_vec3(pts[n - 1]));
    }
    return h;
}

// z-lift for the fan-triangulated alert polygon; see file header for the
// full z-stack. 0.06 sits above every other overlay (ribbons top out at
// 0.05) so an alert never z-fights with what it's warning about.
constexpr float kAlertZLiftM = 0.06f;

// Rebuilds `slot`'s mesh from `poly`'s current points, bound to severity
// `severity`'s opaque template (a fresh rebuild always starts there; the
// staleness pass below re-applies a per-slot fade if already stale this
// frame). Destroys any previous mesh first.
//
// Flat sequential uint16_t indexing (not ribbon.cpp's true-indexed
// pattern): alert polygons are small, so a flat list plus a guard is
// simplest here. The size check runs on a wide counter before any
// uint16_t is assigned, so an oversized polygon fails loudly instead of
// silently wrapping.
void build_slot_mesh(VisualRenderer& r, VisualRenderer::AlertSlot& slot, const AlertPolygon& poly,
                     uint8_t severity) {
    if (slot.mesh.vb != nullptr) destroy_mesh(*r.engine, *r.scene, slot.mesh);

    // The adapter stores rings closed (first == last), so the fan emits one
    // zero-area trailing triangle per polygon -- accepted (a few wasted
    // verts on an already-tiny mesh) rather than special-casing closed-ring
    // detection here. Not a bug.
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

// Rebinds slot `slot`'s one renderable primitive to `mat` for the
// fresh<->stale swap (single-mesh specialization of objects.cpp's
// remap_to_material() / ribbon.cpp's rebind_slot_material()).
void rebind_slot_material(VisualRenderer& r, VisualRenderer::AlertSlot& slot,
                         filament::MaterialInstance* mat) {
    if (!slot.mesh.entity) return;
    filament::RenderableManager& rm = r.engine->getRenderableManager();
    const auto ri = rm.getInstance(slot.mesh.entity);
    if (ri.isValid()) rm.setMaterialInstanceAt(ri, 0, mat);
}

}  // namespace

void update_alert_polygons(VisualRenderer& r, const SceneGraph& s) {
    // Release slots >= alert_count.
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

        // ponytail: severity is a raw uint8_t (no frozen enum here); clamp
        // defensively to critical (fail open to most visible) instead of
        // indexing out of bounds — node-side validation upstream is the
        // real guarantee this never fires.
        const uint8_t severity =
            poly.severity < VisualRenderer::kAlertSeverityCount
                ? poly.severity
                : static_cast<uint8_t>(VisualRenderer::kAlertSeverityCount - 1);

        const uint64_t sig = alert_signature(severity, poly.points, poly.point_count);
        if (!slot.has_signature || slot.signature != sig) {
            // Content or severity changed -- full rebuild. Drop any live
            // fade instance (stale bookkeeping for the old geometry) and
            // let the staleness pass below re-decide from scratch.
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
            // Per-entity MaterialInstance swap (same mechanism as
            // objects.cpp/ribbon.cpp) -- never MaterialInstance::duplicate()
            // of the template.
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

// Filament-free test introspection hooks; see alert_polygons_test_hooks.hpp
// for why these live here.
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
