// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "alert_polygons.hpp"
#include "alert_polygons_test_hooks.hpp"
#include "polyline.hpp"
#include "renderer_internal.hpp"
#include "overlume/scene.h"

#include <filament/RenderableManager.h>

#include <math/vec3.h>
#include <math/vec4.h>

#include <cstdint>
#include <cstdio>
#include <functional>
#include <vector>

namespace overlume {

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

uint64_t alert_signature(uint8_t severity, const Vec3* pts, uint32_t n) {
    uint64_t h = hash_combine(0, static_cast<uint64_t>(severity));
    h = hash_combine(h, static_cast<uint64_t>(n));
    if (n > 0) {
        h = hash_combine(h, hash_vec3(pts[0]));
        h = hash_combine(h, hash_vec3(pts[n - 1]));
    }
    return h;
}

constexpr float kAlertZLiftM = 0.06f;

void build_slot_mesh(VisualRenderer& r, VisualRenderer::AlertSlot& slot, const AlertPolygon& poly,
                     uint8_t severity) {
    if (slot.mesh.vb != nullptr) destroy_mesh(*r.engine, *r.scene, slot.mesh);

    std::vector<Vec3> tris =
        detail::triangulate_convex_polygon(poly.points, poly.point_count, kAlertZLiftM);
    if (tris.empty()) return;

    if (tris.size() > 65535) {
        std::fprintf(stderr,
                     "[overlume] alert polygon mesh (%zu verts) exceeds the uint16 index "
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
             false, true);
}

void rebind_slot_material(VisualRenderer& r, VisualRenderer::AlertSlot& slot,
                          filament::MaterialInstance* mat) {
    if (!slot.mesh.entity) return;
    filament::RenderableManager& rm = r.engine->getRenderableManager();
    const auto ri = rm.getInstance(slot.mesh.entity);
    if (ri.isValid()) rm.setMaterialInstanceAt(ri, 0, mat);
}

}  // namespace

void update_alert_polygons(VisualRenderer& r, const SceneGraph& s) {
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

        const uint8_t severity =
            poly.severity < VisualRenderer::kAlertSeverityCount
                ? poly.severity
                : static_cast<uint8_t>(VisualRenderer::kAlertSeverityCount - 1);

        const uint64_t sig = alert_signature(severity, poly.points, poly.point_count);
        if (!slot.has_signature || slot.signature != sig) {
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

}  // namespace overlume

namespace overlume::testing {

overlume::detail::Float3 alert_severity_base_color(overlume::VisualRenderer* r, uint8_t severity) {
    if (r == nullptr || severity >= overlume::VisualRenderer::kAlertSeverityCount) return {};
    return r->alertTint[severity];
}

size_t alert_slot_count(overlume::VisualRenderer* r) {
    return r == nullptr ? 0 : r->alertSlots.size();
}

AlertMaterialInfo alert_slot_material_info(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->alertSlots.size()) return {};
    const overlume::VisualRenderer::AlertSlot& s = r->alertSlots[slot];
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

}  // namespace overlume::testing
