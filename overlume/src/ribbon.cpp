// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "ribbon.hpp"
#include "ribbon_test_hooks.hpp"
#include "polyline.hpp"
#include "renderer_internal.hpp"
#include "overlume/scene.h"

#include <filament/RenderableManager.h>

#include <math/vec3.h>
#include <math/vec4.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

namespace overlume::detail {

float ribbon_fade_alpha(double station_m, double origin_station_m, float fade_start_m,
                        float fade_end_m) {
    if (!ribbon_length_fade_enabled(fade_start_m, fade_end_m)) return 1.0f;
    const double d = station_m - origin_station_m;
    if (d <= fade_start_m) return 1.0f;
    if (d >= fade_end_m) return 0.0f;
    const double a = 1.0 - (d - fade_start_m) / (double(fade_end_m) - double(fade_start_m));
    return std::clamp(static_cast<float>(a), 0.0f, 1.0f);
}

}

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

uint64_t ribbon_signature(PathRole role, const Vec3* pts, uint32_t n, float half_width_m,
                          float fade_start_m, float fade_end_m) {
    uint64_t h = hash_combine(0, static_cast<uint64_t>(role));
    h = hash_combine(h, static_cast<uint64_t>(n));
    if (n > 0) {
        h = hash_combine(h, hash_vec3(pts[0]));
        h = hash_combine(h, hash_vec3(pts[n - 1]));
    }
    uint32_t widthBits;
    std::memcpy(&widthBits, &half_width_m, sizeof(widthBits));
    h = hash_combine(h, static_cast<uint64_t>(widthBits));
    uint32_t fadeBits;
    std::memcpy(&fadeBits, &fade_start_m, sizeof(fadeBits));
    h = hash_combine(h, static_cast<uint64_t>(fadeBits));
    std::memcpy(&fadeBits, &fade_end_m, sizeof(fadeBits));
    h = hash_combine(h, static_cast<uint64_t>(fadeBits));
    return h;
}

constexpr float kRibbonZLiftByRoleM[3] = {0.058f, 0.038f, 0.046f};
constexpr uint8_t kRibbonPriorityByRole[3] = {5, 2, 3};

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

using RibbonClip = detail::PolylineClip;

RibbonClip compute_ribbon_clip(const PathRibbon& ribbon, const EgoState& ego) {
    if (!ego.valid) return RibbonClip{};
    return detail::compute_polyline_clip(ribbon.points, ribbon.point_count, ego.position);
}

void destroy_slot_meshes(VisualRenderer& r, VisualRenderer::RibbonSlot& slot) {
    for (auto& m : slot.meshes) destroy_mesh(*r.engine, *r.scene, m);
    slot.meshes.clear();
    slot.baseStripPositions.clear();
    slot.pointStations.clear();
    slot.has_applied_clip = false;
}

std::vector<Vertex> build_ribbon_vertices(const std::vector<Vec3>& positions,
                                          const std::vector<double>& stations,
                                          std::optional<double> origin_station_m,
                                          const detail::Theme::Ribbon& cfg) {
    std::vector<Vertex> verts(positions.size());
    for (size_t i = 0; i < positions.size(); ++i) verts[i].position = to_f3(positions[i]);
    fill_tangent_frames(verts, std::vector<float3>(positions.size(), float3{0.0f, 0.0f, 1.0f}));
    const bool haveStations = positions.size() == 2 * stations.size();
    for (size_t i = 0; i < verts.size(); ++i) {
        const float a = (haveStations && origin_station_m)
                            ? detail::ribbon_fade_alpha(stations[i / 2], *origin_station_m,
                                                        cfg.fade_start_m, cfg.fade_end_m)
                            : 1.0f;
        verts[i].color = float4{1.0f, 1.0f, 1.0f, a};
    }
    return verts;
}

void build_slot_meshes(VisualRenderer& r, VisualRenderer::RibbonSlot& slot, PathRole role,
                       const Vec3* pts, uint32_t n) {
    destroy_slot_meshes(r, slot);
    slot.totalVertexCount = 0;
    filament::MaterialInstance* mat = r.ribbonMaterial[static_cast<uint8_t>(role)];
    const float halfWidthM = build_effective_half_width(r.active_theme.ribbon, role);
    slot.halfWidthM = halfWidthM;
    slot.firstPointM = n > 0 ? pts[0] : Vec3{};

    slot.fadeOriginStationM.reset();
    slot.minVertexAlpha = 1.0f;

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

        std::vector<Vertex> verts =
            build_ribbon_vertices(strip, stations, std::nullopt, r.active_theme.ribbon);
        for (const Vertex& v : verts)
            slot.minVertexAlpha = std::min(slot.minVertexAlpha, v.color.a);

        slot.totalVertexCount += static_cast<uint32_t>(verts.size());
        Mesh mesh;
        add_mesh(r, mesh, std::move(verts), std::move(idx),
                 filament::RenderableManager::PrimitiveType::TRIANGLES, mat, false, true);
        filament::RenderableManager& rm = r.engine->getRenderableManager();
        rm.setPriority(rm.getInstance(mesh.entity),
                       kRibbonPriorityByRole[static_cast<uint8_t>(role)]);
        slot.meshes.push_back(std::move(mesh));
        slot.baseStripPositions.push_back(std::move(strip));
        slot.pointStations.push_back(std::move(stations));
    }
}

void apply_ribbon_clip(VisualRenderer& r, VisualRenderer::RibbonSlot& slot,
                       const RibbonClip& clip) {
    const bool changed = !slot.has_applied_clip || slot.appliedClipActive != clip.active ||
                         (clip.active && slot.appliedClipUnits != clip.quantized_units);
    if (!changed) return;
    slot.minVertexAlpha = 1.0f;
    slot.fadeOriginStationM = clip.active ? std::optional<double>(clip.station_m) : std::nullopt;
    for (size_t i = 0; i < slot.meshes.size(); ++i) {
        std::vector<Vec3> positions = slot.baseStripPositions[i];
        detail::collapse_clipped_positions(positions, slot.pointStations[i], clip.active,
                                           clip.station_m);
        if (i == 0 && !positions.empty()) slot.firstPointM = positions[0];
        std::vector<Vertex> verts = build_ribbon_vertices(
            positions, slot.pointStations[i], slot.fadeOriginStationM, r.active_theme.ribbon);
        for (const Vertex& v : verts)
            slot.minVertexAlpha = std::min(slot.minVertexAlpha, v.color.a);
        update_mesh_positions(*r.engine, slot.meshes[i], std::move(verts));
    }
    slot.has_applied_clip = true;
    slot.appliedClipActive = clip.active;
    slot.appliedClipUnits = clip.quantized_units;
}

void rebind_slot_material(VisualRenderer& r, VisualRenderer::RibbonSlot& slot,
                          filament::MaterialInstance* mat) {
    filament::RenderableManager& rm = r.engine->getRenderableManager();
    for (auto& mesh : slot.meshes) {
        const auto ri = rm.getInstance(mesh.entity);
        if (ri.isValid()) rm.setMaterialInstanceAt(ri, 0, mat);
    }
}

}

void update_ribbons(VisualRenderer& r, const SceneGraph& s) {
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

        const RibbonClip clip = compute_ribbon_clip(ribbon, s.ego);

        const float effectiveHalfWidthM =
            build_effective_half_width(r.active_theme.ribbon, ribbon.role);
        const uint64_t sig =
            ribbon_signature(ribbon.role, ribbon.points, ribbon.point_count, effectiveHalfWidthM,
                             r.active_theme.ribbon.fade_start_m, r.active_theme.ribbon.fade_end_m);
        if (!slot.has_signature || slot.signature != sig) {
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

        const auto stalenessAlpha = static_cast<float>(detail::SceneBuffer::staleness_alpha(
            s.sim_time_sec, ribbon.last_update_sec, kStaleFadeStartSec, kStaleFadeTimeoutSec));
        const auto roleIdx = static_cast<uint8_t>(slot.role);
        const float opacity = r.active_theme.ribbon.opacity;
        const bool lengthFade = detail::ribbon_length_fade_enabled(
            r.active_theme.ribbon.fade_start_m, r.active_theme.ribbon.fade_end_m);
        const float alpha = stalenessAlpha * opacity;

        if (alpha >= 1.0f && !lengthFade) {
            if (slot.fadeInstance != nullptr) {
                rebind_slot_material(r, slot, r.ribbonMaterial[roleIdx]);
                r.engine->destroy(slot.fadeInstance);
                slot.fadeInstance = nullptr;
                slot.fadeAlpha = 1.0f;
            }
        } else {
            if (slot.fadeInstance == nullptr) {
                slot.fadeInstance = r.ribbonFadedMaterial->createInstance();
                slot.fadeInstance->setCullingMode(filament::backend::CullingMode::NONE);
                rebind_slot_material(r, slot, slot.fadeInstance);
            }
            const detail::Float3& tint = r.ribbonTint[roleIdx];
            const detail::Float3& glow = r.active_theme.palette.ribbon_glow;
            slot.fadeInstance->setParameter("baseColor", float4{tint.r, tint.g, tint.b, alpha});
            slot.fadeInstance->setParameter("roughness", r.active_theme.material.roughness);
            slot.fadeInstance->setParameter("metallic", r.active_theme.material.metallic);
            slot.fadeInstance->setParameter("emissiveColor", float3{glow.r, glow.g, glow.b});
            slot.fadeInstance->setParameter(
                "emissiveStrength",
                slot.role == PathRole::BEHAVIOR ? r.active_theme.emissive.ribbon_strength : 0.0f);
            slot.fadeAlpha = alpha;
        }
    }
}

}

namespace overlume::testing {

float ribbon_fade_alpha(double station_m, double origin_station_m, float fade_start_m,
                        float fade_end_m) {
    return overlume::detail::ribbon_fade_alpha(station_m, origin_station_m, fade_start_m,
                                               fade_end_m);
}

float ribbon_vertex_fade_alpha(overlume::VisualRenderer* r, size_t slot, size_t vertex_idx) {
    if (r == nullptr || slot >= r->ribbonSlots.size()) return -1.0f;
    const auto& s = r->ribbonSlots[slot];
    if (s.pointStations.empty() || vertex_idx / 2 >= s.pointStations[0].size()) return -1.0f;
    if (!s.fadeOriginStationM) return 1.0f;
    return overlume::detail::ribbon_fade_alpha(
        s.pointStations[0][vertex_idx / 2], *s.fadeOriginStationM,
        r->active_theme.ribbon.fade_start_m, r->active_theme.ribbon.fade_end_m);
}

overlume::detail::Float3 ribbon_role_base_color(overlume::VisualRenderer* r,
                                                overlume::PathRole role) {
    if (r == nullptr) return {};
    return r->ribbonTint[static_cast<uint8_t>(role)];
}

size_t ribbon_mesh_count(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->ribbonSlots.size()) return 0;
    return r->ribbonSlots[slot].meshes.size();
}

size_t ribbon_vertex_count(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->ribbonSlots.size()) return 0;
    return r->ribbonSlots[slot].totalVertexCount;
}

RibbonMaterialInfo ribbon_slot_material_info(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->ribbonSlots.size()) return {};
    const overlume::VisualRenderer::RibbonSlot& s = r->ribbonSlots[slot];
    RibbonMaterialInfo info;
    info.alpha = s.fadeAlpha;
    info.minVertexAlpha = s.minVertexAlpha;
    if (s.meshes.empty()) return info;
    filament::RenderableManager& rm = r->engine->getRenderableManager();
    const auto ri = rm.getInstance(s.meshes.front().entity);
    if (!ri.isValid()) return info;
    filament::MaterialInstance* bound = rm.getMaterialInstanceAt(ri, 0);
    info.bound_to_translucent = bound != nullptr && bound->getMaterial() == r->ribbonFadedMaterial;
    return info;
}

float ribbon_slot_half_width_m(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->ribbonSlots.size()) return 0.0f;
    return r->ribbonSlots[slot].halfWidthM;
}

bool ribbon_slot_first_point(overlume::VisualRenderer* r, size_t slot, overlume::Vec3* out) {
    if (r == nullptr || slot >= r->ribbonSlots.size() || out == nullptr) return false;
    const auto& s = r->ribbonSlots[slot];
    if (s.meshes.empty()) return false;
    *out = s.firstPointM;
    return true;
}

uint64_t ribbon_rebuild_count(overlume::VisualRenderer* r) {
    if (r == nullptr) return 0;
    return r->ribbonRebuildCount;
}

}
