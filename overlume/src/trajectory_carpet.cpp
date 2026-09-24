// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "trajectory_carpet.hpp"
#include "trajectory_carpet_test_hooks.hpp"
#include "polyline.hpp"
#include "ribbon.hpp"
#include "renderer_internal.hpp"
#include "theme.hpp"
#include "overlume/scene.h"

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

namespace overlume {

namespace {

using filament::math::float3;

constexpr float kTrajectoryCarpetBoundsM = 200.0f;

constexpr float kVelocityRibbonZLiftM = 0.052f;
constexpr uint8_t kVelocityRibbonPriority = 2;

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
                       filament::VertexBuffer::AttributeType::UBYTE4, offsetof(CarpetVertex, rgba),
                       sizeof(CarpetVertex))
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

uint64_t trajectory_carpet_signature(const Vec3* pts, uint32_t n, float half_width_m,
                                     float fade_start) {
    uint64_t h = hash_combine(0, static_cast<uint64_t>(n));
    uint32_t fadeBits;
    std::memcpy(&fadeBits, &fade_start, sizeof(fadeBits));
    h = hash_combine(h, static_cast<uint64_t>(fadeBits));
    if (n > 0) {
        h = hash_combine(h, hash_vec3(pts[0]));
        h = hash_combine(h, hash_vec3(pts[n - 1]));
    }
    uint32_t widthBits;
    std::memcpy(&widthBits, &half_width_m, sizeof(widthBits));
    h = hash_combine(h, static_cast<uint64_t>(widthBits));
    return h;
}

float velocity_ribbon_half_width_m(const detail::Theme::Ribbon& cfg) {
    const float halfWidth = (cfg.lane_width_m - 2.0f * cfg.margin_velocity_m) * 0.5f;
    return std::max(halfWidth, kRibbonMinHalfWidthM);
}

uint32_t with_fade_alpha(uint32_t rgba, double station_m, double total_length_m, float fade_start) {
    const float a = detail::ribbon_fade_alpha(station_m, total_length_m, fade_start);
    const auto byte = static_cast<uint32_t>(std::clamp(a, 0.0f, 1.0f) * 255.0f + 0.5f);
    return (rgba & 0x00FFFFFFu) | (byte << 24);
}

uint32_t resolve_rgba(const VisualRenderer& r, uint32_t packed) {
    const uint32_t a = (packed >> 24) & 0xFFu;
    if (a != 0) return packed;
    const detail::Float3& tint = r.active_theme.palette.object_tints.unknown;
    const auto to_byte = [](float c) {
        return static_cast<uint32_t>(std::clamp(c, 0.0f, 1.0f) * 255.0f + 0.5f);
    };
    return to_byte(tint.r) | (to_byte(tint.g) << 8) | (to_byte(tint.b) << 16) | (255u << 24);
}

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
            if ((dx * dx + dy * dy + dz * dz) < (kEps * kEps)) continue;
        }
        out.push_back(pts[i]);
    }
    return out;
}

void destroy_slot_meshes(VisualRenderer& r, VisualRenderer::TrajectoryCarpetSlot& slot) {
    for (auto& m : slot.meshes) destroy_mesh(*r.engine, *r.scene, m);
    slot.meshes.clear();
    slot.firstMeshRgba.clear();
    slot.firstMeshZ.clear();
    slot.baseStripPositions.clear();
    slot.baseStripRgba.clear();
    slot.pointStations.clear();
    slot.totalLengthM = 0.0;
    slot.has_applied_clip = false;
}

void update_carpet_vertex_positions(filament::Engine& engine, Mesh& mesh,
                                    std::vector<CarpetVertex> verts) {
    if (mesh.vb == nullptr) return;
    auto* heap = new std::vector<CarpetVertex>(std::move(verts));
    mesh.vb->setBufferAt(
        engine, 0,
        filament::VertexBuffer::BufferDescriptor(
            heap->data(), heap->size() * sizeof(CarpetVertex),
            [](void*, size_t, void* user) { delete static_cast<std::vector<CarpetVertex>*>(user); },
            heap));
}

void build_slot_meshes(VisualRenderer& r, VisualRenderer::TrajectoryCarpetSlot& slot,
                       const PointCloudPoint* pts, uint32_t n, float halfWidthM) {
    destroy_slot_meshes(r, slot);
    slot.totalVertexCount = 0;
    slot.halfWidthM = halfWidthM;
    slot.boundFaded = false;

    const std::vector<PointCloudPoint> cleaned = clean_carpet_points(pts, n);
    if (cleaned.size() < 2) return;

    std::vector<Vec3> positions(cleaned.size());
    for (size_t i = 0; i < cleaned.size(); ++i) positions[i] = cleaned[i].position;
    slot.firstPointM = positions[0];

    const std::vector<double> stations =
        detail::clean_polyline_stations(positions.data(), positions.size());
    slot.totalLengthM = stations.empty() ? 0.0 : stations.back();
    const float fadeStart = r.active_theme.ribbon.fade_start;

    bool first_chunk = true;
    for (auto [a, b] : detail::polyline_chunks(static_cast<uint32_t>(cleaned.size()))) {
        const uint32_t chunkN = b - a;
        std::vector<Vec3> strip = detail::extrude_polyline(positions.data() + a, chunkN, halfWidthM,
                                                           kVelocityRibbonZLiftM);
        if (strip.empty()) continue;
        std::vector<uint16_t> idx =
            detail::extrude_polyline_indices(static_cast<uint32_t>(strip.size() / 2));
        if (idx.empty()) continue;

        std::vector<uint32_t> rgba(strip.size());
        for (size_t i = 0; i < strip.size(); ++i)
            rgba[i] = resolve_rgba(r, cleaned[a + i / 2].rgba);

        std::vector<CarpetVertex> verts(strip.size());
        for (size_t i = 0; i < strip.size(); ++i) {
            verts[i].position =
                float3{static_cast<float>(strip[i].x), static_cast<float>(strip[i].y),
                       static_cast<float>(strip[i].z)};
            verts[i].rgba =
                with_fade_alpha(rgba[i], stations[a + i / 2], slot.totalLengthM, fadeStart);
        }
        if (first_chunk) {
            slot.firstMeshRgba = rgba;
            slot.firstMeshZ.reserve(verts.size());
            for (const auto& v : verts) slot.firstMeshZ.push_back(v.position.z);
            first_chunk = false;
        }
        slot.totalVertexCount += static_cast<uint32_t>(verts.size());
        Mesh mesh;
        mesh.vertexCount = static_cast<uint32_t>(verts.size());
        mesh.vb = make_carpet_vertex_buffer(*r.engine, std::move(verts));
        mesh.ib = make_index_buffer(*r.engine, std::move(idx));
        mesh.entity = utils::EntityManager::get().create();
        filament::RenderableManager::Builder(1)
            .boundingBox(
                {{0, 0, 0},
                 {kTrajectoryCarpetBoundsM, kTrajectoryCarpetBoundsM, kTrajectoryCarpetBoundsM}})
            .geometry(0, filament::RenderableManager::PrimitiveType::TRIANGLES, mesh.vb, mesh.ib)
            .material(0, r.trajectoryCarpetMaterialInstance)
            .priority(kVelocityRibbonPriority)
            .culling(false)
            .castShadows(false)
            .receiveShadows(false)
            .build(*r.engine, mesh.entity);
        r.scene->addEntity(mesh.entity);
        slot.meshes.push_back(std::move(mesh));
        slot.baseStripPositions.push_back(std::move(strip));
        slot.baseStripRgba.push_back(std::move(rgba));
        slot.pointStations.emplace_back(stations.begin() + a, stations.begin() + b);
    }
}

void apply_carpet_clip(VisualRenderer& r, VisualRenderer::TrajectoryCarpetSlot& slot,
                       const detail::PolylineClip& clip) {
    const bool changed = !slot.has_applied_clip || slot.appliedClipActive != clip.active ||
                         (clip.active && slot.appliedClipUnits != clip.quantized_units);
    if (!changed) return;
    for (size_t i = 0; i < slot.meshes.size(); ++i) {
        std::vector<Vec3> positions = slot.baseStripPositions[i];
        detail::collapse_clipped_positions(positions, slot.pointStations[i], clip.active,
                                           clip.station_m);
        if (i == 0 && !positions.empty()) slot.firstPointM = positions[0];
        std::vector<CarpetVertex> verts(positions.size());
        for (size_t v = 0; v < positions.size(); ++v) {
            verts[v].position =
                float3{static_cast<float>(positions[v].x), static_cast<float>(positions[v].y),
                       static_cast<float>(positions[v].z)};
            verts[v].rgba = with_fade_alpha(slot.baseStripRgba[i][v], slot.pointStations[i][v / 2],
                                            slot.totalLengthM, r.active_theme.ribbon.fade_start);
        }
        update_carpet_vertex_positions(*r.engine, slot.meshes[i], std::move(verts));
    }
    slot.has_applied_clip = true;
    slot.appliedClipActive = clip.active;
    slot.appliedClipUnits = clip.quantized_units;
}

}

void update_trajectory_carpets(VisualRenderer& r, const SceneGraph& s) {
    while (r.trajectoryCarpetSlots.size() > s.trajectory_carpet_count) {
        destroy_slot_meshes(r, r.trajectoryCarpetSlots.back());
        r.trajectoryCarpetSlots.pop_back();
    }
    if (r.trajectoryCarpetSlots.size() < s.trajectory_carpet_count) {
        r.trajectoryCarpetSlots.resize(s.trajectory_carpet_count);
    }

    const float halfWidthM = velocity_ribbon_half_width_m(r.active_theme.ribbon);

    float alpha = s.trajectory_carpet_count > 0 ? 0.0f : 1.0f;
    for (uint32_t i = 0; i < s.trajectory_carpet_count; ++i) {
        const TrajectoryCarpet& tc = s.trajectory_carpets[i];
        VisualRenderer::TrajectoryCarpetSlot& slot = r.trajectoryCarpetSlots[i];

        std::vector<Vec3> rawPositions(tc.point_count);
        for (uint32_t j = 0; j < tc.point_count; ++j) rawPositions[j] = tc.points[j].position;
        const detail::PolylineClip clip =
            s.ego.valid
                ? detail::compute_polyline_clip(rawPositions.data(), tc.point_count, s.ego.position)
                : detail::PolylineClip{};

        const uint64_t sig = trajectory_carpet_signature(
            rawPositions.data(), tc.point_count, halfWidthM, r.active_theme.ribbon.fade_start);
        if (!slot.has_signature || slot.signature != sig) {
            ++r.trajectoryCarpetRebuildCount;
            build_slot_meshes(r, slot, tc.points, tc.point_count, halfWidthM);
            slot.signature = sig;
            slot.has_signature = true;
        }

        apply_carpet_clip(r, slot, clip);

        alpha = std::max(alpha, static_cast<float>(detail::SceneBuffer::staleness_alpha(
                                    s.sim_time_sec, tc.last_update_sec, kStaleFadeStartSec,
                                    kStaleFadeTimeoutSec)));
    }
    alpha *= r.active_theme.ribbon.opacity;
    const bool wantFaded = alpha < 1.0f || r.active_theme.ribbon.fade_start < 1.0f;
    if (wantFaded) {
        r.trajectoryCarpetFadedMaterialInstance->setParameter("alpha", alpha);
    }
    filament::RenderableManager& rm = r.engine->getRenderableManager();
    for (auto& slot : r.trajectoryCarpetSlots) {
        if (slot.boundFaded == wantFaded) continue;
        filament::MaterialInstance* mat = wantFaded ? r.trajectoryCarpetFadedMaterialInstance
                                                    : r.trajectoryCarpetMaterialInstance;
        for (auto& mesh : slot.meshes) {
            const auto ri = rm.getInstance(mesh.entity);
            if (ri.isValid()) rm.setMaterialInstanceAt(ri, 0, mat);
        }
        slot.boundFaded = wantFaded;
    }
    r.trajectoryCarpetAlpha = alpha;
}

}

namespace overlume::testing {

size_t trajectory_carpet_mesh_count(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->trajectoryCarpetSlots.size()) return 0;
    return r->trajectoryCarpetSlots[slot].meshes.size();
}

size_t trajectory_carpet_vertex_count(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->trajectoryCarpetSlots.size()) return 0;
    return r->trajectoryCarpetSlots[slot].totalVertexCount;
}

float trajectory_carpet_material_alpha(overlume::VisualRenderer* r) {
    if (r == nullptr) return 1.0f;
    return r->trajectoryCarpetAlpha;
}

uint32_t trajectory_carpet_vertex_rgba(overlume::VisualRenderer* r, size_t slot,
                                       size_t vertex_idx) {
    if (r == nullptr || slot >= r->trajectoryCarpetSlots.size()) return 0;
    const auto& rgba = r->trajectoryCarpetSlots[slot].firstMeshRgba;
    if (vertex_idx >= rgba.size()) return 0;
    return rgba[vertex_idx];
}

float trajectory_carpet_vertex_z(overlume::VisualRenderer* r, size_t slot, size_t vertex_idx) {
    if (r == nullptr || slot >= r->trajectoryCarpetSlots.size()) return 0.0f;
    const auto& z = r->trajectoryCarpetSlots[slot].firstMeshZ;
    if (vertex_idx >= z.size()) return 0.0f;
    return z[vertex_idx];
}

float trajectory_carpet_half_width_m(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->trajectoryCarpetSlots.size()) return 0.0f;
    return r->trajectoryCarpetSlots[slot].halfWidthM;
}

float trajectory_carpet_vertex_fade_alpha(overlume::VisualRenderer* r, size_t slot,
                                          size_t vertex_idx) {
    if (r == nullptr || slot >= r->trajectoryCarpetSlots.size()) return 0.0f;
    const auto& s = r->trajectoryCarpetSlots[slot];
    if (s.pointStations.empty() || vertex_idx / 2 >= s.pointStations[0].size()) return 0.0f;
    return overlume::detail::ribbon_fade_alpha(s.pointStations[0][vertex_idx / 2], s.totalLengthM,
                                               r->active_theme.ribbon.fade_start);
}

uint64_t trajectory_carpet_rebuild_count(overlume::VisualRenderer* r) {
    if (r == nullptr) return 0;
    return r->trajectoryCarpetRebuildCount;
}

bool trajectory_carpet_slot_first_point(overlume::VisualRenderer* r, size_t slot,
                                        overlume::Vec3* out) {
    if (r == nullptr || slot >= r->trajectoryCarpetSlots.size() || out == nullptr) return false;
    const auto& s = r->trajectoryCarpetSlots[slot];
    if (s.meshes.empty()) return false;
    *out = s.firstPointM;
    return true;
}

}
