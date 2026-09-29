// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "ground_grid.hpp"
#include "ground_grid_test_hooks.hpp"
#include "renderer_internal.hpp"
#include "overlume/scene.h"

#include <filament/RenderableManager.h>
#include <filament/Texture.h>
#include <filament/TextureSampler.h>

#include <backend/PixelBufferDescriptor.h>

#include <math/vec3.h>
#include <math/vec4.h>

#include <utils/EntityManager.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace overlume {

namespace {

using filament::math::float2;
using filament::math::float3;
using filament::math::float4;

constexpr float kGradientZLiftM = 0.010f;
constexpr float kDynamicZLiftM = 0.015f;

float z_lift_for_kind(uint8_t kind) { return kind == 0 ? kDynamicZLiftM : kGradientZLiftM; }

uint8_t priority_for_kind(uint8_t kind) { return kind == 0 ? 1 : 0; }

struct GroundGridVertex {
    float3 position;
    float4 tangentFrame;
    float2 uv;
};

filament::VertexBuffer* make_ground_grid_vertex_buffer(filament::Engine& engine,
                                                       std::vector<GroundGridVertex> verts) {
    auto* heapVerts = new std::vector<GroundGridVertex>(std::move(verts));
    filament::VertexBuffer* vb =
        filament::VertexBuffer::Builder()
            .vertexCount(static_cast<uint32_t>(heapVerts->size()))
            .bufferCount(1)
            .attribute(filament::VertexAttribute::POSITION, 0,
                       filament::VertexBuffer::AttributeType::FLOAT3,
                       offsetof(GroundGridVertex, position), sizeof(GroundGridVertex))
            .attribute(filament::VertexAttribute::TANGENTS, 0,
                       filament::VertexBuffer::AttributeType::FLOAT4,
                       offsetof(GroundGridVertex, tangentFrame), sizeof(GroundGridVertex))
            .attribute(filament::VertexAttribute::UV0, 0,
                       filament::VertexBuffer::AttributeType::FLOAT2,
                       offsetof(GroundGridVertex, uv), sizeof(GroundGridVertex))
            .build(engine);
    vb->setBufferAt(engine, 0,
                    filament::VertexBuffer::BufferDescriptor(
                        heapVerts->data(), heapVerts->size() * sizeof(GroundGridVertex),
                        [](void*, size_t, void* user) {
                            delete static_cast<std::vector<GroundGridVertex>*>(user);
                        },
                        heapVerts));
    return vb;
}

void build_ground_grid_quad(VisualRenderer& r, Mesh& mesh, const GroundGridLayer& g, float z_lift,
                            filament::MaterialInstance* material, Vec3 (&corners)[4]) {
    const float ox = static_cast<float>(g.origin.x);
    const float oy = static_cast<float>(g.origin.y);
    const float oz = static_cast<float>(g.origin.z) + z_lift;
    const float w = static_cast<float>(g.width_cells) * static_cast<float>(g.resolution_m);
    const float h = static_cast<float>(g.height_cells) * static_cast<float>(g.resolution_m);
    const float c = static_cast<float>(std::cos(g.yaw_rad));
    const float s = static_cast<float>(std::sin(g.yaw_rad));
    const auto at = [&](float lx, float ly) {
        return float3{ox + c * lx - s * ly, oy + s * lx + c * ly, oz};
    };

    std::vector<GroundGridVertex> verts = {
        {at(0.0f, 0.0f), {}, {0.0f, 0.0f}},
        {at(w, 0.0f), {}, {1.0f, 0.0f}},
        {at(w, h), {}, {1.0f, 1.0f}},
        {at(0.0f, h), {}, {0.0f, 1.0f}},
    };
    for (size_t i = 0; i < 4; ++i) {
        corners[i] = {verts[i].position.x, verts[i].position.y, verts[i].position.z};
    }
    std::vector<uint16_t> indices = {0, 1, 2, 0, 2, 3};

    std::vector<Vertex> plain(verts.size());
    for (size_t i = 0; i < verts.size(); ++i) plain[i].position = verts[i].position;
    fill_tangent_frames(plain, std::vector<float3>(verts.size(), float3{0.0f, 0.0f, 1.0f}));
    for (size_t i = 0; i < verts.size(); ++i) verts[i].tangentFrame = plain[i].tangentFrame;

    mesh.vb = make_ground_grid_vertex_buffer(*r.engine, std::move(verts));
    mesh.ib = make_index_buffer(*r.engine, std::move(indices));
    mesh.entity = utils::EntityManager::get().create();
    const float reach = std::hypot(w, h) + 1.0f;
    filament::RenderableManager::Builder(1)
        .boundingBox({{0, 0, 0}, {reach + std::abs(ox), reach + std::abs(oy), 1.0f}})
        .geometry(0, filament::RenderableManager::PrimitiveType::TRIANGLES, mesh.vb, mesh.ib)
        .material(0, material)
        .priority(priority_for_kind(g.kind))
        .culling(false)
        .castShadows(false)
        .receiveShadows(true)
        .build(*r.engine, mesh.entity);
    r.scene->addEntity(mesh.entity);
}

filament::Texture* build_occupancy_texture(filament::Engine& engine, uint32_t w, uint32_t h) {
    return filament::Texture::Builder()
        .width(w)
        .height(h)
        .levels(1)
        .format(filament::Texture::InternalFormat::R8)
        .sampler(filament::Texture::Sampler::SAMPLER_2D)
        .build(engine);
}

void upload_occupancy_texture(filament::Engine& engine, filament::Texture* tex,
                              const uint8_t* cells, uint32_t w, uint32_t h) {
    const size_t byteCount = static_cast<size_t>(w) * h;
    auto* heap = new std::vector<uint8_t>(cells, cells + byteCount);
    filament::backend::PixelBufferDescriptor pbd(
        heap->data(), heap->size(), filament::backend::PixelBufferDescriptor::PixelDataFormat::R,
        filament::backend::PixelBufferDescriptor::PixelDataType::UBYTE,
        [](void*, size_t, void* user) { delete static_cast<std::vector<uint8_t>*>(user); }, heap);
    tex->setImage(engine, 0, std::move(pbd));
}

}

void update_ground_grids(VisualRenderer& r, const SceneGraph& s) {
    while (r.groundGridSlots.size() > s.grid_count) {
        VisualRenderer::GroundGridSlot& slot = r.groundGridSlots.back();
        destroy_mesh(*r.engine, *r.scene, slot.quad);
        if (slot.texture != nullptr) r.engine->destroy(slot.texture);
        r.groundGridSlots.pop_back();
    }
    if (r.groundGridSlots.size() < s.grid_count) r.groundGridSlots.resize(s.grid_count);

    for (uint32_t i = 0; i < s.grid_count; ++i) {
        const GroundGridLayer& g = s.grids[i];
        VisualRenderer::GroundGridSlot& slot = r.groundGridSlots[i];

        if (g.cells == nullptr || g.width_cells == 0 || g.height_cells == 0) continue;

        const uint8_t kind =
            g.kind < VisualRenderer::kGroundGridKindCount ? g.kind : static_cast<uint8_t>(0);

        const bool geomChanged =
            !slot.has_geometry || slot.kind != kind || slot.width_cells != g.width_cells ||
            slot.height_cells != g.height_cells || slot.resolution_m != g.resolution_m ||
            slot.origin.x != g.origin.x || slot.origin.y != g.origin.y ||
            slot.origin.z != g.origin.z || slot.yaw_rad != g.yaw_rad;
        if (geomChanged) {
            destroy_mesh(*r.engine, *r.scene, slot.quad);
            build_ground_grid_quad(r, slot.quad, g, z_lift_for_kind(kind),
                                   r.groundGridMaterialInstance[kind], slot.corners);
            slot.kind = kind;
            slot.width_cells = g.width_cells;
            slot.height_cells = g.height_cells;
            slot.resolution_m = g.resolution_m;
            slot.origin = g.origin;
            slot.yaw_rad = g.yaw_rad;
            slot.has_geometry = true;
        }

        const bool dimsChanged = slot.texture == nullptr || slot.texWidth != g.width_cells ||
                                 slot.texHeight != g.height_cells;
        if (dimsChanged) {
            if (slot.texture != nullptr) r.engine->destroy(slot.texture);
            slot.texture = build_occupancy_texture(*r.engine, g.width_cells, g.height_cells);
            slot.texWidth = g.width_cells;
            slot.texHeight = g.height_cells;
            ++slot.textureGeneration;
        }

        if (dimsChanged || geomChanged || slot.last_upload_sec != g.last_update_sec) {
            upload_occupancy_texture(*r.engine, slot.texture, g.cells, g.width_cells,
                                     g.height_cells);
            slot.last_upload_sec = g.last_update_sec;
            ++slot.uploadCount;
        }

        filament::MaterialInstance* mat = r.groundGridMaterialInstance[kind];
        mat->setParameter("occupancyTexture", slot.texture,
                          filament::TextureSampler(filament::TextureSampler::MinFilter::NEAREST,
                                                   filament::TextureSampler::MagFilter::NEAREST));

        const auto alpha = static_cast<float>(detail::SceneBuffer::staleness_alpha(
            s.sim_time_sec, g.last_update_sec, kStaleFadeStartSec, kStaleFadeTimeoutSec));
        mat->setParameter("alpha", alpha);
        r.groundGridAlpha[kind] = alpha;
    }
}

}

namespace overlume::testing {

const void* ground_grid_texture_handle(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->groundGridSlots.size()) return nullptr;
    return r->groundGridSlots[slot].texture;
}

uint32_t ground_grid_texture_generation(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->groundGridSlots.size()) return 0;
    return r->groundGridSlots[slot].textureGeneration;
}

uint32_t ground_grid_texture_upload_count(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->groundGridSlots.size()) return 0;
    return r->groundGridSlots[slot].uploadCount;
}

bool ground_grid_corner(overlume::VisualRenderer* r, size_t slot, size_t corner,
                        overlume::Vec3* out) {
    if (r == nullptr || out == nullptr || corner >= 4 || slot >= r->groundGridSlots.size()) {
        return false;
    }
    const auto& s = r->groundGridSlots[slot];
    if (!s.has_geometry) return false;
    *out = s.corners[corner];
    return true;
}

float ground_grid_material_alpha(overlume::VisualRenderer* r, uint8_t kind) {
    if (r == nullptr || kind >= overlume::VisualRenderer::kGroundGridKindCount) return 1.0f;
    return r->groundGridAlpha[kind];
}

bool ground_grid_ramp_entry(overlume::VisualRenderer* r, uint8_t kind, uint32_t value,
                            overlume::detail::Float3* color, float* alpha) {
    if (r == nullptr || kind >= overlume::VisualRenderer::kGroundGridKindCount ||
        value >= overlume::detail::Theme::OgmRamp::kEntries) {
        return false;
    }
    if (color) *color = r->groundGridRamp[kind].color[value];
    if (alpha) *alpha = r->groundGridRamp[kind].alpha[value];
    return true;
}

}
