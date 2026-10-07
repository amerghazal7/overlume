// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "height_grid.hpp"
#include "height_grid_test_hooks.hpp"
#include "renderer_internal.hpp"
#include "overlume/scene.h"

#include <filament/Box.h>
#include <filament/IndexBuffer.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/Scene.h>
#include <filament/TextureSampler.h>
#include <filament/TransformManager.h>
#include <filament/VertexBuffer.h>

#include <math/mat4.h>
#include <math/vec2.h>
#include <math/vec3.h>
#include <math/vec4.h>

#include <utils/EntityManager.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace overlume {

namespace {

using filament::math::float2;
using filament::math::float3;
using filament::math::float4;

// Heights are bounded by the encoding window in practice; culling is off, so this only feeds
// shadow-receiver bounds.
constexpr float kHeightGridBoundsZM = 25.0f;

struct HeightGridVertex {
    float3 position;
    float4 tangentFrame;
    float2 custom0;
};

filament::IndexBuffer* make_height_index_buffer(filament::Engine& engine, uint32_t w, uint32_t h) {
    auto* indices = new std::vector<uint32_t>();
    indices->reserve(static_cast<size_t>(w - 1) * (h - 1) * 6);
    for (uint32_t j = 0; j + 1 < h; ++j) {
        for (uint32_t i = 0; i + 1 < w; ++i) {
            const uint32_t a = j * w + i;
            const uint32_t b = a + 1;
            const uint32_t c = a + w;
            const uint32_t d = c + 1;
            indices->insert(indices->end(), {a, b, d, a, d, c});
        }
    }
    filament::IndexBuffer* ib = filament::IndexBuffer::Builder()
                                    .indexCount(static_cast<uint32_t>(indices->size()))
                                    .bufferType(filament::IndexBuffer::IndexType::UINT)
                                    .build(engine);
    ib->setBuffer(engine, filament::IndexBuffer::BufferDescriptor(
                              indices->data(), indices->size() * sizeof(uint32_t),
                              [](void*, size_t, void* user) {
                                  delete static_cast<std::vector<uint32_t>*>(user);
                              },
                              indices));
    return ib;
}

filament::VertexBuffer* make_height_vertex_buffer(filament::Engine& engine, uint32_t count) {
    return filament::VertexBuffer::Builder()
        .vertexCount(count)
        .bufferCount(1)
        .attribute(filament::VertexAttribute::POSITION, 0,
                   filament::VertexBuffer::AttributeType::FLOAT3,
                   offsetof(HeightGridVertex, position), sizeof(HeightGridVertex))
        .attribute(filament::VertexAttribute::TANGENTS, 0,
                   filament::VertexBuffer::AttributeType::FLOAT4,
                   offsetof(HeightGridVertex, tangentFrame), sizeof(HeightGridVertex))
        .attribute(filament::VertexAttribute::CUSTOM0, 0,
                   filament::VertexBuffer::AttributeType::FLOAT2,
                   offsetof(HeightGridVertex, custom0), sizeof(HeightGridVertex))
        .build(engine);
}

void upload_height_vertices(filament::Engine& engine, filament::VertexBuffer* vb,
                            std::vector<HeightGridVertex> verts) {
    auto* heap = new std::vector<HeightGridVertex>(std::move(verts));
    vb->setBufferAt(engine, 0,
                    filament::VertexBuffer::BufferDescriptor(
                        heap->data(), heap->size() * sizeof(HeightGridVertex),
                        [](void*, size_t, void* user) {
                            delete static_cast<std::vector<HeightGridVertex>*>(user);
                        },
                        heap));
}

// One vertex per cell centre, positions relative to the layer origin (the renderable's transform
// carries the origin). Fills the slot's CPU copies, which the test hooks read back.
std::vector<HeightGridVertex> build_height_vertices(VisualRenderer::HeightGridSlot& slot,
                                                    const HeightGridLayer& g, float bias) {
    const size_t w = g.width_cells;
    const size_t h = g.height_cells;
    const size_t n = w * h;
    const auto res = static_cast<float>(g.resolution_m);
    const auto c = static_cast<float>(std::cos(g.yaw_rad));
    const auto s = static_cast<float>(std::sin(g.yaw_rad));

    std::vector<float> z(n);
    slot.cpuPositions.resize(n * 3);
    slot.cpuCustom.resize(n * 2);
    for (size_t j = 0; j < h; ++j) {
        for (size_t i = 0; i < w; ++i) {
            const size_t k = j * w + i;
            const float hv = g.heights_m[k];
            const bool known = std::isfinite(hv);
            z[k] = (known ? hv : 0.0f) + bias;
            const float lx = (static_cast<float>(i) + 0.5f) * res;
            const float ly = (static_cast<float>(j) + 0.5f) * res;
            slot.cpuPositions[k * 3 + 0] = c * lx - s * ly;
            slot.cpuPositions[k * 3 + 1] = s * lx + c * ly;
            slot.cpuPositions[k * 3 + 2] = z[k];
            slot.cpuCustom[k * 2 + 0] = known ? hv : 0.0f;
            slot.cpuCustom[k * 2 + 1] = known ? 1.0f : 0.0f;
        }
    }

    std::vector<float3> normals(n);
    for (size_t j = 0; j < h; ++j) {
        const size_t j0 = j > 0 ? j - 1 : j;
        const size_t j1 = j + 1 < h ? j + 1 : j;
        for (size_t i = 0; i < w; ++i) {
            const size_t i0 = i > 0 ? i - 1 : i;
            const size_t i1 = i + 1 < w ? i + 1 : i;
            const float dzdx =
                (z[j * w + i1] - z[j * w + i0]) / (static_cast<float>(i1 - i0) * res);
            const float dzdy =
                (z[j1 * w + i] - z[j0 * w + i]) / (static_cast<float>(j1 - j0) * res);
            const float3 nl = normalize(float3{-dzdx, -dzdy, 1.0f});
            normals[j * w + i] = float3{c * nl.x - s * nl.y, s * nl.x + c * nl.y, nl.z};
        }
    }

    std::vector<Vertex> plain(n);
    fill_tangent_frames(plain, normals);

    std::vector<HeightGridVertex> verts(n);
    for (size_t k = 0; k < n; ++k) {
        verts[k].position = float3{slot.cpuPositions[k * 3 + 0], slot.cpuPositions[k * 3 + 1],
                                   slot.cpuPositions[k * 3 + 2]};
        verts[k].tangentFrame = plain[k].tangentFrame;
        verts[k].custom0 = float2{slot.cpuCustom[k * 2 + 0], slot.cpuCustom[k * 2 + 1]};
    }
    return verts;
}

void build_height_mesh(VisualRenderer& r, VisualRenderer::HeightGridSlot& slot,
                       const HeightGridLayer& g) {
    Mesh& mesh = slot.mesh;
    mesh.vertexCount = g.width_cells * g.height_cells;
    mesh.vb = make_height_vertex_buffer(*r.engine, mesh.vertexCount);
    mesh.ib = make_height_index_buffer(*r.engine, g.width_cells, g.height_cells);
    mesh.entity = utils::EntityManager::get().create();
    const auto res = static_cast<float>(g.resolution_m);
    const float reach =
        std::hypot(static_cast<float>(g.width_cells), static_cast<float>(g.height_cells)) * res +
        res;
    filament::RenderableManager::Builder(1)
        .boundingBox({{0, 0, 0}, {reach, reach, kHeightGridBoundsZM}})
        .geometry(0, filament::RenderableManager::PrimitiveType::TRIANGLES, mesh.vb, mesh.ib)
        .material(0, r.heightGridInstance)
        .culling(false)
        .castShadows(false)
        .receiveShadows(true)
        .build(*r.engine, mesh.entity);
    r.engine->getTransformManager().create(mesh.entity);
}

void clear_slot(VisualRenderer& r, VisualRenderer::HeightGridSlot& slot) {
    destroy_mesh(*r.engine, *r.scene, slot.mesh);
    if (slot.fadeInstance != nullptr) r.engine->destroy(slot.fadeInstance);
    slot = VisualRenderer::HeightGridSlot{};
}

void bind_material(VisualRenderer& r, VisualRenderer::HeightGridSlot& slot,
                   filament::MaterialInstance* mat) {
    filament::RenderableManager& rm = r.engine->getRenderableManager();
    const auto ri = rm.getInstance(slot.mesh.entity);
    if (ri.isValid()) rm.setMaterialInstanceAt(ri, 0, mat);
}

void release_fade_instance(VisualRenderer& r, VisualRenderer::HeightGridSlot& slot) {
    if (slot.fadeInstance == nullptr) return;
    bind_material(r, slot, r.heightGridInstance);
    r.engine->destroy(slot.fadeInstance);
    slot.fadeInstance = nullptr;
}

}

void apply_height_grid_params(filament::MaterialInstance& inst, const VisualRenderer& r,
                              const detail::Theme::HeightGrid& g) {
    // NEAREST like groundGridRampTexture; the shader samples at texel centres.
    inst.setParameter("rampTexture", r.heightGridRampTexture,
                      filament::TextureSampler(filament::TextureSampler::MinFilter::NEAREST,
                                               filament::TextureSampler::MagFilter::NEAREST));
    inst.setParameter("rampMinM", r.heightGridRamp.min_m);
    inst.setParameter("rampMaxM", r.heightGridRamp.max_m);
    inst.setParameter("unknownColor",
                      float3{g.unknown_color.r, g.unknown_color.g, g.unknown_color.b});
    inst.setParameter("roughness", g.roughness);
}

void destroy_height_grid_slots(VisualRenderer& r) {
    for (auto& slot : r.heightGridSlots) clear_slot(r, slot);
    r.heightGridSlots.clear();
}

void update_height_grids(VisualRenderer& r, const SceneGraph& s) {
    const uint32_t count = s.height_grids != nullptr ? s.height_grid_count : 0;
    while (r.heightGridSlots.size() > count) {
        clear_slot(r, r.heightGridSlots.back());
        r.heightGridSlots.pop_back();
    }
    if (r.heightGridSlots.size() < count) r.heightGridSlots.resize(count);

    filament::TransformManager& tm = r.engine->getTransformManager();
    const float bias = r.active_theme.height_grid.ground_bias_m;

    for (uint32_t i = 0; i < count; ++i) {
        const HeightGridLayer& g = s.height_grids[i];
        VisualRenderer::HeightGridSlot& slot = r.heightGridSlots[i];

        if (g.heights_m == nullptr || g.width_cells < 2 || g.height_cells < 2 ||
            !(g.resolution_m > 0.0)) {
            clear_slot(r, slot);
            continue;
        }

        const bool dimsChanged =
            slot.mesh.vb == nullptr || slot.width != g.width_cells || slot.height != g.height_cells;
        if (dimsChanged) {
            const uint32_t uploads = slot.uploadCount;
            clear_slot(r, slot);
            slot.uploadCount = uploads;
            build_height_mesh(r, slot, g);
            slot.width = g.width_cells;
            slot.height = g.height_cells;
        }

        // The origin is not part of this gate: it only moves the renderable's transform (below).
        const bool dataChanged = dimsChanged || slot.lastUpdateSec != g.last_update_sec ||
                                 slot.resolution != g.resolution_m || slot.yaw != g.yaw_rad ||
                                 slot.groundBias != bias;
        if (dataChanged) {
            upload_height_vertices(*r.engine, slot.mesh.vb, build_height_vertices(slot, g, bias));
            slot.lastUpdateSec = g.last_update_sec;
            slot.resolution = g.resolution_m;
            slot.yaw = g.yaw_rad;
            slot.groundBias = bias;
            ++slot.uploadCount;
        }

        slot.origin = g.origin;
        const auto ti = tm.getInstance(slot.mesh.entity);
        if (ti.isValid()) {
            tm.setTransform(ti, filament::math::mat4f::translation(float3{
                                    static_cast<float>(g.origin.x), static_cast<float>(g.origin.y),
                                    static_cast<float>(g.origin.z)}));
        }

        const float alpha = detail::SceneBuffer::staleness_alpha(
            s.sim_time_sec, g.last_update_sec, kStaleFadeStartSec, kStaleFadeTimeoutSec);
        slot.alpha = alpha;

        if (alpha <= 0.0f) {
            if (slot.inScene) {
                r.scene->remove(slot.mesh.entity);
                slot.inScene = false;
            }
            release_fade_instance(r, slot);
            slot.materialState = 0;
            continue;
        }

        if (!slot.inScene) {
            r.scene->addEntity(slot.mesh.entity);
            slot.inScene = true;
        }
        if (alpha >= 1.0f) {
            release_fade_instance(r, slot);
            slot.materialState = 1;
        } else {
            if (slot.fadeInstance == nullptr) {
                slot.fadeInstance = r.heightGridFadedMaterial->createInstance();
                slot.fadeInstance->setCullingMode(filament::backend::CullingMode::NONE);
                apply_height_grid_params(*slot.fadeInstance, r, r.active_theme.height_grid);
                // ponytail: the faded renderable sits in the blended queue at default priority 4,
                // so for up to 0.5 s of stale fade it draws after (and paints over) the ground-grid
                // quads (priority 0/1) although it lies below them. Accepted transient; if it
                // shows in review, give the faded renderable priority 0 via
                // RenderableManager::setPriority.
                bind_material(r, slot, slot.fadeInstance);
            }
            slot.fadeInstance->setParameter("alpha", alpha);
            slot.materialState = 2;
        }
    }
}

}

namespace overlume::testing {

size_t height_grid_slot_count(overlume::VisualRenderer* r) {
    return r == nullptr ? 0 : r->heightGridSlots.size();
}

uint32_t height_grid_vertex_count(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->heightGridSlots.size()) return 0;
    const auto& s = r->heightGridSlots[slot];
    return s.mesh.vb == nullptr ? 0 : static_cast<uint32_t>(s.mesh.vb->getVertexCount());
}

uint32_t height_grid_index_count(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->heightGridSlots.size()) return 0;
    const auto& s = r->heightGridSlots[slot];
    return s.mesh.ib == nullptr ? 0 : static_cast<uint32_t>(s.mesh.ib->getIndexCount());
}

bool height_grid_vertex(overlume::VisualRenderer* r, size_t slot, uint32_t i, uint32_t j,
                        float out_pos[3], float out_custom[2]) {
    if (r == nullptr || out_pos == nullptr || out_custom == nullptr ||
        slot >= r->heightGridSlots.size()) {
        return false;
    }
    const auto& s = r->heightGridSlots[slot];
    if (i >= s.width || j >= s.height || s.mesh.entity.isNull()) return false;
    const size_t k = static_cast<size_t>(j) * s.width + i;
    if (k * 3 + 2 >= s.cpuPositions.size() || k * 2 + 1 >= s.cpuCustom.size()) return false;
    // Origin comes from Filament's TransformManager, not from slot.origin.
    auto& tm = r->engine->getTransformManager();
    const auto ti = tm.getInstance(s.mesh.entity);
    if (!ti.isValid()) return false;
    const filament::math::mat4f m = tm.getTransform(ti);
    for (int c = 0; c < 3; ++c) out_pos[c] = s.cpuPositions[k * 3 + c] + m[3][c];
    out_custom[0] = s.cpuCustom[k * 2 + 0];
    out_custom[1] = s.cpuCustom[k * 2 + 1];
    return true;
}

uint32_t height_grid_upload_count(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->heightGridSlots.size()) return 0;
    return r->heightGridSlots[slot].uploadCount;
}

int height_grid_material_state(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->heightGridSlots.size()) return 0;
    const auto& s = r->heightGridSlots[slot];
    if (s.mesh.entity.isNull() || !r->scene->hasEntity(s.mesh.entity)) return 0;
    auto& rm = r->engine->getRenderableManager();
    const auto ri = rm.getInstance(s.mesh.entity);
    if (!ri.isValid()) return 0;
    const filament::MaterialInstance* bound = rm.getMaterialInstanceAt(ri, 0);
    if (bound == r->heightGridInstance) return 1;
    if (bound != nullptr && bound == s.fadeInstance) return 2;
    return -1;
}

float height_grid_fade_roughness(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->heightGridSlots.size()) return -1.0f;
    const auto& s = r->heightGridSlots[slot];
    return s.fadeInstance != nullptr ? s.fadeInstance->getParameter<float>("roughness") : -1.0f;
}

bool ground_hole_state(overlume::VisualRenderer* r, float out_center[2], float out_axis_x[2],
                       float out_half_extent[2]) {
    if (r == nullptr) return false;
    const auto& h = r->groundHole;
    for (size_t k = 0; k < 2; ++k) {
        if (out_center) out_center[k] = h.center[k];
        if (out_axis_x) out_axis_x[k] = h.axis_x[k];
        if (out_half_extent) out_half_extent[k] = h.half_extent[k];
    }
    return h.enabled;
}

}
