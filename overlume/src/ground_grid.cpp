// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// ground_grid.cpp — OGM occupancy grids as theme-colored ground textures
// with in-place partial updates. The first filament::Texture in this
// library (see build_occupancy_texture() below).
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

// kUnknownCell = 255 -- mirrors overlume_node::OgmAdapter::kUnknownCell
// (ogm.hpp, the source of truth). The frozen POD boundary
// (GroundGridLayer::cells is a bare uint8_t*) can't carry a shared
// constant across the ABI, so this number is stated by comment in three
// places: ogm.hpp, here, and ground_grid.mat's header -- if that value
// moves, all three must move together. The actual `v > 100.0 -> alpha 0`
// check lives in ground_grid.mat's fragment shader, not in this .cpp;
// nothing here reads a raw cell byte, only the width*height byte count.

// dynamic OGM sits above gradient OGM but below every paint/ribbon layer:
// ground patch 0, lane paint 0.02, object predicted-paths 0.03, path
// ribbons 0.04. An OGM layer is ground shading, not a foreground
// primitive, so it must render under all of those.
constexpr float kGradientZLiftM = 0.010f;
constexpr float kDynamicZLiftM = 0.015f;

float z_lift_for_kind(uint8_t kind) { return kind == 0 ? kDynamicZLiftM : kGradientZLiftM; }

// One ground-grid quad vertex: position + tangent frame (for LIT shading)
// + a UV0 attribute -- ground_grid.mat is the first material in this
// library that needs one, so this is its own dedicated vertex layout, not
// a growth of the shared position+tangent `Vertex` type.
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

// Builds one quad, sized width_cells*resolution_m x height_cells*
// resolution_m, with its (0,0) corner at `g.origin` (GroundGridLayer's
// contract: "map-frame position of cell (0,0)") lifted `z_lift` above the
// ground patch. Baked directly in absolute map-frame world space -- same
// convention as map_elements.cpp/ribbon.cpp, unlike the ego-following
// ground/grid patch (which moves via a TransformManager transform
// instead). UV (0,0)..(1,1) across the quad, consistent with the
// row-major cell upload in upload_occupancy_texture() below.
void build_ground_grid_quad(VisualRenderer& r, Mesh& mesh, const GroundGridLayer& g, float z_lift,
                            filament::MaterialInstance* material) {
    const float ox = static_cast<float>(g.origin.x);
    const float oy = static_cast<float>(g.origin.y);
    const float oz = static_cast<float>(g.origin.z) + z_lift;
    const float w = static_cast<float>(g.width_cells) * static_cast<float>(g.resolution_m);
    const float h = static_cast<float>(g.height_cells) * static_cast<float>(g.resolution_m);

    std::vector<GroundGridVertex> verts = {
        {{ox, oy, oz}, {}, {0.0f, 0.0f}},
        {{ox + w, oy, oz}, {}, {1.0f, 0.0f}},
        {{ox + w, oy + h, oz}, {}, {1.0f, 1.0f}},
        {{ox, oy + h, oz}, {}, {0.0f, 1.0f}},
    };
    std::vector<uint16_t> indices = {0, 1, 2, 0, 2, 3};

    // fill_tangent_frames() (renderer_internal.hpp) operates on the shared
    // `Vertex` type -- build a throwaway plain-Vertex array purely to get
    // its orientation quats, same trick map_elements.cpp/renderer.cpp use
    // for their own extended vertex layouts.
    std::vector<Vertex> plain(verts.size());
    for (size_t i = 0; i < verts.size(); ++i) plain[i].position = verts[i].position;
    fill_tangent_frames(plain, std::vector<float3>(verts.size(), float3{0.0f, 0.0f, 1.0f}));
    for (size_t i = 0; i < verts.size(); ++i) verts[i].tangentFrame = plain[i].tangentFrame;

    mesh.vb = make_ground_grid_vertex_buffer(*r.engine, std::move(verts));
    mesh.ib = make_index_buffer(*r.engine, std::move(indices));
    mesh.entity = utils::EntityManager::get().create();
    const float halfW = w * 0.5f, halfH = h * 0.5f;
    filament::RenderableManager::Builder(1)
        .boundingBox({{0, 0, 0}, {halfW + std::abs(ox) + 1.0f, halfH + std::abs(oy) + 1.0f, 1.0f}})
        .geometry(0, filament::RenderableManager::PrimitiveType::TRIANGLES, mesh.vb, mesh.ib)
        .material(0, material)
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

// Uploads `cells` (already the width*height, row-major, 0..100|255-sentinel
// bytes OgmAdapter produces -- see kUnknownCell above) into `tex` via
// setImage -- the same Texture* is reused across calls whenever dims
// haven't changed (update_ground_grids() owns that decision), only the
// pixel data moves. update_ground_grids() also gates whether this runs at
// all on GroundGridSlot::last_upload_sec vs. g.last_update_sec.
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

}  // namespace

void update_ground_grids(VisualRenderer& r, const SceneGraph& s) {
    // Release slots >= grid_count.
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

        // Malformed guard: no cell data, or a degenerate 0-sized grid --
        // leave the slot as it was.
        if (g.cells == nullptr || g.width_cells == 0 || g.height_cells == 0) continue;

        const uint8_t kind =
            g.kind < VisualRenderer::kGroundGridKindCount ? g.kind : static_cast<uint8_t>(0);

        // Geometry rebuild: an OGM's origin/dims/resolution are effectively
        // static after the node adapter's first full grid (a partial
        // _updates patch never resizes, see ogm.hpp), so a plain
        // field-equality check is enough; no content-hash signature needed
        // (unlike map_elements.cpp/ribbon.cpp, whose source topics are
        // rolling windows with genuinely changing point data).
        const bool geomChanged =
            !slot.has_geometry || slot.kind != kind || slot.width_cells != g.width_cells ||
            slot.height_cells != g.height_cells || slot.resolution_m != g.resolution_m ||
            slot.origin.x != g.origin.x || slot.origin.y != g.origin.y ||
            slot.origin.z != g.origin.z;
        if (geomChanged) {
            destroy_mesh(*r.engine, *r.scene, slot.quad);
            build_ground_grid_quad(r, slot.quad, g, z_lift_for_kind(kind),
                                   r.groundGridMaterialInstance[kind]);
            slot.kind = kind;
            slot.width_cells = g.width_cells;
            slot.height_cells = g.height_cells;
            slot.resolution_m = g.resolution_m;
            slot.origin = g.origin;
            slot.has_geometry = true;
        }

        // Texture: dims changed (or no texture yet) -> destroy + recreate,
        // never leaked. Same dims -> reuse the same Texture* and just
        // re-upload -- the TextureIsUpdatedInPlaceNotRecreated contract.
        const bool dimsChanged = slot.texture == nullptr || slot.texWidth != g.width_cells ||
                                 slot.texHeight != g.height_cells;
        if (dimsChanged) {
            if (slot.texture != nullptr) r.engine->destroy(slot.texture);
            slot.texture = build_occupancy_texture(*r.engine, g.width_cells, g.height_cells);
            slot.texWidth = g.width_cells;
            slot.texHeight = g.height_cells;
            ++slot.textureGeneration;
        }

        // Upload gate: only touch the GPU texture on an actual content
        // change -- a new dims/geometry build (a slot re-homed to a
        // different grid at identical dims must never inherit the
        // previous occupant's stale pixels) or a genuinely new
        // ingest()/ingest_update() since the last upload (last_update_sec
        // advances once per accepted message). Without this, every live
        // grid re-uploads its full byte buffer every frame regardless of
        // whether anything changed.
        if (dimsChanged || geomChanged || slot.last_upload_sec != g.last_update_sec) {
            upload_occupancy_texture(*r.engine, slot.texture, g.cells, g.width_cells,
                                     g.height_cells);
            slot.last_upload_sec = g.last_update_sec;
            ++slot.uploadCount;
        }

        filament::MaterialInstance* mat = r.groundGridMaterialInstance[kind];
        // NEAREST, not the Builder default LINEAR/LINEAR: bilinear-
        // filtering an 8-bit occupancy byte would blend real occupancy
        // values with the 255 unknown sentinel at cell boundaries,
        // manufacturing fractional "occupancy" the adapter never reported.
        mat->setParameter("occupancyTexture", slot.texture,
                          filament::TextureSampler(filament::TextureSampler::MinFilter::NEAREST,
                                                   filament::TextureSampler::MagFilter::NEAREST));

        const auto alpha = static_cast<float>(detail::SceneBuffer::staleness_alpha(
            s.sim_time_sec, g.last_update_sec, kStaleFadeStartSec, kStaleFadeTimeoutSec));
        mat->setParameter("alpha", alpha);
        r.groundGridAlpha[kind] = alpha;
    }
}

}  // namespace overlume

// Filament-free test introspection hooks; see ground_grid_test_hooks.hpp
// for why these live here.
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

float ground_grid_material_alpha(overlume::VisualRenderer* r, uint8_t kind) {
    if (r == nullptr || kind >= overlume::VisualRenderer::kGroundGridKindCount) return 1.0f;
    return r->groundGridAlpha[kind];
}

overlume::detail::Float3 ground_grid_free_color(overlume::VisualRenderer* r) {
    if (r == nullptr) return {};
    return r->groundGridFreeColor;
}

overlume::detail::Float3 ground_grid_occupied_color(overlume::VisualRenderer* r) {
    if (r == nullptr) return {};
    return r->groundGridOccupiedColor;
}

}  // namespace overlume::testing
