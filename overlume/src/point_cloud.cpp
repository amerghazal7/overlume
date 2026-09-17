// point_cloud.cpp — point clouds (Epic 3 Task 6 / VM-035). Its own vertex
// layout (position + packed rgba8, no tangent frame -- points have no
// meaningful normal) and its own material (point_cloud.mat, UNLIT,
// requires:[color], blending:fade) -- Epic 2's "one material, one job"
// precedent extended to a fourth. Chunking reuses polyline.hpp's
// kMaxPointsPerMesh/polyline_chunks unmodified (POINTS primitive: 1 vertex
// per point, no 2x multiplier).
//
// KNOWN LIMITATION, stated honestly: polyline_chunks()/kMaxPointsPerMesh
// were built for extrude_polyline()'s "at least 2 points make a segment"
// world, so polyline_chunks(1) returns empty and a genuinely 1-point cloud
// renders nothing. Reused as-is per this task's own decision (a new
// points-only chunker for one degenerate-size edge case was rejected as
// not worth a second ceiling constant); a real lidar/depth-cam publisher
// never emits a 1-point message.
#include "point_cloud.hpp"
#include "point_cloud_test_hooks.hpp"
#include "polyline.hpp"
#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

#include <filament/RenderableManager.h>

#include <math/vec3.h>

#include <utils/EntityManager.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

namespace mpviz {

namespace {

using filament::math::float3;

// Generous half-extent for a point cloud's declared culling box -- lidar
// ranges routinely exceed the 60m ground/lane geometry uses elsewhere in
// this library. culling(false) below (same convention as every other
// category) makes this a formality rather than a real cull bound.
constexpr float kPointCloudBoundsM = 200.0f;

// One point-cloud vertex: world-space position + packed rgba8
// (scene.h's PointCloudPoint::rgba, same byte convention -- byte0=r,
// byte1=g, byte2=b, byte3=a). No tangent frame: UNLIT, points have no
// meaningful normal.
struct PointVertex {
    float3 position;
    uint32_t rgba;
};

filament::VertexBuffer* make_point_vertex_buffer(filament::Engine& engine,
                                                  std::vector<PointVertex> verts) {
    auto* heap = new std::vector<PointVertex>(std::move(verts));
    filament::VertexBuffer* vb =
        filament::VertexBuffer::Builder()
            .vertexCount(static_cast<uint32_t>(heap->size()))
            .bufferCount(1)
            .attribute(filament::VertexAttribute::POSITION, 0,
                       filament::VertexBuffer::AttributeType::FLOAT3,
                       offsetof(PointVertex, position), sizeof(PointVertex))
            // UBYTE4 + normalized(): the raw rgba8 bytes map straight into
            // getColor() as [0,1] floats in point_cloud.mat's fragment
            // shader -- no repacking at this call site.
            .attribute(filament::VertexAttribute::COLOR, 0,
                       filament::VertexBuffer::AttributeType::UBYTE4,
                       offsetof(PointVertex, rgba), sizeof(PointVertex))
            .normalized(filament::VertexAttribute::COLOR)
            .build(engine);
    vb->setBufferAt(
        engine, 0,
        filament::VertexBuffer::BufferDescriptor(
            heap->data(), heap->size() * sizeof(PointVertex),
            [](void*, size_t, void* user) { delete static_cast<std::vector<PointVertex>*>(user); },
            heap));
    return vb;
}

uint64_t hash_combine(uint64_t seed, uint64_t v) {
    return seed ^ (v + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2));
}

uint64_t hash_point(const PointCloudPoint& p) {
    uint64_t h = std::hash<double>{}(p.position.x);
    h = hash_combine(h, std::hash<double>{}(p.position.y));
    h = hash_combine(h, std::hash<double>{}(p.position.z));
    h = hash_combine(h, static_cast<uint64_t>(p.rgba));
    return h;
}

// Content signature: point count + first/last point (rgba included) --
// same "cheap, good enough" shape as ribbon.cpp's ribbon_signature()/
// map_elements.cpp's own signature helpers.
uint64_t point_cloud_signature(const PointCloudPoint* pts, uint32_t n) {
    uint64_t h = hash_combine(0, static_cast<uint64_t>(n));
    if (n > 0) {
        h = hash_combine(h, hash_point(pts[0]));
        h = hash_combine(h, hash_point(pts[n - 1]));
    }
    return h;
}

void destroy_slot_meshes(VisualRenderer& r, VisualRenderer::PointCloudSlot& slot) {
    for (auto& m : slot.meshes) destroy_mesh(*r.engine, *r.scene, m);
    slot.meshes.clear();
}

// Unpacks PointCloudPoint::rgba per scene.h's documented convention
// (byte0=r, byte1=g, byte2=b, byte3=a) and substitutes the theme's neutral
// token whenever alpha==0 -- the "no real per-point color was computed"
// sentinel (color_mode: flat), reusing palette.object_tints.unknown, the
// SAME token GenericMarker's own alpha==0 "theme-neutral default" already
// uses (zero new theme fields). a!=0 (always 255 when the adapter bakes a
// real color) passes r/g/b through verbatim.
uint32_t resolve_rgba(const VisualRenderer& r, uint32_t packed) {
    const uint32_t a = (packed >> 24) & 0xFFu;
    if (a != 0) return packed;
    const detail::Float3& tint = r.active_theme.palette.object_tints.unknown;
    const auto to_byte = [](float c) {
        return static_cast<uint32_t>(std::clamp(c, 0.0f, 1.0f) * 255.0f + 0.5f);
    };
    return to_byte(tint.r) | (to_byte(tint.g) << 8) | (to_byte(tint.b) << 16) | (255u << 24);
}

// Rebuilds `slot`'s geometry from `pts`/`n`, chunked at polyline_chunks()'s
// ceiling -- each chunk becomes its own POINTS-primitive Mesh, all bound to
// the ONE shared pointCloudMaterialInstance (Step 2's decision: a layer
// that fades as one unit needs no per-chunk instancing).
void build_slot_meshes(VisualRenderer& r, VisualRenderer::PointCloudSlot& slot,
                        const PointCloudPoint* pts, uint32_t n) {
    destroy_slot_meshes(r, slot);
    slot.totalVertexCount = 0;
    for (auto [a, b] : detail::polyline_chunks(n)) {
        const uint32_t chunkN = b - a;
        std::vector<PointVertex> verts(chunkN);
        std::vector<uint16_t> indices(chunkN);
        for (uint32_t i = 0; i < chunkN; ++i) {
            const PointCloudPoint& p = pts[a + i];
            verts[i].position = float3{static_cast<float>(p.position.x),
                                        static_cast<float>(p.position.y),
                                        static_cast<float>(p.position.z)};
            verts[i].rgba = resolve_rgba(r, p.rgba);
            indices[i] = static_cast<uint16_t>(i);  // 1 vertex/point -- sequential, uint16-safe
                                                     // (chunkN <= kMaxPointsPerMesh < 65535)
        }
        slot.totalVertexCount += chunkN;
        Mesh mesh;
        mesh.vertexCount = chunkN;
        mesh.vb = make_point_vertex_buffer(*r.engine, std::move(verts));
        mesh.ib = make_index_buffer(*r.engine, std::move(indices));
        mesh.entity = utils::EntityManager::get().create();
        filament::RenderableManager::Builder(1)
            .boundingBox({{0, 0, 0}, {kPointCloudBoundsM, kPointCloudBoundsM, kPointCloudBoundsM}})
            .geometry(0, filament::RenderableManager::PrimitiveType::POINTS, mesh.vb, mesh.ib)
            .material(0, r.pointCloudMaterialInstance)
            .culling(false)
            .castShadows(false)
            .receiveShadows(false)
            .build(*r.engine, mesh.entity);
        r.scene->addEntity(mesh.entity);
        slot.meshes.push_back(std::move(mesh));
    }
}

}  // namespace

void update_point_clouds(VisualRenderer& r, const SceneGraph& s) {
    while (r.pointCloudSlots.size() > s.point_cloud_count) {
        destroy_slot_meshes(r, r.pointCloudSlots.back());
        r.pointCloudSlots.pop_back();
    }
    if (r.pointCloudSlots.size() < s.point_cloud_count) {
        r.pointCloudSlots.resize(s.point_cloud_count);
    }

    // ONE MaterialInstance, ONE alpha: with more than one live cloud this
    // frame, the freshest of them wins (max, not last-processed) -- an
    // explicit, stated choice for the N>1 case (today's shipped profiles
    // carry no live PointCloud2 row at all -- FIXTURE GAP, see the
    // adapter's own header comment -- so this never fires in practice yet).
    float alpha = s.point_cloud_count > 0 ? 0.0f : 1.0f;
    for (uint32_t i = 0; i < s.point_cloud_count; ++i) {
        const PointCloud& pc = s.point_clouds[i];
        VisualRenderer::PointCloudSlot& slot = r.pointCloudSlots[i];
        const uint64_t sig = point_cloud_signature(pc.points, pc.point_count);
        if (!slot.has_signature || slot.signature != sig) {
            build_slot_meshes(r, slot, pc.points, pc.point_count);
            slot.signature = sig;
            slot.has_signature = true;
        }
        alpha = std::max(alpha, static_cast<float>(detail::SceneBuffer::staleness_alpha(
                                     s.sim_time_sec, pc.last_update_sec, kStaleFadeStartSec,
                                     kStaleFadeTimeoutSec)));
    }
    r.pointCloudMaterialInstance->setParameter("alpha", alpha);
    // Live (mid-transition-aware) theme read, same per-frame convention as
    // alpha above -- one uniform on the one shared instance, no rebuild.
    r.pointCloudMaterialInstance->setParameter("pointSizePx",
                                               r.active_theme.point_cloud.point_size_px);
    r.pointCloudAlpha = alpha;
}

}  // namespace mpviz

// Filament-free test introspection hooks; see point_cloud_test_hooks.hpp
// for why these live here.
namespace mpviz::testing {

size_t point_cloud_mesh_count(mpviz::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->pointCloudSlots.size()) return 0;
    return r->pointCloudSlots[slot].meshes.size();
}

size_t point_cloud_vertex_count(mpviz::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->pointCloudSlots.size()) return 0;
    return r->pointCloudSlots[slot].totalVertexCount;
}

float point_cloud_material_alpha(mpviz::VisualRenderer* r) {
    if (r == nullptr) return 1.0f;
    return r->pointCloudAlpha;
}

}  // namespace mpviz::testing
