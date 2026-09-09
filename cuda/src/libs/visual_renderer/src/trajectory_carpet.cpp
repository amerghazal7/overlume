// trajectory_carpet.cpp — output_trajectory_carpet (VM-077). A per-vertex
// velocity-colored trajectory ribbon: TRIANGLE_LIST, always a multiple of 3,
// genuine per-vertex color (colors[] on the wire, r/g vary, b≡0, a≡0.70 --
// see the VM-077 measurement report). Mirrors point_cloud.cpp's own
// PointVertex/COLOR-attribute/UNLIT/blending:fade machinery exactly, minus
// pointSizePx (meaningless for triangles): same vertex layout
// (mpviz::PointCloudPoint, reused verbatim per D1), chunked at the same
// per-mesh vertex ceiling as polyline.hpp's kMaxPointsPerMesh but with a
// file-local triangle-aligned chunker (see triangle_chunks() below) --
// polyline_chunks() itself is LINE_STRIP-shaped (it deliberately overlaps
// consecutive chunks by one shared join vertex) and corrupts a TRIANGLES
// vertex list at every seam, so it is NOT reused here despite the shared
// ceiling constant. Same one shared MaterialInstance whose "alpha"
// parameter carries the staleness fade. The ONE difference from
// point_cloud.cpp: PrimitiveType::TRIANGLES instead of POINTS, and a plain
// sequential index buffer (indices[i] = i) since the wire data is already
// ordered triangles -- no fan/strip reinterpretation needed.
#include "trajectory_carpet.hpp"
#include "trajectory_carpet_test_hooks.hpp"
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

// Same generous half-extent reasoning as point_cloud.cpp's
// kPointCloudBoundsM -- a trajectory carpet can run the length of a
// planning horizon, well past the 60m ground/lane geometry elsewhere.
// culling(false) below makes this a formality, same convention as every
// other category.
constexpr float kTrajectoryCarpetBoundsM = 200.0f;

// Z-lift: above every path ribbon (ribbon.cpp's kRibbonZLiftByRoleM tops
// out at 0.05) but strictly BELOW alert_polygons.cpp's kAlertZLiftM (0.06,
// topmost) -- collision rings on the ego's own corridor routinely overlap
// the carpet footprint, and a translucent carpet at the same depth would
// visually merge with them. The adapter flattens z to 0.0; the lift is
// applied HERE like every other flattened category (coplanar-with-ground
// geometry loses the depth test and renders nothing). History: plan
// 2026-09-09-vm077-new-stack-rendering.md Task 2.
constexpr float kTrajectoryCarpetZLiftM = 0.055f;

// Measured producer opacity: every ADD marker on output_trajectory_carpet
// carries m.color.a == 0.7 (1300/1300 in stack_v2_fixtures, 1647/1647 in
// stack_v2_full_sensors) -- rviz parity takes TriangleList material alpha
// from marker.color.a. Multiplied into the staleness `alpha` uniform below.
// ponytail: fixed at the measured 0.7; honouring m.color.a per message
// needs a scene.h field + kSceneVersion bump.
constexpr float kTrajectoryCarpetBaseAlpha = 0.7f;

// One trajectory-carpet vertex: world-space position + packed rgba8, EXACTLY
// point_cloud.cpp's PointVertex layout (D1: PointCloudPoint reused verbatim
// as the wire vertex type, so the GPU-side vertex struct matches too -- a
// second, byte-identical struct only exists here because point_cloud.cpp's
// is file-local (anonymous namespace), not because the layout differs).
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
                       filament::VertexBuffer::AttributeType::UBYTE4,
                       offsetof(CarpetVertex, rgba), sizeof(CarpetVertex))
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

uint64_t hash_point(const PointCloudPoint& p) {
    uint64_t h = std::hash<double>{}(p.position.x);
    h = hash_combine(h, std::hash<double>{}(p.position.y));
    h = hash_combine(h, std::hash<double>{}(p.position.z));
    h = hash_combine(h, static_cast<uint64_t>(p.rgba));
    return h;
}

// Content signature: point count + first/last point (rgba included) -- same
// "cheap, good enough" shape as point_cloud.cpp's point_cloud_signature().
uint64_t trajectory_carpet_signature(const PointCloudPoint* pts, uint32_t n) {
    uint64_t h = hash_combine(0, static_cast<uint64_t>(n));
    if (n > 0) {
        h = hash_combine(h, hash_point(pts[0]));
        h = hash_combine(h, hash_point(pts[n - 1]));
    }
    return h;
}

// Triangle-aligned chunk ranges for a TRIANGLES-primitive vertex list of
// `n` vertices. polyline.hpp's polyline_chunks() cannot be reused here: it
// is built for LINE_STRIPs, where consecutive chunks must share one
// overlapping join vertex, and 32000 (its ceiling) is not a multiple of 3
// -- applied to a flat triangle list, both of those corrupt geometry at
// the seam (a dropped/incomplete triangle, then every triangle after it
// stitched from three different source triangles). A triangle list has no
// shared vertex between chunks, so each range here is a whole number of
// triangles with NO overlap, stepping by kMaxTrianglePointsPerMesh
// (polyline.hpp's kMaxPointsPerMesh rounded down to a multiple of 3 --
// still comfortably under the uint16_t index ceiling per mesh). n==0 ->
// empty.
constexpr uint32_t kMaxTrianglePointsPerMesh = (detail::kMaxPointsPerMesh / 3) * 3;  // 31998

std::vector<std::pair<uint32_t, uint32_t>> triangle_chunks(uint32_t n) {
    std::vector<std::pair<uint32_t, uint32_t>> chunks;
    uint32_t start = 0;
    while (start < n) {
        const uint32_t end = std::min(start + kMaxTrianglePointsPerMesh, n);
        chunks.emplace_back(start, end);
        start = end;
    }
    return chunks;
}

void destroy_slot_meshes(VisualRenderer& r, VisualRenderer::TrajectoryCarpetSlot& slot) {
    for (auto& m : slot.meshes) destroy_mesh(*r.engine, *r.scene, m);
    slot.meshes.clear();
    slot.firstMeshRgba.clear();
    slot.firstMeshZ.clear();
}

// Unpacks PointCloudPoint::rgba per scene.h's documented convention and
// substitutes the theme's neutral token whenever alpha==0 -- IDENTICAL
// sentinel rule to point_cloud.cpp's own resolve_rgba() (D1/Task 2 Step 1:
// "duplicate the 3-line helper, promote to a shared header if a third
// caller ever needs it" -- ponytail, this is the second caller).
uint32_t resolve_rgba(const VisualRenderer& r, uint32_t packed) {
    const uint32_t a = (packed >> 24) & 0xFFu;
    if (a != 0) return packed;
    const detail::Float3& tint = r.active_theme.palette.object_tints.unknown;
    const auto to_byte = [](float c) {
        return static_cast<uint32_t>(std::clamp(c, 0.0f, 1.0f) * 255.0f + 0.5f);
    };
    return to_byte(tint.r) | (to_byte(tint.g) << 8) | (to_byte(tint.b) << 16) | (255u << 24);
}

// Rebuilds `slot`'s geometry from `pts`/`n`, chunked at triangle_chunks()'s
// triangle-aligned, non-overlapping ceiling -- each chunk becomes its own
// TRIANGLES-primitive Mesh (sequential indices: the wire data is already
// ordered triangles, no fan/strip reinterpretation), all bound to the ONE
// shared trajectoryCarpetMaterialInstance.
void build_slot_meshes(VisualRenderer& r, VisualRenderer::TrajectoryCarpetSlot& slot,
                        const PointCloudPoint* pts, uint32_t n) {
    destroy_slot_meshes(r, slot);
    slot.totalVertexCount = 0;
    bool first_chunk = true;
    for (auto [a, b] : triangle_chunks(n)) {
        const uint32_t chunkN = b - a;
        std::vector<CarpetVertex> verts(chunkN);
        std::vector<uint16_t> indices(chunkN);
        for (uint32_t i = 0; i < chunkN; ++i) {
            const PointCloudPoint& p = pts[a + i];
            verts[i].position = float3{static_cast<float>(p.position.x),
                                        static_cast<float>(p.position.y),
                                        static_cast<float>(p.position.z) + kTrajectoryCarpetZLiftM};
            verts[i].rgba = resolve_rgba(r, p.rgba);
            indices[i] = static_cast<uint16_t>(i);  // sequential -- already-ordered triangles
                                                     // (chunkN <= kMaxTrianglePointsPerMesh < 65535)
        }
        if (first_chunk) {
            // Test-hook mirror only (trajectory_carpet_test_hooks.hpp) --
            // NOT a Filament read-back, same reasoning as RibbonSlot's
            // firstPointM/halfWidthM (renderer_internal.hpp).
            slot.firstMeshRgba.reserve(chunkN);
            slot.firstMeshZ.reserve(chunkN);
            for (const auto& v : verts) {
                slot.firstMeshRgba.push_back(v.rgba);
                slot.firstMeshZ.push_back(v.position.z);
            }
            first_chunk = false;
        }
        slot.totalVertexCount += chunkN;
        Mesh mesh;
        mesh.vertexCount = chunkN;
        mesh.vb = make_carpet_vertex_buffer(*r.engine, std::move(verts));
        mesh.ib = make_index_buffer(*r.engine, std::move(indices));
        mesh.entity = utils::EntityManager::get().create();
        filament::RenderableManager::Builder(1)
            .boundingBox({{0, 0, 0},
                          {kTrajectoryCarpetBoundsM, kTrajectoryCarpetBoundsM,
                           kTrajectoryCarpetBoundsM}})
            .geometry(0, filament::RenderableManager::PrimitiveType::TRIANGLES, mesh.vb, mesh.ib)
            .material(0, r.trajectoryCarpetMaterialInstance)
            .culling(false)
            .castShadows(false)
            .receiveShadows(false)
            .build(*r.engine, mesh.entity);
        r.scene->addEntity(mesh.entity);
        slot.meshes.push_back(std::move(mesh));
    }
}

}  // namespace

void update_trajectory_carpets(VisualRenderer& r, const SceneGraph& s) {
    while (r.trajectoryCarpetSlots.size() > s.trajectory_carpet_count) {
        destroy_slot_meshes(r, r.trajectoryCarpetSlots.back());
        r.trajectoryCarpetSlots.pop_back();
    }
    if (r.trajectoryCarpetSlots.size() < s.trajectory_carpet_count) {
        r.trajectoryCarpetSlots.resize(s.trajectory_carpet_count);
    }

    // ONE MaterialInstance, ONE alpha: same "freshest of several wins" N>1
    // policy as point_cloud.cpp (today's shipped profile carries exactly one
    // carpet row, so this never fires in practice).
    float alpha = s.trajectory_carpet_count > 0 ? 0.0f : 1.0f;
    for (uint32_t i = 0; i < s.trajectory_carpet_count; ++i) {
        const TrajectoryCarpet& tc = s.trajectory_carpets[i];
        VisualRenderer::TrajectoryCarpetSlot& slot = r.trajectoryCarpetSlots[i];
        const uint64_t sig = trajectory_carpet_signature(tc.points, tc.point_count);
        if (!slot.has_signature || slot.signature != sig) {
            build_slot_meshes(r, slot, tc.points, tc.point_count);
            slot.signature = sig;
            slot.has_signature = true;
        }
        alpha = std::max(alpha, static_cast<float>(detail::SceneBuffer::staleness_alpha(
                                     s.sim_time_sec, tc.last_update_sec, kStaleFadeStartSec,
                                     kStaleFadeTimeoutSec)));
    }
    const float pushed_alpha = alpha * kTrajectoryCarpetBaseAlpha;
    r.trajectoryCarpetMaterialInstance->setParameter("alpha", pushed_alpha);
    r.trajectoryCarpetAlpha = pushed_alpha;
}

}  // namespace mpviz

// Filament-free test introspection hooks; see trajectory_carpet_test_hooks.hpp
// for why these live here.
namespace mpviz::testing {

size_t trajectory_carpet_mesh_count(mpviz::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->trajectoryCarpetSlots.size()) return 0;
    return r->trajectoryCarpetSlots[slot].meshes.size();
}

size_t trajectory_carpet_vertex_count(mpviz::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->trajectoryCarpetSlots.size()) return 0;
    return r->trajectoryCarpetSlots[slot].totalVertexCount;
}

float trajectory_carpet_material_alpha(mpviz::VisualRenderer* r) {
    if (r == nullptr) return 1.0f;
    return r->trajectoryCarpetAlpha;
}

uint32_t trajectory_carpet_vertex_rgba(mpviz::VisualRenderer* r, size_t slot, size_t vertex_idx) {
    if (r == nullptr || slot >= r->trajectoryCarpetSlots.size()) return 0;
    const auto& rgba = r->trajectoryCarpetSlots[slot].firstMeshRgba;
    if (vertex_idx >= rgba.size()) return 0;
    return rgba[vertex_idx];
}

float trajectory_carpet_vertex_z(mpviz::VisualRenderer* r, size_t slot, size_t vertex_idx) {
    if (r == nullptr || slot >= r->trajectoryCarpetSlots.size()) return 0.0f;
    const auto& z = r->trajectoryCarpetSlots[slot].firstMeshZ;
    if (vertex_idx >= z.size()) return 0.0f;
    return z[vertex_idx];
}

}  // namespace mpviz::testing
