// bowl.cpp — see bowl.hpp. VM-091 (unified-engine migration Task 2):
// builds/rebuilds the camera bowl's Filament mesh + material from
// bowl_mesh.hpp's CPU-only bake, and does the per-tick ego-motion-delta ->
// effective-extrinsics composition bowl.mat's header explains.
//
// VM-092 (Task 3, Decision 4) extends build_bowl()'s bake call with the
// analytic self-view occlusion test (bowl_mesh.cpp's BakeBowlMesh, gated on
// r.selfViewMasksEnabled/set_self_view_masks, off by default) -- see
// ego_rig_frame_box() below for how the ego's bounding box is read.
// Robot-proxy compositing itself (the OTHER half of Task 3, Decision 4)
// needed no code here at all: the ego entity (ego.cpp's set_ego_model)
// and the bowl entity both already live in the one r.scene this file's
// build_bowl()/update_bowl() add/remove the bowl from, so Filament's own
// depth test composites them correctly for free (see renderer.cpp's scene
// creation for the confirming note).
#include "bowl.hpp"

#include "bowl_mesh.hpp"
#include "bowl_projection.hpp"
#include "renderer_internal.hpp"
#include "overlume/scene.h"

#include "bowl_filamat.h"

#include <filament/Box.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/TextureSampler.h>
#include <filament/TransformManager.h>

#include <gltfio/FilamentAsset.h>

#include <utils/EntityManager.h>

#include <cstdio>
#include <string>
#include <vector>

namespace overlume {

namespace {

using filament::math::float2;
using filament::math::float3;
using filament::math::float4;
using filament::math::mat4f;
using filament::math::quatf;

// One bowl-mesh vertex: rig-frame position (drives BOTH the standard
// transform/depth pipeline AND, redundantly, the fragment shader's
// per-camera math -- see bowl.mat's header for why the latter can't just
// reuse Filament's own getWorldPosition()) + the bake's weight/index
// payload, reused through the COLOR channel (bowl.mat's `requires: [
// color, custom0 ]`).
//
// rigPos rides a real custom vertex attribute (CUSTOM0, bound FLOAT3),
// NOT uv0/uv1 -- Filament's generated vertex shader V-flips both
// (`vec2(mesh_uv0.x, 1.0 - mesh_uv0.y)`), fine for an actual texture UV but
// wrong for an arbitrary position payload. Filament does no per-component
// arithmetic on CUSTOM attributes, so rigPos passes through untouched.
struct BowlGpuVertex {
    float3 position;
    float4 color;    // coverage_a, index_a, coverage_b, index_b
    float3 rigPos;   // == position, verbatim -- CUSTOM0, untouched by Filament
    // Third camera slot (VM-091 gate close-out finding 7, Decision
    // resolution 2's "2-3 contributing cameras per fragment"): coverage_c,
    // index_c -- CUSTOM1, a physically separate raw-attribute slot from
    // CUSTOM0 above (Filament reserves 8 raw attribute slots against only 4
    // varying slots, bowl.mat's header), carried to the fragment shader on
    // the spare custom3 interpolant.
    float2 covIdxC;
};

filament::VertexBuffer* make_bowl_vertex_buffer(filament::Engine& engine,
                                                 std::vector<BowlGpuVertex> verts) {
    auto* heap = new std::vector<BowlGpuVertex>(std::move(verts));
    filament::VertexBuffer* vb =
        filament::VertexBuffer::Builder()
            .vertexCount(static_cast<uint32_t>(heap->size()))
            .bufferCount(1)
            .attribute(filament::VertexAttribute::POSITION, 0,
                       filament::VertexBuffer::AttributeType::FLOAT3,
                       offsetof(BowlGpuVertex, position), sizeof(BowlGpuVertex))
            .attribute(filament::VertexAttribute::COLOR, 0,
                       filament::VertexBuffer::AttributeType::FLOAT4,
                       offsetof(BowlGpuVertex, color), sizeof(BowlGpuVertex))
            .attribute(filament::VertexAttribute::CUSTOM0, 0,
                       filament::VertexBuffer::AttributeType::FLOAT3,
                       offsetof(BowlGpuVertex, rigPos), sizeof(BowlGpuVertex))
            .attribute(filament::VertexAttribute::CUSTOM1, 0,
                       filament::VertexBuffer::AttributeType::FLOAT2,
                       offsetof(BowlGpuVertex, covIdxC), sizeof(BowlGpuVertex))
            .build(engine);
    vb->setBufferAt(engine, 0,
                     filament::VertexBuffer::BufferDescriptor(
                         heap->data(), heap->size() * sizeof(BowlGpuVertex),
                         [](void*, size_t, void* user) {
                             delete static_cast<std::vector<BowlGpuVertex>*>(user);
                         },
                         heap));
    return vb;
}

// R is row-major 3x3; column j is (R[j], R[3+j], R[6+j]) -- same convention
// as bowl_projection.cpp's own column() (reproject.cu's "R columns =
// (right,down,fwd)").
float3 rotation_column(const double R[9], int j) {
    return {static_cast<float>(R[j]), static_cast<float>(R[3 + j]), static_cast<float>(R[6 + j])};
}

// delta is row-major 4x4 (scene.h's set_camera_motion_delta contract):
// row0=[m0..m3], row1=[m4..m7], row2=[m8..m11], row3=[m12..m15] (== [0,0,0,1]
// for a rigid ego-motion delta). Returns delta's rotation block, TRANSPOSED
// applied to v -- i.e. column k of the transpose is row k of delta, so
// component k of the result is dot(delta's OWN column k, v). An ego-motion
// delta's rotation is orthonormal, so its transpose IS its inverse -- no
// matrix inversion needed to undo it.
float3 apply_delta_rotation_transposed(const double delta[16], float3 v) {
    return {static_cast<float>(delta[0] * v.x + delta[4] * v.y + delta[8] * v.z),
            static_cast<float>(delta[1] * v.x + delta[5] * v.y + delta[9] * v.z),
            static_cast<float>(delta[2] * v.x + delta[6] * v.y + delta[10] * v.z)};
}

float3 delta_translation(const double delta[16]) {
    return {static_cast<float>(delta[3]), static_cast<float>(delta[7]),
            static_cast<float>(delta[11])};
}

std::string cam_param(const char* prefix, uint32_t i) { return std::string(prefix) + std::to_string(i); }

// VM-092: the ego's bounding box, RIG FRAME -- from whichever of
// set_ego_model()'s two outcomes populated r (a loaded glTF's own AABB, or
// the clay-box fallback's dims). A zero-extent box (neither populated)
// means no ego configured, BakeBowlMesh's own no-op convention.
//
// Ordering contract: set_ego_model() must have run before the
// set_bowl_config() that bakes with masks enabled -- build_bowl() reads
// this box ONCE per bake, so an ego model set after that bake silently
// no-ops the self-view mask until the next re-bake (same on_activate()
// ordering hazard camera_textures.cpp's set_self_view_masks() comment
// already names for the enable flag; it applies just as much to the ego
// model itself).
bowl::EgoBox ego_rig_frame_box(const VisualRenderer& r) {
    if (r.egoAsset != nullptr) {
        const filament::Aabb box = r.egoAsset->getBoundingBox();
        const float3 half = (box.max - box.min) * 0.5f;
        const float3 center = (box.max + box.min) * 0.5f;
        return {{center.x, center.y, center.z}, {half.x, half.y, half.z}};
    }
    if (r.egoFallback.entity) {
        // Matches ego.cpp's build_ego_box(): centered on X/Y, resting on
        // the ground plane (Z in [0, dims.z]).
        const Vec3& d = r.egoFallbackDims;
        return {{0.0, 0.0, d.z * 0.5}, {d.x * 0.5, d.y * 0.5, d.z * 0.5}};
    }
    return {};  // no ego configured -- zero-extent, BakeBowlMesh no-ops
}

}  // namespace

bool build_bowl(VisualRenderer& r, const BowlConfig& cfg) {
    // Tear down any previous bowl entirely -- set_bowl_config() is a full
    // re-bake, never an incremental update (camera_textures.cpp's own
    // set_bowl_config comment: "This task never reuses a still-valid
    // texture across a reconfigure").
    if (r.bowl) {
        destroy_mesh(*r.engine, *r.scene, r.bowl->mesh);
        if (r.bowl->instance) r.engine->destroy(r.bowl->instance);
        if (r.bowl->material) r.engine->destroy(r.bowl->material);
        r.bowl.reset();
    }

    bowl::BowlMeshParams params;
    // VM-092: self-view masks, off by default (set_self_view_masks/
    // r.selfViewMasksEnabled) -- a zero-extent EgoBox when disabled, same
    // no-op convention BakeBowlMesh already applies when no ego is
    // configured (ego_rig_frame_box() returns zero-extent in that case too).
    const bowl::EgoBox ego_box = r.selfViewMasksEnabled ? ego_rig_frame_box(r) : bowl::EgoBox{};
    const bowl::BowlMesh baked =
        bowl::BakeBowlMesh(params, cfg.bowl_R0, cfg.bowl_k, cfg.bowl_Rmax, cfg.camera_count,
                           cfg.extrinsics, cfg.intrinsics, cfg.cam_width, cfg.cam_height, ego_box);
    if (baked.vertices.empty() || baked.indices.empty()) return false;
    if (baked.vertices.size() > 65535) {
        // ponytail: uint16 index buffer ceiling, same class of limit
        // point_cloud.cpp's chunk-split already works around elsewhere in
        // this library. Default tessellation (64 theta x 24 rings, ~9.2k
        // duplicated vertices) stays well under this; a future tessellation
        // bump past it needs the same chunk-split, not a silent truncation.
        std::fprintf(stderr,
                      "bowl: baked mesh has %zu vertices (> 65535); refusing to build a "
                      "truncated bowl -- reduce BowlMeshParams tessellation.\n",
                      baked.vertices.size());
        return false;
    }

    auto owned = std::make_unique<BowlState>();
    owned->material = filament::Material::Builder()
                          .package(overlume::materials::kbowlFilamat, overlume::materials::kbowlFilamatSize)
                          .build(*r.engine);
    owned->instance = owned->material->createInstance();

    std::vector<BowlGpuVertex> verts(baked.vertices.size());
    for (size_t i = 0; i < baked.vertices.size(); ++i) {
        const bowl::BowlVertex& bv = baked.vertices[i];
        verts[i].position = {static_cast<float>(bv.position.x), static_cast<float>(bv.position.y),
                              static_cast<float>(bv.position.z)};
        verts[i].color = {bv.coverage_a, static_cast<float>(bv.index_a), bv.coverage_b,
                           static_cast<float>(bv.index_b)};
        verts[i].rigPos = verts[i].position;
        verts[i].covIdxC = {bv.coverage_c, static_cast<float>(bv.index_c)};
    }
    std::vector<uint16_t> indices(baked.indices.begin(), baked.indices.end());

    owned->mesh.vb = make_bowl_vertex_buffer(*r.engine, std::move(verts));
    owned->mesh.ib = make_index_buffer(*r.engine, std::move(indices));
    owned->mesh.entity = utils::EntityManager::get().create();
    owned->mesh.vertexCount = static_cast<uint32_t>(baked.vertices.size());
    // Generous bounding box: bowl_Rmax in every horizontal direction, a
    // couple of metres of headroom vertically -- this is a bounds hint for
    // culling only, not a rendered surface, so looseness costs nothing.
    const float extent = static_cast<float>(cfg.bowl_Rmax) + 1.0f;
    filament::RenderableManager::Builder(1)
        .boundingBox({{0, 0, 0}, {extent, extent, extent}})
        .geometry(0, filament::RenderableManager::PrimitiveType::TRIANGLES, owned->mesh.vb,
                  owned->mesh.ib)
        .material(0, owned->instance)
        .culling(false)
        .castShadows(false)
        .receiveShadows(false)
        .build(*r.engine, owned->mesh.entity);
    r.engine->getTransformManager().create(owned->mesh.entity);
    // NOT added to the scene here -- update_bowl() adds/removes it every
    // render_frame() call based on r.bowlVisible (set_bowl_visible), so a
    // bowl built while hidden (or built before the first set_bowl_visible
    // call) never appears until requested.

    // Per-camera uniforms that do NOT change per-tick: K/plumb_bob dist/k3,
    // the camera texture sampler, and the BASE (identity-delta) right/fwd/t
    // -- update_bowl() overwrites right/fwd/t every render_frame() call
    // with the ego-motion-delta-composed EFFECTIVE values (identity delta
    // at this point in time reproduces these same base values, so the bowl
    // renders correctly even before the first update_bowl() call, e.g. in
    // a test that calls render_frame() without ever calling
    // set_camera_motion_delta()).
    filament::TextureSampler linear(filament::TextureSampler::MinFilter::LINEAR,
                                     filament::TextureSampler::MagFilter::LINEAR);
    for (uint32_t i = 0; i < kMaxBowlCameras; ++i) {
        const uint32_t src = i < cfg.camera_count ? i : 0;  // unused slots mirror camera 0's
                                                             // texture (never sampled: their
                                                             // weight is always 0 by
                                                             // construction, bowl_mesh.cpp)
        owned->instance->setParameter(cam_param("camTex", i).c_str(), r.cameraSlots[src].texture,
                                       linear);
        if (i >= cfg.camera_count) {
            owned->instance->setParameter(cam_param("camRight", i).c_str(), float3{0, 0, 0});
            owned->instance->setParameter(cam_param("camFwd", i).c_str(), float3{0, 0, 0});
            owned->instance->setParameter(cam_param("camT", i).c_str(), float3{0, 0, 0});
            owned->instance->setParameter(cam_param("camK", i).c_str(), float4{1, 1, 0, 0});
            owned->instance->setParameter(cam_param("camDist", i).c_str(), float4{0, 0, 0, 0});
            owned->instance->setParameter(cam_param("camK3_", i).c_str(), 0.0f);
            continue;
        }
        const CameraExtrinsics& ext = cfg.extrinsics[i];
        const CameraIntrinsics& in = cfg.intrinsics[i];
        owned->instance->setParameter(cam_param("camRight", i).c_str(), rotation_column(ext.R, 0));
        owned->instance->setParameter(cam_param("camFwd", i).c_str(), rotation_column(ext.R, 2));
        owned->instance->setParameter(
            cam_param("camT", i).c_str(),
            float3{static_cast<float>(ext.t[0]), static_cast<float>(ext.t[1]),
                   static_cast<float>(ext.t[2])});
        owned->instance->setParameter(
            cam_param("camK", i).c_str(),
            float4{static_cast<float>(in.fx), static_cast<float>(in.fy),
                   static_cast<float>(in.cx), static_cast<float>(in.cy)});
        owned->instance->setParameter(
            cam_param("camDist", i).c_str(),
            float4{static_cast<float>(in.dist[0]), static_cast<float>(in.dist[1]),
                   static_cast<float>(in.dist[2]), static_cast<float>(in.dist[3])});
        owned->instance->setParameter(cam_param("camK3_", i).c_str(),
                                       static_cast<float>(in.dist[4]));
    }
    owned->instance->setParameter(
        "skyColor", float3{cfg.sky_color[0], cfg.sky_color[1], cfg.sky_color[2]});
    // Per-fragment feather + the exposure-compensation style knob --
    // both node-side style knobs alongside sky_color, per the STANDING
    // "every rendered element ships style tokens" directive.
    owned->instance->setParameter("featherMargin", static_cast<float>(cfg.feather_margin));
    owned->instance->setParameter("exposureCompensation", cfg.exposure_compensation);

    r.bowl = std::move(owned);
    return true;
}

void update_bowl(VisualRenderer& r, const EgoState& ego) {
    if (!r.bowl) return;

    for (uint32_t i = 0; i < r.cameraCount; ++i) {
        const CameraTextureSlot& slot = r.cameraSlots[i];
        const float3 right_base = rotation_column(slot.extrinsics.R, 0);
        const float3 fwd_base = rotation_column(slot.extrinsics.R, 2);
        const float3 t_base{static_cast<float>(slot.extrinsics.t[0]),
                             static_cast<float>(slot.extrinsics.t[1]),
                             static_cast<float>(slot.extrinsics.t[2])};
        // Effective (ego-motion-delta-composed) extrinsics -- see bowl.mat's
        // header for the derivation: an identity delta reproduces the base
        // values exactly (transpose of identity is identity, delta_t is
        // zero), so this is a strict generalization of the no-compensation
        // case, not a special-cased branch.
        const float3 right_eff = apply_delta_rotation_transposed(slot.motionDelta, right_base);
        const float3 fwd_eff = apply_delta_rotation_transposed(slot.motionDelta, fwd_base);
        const float3 dt = delta_translation(slot.motionDelta);
        const float3 t_eff =
            apply_delta_rotation_transposed(slot.motionDelta, {t_base.x - dt.x, t_base.y - dt.y,
                                                                t_base.z - dt.z});
        r.bowl->instance->setParameter(cam_param("camRight", i).c_str(), right_eff);
        r.bowl->instance->setParameter(cam_param("camFwd", i).c_str(), fwd_eff);
        r.bowl->instance->setParameter(cam_param("camT", i).c_str(), t_eff);
    }

    if (r.bowlVisible && !r.bowl->inScene) {
        r.scene->addEntity(r.bowl->mesh.entity);
        r.bowl->inScene = true;
    } else if (!r.bowlVisible && r.bowl->inScene) {
        r.scene->remove(r.bowl->mesh.entity);
        r.bowl->inScene = false;
    }

    filament::TransformManager& tm = r.engine->getTransformManager();
    const auto inst = tm.getInstance(r.bowl->mesh.entity);
    if (!inst.isValid()) return;
    if (!ego.valid) {
        tm.setTransform(inst, mat4f());  // identity -- map origin, same
                                          // convention as
                                          // update_ground_grid_transform
        return;
    }
    const float3 pos{static_cast<float>(ego.position.x), static_cast<float>(ego.position.y),
                      static_cast<float>(ego.position.z)};
    const quatf rot = quatf::fromAxisAngle(float3{0, 0, 1}, static_cast<float>(ego.heading_rad));
    tm.setTransform(inst, mat4f::translation(pos) * mat4f(rot));
}

}  // namespace overlume
