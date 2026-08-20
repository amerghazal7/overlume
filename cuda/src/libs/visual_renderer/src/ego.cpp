// ego.cpp — Epic 1 Task 4 (VM-012): ego robot via Filament gltfio, with a
// themed clay-box fallback on any load failure (spec §9's non-fatal
// asset-load-failure path). Compiled as part of the same visual_renderer
// library target as renderer.cpp, so it shares that target's PRIVATE
// Filament include access — see renderer_internal.hpp's own header comment
// for why this is a separate translation unit at all (test binaries must
// never see <filament/...>, but ego needs it, same reasoning renderer.cpp
// already established).
#include "ego.hpp"
#include "ego_test_hooks.hpp"
#include "renderer_internal.hpp"
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

#include <filament/Box.h>
#include <filament/RenderableManager.h>
#include <filament/TransformManager.h>

#include <geometry/SurfaceOrientation.h>

#include <gltfio/AssetLoader.h>
#include <gltfio/FilamentAsset.h>

#include <math/mat4.h>
#include <math/quat.h>
#include <math/vec3.h>
#include <math/vec4.h>

#include <utils/Entity.h>
#include <utils/EntityManager.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <vector>

namespace mpviz {

namespace {

using filament::math::float3;
using filament::math::float4;
using filament::math::mat4f;
using filament::math::quatf;

// fill_tangent_frames() used to be duplicated here rather than promoted to
// renderer_internal.hpp ("not worth widening the Step 7e extraction's
// surface for one more helper just for this one box builder"). Epic 2 Task
// 2 (VM-024) needed the same helper from a SECOND new translation unit
// (map_elements.cpp) and promoted it there instead — see that header's
// comment. Using the promoted mpviz::fill_tangent_frames here now (this
// file already includes renderer_internal.hpp) rather than keeping two
// copies that would have to be kept behaviorally identical by hand.

// A box centered on X/Y, resting on the ground plane (Z in [0, dims.z]) —
// the ego's origin is its ground-contact point, matching how the TF
// adapter's (Task 4c, node-side) base_link position is defined for a
// ground vehicle.
void build_ego_box(std::vector<Vertex>& verts, std::vector<uint16_t>& indices,
                    const Vec3& dims) {
    const float hx = static_cast<float>(dims.x) * 0.5f;
    const float hy = static_cast<float>(dims.y) * 0.5f;
    const float hz = static_cast<float>(dims.z);
    const float3 p[8] = {
        {-hx, -hy, 0.0f}, {hx, -hy, 0.0f}, {hx, hy, 0.0f}, {-hx, hy, 0.0f},
        {-hx, -hy, hz},   {hx, -hy, hz},   {hx, hy, hz},   {-hx, hy, hz},
    };
    struct Face {
        float3 n;
        int i[4];
    };
    const Face faces[6] = {
        {{0, 0, -1}, {0, 3, 2, 1}},  // bottom
        {{0, 0, 1}, {4, 5, 6, 7}},   // top
        {{0, -1, 0}, {0, 1, 5, 4}},  // -Y
        {{0, 1, 0}, {3, 7, 6, 2}},   // +Y
        {{-1, 0, 0}, {0, 4, 7, 3}},  // -X
        {{1, 0, 0}, {1, 2, 6, 5}},   // +X
    };
    std::vector<float3> normals;
    for (const Face& f : faces) {
        const auto base = static_cast<uint16_t>(verts.size());
        for (int idx : f.i) {
            verts.push_back(Vertex{p[idx], {}});
            normals.push_back(f.n);
        }
        indices.insert(indices.end(),
                        {base, static_cast<uint16_t>(base + 1), static_cast<uint16_t>(base + 2),
                         base, static_cast<uint16_t>(base + 2), static_cast<uint16_t>(base + 3)});
    }
    fill_tangent_frames(verts, normals);
}

// Builds the themed clay-box fallback — the non-fatal path for a missing/
// unparseable glTF (spec §9). Binds r.egoMaterial, a DEDICATED clay.mat
// instance (user contrast directive 2026-08-20) — Task 4 originally reused
// r.groundMaterial here (Step 5's remap comment allowed "the SAME opaque
// clay.mat MaterialInstance"), but that made the ego render as
// palette.ground, i.e. the exact color of the ground plane it stands on,
// invisible against it in both shipped themes. egoMaterial is themed with
// its own palette.ego token (a deliberate cross-theme swap — see
// theme.hpp's Palette::ego comment) instead, created eagerly and pushed by
// push_theme_to_scene() the same way groundMaterial/laneMaterial are.
void build_ego_fallback(VisualRenderer& r, const Vec3& dims) {
    std::vector<Vertex> verts;
    std::vector<uint16_t> indices;
    build_ego_box(verts, indices, dims);
    add_mesh(r, r.egoFallback, std::move(verts), std::move(indices),
             filament::RenderableManager::PrimitiveType::TRIANGLES, r.egoMaterial,
             /*cast_shadows=*/true, /*receive_shadows=*/false);
    // Recorded for rendered_bounding_box_diagonal() (testing-only): the
    // actual box built here, not add_mesh()'s unrelated declared culling
    // AABB (see renderer_internal.hpp's egoFallbackDims comment).
    r.egoFallbackDims = dims;
    // add_mesh() doesn't create a TransformManager component — ground/grid
    // never move, so Task 2 never needed one. The ego is the first
    // renderable that does, so it's created explicitly, once, here.
    r.engine->getTransformManager().create(r.egoFallback.entity);
    r.egoTransformEntity = r.egoFallback.entity;
}

}  // namespace

bool set_ego_model(VisualRenderer* r, const char* gltf_path, Vec3 fallback_dims) {
    if (r == nullptr || gltf_path == nullptr) return false;

    std::ifstream file(gltf_path, std::ios::binary | std::ios::ate);
    if (!file) {
        build_ego_fallback(*r, fallback_dims);
        return false;
    }
    const std::streamsize size = file.tellg();
    if (size <= 0) {
        build_ego_fallback(*r, fallback_dims);
        return false;
    }
    std::vector<uint8_t> bytes(static_cast<size_t>(size));
    file.seekg(0);
    if (!file.read(reinterpret_cast<char*>(bytes.data()), size)) {
        build_ego_fallback(*r, fallback_dims);
        return false;
    }

    namespace gltfio = filament::gltfio;

    // Shared gltfio machinery (Epic 2 Task 4 / VM-022): hoisted out of this
    // function's former per-call AssetLoader + ubershader MaterialProvider +
    // ResourceLoader (Step 5's original comment on MaterialProvider/
    // ResourceLoader construction now lives in ensure_gltf_loader(),
    // renderer_internal.hpp/objects.cpp) -- objects.cpp's
    // set_object_model_dir() needs the exact same machinery, and a second
    // AssetLoader in this library is a blocking duplication finding (see
    // the plan). ensure_gltf_loader() is a no-op if either this function or
    // set_object_model_dir() already built it; behavior here is otherwise
    // unchanged (still createAsset()+releaseSourceData(), not the
    // instanced path objects.cpp uses).
    if (!ensure_gltf_loader(*r)) {
        build_ego_fallback(*r, fallback_dims);
        return false;
    }
    gltfio::FilamentAsset* asset =
        r->sharedAssetLoader->createAsset(bytes.data(), static_cast<uint32_t>(bytes.size()));
    if (asset == nullptr) {
        build_ego_fallback(*r, fallback_dims);
        return false;
    }

    // ResourceLoader::loadResources (Step 5): uploads the geometry gltfio
    // parsed above to the GPU — skipping this is the "silently renders
    // nothing" trap the finding calls out. Synchronous: the GLB's buffers
    // are embedded (obj2gltf_m02p.py's trimesh export embeds them), so
    // there's no external URI to resolve asynchronously.
    if (!r->sharedResourceLoader->loadResources(asset)) {
        r->sharedAssetLoader->destroyAsset(asset);
        build_ego_fallback(*r, fallback_dims);
        return false;
    }
    asset->releaseSourceData();

    // Material remap (Step 5, spec §4.2): the ego reads as clay like
    // everything else, not whatever materials the OBJ->glTF conversion
    // produced. Safe against the current opaque clay.mat (Task 2 dropped
    // its old `requires: [color]`) — the gltfio-loaded mesh has no vertex
    // COLOR attribute, and would have failed this remap against the old
    // material. Remaps to r->egoMaterial, NOT r->groundMaterial (user
    // contrast directive 2026-08-20): the ego needs its own themed instance
    // (palette.ego, a cross-theme swap against the ground it stands on —
    // see build_ego_fallback()'s comment above and theme.hpp's Palette::ego
    // comment) so it doesn't render as the same color as the ground plane.
    filament::RenderableManager& rm = r->engine->getRenderableManager();
    const utils::Entity* renderables = asset->getRenderableEntities();
    const size_t renderableCount = asset->getRenderableEntityCount();
    for (size_t i = 0; i < renderableCount; ++i) {
        const auto inst = rm.getInstance(renderables[i]);
        if (!inst.isValid()) continue;
        const size_t primCount = rm.getPrimitiveCount(inst);
        for (size_t p = 0; p < primCount; ++p) {
            rm.setMaterialInstanceAt(inst, p, r->egoMaterial);
        }
        // The ego is the one thing in this epic's scene that should
        // actually darken the ground it stands on; nothing casts onto the
        // ego itself yet (Step 5).
        rm.setCastShadows(inst, true);
        rm.setReceiveShadows(inst, false);
    }

    r->scene->addEntities(asset->getEntities(), asset->getEntityCount());

    r->egoAsset = asset;
    // The asset's transform root already has a TransformManager component
    // (built by gltfio's own node hierarchy) — no explicit create() needed,
    // unlike the clay-box fallback path above.
    r->egoTransformEntity = asset->getRoot();
    return true;
}

// Epic 1 Task 4 (VM-012): see ego.hpp's header comment.
void update_ego_transform(VisualRenderer& r, const EgoState& ego) {
    if (!r.egoTransformEntity) return;  // set_ego_model() never called
    filament::TransformManager& tm = r.engine->getTransformManager();
    const auto inst = tm.getInstance(r.egoTransformEntity);
    if (!inst.isValid()) return;
    const float3 pos{static_cast<float>(ego.position.x), static_cast<float>(ego.position.y),
                      static_cast<float>(ego.position.z)};
    const quatf rot = quatf::fromAxisAngle(float3{0, 0, 1}, static_cast<float>(ego.heading_rad));
    // ego.valid == 0 (no TF yet, spec §9's non-fatal "no data" path) hides
    // the ego by zero-scaling its transform rather than tracking scene
    // membership across the glTF asset's whole entity list (root + N node
    // entities) — simpler, and a zero-scale renderable has no visible
    // extent and casts no shadow either. Upgrade path if this ever needs
    // to skip vertex-shader cost too: scene->removeEntities()/
    // addEntities() instead, tracked against a "currently in scene" bool.
    const float scale = ego.valid ? 1.0f : 0.0f;
    tm.setTransform(inst, mat4f::translation(pos) * mat4f(rot) * mat4f::scaling(scale));
}

}  // namespace mpviz

namespace mpviz::testing {

double rendered_bounding_box_diagonal(mpviz::VisualRenderer* r) {
    if (r == nullptr) return 0.0;
    if (r->egoAsset != nullptr) {
        const filament::Aabb box = r->egoAsset->getBoundingBox();
        const filament::math::float3 d = box.max - box.min;
        return std::sqrt(static_cast<double>(d.x) * d.x + static_cast<double>(d.y) * d.y +
                          static_cast<double>(d.z) * d.z);
    }
    if (r->egoFallback.entity) {
        // NOT RenderableManager::getAxisAlignedBoundingBox(): that reports
        // add_mesh()'s hard-coded declared culling AABB (renderer.cpp's
        // kGroundHalfExtent box), a constant unrelated to what
        // build_ego_box() actually built — reading it made this fallback
        // path's diagonal vacuous regardless of `dims` (review round 8).
        // egoFallbackDims is the real dims build_ego_fallback() used; a
        // {0,0,0} guard covers the "never actually built" case too.
        const Vec3& d = r->egoFallbackDims;
        return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    }
    return 0.0;
}

}  // namespace mpviz::testing
