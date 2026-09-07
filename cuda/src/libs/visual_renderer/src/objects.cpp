// objects.cpp — Epic 2 Task 4 (VM-022): clay object rendering. Instanced
// glTF per class (car/truck_van/bus/pedestrian/cyclist), scaled to each
// TrackedObject's measured bbox, per-class theme tints, velocity arrows,
// predicted-path ribbons (reusing Task 2's extrude_polyline — no second
// extruder), and the staleness fade via clay_translucent.mat (see the
// plan's "…and the material that can actually do it" for the full argument
// this file's fade mechanism rests on).
//
// UNKNOWN and any class with no loaded/loadable model (Step 0's default:
// bus/cyclist ship with no model — see assets/models/ATTRIBUTION.md) fall
// back to a procedural clay box, ALWAYS built as a unit 1x1x1 cube so its
// TransformManager scale is `dims / (1,1,1)` — identical math to the glTF
// path (`dims / class_unit_footprint`), no separate "box is baked at dims"
// special case.
#include "objects.hpp"
#include "objects_test_hooks.hpp"
#include "polyline.hpp"
#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

#include <filament/Box.h>
#include <filament/RenderableManager.h>
#include <filament/TransformManager.h>

#include <gltfio/AssetLoader.h>
#include <gltfio/FilamentAsset.h>
#include <gltfio/MaterialProvider.h>
#include <gltfio/ResourceLoader.h>
#include <gltfio/materials/uberarchive.h>

#include <math/mat4.h>
#include <math/quat.h>
#include <math/vec3.h>
#include <math/vec4.h>

#include <utils/Entity.h>
#include <utils/EntityManager.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

namespace mpviz {

namespace {

using filament::math::float3;
using filament::math::float4;
using filament::math::mat4f;
using filament::math::quatf;

float3 to_f3(const Vec3& v) {
    return float3{static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

bool is_zero_vec3(const Vec3& v) { return v.x == 0.0 && v.y == 0.0 && v.z == 0.0; }

// A plain (not rounded — the fillet buys nothing a test or a golden checks,
// see the plan's "procedural rounded clay box"; ponytail: upgrade if a
// human reviewing the golden ever flags the boxy look) unit cube: X/Y in
// [-0.5, 0.5], Z in [0, 1] (ground-contact origin, same convention as
// ego.cpp's build_ego_box — deliberately NOT shared with it: this file's
// box is always UNIT-sized so per-object dims apply as a TransformManager
// scale like the glTF path does, whereas ego's is baked directly at its
// fallback_dims; sharing would mean threading that difference through one
// more parameter for a ~20-line function).
void build_unit_box(std::vector<Vertex>& verts, std::vector<uint16_t>& indices) {
    constexpr float h = 0.5f;
    const float3 p[8] = {
        {-h, -h, 0.0f}, {h, -h, 0.0f}, {h, h, 0.0f}, {-h, h, 0.0f},
        {-h, -h, 1.0f}, {h, -h, 1.0f}, {h, h, 1.0f}, {-h, h, 1.0f},
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

// The unit velocity-arrow geometry is the promoted shared build_unit_arrow()
// (renderer_internal.hpp / renderer.cpp) -- generic_markers.cpp's ARROW
// primitive draws the same mesh (review 2026-08-20).

void remap_to_material(filament::RenderableManager& rm, const utils::Entity* ents, size_t n,
                        filament::MaterialInstance* material) {
    for (size_t i = 0; i < n; ++i) {
        const auto ri = rm.getInstance(ents[i]);
        if (!ri.isValid()) continue;
        const size_t primCount = rm.getPrimitiveCount(ri);
        for (size_t p = 0; p < primCount; ++p) {
            rm.setMaterialInstanceAt(ri, p, material);
        }
    }
}

// This class's own normalized unit footprint (X length / Y width / Z
// height) — (1,1,1) for the procedural-box fallback (see build_unit_box's
// comment), else the class pool's own value read back from the loaded
// glb's bounding box (set_object_model_dir()).
Vec3 unit_footprint_for(VisualRenderer& r, const ObjectEntity& e) {
    if (e.glInstance == nullptr) return {1.0, 1.0, 1.0};
    auto it = r.objectClassPools.find(static_cast<uint8_t>(e.cls));
    return it != r.objectClassPools.end() ? it->second.unitFootprint : Vec3{1.0, 1.0, 1.0};
}

// Acquires a renderable for a NEWLY-seen track (Step 3): pulls a
// FilamentInstance off its class's free list, grows the pool
// (createInstance(), logged once per class) if the free list is empty and
// the class hasn't hit kMaxInstancesPerClass, or falls back to the
// procedural box (missing/unloadable model, UNKNOWN, or pool exhaustion —
// all non-fatal, spec §9). The clay remap (setMaterialInstanceAt per
// primitive) happens HERE, once per acquire, never per frame.
void acquire_entity(VisualRenderer& r, const TrackedObject& obj, ObjectEntity& out) {
    out.cls = obj.cls;
    const auto clsIdx = static_cast<uint8_t>(obj.cls);
    auto poolIt = r.objectClassPools.find(clsIdx);
    const bool hasPool = obj.cls != ObjectClass::UNKNOWN && poolIt != r.objectClassPools.end();

    if (hasPool) {
        ObjectClassPool& pool = poolIt->second;
        filament::gltfio::FilamentInstance* inst = nullptr;
        if (!pool.freeList.empty()) {
            inst = pool.freeList.back();
            pool.freeList.pop_back();
        } else if (pool.pool.size() < kMaxInstancesPerClass) {
            // Growth past the initial createInstancedAsset() batch (Step 3):
            // one at a time, amortized, DEBUG-logged once per class.
            inst = r.sharedAssetLoader->createInstance(pool.asset);
            if (inst != nullptr) {
                pool.pool.push_back(inst);
                if (!pool.growthLogged) {
                    std::fprintf(stderr,
                                 "[visual_renderer] object class %u grew past %zu instances\n",
                                 static_cast<unsigned>(clsIdx), kInitialInstancesPerClass);
                    pool.growthLogged = true;
                }
            }
        } else if (!pool.capWarned) {
            std::fprintf(stderr,
                         "[visual_renderer] object class %u hit the %zu-instance cap; "
                         "falling back to the procedural clay box\n",
                         static_cast<unsigned>(clsIdx), kMaxInstancesPerClass);
            pool.capWarned = true;
        }
        if (inst != nullptr) {
            out.glInstance = inst;
            out.transformRoot = inst->getRoot();
            filament::RenderableManager& rm = r.engine->getRenderableManager();
            const utils::Entity* ents = inst->getEntities();
            const size_t entCount = inst->getEntityCount();
            remap_to_material(rm, ents, entCount, r.objectClassMaterial[clsIdx]);
            for (size_t i = 0; i < entCount; ++i) {
                const auto ri = rm.getInstance(ents[i]);
                if (!ri.isValid()) continue;
                rm.setCastShadows(ri, true);
                rm.setReceiveShadows(ri, false);
            }
            r.scene->addEntities(ents, entCount);
            return;
        }
    } else if (obj.cls != ObjectClass::UNKNOWN && !r.objectClassMissingWarned[clsIdx]) {
        std::fprintf(stderr,
                     "[visual_renderer] object class %u has no loaded model; using the "
                     "procedural clay box\n",
                     static_cast<unsigned>(clsIdx));
        r.objectClassMissingWarned[clsIdx] = true;
    }

    std::vector<Vertex> verts;
    std::vector<uint16_t> indices;
    build_unit_box(verts, indices);
    add_mesh(r, out.proceduralBox, std::move(verts), std::move(indices),
             filament::RenderableManager::PrimitiveType::TRIANGLES, r.objectClassMaterial[clsIdx],
             /*cast_shadows=*/true, /*receive_shadows=*/false);
    r.engine->getTransformManager().create(out.proceduralBox.entity);
    out.transformRoot = out.proceduralBox.entity;
    out.glInstance = nullptr;
}

void update_entity_transform(VisualRenderer& r, const TrackedObject& obj, ObjectEntity& e) {
    filament::TransformManager& tm = r.engine->getTransformManager();
    const auto inst = tm.getInstance(e.transformRoot);
    if (!inst.isValid()) return;
    const float3 pos{static_cast<float>(obj.position.x), static_cast<float>(obj.position.y),
                      static_cast<float>(obj.position.z)};
    const quatf rot = quatf::fromAxisAngle(float3{0, 0, 1}, static_cast<float>(obj.heading_rad));
    const Vec3 unit = unit_footprint_for(r, e);
    // Perception bbox always wins (stretch, never clip) — a misclassified
    // object is cosmetic, never a rendering failure (Objects.
    // DimensionsDriveScaleNotTheClassModel).
    const float3 scale{
        static_cast<float>(obj.dimensions.x / (unit.x > 0.0 ? unit.x : 1.0)),
        static_cast<float>(obj.dimensions.y / (unit.y > 0.0 ? unit.y : 1.0)),
        static_cast<float>(obj.dimensions.z / (unit.z > 0.0 ? unit.z : 1.0)),
    };
    e.appliedScale = {scale.x, scale.y, scale.z};
    tm.setTransform(inst, mat4f::translation(pos) * mat4f(rot) * mat4f::scaling(scale));
}

void ensure_shared_arrow_mesh(VisualRenderer& r) {
    if (r.sharedArrowMesh.vb != nullptr) return;
    std::vector<Vertex> verts;
    std::vector<uint16_t> indices;
    build_unit_arrow(verts, indices);
    r.sharedArrowMesh.vb = make_vertex_buffer(*r.engine, std::move(verts));
    r.sharedArrowMesh.ib = make_index_buffer(*r.engine, std::move(indices));
}

// velocity == {0,0,0} -> no arrow (spec §4.1). Otherwise one entity
// (created once, recycled by transform alone afterward — never rebuilt)
// bound to the ONE shared unit-arrow vb/ib, rotated to the velocity
// heading and scaled in length by (clamped) speed, floating just above the
// object's own roof.
//
// ponytail: the arrow doesn't participate in the staleness fade (only the
// object's own body does) -- no test checks it, and the fade mechanism's
// stated scope ("that entity's renderable") is naturally read as the
// object body; revisit if a fading arrow ever shows up as a visual bug.
void update_entity_arrow(VisualRenderer& r, const TrackedObject& obj, ObjectEntity& e) {
    filament::TransformManager& tm = r.engine->getTransformManager();
    if (is_zero_vec3(obj.velocity)) {
        if (e.arrowEntity) {
            r.scene->remove(e.arrowEntity);
            r.engine->destroy(e.arrowEntity);
            utils::EntityManager::get().destroy(e.arrowEntity);
            e.arrowEntity = {};
        }
        return;
    }
    ensure_shared_arrow_mesh(r);
    if (!e.arrowEntity) {
        e.arrowEntity = utils::EntityManager::get().create();
        filament::RenderableManager::Builder(1)
            .boundingBox({{0, 0, 0}, {50.0f, 50.0f, 50.0f}})  // culling(false) below -- exact box irrelevant
            .geometry(0, filament::RenderableManager::PrimitiveType::TRIANGLES, r.sharedArrowMesh.vb,
                      r.sharedArrowMesh.ib)
            .material(0, r.objectClassMaterial[static_cast<uint8_t>(e.cls)])
            .culling(false)
            .castShadows(false)
            .receiveShadows(false)
            .build(*r.engine, e.arrowEntity);
        r.scene->addEntity(e.arrowEntity);
        tm.create(e.arrowEntity);
    }
    const double speed = std::sqrt(obj.velocity.x * obj.velocity.x + obj.velocity.y * obj.velocity.y);
    const double heading = std::atan2(obj.velocity.y, obj.velocity.x);
    const float3 pos{static_cast<float>(obj.position.x), static_cast<float>(obj.position.y),
                      static_cast<float>(obj.position.z + obj.dimensions.z + 0.15)};
    const quatf rot = quatf::fromAxisAngle(float3{0, 0, 1}, static_cast<float>(heading));
    const auto len = static_cast<float>(std::clamp(speed, 0.5, 5.0));
    const auto inst = tm.getInstance(e.arrowEntity);
    if (inst.isValid()) {
        tm.setTransform(inst, mat4f::translation(pos) * mat4f(rot) * mat4f::scaling(float3{len, 1.0f, 1.0f}));
    }
}

// Content signature for a predicted path -- same shape as map_elements.cpp's
// chunk_signature() (point count + first/last point; the source data is
// re-published wholesale each tick, so array identity means nothing, but a
// literally-unchanged prediction shouldn't re-extrude every frame). A small
// hash, not the extrusion algorithm itself -- duplicating THIS is not the
// "second extruder" the plan's duplication finding is about.
uint64_t path_signature(const Vec3* pts, uint32_t n) {
    auto mix = [](uint64_t seed, uint64_t v) {
        return seed ^ (v + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2));
    };
    auto hv = [&](const Vec3& v) {
        uint64_t h = mix(0, std::hash<double>{}(v.x));
        h = mix(h, std::hash<double>{}(v.y));
        h = mix(h, std::hash<double>{}(v.z));
        return h;
    };
    uint64_t h = mix(0, n);
    if (n > 0) {
        h = mix(h, hv(pts[0]));
        h = mix(h, hv(pts[n - 1]));
    }
    return h;
}

// Predicted-path ribbon: extrude_polyline() (Task 2's shared helper — no
// second extruder), rebuilt ONLY when the path's content signature changes
// since the last update (skip-rebuild guard, same spirit as
// map_elements.cpp's diff cache). Baked directly in WORLD space from the
// TrackedObject's own absolute predicted_path points, so — unlike the
// object body — it needs no TransformManager transform at all.
void update_entity_path(VisualRenderer& r, const TrackedObject& obj, ObjectEntity& e) {
    const uint32_t n = std::min(obj.predicted_path_count, detail::kMaxPointsPerMesh);
    if (obj.predicted_path == nullptr || n < 2) {
        if (e.pathRibbon.entity) {
            destroy_mesh(*r.engine, *r.scene, e.pathRibbon);
            e.pathSignature = 0;
        }
        return;
    }
    const uint64_t sig = path_signature(obj.predicted_path, n);
    // Signature-only guard, NOT entity-gated: a degenerate path (all points
    // coincident -> extrude_polyline returns empty, no entity built) must
    // still be skipped on every later frame while unchanged, or it is
    // re-extruded ~30x/s forever.
    if (sig == e.pathSignature) return;
    if (e.pathRibbon.entity) destroy_mesh(*r.engine, *r.scene, e.pathRibbon);

    constexpr float kPathHalfWidthM = 0.08f;
    constexpr float kPathZLiftM = 0.03f;
    std::vector<Vec3> ribbon = detail::extrude_polyline(obj.predicted_path, n, kPathHalfWidthM, kPathZLiftM);
    e.pathSignature = sig;
    if (ribbon.empty()) return;
    // TRUE indexed mesh, ribbon.cpp's pattern (review 2026-08-20): the old
    // "flatten then identity uint16 indices" shortcut wraps its counter and
    // never terminates once flattened verts exceed 65535 -- reachable, since
    // n is only capped at kMaxPointsPerMesh (32000) and flattening
    // multiplies by ~6. Indexed: 2n distinct verts (<= 64000) + index
    // values 0..2n-1, both safely under the uint16 ceiling.
    std::vector<uint16_t> idx =
        detail::extrude_polyline_indices(static_cast<uint32_t>(ribbon.size() / 2));
    if (idx.empty()) return;
    std::vector<Vertex> verts(ribbon.size());
    for (size_t i = 0; i < ribbon.size(); ++i) verts[i].position = to_f3(ribbon[i]);
    fill_tangent_frames(verts, std::vector<float3>(ribbon.size(), float3{0, 0, 1}));
    add_mesh(r, e.pathRibbon, std::move(verts), std::move(idx),
             filament::RenderableManager::PrimitiveType::TRIANGLES,
             r.objectClassMaterial[static_cast<uint8_t>(e.cls)],
             /*cast_shadows=*/false, /*receive_shadows=*/true);
}

// Staleness fade — see the plan's "…and the material that can actually do
// it". Fresh (alpha>=1.0): stays/returns to the shared OPAQUE
// objectClassMaterial[cls] template, no per-entity instance. Fading: a
// per-entity clay_translucent.mat instance, created from
// r.clayTranslucentMaterial (NEVER MaterialInstance::duplicate() of the
// opaque template — a duplicate of a clay.mat instance is still clay.mat:
// opaque, float3 baseColor), seeded from the STORED class tint
// (objectClassTint — MaterialInstance has no getter), alpha set every call.
void update_entity_staleness(VisualRenderer& r, const TrackedObject& obj, ObjectEntity& e,
                              double sim_time_sec) {
    const auto alpha = static_cast<float>(detail::SceneBuffer::staleness_alpha(
        sim_time_sec, obj.last_update_sec, kStaleFadeStartSec, kStaleFadeTimeoutSec));
    filament::RenderableManager& rm = r.engine->getRenderableManager();
    const auto clsIdx = static_cast<uint8_t>(e.cls);

    auto bind_everywhere = [&](filament::MaterialInstance* mat) {
        if (e.glInstance != nullptr) {
            remap_to_material(rm, e.glInstance->getEntities(), e.glInstance->getEntityCount(), mat);
        } else if (e.proceduralBox.entity) {
            remap_to_material(rm, &e.proceduralBox.entity, 1, mat);
        }
    };

    if (alpha >= 1.0f) {
        if (e.fadeInstance != nullptr) {
            bind_everywhere(r.objectClassMaterial[clsIdx]);
            r.engine->destroy(e.fadeInstance);
            e.fadeInstance = nullptr;
            e.fadeAlpha = 1.0f;
        }
        return;
    }
    if (e.fadeInstance == nullptr) {
        e.fadeInstance = r.clayTranslucentMaterial->createInstance();
        e.fadeInstance->setCullingMode(filament::backend::CullingMode::NONE);
        bind_everywhere(e.fadeInstance);
    }
    const detail::Float3& tint = r.objectClassTint[clsIdx];
    e.fadeInstance->setParameter("baseColor", float4{tint.r, tint.g, tint.b, alpha});
    e.fadeInstance->setParameter("roughness", r.active_theme.material.roughness);
    e.fadeInstance->setParameter("metallic", r.active_theme.material.metallic);
    e.fadeAlpha = alpha;
}

}  // namespace

bool ensure_gltf_loader(VisualRenderer& r) {
    if (r.sharedAssetLoader != nullptr) return true;

    namespace gltfio = filament::gltfio;
    r.sharedMaterialProvider = gltfio::createUbershaderProvider(
        r.engine, UBERARCHIVE_DEFAULT_DATA, static_cast<size_t>(UBERARCHIVE_DEFAULT_SIZE));
    if (r.sharedMaterialProvider == nullptr) return false;

    gltfio::AssetConfiguration assetConfig{};
    assetConfig.engine = r.engine;
    assetConfig.materials = r.sharedMaterialProvider;
    r.sharedAssetLoader = gltfio::AssetLoader::create(assetConfig);
    if (r.sharedAssetLoader == nullptr) {
        r.sharedMaterialProvider->destroyMaterials();
        delete r.sharedMaterialProvider;
        r.sharedMaterialProvider = nullptr;
        return false;
    }

    gltfio::ResourceConfiguration resConfig{};
    resConfig.engine = r.engine;
    resConfig.gltfPath = nullptr;
    resConfig.normalizeSkinningWeights = true;
    r.sharedResourceLoader = new gltfio::ResourceLoader(resConfig);
    return true;
}

void release_object_entity(VisualRenderer& r, ObjectEntity& e) {
    if (e.fadeInstance != nullptr) {
        r.engine->destroy(e.fadeInstance);
        e.fadeInstance = nullptr;
    }
    if (e.glInstance != nullptr) {
        // Recycle -- remove from the scene, return to the class free list.
        // NO destroyInstance() exists in gltfio (Step 3's own comment):
        // recycled, never destroyed.
        r.scene->removeEntities(e.glInstance->getEntities(), e.glInstance->getEntityCount());
        r.objectClassPools[static_cast<uint8_t>(e.cls)].freeList.push_back(e.glInstance);
        e.glInstance = nullptr;
    } else if (e.proceduralBox.entity) {
        destroy_mesh(*r.engine, *r.scene, e.proceduralBox);
    }
    if (e.arrowEntity) {
        r.scene->remove(e.arrowEntity);
        r.engine->destroy(e.arrowEntity);
        utils::EntityManager::get().destroy(e.arrowEntity);
        e.arrowEntity = {};
    }
    if (e.pathRibbon.entity) destroy_mesh(*r.engine, *r.scene, e.pathRibbon);
}

// Epic 2 Task 4 (VM-022) Step 5: diffs `s.objects`/`object_count` against
// `r.objectEntities` (keyed by TrackedObject::id), acquiring/updating/
// releasing only what changed -- never rebuilt wholesale.
void update_objects(VisualRenderer& r, const SceneGraph& s) {
    std::unordered_map<uint32_t, ObjectEntity> next;
    next.reserve(s.object_count);

    for (uint32_t i = 0; i < s.object_count; ++i) {
        const TrackedObject& obj = s.objects[i];
        ObjectEntity entity;
        auto it = r.objectEntities.find(obj.id);
        if (it != r.objectEntities.end()) {
            entity = std::move(it->second);
            r.objectEntities.erase(it);
            if (entity.cls != obj.cls) {
                // Class flip mid-track (footprint-band jitter near a band
                // boundary): re-acquire so model + tint follow the inference
                // instead of sticking to the class the track first arrived as.
                release_object_entity(r, entity);
                entity = {};
                acquire_entity(r, obj, entity);
            }
        } else {
            acquire_entity(r, obj, entity);
        }
        update_entity_transform(r, obj, entity);
        update_entity_arrow(r, obj, entity);
        update_entity_path(r, obj, entity);
        update_entity_staleness(r, obj, entity, s.sim_time_sec);
        if (next.count(obj.id)) {
            // Duplicate TrackedObject::id within one publish = malformed
            // input (spec §9): first wins, and the loser's entity must be
            // released or its instance leaks in-scene until the class cap.
            release_object_entity(r, entity);
        } else {
            next.emplace(obj.id, std::move(entity));
        }
    }

    // Anything left in r.objectEntities is a track that vanished between
    // this publish and the last one -- released, not leaked, not left
    // rendering at its stale pose.
    for (auto& [id, entity] : r.objectEntities) {
        release_object_entity(r, entity);
    }
    r.objectEntities = std::move(next);
}

// Epic 2 Task 4 (VM-022): see scene.h's frozen doc comment. Expected stems
// car.glb/truck_van.glb/bus.glb/pedestrian.glb/cyclist.glb -- UNKNOWN is
// deliberately never looked up here, always the procedural box. Any
// missing/unparseable stem is non-fatal (that class stays on the box);
// returns the count that actually loaded (0 is legal, spec §9 / Step 0).
uint32_t set_object_model_dir(VisualRenderer* r, const char* dir) {
    if (r == nullptr || dir == nullptr) return 0;
    if (!r->objectClassPools.empty()) {
        // scene.h's contract is "call once, from on_configure()". A second
        // call would overwrite the pools and leak every previously loaded
        // FilamentAsset + its live instances; refuse it instead.
        std::fprintf(stderr,
                     "[visual_renderer] set_object_model_dir called twice; "
                     "ignoring second call (contract: call once before the "
                     "first render_frame)\n");
        return 0;
    }
    if (!ensure_gltf_loader(*r)) return 0;

    struct ClassStem {
        ObjectClass cls;
        const char* stem;
    };
    static constexpr ClassStem kClasses[] = {
        {ObjectClass::CAR, "car"},       {ObjectClass::TRUCK_VAN, "truck_van"},
        {ObjectClass::BUS, "bus"},       {ObjectClass::PEDESTRIAN, "pedestrian"},
        {ObjectClass::CYCLIST, "cyclist"},
    };

    uint32_t loaded = 0;
    for (const ClassStem& c : kClasses) {
        const std::string path = std::string(dir) + "/" + c.stem + ".glb";
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file) continue;  // missing stem -- non-fatal, procedural box for this class
        const std::streamsize size = file.tellg();
        if (size <= 0) continue;
        std::vector<uint8_t> bytes(static_cast<size_t>(size));
        file.seekg(0);
        if (!file.read(reinterpret_cast<char*>(bytes.data()), size)) continue;

        // createInstancedAsset (Step 3): ONE parse feeding
        // kInitialInstancesPerClass placements, not N re-parses per
        // vehicle. Do NOT call releaseSourceData() -- it kills
        // createInstance() growth past this initial batch.
        std::vector<filament::gltfio::FilamentInstance*> instances(kInitialInstancesPerClass);
        filament::gltfio::FilamentAsset* asset = r->sharedAssetLoader->createInstancedAsset(
            bytes.data(), static_cast<uint32_t>(bytes.size()), instances.data(), instances.size());
        if (asset == nullptr) continue;
        if (!r->sharedResourceLoader->loadResources(asset)) {
            r->sharedAssetLoader->destroyAsset(asset);
            continue;
        }

        ObjectClassPool pool;
        pool.asset = asset;
        pool.pool = instances;
        pool.freeList = instances;  // none bound to a track yet
        const filament::Aabb box = asset->getBoundingBox();
        const filament::math::float3 d = box.max - box.min;
        pool.unitFootprint = {static_cast<double>(d.x), static_cast<double>(d.y),
                              static_cast<double>(d.z)};
        r->objectClassPools[static_cast<uint8_t>(c.cls)] = std::move(pool);
        ++loaded;
    }
    return loaded;
}

}  // namespace mpviz

// Epic 2 Task 4 (VM-022): Filament-free test introspection hooks (see
// objects_test_hooks.hpp's own comment for why these live here, mirroring
// ego_test_hooks.hpp's definitions living in ego.cpp).
namespace mpviz::testing {

namespace {

utils::Entity first_renderable(filament::RenderableManager& rm, const mpviz::ObjectEntity& e) {
    if (e.glInstance != nullptr) {
        const utils::Entity* ents = e.glInstance->getEntities();
        for (size_t i = 0; i < e.glInstance->getEntityCount(); ++i) {
            if (rm.getInstance(ents[i]).isValid()) return ents[i];
        }
        return {};
    }
    return e.proceduralBox.entity;
}

}  // namespace

bool object_in_scene(mpviz::VisualRenderer* r, uint32_t id) {
    if (r == nullptr) return false;
    auto it = r->objectEntities.find(id);
    if (it == r->objectEntities.end()) return false;
    // NOT FilamentInstance::getRoot() -- that entity "has no matching glTF
    // node" (FilamentInstance.h's own doc comment) and is deliberately
    // absent from getEntities(), so it is never passed to
    // scene->addEntities()/removeEntities() and checking it here would
    // read as "never in the scene" even for a correctly-rendering object.
    // Check one of the actual renderable entities instead -- they're always
    // added/removed together, so any one of them proves membership.
    filament::RenderableManager& rm = r->engine->getRenderableManager();
    const utils::Entity ent = first_renderable(rm, it->second);
    return ent && r->scene->hasEntity(ent);
}

uint64_t object_entity_identity(mpviz::VisualRenderer* r, uint32_t id) {
    if (r == nullptr) return 0;
    auto it = r->objectEntities.find(id);
    if (it == r->objectEntities.end()) return 0;
    if (it->second.glInstance != nullptr) {
        return reinterpret_cast<uint64_t>(it->second.glInstance);
    }
    return it->second.proceduralBox.entity.getId();
}

mpviz::Vec3 object_transform_scale(mpviz::VisualRenderer* r, uint32_t id) {
    if (r == nullptr) return {0.0, 0.0, 0.0};
    auto it = r->objectEntities.find(id);
    if (it == r->objectEntities.end()) return {0.0, 0.0, 0.0};
    return it->second.appliedScale;
}

mpviz::detail::Float3 object_class_tint(mpviz::VisualRenderer* r, mpviz::ObjectClass cls) {
    if (r == nullptr) return {};
    return r->objectClassTint[static_cast<uint8_t>(cls)];
}

ObjectMaterialInfo object_material_info(mpviz::VisualRenderer* r, uint32_t id) {
    if (r == nullptr) return {};
    auto it = r->objectEntities.find(id);
    if (it == r->objectEntities.end()) return {};
    const mpviz::ObjectEntity& e = it->second;
    filament::RenderableManager& rm = r->engine->getRenderableManager();
    const utils::Entity ent = first_renderable(rm, e);
    const auto ri = rm.getInstance(ent);
    if (!ri.isValid()) return {};
    filament::MaterialInstance* bound = rm.getMaterialInstanceAt(ri, 0);
    const bool translucent = bound != nullptr && bound->getMaterial() == r->clayTranslucentMaterial;
    ObjectMaterialInfo info;
    info.bound_to_translucent = translucent;
    info.alpha = translucent ? e.fadeAlpha : 1.0f;
    return info;
}

}  // namespace mpviz::testing
