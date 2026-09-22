// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "objects.hpp"
#include "objects_test_hooks.hpp"
#include "polyline.hpp"
#include "renderer_internal.hpp"
#include "overlume/scene.h"

#include <filament/Box.h>
#include <filament/RenderableManager.h>
#include <filament/TransformManager.h>

#include <gltfio/AssetLoader.h>
#include <gltfio/FilamentAsset.h>
#include <gltfio/MaterialProvider.h>
#include <gltfio/ResourceLoader.h>
#include <gltfio/TextureProvider.h>
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

namespace overlume {

namespace {

using filament::math::float3;
using filament::math::float4;
using filament::math::mat4f;
using filament::math::quatf;

float3 to_f3(const Vec3& v) {
    return float3{static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

bool is_zero_vec3(const Vec3& v) { return v.x == 0.0 && v.y == 0.0 && v.z == 0.0; }

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
        {{0, 0, -1}, {0, 3, 2, 1}}, {{0, 0, 1}, {4, 5, 6, 7}},  {{0, -1, 0}, {0, 1, 5, 4}},
        {{0, 1, 0}, {3, 7, 6, 2}},  {{-1, 0, 0}, {0, 4, 7, 3}}, {{1, 0, 0}, {1, 2, 6, 5}},
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

Vec3 unit_footprint_for(VisualRenderer& r, const ObjectEntity& e) {
    if (e.glInstance == nullptr) return {1.0, 1.0, 1.0};
    auto it = r.objectClassPools.find(static_cast<uint8_t>(e.cls));
    return it != r.objectClassPools.end() ? it->second.unitFootprint : Vec3{1.0, 1.0, 1.0};
}

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
            inst = r.sharedAssetLoader->createInstance(pool.asset);
            if (inst != nullptr) {
                pool.pool.push_back(inst);
                if (!pool.growthLogged) {
                    std::fprintf(stderr, "[overlume] object class %u grew past %zu instances\n",
                                 static_cast<unsigned>(clsIdx), kInitialInstancesPerClass);
                    pool.growthLogged = true;
                }
            }
        } else if (!pool.capWarned) {
            std::fprintf(stderr,
                         "[overlume] object class %u hit the %zu-instance cap; "
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
                     "[overlume] object class %u has no loaded model; using the "
                     "procedural clay box\n",
                     static_cast<unsigned>(clsIdx));
        r.objectClassMissingWarned[clsIdx] = true;
    }

    std::vector<Vertex> verts;
    std::vector<uint16_t> indices;
    build_unit_box(verts, indices);
    add_mesh(r, out.proceduralBox, std::move(verts), std::move(indices),
             filament::RenderableManager::PrimitiveType::TRIANGLES, r.objectClassMaterial[clsIdx],
             true, false);
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
            .boundingBox({{0, 0, 0}, {50.0f, 50.0f, 50.0f}})
            .geometry(0, filament::RenderableManager::PrimitiveType::TRIANGLES,
                      r.sharedArrowMesh.vb, r.sharedArrowMesh.ib)
            .material(0, r.objectClassMaterial[static_cast<uint8_t>(e.cls)])
            .culling(false)
            .castShadows(false)
            .receiveShadows(false)
            .build(*r.engine, e.arrowEntity);
        r.scene->addEntity(e.arrowEntity);
        tm.create(e.arrowEntity);
    }
    const double speed =
        std::sqrt(obj.velocity.x * obj.velocity.x + obj.velocity.y * obj.velocity.y);
    const double heading = std::atan2(obj.velocity.y, obj.velocity.x);
    const float3 pos{static_cast<float>(obj.position.x), static_cast<float>(obj.position.y),
                     static_cast<float>(obj.position.z + obj.dimensions.z + 0.15)};
    const quatf rot = quatf::fromAxisAngle(float3{0, 0, 1}, static_cast<float>(heading));
    const auto len = static_cast<float>(std::clamp(speed, 0.5, 5.0));
    const auto inst = tm.getInstance(e.arrowEntity);
    if (inst.isValid()) {
        tm.setTransform(
            inst, mat4f::translation(pos) * mat4f(rot) * mat4f::scaling(float3{len, 1.0f, 1.0f}));
    }
}

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
    if (sig == e.pathSignature) return;
    if (e.pathRibbon.entity) destroy_mesh(*r.engine, *r.scene, e.pathRibbon);

    constexpr float kPathHalfWidthM = 0.08f;
    constexpr float kPathZLiftM = 0.03f;
    std::vector<Vec3> ribbon =
        detail::extrude_polyline(obj.predicted_path, n, kPathHalfWidthM, kPathZLiftM);
    e.pathSignature = sig;
    if (ribbon.empty()) return;
    std::vector<uint16_t> idx =
        detail::extrude_polyline_indices(static_cast<uint32_t>(ribbon.size() / 2));
    if (idx.empty()) return;
    std::vector<Vertex> verts(ribbon.size());
    for (size_t i = 0; i < ribbon.size(); ++i) verts[i].position = to_f3(ribbon[i]);
    fill_tangent_frames(verts, std::vector<float3>(ribbon.size(), float3{0, 0, 1}));
    add_mesh(r, e.pathRibbon, std::move(verts), std::move(idx),
             filament::RenderableManager::PrimitiveType::TRIANGLES,
             r.objectClassMaterial[static_cast<uint8_t>(e.cls)], false, true);
}

void update_entity_staleness(VisualRenderer& r, const TrackedObject& obj, ObjectEntity& e,
                             double sim_time_sec) {
    const auto staleness = static_cast<float>(detail::SceneBuffer::staleness_alpha(
        sim_time_sec, obj.last_update_sec, kStaleFadeStartSec, kStaleFadeTimeoutSec));
    const float alpha = staleness * r.active_theme.objects.opacity;
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

}

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

    r.sharedTextureProvider = gltfio::createStbProvider(r.engine);
    if (r.sharedTextureProvider != nullptr) {
        r.sharedResourceLoader->addTextureProvider("image/jpeg", r.sharedTextureProvider);
        r.sharedResourceLoader->addTextureProvider("image/png", r.sharedTextureProvider);
    }
    return true;
}

void release_object_entity(VisualRenderer& r, ObjectEntity& e) {
    if (e.fadeInstance != nullptr) {
        r.engine->destroy(e.fadeInstance);
        e.fadeInstance = nullptr;
    }
    if (e.glInstance != nullptr) {
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
            release_object_entity(r, entity);
        } else {
            next.emplace(obj.id, std::move(entity));
        }
    }

    for (auto& [id, entity] : r.objectEntities) {
        release_object_entity(r, entity);
    }
    r.objectEntities = std::move(next);
}

uint32_t set_object_model_dir(VisualRenderer* r, const char* dir) {
    if (r == nullptr || dir == nullptr) return 0;
    if (!r->objectClassPools.empty()) {
        std::fprintf(stderr,
                     "[overlume] set_object_model_dir called twice; "
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
        {ObjectClass::CAR, "car"},         {ObjectClass::TRUCK_VAN, "truck_van"},
        {ObjectClass::BUS, "bus"},         {ObjectClass::PEDESTRIAN, "pedestrian"},
        {ObjectClass::CYCLIST, "cyclist"},
    };

    uint32_t loaded = 0;
    for (const ClassStem& c : kClasses) {
        const std::string path = std::string(dir) + "/" + c.stem + ".glb";
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file) continue;
        const std::streamsize size = file.tellg();
        if (size <= 0) continue;
        std::vector<uint8_t> bytes(static_cast<size_t>(size));
        file.seekg(0);
        if (!file.read(reinterpret_cast<char*>(bytes.data()), size)) continue;

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
        pool.freeList = instances;
        const filament::Aabb box = asset->getBoundingBox();
        const filament::math::float3 d = box.max - box.min;
        pool.unitFootprint = {static_cast<double>(d.x), static_cast<double>(d.y),
                              static_cast<double>(d.z)};
        r->objectClassPools[static_cast<uint8_t>(c.cls)] = std::move(pool);
        ++loaded;
    }
    return loaded;
}

}

namespace overlume::testing {

namespace {

utils::Entity first_renderable(filament::RenderableManager& rm, const overlume::ObjectEntity& e) {
    if (e.glInstance != nullptr) {
        const utils::Entity* ents = e.glInstance->getEntities();
        for (size_t i = 0; i < e.glInstance->getEntityCount(); ++i) {
            if (rm.getInstance(ents[i]).isValid()) return ents[i];
        }
        return {};
    }
    return e.proceduralBox.entity;
}

}

bool object_in_scene(overlume::VisualRenderer* r, uint32_t id) {
    if (r == nullptr) return false;
    auto it = r->objectEntities.find(id);
    if (it == r->objectEntities.end()) return false;
    filament::RenderableManager& rm = r->engine->getRenderableManager();
    const utils::Entity ent = first_renderable(rm, it->second);
    return ent && r->scene->hasEntity(ent);
}

uint64_t object_entity_identity(overlume::VisualRenderer* r, uint32_t id) {
    if (r == nullptr) return 0;
    auto it = r->objectEntities.find(id);
    if (it == r->objectEntities.end()) return 0;
    if (it->second.glInstance != nullptr) {
        return reinterpret_cast<uint64_t>(it->second.glInstance);
    }
    return it->second.proceduralBox.entity.getId();
}

overlume::Vec3 object_transform_scale(overlume::VisualRenderer* r, uint32_t id) {
    if (r == nullptr) return {0.0, 0.0, 0.0};
    auto it = r->objectEntities.find(id);
    if (it == r->objectEntities.end()) return {0.0, 0.0, 0.0};
    return it->second.appliedScale;
}

overlume::detail::Float3 object_class_tint(overlume::VisualRenderer* r, overlume::ObjectClass cls) {
    if (r == nullptr) return {};
    return r->objectClassTint[static_cast<uint8_t>(cls)];
}

ObjectMaterialInfo object_material_info(overlume::VisualRenderer* r, uint32_t id) {
    if (r == nullptr) return {};
    auto it = r->objectEntities.find(id);
    if (it == r->objectEntities.end()) return {};
    const overlume::ObjectEntity& e = it->second;
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

}
